#include <ros/ros.h>
#include <ros/serialization.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <traj_utils/Bspline.h>
#include <traj_utils/MultiBsplines.h>

#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>
#include <tuple>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;
using Bytes = std::vector<uint8_t>;
constexpr size_t HEADER = 12, MAX_TCP = 1024 * 1024, MAX_UDP = 60000;
enum Kind : uint16_t { MULTI = 1, SINGLE = 2 };

// MSB2 is deliberately incompatible with the old native-size_t wire format.
// Both peers must be upgraded. ROS payloads use bounded preflight before allocation.
struct Header { uint32_t length; uint16_t kind, sender; };
bool header(const uint8_t *p, Header &h) {
  if (std::memcmp(p, "MSB2", 4)) return false;
  uint32_t n; uint16_t k, s;
  std::memcpy(&n, p+4, 4); std::memcpy(&k, p+8, 2); std::memcpy(&s, p+10, 2);
  h = {ntohl(n), ntohs(k), ntohs(s)};
  return h.length > 0 && h.length <= MAX_TCP && (h.kind == MULTI || h.kind == SINGLE);
}
struct Probe {
  const uint8_t *p; size_t left;
  bool skip(size_t n) { if (n > left) return false; p += n; left -= n; return true; }
  bool u32(uint32_t &n) {
    if (left < 4) return false;
    n = uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
    return skip(4);
  }
  bool array(size_t stride) {
    uint32_t n;
    return u32(n) && n <= 10000 && n <= left / stride && skip(size_t(n)*stride);
  }
  bool spline() {
    // id, order, traj_id, start_time; then knots, positions, yaw, yaw_dt.
    return skip(24) && array(8) && array(24) && array(8) && skip(8);
  }
};
bool preflight(const Bytes &b, uint16_t kind) {
  Probe p{b.data(), b.size()};
  if (kind == SINGLE) return p.spline() && !p.left;
  uint32_t count;
  if (!p.skip(4) || !p.u32(count) || count > 51) return false;
  for (uint32_t i=0; i<count; ++i) if (!p.spline()) return false;
  return !p.left;
}
template<class M> Bytes encode(const M &m, Kind kind, int sender) {
  const uint32_t size = ros::serialization::serializationLength(m);
  if (!size || size > MAX_TCP) return {};
  Bytes b(HEADER + size);
  std::memcpy(b.data(), "MSB2", 4);
  uint32_t n = htonl(size); uint16_t k = htons(kind), s = htons(sender);
  std::memcpy(b.data()+4,&n,4); std::memcpy(b.data()+8,&k,2); std::memcpy(b.data()+10,&s,2);
  ros::serialization::OStream stream(b.data()+HEADER,size);
  ros::serialization::serialize(stream,m);
  return b;
}
template<class M> bool decode(Bytes &b, uint16_t kind, M &m) {
  if (!preflight(b,kind)) return false;
  try {
    ros::serialization::IStream stream(b.data(),b.size());
    ros::serialization::deserialize(stream,m);
    return stream.getLength() == 0;
  } catch (const std::exception &) { return false; }
}
bool valid(const traj_utils::Bspline &m, int id) {
  if (m.drone_id != id || m.order != 3 || m.pos_pts.size() < size_t(m.order+1)
      || m.knots.size() != m.pos_pts.size()+m.order+1 || m.start_time.isZero()
      || !std::isfinite(m.yaw_dt) || m.yaw_dt < 0) return false;
  double previous = -INFINITY;
  for (double t : m.knots) {
    if (!std::isfinite(t) || t <= previous) return false;
    previous = t;
  }
  if (m.knots[m.pos_pts.size()] <= std::max(0.0,m.knots[m.order])) return false;
  for (const auto &p : m.pos_pts)
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return false;
  for (double y : m.yaw_pts) if (!std::isfinite(y)) return false;
  return true;
}
bool valid(const traj_utils::MultiBsplines &m, int sender) {
  if (m.drone_id_from != sender || m.traj.size() != size_t(sender+1)) return false;
  // The 1-based EGO chain reserves entry zero.
  if (m.traj[0].drone_id != 0 || !m.traj[0].pos_pts.empty() || !m.traj[0].knots.empty()
      || !m.traj[0].yaw_pts.empty()) return false;
  for (int i=1; i<=sender; ++i) if (!valid(m.traj[i],i)) return false;
  return true;
}
void closeFd(int &fd) {
  if (fd >= 0) { ::shutdown(fd,SHUT_RDWR); ::close(fd); fd = -1; }
}
void nonblock(int fd) { fcntl(fd,F_SETFL,fcntl(fd,F_GETFL,0)|O_NONBLOCK); }
bool ready(int fd, short events, int ms=100) {
  pollfd p{fd,events,0};
  return poll(&p,1,ms)>0 && (p.revents & events);
}
sockaddr_in address(const std::string &ip, int port) {
  sockaddr_in a{}; a.sin_family=AF_INET; a.sin_port=htons(port);
  if (port < 1 || port > 65535 || inet_pton(AF_INET,ip.c_str(),&a.sin_addr)!=1)
    throw std::runtime_error("invalid IPv4 address or port");
  return a;
}
int boundSocket(int type, const sockaddr_in &a) {
  int fd=socket(AF_INET,type,0);
  if (fd<0) throw std::runtime_error("socket creation failed");
  int one=1;
  if (type==SOCK_STREAM) setsockopt(fd,SOL_SOCKET,SO_REUSEADDR,&one,sizeof(one));
  else setsockopt(fd,SOL_SOCKET,SO_BROADCAST,&one,sizeof(one));
  if (bind(fd,reinterpret_cast<const sockaddr*>(&a),sizeof(a))<0) {
    closeFd(fd); throw std::runtime_error("bind failed; check duplicate bridge/port ownership");
  }
  nonblock(fd); return fd;
}
void keepalive(int fd) {
  int one=1, idle=5, interval=3, count=2;
  setsockopt(fd,SOL_SOCKET,SO_KEEPALIVE,&one,sizeof(one));
  setsockopt(fd,IPPROTO_TCP,TCP_KEEPIDLE,&idle,sizeof(idle));
  setsockopt(fd,IPPROTO_TCP,TCP_KEEPINTVL,&interval,sizeof(interval));
  setsockopt(fd,IPPROTO_TCP,TCP_KEEPCNT,&count,sizeof(count));
}
} // namespace

class Bridge {
  ros::NodeHandle nh_{"~"};
  int id_, count_, tcp_port_, peer_port_, udp_port_, udp_peer_port_;
  std::string peer_ip_, udp_ip_, bind_ip_, ns_;
  int listener_=-1, udp_=-1, tx_=-1;
  sockaddr_in peer_{}, udp_peer_{};
  std::atomic<bool> alive_{true}, tx_up_{false}, rx_up_{false};
  std::mutex tx_mutex_;
  std::thread receiver_, sender_, udp_receiver_;
  ros::Publisher multi_rx_, single_rx_, state_, request_current_;
  ros::Subscriber multi_tx_, single_tx_;
  ros::WallTimer timer_;
  using Key=std::tuple<int,int64_t,uint64_t>;
  std::set<Key> seen_;
  std::deque<Key> seen_order_;

  bool running() const { return alive_ && ros::ok(); }
  bool connected() const {
    return (id_==count_ || tx_up_) && (id_==1 || rx_up_);
  }
  void publishState() {
    std_msgs::Bool m; m.data=connected(); state_.publish(m);
    // Repeat across ROS discovery and peer/planner restarts. Ask the owner to
    // rebuild its current chain; never replay a cached startup trajectory.
    if (id_<count_ && tx_up_) request_current_.publish(std_msgs::Empty());
  }
  int connectPeer() {
    int fd=socket(AF_INET,SOCK_STREAM,0);
    if (fd<0) return -1;
    nonblock(fd);
    int rc=connect(fd,reinterpret_cast<const sockaddr*>(&peer_),sizeof(peer_));
    if (rc<0 && (errno!=EINPROGRESS || !ready(fd,POLLOUT,300))) { closeFd(fd); return -1; }
    int error=0; socklen_t len=sizeof(error);
    if (getsockopt(fd,SOL_SOCKET,SO_ERROR,&error,&len)<0 || error) { closeFd(fd); return -1; }
    keepalive(fd); return fd;
  }
  void senderLoop() {
    while (running()) {
      {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        if (tx_>=0) {
          char c;
          int n=recv(tx_,&c,1,MSG_PEEK|MSG_DONTWAIT);
          if (n==0 || (n<0 && errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR)) {
            closeFd(tx_); tx_up_=false;
          }
        }
      }
      if (!tx_up_) {
        int fd=connectPeer();
        if (fd>=0) {
          std::lock_guard<std::mutex> lock(tx_mutex_);
          tx_=fd; tx_up_=true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  void sendMulti(const traj_utils::MultiBsplines::ConstPtr &m) {
    if (!valid(*m,id_)) { ROS_WARN_THROTTLE(2,"Reject invalid/nonlocal multi trajectory"); return; }
    Bytes b=encode(*m,MULTI,id_);
    if (b.empty()) return;
    std::lock_guard<std::mutex> lock(tx_mutex_);
    if (tx_<0) return; // Never replay a queued trajectory after reconnect.
    size_t sent=0;
    auto end=Clock::now()+std::chrono::milliseconds(200);
    while (sent<b.size() && running() && Clock::now()<end) {
      ssize_t n=send(tx_,b.data()+sent,b.size()-sent,MSG_NOSIGNAL);
      if (n>0) sent+=size_t(n);
      else if (n<0 && errno==EINTR) continue;
      else if (n<0 && (errno==EAGAIN || errno==EWOULDBLOCK)) ready(tx_,POLLOUT,20);
      else break;
    }
    if (sent!=b.size()) { closeFd(tx_); tx_up_=false; }
  }
  void sendSingle(const traj_utils::Bspline::ConstPtr &m) {
    if (!valid(*m,id_)) { ROS_WARN_THROTTLE(2,"Reject invalid/nonlocal UDP trajectory"); return; }
    Bytes b=encode(*m,SINGLE,id_);
    if (b.size()>MAX_UDP || b.empty()) { ROS_WARN_THROTTLE(2,"UDP trajectory exceeds packet budget"); return; }
    if (sendto(udp_,b.data(),b.size(),0,reinterpret_cast<const sockaddr*>(&udp_peer_),sizeof(udp_peer_))
        != ssize_t(b.size())) ROS_WARN_THROTTLE(2,"UDP trajectory send failed");
  }
  bool consume(Bytes &pending) {
    size_t offset=0;
    while (pending.size()-offset>=HEADER) {
      Header h;
      if (!header(pending.data()+offset,h) || h.kind!=MULTI || h.sender!=id_-1) return false;
      if (pending.size()-offset<HEADER+h.length) break;
      Bytes payload(pending.begin()+offset+HEADER,pending.begin()+offset+HEADER+h.length);
      traj_utils::MultiBsplines m;
      if (!decode(payload,MULTI,m) || !valid(m,h.sender)) return false;
      multi_rx_.publish(m);
      offset+=HEADER+h.length;
    }
    if (offset) pending.erase(pending.begin(),pending.begin()+offset);
    return true;
  }
  void receiverLoop() {
    while (running()) {
      if (!ready(listener_,POLLIN)) continue;
      int fd=accept(listener_,nullptr,nullptr);
      if (fd<0) continue;
      nonblock(fd); keepalive(fd); rx_up_=true;
      Bytes pending; pending.reserve(8192);
      auto frame_start=Clock::now();
      while (running()) {
        if (!pending.empty() && Clock::now()-frame_start>std::chrono::seconds(2)) break;
        if (!ready(fd,POLLIN)) continue;
        uint8_t chunk[8192]; ssize_t n=recv(fd,chunk,sizeof(chunk),0);
        if (n==0) break;
        if (n<0) { if (errno==EINTR || errno==EAGAIN || errno==EWOULDBLOCK) continue; break; }
        if (pending.empty()) frame_start=Clock::now();
        pending.insert(pending.end(),chunk,chunk+n);
        const auto before=pending.size();
        if (!consume(pending)) { ROS_WARN("Rejected invalid/version-mismatched TCP frame"); break; }
        if (pending.size()<before) frame_start=Clock::now();
        if (pending.size()>HEADER+MAX_TCP) break;
      }
      rx_up_=false; closeFd(fd);
    }
  }
  void udpLoop() {
    Bytes bytes(MAX_UDP+1);
    while (running()) {
      if (!ready(udp_,POLLIN)) continue;
      ssize_t n=recv(udp_,bytes.data(),bytes.size(),0);
      if (n<ssize_t(HEADER) || n>ssize_t(MAX_UDP)) continue;
      Header h;
      if (!header(bytes.data(),h) || h.kind!=SINGLE || HEADER+h.length!=size_t(n)
          || h.sender<1 || h.sender>count_ || h.sender==id_) continue;
      Bytes payload(bytes.begin()+HEADER,bytes.begin()+n);
      traj_utils::Bspline m;
      if (!decode(payload,SINGLE,m) || !valid(m,h.sender)) continue;
      // Trajectories are immutable for (sender, trajectory ID, start time).
      Key k{h.sender,m.traj_id,m.start_time.toNSec()};
      if (!seen_.insert(k).second) continue;
      seen_order_.push_back(k);
      if (seen_order_.size()>256) { seen_.erase(seen_order_.front()); seen_order_.pop_front(); }
      single_rx_.publish(m); // Preserve the original trajectory time, never stamp now.
    }
  }
public:
  Bridge() {
    nh_.param("drone_id",id_,1); nh_.param("swarm_num",count_,2);
    nh_.param("tcp_port",tcp_port_,8080); nh_.param("tcp_peer_port",peer_port_,8080);
    nh_.param("udp_port",udp_port_,8081); nh_.param("udp_peer_port",udp_peer_port_,8081);
    nh_.param<std::string>("next_drone_ip",peer_ip_,"127.0.0.1");
    nh_.param<std::string>("broadcast_ip",udp_ip_,"192.168.1.255");
    nh_.param<std::string>("bind_ip",bind_ip_,"0.0.0.0");
    if (id_<1 || count_<id_ || count_>50) throw std::runtime_error("IDs must be 1..swarm_num, maximum 50");
    ns_="/uav"+std::to_string(id_)+"/bridge/";
    peer_=address(peer_ip_,peer_port_); udp_peer_=address(udp_ip_,udp_peer_port_);
    single_rx_=nh_.advertise<traj_utils::Bspline>(ns_+"rx/bspline",100);
    if (id_<count_) request_current_=nh_.advertise<std_msgs::Empty>(ns_+"request_current",1);
    state_=nh_.advertise<std_msgs::Bool>("/uav"+std::to_string(id_)+"/bridge_connection_state",1,true);
    state_.publish(std_msgs::Bool());
    try {
      udp_=boundSocket(SOCK_DGRAM,address(bind_ip_,udp_port_));
      if (id_>1) {
        listener_=boundSocket(SOCK_STREAM,address(bind_ip_,tcp_port_));
        if (listen(listener_,2)<0) throw std::runtime_error("TCP listen failed");
        multi_rx_=nh_.advertise<traj_utils::MultiBsplines>(ns_+"rx/prev_multi_traj",10);
      }
    } catch (...) { closeFd(udp_); closeFd(listener_); throw; }
    single_tx_=nh_.subscribe(ns_+"tx/bspline",100,&Bridge::sendSingle,this,ros::TransportHints().tcpNoDelay());
    if (id_<count_)
      multi_tx_=nh_.subscribe(ns_+"tx/multi_traj",10,&Bridge::sendMulti,this,ros::TransportHints().tcpNoDelay());
    timer_=nh_.createWallTimer(ros::WallDuration(.5),[this](const ros::WallTimerEvent&){publishState();});
    udp_receiver_=std::thread(&Bridge::udpLoop,this);
    if (id_>1) receiver_=std::thread(&Bridge::receiverLoop,this);
    if (id_<count_) sender_=std::thread(&Bridge::senderLoop,this);
    ROS_INFO("MSB2 bridge uav%d/%d; separate TX/RX; TCP chain and UDP peer exchange",id_,count_);
  }
  ~Bridge() {
    alive_=false;
    if (sender_.joinable()) sender_.join();
    if (receiver_.joinable()) receiver_.join();
    if (udp_receiver_.joinable()) udp_receiver_.join();
    closeFd(tx_); closeFd(listener_); closeFd(udp_);
  }
};

int main(int argc,char **argv) {
  ros::init(argc,argv,"rosmsg_tcp_bridge");
  try { Bridge bridge; ros::spin(); }
  catch (const std::exception &e) { ROS_FATAL("bridge: %s",e.what()); return 1; }
  return 0;
}
