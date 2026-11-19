// Exercise the production callback against a real TCP peer that never reads.
#define main bridge_node_program_main
#include "../src/bridge_node.cpp"
#undef main
#include <iostream>

struct TestSocket
{
  int fd;
  explicit TestSocket(int socket) : fd(socket) {}
  ~TestSocket() { closeFd(fd); }
};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "bridge_sender_regression", ros::init_options::NoSigintHandler);
  try
  {
    // The Python harness supplies free ports; no flight/controller nodes are involved.
    ros::NodeHandle nh("~");
    int peer_port;
    nh.getParam("test_peer_port", peer_port);
    TestSocket listener(boundSocket(SOCK_STREAM, address("127.0.0.1", peer_port)));
    int small = 4096;
    setsockopt(listener.fd, SOL_SOCKET, SO_RCVBUF, &small, sizeof(small));
    if (listen(listener.fd, 1) < 0)
      throw std::runtime_error("test listen failed");
    nh.setParam("drone_id", 1);
    nh.setParam("swarm_num", 2);
    nh.setParam("next_drone_ip", "127.0.0.1");
    nh.setParam("broadcast_ip", "127.0.0.1");
    nh.setParam("bind_ip", "127.0.0.1");
    nh.setParam("tcp_peer_port", peer_port);
    Bridge bridge;
    if (!ready(listener.fd, POLLIN, 3000))
      throw std::runtime_error("sender did not connect");
    TestSocket peer(accept(listener.fd, nullptr, nullptr));
    closeFd(listener.fd); // Prevent a new connection until backpressure cleanup is checked.
    auto m = boost::make_shared<traj_utils::MultiBsplines>();
    m->drone_id_from = 1;
    m->traj.resize(2);
    auto &b = m->traj[1];
    b.drone_id = 1;
    b.order = 3;
    b.start_time = ros::Time::now();
    b.pos_pts.resize(9000);
    b.knots.resize(9004);
    for (size_t i = 0; i < b.knots.size(); ++i)
      b.knots[i] = double(i) - 3;
    double worst_ms = 0;
    for (int i = 0; i < 40; ++i)
    {
      b.traj_id = i;
      const auto start = Clock::now();
      bridge.sendMulti(m);
      worst_ms = std::max(worst_ms, std::chrono::duration<double, std::milli>(Clock::now() - start).count());
    }
    // Keep offering frames until the real non-reading peer exhausts the TCP window.
    const auto blocked_until = Clock::now() + std::chrono::seconds(4);
    while (!bridge.tcp_send_failures_ && Clock::now() < blocked_until)
    {
      const auto start = Clock::now();
      bridge.sendMulti(m);
      worst_ms = std::max(worst_ms, std::chrono::duration<double, std::milli>(Clock::now() - start).count());
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (!bridge.tcp_send_failures_)
      throw std::runtime_error("test did not establish TCP backpressure");
    if (bridge.last_send_ms_ < 180 || bridge.last_send_ms_ > 1000)
      throw std::runtime_error("TCP send deadline was not bounded");
    if (worst_ms >= 100)
      throw std::runtime_error("ROS callback blocked on slow TCP peer: " + std::to_string(worst_ms) + " ms");
    const auto cleared_until = Clock::now() + std::chrono::seconds(1);
    while (bridge.tx_up_ && Clock::now() < cleared_until)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (bridge.tx_pending_bytes_ || bridge.tx_active_bytes_)
      throw std::runtime_error("disconnect did not clear active/pending TCP frames");
    closeFd(peer.fd);
    listener.fd = boundSocket(SOCK_STREAM, address("127.0.0.1", peer_port));
    if (listen(listener.fd, 1) < 0)
      throw std::runtime_error("test relisten failed");
    const auto until = Clock::now() + std::chrono::seconds(3);
    while (bridge.tcp_connections_ < 2 && Clock::now() < until)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (bridge.tcp_connections_ < 2)
      throw std::runtime_error("sender did not reconnect");
    if (!ready(listener.fd, POLLIN, 1000))
      throw std::runtime_error("reconnection not accepted");
    TestSocket recovered(accept(listener.fd, nullptr, nullptr));
    if (ready(recovered.fd, POLLIN, 250))
      throw std::runtime_error("old queued trajectory replayed after reconnect");
    std::cout << "Slow TCP peer: worst callback " << worst_ms << " ms; bounded timeout, cleared queue and no reconnect replay verified" << std::endl;
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
