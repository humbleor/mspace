#include "swarm_frame_transform.h"
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <mavros_msgs/PositionTarget.h>
#include <prometheus_msgs/DroneState.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TransformStamped.h>
#include <array>
#include <algorithm>
#include <cstring>
#include <limits>
#include <mutex>
#include <sstream>
#include <vector>

namespace
{
  using V3 = Eigen::Vector3d;
  using Q = Eigen::Quaterniond;
  using swarm_frame::YawTransform;
  template <class P>
  V3 vec(const P &p) { return V3(p.x, p.y, p.z); }
  template <class P>
  void put(const V3 &v, P &p)
  {
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
  }
  Q quat(const geometry_msgs::Quaternion &q) { return Q(q.w, q.x, q.y, q.z); }
  geometry_msgs::Quaternion msgQuat(const Q &q)
  {
    geometry_msgs::Quaternion out;
    out.w = q.w();
    out.x = q.x();
    out.y = q.y();
    out.z = q.z();
    return out;
  }
  bool finitePose(const nav_msgs::Odometry &m)
  {
    return vec(m.pose.pose.position).allFinite() && swarm_frame::validQuaternion(quat(m.pose.pose.orientation)) &&
           vec(m.twist.twist.linear).allFinite() && vec(m.twist.twist.angular).allFinite();
  }
  struct Slot
  {
    nav_msgs::Odometry msg;
    ros::WallTime received;
    bool have = false;
  };
  class Adapter
  {
    ros::NodeHandle nh_, pnh_{"~"};
    ros::Subscriber lio_sub_, fcu_sub_, raw_sub_, cloud_sub_, cmd_sub_;
    ros::Publisher odom_pub_, state_pub_, cloud_pub_, cmd_pub_, ready_pub_;
    ros::WallTimer timer_;
    tf2_ros::StaticTransformBroadcaster static_tf_;
    Slot lio_, fcu_;
    prometheus_msgs::DroneState raw_;
    ros::WallTime raw_received_, stable_since_;
    // Readiness, state and command paths run on several spinner threads so that a
    // slow cloud callback can never starve the LIO/FCU freshness watchdog.
    std::mutex mutex_;
    std::string prefix_, lio_frame_, fcu_frame_, body_frame_, imu_frame_;
    V3 desired_, body_in_imu_, lio_anchor_, fcu_anchor_, lio_sum_, fcu_sum_;
    Q body_rotation_;
    YawTransform world_lio_, world_fcu_;
    double desired_yaw_, stable_duration_, timeout_, receive_timeout_, skew_, position_tolerance_, yaw_tolerance_, tilt_limit_, speed_limit_;
    double lio_anchor_yaw_ = 0, fcu_anchor_yaw_ = 0, lio_sin_ = 0, lio_cos_ = 0, fcu_sin_ = 0, fcu_cos_ = 0;
    int samples_ = 0;
    ros::Time sampled_lio_, sampled_fcu_;
    bool ready_ = false, invalid_ = false;

    double param(const std::string &key, double fallback, bool positive = true)
    {
      double value;
      pnh_.param(key, value, fallback);
      if (!std::isfinite(value) || (positive && value <= 0))
        throw std::runtime_error("invalid parameter: " + key);
      return value;
    }
    V3 vectorParam(const std::string &key)
    {
      std::vector<double> values;
      pnh_.param(key, values, std::vector<double>{0, 0, 0});
      if (values.size() != 3)
        throw std::runtime_error(key + " needs three values");
      V3 out(values[0], values[1], values[2]);
      if (!out.allFinite())
        throw std::runtime_error("invalid vector: " + key);
      return out;
    }
    bool fresh(const ros::Time &stamp, const ros::WallTime &received) const
    {
      const double age = (ros::Time::now() - stamp).toSec();
      return !stamp.isZero() && age >= -.1 && age <= timeout_ &&
             !received.isZero() && (ros::WallTime::now() - received).toSec() <= receive_timeout_;
    }
    bool sourcesFresh() const
    {
      return lio_.have && fcu_.have && raw_.connected &&
             fresh(lio_.msg.header.stamp, lio_.received) && fresh(fcu_.msg.header.stamp, fcu_.received) &&
             fresh(raw_.header.stamp, raw_received_) &&
             std::abs((lio_.msg.header.stamp - fcu_.msg.header.stamp).toSec()) <= skew_;
    }
    // Name the failing source: timeouts during a mission must be diagnosable from the log alone.
    std::string freshness() const
    {
      std::ostringstream os;
      os << "lio age=" << (ros::Time::now() - lio_.msg.header.stamp).toSec()
         << " recv=" << (ros::WallTime::now() - lio_.received).toSec()
         << "; fcu age=" << (ros::Time::now() - fcu_.msg.header.stamp).toSec()
         << " recv=" << (ros::WallTime::now() - fcu_.received).toSec()
         << "; state age=" << (ros::Time::now() - raw_.header.stamp).toSec()
         << " recv=" << (ros::WallTime::now() - raw_received_).toSec()
         << "; skew=" << (lio_.msg.header.stamp - fcu_.msg.header.stamp).toSec()
         << " (limit " << skew_ << "); have lio=" << lio_.have << " fcu=" << fcu_.have
         << " connected=" << raw_.connected << "; timeout=" << timeout_ << " recv_limit=" << receive_timeout_;
      return os.str();
    }
    void resetWindow()
    {
      stable_since_ = ros::WallTime();
      samples_ = 0;
      lio_sum_.setZero();
      fcu_sum_.setZero();
      lio_sin_ = lio_cos_ = fcu_sin_ = fcu_cos_ = 0;
      sampled_lio_ = sampled_fcu_ = ros::Time();
    }
    void invalidate(const std::string &reason)
    {
      resetWindow();
      if (ready_)
      {
        ready_ = false;
        invalid_ = true;
        ROS_ERROR_STREAM("Frame alignment INVALID: " << reason << ". Land using FCU procedures; restart adapter disarmed to realign.");
        std_msgs::Bool m;
        m.data = false;
        ready_pub_.publish(m);
      }
      else
        ROS_WARN_STREAM_THROTTLE(3.0, "Waiting for frame alignment: " << reason);
    }
    void odometry(const nav_msgs::Odometry::ConstPtr &m, Slot &slot, bool is_lio)
    {
      std::lock_guard<std::mutex> guard(mutex_);
      const std::string expected = is_lio ? lio_frame_ : fcu_frame_;
      const std::string child = is_lio ? imu_frame_ : body_frame_;
      if (m->header.frame_id != expected || m->child_frame_id != child || !finitePose(*m) ||
          !fresh(m->header.stamp, ros::WallTime::now()))
      {
        slot.have = false;
        invalidate("invalid/stale odometry or unexpected frame");
        return;
      }
      if (slot.have && m->header.stamp <= slot.msg.header.stamp)
      {
        if (m->header.stamp < slot.msg.header.stamp)
          invalidate("odometry timestamp moved backward");
        return; // duplicates cannot keep an input alive
      }
      if (ready_ && slot.have)
      {
        const double dt = (m->header.stamp - slot.msg.header.stamp).toSec();
        V3 v = vec(slot.msg.twist.twist.linear);
        if (!is_lio)
          v = quat(slot.msg.pose.pose.orientation) * v; // FCU twist is body; LIO high-rate velocity is local
        const V3 residual = vec(m->pose.pose.position) - vec(slot.msg.pose.pose.position) - v * dt;
        const double da = std::abs(swarm_frame::wrap(swarm_frame::yaw(quat(m->pose.pose.orientation)) -
                                                     swarm_frame::yaw(quat(slot.msg.pose.pose.orientation))));
        if (dt > timeout_ || residual.norm() > .5 || da > .35 + 2 * vec(m->twist.twist.angular).norm() * dt)
        {
          invalidate("local reference discontinuity");
          return;
        }
      }
      slot.msg = *m;
      slot.received = ros::WallTime::now();
      slot.have = true;
      if (!is_lio && ready_ && !invalid_ && sourcesFresh())
        publishState();
    }
    void lioCb(const nav_msgs::Odometry::ConstPtr &m) { odometry(m, lio_, true); }
    void fcuCb(const nav_msgs::Odometry::ConstPtr &m) { odometry(m, fcu_, false); }
    void rawCb(const prometheus_msgs::DroneState::ConstPtr &m)
    {
      std::lock_guard<std::mutex> guard(mutex_);
      raw_ = *m;
      raw_received_ = ros::WallTime::now();
      if (!m->connected)
        invalidate("FCU disconnected");
    }
    void publishStatic(const YawTransform &t, const std::string &child,
                       std::vector<geometry_msgs::TransformStamped> &transforms)
    {
      geometry_msgs::TransformStamped m;
      m.header.stamp = ros::Time::now();
      m.header.frame_id = "world";
      m.child_frame_id = child;
      put(t.translation, m.transform.translation);
      m.transform.rotation = msgQuat(swarm_frame::yawQuaternion(t.angle));
      transforms.push_back(m);
    }
    void tick(const ros::WallTimerEvent &)
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (!invalid_ && !sourcesFresh())
        invalidate("waiting for fresh LIO/FCU/state feedback [" + freshness() + "]");
      if (!invalid_ && sourcesFresh() && !ready_)
      {
        const Q ql = swarm_frame::bodyOrientation(quat(lio_.msg.pose.pose.orientation), body_rotation_);
        const Q qf = quat(fcu_.msg.pose.pose.orientation).normalized();
        const V3 pl = swarm_frame::bodyPoint(vec(lio_.msg.pose.pose.position), quat(lio_.msg.pose.pose.orientation), body_in_imu_);
        const V3 pf = vec(fcu_.msg.pose.pose.position);
        const double yl = swarm_frame::yaw(ql), yf = swarm_frame::yaw(qf);
        if (raw_.armed || vec(lio_.msg.twist.twist.linear).norm() > speed_limit_ ||
            vec(fcu_.msg.twist.twist.linear).norm() > speed_limit_ ||
            vec(lio_.msg.twist.twist.angular).norm() > .1 || vec(fcu_.msg.twist.twist.angular).norm() > .1 ||
            swarm_frame::tilt(ql) > tilt_limit_ || swarm_frame::tilt(qf) > tilt_limit_)
        {
          resetWindow();
        }
        else
        {
          if (!stable_since_.isZero() &&
              ((pl - lio_anchor_).norm() > position_tolerance_ || (pf - fcu_anchor_).norm() > position_tolerance_ ||
               std::abs(swarm_frame::wrap(yl - lio_anchor_yaw_)) > yaw_tolerance_ ||
               std::abs(swarm_frame::wrap(yf - fcu_anchor_yaw_)) > yaw_tolerance_))
            resetWindow();
          if (stable_since_.isZero())
          {
            stable_since_ = ros::WallTime::now();
            lio_anchor_ = pl;
            fcu_anchor_ = pf;
            lio_anchor_yaw_ = yl;
            fcu_anchor_yaw_ = yf;
          }
          if (sampled_lio_ != lio_.msg.header.stamp && sampled_fcu_ != fcu_.msg.header.stamp)
          {
            lio_sum_ += pl;
            fcu_sum_ += pf;
            lio_sin_ += std::sin(yl);
            lio_cos_ += std::cos(yl);
            fcu_sin_ += std::sin(yf);
            fcu_cos_ += std::cos(yf);
            ++samples_;
            sampled_lio_ = lio_.msg.header.stamp;
            sampled_fcu_ = fcu_.msg.header.stamp;
          }
          if ((ros::WallTime::now() - stable_since_).toSec() >= stable_duration_ && samples_ >= 3)
          {
            world_lio_ = YawTransform::fromInitial(desired_, desired_yaw_, lio_sum_ / samples_,
                                                   swarm_frame::yawQuaternion(std::atan2(lio_sin_, lio_cos_)));
            world_fcu_ = YawTransform::fromInitial(desired_, desired_yaw_, fcu_sum_ / samples_,
                                                   swarm_frame::yawQuaternion(std::atan2(fcu_sin_, fcu_cos_)));
            ready_ = true;
            std::vector<geometry_msgs::TransformStamped> transforms;
            publishStatic(world_lio_, lio_frame_, transforms);
            publishStatic(world_fcu_, fcu_frame_, transforms);
            static_tf_.sendTransform(transforms);
            ROS_INFO_STREAM("Frame alignment READY: initial world position " << desired_.transpose()
                                                                             << "; LIO yaw offset " << world_lio_.angle << "; FCU yaw offset " << world_fcu_.angle);
            publishState();
          }
        }
      }
      std_msgs::Bool m;
      m.data = ready_ && !invalid_ && sourcesFresh();
      ready_pub_.publish(m);
    }
    void publishState()
    {
      nav_msgs::Odometry out = fcu_.msg;
      out.header.frame_id = "world";
      out.child_frame_id = body_frame_;
      put(world_fcu_.point(vec(fcu_.msg.pose.pose.position)), out.pose.pose.position);
      out.pose.pose.orientation = msgQuat(world_fcu_.orientation(quat(fcu_.msg.pose.pose.orientation)));
      // Pose covariance changes parent basis; body twist and its covariance stay unchanged.
      Eigen::Matrix<double, 6, 6> j = Eigen::Matrix<double, 6, 6>::Zero();
      j.block<3, 3>(0, 0) = world_fcu_.rotation();
      j.block<3, 3>(3, 3) = world_fcu_.rotation();
      Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> covariance(out.pose.covariance.data());
      covariance = (j * covariance * j.transpose()).eval();
      odom_pub_.publish(out);
      prometheus_msgs::DroneState state = raw_;
      state.header = out.header;
      const V3 p = vec(out.pose.pose.position);
      const V3 v = quat(out.pose.pose.orientation) * vec(out.twist.twist.linear);
      for (int i = 0; i < 3; ++i)
      {
        state.position[i] = p[i];
        state.velocity[i] = v[i];
      }
      state.attitude_q = out.pose.pose.orientation;
      tf::Quaternion q;
      tf::quaternionMsgToTF(state.attitude_q, q);
      double r, pitch, y;
      tf::Matrix3x3(q).getRPY(r, pitch, y);
      state.attitude[0] = r;
      state.attitude[1] = pitch;
      state.attitude[2] = y;
      state.attitude_rate[0] = out.twist.twist.angular.x;
      state.attitude_rate[1] = out.twist.twist.angular.y;
      state.attitude_rate[2] = out.twist.twist.angular.z;
      state_pub_.publish(state);
    }
    static float readFloat(const uint8_t *p, bool big)
    {
      uint8_t b[4];
      uint16_t one = 1;
      const bool host_big = (*reinterpret_cast<uint8_t *>(&one) == 0);
      for (int i = 0; i < 4; ++i)
        b[i] = p[(big != host_big) ? 3 - i : i];
      float f;
      std::memcpy(&f, b, 4);
      return f;
    }
    static void writeFloat(uint8_t *p, bool big, float f)
    {
      uint8_t b[4];
      std::memcpy(b, &f, 4);
      uint16_t one = 1;
      const bool host_big = (*reinterpret_cast<uint8_t *>(&one) == 0);
      for (int i = 0; i < 4; ++i)
        p[i] = b[(big != host_big) ? 3 - i : i];
    }
    void cloudCb(const sensor_msgs::PointCloud2::ConstPtr &m)
    {
      sensor_msgs::PointCloud2::ConstPtr in;
      YawTransform transform;
      {
        std::lock_guard<std::mutex> guard(mutex_);
        if (!ready_ || invalid_ || !sourcesFresh())
          return;
        if (m->header.frame_id != lio_frame_ || !fresh(m->header.stamp, ros::WallTime::now()) ||
            m->point_step == 0 || uint64_t(m->row_step) < uint64_t(m->width) * m->point_step ||
            uint64_t(m->row_step) * m->height > m->data.size())
        {
          ROS_WARN_THROTTLE(3, "Rejecting invalid/stale LIO cloud");
          return;
        }
        in = m;
        transform = world_lio_;
      }
      // Transform and publish outside the lock; this is the only heavy callback.
      std::array<int, 3> xyz{{-1, -1, -1}}, normals{{-1, -1, -1}};
      const std::array<std::string, 3> names{{"x", "y", "z"}}, nn{{"normal_x", "normal_y", "normal_z"}};
      for (const auto &f : in->fields)
        for (int i = 0; i < 3; ++i)
        {
          if (f.datatype == sensor_msgs::PointField::FLOAT32 && f.count == 1 && uint64_t(f.offset) + 4 <= in->point_step)
          {
            if (f.name == names[i])
              xyz[i] = f.offset;
            if (f.name == nn[i])
              normals[i] = f.offset;
          }
        }
      if (xyz[0] < 0 || xyz[1] < 0 || xyz[2] < 0)
      {
        ROS_WARN_THROTTLE(3, "Cloud requires FLOAT32 xyz");
        return;
      }
      sensor_msgs::PointCloud2 out = *in;
      out.header.frame_id = "world";
      for (uint32_t row = 0; row < out.height; ++row)
        for (uint32_t col = 0; col < out.width; ++col)
        {
          uint8_t *data = out.data.data() + size_t(row) * out.row_step + size_t(col) * out.point_step;
          V3 p;
          for (int i = 0; i < 3; ++i)
            p[i] = readFloat(data + xyz[i], out.is_bigendian);
          if (p.allFinite())
          {
            p = transform.point(p);
            for (int i = 0; i < 3; ++i)
              writeFloat(data + xyz[i], out.is_bigendian, p[i]);
          }
          if (normals[0] >= 0 && normals[1] >= 0 && normals[2] >= 0)
          {
            for (int i = 0; i < 3; ++i)
              p[i] = readFloat(data + normals[i], out.is_bigendian);
            if (p.allFinite())
            {
              p = transform.vector(p);
              for (int i = 0; i < 3; ++i)
                writeFloat(data + normals[i], out.is_bigendian, p[i]);
            }
          }
        }
      cloud_pub_.publish(out);
    }
    void cmdCb(const mavros_msgs::PositionTarget::ConstPtr &m)
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (m->type_mask == 0x4000)
      {
        if (!invalid_ && !raw_.armed)
          cmd_pub_.publish(*m); // pre-arm idle heartbeat only
        return;
      }
      if (!ready_ || invalid_ || !sourcesFresh() || m->header.frame_id != "world" ||
          !fresh(m->header.stamp, ros::WallTime::now()) ||
          m->coordinate_frame != mavros_msgs::PositionTarget::FRAME_LOCAL_NED)
        return;
      // A yaw rotation mixes XY; partially enabled horizontal vectors are ambiguous.
      for (int shift : {0, 3, 6})
        if (((m->type_mask >> shift) & 1) != ((m->type_mask >> (shift + 1)) & 1))
          return;
      mavros_msgs::PositionTarget out = *m;
      V3 p = vec(m->position), v = vec(m->velocity), a = vec(m->acceleration_or_force);
      for (int i = 0; i < 3; ++i)
      {
        if (m->type_mask & (1 << i))
          p[i] = 0;
        else if (!std::isfinite(p[i]))
          return;
        if (m->type_mask & (1 << (i + 3)))
          v[i] = 0;
        else if (!std::isfinite(v[i]))
          return;
        if (m->type_mask & (1 << (i + 6)))
          a[i] = 0;
        else if (!std::isfinite(a[i]))
          return;
      }
      put(world_fcu_.inversePoint(p), out.position);
      put(world_fcu_.inverseVector(v), out.velocity);
      put(world_fcu_.inverseVector(a), out.acceleration_or_force);
      if (!(m->type_mask & mavros_msgs::PositionTarget::IGNORE_YAW))
      {
        if (!std::isfinite(m->yaw))
          return;
        out.yaw = world_fcu_.inverseYaw(m->yaw);
      }
      if (!(m->type_mask & mavros_msgs::PositionTarget::IGNORE_YAW_RATE) && !std::isfinite(m->yaw_rate))
        return;
      out.header.frame_id = fcu_frame_;
      cmd_pub_.publish(out);
    }

  public:
    Adapter()
    {
      int id;
      pnh_.param("uav_id", id, 1);
      if (id < 1 || id > 50)
        throw std::runtime_error("uav_id must be in 1..50");
      prefix_ = "/uav" + std::to_string(id);
      lio_frame_ = prefix_.substr(1) + "/lio_init";
      imu_frame_ = prefix_.substr(1) + "/lio_body";
      fcu_frame_ = prefix_.substr(1) + "/fcu_local";
      body_frame_ = prefix_.substr(1) + "/base_link";
      desired_ = V3(param("init_x", 0, false), param("init_y", 0, false), param("init_z", 0, false));
      if (std::abs(desired_.z()) > 1e-6)
        throw std::runtime_error("first version requires common takeoff z=0");
      desired_yaw_ = param("init_yaw", 0, false);
      stable_duration_ = param("stable_duration", 3);
      timeout_ = param("source_timeout", .5);
      // Stamp staleness is the mission signal; the receive watchdog only catches a
      // silent publisher and tolerates scheduler delay under heavy CPU load.
      receive_timeout_ = param("receive_timeout", std::max(1.0, 2 * timeout_));
      skew_ = param("max_source_skew", .15);
      position_tolerance_ = param("stationary_position_tolerance", .03);
      yaw_tolerance_ = param("stationary_yaw_tolerance", .01745329252);
      tilt_limit_ = param("max_initial_tilt", .0872664626);
      speed_limit_ = param("stationary_speed", .05);
      body_in_imu_ = vectorParam("body_in_imu/translation");
      const V3 rpy = vectorParam("body_in_imu/rpy");
      body_rotation_ = Eigen::AngleAxisd(rpy.z(), V3::UnitZ()) * Eigen::AngleAxisd(rpy.y(), V3::UnitY()) * Eigen::AngleAxisd(rpy.x(), V3::UnitX());
      resetWindow();
      odom_pub_ = nh_.advertise<nav_msgs::Odometry>(prefix_ + "/world/odom", 10);
      state_pub_ = nh_.advertise<prometheus_msgs::DroneState>(prefix_ + "/prometheus/drone_state", 10);
      cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(prefix_ + "/world/cloud", 2);
      cmd_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(prefix_ + "/mavros/setpoint_raw/local", 10);
      ready_pub_ = nh_.advertise<std_msgs::Bool>(prefix_ + "/frame_ready", 1, true);
      std_msgs::Bool initial;
      initial.data = false;
      ready_pub_.publish(initial);
      lio_sub_ = nh_.subscribe(prefix_ + "/drone_Odom_high_freq", 30, &Adapter::lioCb, this);
      fcu_sub_ = nh_.subscribe(prefix_ + "/mavros/local_position/odom", 30, &Adapter::fcuCb, this);
      raw_sub_ = nh_.subscribe(prefix_ + "/local/drone_state", 10, &Adapter::rawCb, this);
      cloud_sub_ = nh_.subscribe(prefix_ + "/drone_cloud_registered", 2, &Adapter::cloudCb, this);
      cmd_sub_ = nh_.subscribe(prefix_ + "/world/setpoint_raw/local", 10, &Adapter::cmdCb, this);
      timer_ = nh_.createWallTimer(ros::WallDuration(.02), &Adapter::tick, this);
      ROS_INFO("Frame adapter waiting for disarmed, level, stationary LIO/FCU feedback");
    }
  };
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_frame_adapter");
  try
  {
    Adapter adapter;
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();
  }
  catch (const std::exception &e)
  {
    ROS_FATAL_STREAM(e.what());
    return 1;
  }
  return 0;
}
