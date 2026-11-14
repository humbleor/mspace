#include "trunk_worker.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <utility>

#include <fast_lio/TreeCylinderArray.h>
#include <visualization_msgs/MarkerArray.h>

namespace fast_lio_trunks {
namespace {
constexpr std::size_t kQueueFrames = 4;
constexpr std::size_t kBatchPoints = 15000;
constexpr std::size_t kFramePoints = 6000;

double param(ros::NodeHandle& nh, const std::string& key, double fallback,
             double minimum, double maximum)
{
    double value = nh.param(key, fallback);
    if (std::isfinite(value) && value >= minimum && value <= maximum)
        return value;
    ROS_WARN_STREAM("Invalid " << nh.resolveName(key) << "; using " << fallback);
    return fallback;
}
}  // namespace

TrunkWorker::TrunkWorker(ros::NodeHandle& nh, const std::string& frame)
    : frame_(frame),
      publisher_(nh.advertise<visualization_msgs::MarkerArray>("tree_trunks", 1, true)),
      data_publisher_(nh.advertise<fast_lio::TreeCylinderArray>("tree_trunks_data", 1, true)),
      window_sec_(param(nh, "tree_trunks/window_sec", 2.0, 0.5, 20.0)),
      range_(param(nh, "tree_trunks/range_m", 7.0, 1.0, 25.0)),
      leaf_size_(param(nh, "tree_trunks/voxel_m", 0.035, 0.02, 0.08)),
      dump_path_(nh.param<std::string>("tree_trunks/dump_path", "")),
      thread_(&TrunkWorker::run, this)
{
}

TrunkWorker::~TrunkWorker()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
        queue_.clear();
    }
    ready_.notify_one();
    thread_.join();
}

void TrunkWorker::submit(Scan::ConstPtr scan, const Eigen::Isometry3f& pose,
                         const Eigen::Vector3f& up, double stamp)
{
    if (!scan || scan->empty() || !pose.matrix().allFinite() || !up.allFinite() ||
        up.squaredNorm() < 1e-6f || !std::isfinite(stamp) || stamp < 0.0)
        return;
    bool dropped = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() == kQueueFrames) {
            queue_.pop_front();
            dropped = true;
        }
        queue_.push_back({std::move(scan), pose, up.normalized(), stamp});
    }
    ready_.notify_one();
    if (dropped)
        ROS_WARN_THROTTLE(5.0, "Tree trunk worker is busy; dropping older scans.");
}

void TrunkWorker::run()
{
    TrunkMapper mapper(leaf_size_);
    Cloud batch;
    batch.reserve(kBatchPoints);
    double start = -1.0, previous = -1.0;
    for (;;) {
        Frame frame;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            ready_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
            if (stopping_)
                return;
            frame = std::move(queue_.front());
            queue_.pop_front();
        }
        if (frame.stamp < previous) {
            mapper = TrunkMapper(leaf_size_);
            batch.clear();
            start = -1.0;
            publish({}, frame.stamp);
        }
        previous = frame.stamp;
        if (start < 0.0)
            start = frame.stamp;

        Cloud near;
        for (const auto& point : *frame.scan) {
            const Eigen::Vector3f p = point.getVector3fMap();
            if (!p.allFinite() || p.squaredNorm() > range_ * range_)
                continue;
            const Eigen::Vector3f world = frame.pose * p;
            near.push_back(pcl::PointXYZ(world.x(), world.y(), world.z()));
        }
        const Cloud down = voxelize(near, leaf_size_);
        const auto count = std::min(kFramePoints, down.size());
        for (std::size_t i = 0; i < count; ++i)
            batch.push_back(down[i * down.size() / count]);
        if (batch.size() >= kBatchPoints)
            batch = voxelize(batch, leaf_size_);

        if (frame.stamp - start >= window_sec_ || batch.size() >= kBatchPoints) {
            const double span = frame.stamp - start;
            const auto before = std::chrono::steady_clock::now();
            mapper.update(batch, frame.up);
            const auto cylinders = mapper.models();
            publish(cylinders, frame.stamp);
            const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - before).count();
            ROS_DEBUG_STREAM("Tree trunks: " << batch.size() << " batch points over "
                             << span << " s, " << cylinders.size() << " cylinders, "
                             << ms << " ms");
            batch.clear();
            start = -1.0;
        }
    }
}

void TrunkWorker::publish(const std::vector<Cylinder>& cylinders, double stamp)
{
    visualization_msgs::MarkerArray msg;
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_;
    marker.header.stamp.fromSec(stamp);
    marker.ns = "tree_trunks";
    // A latched message is a complete snapshot, including removal of evicted tracks.
    marker.action = visualization_msgs::Marker::DELETEALL;
    msg.markers.push_back(marker);
    for (const auto& cylinder : cylinders) {
        marker.id = cylinder.id;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = cylinder.center.x();
        marker.pose.position.y = cylinder.center.y();
        marker.pose.position.z = cylinder.center.z();
        const auto orientation = Eigen::Quaternionf::FromTwoVectors(
            Eigen::Vector3f::UnitZ(), cylinder.axis);
        marker.pose.orientation.x = orientation.x();
        marker.pose.orientation.y = orientation.y();
        marker.pose.orientation.z = orientation.z();
        marker.pose.orientation.w = orientation.w();
        marker.scale.x = marker.scale.y = 2.f * cylinder.radius;
        marker.scale.z = cylinder.height;
        marker.color.r = 0.2f;
        marker.color.g = 0.8f;
        marker.color.b = 0.4f;
        marker.color.a = 0.8f;
        msg.markers.push_back(marker);
    }
    publisher_.publish(msg);

    fast_lio::TreeCylinderArray data;
    data.header.frame_id = frame_;
    data.header.stamp.fromSec(stamp);
    data.trees.reserve(cylinders.size());
    for (const auto& cylinder : cylinders) {
        const Eigen::Vector3f base =
            cylinder.center - 0.5f * cylinder.height * cylinder.axis;
        fast_lio::TreeCylinder tree;
        tree.id = cylinder.id;
        tree.x = base.x();
        tree.y = base.y();
        tree.z_base = base.z();
        tree.height = cylinder.height;
        tree.radius = cylinder.radius;
        tree.axis.x = cylinder.axis.x();
        tree.axis.y = cylinder.axis.y();
        tree.axis.z = cylinder.axis.z();
        tree.coverage = cylinder.coverage;
        tree.observations = static_cast<decltype(tree.observations)>(cylinder.observations);
        data.trees.push_back(tree);
    }
    data_publisher_.publish(data);
    dump(cylinders, stamp);
}

void TrunkWorker::dump(const std::vector<Cylinder>& cylinders, double stamp) const
{
    if (dump_path_.empty())
        return;
    const std::string temporary = dump_path_ + ".tmp";
    std::ofstream out(temporary, std::ios::trunc);
    if (!out) {
        ROS_WARN_THROTTLE(10.0, "Cannot write tree trunk dump: %s", dump_path_.c_str());
        return;
    }
    out << "# frame_id=" << frame_ << "\n";
    out << "# stamp=" << std::fixed << std::setprecision(9) << stamp << "\n";
    out << "id,x,y,z_base,height,radius,axis_x,axis_y,axis_z,coverage,observations\n";
    for (const auto& cylinder : cylinders) {
        const Eigen::Vector3f base = cylinder.center - 0.5f * cylinder.height * cylinder.axis;
        out << cylinder.id << ',' << base.x() << ',' << base.y()
            << ',' << base.z() << ',' << cylinder.height << ',' << cylinder.radius
            << ',' << cylinder.axis.x() << ',' << cylinder.axis.y()
            << ',' << cylinder.axis.z() << ',' << cylinder.coverage
            << ',' << cylinder.observations << '\n';
    }
    out.close();
    if (std::rename(temporary.c_str(), dump_path_.c_str()) != 0)
        ROS_WARN_THROTTLE(10.0, "Cannot replace tree trunk dump: %s", dump_path_.c_str());
}

}  // namespace fast_lio_trunks
