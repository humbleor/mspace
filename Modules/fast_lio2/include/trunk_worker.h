#ifndef FAST_LIO_TRUNK_WORKER_H
#define FAST_LIO_TRUNK_WORKER_H

#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

#include <Eigen/Geometry>
#include <ros/ros.h>

#include "trunk_mapper.h"

namespace fast_lio_trunks {

class TrunkWorker {
public:
    using Scan = pcl::PointCloud<pcl::PointXYZINormal>;

    TrunkWorker(ros::NodeHandle& nh, const std::string& frame);
    ~TrunkWorker();

    // The producer hands off this cloud and will no longer modify it.
    void submit(Scan::ConstPtr scan, const Eigen::Isometry3f& world_from_lidar,
                const Eigen::Vector3f& up, double stamp);

private:
    struct Frame {
        Scan::ConstPtr scan;
        Eigen::Isometry3f pose;
        Eigen::Vector3f up;
        double stamp;
    };

    void run();
    void publish(const std::vector<Cylinder>& cylinders, double stamp);
    void dump(const std::vector<Cylinder>& cylinders, double stamp) const;

    const std::string frame_;
    ros::Publisher publisher_;
    ros::Publisher data_publisher_;
    double window_sec_;
    double range_;
    double leaf_size_;
    std::string dump_path_;
    std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<Frame, Eigen::aligned_allocator<Frame>> queue_;
    bool stopping_ = false;
    std::thread thread_;
};

}  // namespace fast_lio_trunks

#endif
