#ifndef FAST_LIO_TRUNK_MAPPER_H
#define FAST_LIO_TRUNK_MAPPER_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fast_lio_trunks {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

struct Cylinder {
    int id = 0;
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    Eigen::Vector3f axis = Eigen::Vector3f::UnitZ();
    float radius = 0.f;
    float height = 0.f;
    float coverage = 0.f;  // radians of observed circumference
    int observations = 1;  // number of batches associated with this track
};

Cloud voxelize(const Cloud& cloud, float leaf_size);

class TrunkMapper {
public:
    explicit TrunkMapper(float voxel = 0.035f);
    void update(const Cloud& cloud, const Eigen::Vector3f& up);
    std::vector<Cylinder> models() const;

private:
    struct Track {
        Cylinder model;
        Cloud inliers;
        std::uint64_t last_seen;
    };
    float voxel_;
    int next_id_ = 1;
    std::uint64_t batch_ = 0;
    std::vector<Track> tracks_;
};

}  // namespace fast_lio_trunks

#endif
