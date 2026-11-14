#include "trunk_mapper.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <Eigen/Eigenvalues>
#include <pcl/common/centroid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace fast_lio_trunks {
namespace {
constexpr float kPi = 3.14159265f;
constexpr int kMinPoints = 60;
constexpr int kNeighbors = 24;
constexpr float kMinHeight = 1.5f;
constexpr float kMinRadius = 0.035f;
constexpr float kMaxRadius = 0.5f;
constexpr float kRadialTolerance = 0.025f;
// A moving vehicle may only see a narrow arc per batch. Accept that as an
// observation and let the track accumulate coverage across batches.
constexpr float kMinObservationCoverage = 0.35f;
constexpr std::size_t kTrackPoints = 1800;
constexpr std::size_t kTrackLimit = 512;
using Normals = pcl::PointCloud<pcl::Normal>;

bool localNormal(const Cloud& cloud, const std::vector<int>& indices, Eigen::Vector3f& normal)
{
    if (indices.size() < 8)
        return false;
    Eigen::Vector4f center;
    Eigen::Matrix3f covariance;
    pcl::compute3DCentroid(cloud, indices, center);
    pcl::computeCovarianceMatrixNormalized(cloud, indices, center, covariance);
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> pca(covariance);
    if (pca.info() != Eigen::Success)
        return false;
    const Eigen::Vector3f eigenvalues = pca.eigenvalues();
    if (eigenvalues.z() <= 1e-8f ||
        (eigenvalues.y() - eigenvalues.x()) / eigenvalues.z() < 0.12f)
        return false;  // A line or an isotropic cluster has no reliable surface normal.
    normal = pca.eigenvectors().col(0);
    return true;
}

Cloud verticalSurfaces(const Cloud::ConstPtr& cloud, const Eigen::Vector3f& up)
{
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);
    Cloud candidates;
    std::vector<int> indices;
    std::vector<float> distances;
    for (const auto& point : *cloud) {
        tree.radiusSearch(point, 0.16, indices, distances, kNeighbors);
        Eigen::Vector3f normal;
        if (localNormal(*cloud, indices, normal) && std::abs(normal.dot(up)) < 0.45f)
            candidates.push_back(point);
    }
    if (candidates.size() < kMinPoints)
        return candidates;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
    filter.setInputCloud(candidates.makeShared());
    filter.setMeanK(12);
    filter.setStddevMulThresh(2.0);
    Cloud filtered;
    filter.filter(filtered);
    return filtered;
}

float radialError(const Eigen::Vector3f& point, const Cylinder& model)
{
    return std::abs((point - model.center).cross(model.axis).norm() - model.radius);
}

float support(const Cloud& cloud, const Cylinder& model)
{
    const auto count = std::count_if(cloud.begin(), cloud.end(), [&](const pcl::PointXYZ& p) {
        return radialError(p.getVector3fMap(), model) <= kRadialTolerance;
    });
    return cloud.empty() ? 0.f : static_cast<float>(count) / cloud.size();
}

bool fit(const Cloud::ConstPtr& cloud, const Eigen::Vector3f& up, Cylinder& model, Cloud& inliers,
         float min_support = 0.60f)
{
    if (cloud->size() < kMinPoints)
        return false;
    float span_low = std::numeric_limits<float>::infinity(), span_high = -span_low;
    for (const auto& point : *cloud) {
        const float h = point.getVector3fMap().dot(up);
        span_low = std::min(span_low, h);
        span_high = std::max(span_high, h);
    }
    if (span_high - span_low < kMinHeight)
        return false;  // Cheap rejection before normals and RANSAC.

    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);
    Normals::Ptr normals(new Normals);
    Cloud::Ptr valid(new Cloud);
    std::vector<int> indices;
    std::vector<float> distances;
    for (const auto& point : *cloud) {
        tree.radiusSearch(point, 0.14, indices, distances, kNeighbors);
        Eigen::Vector3f normal;
        if (localNormal(*cloud, indices, normal)) {
            pcl::Normal n;
            n.getNormalVector3fMap() = normal;
            n.curvature = 0.f;
            normals->push_back(n);
            valid->push_back(point);
        }
    }
    if (valid->size() < kMinPoints)
        return false;

    // PCA normals have an arbitrary sign; orient them away from the cluster
    // centre so the cylinder model receives consistent outward normals.
    Eigen::Vector4f cluster_centroid;
    pcl::compute3DCentroid(*valid, cluster_centroid);
    const Eigen::Vector3f centre = cluster_centroid.head<3>();
    for (std::size_t i = 0; i < valid->size(); ++i) {
        const Eigen::Vector3f n = normals->points[i].getNormalVector3fMap();
        if (n.dot((*valid)[i].getVector3fMap() - centre) < 0.f)
            normals->points[i].getNormalVector3fMap() = -n;
    }

    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> segmentation;
    segmentation.setInputCloud(valid);
    segmentation.setInputNormals(normals);
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_CYLINDER);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setNormalDistanceWeight(0.025);
    segmentation.setMaxIterations(300);
    segmentation.setDistanceThreshold(kRadialTolerance);
    segmentation.setRadiusLimits(kMinRadius, kMaxRadius);
    segmentation.setAxis(up);
    segmentation.setEpsAngle(30.0 * kPi / 180.0);
    pcl::PointIndices consensus;
    pcl::ModelCoefficients coefficients;
    segmentation.segment(consensus, coefficients);
    if (consensus.indices.size() < kMinPoints || coefficients.values.size() != 7)
        return false;

    const Eigen::Map<const Eigen::Matrix<float, 7, 1>> params(coefficients.values.data());
    if (!params.allFinite() || params.segment<3>(3).norm() < 1e-6f ||
        params[6] < kMinRadius || params[6] > kMaxRadius)
        return false;
    model.center = params.head<3>();
    model.axis = params.segment<3>(3).normalized();
    if (model.axis.dot(up) < 0.f)
        model.axis = -model.axis;
    if (model.axis.dot(up) < std::cos(30.f * kPi / 180.f))
        return false;
    model.radius = params[6];

    const Eigen::Vector3f u = model.axis.unitOrthogonal();
    const Eigen::Vector3f v = model.axis.cross(u);
    float low = std::numeric_limits<float>::infinity(), high = -low;
    float squared_error = 0.f;
    std::vector<float> angles;
    for (int i : consensus.indices) {
        const auto& point = (*valid)[i];
        const float error = radialError(point.getVector3fMap(), model);
        if (error > kRadialTolerance)
            continue;
        const Eigen::Vector3f d = point.getVector3fMap() - model.center;
        const float h = d.dot(model.axis);
        low = std::min(low, h);
        high = std::max(high, h);
        angles.push_back(std::atan2(d.dot(v), d.dot(u)));
        squared_error += error * error;
        inliers.push_back(point);
    }
    if (inliers.size() < kMinPoints || inliers.size() < min_support * cloud->size() ||
        squared_error / inliers.size() > 0.015f * 0.015f || high - low < kMinHeight)
        return false;

    std::sort(angles.begin(), angles.end());
    float gap = angles.front() + 2.f * kPi - angles.back();
    for (std::size_t i = 1; i < angles.size(); ++i)
        gap = std::max(gap, angles[i] - angles[i - 1]);
    model.coverage = 2.f * kPi - gap;
    model.height = high - low;
    model.center += (low + high) * 0.5f * model.axis;
    return model.coverage >= kMinObservationCoverage;
}

float axisDistance(const Cylinder& a, const Cylinder& b, const Eigen::Vector3f& up)
{
    const float ah = a.center.dot(up), bh = b.center.dot(up);
    const float h = (ah + bh) * 0.5f;
    const Eigen::Vector3f p = a.center + a.axis * ((h - ah) / a.axis.dot(up));
    const Eigen::Vector3f q = b.center + b.axis * ((h - bh) / b.axis.dot(up));
    return (p - q).norm();
}

float heightGap(const Cylinder& a, const Cylinder& b, const Eigen::Vector3f& up)
{
    return std::abs((a.center - b.center).dot(up)) -
           0.5f * (a.height * a.axis.dot(up) + b.height * b.axis.dot(up));
}

struct Observation {
    Cylinder model;
    Cloud inliers;
};

std::vector<Observation> fitCluster(const Cloud::ConstPtr& cluster, const Eigen::Vector3f& up)
{
    Observation first;
    if (!fit(cluster, up, first.model, first.inliers, 0.30f))
        return {};
    const float first_ratio = static_cast<float>(first.inliers.size()) / cluster->size();
    if (first_ratio >= 0.85f)
        return {std::move(first)};

    // Euclidean clustering can join two nearby stems. Test the leftover points separately.
    Cloud::Ptr remaining(new Cloud);
    for (const auto& point : *cluster) {
        if (radialError(point.getVector3fMap(), first.model) > kRadialTolerance)
            remaining->push_back(point);
    }
    Observation second;
    if (fit(remaining, up, second.model, second.inliers) &&
        axisDistance(first.model, second.model, up) >= first.model.radius + second.model.radius &&
        first.inliers.size() + second.inliers.size() >= 0.75f * cluster->size())
        return {std::move(first), std::move(second)};
    if (first_ratio >= 0.60f)
        return {std::move(first)};
    return {};
}

Cloud limitTrack(Cloud cloud)
{
    if (cloud.size() <= kTrackPoints)
        return cloud;
    Cloud bounded;
    bounded.reserve(kTrackPoints);
    for (std::size_t i = 0; i < kTrackPoints; ++i)
        bounded.push_back(cloud[i * cloud.size() / kTrackPoints]);
    return bounded;
}
}  // namespace

Cloud voxelize(const Cloud& cloud, float leaf_size)
{
    if (cloud.empty())
        return {};
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.setInputCloud(Cloud::ConstPtr(&cloud, [](const Cloud*) {}));
    Cloud filtered;
    grid.filter(filtered);
    return filtered;
}

TrunkMapper::TrunkMapper(float voxel) : voxel_(voxel) {}

std::vector<Cylinder> TrunkMapper::models() const
{
    std::vector<Cylinder> result;
    result.reserve(tracks_.size());
    for (const auto& track : tracks_)
        result.push_back(track.model);
    return result;
}

void TrunkMapper::update(const Cloud& cloud, const Eigen::Vector3f& up)
{
    ++batch_;
    const auto down = voxelize(cloud, voxel_).makeShared();
    if (down->size() < kMinPoints)
        return;
    const auto candidates = verticalSurfaces(down, up).makeShared();
    if (candidates->size() < kMinPoints)
        return;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
    clustering.setInputCloud(candidates);
    clustering.setClusterTolerance(0.13);
    clustering.setMinClusterSize(kMinPoints);
    clustering.setMaxClusterSize(8000);
    std::vector<pcl::PointIndices> clusters;
    clustering.extract(clusters);
    std::vector<Observation> observations;
    for (const auto& indices : clusters) {
        Cloud::Ptr cluster(new Cloud(*candidates, indices.indices));
        for (auto& observation : fitCluster(cluster, up))
            observations.push_back(std::move(observation));
    }
    for (auto& observation : observations) {
        auto& observed = observation.model;
        auto& observed_inliers = observation.inliers;
        Track* best = nullptr;
        float best_score = std::numeric_limits<float>::infinity();
        for (auto& track : tracks_) {
            const auto& previous = track.model;
            const float d = axisDistance(previous, observed, up);
            const float radius_diff = std::abs(previous.radius - observed.radius);
            const float angle = previous.axis.dot(observed.axis);
            // Partial-arc fits shift the centre by ~0.2 m between batches.
            // The radius-sum test below still rejects physically separate stems.
            if (d > 0.35f || radius_diff > 0.08f || angle < 0.97f ||
                heightGap(previous, observed, up) > 0.6f)
                continue;
            if (d > previous.radius + observed.radius + 0.02f)
                continue;
            const float score = d + radius_diff + (1.f - angle);
            if (score < best_score) {
                best = &track;
                best_score = score;
            }
        }
        if (best) {
            // Fit a temporary union; a failed update cannot contaminate either the model or its stored points.
            // A matching observation is evidence even when the joint fit cannot
            // improve the model; keep the previous geometry in that case.
            best->last_seen = batch_;
            ++best->model.observations;
            Cloud::Ptr merged(new Cloud(voxelize(best->inliers + limitTrack(observed_inliers), voxel_)));
            Cylinder refined;
            Cloud refined_inliers;
            if (fit(merged, up, refined, refined_inliers) &&
                support(best->inliers, refined) >= 0.65f &&
                support(observed_inliers, refined) >= 0.65f &&
                axisDistance(best->model, refined, up) <= 0.05f &&
                std::abs(best->model.radius - refined.radius) <= 0.035f &&
                refined.coverage + 0.25f >= best->model.coverage) {
                refined.id = best->model.id;
                refined.observations = best->model.observations;
                *best = Track{refined, limitTrack(std::move(refined_inliers)), batch_};
            }
            continue;
        }
        if (tracks_.size() == kTrackLimit) {
            const auto oldest = std::min_element(tracks_.begin(), tracks_.end(),
                [](const Track& a, const Track& b) { return a.last_seen < b.last_seen; });
            tracks_.erase(oldest);
        }
        observed.id = next_id_++;
        tracks_.push_back({observed, limitTrack(std::move(observed_inliers)), batch_});
    }
}

}  // namespace fast_lio_trunks
