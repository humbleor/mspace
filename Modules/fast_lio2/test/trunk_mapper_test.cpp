#include "trunk_mapper.h"

#include <cmath>
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>

namespace fast_lio_trunks {
namespace {
constexpr float pi = 3.14159265f;

Cloud arc(float x, float radius, float begin, float end, float z0 = 0.f,
          float tilt = 0.f)
{
    Cloud cloud;
    for (float z = z0; z <= z0 + 2.1f; z += 0.035f) {
        for (float a = begin; a <= end; a += 0.14f) {
            cloud.push_back(pcl::PointXYZ(x + z * tilt + radius * std::cos(a),
                                          radius * std::sin(a), z));
        }
    }
    return cloud;
}

void halfTrunk(Cloud& cloud, float cx, float cy, float radius, float height)
{
    const float facing = std::atan2(-cy, -cx);
    for (float z = 0.f; z <= height; z += 0.035f) {
        for (float a = facing - pi / 2.f; a <= facing + pi / 2.f; a += 0.14f) {
            cloud.push_back(pcl::PointXYZ(cx + radius * std::cos(a),
                                          cy + radius * std::sin(a), z));
        }
    }
}

TEST(TrunkMapper, DetectsSeveralTrunksInOneScene)
{
    Cloud scene;
    const float trunks[][4] = {
        {3.0f, 0.0f, 0.12f, 3.5f},  {-3.0f, 0.5f, 0.15f, 4.0f},
        {0.0f, 3.0f, 0.10f, 3.0f},  {0.0f, -3.0f, 0.18f, 3.8f},
        {2.2f, 2.2f, 0.13f, 3.2f},  {-2.2f, -2.2f, 0.16f, 3.6f},
    };
    for (const auto& trunk : trunks)
        halfTrunk(scene, trunk[0], trunk[1], trunk[2], trunk[3]);
    TrunkMapper mapper;
    mapper.update(scene, Eigen::Vector3f::UnitZ());
    EXPECT_EQ(mapper.models().size(), 6u);
}

TEST(TrunkMapper, FittingIsInvariantToMapTranslation)
{
    const Cloud input = arc(0.f, 0.10f, 0.f, pi);
    Cloud shifted;
    const Eigen::Vector3f offset(35.f, -26.f, 12.f);
    const Eigen::Affine3f transform{Eigen::Translation3f(offset)};
    pcl::transformPointCloud(input, shifted, transform);
    TrunkMapper a, b;
    a.update(input, Eigen::Vector3f::UnitZ());
    b.update(shifted, Eigen::Vector3f::UnitZ());
    ASSERT_EQ(a.models().size(), 1u);
    ASSERT_EQ(b.models().size(), 1u);
    EXPECT_LT((b.models()[0].center - a.models()[0].center - offset).norm(), 0.04f);
    EXPECT_NEAR(b.models()[0].radius, a.models()[0].radius, 0.015f);
}

TEST(TrunkMapper, FusesOppositeViewsAndKeepsNearbyTrunkSeparate)
{
    TrunkMapper mapper;
    mapper.update(arc(0.f, 0.10f, 0.f, pi), Eigen::Vector3f::UnitZ());
    ASSERT_EQ(mapper.models().size(), 1u);
    const auto first = mapper.models()[0];
    mapper.update(arc(0.f, 0.10f, pi, 2.f * pi), Eigen::Vector3f::UnitZ());
    ASSERT_EQ(mapper.models().size(), 1u);
    EXPECT_EQ(mapper.models()[0].id, first.id);
    EXPECT_GT(mapper.models()[0].coverage, first.coverage + 1.f);
    EXPECT_NEAR(mapper.models()[0].radius, 0.10f, 0.02f);
    mapper.update(arc(0.27f, 0.10f, 0.f, pi), Eigen::Vector3f::UnitZ());
    EXPECT_EQ(mapper.models().size(), 2u);
}

TEST(TrunkMapper, AcceptsNarrowArcAsFirstObservation)
{
    TrunkMapper mapper;
    mapper.update(arc(0.f, 0.15f, 0.f, 1.f), Eigen::Vector3f::UnitZ());
    ASSERT_EQ(mapper.models().size(), 1u);
    EXPECT_EQ(mapper.models()[0].observations, 1);
    EXPECT_LT(mapper.models()[0].coverage, 1.f);
}

TEST(TrunkMapper, ConfirmsTrackAfterTwoNarrowArcs)
{
    TrunkMapper mapper;
    mapper.update(arc(0.f, 0.15f, 0.f, 1.f), Eigen::Vector3f::UnitZ());
    ASSERT_EQ(mapper.models().size(), 1u);
    mapper.update(arc(0.f, 0.15f, pi, pi + 1.f), Eigen::Vector3f::UnitZ());
    ASSERT_EQ(mapper.models().size(), 1u);
    EXPECT_EQ(mapper.models()[0].observations, 2);
    EXPECT_GT(mapper.models()[0].coverage, 1.f);
}

TEST(TrunkMapper, AssociatesJitteredPartialViewsOfTheSameTrunk)
{
    TrunkMapper mapper;
    mapper.update(arc(0.f, 0.15f, 0.f, pi), Eigen::Vector3f::UnitZ());
    ASSERT_EQ(mapper.models().size(), 1u);
    // A partial-arc fit can shift the centre by a few centimetres between batches.
    mapper.update(arc(0.18f, 0.15f, 0.f, pi), Eigen::Vector3f::UnitZ());
    ASSERT_EQ(mapper.models().size(), 1u);
    EXPECT_GE(mapper.models()[0].observations, 2);
}

TEST(TrunkMapper, AssociatesSlantedSegmentsAtTheSameHeight)
{
    TrunkMapper mapper;
    mapper.update(arc(0.f, 0.14f, 0.f, pi, 0.f, 0.18f), Eigen::Vector3f::UnitZ());
    ASSERT_EQ(mapper.models().size(), 1u);
    mapper.update(arc(0.f, 0.14f, pi, 2.f * pi, 1.5f, 0.18f), Eigen::Vector3f::UnitZ());
    ASSERT_EQ(mapper.models().size(), 1u);
    EXPECT_GT(mapper.models()[0].height, 3.f);
}

TEST(TrunkMapper, ConflictingObservationCannotOverwriteHistory)
{
    TrunkMapper mapper;
    mapper.update(arc(0.f, 0.10f, 0.f, pi), Eigen::Vector3f::UnitZ());
    ASSERT_EQ(mapper.models().size(), 1u);
    const auto first = mapper.models()[0];
    mapper.update(arc(0.11f, 0.10f, pi, 2.f * pi), Eigen::Vector3f::UnitZ());
    ASSERT_EQ(mapper.models().size(), 1u);
    EXPECT_EQ(mapper.models()[0].id, first.id);
    EXPECT_NEAR(mapper.models()[0].radius, first.radius, 0.005f);
    EXPECT_LT((mapper.models()[0].center - first.center).norm(), 0.01f);
    EXPECT_NEAR(mapper.models()[0].coverage, first.coverage, 0.01f);
}

TEST(TrunkMapper, FlatWallIsNotATrunk)
{
    Cloud wall;
    for (float y = -0.4f; y <= 0.4f; y += 0.03f)
        for (float z = 0.f; z <= 2.1f; z += 0.03f)
            wall.push_back(pcl::PointXYZ(0.f, y, z));
    TrunkMapper mapper;
    mapper.update(wall, Eigen::Vector3f::UnitZ());
    EXPECT_TRUE(mapper.models().empty());
}

TEST(TrunkMapper, SeparatesNearbyStemsInTheSameBatch)
{
    TrunkMapper mapper;
    mapper.update(arc(0.f, 0.10f, 0.f, 2.f * pi) +
                  arc(0.27f, 0.10f, 0.f, 2.f * pi), Eigen::Vector3f::UnitZ());
    EXPECT_EQ(mapper.models().size(), 2u);
}
}  // namespace
}  // namespace fast_lio_trunks

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
