#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <stdexcept>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/collision/common.h>
#include <tesseract/collision/test_suite/collision_link_fixtures.hpp>

using namespace tesseract::collision;
using namespace tesseract_collision_coal;

/**
 * @brief Every setter taking parallel containers rejects a size mismatch the same way.
 *
 * A swept hull needs both endpoints, so a caller whose two containers fail to line up -- by length, or
 * by which links they name -- has made one mistake whichever overload they reached for. The vector
 * forms have always thrown; the map pair only asserted, which is compiled out of a release build, and
 * there the unmatched links keep their previous swept hull -- a stale volume, reported as a missed
 * collision rather than as an error.
 *
 * Links absent from both maps remain a legitimate partial update and are not this file's subject.
 */
class TransformOverloadContractUnit : public ::testing::Test
{
protected:
  void SetUp() override
  {
    for (const auto& link : { std::string("sphere_a"), std::string("sphere_b") })
      test_suite::detail::addSphereLink(checker_, link, 0.5);
    checker_.setActiveCollisionObjects({ "sphere_a", "sphere_b" });
    checker_.setDefaultCollisionMargin(0.0);
  }

  CoalCastBVHManager checker_;
};

TEST_F(TransformOverloadContractUnit, MapPairThrowsOnMismatchedSizes)  // NOLINT
{
  tesseract::common::LinkIdTransformMap pose1;
  pose1["sphere_a"] = Eigen::Isometry3d::Identity();
  pose1["sphere_b"] = Eigen::Isometry3d::Identity();

  tesseract::common::LinkIdTransformMap pose2;
  pose2["sphere_a"] = Eigen::Isometry3d::Identity();

  EXPECT_THROW(checker_.setCollisionObjectsTransform(pose1, pose2), std::runtime_error);  // NOLINT
}

TEST_F(TransformOverloadContractUnit, MapPairThrowsWhenEqualSizedMapsNameDifferentLinks)  // NOLINT
{
  // The sizes agree and the keys do not, which a size comparison cannot see. Without the per-link
  // check the unmatched link keeps its previous swept hull.
  tesseract::common::LinkIdTransformMap pose1;
  pose1["sphere_a"] = Eigen::Isometry3d::Identity();

  tesseract::common::LinkIdTransformMap pose2;
  pose2["sphere_b"] = Eigen::Isometry3d::Identity();

  EXPECT_THROW(checker_.setCollisionObjectsTransform(pose1, pose2), std::runtime_error);  // NOLINT
}

TEST_F(TransformOverloadContractUnit, MapPairAcceptsMatchedSizes)  // NOLINT
{
  // The control: the throw must be reachable only by the mistake, not by ordinary use.
  tesseract::common::LinkIdTransformMap pose1;
  pose1["sphere_a"] = Eigen::Isometry3d::Identity();
  pose1["sphere_b"] = Eigen::Isometry3d::Identity();

  EXPECT_NO_THROW(checker_.setCollisionObjectsTransform(pose1, pose1));  // NOLINT
}

TEST_F(TransformOverloadContractUnit, VectorTripleThrowsOnMismatchedSizes)  // NOLINT
{
  std::vector<tesseract::common::LinkId> ids{ "sphere_a", "sphere_b" };
  tesseract::common::VectorIsometry3d pose1{ Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity() };
  tesseract::common::VectorIsometry3d pose2{ Eigen::Isometry3d::Identity() };

  EXPECT_THROW(checker_.setCollisionObjectsTransform(ids, pose1, pose2), std::runtime_error);  // NOLINT
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
