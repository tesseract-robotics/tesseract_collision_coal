#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <memory>
#include <stdexcept>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/collision/coal/coal_discrete_managers.h>
#include <tesseract/geometry/impl/box.h>

using namespace tesseract::collision;

namespace
{
/** @brief Add a unit box collision object at the origin */
template <typename ManagerType>
void addBox(ManagerType& checker, const tesseract::common::LinkId& id)
{
  CollisionShapePtr box = std::make_shared<tesseract::geometry::Box>(1, 1, 1);
  CollisionShapesConst shapes{ box };
  tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject(id, 0, shapes, poses);
}

/** @brief Move two boxes into contact with a single one-pose parallel-array call and confirm the broadphase sees it */
template <typename ManagerType>
void runOnePoseArraySetterTest()
{
  ManagerType checker;
  addBox(checker, tesseract::common::LinkId("box_a"));
  addBox(checker, tesseract::common::LinkId("box_b"));
  checker.setActiveCollisionObjects({ tesseract::common::LinkId("box_a"), tesseract::common::LinkId("box_b") });
  checker.setDefaultCollisionMargin(0.0);

  // Apart
  const std::vector<tesseract::common::LinkId> ids{ "box_a", "box_b" };
  Eigen::Isometry3d far_a{ Eigen::Isometry3d::Identity() };
  far_a.translation() = Eigen::Vector3d(-10, 0, 0);
  Eigen::Isometry3d far_b{ Eigen::Isometry3d::Identity() };
  far_b.translation() = Eigen::Vector3d(10, 0, 0);
  checker.setCollisionObjectsTransform(ids, tesseract::common::VectorIsometry3d{ far_a, far_b });

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_TRUE(result.empty());

  // Overlapping, set in one call
  Eigen::Isometry3d near_a{ Eigen::Isometry3d::Identity() };
  near_a.translation() = Eigen::Vector3d(-0.25, 0, 0);
  Eigen::Isometry3d near_b{ Eigen::Isometry3d::Identity() };
  near_b.translation() = Eigen::Vector3d(0.25, 0, 0);
  checker.setCollisionObjectsTransform(ids, tesseract::common::VectorIsometry3d{ near_a, near_b });

  result.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(result.empty());

  EXPECT_TRUE(checker.getCollisionObjectsTransform(tesseract::common::LinkId("box_a")).isApprox(near_a, 1e-8));
  EXPECT_TRUE(checker.getCollisionObjectsTransform(tesseract::common::LinkId("box_b")).isApprox(near_b, 1e-8));
}
}  // namespace

// Two boxes are moved into contact with a single parallel-array call. If the array setter does not apply its
// updates to the broadphase, contactTest sees the stale poses and reports nothing: contactTest does not refresh
// the broadphase itself.
TEST(CoalBatchedTransformUnit, DiscreteArraySetterUpdatesBroadphase)  // NOLINT
{
  runOnePoseArraySetterTest<tesseract_collision_coal::CoalDiscreteBVHManager>();
}

// A cast sweep set through the array setter must produce the same contact as the same sweep set one object at a time.
TEST(CoalBatchedTransformUnit, CastArraySetterUpdatesBroadphase)  // NOLINT
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  addBox(checker, tesseract::common::LinkId("box_a"));
  addBox(checker, tesseract::common::LinkId("box_b"));
  checker.setActiveCollisionObjects({ tesseract::common::LinkId("box_a") });
  checker.setDefaultCollisionMargin(0.0);

  Eigen::Isometry3d start{ Eigen::Isometry3d::Identity() };
  start.translation() = Eigen::Vector3d(-5, 0, 0);
  Eigen::Isometry3d end{ Eigen::Isometry3d::Identity() };
  end.translation() = Eigen::Vector3d(5, 0, 0);

  const std::vector<tesseract::common::LinkId> ids{ "box_a" };
  checker.setCollisionObjectsTransform(
      ids, tesseract::common::VectorIsometry3d{ start }, tesseract::common::VectorIsometry3d{ end });

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(result.empty());  // the swept box passes through box_b at the origin
}

// The cast manager's one-pose array form positions objects without a sweep; it must apply its updates to the
// broadphase just like the discrete form, with a zero-length sweep reporting the same overlap.
TEST(CoalBatchedTransformUnit, CastOnePoseArraySetterUpdatesBroadphase)  // NOLINT
{
  runOnePoseArraySetterTest<tesseract_collision_coal::CoalCastBVHManager>();
}

// Unregistered ids are skipped, exactly as the single-object setter skips them.
TEST(CoalBatchedTransformUnit, DiscreteArraySetterSkipsUnknownIds)  // NOLINT
{
  tesseract_collision_coal::CoalDiscreteBVHManager checker;
  addBox(checker, tesseract::common::LinkId("box_a"));

  Eigen::Isometry3d pose{ Eigen::Isometry3d::Identity() };
  pose.translation() = Eigen::Vector3d(1, 2, 3);
  const std::vector<tesseract::common::LinkId> ids{ "box_a", "does_not_exist" };

  EXPECT_NO_THROW(checker.setCollisionObjectsTransform(  // NOLINT
      ids,
      tesseract::common::VectorIsometry3d{ pose, Eigen::Isometry3d::Identity() }));
  EXPECT_TRUE(checker.getCollisionObjectsTransform(tesseract::common::LinkId("box_a")).isApprox(pose, 1e-8));
}

// The size check lives in the base class; an override must not drop it.
TEST(CoalBatchedTransformUnit, ArraySetterSizeMismatchThrows)  // NOLINT
{
  tesseract_collision_coal::CoalDiscreteBVHManager discrete;
  tesseract_collision_coal::CoalCastBVHManager cast;
  const std::vector<tesseract::common::LinkId> ids{ "box_a", "box_b" };
  const tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };

  EXPECT_THROW(discrete.setCollisionObjectsTransform(ids, poses), std::runtime_error);     // NOLINT
  EXPECT_THROW(cast.setCollisionObjectsTransform(ids, poses), std::runtime_error);         // NOLINT
  EXPECT_THROW(cast.setCollisionObjectsTransform(ids, poses, poses), std::runtime_error);  // NOLINT
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
