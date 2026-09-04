#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/collision/coal/coal_utils.h>
#include <tesseract/collision/common.h>
#include <tesseract/common/ply_io.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/geometry/geometries.h>

using namespace tesseract::collision;
using namespace tesseract::collision::tesseract_collision_coal;

namespace
{
/// The support mesh is an 80-face icosphere. Every one of its 42 vertices sits at radius 0.25, and no face
/// plane comes closer to the centre than 0.2335, so its surface lies between those two radii in every
/// direction. Tests that place a shape a known distance from it must clear that 0.0165 band.
CollisionShapePtr makeMeshSphere()
{
  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
  auto faces = std::make_shared<Eigen::VectorXi>();
  const tesseract::common::GeneralResourceLocator locator;
  const int num_faces = tesseract::common::loadSimplePlyFile(
      locator.locateResource("package://tesseract/support/meshes/sphere_p25m.ply")->getFilePath(),
      *vertices,
      *faces,
      true);
  EXPECT_GT(num_faces, 0);
  return std::make_shared<tesseract::geometry::Mesh>(vertices, faces);
}
}  // namespace

/**
 * @brief A mesh has no swept form, so it can only be a static target. The cast manager collides a static
 * link through its regular wrapper, so the link is representable as long as its cast wrapper stays
 * deferred; promoting it to active is still an error.
 */
class StaticMeshCastUnit : public ::testing::Test
{
protected:
  void SetUp() override
  {
    CollisionShapesConst mesh_shapes{ makeMeshSphere() };
    const tesseract::common::VectorIsometry3d mesh_poses{ Eigen::Isometry3d::Identity() };
    checker_.addCollisionObject("mesh_link", 0, mesh_shapes, mesh_poses);

    CollisionShapesConst sphere_shapes{ std::make_shared<tesseract::geometry::Sphere>(0.1) };
    const tesseract::common::VectorIsometry3d sphere_poses{ Eigen::Isometry3d::Identity() };
    checker_.addCollisionObject("sphere_link", 0, sphere_shapes, sphere_poses);

    checker_.setDefaultCollisionMargin(0.0);
  }

  CoalCastBVHManager checker_;
};

TEST_F(StaticMeshCastUnit, StaticMeshCastWrapperIsDeferred)  // NOLINT
{
  const auto* cast_cow = checker_.getCastCollisionObject("mesh_link");
  ASSERT_NE(cast_cow, nullptr);

  EXPECT_TRUE(castCowNeedsSweptBuild(*cast_cow));
  ASSERT_FALSE(cast_cow->getCollisionObjects().empty());
  EXPECT_EQ(cast_cow->getCollisionObjects()[0]->collisionGeometryPtr()->getNodeType(), coal::BV_OBBRSS);
}

TEST_F(StaticMeshCastUnit, PromotingMeshLinkThrows)  // NOLINT
{
  // The swept-side limitation is unchanged: a mesh still has no castable form.
  EXPECT_THROW(checker_.setActiveCollisionObjects({ "mesh_link" }), std::runtime_error);  // NOLINT
}

TEST_F(StaticMeshCastUnit, AddingAMeshLinkThatIsAlreadyActiveThrows)  // NOLINT
{
  // The other public entry point that promotes a link. addCollisionObject applies the active set to the
  // link it has just added, so a mesh named active before it is added throws from the add rather than
  // from setActiveCollisionObjects. Naming a link that does not exist yet is what puts the two in that
  // order.
  checker_.setActiveCollisionObjects({ "sphere_link", "second_mesh_link" });

  CollisionShapesConst shapes{ makeMeshSphere() };
  const tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };

  EXPECT_THROW(checker_.addCollisionObject("second_mesh_link", 0, shapes, poses), std::runtime_error);  // NOLINT
}

TEST_F(StaticMeshCastUnit, SweepIntoStaticMeshReportsContact)  // NOLINT
{
  checker_.setActiveCollisionObjects({ "sphere_link" });

  // Sweep from clear of the mesh to its centre. The end pose alone is a collision, so this passes whether
  // or not the swept volume is read; the sweep-only case is covered below.
  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  start.translation() = Eigen::Vector3d(1.0, 0, 0);
  Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
  end.translation() = Eigen::Vector3d(0.0, 0, 0);
  checker_.setCollisionObjectsTransform("sphere_link", start, end);

  ContactResultMap result;
  checker_.contactTest(result, ContactRequest(ContactTestType::ALL));

  ASSERT_FALSE(result.empty());
  auto it = result.find(tesseract::common::LinkIdPair("mesh_link", "sphere_link"));
  ASSERT_NE(it, result.end());
  ASSERT_FALSE(it->second.empty());
  EXPECT_LT(it->second.front().distance, 0.0);
}

TEST_F(StaticMeshCastUnit, SweepPastStaticMeshReportsContact)  // NOLINT
{
  checker_.setActiveCollisionObjects({ "sphere_link" });

  // Both end poses are clear of the mesh; only the swept volume between them intersects it. This is what
  // separates a continuous check from a discrete one at the end pose.
  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  start.translation() = Eigen::Vector3d(1.0, 0, 0);
  Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
  end.translation() = Eigen::Vector3d(-1.0, 0, 0);
  checker_.setCollisionObjectsTransform("sphere_link", start, end);

  ContactResultMap result;
  checker_.contactTest(result, ContactRequest(ContactTestType::ALL));

  ASSERT_FALSE(result.empty());
  auto it = result.find(tesseract::common::LinkIdPair("mesh_link", "sphere_link"));
  ASSERT_NE(it, result.end());
  EXPECT_FALSE(it->second.empty());
}

TEST_F(StaticMeshCastUnit, SweepClearOfStaticMeshReportsNothing)  // NOLINT
{
  checker_.setActiveCollisionObjects({ "sphere_link" });

  // Parallel to the x axis but well clear in y: the swept volume never reaches the mesh.
  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  start.translation() = Eigen::Vector3d(1.0, 2.0, 0);
  Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
  end.translation() = Eigen::Vector3d(-1.0, 2.0, 0);
  checker_.setCollisionObjectsTransform("sphere_link", start, end);

  ContactResultMap result;
  checker_.contactTest(result, ContactRequest(ContactTestType::ALL));

  EXPECT_TRUE(result.empty());
}

TEST_F(StaticMeshCastUnit, NonZeroMarginReportsSeparationAgainstStaticMesh)  // NOLINT
{
  // The distance arm of Coal's function matrix is registered separately from the collision arm, so a
  // near-miss inside the margin exercises a path the penetrating cases above do not.
  checker_.setDefaultCollisionMargin(0.05);
  checker_.setActiveCollisionObjects({ "sphere_link" });

  // The mesh surface lies between radius 0.2335 and 0.25 (see makeMeshSphere), and the sphere has radius
  // 0.1, so an end pose at x = 0.37 leaves a gap somewhere in [0.020, 0.037] whichever way the tessellation
  // falls. That is inside the 0.05 margin with roughly 0.013 to spare, and clear of zero by 0.020. Moving
  // either constant needs that arithmetic redone - the margin has to stay above 0.037 and the gap above 0.
  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  start.translation() = Eigen::Vector3d(1.0, 0, 0);
  Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
  end.translation() = Eigen::Vector3d(0.37, 0, 0);
  checker_.setCollisionObjectsTransform("sphere_link", start, end);

  ContactResultMap result;
  checker_.contactTest(result, ContactRequest(ContactTestType::ALL));

  ASSERT_FALSE(result.empty());
  auto it = result.find(tesseract::common::LinkIdPair("mesh_link", "sphere_link"));
  ASSERT_NE(it, result.end());
  ASSERT_FALSE(it->second.empty());
  EXPECT_GT(it->second.front().distance, 0.0);
  EXPECT_LT(it->second.front().distance, 0.05);
}

TEST_F(StaticMeshCastUnit, CloneKeepsStaticMeshCollidable)  // NOLINT
{
  // clone() re-adds every link through addCollisionObjects, which is the second deferral call site.
  checker_.setActiveCollisionObjects({ "sphere_link" });

  auto clone = checker_.clone();
  auto* cast_clone = dynamic_cast<CoalCastBVHManager*>(clone.get());
  ASSERT_NE(cast_clone, nullptr);

  const auto* cast_cow = cast_clone->getCastCollisionObject("mesh_link");
  ASSERT_NE(cast_cow, nullptr);
  EXPECT_TRUE(castCowNeedsSweptBuild(*cast_cow));

  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  start.translation() = Eigen::Vector3d(1.0, 0, 0);
  Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
  end.translation() = Eigen::Vector3d(0.0, 0, 0);
  cast_clone->setCollisionObjectsTransform("sphere_link", start, end);

  ContactResultMap result;
  cast_clone->contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(result.empty());
}

TEST_F(StaticMeshCastUnit, DualPoseTransformOnStaticMeshIsHarmless)  // NOLINT
{
  // The mesh link is static, so its cast wrapper is deferred and holds the mesh's own BVHModel, not a
  // CastHullShape. A dual-pose transform must not run the static_cast<CastHullShape*> in
  // updateCastShapeTransforms against it (exercises the StaticFilter guard in collectCastTransformUpdate).
  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.translation() = Eigen::Vector3d(1, 0, 0);

  EXPECT_NO_THROW(checker_.setCollisionObjectsTransform("mesh_link", pose1, pose2));

  // Promote the sphere and sweep it into the mesh; the mesh must still be a working collision target.
  checker_.setActiveCollisionObjects({ "sphere_link" });

  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  start.translation() = Eigen::Vector3d(1.0, 0, 0);
  Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
  end.translation() = Eigen::Vector3d(0.0, 0, 0);
  checker_.setCollisionObjectsTransform("sphere_link", start, end);

  ContactResultMap result;
  checker_.contactTest(result, ContactRequest(ContactTestType::ALL));

  ASSERT_FALSE(result.empty());
  auto it = result.find(tesseract::common::LinkIdPair("mesh_link", "sphere_link"));
  ASSERT_NE(it, result.end());
  EXPECT_FALSE(it->second.empty());
}

TEST_F(StaticMeshCastUnit, DualPoseTransformOnDisabledStaticMeshIsHarmless)  // NOLINT
{
  // Disabling the link takes collectCastTransformUpdate's !m_enabled branch, which runs before the
  // StaticFilter gate and writes cow->setCollisionObjectsTransform(pose1) straight into the deferred
  // wrapper. That call is safe only because CollisionObjectWrapper::setCollisionObjectsTransform performs
  // no downcast on the collision objects it walks.
  checker_.disableCollisionObject("mesh_link");

  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.translation() = Eigen::Vector3d(1, 0, 0);

  EXPECT_NO_THROW(checker_.setCollisionObjectsTransform("mesh_link", pose1, pose2));

  // A disabled link is excluded from collision checks outright (needsCollisionCheck requires both objects
  // enabled), so sweeping the sphere to where the mesh sits must report nothing, unlike the enabled case.
  checker_.setActiveCollisionObjects({ "sphere_link" });

  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  start.translation() = Eigen::Vector3d(1.0, 0, 0);
  Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
  end.translation() = Eigen::Vector3d(0.0, 0, 0);
  checker_.setCollisionObjectsTransform("sphere_link", start, end);

  ContactResultMap result;
  checker_.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_TRUE(result.empty());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
