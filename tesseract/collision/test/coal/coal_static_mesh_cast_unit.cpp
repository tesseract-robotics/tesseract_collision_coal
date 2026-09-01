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
  const auto& cast_map = checker_.getCastCollisionObjectMap();
  auto it = cast_map.find("mesh_link");
  ASSERT_NE(it, cast_map.end());

  EXPECT_TRUE(castCowNeedsSweptBuild(*it->second));
  ASSERT_FALSE(it->second->getCollisionObjects().empty());
  EXPECT_EQ(it->second->getCollisionObjects()[0]->collisionGeometryPtr()->getNodeType(), coal::BV_OBBRSS);
}

TEST_F(StaticMeshCastUnit, PromotingMeshLinkThrows)  // NOLINT
{
  // The swept-side limitation is unchanged: a mesh still has no castable form.
  EXPECT_THROW(checker_.setActiveCollisionObjects({ "mesh_link" }), std::runtime_error);  // NOLINT
}

TEST_F(StaticMeshCastUnit, AddingAMeshLinkThatIsAlreadyActiveThrows)  // NOLINT
{
  // The other throw site. addCollisionObject applies the active set to the link it has just added, so a
  // mesh named active before it is added throws from the add rather than from setActiveCollisionObjects.
  // Naming a link that does not exist yet is what puts the two in that order.
  checker_.setActiveCollisionObjects({ "sphere_link", "second_mesh_link" });

  CollisionShapesConst shapes{ makeMeshSphere() };
  const tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };

  EXPECT_THROW(checker_.addCollisionObject("second_mesh_link", 0, shapes, poses), std::runtime_error);  // NOLINT
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
