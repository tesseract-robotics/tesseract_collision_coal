#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <memory>
#include <stdexcept>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/collision/coal/coal_utils.h>
#include <tesseract/geometry/impl/plane.h>

using namespace tesseract::collision;
using namespace tesseract::collision::tesseract_collision_coal;

namespace
{
/// Add the link plane_link, which holds the plane z = 0.
bool addPlaneLink(CoalCastBVHManager& checker)
{
  const CollisionShapesConst shapes{ std::make_shared<tesseract::geometry::Plane>(0, 0, 1, 0) };
  const tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };
  return checker.addCollisionObject("plane_link", 0, shapes, poses);
}
}  // namespace

TEST(CoalPlaneCastUnit, StaticPlaneLinkIsDeferred)  // NOLINT
{
  CoalCastBVHManager checker;
  ASSERT_TRUE(addPlaneLink(checker));

  const auto* cast_cow = checker.getCastCollisionObject("plane_link");
  ASSERT_NE(cast_cow, nullptr);

  EXPECT_TRUE(castCowNeedsSweptBuild(*cast_cow));
  ASSERT_FALSE(cast_cow->getCollisionObjects().empty());
  EXPECT_EQ(cast_cow->getCollisionObjects()[0]->collisionGeometryPtr()->getNodeType(), coal::GEOM_PLANE);
}

TEST(CoalPlaneCastUnit, PromotingPlaneLinkThrowsNamingTheLink)  // NOLINT
{
  CoalCastBVHManager checker;
  ASSERT_TRUE(addPlaneLink(checker));

  try
  {
    checker.setActiveCollisionObjects({ "plane_link" });
    FAIL() << "expected std::runtime_error";
  }
  catch (const std::runtime_error& e)
  {
    EXPECT_NE(std::string(e.what()).find("plane_link"), std::string::npos) << e.what();
  }
}

TEST(CoalPlaneCastUnit, AddingAPlaneLinkThatIsAlreadyActiveThrows)  // NOLINT
{
  // The other public entry point that promotes a link: addCollisionObject applies the active set to the
  // link it has just added.
  CoalCastBVHManager checker;
  checker.setActiveCollisionObjects({ "plane_link" });

  EXPECT_THROW(addPlaneLink(checker), std::runtime_error);  // NOLINT
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
