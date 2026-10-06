#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <memory>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/collision/coal/coal_utils.h>
#include <tesseract/geometry/geometries.h>

using namespace tesseract::collision;
using tesseract::collision::tesseract_collision_coal::CoalCastBVHManager;
using tesseract::common::LinkId;

namespace
{
constexpr double BOX = 0.2;

Eigen::Isometry3d at(double x, double y = 0.0, double z = 0.0)
{
  Eigen::Isometry3d tf{ Eigen::Isometry3d::Identity() };
  tf.translation() = Eigen::Vector3d(x, y, z);
  return tf;
}

void addShape(CoalCastBVHManager& checker,
              const std::string& link,
              const CollisionShapeConstPtr& shape,
              const Eigen::Isometry3d& shape_pose = Eigen::Isometry3d::Identity())
{
  const CollisionShapesConst shapes{ shape };
  const tesseract::common::VectorIsometry3d poses{ shape_pose };
  checker.addCollisionObject(LinkId(link), 0, shapes, poses, true);
}

void addBox(CoalCastBVHManager& checker,
            const std::string& link,
            const Eigen::Isometry3d& shape_pose = Eigen::Isometry3d::Identity())
{
  addShape(checker, link, std::make_shared<tesseract::geometry::Box>(BOX, BOX, BOX), shape_pose);
}

constexpr double GAP = 0.05;

ContactResultVector contacts(ContinuousContactManager& checker,
                             const ContactRequest& request = ContactRequest(ContactTestType::ALL))
{
  ContactResultMap result;
  checker.contactTest(result, request);
  ContactResultVector flat;
  result.flattenMoveResults(flat);
  return flat;
}

/// The one contact @p checker is expected to find; a contact with no data where it finds another number.
ContactResult onlyContact(ContinuousContactManager& checker,
                          const ContactRequest& request = ContactRequest(ContactTestType::ALL))
{
  const ContactResultVector found = contacts(checker, request);
  EXPECT_EQ(found.size(), 1U);
  return (found.size() == 1U) ? found[0] : ContactResult();
}

/// The slot @p link occupies in @p contact.
std::size_t slotOf(const ContactResult& contact, const std::string& link)
{
  return (contact.link_ids[0].name() == link) ? 0U : 1U;
}
}  // namespace

TEST(CoalCastMovingPairsUnit, CastWrapperRecordsWhereItsSweepEnds)  // NOLINT
{
  CoalCastBVHManager checker;
  addBox(checker, "a");
  checker.setActiveCollisionObjects({ LinkId("a") });

  checker.setCollisionObjectsTransform("a", at(1.0), at(2.0));
  const auto* cast = checker.getCastCollisionObject("a");
  ASSERT_NE(cast, nullptr);
  EXPECT_TRUE(cast->getCollisionObjectsTransform().isApprox(at(1.0)));
  EXPECT_TRUE(cast->getSweepEndTransform().isApprox(at(2.0)));

  // A pose set without a sweep leaves none behind.
  checker.setCollisionObjectsTransform("a", at(3.0));
  cast = checker.getCastCollisionObject("a");
  ASSERT_NE(cast, nullptr);
  EXPECT_TRUE(cast->getSweepEndTransform().isApprox(at(3.0)));
}

TEST(CoalCastMovingPairsUnit, SweepSetFromTheWrappersOwnPosesIsKept)  // NOLINT
{
  CoalCastBVHManager checker;
  addBox(checker, "a");
  checker.setActiveCollisionObjects({ LinkId("a") });
  checker.setCollisionObjectsTransform("a", at(1.0), at(2.0));
  const auto* cast = checker.getCastCollisionObject("a");
  ASSERT_NE(cast, nullptr);

  // The sweep run backwards, each pose read from the wrapper the call writes.
  checker.setCollisionObjectsTransform("a", cast->getSweepEndTransform(), cast->getCollisionObjectsTransform());
  EXPECT_TRUE(cast->isSwept());
  EXPECT_TRUE(cast->getCollisionObjectsTransform().isApprox(at(2.0)));
  EXPECT_TRUE(cast->getSweepEndTransform().isApprox(at(1.0)));
}

TEST(CoalCastMovingPairsUnit, SweepSetOnADisabledLinkIsCheckedOnceEnabled)  // NOLINT
{
  CoalCastBVHManager checker;
  addBox(checker, "s");
  addBox(checker, "m");
  // Further active links, out of reach, so that m is not the whole of the broadphase.
  for (const std::string far_link : { "f1", "f2", "f3" })
    addBox(checker, far_link);
  checker.setActiveCollisionObjects({ LinkId("m"), LinkId("f1"), LinkId("f2"), LinkId("f3") });
  checker.setDefaultCollisionMargin(0.1);
  checker.setCollisionObjectsTransform("s", at(0.0));
  checker.setCollisionObjectsTransform("f1", at(50.0), at(51.0));
  checker.setCollisionObjectsTransform("f2", at(50.0, 50.0), at(51.0, 50.0));
  checker.setCollisionObjectsTransform("f3", at(50.0, 0.0, 50.0), at(51.0, 0.0, 50.0));

  // Far from the static box and unswept, m touches nothing.
  checker.setCollisionObjectsTransform("m", at(5.0));
  ASSERT_TRUE(contacts(checker).empty());

  // A sweep set while m is disabled is the one it is checked along once enabled. It ends beside the box,
  // where neither its start pose nor the pose m held before puts it.
  checker.disableCollisionObject("m");
  checker.setCollisionObjectsTransform("m", at(0.8), at(BOX + GAP));
  EXPECT_TRUE(contacts(checker).empty());
  checker.enableCollisionObject("m");
  const auto* cast = checker.getCastCollisionObject("m");
  ASSERT_NE(cast, nullptr);
  EXPECT_TRUE(cast->isSwept());
  EXPECT_TRUE(cast->getCollisionObjectsTransform().isApprox(at(0.8)));
  EXPECT_TRUE(cast->getSweepEndTransform().isApprox(at(BOX + GAP)));
  const ContactResult contact = onlyContact(checker);
  const std::size_t m = slotOf(contact, "m");
  EXPECT_NEAR(contact.distance, GAP, 1e-5);
  EXPECT_TRUE(contact.cc_transform[m].isApprox(at(BOX + GAP)));

  // It also replaces the sweep m held: set far from the box while disabled, m no longer reaches it.
  checker.disableCollisionObject("m");
  checker.setCollisionObjectsTransform("m", at(5.0), at(5.5));
  checker.enableCollisionObject("m");
  EXPECT_TRUE(contacts(checker).empty());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
