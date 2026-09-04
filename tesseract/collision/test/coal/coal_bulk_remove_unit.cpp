#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <memory>
#include <set>
#include <string>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/collision/coal/coal_discrete_managers.h>
#include <tesseract/geometry/impl/box.h>

using namespace tesseract::collision;
using tesseract::collision::tesseract_collision_coal::CoalCastBVHManager;
using tesseract::collision::tesseract_collision_coal::CoalDiscreteBVHManager;
using tesseract::common::LinkId;

namespace
{
Eigen::Isometry3d at(double x)
{
  Eigen::Isometry3d tf{ Eigen::Isometry3d::Identity() };
  tf.translation() = Eigen::Vector3d(x, 0, 0);
  return tf;
}

/** @brief Register one unit box at @p pose through the single-object entry point */
template <typename ManagerType>
void addBox(ManagerType& checker, const LinkId& id, const Eigen::Isometry3d& pose, double edge = 1.0)
{
  CollisionShapesConst shapes{ std::make_shared<tesseract::geometry::Box>(edge, edge, edge) };
  tesseract::common::VectorIsometry3d poses{ pose };
  checker.addCollisionObject(id, 0, shapes, poses, true);
}
}  // namespace

template <typename ManagerType>
void runBulkRemoveEquivalenceTest()
{
  const std::vector<LinkId> ids{ LinkId("a"), LinkId("b"), LinkId("c"), LinkId("d") };

  ManagerType bulk;
  for (const auto& id : ids)
    addBox(bulk, id, at(0));
  EXPECT_TRUE(bulk.removeCollisionObjects({ LinkId("b"), LinkId("c") }));

  ManagerType looped;
  for (const auto& id : ids)
    addBox(looped, id, at(0));
  EXPECT_TRUE(looped.removeCollisionObject(LinkId("b")));
  EXPECT_TRUE(looped.removeCollisionObject(LinkId("c")));

  // Same survivors, in the same order: collision_objects_ order decides tree build and contact tie-break.
  EXPECT_EQ(bulk.getCollisionObjects(), looped.getCollisionObjects());
  EXPECT_FALSE(bulk.hasCollisionObject(LinkId("b")));
  EXPECT_FALSE(bulk.hasCollisionObject(LinkId("c")));
  EXPECT_TRUE(bulk.hasCollisionObject(LinkId("a")));
  EXPECT_TRUE(bulk.hasCollisionObject(LinkId("d")));
}

// An absent id reports false and does not stop the rest being removed.
template <typename ManagerType>
void runBulkRemovePartialTest()
{
  ManagerType checker;
  addBox(checker, LinkId("a"), at(0));
  addBox(checker, LinkId("b"), at(0));
  EXPECT_FALSE(checker.removeCollisionObjects({ LinkId("a"), LinkId("not_registered") }));
  EXPECT_FALSE(checker.hasCollisionObject(LinkId("a")));
  EXPECT_TRUE(checker.hasCollisionObject(LinkId("b")));
}

// A removed object must leave the broadphase, not merely the id maps. This is the assertion an override fails if
// it erases link2cow_ without unregistering, and contactTest does not refresh the broadphase, so the failure would
// otherwise be a silent wrong answer.
template <typename ManagerType>
void runBulkRemoveLeavesBroadphaseTest()
{
  ManagerType checker;
  addBox(checker, LinkId("a"), at(-0.25));
  addBox(checker, LinkId("b"), at(0.25));
  checker.setActiveCollisionObjects({ LinkId("a"), LinkId("b") });
  checker.setDefaultCollisionMargin(0.0);

  ContactResultMap before;
  checker.contactTest(before, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(before.empty());

  EXPECT_TRUE(checker.removeCollisionObjects({ LinkId("b") }));

  ContactResultMap after;
  checker.contactTest(after, ContactRequest(ContactTestType::ALL));
  EXPECT_TRUE(after.empty()) << "removed object still in the broadphase";
}

// Removing every object must leave a manager that is still usable, not one with a corrupt tree.
template <typename ManagerType>
void runBulkRemoveAllThenReuseTest()
{
  ManagerType checker;
  addBox(checker, LinkId("a"), at(0));
  addBox(checker, LinkId("b"), at(0));
  EXPECT_TRUE(checker.removeCollisionObjects({ LinkId("a"), LinkId("b") }));
  EXPECT_TRUE(checker.getCollisionObjects().empty());

  ContactResultMap empty_result;
  checker.contactTest(empty_result, ContactRequest(ContactTestType::ALL));
  EXPECT_TRUE(empty_result.empty());

  // Re-add and check the manager still works; on Coal and FCL this is the topdown build into an empty tree.
  addBox(checker, LinkId("a"), at(-0.25));
  addBox(checker, LinkId("b"), at(0.25));
  checker.setActiveCollisionObjects({ LinkId("a"), LinkId("b") });
  checker.setDefaultCollisionMargin(0.0);

  ContactResultMap reused;
  checker.contactTest(reused, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(reused.empty());
}

// A batch naming the same id twice removes it once and reports the second as absent.
template <typename ManagerType>
void runBulkRemoveRepeatedIdTest()
{
  ManagerType checker;
  addBox(checker, LinkId("a"), at(0));
  EXPECT_FALSE(checker.removeCollisionObjects({ LinkId("a"), LinkId("a") }));
  EXPECT_TRUE(checker.getCollisionObjects().empty());
}

// A batch that mixes a static-routed id with a kinematic-routed one must unregister each from the tree
// that holds it. Unregistering from the wrong tree removes a null node rather than reporting an error,
// so only the broadphase state afterwards shows it: neither removed link may appear in a contact, and
// the surviving pair, one link from each routing branch, must still report one.
template <typename ManagerType>
void runBulkRemoveMixedRoutingTest()
{
  ManagerType checker;
  addBox(checker, LinkId("kinematic_gone"), at(-0.3));
  addBox(checker, LinkId("kinematic_kept"), at(-0.1));
  addBox(checker, LinkId("static_gone"), at(0.1));
  addBox(checker, LinkId("static_kept"), at(0.3));

  // An active link is registered in the dynamic tree, every other link in the static tree.
  checker.setActiveCollisionObjects({ LinkId("kinematic_gone"), LinkId("kinematic_kept") });
  checker.setDefaultCollisionMargin(0.0);

  ContactResultMap before;
  checker.contactTest(before, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(before.empty());

  EXPECT_TRUE(checker.removeCollisionObjects({ LinkId("kinematic_gone"), LinkId("static_gone") }));

  ContactResultMap after;
  checker.contactTest(after, ContactRequest(ContactTestType::ALL));

  ContactResultVector flat;
  after.flattenCopyResults(flat);

  // Compare the whole pair set, not the presence of the survivors: a link left in a tree after its wrapper is
  // destroyed reports a contact whose link ids are read from freed memory, so an extra entry is what identifies
  // it, not the name that entry carries.
  std::set<std::string> pairs;
  for (const auto& contact : flat)
  {
    const std::string first = contact.link_ids[0].name();
    const std::string second = contact.link_ids[1].name();
    pairs.insert((first < second) ? (first + "|" + second) : (second + "|" + first));
  }

  EXPECT_EQ(pairs, std::set<std::string>{ "kinematic_kept|static_kept" });
}

TEST(CoalBulkRemoveUnit, DiscreteEquivalence)  // NOLINT
{
  runBulkRemoveEquivalenceTest<CoalDiscreteBVHManager>();
}

TEST(CoalBulkRemoveUnit, CastEquivalence)  // NOLINT
{
  runBulkRemoveEquivalenceTest<CoalCastBVHManager>();
}

TEST(CoalBulkRemoveUnit, DiscretePartial)  // NOLINT
{
  runBulkRemovePartialTest<CoalDiscreteBVHManager>();
}

TEST(CoalBulkRemoveUnit, CastPartial)  // NOLINT
{
  runBulkRemovePartialTest<CoalCastBVHManager>();
}

TEST(CoalBulkRemoveUnit, DiscreteLeavesBroadphase)  // NOLINT
{
  runBulkRemoveLeavesBroadphaseTest<CoalDiscreteBVHManager>();
}

TEST(CoalBulkRemoveUnit, CastLeavesBroadphase)  // NOLINT
{
  runBulkRemoveLeavesBroadphaseTest<CoalCastBVHManager>();
}

TEST(CoalBulkRemoveUnit, DiscreteRemoveAllThenReuse)  // NOLINT
{
  runBulkRemoveAllThenReuseTest<CoalDiscreteBVHManager>();
}

TEST(CoalBulkRemoveUnit, CastRemoveAllThenReuse)  // NOLINT
{
  runBulkRemoveAllThenReuseTest<CoalCastBVHManager>();
}

TEST(CoalBulkRemoveUnit, DiscreteRepeatedId)  // NOLINT
{
  runBulkRemoveRepeatedIdTest<CoalDiscreteBVHManager>();
}

TEST(CoalBulkRemoveUnit, CastRepeatedId)  // NOLINT
{
  runBulkRemoveRepeatedIdTest<CoalCastBVHManager>();
}

TEST(CoalBulkRemoveUnit, DiscreteMixedRouting)  // NOLINT
{
  runBulkRemoveMixedRoutingTest<CoalDiscreteBVHManager>();
}

TEST(CoalBulkRemoveUnit, CastMixedRouting)  // NOLINT
{
  runBulkRemoveMixedRoutingTest<CoalCastBVHManager>();
}

// A static link and a kinematic link are registered through different wrappers, so removal has to unregister the
// right one for each. Remove a third link and confirm the survivors still sweep.
TEST(CoalBulkRemoveUnit, CastMixedStaticAndKinematicBatch)  // NOLINT
{
  CoalCastBVHManager checker;
  addBox(checker, LinkId("moving"), at(0));
  addBox(checker, LinkId("fixed"), at(0));
  addBox(checker, LinkId("gone"), at(0));
  checker.setActiveCollisionObjects({ LinkId("moving") });
  checker.setDefaultCollisionMargin(0.0);

  EXPECT_TRUE(checker.removeCollisionObjects({ LinkId("gone") }));

  checker.setCollisionObjectsTransform(std::vector<LinkId>{ "fixed" }, tesseract::common::VectorIsometry3d{ at(0) });
  checker.setCollisionObjectsTransform(std::vector<LinkId>{ "moving" },
                                       tesseract::common::VectorIsometry3d{ at(-5) },
                                       tesseract::common::VectorIsometry3d{ at(5) });

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(result.empty());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
