#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <memory>
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

/** @brief A unit-box spec centred at the origin; the caller positions it with setCollisionObjectsTransform */
CollisionObjectSpec boxSpec(const LinkId& id, double edge = 1.0)
{
  CollisionObjectSpec spec;
  spec.id = id;
  spec.shapes = CollisionShapesConst{ std::make_shared<tesseract::geometry::Box>(edge, edge, edge) };
  spec.shape_poses = tesseract::common::VectorIsometry3d{ Eigen::Isometry3d::Identity() };
  return spec;
}

std::size_t countEntries(const std::vector<LinkId>& ids, const LinkId& id)
{
  return static_cast<std::size_t>(std::count(ids.begin(), ids.end(), id));
}
}  // namespace

template <typename ManagerType>
void runBulkAddEquivalenceTest()
{
  const std::vector<CollisionObjectSpec> specs{ boxSpec(LinkId("box_a")),
                                                boxSpec(LinkId("box_b")),
                                                boxSpec(LinkId("box_c")) };

  ManagerType bulk;
  EXPECT_TRUE(bulk.addCollisionObjects(specs));

  ManagerType looped;
  for (const auto& spec : specs)
    EXPECT_TRUE(looped.addCollisionObject(spec.id, spec.mask_id, spec.shapes, spec.shape_poses, spec.enabled));

  EXPECT_EQ(bulk.getCollisionObjects(), looped.getCollisionObjects());  // same ids, same order

  for (auto* checker : { &bulk, &looped })
  {
    checker->setActiveCollisionObjects({ LinkId("box_a"), LinkId("box_b"), LinkId("box_c") });
    checker->setDefaultCollisionMargin(0.0);
    checker->setCollisionObjectsTransform(std::vector<LinkId>{ "box_a", "box_b", "box_c" },
                                          tesseract::common::VectorIsometry3d{ at(-0.25), at(0.25), at(10) });
  }

  ContactResultMap bulk_result;
  bulk.contactTest(bulk_result, ContactRequest(ContactTestType::ALL));
  ContactResultMap looped_result;
  looped.contactTest(looped_result, ContactRequest(ContactTestType::ALL));
  EXPECT_EQ(bulk_result.count(), looped_result.count());
  EXPECT_FALSE(bulk_result.empty());
}

TEST(CoalBulkAddUnit, DiscreteEquivalence)  // NOLINT
{
  runBulkAddEquivalenceTest<CoalDiscreteBVHManager>();
}

TEST(CoalBulkAddUnit, CastEquivalence)  // NOLINT
{
  runBulkAddEquivalenceTest<CoalCastBVHManager>();
}

template <typename ManagerType>
void runBulkAddDuplicateIdTest()
{
  ManagerType checker;
  EXPECT_TRUE(checker.addCollisionObjects({ boxSpec(LinkId("box_a"), 1.0), boxSpec(LinkId("box_b"), 1.0) }));

  // Re-register box_a with a box small enough that it no longer reaches box_b.
  EXPECT_TRUE(checker.addCollisionObjects({ boxSpec(LinkId("box_a"), 0.1) }));

  // Half one: no second entry for the id.
  EXPECT_EQ(countEntries(checker.getCollisionObjects(), LinkId("box_a")), 1U);

  // Half two: the broadphase sees the new geometry, not the old one still sitting in the tree.
  checker.setActiveCollisionObjects({ LinkId("box_a"), LinkId("box_b") });
  checker.setDefaultCollisionMargin(0.0);
  checker.setCollisionObjectsTransform(std::vector<LinkId>{ "box_a", "box_b" },
                                       tesseract::common::VectorIsometry3d{ at(-0.4), at(0.4) });

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_TRUE(result.empty()) << "the displaced 1.0 box is still in the broadphase";
}

TEST(CoalBulkAddUnit, DiscreteDuplicateIdReplaces)  // NOLINT
{
  runBulkAddDuplicateIdTest<CoalDiscreteBVHManager>();
}

TEST(CoalBulkAddUnit, CastDuplicateIdReplaces)  // NOLINT
{
  runBulkAddDuplicateIdTest<CoalCastBVHManager>();
}

// A batch naming the same id twice is not collapsed by a vector the way the old Link2COW map collapsed it, so the
// override owes an in-batch dedup pass. Last spec wins, matching the replaced per-object loop.
template <typename ManagerType>
void runBulkAddRepeatedIdWithinBatchTest()
{
  ManagerType checker;
  EXPECT_TRUE(checker.addCollisionObjects({ boxSpec(LinkId("box_a"), 1.0), boxSpec(LinkId("box_a"), 0.1) }));
  EXPECT_EQ(countEntries(checker.getCollisionObjects(), LinkId("box_a")), 1U);
}

TEST(CoalBulkAddUnit, DiscreteRepeatedIdWithinBatch)  // NOLINT
{
  runBulkAddRepeatedIdWithinBatchTest<CoalDiscreteBVHManager>();
}

TEST(CoalBulkAddUnit, CastRepeatedIdWithinBatch)  // NOLINT
{
  runBulkAddRepeatedIdWithinBatchTest<CoalCastBVHManager>();
}

template <typename ManagerType>
void runBulkAddPartialFailureTest()
{
  CollisionObjectSpec bad;
  bad.id = LinkId("no_geometry");  // empty shapes and shape_poses

  ManagerType checker;
  EXPECT_FALSE(checker.addCollisionObjects({ boxSpec(LinkId("box_a")), bad, boxSpec(LinkId("box_b")) }));

  EXPECT_TRUE(checker.hasCollisionObject(LinkId("box_a")));
  EXPECT_TRUE(checker.hasCollisionObject(LinkId("box_b")));
  EXPECT_FALSE(checker.hasCollisionObject(LinkId("no_geometry")));
}

TEST(CoalBulkAddUnit, DiscretePartialFailure)  // NOLINT
{
  runBulkAddPartialFailureTest<CoalDiscreteBVHManager>();
}

TEST(CoalBulkAddUnit, CastPartialFailure)  // NOLINT
{
  runBulkAddPartialFailureTest<CoalCastBVHManager>();
}

template <typename ManagerType>
void runBulkAddBroadphaseLiveTest()
{
  ManagerType checker;
  // Position through the spec's own shape_poses, so nothing after the bulk add can refresh the broadphase.
  CollisionObjectSpec a = boxSpec(LinkId("box_a"));
  a.shape_poses = tesseract::common::VectorIsometry3d{ at(-0.25) };
  CollisionObjectSpec b = boxSpec(LinkId("box_b"));
  b.shape_poses = tesseract::common::VectorIsometry3d{ at(0.25) };

  checker.setActiveCollisionObjects({ LinkId("box_a"), LinkId("box_b") });
  checker.setDefaultCollisionMargin(0.0);
  EXPECT_TRUE(checker.addCollisionObjects({ a, b }));

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(result.empty()) << "bulk add left the broadphase unrefreshed";
}

TEST(CoalBulkAddUnit, DiscreteBroadphaseLiveAfterBulkAdd)  // NOLINT
{
  runBulkAddBroadphaseLiveTest<CoalDiscreteBVHManager>();
}

TEST(CoalBulkAddUnit, CastBroadphaseLiveAfterBulkAdd)  // NOLINT
{
  runBulkAddBroadphaseLiveTest<CoalCastBVHManager>();
}

template <typename ManagerType>
void runBulkAddStaticDynamicSplitTest()
{
  ManagerType checker;
  checker.setActiveCollisionObjects({ LinkId("moving") });  // "fixed" is static
  checker.setDefaultCollisionMargin(0.0);
  EXPECT_TRUE(checker.addCollisionObjects({ boxSpec(LinkId("moving")), boxSpec(LinkId("fixed")) }));

  checker.setCollisionObjectsTransform(std::vector<LinkId>{ "moving", "fixed" },
                                       tesseract::common::VectorIsometry3d{ at(-0.25), at(0.25) });

  // An active-vs-static pair collides; a static-vs-static pair does not.
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  EXPECT_FALSE(result.empty());

  // An empty active set means every link is active, so demote both links by activating an unrelated one.
  checker.setActiveCollisionObjects({ LinkId("unrelated") });
  ContactResultMap none_active;
  checker.contactTest(none_active, ContactRequest(ContactTestType::ALL));
  EXPECT_TRUE(none_active.empty());
}

TEST(CoalBulkAddUnit, DiscreteStaticDynamicSplit)  // NOLINT
{
  runBulkAddStaticDynamicSplitTest<CoalDiscreteBVHManager>();
}

TEST(CoalBulkAddUnit, CastStaticDynamicSplit)  // NOLINT
{
  runBulkAddStaticDynamicSplitTest<CoalCastBVHManager>();
}

TEST(CoalBulkAddUnit, CastBulkAddLeavesStaticCastWrapperDeferred)  // NOLINT
{
  CoalCastBVHManager checker;
  checker.setActiveCollisionObjects({ LinkId("moving") });
  checker.setDefaultCollisionMargin(0.0);
  EXPECT_TRUE(checker.addCollisionObjects({ boxSpec(LinkId("moving")), boxSpec(LinkId("fixed")) }));

  // The swept path must work immediately: the moving box sweeps across the static one at the origin.
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
