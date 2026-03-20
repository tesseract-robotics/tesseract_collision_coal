#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_utils.h>
#include <tesseract/collision/coal/coal_discrete_managers.h>
#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/geometry/geometries.h>

using namespace tesseract::collision;
using namespace tesseract::collision::tesseract_collision_coal;

/**
 * @brief Verify that the collision cache produces consistent results across
 * repeated contactTest calls.
 *
 * The first call populates the cache; the second reuses it.  This validates
 * that cached GJK guesses and collision functors produce identical results
 * when reused.
 */
TEST(CoalCachePairUnit, RepeatedContactTestReusesCache)  // NOLINT
{
  // Create two overlapping sphere collision objects via the manager API,
  // then pull out the raw Coal objects to call the callback directly.
  tesseract_collision_coal::CoalDiscreteBVHManager checker;

  auto sphere = std::make_shared<tesseract::geometry::Sphere>(0.5);
  CollisionShapesConst shapes1 = { sphere };
  CollisionShapesConst shapes2 = { sphere };
  tesseract::common::VectorIsometry3d poses1 = { Eigen::Isometry3d::Identity() };
  tesseract::common::VectorIsometry3d poses2 = { Eigen::Isometry3d::Identity() };

  checker.addCollisionObject("link_a", 0, shapes1, poses1, true);
  checker.addCollisionObject("link_b", 0, shapes2, poses2, true);
  checker.setActiveCollisionObjects({ "link_a", "link_b" });
  checker.setDefaultCollisionMargin(0.0);

  // Place spheres so they overlap
  Eigen::Isometry3d pose_a = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose_b = Eigen::Isometry3d::Identity();
  pose_b.translation() = Eigen::Vector3d(0.5, 0, 0);

  checker.setCollisionObjectsTransform("link_a", pose_a);
  checker.setCollisionObjectsTransform("link_b", pose_b);

  // Run contactTest twice.  The first call populates the cache; the second
  // reuses it.  Because the broadphase order is deterministic for two calls
  // with the same tree state, this baseline run just verifies correctness.
  ContactResultMap result1;
  ContactRequest request1(ContactTestType::ALL);
  request1.calculate_penetration = true;
  checker.contactTest(result1, request1);
  ASSERT_FALSE(result1.empty());

  // Flatten to get contact details for comparison
  ContactResultVector result1_vec;
  result1.flattenCopyResults(result1_vec);
  ASSERT_EQ(result1_vec.size(), 1);

  const auto& c1 = result1_vec[0];

  // Identify which slot corresponds to link_a vs link_b
  int a_idx = (c1.link_names[0] == "link_a") ? 0 : 1;
  int b_idx = 1 - a_idx;

  EXPECT_EQ(c1.link_names[a_idx], "link_a");
  EXPECT_EQ(c1.link_names[b_idx], "link_b");

  // Nearest point on link_a's sphere should be on the +x side (toward link_b)
  EXPECT_GT(c1.nearest_points[a_idx].x(), -0.01);
  // Nearest point on link_b's sphere should be on the -x side (toward link_a)
  EXPECT_LT(c1.nearest_points[b_idx].x(), 0.51);

  // The normal should point from one to the other along x (either direction
  // depending on convention, but the magnitude along x should dominate).
  EXPECT_GT(std::abs(c1.normal.x()), 0.9);

  // Second run to verify results are identical (cache reuse)
  ContactResultMap result2;
  checker.contactTest(result2, request1);
  ContactResultVector result2_vec;
  result2.flattenCopyResults(result2_vec);
  ASSERT_EQ(result2_vec.size(), 1);

  EXPECT_NEAR(result2_vec[0].distance, c1.distance, 1e-12);
  EXPECT_TRUE(result2_vec[0].nearest_points[0].isApprox(c1.nearest_points[0], 1e-12));
  EXPECT_TRUE(result2_vec[0].nearest_points[1].isApprox(c1.nearest_points[1], 1e-12));
}

/**
 * @brief Verify results are consistent when objects are re-registered in
 * reversed order, which can change broadphase pointer ordering.
 */
TEST(CoalCachePairUnit, ReregisteredObjectsGiveConsistentResults)  // NOLINT
{
  auto sphere = std::make_shared<tesseract::geometry::Sphere>(0.5);
  CollisionShapesConst shapes = { sphere };
  tesseract::common::VectorIsometry3d poses = { Eigen::Isometry3d::Identity() };

  Eigen::Isometry3d pose_a = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose_b = Eigen::Isometry3d::Identity();
  pose_b.translation() = Eigen::Vector3d(0.5, 0, 0);

  ContactRequest request(ContactTestType::ALL);
  request.calculate_penetration = true;

  // First manager: add link_a then link_b
  tesseract_collision_coal::CoalDiscreteBVHManager checker1;
  checker1.addCollisionObject("link_a", 0, shapes, poses, true);
  checker1.addCollisionObject("link_b", 0, shapes, poses, true);
  checker1.setActiveCollisionObjects({ "link_a", "link_b" });
  checker1.setDefaultCollisionMargin(0.0);
  checker1.setCollisionObjectsTransform("link_a", pose_a);
  checker1.setCollisionObjectsTransform("link_b", pose_b);

  ContactResultMap result1;
  checker1.contactTest(result1, request);
  ContactResultVector result1_vec;
  result1.flattenCopyResults(result1_vec);
  ASSERT_EQ(result1_vec.size(), 1);

  // Second manager: add link_b then link_a (reversed registration order)
  tesseract_collision_coal::CoalDiscreteBVHManager checker2;
  checker2.addCollisionObject("link_b", 0, shapes, poses, true);
  checker2.addCollisionObject("link_a", 0, shapes, poses, true);
  checker2.setActiveCollisionObjects({ "link_a", "link_b" });
  checker2.setDefaultCollisionMargin(0.0);
  checker2.setCollisionObjectsTransform("link_a", pose_a);
  checker2.setCollisionObjectsTransform("link_b", pose_b);

  ContactResultMap result2;
  checker2.contactTest(result2, request);
  ContactResultVector result2_vec;
  result2.flattenCopyResults(result2_vec);
  ASSERT_EQ(result2_vec.size(), 1);

  // Both results should use makeOrderedLinkPair for the key, so
  // the link_names ordering in the result may differ, but the
  // physical contact data should be identical.

  // Find the contact for link_a in each result
  auto get_a_nearest = [](const ContactResult& c) {
    return (c.link_names[0] == "link_a") ? c.nearest_points[0] : c.nearest_points[1];
  };
  auto get_b_nearest = [](const ContactResult& c) {
    return (c.link_names[0] == "link_b") ? c.nearest_points[0] : c.nearest_points[1];
  };

  EXPECT_NEAR(result1_vec[0].distance, result2_vec[0].distance, 1e-12);
  EXPECT_TRUE(get_a_nearest(result1_vec[0]).isApprox(get_a_nearest(result2_vec[0]), 1e-12));
  EXPECT_TRUE(get_b_nearest(result1_vec[0]).isApprox(get_b_nearest(result2_vec[0]), 1e-12));
}

/**
 * @brief Verify that re-running contactTest after removing and re-adding one
 * object (which clears the cache for that pair and may change broadphase
 * traversal order) still produces correct results.
 */
TEST(CoalCachePairUnit, RemoveReaddProducesCorrectResults)  // NOLINT
{
  auto sphere = std::make_shared<tesseract::geometry::Sphere>(0.5);
  CollisionShapesConst shapes = { sphere };
  tesseract::common::VectorIsometry3d poses = { Eigen::Isometry3d::Identity() };

  tesseract_collision_coal::CoalDiscreteBVHManager checker;
  checker.addCollisionObject("link_a", 0, shapes, poses, true);
  checker.addCollisionObject("link_b", 0, shapes, poses, true);
  checker.setActiveCollisionObjects({ "link_a", "link_b" });
  checker.setDefaultCollisionMargin(0.0);

  Eigen::Isometry3d pose_a = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose_b = Eigen::Isometry3d::Identity();
  pose_b.translation() = Eigen::Vector3d(0.5, 0, 0);

  checker.setCollisionObjectsTransform("link_a", pose_a);
  checker.setCollisionObjectsTransform("link_b", pose_b);

  ContactRequest request(ContactTestType::ALL);
  request.calculate_penetration = true;

  // Baseline
  ContactResultMap result1;
  checker.contactTest(result1, request);
  ContactResultVector result1_vec;
  result1.flattenCopyResults(result1_vec);
  ASSERT_EQ(result1_vec.size(), 1);

  // Remove and re-add link_b — this clears cache entries involving link_b
  // and may change the internal Coal object pointer, altering broadphase order.
  checker.removeCollisionObject("link_b");
  checker.addCollisionObject("link_b", 0, shapes, poses, true);
  checker.setActiveCollisionObjects({ "link_a", "link_b" });
  checker.setCollisionObjectsTransform("link_b", pose_b);

  ContactResultMap result2;
  checker.contactTest(result2, request);
  ContactResultVector result2_vec;
  result2.flattenCopyResults(result2_vec);
  ASSERT_EQ(result2_vec.size(), 1);

  EXPECT_NEAR(result1_vec[0].distance, result2_vec[0].distance, 1e-12);
}

/**
 * @brief Verify that objects default to StaticFilter and produce no contacts
 * without an explicit setActiveCollisionObjects call.
 *
 * COWs default to m_collisionFilterGroup = StaticFilter. This means newly added
 * objects are registered in the static broadphase, and only static-vs-kinematic
 * pairs are checked during contactTest. With no kinematic objects (no active list
 * set), there are no valid pairs and contactTest must return zero contacts.
 *
 * This test exists because the default was changed from KinematicFilter to
 * StaticFilter to avoid unnecessary broadphase churn: with the old default, all
 * objects started in the dynamic manager and were immediately moved to static
 * when setActiveCollisionObjects was called — wasting register/unregister cycles
 * for the majority of objects (environment geometry) that are never active.
 *
 * Tested for both discrete and cast managers.
 */
TEST(CoalCachePairUnit, DefaultStaticFilterProducesNoContactsWithoutActiveList)  // NOLINT
{
  auto sphere = std::make_shared<tesseract::geometry::Sphere>(0.5);
  CollisionShapesConst shapes = { sphere };
  tesseract::common::VectorIsometry3d poses = { Eigen::Isometry3d::Identity() };

  // --- Discrete manager ---
  {
    tesseract_collision_coal::CoalDiscreteBVHManager checker;
    checker.addCollisionObject("link_a", 0, shapes, poses, true);
    checker.addCollisionObject("link_b", 0, shapes, poses, true);
    // Deliberately do NOT call setActiveCollisionObjects.

    // Place spheres so they fully overlap at origin.
    checker.setCollisionObjectsTransform("link_a", Eigen::Isometry3d::Identity());
    checker.setCollisionObjectsTransform("link_b", Eigen::Isometry3d::Identity());

    ContactResultMap result;
    checker.contactTest(result, ContactRequest(ContactTestType::ALL));

    EXPECT_TRUE(result.empty())
        << "With no active objects set, all objects are static. "
        << "Static-vs-static pairs are not checked, so no contacts should be reported.";
  }

  // --- Cast (continuous) manager ---
  {
    tesseract_collision_coal::CoalCastBVHManager checker;
    checker.addCollisionObject("link_a", 0, shapes, poses, true);
    checker.addCollisionObject("link_b", 0, shapes, poses, true);
    // Deliberately do NOT call setActiveCollisionObjects.

    checker.setCollisionObjectsTransform("link_a", Eigen::Isometry3d::Identity());
    checker.setCollisionObjectsTransform("link_b", Eigen::Isometry3d::Identity());

    ContactResultMap result;
    checker.contactTest(result, ContactRequest(ContactTestType::ALL));

    EXPECT_TRUE(result.empty())
        << "With no active objects set, all objects are static. "
        << "Static-vs-static pairs are not checked, so no contacts should be reported.";
  }
}

/**
 * @brief Verify that setting active objects after add produces the expected contacts.
 *
 * Complements DefaultStaticFilterProducesNoContactsWithoutActiveList: after calling
 * setActiveCollisionObjects, the promoted objects move from static to kinematic and
 * contacts are found normally. This confirms the StaticFilter default doesn't prevent
 * collision detection when the active list is properly configured.
 */
TEST(CoalCachePairUnit, ActiveListPromotionEnablesContacts)  // NOLINT
{
  auto sphere = std::make_shared<tesseract::geometry::Sphere>(0.5);
  CollisionShapesConst shapes = { sphere };
  tesseract::common::VectorIsometry3d poses = { Eigen::Isometry3d::Identity() };

  // --- Discrete manager ---
  {
    tesseract_collision_coal::CoalDiscreteBVHManager checker;
    checker.addCollisionObject("link_a", 0, shapes, poses, true);
    checker.addCollisionObject("link_b", 0, shapes, poses, true);

    // No contacts before setting active list
    checker.setCollisionObjectsTransform("link_a", Eigen::Isometry3d::Identity());
    checker.setCollisionObjectsTransform("link_b", Eigen::Isometry3d::Identity());

    ContactResultMap result_before;
    checker.contactTest(result_before, ContactRequest(ContactTestType::ALL));
    EXPECT_TRUE(result_before.empty());

    // Now set one object active — static-vs-kinematic pair should produce contact
    checker.setActiveCollisionObjects({ "link_a" });

    ContactResultMap result_after;
    checker.contactTest(result_after, ContactRequest(ContactTestType::ALL));
    EXPECT_FALSE(result_after.empty())
        << "After setting link_a active, link_a (kinematic) vs link_b (static) "
        << "should produce a contact for two overlapping spheres.";
  }

  // --- Cast (continuous) manager ---
  {
    tesseract_collision_coal::CoalCastBVHManager checker;
    checker.addCollisionObject("link_a", 0, shapes, poses, true);
    checker.addCollisionObject("link_b", 0, shapes, poses, true);

    checker.setCollisionObjectsTransform("link_a", Eigen::Isometry3d::Identity());
    checker.setCollisionObjectsTransform("link_b", Eigen::Isometry3d::Identity());

    ContactResultMap result_before;
    checker.contactTest(result_before, ContactRequest(ContactTestType::ALL));
    EXPECT_TRUE(result_before.empty());

    // Set link_a active and give it a sweep transform
    checker.setActiveCollisionObjects({ "link_a" });
    Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
    end.translation() = Eigen::Vector3d(0.1, 0, 0);
    checker.setCollisionObjectsTransform("link_a", start, end);

    ContactResultMap result_after;
    checker.contactTest(result_after, ContactRequest(ContactTestType::ALL));
    EXPECT_FALSE(result_after.empty())
        << "After setting link_a active with a sweep, link_a (kinematic) vs "
        << "link_b (static) should produce a contact.";
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
