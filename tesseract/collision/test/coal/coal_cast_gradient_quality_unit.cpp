#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <octomap/octomap.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/collision/common.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/common/resource_locator.h>

using namespace tesseract::collision;

/**
 * @brief Verify that cast contact results are accurate enough to drive gradient-based optimization.
 *
 * The existing collision tests check per-contact properties (distance, cc_time, normals) with loose
 * tolerances. TrajOpt relies on contact normals being accurate enough to compute valid constraint
 * gradients -- a subtly wrong normal can pass EXPECT_NEAR but produce a gradient that points the
 * wrong way, preventing the optimizer from resolving the collision.
 *
 * This test performs a single gradient step: move the kinematic link along the contact normal by a
 * fraction of the penetration depth, then re-run the collision check and verify that penetration
 * decreased. This directly tests what TrajOpt needs without depending on TrajOpt.
 *
 * The sweep end poses are chosen so the box just barely enters the octree boundary (not deep inside),
 * ensuring a clear escape direction exists. The octree is a 2m box at the origin (voxels from -1 to 1,
 * resolution 0.5m).
 */
class CoalCastGradientQualityUnit : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Add static octree (2m box at origin, res 0.5m, voxels from -1 to 1)
    tesseract::common::GeneralResourceLocator locator;
    std::string path = locator.locateResource("package://tesseract/support/meshes/box_2m.bt")->getFilePath();
    auto ot = std::make_shared<octomap::OcTree>(path);
    CollisionShapePtr dense_octomap =
        std::make_shared<tesseract::geometry::Octree>(ot, tesseract::geometry::OctreeSubType::BOX);

    CollisionShapesConst octree_shapes;
    tesseract::common::VectorIsometry3d octree_poses;
    octree_shapes.push_back(dense_octomap);
    octree_poses.push_back(Eigen::Isometry3d::Identity());
    checker_.addCollisionObject("octomap_link", 0, octree_shapes, octree_poses);

    // Add active box (small, will be swept into octree boundary)
    CollisionShapePtr box = std::make_shared<tesseract::geometry::Box>(0.2, 0.2, 0.2);
    CollisionShapesConst box_shapes;
    tesseract::common::VectorIsometry3d box_poses;
    box_shapes.push_back(box);
    box_poses.push_back(Eigen::Isometry3d::Identity());
    checker_.addCollisionObject("box_link", 0, box_shapes, box_poses);

    ASSERT_EQ(checker_.getCollisionObjects().size(), 2);

    // Only the box is active
    checker_.setActiveCollisionObjects(std::vector<std::string>{ "box_link" });
    checker_.setDefaultCollisionMargin(0.0);
  }

  /**
   * @brief Compute total penetration cost: sum of |d| for all contacts with d < 0.
   *
   * This mimics what a gradient-based optimizer like TrajOpt minimizes.
   */
  static double totalPenetrationCost(const ContactResultVector& contacts)
  {
    double cost = 0.0;
    for (const auto& cr : contacts)
    {
      if (cr.distance < 0.0)
        cost += -cr.distance;
    }
    return cost;
  }

  /**
   * @brief Compute a weighted-average gradient direction from all penetrating contacts.
   *
   * Each contact contributes a direction (the contact normal, oriented to push the kinematic
   * link out of collision) weighted by its penetration depth. This approximates the gradient
   * of the total penetration cost w.r.t. the kinematic link's position.
   */
  static Eigen::Vector3d computeAverageGradient(const ContactResultVector& contacts, const std::string& active_link)
  {
    Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
    for (const auto& cr : contacts)
    {
      if (cr.distance >= 0.0)
        continue;

      const std::size_t ki = (cr.link_ids[0].name() == active_link) ? 0 : 1;
      // Normal points from shape[0] to shape[1].
      // For ki==0: move in +normal to increase distance (push shape[0] away from shape[1])
      // For ki==1: move in -normal to increase distance (push shape[1] away from shape[0])
      Eigen::Vector3d dir = (ki == 0) ? cr.normal : -cr.normal;
      gradient += -cr.distance * dir;
    }
    return gradient;
  }

  /**
   * @brief Run the gradient-quality check for a given sweep configuration.
   *
   * @param start  Start pose for the kinematic link (outside octree).
   * @param end    End pose for the kinematic link (just inside octree boundary).
   */
  void runGradientCheck(const Eigen::Isometry3d& start, const Eigen::Isometry3d& end)
  {
    // Set static octree pose
    tesseract::common::LinkIdTransformMap location;
    location[tesseract::common::LinkId::fromName("octomap_link")] = Eigen::Isometry3d::Identity();
    checker_.setCollisionObjectsTransform(location);

    // Set sweep poses
    checker_.setCollisionObjectsTransform("box_link", start, end);

    // Get contacts
    ContactResultMap result;
    checker_.contactTest(result, ContactRequest(ContactTestType::ALL));

    ContactResultVector contacts;
    result.flattenMoveResults(contacts);
    ASSERT_FALSE(contacts.empty()) << "Expected contacts for box sweeping into octree";

    const double initial_cost = totalPenetrationCost(contacts);
    ASSERT_GT(initial_cost, 1e-6) << "Expected non-zero penetration cost";

    // Compute gradient direction from all contacts
    Eigen::Vector3d gradient = computeAverageGradient(contacts, "box_link");
    ASSERT_GT(gradient.norm(), 1e-10) << "Gradient should be non-zero";

    // Normalize and scale: step = alpha * gradient_direction
    // Use a step size proportional to the average penetration depth
    const double avg_penetration = initial_cost / static_cast<double>(contacts.size());
    const double alpha = 0.5;
    Eigen::Vector3d step = alpha * avg_penetration * gradient.normalized();

    // Apply displacement to both start and end poses
    Eigen::Isometry3d new_start = start;
    new_start.translation() += step;
    Eigen::Isometry3d new_end = end;
    new_end.translation() += step;

    // Re-run collision with displaced poses
    checker_.setCollisionObjectsTransform("box_link", new_start, new_end);
    ContactResultMap result2;
    checker_.contactTest(result2, ContactRequest(ContactTestType::ALL));

    ContactResultVector contacts2;
    result2.flattenMoveResults(contacts2);

    const double new_cost = totalPenetrationCost(contacts2);
    EXPECT_LT(new_cost, initial_cost) << "Total penetration cost should decrease after gradient step.\n"
                                      << "Initial cost: " << initial_cost << " (" << contacts.size() << " contacts)\n"
                                      << "After step cost: " << new_cost << " (" << contacts2.size() << " contacts)\n"
                                      << "Step: (" << step.x() << ", " << step.y() << ", " << step.z() << ")\n"
                                      << "Gradient: (" << gradient.x() << ", " << gradient.y() << ", " << gradient.z()
                                      << ")\n"
                                      << "If this fails, the contact normals are not accurate enough for "
                                         "gradient-based optimization.";
  }

  tesseract_collision_coal::CoalCastBVHManager checker_;
};

// Sweep along -X: box starts outside the +X face, ends just inside.
// Box at x=0.85 with size 0.2 overlaps the outermost voxel layer.
TEST_F(CoalCastGradientQualityUnit, OctreeBoxSweepAlongX)
{
  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  start.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
  Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
  // Box at x=0.85 barely enters the octree from the +X side
  end.translation() = Eigen::Vector3d(0.85, 0.0, 0.0);

  runGradientCheck(start, end);
}

// Sweep along -Z: box enters from the +Z face
TEST_F(CoalCastGradientQualityUnit, OctreeBoxSweepAlongZ)
{
  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  start.translation() = Eigen::Vector3d(0.0, 0.0, 2.0);
  Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
  end.translation() = Eigen::Vector3d(0.0, 0.0, 0.85);

  runGradientCheck(start, end);
}

// Sweep along -Y: box enters from the +Y face
TEST_F(CoalCastGradientQualityUnit, OctreeBoxSweepAlongY)
{
  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  start.translation() = Eigen::Vector3d(0.0, 2.0, 0.0);
  Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
  end.translation() = Eigen::Vector3d(0.0, 0.85, 0.0);

  runGradientCheck(start, end);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
