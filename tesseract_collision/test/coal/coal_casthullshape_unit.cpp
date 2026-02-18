#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/coal/coal_casthullshape.h>
#include <tesseract_collision/coal/coal_casthullshape_utility.h>

using namespace tesseract_collision;

TEST(CoalCastHullShapeUnit, LocalAABBUnit)
{
  using namespace tesseract_collision::tesseract_collision_coal;

  // Test with a Box shape - simpler to verify expected AABB
  auto box = std::make_shared<coal::Box>(1.0, 2.0, 3.0);  // width, height, depth

  // Test case 1: Identity transform (no casting)
  {
    coal::Transform3s identity = coal::Transform3s::Identity();
    CastHullShape cast_hull(box, identity);

    // Compute the local AABB
    cast_hull.computeLocalAABB();

    // For a box with identity cast transform, the AABB should be the same as the original box
    // Box dimensions are: x=[-0.5,0.5], y=[-1.0,1.0], z=[-1.5,1.5]
    coal::Vec3s expected_min(-0.5, -1.0, -1.5);
    coal::Vec3s expected_max(0.5, 1.0, 1.5);
    coal::Vec3s expected_center(0.0, 0.0, 0.0);

    // Check AABB bounds with tolerance
    const double tolerance = 1e-6;
    EXPECT_NEAR(cast_hull.aabb_local.min_[0], expected_min[0], tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.min_[1], expected_min[1], tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.min_[2], expected_min[2], tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.max_[0], expected_max[0], tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.max_[1], expected_max[1], tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.max_[2], expected_max[2], tolerance);

    // Check center
    coal::Vec3s computed_center = cast_hull.aabb_center;
    EXPECT_NEAR(computed_center[0], expected_center[0], tolerance);
    EXPECT_NEAR(computed_center[1], expected_center[1], tolerance);
    EXPECT_NEAR(computed_center[2], expected_center[2], tolerance);

    // Check radius (distance from center to corner)
    double expected_radius = (expected_max - expected_center).norm();
    EXPECT_NEAR(cast_hull.aabb_radius, expected_radius, tolerance);
  }

  // Test case 2: Translation transform
  {
    coal::Transform3s translation;
    translation.setIdentity();
    translation.translation() = coal::Vec3s(1.0, 0.0, 0.0);  // Translate 1 unit in x

    CastHullShape cast_hull(box, translation);
    cast_hull.computeLocalAABB();

    // The swept hull should now extend from the original box position to the translated position
    // Original box: x=[-0.5,0.5], translated box: x=[0.5,1.5]
    // Combined AABB should be: x=[-0.5,1.5], y=[-1.0,1.0], z=[-1.5,1.5]
    coal::Vec3s expected_min_swept(-0.5, -1.0, -1.5);
    coal::Vec3s expected_max_swept(1.5, 1.0, 1.5);

    const double tolerance = 1e-6;
    EXPECT_NEAR(cast_hull.aabb_local.min_[0], expected_min_swept[0], tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.min_[1], expected_min_swept[1], tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.min_[2], expected_min_swept[2], tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.max_[0], expected_max_swept[0], tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.max_[1], expected_max_swept[1], tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.max_[2], expected_max_swept[2], tolerance);

    // Verify that the AABB is larger than the original box due to sweeping
    EXPECT_GT(cast_hull.aabb_local.max_[0] - cast_hull.aabb_local.min_[0], 1.0);  // x dimension should be > 1.0
  }

  // Test case 3: Sphere shape with translation
  {
    auto sphere = std::make_shared<coal::Sphere>(0.5);  // radius = 0.5
    coal::Transform3s translation;
    translation.setIdentity();
    translation.translation() = coal::Vec3s(0.0, 1.0, 0.0);  // Translate 1 unit in y

    CastHullShape cast_hull_sphere(sphere, translation);
    cast_hull_sphere.computeLocalAABB();

    // Original sphere: all axes=[-0.5,0.5], translated sphere: y=[0.5,1.5], x&z=[-0.5,0.5]
    // Combined swept AABB: x=[-0.5,0.5], y=[-0.5,1.5], z=[-0.5,0.5]
    const double tolerance = 1e-6;
    EXPECT_NEAR(cast_hull_sphere.aabb_local.min_[0], -0.5, tolerance);
    EXPECT_NEAR(cast_hull_sphere.aabb_local.max_[0], 0.5, tolerance);
    EXPECT_NEAR(cast_hull_sphere.aabb_local.min_[1], -0.5, tolerance);
    EXPECT_NEAR(cast_hull_sphere.aabb_local.max_[1], 1.5, tolerance);
    EXPECT_NEAR(cast_hull_sphere.aabb_local.min_[2], -0.5, tolerance);
    EXPECT_NEAR(cast_hull_sphere.aabb_local.max_[2], 0.5, tolerance);

    // Verify AABB center and radius are computed correctly
    coal::Vec3s expected_center = cast_hull_sphere.aabb_local.center();
    EXPECT_NEAR(cast_hull_sphere.aabb_center[0], expected_center[0], tolerance);
    EXPECT_NEAR(cast_hull_sphere.aabb_center[1], expected_center[1], tolerance);
    EXPECT_NEAR(cast_hull_sphere.aabb_center[2], expected_center[2], tolerance);

    double expected_radius = (cast_hull_sphere.aabb_local.min_ - expected_center).norm();
    EXPECT_NEAR(cast_hull_sphere.aabb_radius, expected_radius, tolerance);
  }

  // Test case 4: Translation and rotation combined
  {
    auto box_small = std::make_shared<coal::Box>(1.0, 1.0, 1.0);  // unit cube for easier calculation
    coal::Transform3s transform;
    transform.setIdentity();

    // Apply a 45-degree rotation around z-axis and translation
    double angle = M_PI / 4.0;  // 45 degrees
    coal::Matrix3s rotation;
    rotation << cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0, 0, 0, 1;
    transform.rotation() = rotation;
    transform.translation() = coal::Vec3s(1.0, 1.0, 0.0);  // Translate in x and y

    CastHullShape cast_hull_rotated(box_small, transform);
    cast_hull_rotated.computeLocalAABB();

    const double tolerance = 1e-6;

    // The original unit cube has bounds [-0.5, 0.5] in all axes
    // After 45-degree rotation around z, the projected bounds in x and y will be larger
    // The rotated cube will have corners at approximately ±0.707 in x and y
    // Combined with translation, we expect the swept volume to encompass both positions

    // Original cube bounds: x=[-0.5, 0.5], y=[-0.5, 0.5], z=[-0.5, 0.5]
    // Rotated cube center at (1, 1, 0) with rotated bounds
    // The AABB should encompass both the original and transformed positions

    // Verify that the AABB is larger due to the sweep operation
    double x_extent = cast_hull_rotated.aabb_local.max_[0] - cast_hull_rotated.aabb_local.min_[0];
    double y_extent = cast_hull_rotated.aabb_local.max_[1] - cast_hull_rotated.aabb_local.min_[1];
    double z_extent = cast_hull_rotated.aabb_local.max_[2] - cast_hull_rotated.aabb_local.min_[2];

    // The x and y extents should be larger than the original cube (1.0) due to sweep
    EXPECT_GT(x_extent, 1.0);
    EXPECT_GT(y_extent, 1.0);
    // Z extent should remain approximately 1.0 since no rotation/translation in z
    EXPECT_NEAR(z_extent, 1.0, 0.1);  // Allow some tolerance for rotation effects

    // Verify that both original and transformed positions are contained in the AABB
    // Original cube corners should be within bounds
    EXPECT_LE(cast_hull_rotated.aabb_local.min_[0], -0.5);
    EXPECT_GE(cast_hull_rotated.aabb_local.max_[0], 0.5);
    EXPECT_LE(cast_hull_rotated.aabb_local.min_[1], -0.5);
    EXPECT_GE(cast_hull_rotated.aabb_local.max_[1], 0.5);

    // Transformed position should also be within bounds
    // After rotation and translation, corners will be around (1±0.707, 1±0.707, ±0.5)
    double sqrt2_half = sqrt(2.0) / 2.0;  // ≈ 0.707
    EXPECT_LE(cast_hull_rotated.aabb_local.min_[0], 1.0 - sqrt2_half - tolerance);
    EXPECT_GE(cast_hull_rotated.aabb_local.max_[0], 1.0 + sqrt2_half - tolerance);
    EXPECT_LE(cast_hull_rotated.aabb_local.min_[1], 1.0 - sqrt2_half - tolerance);
    EXPECT_GE(cast_hull_rotated.aabb_local.max_[1], 1.0 + sqrt2_half - tolerance);

    // Verify AABB center and radius are computed correctly
    coal::Vec3s computed_center = cast_hull_rotated.aabb_center;
    coal::Vec3s expected_center_from_aabb = cast_hull_rotated.aabb_local.center();
    EXPECT_NEAR(computed_center[0], expected_center_from_aabb[0], tolerance);
    EXPECT_NEAR(computed_center[1], expected_center_from_aabb[1], tolerance);
    EXPECT_NEAR(computed_center[2], expected_center_from_aabb[2], tolerance);

    double expected_radius = (cast_hull_rotated.aabb_local.min_ - computed_center).norm();
    EXPECT_NEAR(cast_hull_rotated.aabb_radius, expected_radius, tolerance);
  }
}

TEST(CoalCastHullShapeUnit, ComputeVolumeUnit)
{
  using namespace tesseract_collision::tesseract_collision_coal;
  const double tolerance = 1e-6;

  // Test 1: Box with identity transform (no sweeping)
  {
    auto box = std::make_shared<coal::Box>(2.0, 2.0, 2.0);  // Volume = 2*2*2 = 8.0
    coal::Transform3s identity = coal::Transform3s::Identity();
    CastHullShape cast_hull_identity(box, identity);

    double box_volume = box->computeVolume();
    double cast_hull_volume = cast_hull_identity.computeVolume();

    // For identity transform, volumes should be equal
    EXPECT_NEAR(cast_hull_volume, box_volume, tolerance);
    EXPECT_NEAR(cast_hull_volume, 8.0, tolerance);
  }

  // Test 2: Box with translation - volume should increase
  {
    auto box = std::make_shared<coal::Box>(2.0, 2.0, 2.0);  // Volume = 8.0
    coal::Transform3s translation;
    translation.setIdentity();
    translation.translation() = coal::Vec3s(1.0, 0.0, 0.0);
    CastHullShape cast_hull_translated(box, translation);

    double box_volume = box->computeVolume();
    double cast_hull_translated_volume = cast_hull_translated.computeVolume();

    // Swept volume should be larger than original
    EXPECT_GT(cast_hull_translated_volume, box_volume);
    EXPECT_GT(cast_hull_translated_volume, 8.0);

    // Should be reasonable (not infinite or negative)
    EXPECT_LT(cast_hull_translated_volume, 100.0);  // Reasonable upper bound
    EXPECT_GT(cast_hull_translated_volume, 0.0);
  }

  // Test 3: Sphere with identity transform
  {
    auto sphere = std::make_shared<coal::Sphere>(1.0);  // radius 1, Volume = (4/3)π ≈ 4.189
    coal::Transform3s identity = coal::Transform3s::Identity();
    CastHullShape cast_hull_sphere(sphere, identity);

    double sphere_volume = sphere->computeVolume();
    double cast_hull_sphere_volume = cast_hull_sphere.computeVolume();

    // For identity transform, volumes should be equal
    EXPECT_NEAR(cast_hull_sphere_volume, sphere_volume, tolerance);
    EXPECT_NEAR(cast_hull_sphere_volume, 4.0 * M_PI / 3.0, tolerance);
  }

  // Test 4: Sphere with translation - volume should increase
  {
    auto sphere = std::make_shared<coal::Sphere>(0.5);  // radius 0.5
    coal::Transform3s translation;
    translation.setIdentity();
    translation.translation() = coal::Vec3s(0.0, 1.0, 0.0);  // Translate 1 unit in y
    CastHullShape cast_hull_sphere_translated(sphere, translation);

    double sphere_volume = sphere->computeVolume();
    double cast_hull_sphere_translated_volume = cast_hull_sphere_translated.computeVolume();

    // Swept volume should be larger than original
    EXPECT_GT(cast_hull_sphere_translated_volume, sphere_volume);

    // Should be reasonable (not infinite or negative)
    EXPECT_GT(cast_hull_sphere_translated_volume, 0.0);
    EXPECT_LT(cast_hull_sphere_translated_volume, 10.0);  // Reasonable upper bound
  }

  // Test 5: Small translation should result in small volume increase
  {
    auto box = std::make_shared<coal::Box>(1.0, 1.0, 1.0);  // Unit cube, volume = 1.0
    coal::Transform3s small_translation;
    small_translation.setIdentity();
    small_translation.translation() = coal::Vec3s(0.1, 0.0, 0.0);  // Small translation
    CastHullShape cast_hull_small_translation(box, small_translation);

    double box_volume = box->computeVolume();
    double cast_hull_small_volume = cast_hull_small_translation.computeVolume();

    // Should be larger but not dramatically so
    EXPECT_GT(cast_hull_small_volume, box_volume);
    EXPECT_LT(cast_hull_small_volume, box_volume * 2.0);  // Shouldn't double the volume
  }

  // Test 6: Rotation with translation
  {
    auto box = std::make_shared<coal::Box>(1.0, 1.0, 1.0);  // Unit cube
    coal::Transform3s transform;
    transform.setIdentity();

    // Apply a 45-degree rotation around z-axis and translation
    double angle = M_PI / 4.0;  // 45 degrees
    coal::Matrix3s rotation;
    rotation << cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0, 0, 0, 1;
    transform.rotation() = rotation;
    transform.translation() = coal::Vec3s(1.0, 1.0, 0.0);
    CastHullShape cast_hull_rotated(box, transform);

    double box_volume = box->computeVolume();
    double cast_hull_rotated_volume = cast_hull_rotated.computeVolume();

    // Should be larger due to both rotation and translation
    EXPECT_GT(cast_hull_rotated_volume, box_volume);

    // Should be reasonable
    EXPECT_GT(cast_hull_rotated_volume, 0.0);
    EXPECT_LT(cast_hull_rotated_volume, 20.0);  // Reasonable upper bound
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
