#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <cmath>
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_casthullshape.h>
#include <tesseract/collision/coal/coal_utils.h>
#include <tesseract/geometry/impl/convex_mesh.h>

using namespace tesseract::collision;

namespace
{
/** @brief A 35-vertex bipyramid: past coal's 32-vertex threshold, so support
 * queries take the hill-climbing path that reads and writes the vertex hint
 * and ShapeSupportData. */
std::shared_ptr<coal::ShapeBase> makeLargeConvex()
{
  using tesseract::collision::tesseract_collision_coal::createShapePrimitive;

  constexpr int ring = 33;
  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
  vertices->emplace_back(0.0, 0.0, 1.0);
  vertices->emplace_back(0.0, 0.0, -1.0);
  for (int i = 0; i < ring; ++i)
  {
    const double a = 2.0 * M_PI * i / ring;
    vertices->emplace_back(std::cos(a), std::sin(a), 0.0);
  }

  auto faces = std::make_shared<Eigen::VectorXi>(8 * ring);
  int idx = 0;
  for (int i = 0; i < ring; ++i)
  {
    const int a = 2 + i;
    const int b = 2 + ((i + 1) % ring);
    (*faces)[idx++] = 3;
    (*faces)[idx++] = 0;
    (*faces)[idx++] = a;
    (*faces)[idx++] = b;
    (*faces)[idx++] = 3;
    (*faces)[idx++] = 1;
    (*faces)[idx++] = b;
    (*faces)[idx++] = a;
  }

  auto mesh = std::make_shared<tesseract::geometry::ConvexMesh>(vertices, faces, 2 * ring);
  return std::dynamic_pointer_cast<coal::ShapeBase>(createShapePrimitive(mesh));
}
/** @brief A unit cube as an 8-vertex convex hull: exact half-extents of 0.5 on
 * every axis, and a node type that takes computeShapeAABB's convex branch. */
std::shared_ptr<coal::ShapeBase> makeCubeConvex()
{
  using tesseract::collision::tesseract_collision_coal::createShapePrimitive;

  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
  for (int i = 0; i < 8; ++i)
    vertices->emplace_back((i & 1) ? 0.5 : -0.5, (i & 2) ? 0.5 : -0.5, (i & 4) ? 0.5 : -0.5);

  // Six quads, each written as [vertex count, indices...].
  auto faces = std::make_shared<Eigen::VectorXi>(30);
  (*faces) << 4, 0, 1, 3, 2,  // -z
      4, 4, 6, 7, 5,          // +z
      4, 0, 4, 5, 1,          // -y
      4, 2, 3, 7, 6,          // +y
      4, 0, 2, 6, 4,          // -x
      4, 1, 5, 7, 3;          // +x

  auto mesh = std::make_shared<tesseract::geometry::ConvexMesh>(vertices, faces, 6);
  return std::dynamic_pointer_cast<coal::ShapeBase>(createShapePrimitive(mesh));
}
}  // namespace

TEST(CoalCastHullShapeUnit, LocalAABBCountsWrappedSweptSphereRadiusOnce)
{
  using tesseract::collision::tesseract_collision_coal::CastHullShape;

  constexpr double ssr = 0.1;
  constexpr double tolerance = 1e-6;

  coal::Transform3s shifted = coal::Transform3s::Identity();
  shifted.translation() = coal::Vec3s(1.0, 0.0, 0.0);

  // computeShapeAABB dispatches convex hulls to the computeBV<AABB, ShapeBase>
  // fallback, which derives its bound from aabb_local and so already carries the
  // radius; the parametric primitive branches do not and add it themselves. Both
  // poses must end up with it exactly once, whichever branch ran.
  {
    auto convex = makeCubeConvex();
    ASSERT_NE(convex, nullptr);
    convex->setSweptSphereRadius(ssr);
    CastHullShape cast_hull(convex, shifted);
    cast_hull.computeLocalAABB();

    // Pose 0 spans [-0.6, 0.6]; pose 1 is that box shifted 1 in x.
    EXPECT_NEAR(cast_hull.aabb_local.min_[0], -0.5 - ssr, tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.max_[0], 1.5 + ssr, tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.min_[1], -0.5 - ssr, tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.max_[1], 0.5 + ssr, tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.min_[2], -0.5 - ssr, tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.max_[2], 0.5 + ssr, tolerance);
  }

  {
    auto box = std::make_shared<coal::Box>(1.0, 1.0, 1.0);
    box->setSweptSphereRadius(ssr);
    CastHullShape cast_hull(box, shifted);
    cast_hull.computeLocalAABB();

    EXPECT_NEAR(cast_hull.aabb_local.min_[0], -0.5 - ssr, tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.max_[0], 1.5 + ssr, tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.min_[1], -0.5 - ssr, tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.max_[1], 0.5 + ssr, tolerance);
  }
}

TEST(CoalCastHullShapeUnit, LocalAABBIgnoresTheWrappedShapesStaleCache)
{
  using tesseract::collision::tesseract_collision_coal::CastHullShape;

  constexpr double ssr = 0.1;
  constexpr double tolerance = 1e-6;

  coal::Transform3s shifted = coal::Transform3s::Identity();
  shifted.translation() = coal::Vec3s(1.0, 0.0, 0.0);

  // Same sweep and same radius as the test above, but the radius is set after the wrapper is
  // built. coal leaves aabb_local stale on setSweptSphereRadius, so a bound taken from it
  // would come out smaller than the shape and the broadphase would drop the pair. The bound
  // must not depend on which side of construction the radius was set.
  {
    auto convex = makeCubeConvex();
    ASSERT_NE(convex, nullptr);
    CastHullShape cast_hull(convex, shifted);
    convex->setSweptSphereRadius(ssr);
    cast_hull.computeLocalAABB();

    EXPECT_NEAR(cast_hull.aabb_local.min_[0], -0.5 - ssr, tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.max_[0], 1.5 + ssr, tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.min_[1], -0.5 - ssr, tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.max_[1], 0.5 + ssr, tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.min_[2], -0.5 - ssr, tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.max_[2], 0.5 + ssr, tolerance);
  }

  {
    auto box = std::make_shared<coal::Box>(1.0, 1.0, 1.0);
    CastHullShape cast_hull(box, shifted);
    box->setSweptSphereRadius(ssr);
    cast_hull.computeLocalAABB();

    EXPECT_NEAR(cast_hull.aabb_local.min_[0], -0.5 - ssr, tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.max_[0], 1.5 + ssr, tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.min_[1], -0.5 - ssr, tolerance);
    EXPECT_NEAR(cast_hull.aabb_local.max_[1], 0.5 + ssr, tolerance);
  }
}

TEST(CoalCastHullShapeUnit, LocalAABBUnit)
{
  using namespace tesseract::collision::tesseract_collision_coal;

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
    const coal::Vec3s expected_min(-0.5, -0.5, -0.5);
    const coal::Vec3s expected_max(0.5, 1.5, 0.5);
    for (int i = 0; i < 3; ++i)
    {
      EXPECT_NEAR(cast_hull_sphere.aabb_local.min_[i], expected_min[i], tolerance);
      EXPECT_NEAR(cast_hull_sphere.aabb_local.max_[i], expected_max[i], tolerance);
    }

    // Centre and radius of those same bounds, stated independently of the shape:
    // midpoint (0, 0.5, 0) and half-diagonal sqrt(0.5^2 + 1.0^2 + 0.5^2).
    const coal::Vec3s expected_center = (expected_min + expected_max) * 0.5;
    EXPECT_NEAR(cast_hull_sphere.aabb_center[0], expected_center[0], tolerance);
    EXPECT_NEAR(cast_hull_sphere.aabb_center[1], expected_center[1], tolerance);
    EXPECT_NEAR(cast_hull_sphere.aabb_center[2], expected_center[2], tolerance);

    EXPECT_NEAR(cast_hull_sphere.aabb_radius, sqrt(1.5), tolerance);
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

    // Centre and radius of the union stated independently: pose 0 spans
    // [-0.5, 0.5] and pose 1 reaches 1 +/- sqrt(2)/2 in x and y, 0.5 in z.
    const coal::Vec3s expected_min(-0.5, -0.5, -0.5);
    const coal::Vec3s expected_max(1.0 + sqrt2_half, 1.0 + sqrt2_half, 0.5);
    const coal::Vec3s expected_center = (expected_min + expected_max) * 0.5;
    EXPECT_NEAR(cast_hull_rotated.aabb_center[0], expected_center[0], tolerance);
    EXPECT_NEAR(cast_hull_rotated.aabb_center[1], expected_center[1], tolerance);
    EXPECT_NEAR(cast_hull_rotated.aabb_center[2], expected_center[2], tolerance);

    const double expected_radius = (expected_min - expected_center).norm();
    EXPECT_NEAR(cast_hull_rotated.aabb_radius, expected_radius, tolerance);
  }
}

TEST(CoalCastHullShapeUnit, ComputeVolumeUnit)
{
  using namespace tesseract::collision::tesseract_collision_coal;
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

    double cast_hull_translated_volume = cast_hull_translated.computeVolume();

    // Axis-aligned pure translation: the swept hull is exactly a 3 x 2 x 2 box,
    // so the volume is stated rather than bounded.
    EXPECT_NEAR(cast_hull_translated_volume, 12.0, tolerance);
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

    double cast_hull_small_volume = cast_hull_small_translation.computeVolume();

    // Same shape as test 2 at a smaller step: exactly a 1.1 x 1 x 1 hull.
    EXPECT_NEAR(cast_hull_small_volume, 1.1, tolerance);
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

TEST(CoalCastHullShapeUnit, ComputeVolumeLeavesWarmStartStateUntouched)  // NOLINT
{
  using namespace tesseract::collision::tesseract_collision_coal;

  auto convex = makeLargeConvex();
  ASSERT_NE(convex, nullptr);

  coal::Transform3s cast_tf;
  cast_tf.setIdentity();
  cast_tf.translation() = coal::Vec3s(0.5, 0.0, 0.0);
  CastHullShape cast_hull(convex, cast_tf);

  // Seed the warm-start state as a converged GJK run would have left it.
  cast_hull.getHint0() = 4;
  cast_hull.getHint1() = 7;
  cast_hull.getSupportData0().last_dir = coal::Vec3s(1.0, 0.0, 0.0);
  cast_hull.getSupportData1().last_dir = coal::Vec3s(1.0, 0.0, 0.0);

  const double volume = cast_hull.computeVolume();

  // The swept-hull AABB estimate must exceed the unswept base volume.
  EXPECT_GT(volume, convex->computeVolume());

  EXPECT_EQ(cast_hull.getHint0(), 4);
  EXPECT_EQ(cast_hull.getHint1(), 7);
  EXPECT_TRUE(cast_hull.getSupportData0().last_dir.isApprox(coal::Vec3s(1.0, 0.0, 0.0)));
  EXPECT_TRUE(cast_hull.getSupportData1().last_dir.isApprox(coal::Vec3s(1.0, 0.0, 0.0)));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
