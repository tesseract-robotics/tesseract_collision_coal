#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <coal/shape/geometric_shapes.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_collision_geometry_cache.h>
#include <tesseract/geometry/geometries.h>

namespace tesseract::collision::tesseract_collision_coal
{
TEST(CoalCollisionGeometryCacheUnit, InsertAndGetReturnsStoredGeometry)  // NOLINT
{
  auto key = std::make_shared<tesseract::geometry::Sphere>(0.25);
  auto value = std::make_shared<coal::Sphere>(0.25);

  ASSERT_FALSE(key->getUUID().is_nil());

  CoalCollisionGeometryCache::insert(key, value);
  std::shared_ptr<coal::CollisionGeometry> cached = CoalCollisionGeometryCache::get(key);

  ASSERT_NE(cached, nullptr);
  EXPECT_EQ(cached.get(), value.get());
}

TEST(CoalCollisionGeometryCacheUnit, ExpiredEntriesReturnNullAndPruneKeepsLiveEntries)  // NOLINT
{
  auto expired_key = std::make_shared<tesseract::geometry::Sphere>(0.33);
  auto live_key = std::make_shared<tesseract::geometry::Sphere>(0.44);

  {
    auto expired_value = std::make_shared<coal::Sphere>(0.33);
    CoalCollisionGeometryCache::insert(expired_key, expired_value);
  }

  auto live_value = std::make_shared<coal::Sphere>(0.44);
  CoalCollisionGeometryCache::insert(live_key, live_value);

  EXPECT_EQ(CoalCollisionGeometryCache::get(expired_key), nullptr);

  CoalCollisionGeometryCache::prune();

  std::shared_ptr<coal::CollisionGeometry> live_cached = CoalCollisionGeometryCache::get(live_key);
  ASSERT_NE(live_cached, nullptr);
  EXPECT_EQ(live_cached.get(), live_value.get());
}
}  // namespace tesseract::collision::tesseract_collision_coal

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
