#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_utils.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/common/types.h>

namespace tesseract::collision::tesseract_collision_coal
{
namespace
{
class UnsupportedGeometry final : public tesseract::geometry::Geometry
{
public:
  UnsupportedGeometry() : tesseract::geometry::Geometry(tesseract::geometry::GeometryType::UNINITIALIZED) {}

  tesseract::geometry::Geometry::Ptr clone() const override { return std::make_shared<UnsupportedGeometry>(); }
};

std::shared_ptr<tesseract::geometry::Mesh> makeTriangleMesh()
{
  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
  vertices->emplace_back(0, 0, 0);
  vertices->emplace_back(1, 0, 0);
  vertices->emplace_back(0, 1, 0);

  auto triangles = std::make_shared<Eigen::VectorXi>(4);
  (*triangles) << 3, 0, 1, 2;

  return std::make_shared<tesseract::geometry::Mesh>(vertices, triangles, 1);
}
}  // namespace

TEST(CoalUtilsShapeConversionUnit, CompoundMeshDirectConversionThrows)  // NOLINT
{
  std::vector<std::shared_ptr<tesseract::geometry::Mesh>> meshes;
  meshes.push_back(makeTriangleMesh());
  meshes.push_back(makeTriangleMesh());

  auto compound_mesh = std::make_shared<tesseract::geometry::CompoundMesh>(std::move(meshes));

  EXPECT_THROW(createShapePrimitive(compound_mesh), std::runtime_error);
}

TEST(CoalUtilsShapeConversionUnit, UnsupportedGeometryTypeReturnsNullptr)  // NOLINT
{
  auto unsupported = std::make_shared<UnsupportedGeometry>();

  CollisionGeometryPtr shape = createShapePrimitive(unsupported);
  EXPECT_EQ(shape, nullptr);
}

TEST(CoalUtilsShapeConversionUnit, EmptyMeshReturnsNullptr)  // NOLINT
{
  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
  auto triangles = std::make_shared<Eigen::VectorXi>();
  auto empty_mesh = std::make_shared<tesseract::geometry::Mesh>(vertices, triangles, 0);

  CollisionGeometryPtr shape = createShapePrimitive(empty_mesh);
  EXPECT_EQ(shape, nullptr);
}

TEST(CoalUtilsShapeConversionUnit, EmptyConvexMeshReturnsNullptr)  // NOLINT
{
  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
  auto faces = std::make_shared<Eigen::VectorXi>();
  auto empty_convex_mesh = std::make_shared<tesseract::geometry::ConvexMesh>(vertices, faces, 0);

  CollisionGeometryPtr shape = createShapePrimitive(empty_convex_mesh);
  EXPECT_EQ(shape, nullptr);
}
}  // namespace tesseract::collision::tesseract_collision_coal

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
