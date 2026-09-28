#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <map>
#include <utility>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/collision/test_suite/collision_cast_scenario_unit.hpp>

using namespace tesseract::collision;

/**
 * @brief Each octree leaf must be seeded from its own geometry, so a repeated query reproduces the first.
 *
 * Ending the sweep at 0.85 along an axis leaves the 0.2m box overlapping the outermost voxel layer
 * and centred on the two voxel boundaries transverse to that axis, so it penetrates four voxels,
 * each by 0.1m along both transverse axes -- a genuine tie between two minimum-translation
 * directions per voxel, both shorter than the 0.25m push back along the sweep. Either direction is
 * correct, and which one a voxel reports depends on its narrowphase seed.
 *
 * A guess cached against the pair is a single direction, while the query runs one narrowphase call
 * per voxel. A second query that reused it would seed every voxel from the direction the first
 * query's last voxel ended on, rather than from each voxel's own bounding volume, and resolve the
 * ties differently from the first query.
 */
namespace
{
/** @brief Sweep a 0.2m box from 2.0 to 0.85 along @p axis, into a 2m box octree at the origin, twice. */
std::pair<ContactResultVector, ContactResultVector> sweptBoxContacts(const Eigen::Vector3d& axis)
{
  tesseract_collision_coal::CoalCastBVHManager checker;
  test_suite::detail::addOctree(checker, "octomap_link");
  test_suite::detail::addBoxLink(checker, "box_link", Eigen::Vector3d(0.1, 0.1, 0.1));

  checker.setActiveCollisionObjects({ "box_link" });
  checker.setDefaultCollisionMargin(0.0);
  checker.setCollisionObjectsTransform("octomap_link", Eigen::Isometry3d::Identity());

  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  start.translation() = axis * 2.0;
  Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
  end.translation() = axis * 0.85;
  checker.setCollisionObjectsTransform("box_link", start, end);

  // The first query leaves a warm-start guess behind; the second is the one that must not reuse it.
  ContactResultVector first = test_suite::detail::runContact(checker);
  ContactResultVector second = test_suite::detail::runContact(checker);
  return { first, second };
}

/** @brief Map each penetrated voxel to the normal reported for it, pointing away from the box. */
std::map<int, Eigen::Vector3d> voxelNormals(const ContactResultVector& contacts, const Eigen::Vector3d& axis)
{
  EXPECT_EQ(contacts.size(), 4U) << "the box straddles both transverse voxel boundaries, so it "
                                    "penetrates exactly four voxels";

  // The two axes transverse to the sweep.
  const Eigen::Vector3d u(axis.y(), axis.z(), axis.x());
  const Eigen::Vector3d v(axis.z(), axis.x(), axis.y());

  std::map<int, Eigen::Vector3d> normals;
  for (const auto& cr : contacts)
  {
    // A transverse normal on the wrong side of its voxel would report 0.3m, so a 0.1m depth along
    // a transverse axis is one of that voxel's two tied directions.
    EXPECT_NEAR(cr.distance, -0.1, 1e-6) << "each voxel is penetrated by the box's transverse half-width";

    const bool box_first = (cr.link_ids[0].name() == "box_link");
    const Eigen::Vector3d normal = box_first ? cr.normal : Eigen::Vector3d(-cr.normal);
    EXPECT_NEAR(std::abs(normal.dot(u)) + std::abs(normal.dot(v)), 1.0, 1e-6)
        << "normal (" << normal.x() << ", " << normal.y() << ", " << normal.z()
        << ") is not along either transverse axis";

    const int voxel = cr.subshape_id[box_first ? 1 : 0];
    EXPECT_TRUE(normals.emplace(voxel, normal).second) << "voxel " << voxel << " reported twice";
  }
  return normals;
}

void expectRepeatableNormals(const Eigen::Vector3d& axis)
{
  const auto [first_contacts, second_contacts] = sweptBoxContacts(axis);
  const std::map<int, Eigen::Vector3d> first = voxelNormals(first_contacts, axis);
  const std::map<int, Eigen::Vector3d> second = voxelNormals(second_contacts, axis);

  ASSERT_EQ(first.size(), second.size());
  for (const auto& [voxel, normal] : first)
  {
    const auto it = second.find(voxel);
    ASSERT_NE(it, second.end()) << "voxel " << voxel << " missing from the repeated query";
    EXPECT_LE((it->second - normal).norm(), 1e-6) << "voxel " << voxel << " resolved its tie to (" << normal.transpose()
                                                  << ") first, then to (" << it->second.transpose() << ")";
  }
}
}  // namespace

TEST(CoalCastOctreeLeafSeedUnit, BoxSweepAlongX)  // NOLINT
{
  expectRepeatableNormals(Eigen::Vector3d::UnitX());
}

TEST(CoalCastOctreeLeafSeedUnit, BoxSweepAlongY)  // NOLINT
{
  expectRepeatableNormals(Eigen::Vector3d::UnitY());
}

TEST(CoalCastOctreeLeafSeedUnit, BoxSweepAlongZ)  // NOLINT
{
  expectRepeatableNormals(Eigen::Vector3d::UnitZ());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
