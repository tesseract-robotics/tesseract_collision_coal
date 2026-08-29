#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <cmath>
#include <cstddef>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/collision/test_suite/collision_cast_scenario_unit.hpp>

using namespace tesseract::collision;

/**
 * @brief A box swept into a symmetric straddle of octree voxels must get a symmetric normal set.
 *
 * Ending the sweep at 0.85 along an axis leaves the 0.2m box overlapping the outermost voxel layer
 * and straddling the two voxel boundaries transverse to that axis, so it penetrates four voxels by
 * the same 0.1m. Each voxel's minimum-translation direction is unique -- 0.1m transverse against
 * 0.25m along the sweep -- so the four normals must be the four distinct unit vectors along the two
 * transverse axes, each the mirror of another. A narrowphase that resolves the tie inconsistently
 * returns two voxels claiming the same normal, which no per-contact check catches: every contact
 * still carries the correct depth, and only the set as a whole is wrong.
 *
 * The pair is queried twice, because a first query is what leaves a warm-start guess behind. A
 * guess cached against the pair is a single direction, while the query runs one narrowphase call
 * per voxel, so a second query that reused it would seed every voxel from one direction and
 * collapse the tie the same way for all of them.
 */
namespace
{
/** @brief Sweep a 0.2m box from 2.0 to 0.85 along @p axis, into a 2m box octree at the origin. */
ContactResultVector sweptBoxContacts(const Eigen::Vector3d& axis)
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

  // Discarded: this first query is what leaves a warm-start guess behind, so the query under test
  // is the one that must not reuse it.
  test_suite::detail::runContact(checker);

  return test_suite::detail::runContact(checker);
}

void expectSymmetricNormals(const Eigen::Vector3d& axis)
{
  const ContactResultVector contacts = sweptBoxContacts(axis);
  ASSERT_EQ(contacts.size(), 4U) << "the box straddles both transverse voxel boundaries, so it "
                                    "penetrates exactly four voxels";

  // The two axes transverse to the sweep, and the four normals they must produce.
  const Eigen::Vector3d u(axis.y(), axis.z(), axis.x());
  const Eigen::Vector3d v(axis.z(), axis.x(), axis.y());
  const std::vector<Eigen::Vector3d> expected{ u, -u, v, -v };

  std::vector<bool> seen(expected.size(), false);
  for (const auto& cr : contacts)
  {
    EXPECT_NEAR(cr.distance, -0.1, 1e-6) << "each voxel is penetrated by the box's transverse half-width";

    const Eigen::Vector3d normal = (cr.link_ids[0].name() == "box_link") ? cr.normal : Eigen::Vector3d(-cr.normal);
    bool found = false;
    for (std::size_t j = 0; j < expected.size() && !found; ++j)
    {
      if (!seen[j] && (normal - expected[j]).norm() <= 1e-6)
      {
        seen[j] = true;
        found = true;
      }
    }
    EXPECT_TRUE(found) << "normal (" << normal.x() << ", " << normal.y() << ", " << normal.z()
                       << ") is not one of the four transverse directions, or repeats one already reported";
  }
}
}  // namespace

TEST(CoalCastOctreeNormalSymmetryUnit, BoxSweepAlongX)  // NOLINT
{
  expectSymmetricNormals(Eigen::Vector3d::UnitX());
}

TEST(CoalCastOctreeNormalSymmetryUnit, BoxSweepAlongY)  // NOLINT
{
  expectSymmetricNormals(Eigen::Vector3d::UnitY());
}

TEST(CoalCastOctreeNormalSymmetryUnit, BoxSweepAlongZ)  // NOLINT
{
  expectSymmetricNormals(Eigen::Vector3d::UnitZ());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
