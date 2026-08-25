#ifndef TESSERACT_COLLISION_OCTREE_SUBSHAPE_ID_CONTRACT_HPP
#define TESSERACT_COLLISION_OCTREE_SUBSHAPE_ID_CONTRACT_HPP

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <algorithm>
#include <set>
#include <vector>
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract::collision::test_suite
{
/**
 * @brief What kind of identity a backend's octree subshape_id carries.
 *
 * A backend that expands an octree into per-voxel shapes reports an index into the primitives it
 * built, so the value is bounded by the occupied-leaf count. A backend that collides against the
 * tree itself has no such array and instead names the node it hit; that value separates nodes but
 * indexes nothing, so it carries no bound.
 */
enum class OctreeSubshapeIdKind
{
  LeafOrdinal,
  NodeHandle
};

/**
 * @brief Assert an octree contact's subshape_id names the primitive that was hit.
 *
 * -1 spells unset, and a consumer that partitions contacts by subshape cannot separate voxels that
 * all report unset, so every backend owes a set value. A backend that expands the tree owes more:
 * its value is an index into the primitives it built and must lie inside that range.
 */
inline void expectOctreeSubshapeIdNamesPrimitive(int subshape_id, OctreeSubshapeIdKind kind, int leaf_count)
{
  EXPECT_GE(subshape_id, 0) << "Octree subshape_id is unset; an octree contact must name the primitive it hit.";

  if (kind == OctreeSubshapeIdKind::LeafOrdinal)
  {
    EXPECT_LT(subshape_id, leaf_count) << "Octree subshape_id " << subshape_id
                                       << " is not a valid occupied-leaf index in [0, " << leaf_count << ").";
  }
}

/**
 * @brief Assert two identical queries report the same octree subshape ids.
 *
 * Nothing about the scene changed between the queries, so a value that moves does not identify a
 * primitive and cannot be grouped on. Order is not part of the contract, so compare as multisets.
 */
inline void expectOctreeSubshapeIdsStable(std::vector<int> first, std::vector<int> second)
{
  std::sort(first.begin(), first.end());
  std::sort(second.begin(), second.end());
  EXPECT_EQ(first, second) << "Repeating an unchanged query reported different octree subshape ids.";
}

/**
 * @brief Assert the reported octree subshape ids separate the primitives that were hit.
 *
 * A backend may legitimately report one id many times -- several contacts can fall on one voxel --
 * so this only asserts the value is not constant across a scene whose contacts span many voxels. A
 * constant id partitions nothing, which is indistinguishable from reporting nothing at all.
 */
inline void expectOctreeSubshapeIdsDiscriminate(const std::vector<int>& subshape_ids)
{
  ASSERT_GT(subshape_ids.size(), 1U);
  const std::set<int> distinct(subshape_ids.begin(), subshape_ids.end());
  EXPECT_GT(distinct.size(), 1U) << "Every octree contact reported the same subshape_id " << subshape_ids.front()
                                 << ", so the id separates no primitives.";
}

}  // namespace tesseract::collision::test_suite

#endif  // TESSERACT_COLLISION_OCTREE_SUBSHAPE_ID_CONTRACT_HPP
