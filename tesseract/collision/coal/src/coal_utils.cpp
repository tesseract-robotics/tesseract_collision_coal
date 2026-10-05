/**
 * @file coal_utils.cpp
 * @brief Tesseract Coal Utility Functions.
 *
 * @author Roelof Oomen, Levi Armstrong
 * @date Dec 18, 2017
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (BSD)
 * @par
 * All rights reserved.
 * @par
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * @par
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * @par
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <coal/collision_data.h>
#include <coal/collision.h>
#include <coal/distance.h>
#include <coal/BVH/BVH_model.h>
#include <coal/shape/geometric_shapes.h>
#include <coal/shape/geometric_shapes_utility.h>
#include <coal/narrowphase/support_functions.h>
#include <coal/shape/convex.h>
#include <coal/data_types.h>
#include <coal/octree.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <unordered_set>
#include <utility>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/logging.h>
#include <tesseract/collision/coal/coal_utils.h>
#include <tesseract/collision/coal/coal_collision_geometry_cache.h>
#include <tesseract/collision/coal/coal_casthullshape.h>
#include <tesseract/collision/coal/coal_d_arc.h>
#include <tesseract/collision/coal/coal_collision_object_wrapper.h>
#include <tesseract/geometry/geometries.h>

namespace tesseract::collision::tesseract_collision_coal
{
namespace
{
/** @brief Bound the shape from its parameters, for the node types coal specializes computeBV
 * for. The parametric specializations leave the swept-sphere radius out; callers add it.
 * @return false for convex hulls, GEOM_CUSTOM and future types, which have no parametric form
 * and must be bounded from a local AABB instead. */
bool computeParametricShapeAABB(const coal::ShapeBase& s, const coal::Transform3s& tf, coal::AABB& bv)
{
  switch (s.getNodeType())
  {
    case coal::GEOM_BOX:
      coal::computeBV<coal::AABB>(static_cast<const coal::Box&>(s), tf, bv);
      break;
    case coal::GEOM_SPHERE:
      coal::computeBV<coal::AABB>(static_cast<const coal::Sphere&>(s), tf, bv);
      break;
    case coal::GEOM_ELLIPSOID:
      coal::computeBV<coal::AABB>(static_cast<const coal::Ellipsoid&>(s), tf, bv);
      break;
    case coal::GEOM_CAPSULE:
      coal::computeBV<coal::AABB>(static_cast<const coal::Capsule&>(s), tf, bv);
      break;
    case coal::GEOM_CONE:
      coal::computeBV<coal::AABB>(static_cast<const coal::Cone&>(s), tf, bv);
      break;
    case coal::GEOM_CYLINDER:
      coal::computeBV<coal::AABB>(static_cast<const coal::Cylinder&>(s), tf, bv);
      break;
    case coal::GEOM_TRIANGLE:
      coal::computeBV<coal::AABB>(static_cast<const coal::TriangleP&>(s), tf, bv);
      break;
    case coal::GEOM_HALFSPACE:
      coal::computeBV<coal::AABB>(static_cast<const coal::Halfspace&>(s), tf, bv);
      break;
    case coal::GEOM_PLANE:
      coal::computeBV<coal::AABB>(static_cast<const coal::Plane&>(s), tf, bv);
      break;
    case coal::GEOM_CONVEX32:
    case coal::GEOM_CONVEX16:
    default:
      return false;
  }

  return true;
}

/** @brief coal's computeBV<AABB, ShapeBase> formula against a caller-supplied local AABB:
 * conservative O(1) |R|*half-extents about the transformed centre. The exact convex fit
 * (computeAABBConvex) is O(num_points) and too costly on the per-check cast path for the
 * broadphase tightness it buys. */
void computeShapeAABBFromLocal(const coal::AABB& local_aabb, const coal::Transform3s& tf, coal::AABB& bv)
{
  const coal::Matrix3s& rotation = tf.getRotation();
  const coal::Vec3s half = (local_aabb.max_ - local_aabb.min_) * 0.5;
  const coal::Vec3s center = rotation * local_aabb.center() + tf.getTranslation();
  const coal::Vec3s delta(rotation.cwiseAbs() * half);

  bv.min_ = center - delta;
  bv.max_ = center + delta;
}

}  // namespace

bool computeTightLocalAABB(const coal::ShapeBase& s, coal::AABB& bv)
{
  const coal::Transform3s identity_tf;

  if (computeParametricShapeAABB(s, identity_tf, bv))
    return true;

  // The convex arms live here rather than in computeParametricShapeAABB because the exact fit is
  // O(num_points) and that function runs on the per-check path, where the conservative O(1)
  // formula is used instead. This runs once per shape.
  switch (s.getNodeType())
  {
    case coal::GEOM_CONVEX32:
      coal::computeBV<coal::AABB, coal::ConvexBase32>(static_cast<const coal::ConvexBase32&>(s), identity_tf, bv);
      return true;
    case coal::GEOM_CONVEX16:
      coal::computeBV<coal::AABB, coal::ConvexBase16>(static_cast<const coal::ConvexBase16&>(s), identity_tf, bv);
      return true;
    default:
      return false;
  }
}

void computeShapeAABB(const coal::ShapeBase& s,
                      const coal::Transform3s& tf,
                      const coal::AABB& local_aabb,
                      coal::AABB& bv)
{
  if (!computeParametricShapeAABB(s, tf, bv))
    computeShapeAABBFromLocal(local_aabb, tf, bv);

  // Uniform across both branches: the parametric specializations leave the radius out, and
  // local_aabb is radius-free by contract.
  const coal::Scalar ssr = s.getSweptSphereRadius();
  if (ssr > 0)
    bv.expand(ssr);
}

namespace
{
// Apply the inverse of a rigid transform to a point without materializing the
// inverse: tf⁻¹ · p == Rᵀ (p − t).
inline Eigen::Vector3d applyInverse(const Eigen::Isometry3d& tf, const Eigen::Vector3d& p)
{
  return tf.linear().transpose() * (p - tf.translation());
}

CollisionGeometryPtr createShapePrimitive(const tesseract::geometry::Plane::ConstPtr& geom)
{
  return std::make_shared<coal::Plane>(geom->getA(), geom->getB(), geom->getC(), geom->getD());
}

CollisionGeometryPtr createShapePrimitive(const tesseract::geometry::Box::ConstPtr& geom)
{
  return std::make_shared<coal::Box>(geom->getX(), geom->getY(), geom->getZ());
}

CollisionGeometryPtr createShapePrimitive(const tesseract::geometry::Sphere::ConstPtr& geom)
{
  return std::make_shared<coal::Sphere>(geom->getRadius());
}

CollisionGeometryPtr createShapePrimitive(const tesseract::geometry::Cylinder::ConstPtr& geom)
{
  return std::make_shared<coal::Cylinder>(geom->getRadius(), geom->getLength());
}

CollisionGeometryPtr createShapePrimitive(const tesseract::geometry::Cone::ConstPtr& geom)
{
  return std::make_shared<coal::Cone>(geom->getRadius(), geom->getLength());
}

CollisionGeometryPtr createShapePrimitive(const tesseract::geometry::Capsule::ConstPtr& geom)
{
  return std::make_shared<coal::Capsule>(geom->getRadius(), geom->getLength());
}

CollisionGeometryPtr createShapePrimitive(const tesseract::geometry::Mesh::ConstPtr& geom)
{
  const int vertex_count = geom->getVertexCount();
  const int triangle_count = geom->getFaceCount();
  const tesseract::common::VectorVector3d& vertices = *(geom->getVertices());
  const Eigen::VectorXi& triangles = *(geom->getFaces());

  auto g = std::make_shared<coal::BVHModel<coal::OBBRSS>>();
  if (vertex_count > 0 && triangle_count > 0)
  {
    using Index = coal::Triangle32::IndexType;
    std::vector<coal::Triangle32> tri_indices(static_cast<size_t>(triangle_count));
    for (int i = 0; i < triangle_count; ++i)
    {
      assert(triangles[4L * i] == 3);
      tri_indices[static_cast<size_t>(i)] = coal::Triangle32(static_cast<Index>(triangles[(4 * i) + 1]),
                                                             static_cast<Index>(triangles[(4 * i) + 2]),
                                                             static_cast<Index>(triangles[(4 * i) + 3]));
    }

    g->beginModel();
    g->addSubModel(vertices, tri_indices);
    g->endModel();

    return g;
  }

  TESSERACT_LOG_ERROR("The mesh is empty!");
  return nullptr;
}

// Coal polygon type (modelled after TriangleTpl)
template <typename IndexType_>
struct PolygonTpl : Eigen::Matrix<IndexType_, -1, 1>
{
  using IndexType = IndexType_;
  using size_type = int;

  // template <typename OtherIndexType>
  // friend class Polygon;

  /// @brief Default constructor
  PolygonTpl() = default;

  /// @brief Copy constructor
  PolygonTpl(const PolygonTpl& other) : Eigen::Matrix<IndexType_, -1, 1>(other) {}

  /// @brief Move constructor
  PolygonTpl(PolygonTpl&& other) noexcept : Eigen::Matrix<IndexType_, -1, 1>(std::move(other)) {}

  /// @brief Destructor
  ~PolygonTpl() = default;

  /// @brief Copy constructor from another vertex index type.
  template <typename OtherIndexType>
  PolygonTpl(const PolygonTpl<OtherIndexType>& other)
  {
    *this = other;
  }

  /// @brief Copy operator
  PolygonTpl& operator=(const PolygonTpl& other)
  {
    this->_set(other);
    return *this;
  }

  /// @brief Move assignment
  PolygonTpl& operator=(PolygonTpl&& other) noexcept
  {
    this->_set(std::move(other));
    return *this;
  }

  /// @brief Copy operator from another index type.
  template <typename OtherIndexType>
  PolygonTpl& operator=(const PolygonTpl<OtherIndexType>& other)
  {
    *this = other.template cast<OtherIndexType>();
    return *this;
  }

  template <typename OtherIndexType>
  PolygonTpl<OtherIndexType> cast() const
  {
    PolygonTpl<OtherIndexType> res;
    res._set(*this);
    return res;
  }
};

using Polygon = PolygonTpl<std::uint32_t>;

CollisionGeometryPtr createShapePrimitive(const tesseract::geometry::ConvexMesh::ConstPtr& geom)
{
  const auto vertex_count = geom->getVertexCount();
  const auto face_count = geom->getFaceCount();
  const auto& faces = *geom->getFaces();

  if (vertex_count > 0 && face_count > 0)
  {
    auto vertices = std::const_pointer_cast<tesseract::common::VectorVector3d>(geom->getVertices());

    auto new_faces = std::make_shared<std::vector<Polygon>>();
    new_faces->reserve(static_cast<size_t>(face_count));
    for (int i = 0; i < faces.size(); ++i)
    {
      Polygon new_face;
      // First value of each face is the number of vertices
      new_face.resize(faces[i]);
      for (std::uint32_t& j : new_face)
      {
        ++i;
        j = static_cast<std::uint32_t>(faces[i]);
      }
      new_faces->emplace_back(new_face);
    }
    assert(new_faces->size() == static_cast<size_t>(face_count));

    return std::make_shared<coal::Convex<Polygon>>(vertices, vertex_count, new_faces, face_count);
  }

  TESSERACT_LOG_ERROR("The mesh is empty!");
  return nullptr;
}

CollisionGeometryPtr createShapePrimitive(const tesseract::geometry::Octree::ConstPtr& geom)
{
  switch (geom->getSubType())
  {
    case tesseract::geometry::OctreeSubType::BOX:
    {
      return std::make_shared<coal::OcTree>(geom->getOctree());
    }
    default:
    {
      TESSERACT_LOG_ERROR("This Coal octree sub shape type ({}) is not supported for geometry octree",
                          static_cast<int>(geom->getSubType()));
      return nullptr;
    }
  }
}

CollisionGeometryPtr createShapePrimitiveHelper(const CollisionShapeConstPtr& geom)
{
  switch (geom->getType())
  {
    case tesseract::geometry::GeometryType::PLANE:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract::geometry::Plane>(geom));
    }
    case tesseract::geometry::GeometryType::BOX:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract::geometry::Box>(geom));
    }
    case tesseract::geometry::GeometryType::SPHERE:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract::geometry::Sphere>(geom));
    }
    case tesseract::geometry::GeometryType::CYLINDER:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract::geometry::Cylinder>(geom));
    }
    case tesseract::geometry::GeometryType::CONE:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract::geometry::Cone>(geom));
    }
    case tesseract::geometry::GeometryType::CAPSULE:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract::geometry::Capsule>(geom));
    }
    case tesseract::geometry::GeometryType::MESH:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract::geometry::Mesh>(geom));
    }
    case tesseract::geometry::GeometryType::CONVEX_MESH:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract::geometry::ConvexMesh>(geom));
    }
    case tesseract::geometry::GeometryType::OCTREE:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract::geometry::Octree>(geom));
    }
    case tesseract::geometry::GeometryType::COMPOUND_MESH:
    {
      throw std::runtime_error("CompoundMesh type should not be passed to this function!");
    }
    default:
    {
      TESSERACT_LOG_ERROR("This geometric shape type ({}) is not supported using Coal yet",
                          static_cast<int>(geom->getType()));
      return nullptr;
    }
  }
}
}  // namespace

CollisionGeometryPtr createShapePrimitive(const CollisionShapeConstPtr& geom)
{
  CollisionGeometryPtr shape = CoalCollisionGeometryCache::get(geom);
  if (shape != nullptr)
    return shape;

  shape = createShapePrimitiveHelper(geom);

  // Bound the geometry before it is shared. Every collision object built over it reads these
  // fields, and coal's CollisionObject constructor would otherwise recompute them per object --
  // a write through geometry other objects, and other threads, already hold.
  if (shape != nullptr)
    shape->computeLocalAABB();

  CoalCollisionGeometryCache::insert(geom, shape);
  return shape;
}

constexpr double COAL_SUPPORT_FUNC_TOLERANCE = 0.01;
constexpr double COAL_LENGTH_TOLERANCE = 0.001;
constexpr double COAL_EPSILON = 1e-6;

namespace
{
template <typename ConvexT>
void getAverageSupportFromConvex(const ConvexT* convex,
                                 const coal::Vec3s& localNormal,
                                 double& outsupport,
                                 coal::Vec3s& outpt,
                                 int& hint,
                                 coal::details::ShapeSupportData& support_data,
                                 bool use_flat)
{
  using IndexType = typename ConvexT::IndexType;

  if (use_flat)
  {
    // Direction-independent O(V) scan: average all vertices whose support is
    // tied (within COAL_EPSILON) for the maximum along localNormal. Used when
    // penetration is not requested: GJK boolean/distance early-outs without EPA,
    // leaving the shared warm-start seed poorly aimed, so the hill-climb below
    // would be a net loss.
    coal::Vec3s ptSum = coal::Vec3s::Zero();
    double ptCount = 0.0;
    double maxSupport = std::numeric_limits<double>::lowest();
    for (const auto& pt : *convex->points)
    {
      const double sup = pt.dot(localNormal);
      if (sup > maxSupport + COAL_EPSILON)
      {
        ptCount = 1.0;
        ptSum = pt;
        maxSupport = sup;
      }
      else if (sup >= maxSupport - COAL_EPSILON)
      {
        ptCount += 1.0;
        ptSum += pt;
      }
    }
    outsupport = maxSupport;
    outpt = ptSum / ptCount;
    return;
  }

  coal::Vec3s support;
  coal::details::getShapeSupport<coal::details::SupportOptions::NoSweptSphere>(
      convex, localNormal, support, hint, support_data);
  const double maxSupport = support.dot(localNormal);

  if (convex->neighbors == nullptr)
  {
    outsupport = maxSupport;
    outpt = support;
    return;
  }

  // Ensure visited is correctly sized. Coal's linear-path dispatch (V ≤ 32)
  // leaves the buffer untouched, so without this we could index out of bounds
  // on a fresh ShapeSupportData or one previously sized for a smaller shape.
  const auto num_points = static_cast<std::size_t>(convex->num_points);
  if (support_data.visited.size() != num_points)
    support_data.visited.assign(num_points, 0);
  else
    std::fill(support_data.visited.begin(), support_data.visited.end(), 0);

  support_data.visited[static_cast<std::size_t>(hint)] = 1;
  thread_local std::vector<IndexType> traversal_stack;
  traversal_stack.clear();
  traversal_stack.push_back(static_cast<IndexType>(hint));

  const auto& pts = *convex->points;
  const auto& nns = *convex->neighbors;
  coal::Vec3s ptSum = support;
  double ptCount = 1.0;
  while (!traversal_stack.empty())
  {
    const IndexType cur = traversal_stack.back();
    traversal_stack.pop_back();
    const auto& n = nns[cur];
    for (IndexType in = 0; in < n.count; ++in)
    {
      const IndexType ip = convex->neighbor(cur, in);
      if (support_data.visited[ip] != 0)
        continue;
      support_data.visited[ip] = 1;
      if (pts[ip].dot(localNormal) >= maxSupport - COAL_EPSILON)
      {
        ptSum += pts[ip];
        ptCount += 1.0;
        traversal_stack.push_back(ip);
      }
    }
  }
  outsupport = maxSupport;
  outpt = ptSum / ptCount;
}
}  // namespace

/**
 * @brief Compute the average support point for a shape along a direction.
 *
 * For polyhedral shapes, delegates to coal's hill-climb (logarithmic when a
 * neighbor graph is present and V > 32, linear otherwise) to find an extreme
 * vertex, then averages all tied vertices reached via a DFS over neighbors.
 * Tied vertices on a convex polytope form a connected supporting face, so the
 * traversal visits exactly that face. Matches Bullet's GetAverageSupport in
 * the practically-equivalent sense: identical for unique maxima and exact
 * ties; divergence bounded by COAL_EPSILON × face_extent in near-epsilon cases.
 * Box, capsule, cone and cylinder resolve their axis-aligned ties by zeroing
 * the components of the unit @p localNormal within COAL_EPSILON.
 *
 * @param hint In/out warm-start vertex index. On input, the starting vertex
 *             for hill-climb (typically the result of a previous call with a
 *             similar direction). On output, the extreme vertex index.
 * @param support_data In/out scratch data. Coal uses `visited` and `last_dir`
 *             to accelerate subsequent calls with related directions; pass a
 *             per-shape instance to preserve warm-start quality across calls.
 */
void GetAverageSupport(const coal::ShapeBase* shape,
                       const coal::Vec3s& localNormal,
                       double& outsupport,
                       coal::Vec3s& outpt,
                       int& hint,
                       coal::details::ShapeSupportData& support_data,
                       bool use_flat)
{
  coal::Vec3s support_dir = localNormal;
  switch (shape->getNodeType())
  {
    case coal::GEOM_CONVEX32:
    {
      const auto* convex = static_cast<const coal::ConvexBase32*>(shape);
      if (convex->points && !convex->points->empty())
      {
        getAverageSupportFromConvex(convex, localNormal, outsupport, outpt, hint, support_data, use_flat);
        return;
      }
      break;
    }
    case coal::GEOM_CONVEX16:
    {
      const auto* convex = static_cast<const coal::ConvexBase16*>(shape);
      if (convex->points && !convex->points->empty())
      {
        getAverageSupportFromConvex(convex, localNormal, outsupport, outpt, hint, support_data, use_flat);
        return;
      }
      break;
    }
    case coal::GEOM_BOX:
    case coal::GEOM_CAPSULE:
    case coal::GEOM_CONE:
    case coal::GEOM_CYLINDER:
      // Coal returns the centre of these shapes' axis-aligned tied sets (a box face or edge, a
      // capsule or cylinder side, a cylinder or cone cap) only for an exactly zero direction
      // component, so solver noise in the normal would move the point to one end of the set. Zero
      // every component of the unit normal within COAL_EPSILON.
      support_dir = (support_dir.array().abs() <= COAL_EPSILON).select(0.0, support_dir);
      break;
    default:
      break;
  }

  // Primitives and empty convex fall here. WithSweptSphere ensures Sphere returns
  // radius*normalize(dir) instead of zero (NoSweptSphere treats Sphere as a point + inflation).
  outpt = coal::details::getSupport<coal::details::SupportOptions::WithSweptSphere>(shape, support_dir, hint);
  outsupport = localNormal.dot(outpt);
}

bool needsCollisionCheck(const CollisionObjectWrapperBase* cd1,
                         const CollisionObjectWrapperBase* cd2,
                         const tesseract::common::LinkIdPair& pair,
                         const std::shared_ptr<const tesseract::common::ContactAllowedValidator>& validator)
{
  return cd1->m_enabled && cd2->m_enabled && (cd2->m_collisionFilterGroup & cd1->m_collisionFilterMask) &&  // NOLINT
         (cd1->m_collisionFilterGroup & cd2->m_collisionFilterMask) &&                                      // NOLINT
         !isContactAllowed(pair, validator);
}

/// Where along a sweep a contact lies, with the swept shape's resting points at either end of it.
struct SweepWitness
{
  double cc_time{ -1 };
  ContinuousCollisionType cc_type{ ContinuousCollisionType::CCType_None };
  coal::Vec3s pt_local0{ coal::Vec3s::Zero() };  ///< Averaged support at the start pose, in the shape's frame.
  coal::Vec3s pt_local1{ coal::Vec3s::Zero() };  ///< Averaged support at the end pose, in the shape's frame.
};

/// The scratch GetAverageSupport climbs on, one per thread, its visited buffer staying allocated across
/// calls. A call leaves nothing a later one reads: a caller seeds last_dir ahead of every climb, and the
/// climb re-initialises visited.
static coal::details::ShapeSupportData& averagingScratch()
{
  thread_local coal::details::ShapeSupportData scratch;
  return scratch;
}

/**
 * @brief Locate a contact on a swept shape.
 *
 * Uses the support-function approach of Bullet's calculateContinuousData: finds the shape's extreme points
 * along the contact normal at either end of the sweep, then classifies the contact time by which pose has
 * the greater support.
 *
 * @param hull The swept hull the narrowphase collided; its support hints seed the support queries
 * @param shape_tf0 The shape's pose at the start of the sweep
 * @param shape_tf1 The shape's pose at the end of the sweep, in the same frame
 * @param link_origin0 The origin of the shape's link, not of the shape, at the start of the sweep
 * @param link_origin1 The origin of the shape's link, not of the shape, at the end of the sweep
 * @param normal Unit contact normal pointing from this object toward the other
 * @param witness The narrowphase witness point on this object
 * @param use_flat Scan every vertex for the support instead of climbing from the hull's hints
 */
static SweepWitness locateOnSweep(const CastHullShape& hull,
                                  const Eigen::Isometry3d& shape_tf0,
                                  const Eigen::Isometry3d& shape_tf1,
                                  const Eigen::Vector3d& link_origin0,
                                  const Eigen::Vector3d& link_origin1,
                                  const coal::Vec3s& normal,
                                  const Eigen::Vector3d& witness,
                                  bool use_flat)
{
  SweepWitness w;

  // Transform normal into local frames at t=0 and t=1
  const coal::Vec3s normal_local0 = shape_tf0.linear().transpose() * normal;
  const coal::Vec3s normal_local1 = shape_tf1.linear().transpose() * normal;

  // Get averaged support points on the underlying shape at both local normals.
  // The averaging climb runs on thread_local scratch (its visited buffer stays
  // allocated across calls, like traversal_stack) rather than the sweep's own
  // hint/ShapeSupportData, so it never perturbs the sweep's warm-start chain.
  // When use_flat is false (penetration requested), EPA has converged, so the
  // sweep's hint/last_dir already match the contact normal — seed the scratch
  // from them for a high-quality start. When use_flat is true the flat scan
  // ignores the scratch entirely (see getAverageSupportFromConvex), so the
  // stale seed is harmless; the warm branch re-seeds last_dir and re-inits
  // visited on every call, so interleaved flat/warm queries never corrupt it.
  const coal::ShapeBase* underlying = hull.getUnderlyingShape().get();
  coal::details::ShapeSupportData& avg_data = averagingScratch();

  double sup_local0 = 0;
  int hint0 = 0;
  if (!use_flat)
  {
    hint0 = hull.getHint0();
    avg_data.last_dir = hull.getSupportData0().last_dir;
  }
  GetAverageSupport(underlying, normal_local0, sup_local0, w.pt_local0, hint0, avg_data, use_flat);

  double sup_local1 = 0;
  int hint1 = 0;
  if (!use_flat)
  {
    hint1 = hull.getHint1();
    avg_data.last_dir = hull.getSupportData1().last_dir;
  }
  GetAverageSupport(underlying, normal_local1, sup_local1, w.pt_local1, hint1, avg_data, use_flat);

  // Compare world-frame supports at the LINK origin as reference center,
  // matching Bullet's compound-child treatment:
  //
  //   link_sup = sup_local + normal · link_origin
  //
  // Using the link origin (not the per-shape world center) avoids orbital-
  // motion bias: when a multi-shape link rotates, each per-shape center
  // orbits the link origin, adding a spurious translational term
  // (normal · link_R * local_offset) to the comparison that differs between
  // sub-shapes even though the link undergoes the same motion.  Bullet
  // avoids this by building compound-child cast transforms at link level
  // (no per-shape translation component); we replicate that here.
  //
  // For pure rotation: link_origin0 == link_origin1, so the dot-product
  // terms cancel and the comparison reduces to sup_local1 vs sup_local0.
  //
  // For pure translation: sup_local0 == sup_local1 (same rotation ⇒ same
  // local normal), so link_sup1 − link_sup0 = normal · (link_origin1 − link_origin0).
  // A positive value means the shape's surface advances in the normal
  // direction over the sweep → CCType_Time1, matching Bullet.
  const double link_sup0 = sup_local0 + normal.dot(link_origin0);
  const double link_sup1 = sup_local1 + normal.dot(link_origin1);

  if (link_sup0 - link_sup1 > COAL_SUPPORT_FUNC_TOLERANCE)
  {
    w.cc_time = 0;
    w.cc_type = ContinuousCollisionType::CCType_Time0;
  }
  else if (link_sup1 - link_sup0 > COAL_SUPPORT_FUNC_TOLERANCE)
  {
    w.cc_time = 1;
    w.cc_type = ContinuousCollisionType::CCType_Time1;
  }
  else
  {
    w.cc_type = ContinuousCollisionType::CCType_Between;

    // Compute cc_time from the ratio of distances between the GJK witness
    // point and the surface support points at t=0 and t=1, matching
    // Schulman et al. IJRR 2014, Eq. (17):
    //   α = ||p1 - p_swept|| / (||p1 - p_swept|| + ||p0 - p_swept||)
    // where α weights p0 (i.e., cc_time = 1-α = l0c/(l0c+l1c)).
    const Eigen::Vector3d shape_ptWorld0 = shape_tf0 * Eigen::Vector3d(w.pt_local0);
    const Eigen::Vector3d shape_ptWorld1 = shape_tf1 * Eigen::Vector3d(w.pt_local1);
    const double l0c = (witness - shape_ptWorld0).norm();
    const double l1c = (witness - shape_ptWorld1).norm();

    if (l0c + l1c < COAL_LENGTH_TOLERANCE)
      w.cc_time = 0.5;
    else
      w.cc_time = std::clamp(l0c / (l0c + l1c), 0.0, 1.0);
  }

  return w;
}

/**
 * @brief The contact point of a located witness, in the link's frame at the start of its sweep.
 *
 * A contact pinned to one end of the sweep reports that end's support point, taken back through the
 * link's start pose: the end-pose world point for a contact at time 1, so that the link's start pose times
 * the result is the world contact point, as Bullet's calculateContinuousData has it. A contact in between
 * reports the average of the two supports, which is a point of the shape.
 *
 * @param w The located witness: the contact's type and the shape's support point at either end of the sweep
 * @param link_tf0 The link's pose at the start of the sweep
 * @param shape_tf0 The shape's world pose at the start of the sweep
 * @param shape_tf1 The shape's world pose at the end of the sweep
 */
static Eigen::Vector3d sweepWitnessInLinkFrame(const SweepWitness& w,
                                               const Eigen::Isometry3d& link_tf0,
                                               const Eigen::Isometry3d& shape_tf0,
                                               const Eigen::Isometry3d& shape_tf1)
{
  switch (w.cc_type)
  {
    case ContinuousCollisionType::CCType_Time0:
      return applyInverse(link_tf0, shape_tf0 * Eigen::Vector3d(w.pt_local0));
    case ContinuousCollisionType::CCType_Time1:
      return applyInverse(link_tf0, shape_tf1 * Eigen::Vector3d(w.pt_local1));
    default:
    {
      const coal::Vec3s avg_pt_local = (w.pt_local0 + w.pt_local1) / 2.0;
      return applyInverse(link_tf0, shape_tf0 * Eigen::Vector3d(avg_pt_local));
    }
  }
}

static Eigen::Isometry3d toIsometry(const coal::Transform3s& tf)
{
  Eigen::Isometry3d out{ Eigen::Isometry3d::Identity() };
  out.linear() = tf.getRotation();
  out.translation() = Eigen::Vector3d(tf.getTranslation());
  return out;
}

/// Sweep @p hull, whose shape sits at @p shape_tf, through the world-frame @p motion. With
/// @p d_arc_compensation the hull is padded by the arc sagitta of the motion's turn.
static void writePairSweep(CastHullShape& hull,
                           const Eigen::Isometry3d& motion,
                           const coal::Transform3s& shape_tf,
                           bool d_arc_compensation)
{
  // No motion is the unswept state, which the conjugation below reaches only to rounding. Clearing the
  // sweep drops the radius with it.
  if (motion.matrix() == Eigen::Isometry3d::Identity().matrix())
  {
    hull.clearSweep();
    return;
  }

  const coal::Transform3s motion_tf(motion.linear(), motion.translation());
  const coal::Transform3s cast_tf = shape_tf.inverseTimes(motion_tf * shape_tf);

  // The hull serves every pair that sweeps its object, and partners that move alike ask for the same sweep.
  // The radius follows from the cast transform, so a hull that holds this one holds both.
  if (cast_tf == hull.getCastTransform())
    return;

  // Ahead of the cast transform: writing that one recomputes the bound, which reads the radius.
  if (d_arc_compensation)
    hull.setSweptSphereRadius(computeDArc(cast_tf, *hull.getUnderlyingShape(), computeDArcScalars(cast_tf)));

  hull.updateCastTransform(cast_tf);
}

/// The point of @p shape that a contact with outward normal @p normal_local rests on: its support along
/// the normal, averaged over the supporting face when there is one.
static coal::Vec3s restingPoint(const coal::ShapeBase& shape, const coal::Vec3s& normal_local, int hint, bool use_flat)
{
  coal::details::ShapeSupportData& scratch = averagingScratch();
  // There is no earlier direction to warm-start from, so the hint alone seeds the climb.
  scratch.last_dir.setZero();
  double support = 0;
  coal::Vec3s point;
  GetAverageSupport(&shape, normal_local, support, point, hint, scratch, use_flat);
  return point;
}

/// One query of a pair of two swept objects: which is held and which is swept, and by what motion.
struct PairSweep
{
  /// @p co1 and @p co2 are the pair in the order of its cache key, @p cow1 and @p cow2 their wrappers and
  /// @p entry its cache entry. @p pair_swapped tells whether a contact reports the key's first object in
  /// its second slot.
  PairSweep(const CollisionCacheEntry& entry,
            const coal::CollisionObject* co1,
            const coal::CollisionObject* co2,
            const CastCollisionObjectWrapper* cow1,
            const CastCollisionObjectWrapper* cow2,
            bool pair_swapped)
    : hull(entry.pair_hull)
    , held_object(entry.sweeps_first ? co2 : co1)
    , swept_object(entry.sweeps_first ? co1 : co2)
    , held(entry.sweeps_first ? cow2 : cow1)
    , swept(entry.sweeps_first ? cow1 : cow2)
    , held_slot((entry.sweeps_first != pair_swapped) ? 1U : 0U)
    , swept_slot(1U - held_slot)
    , motion(held->getSweepDisplacementInverse() * swept->getSweepDisplacement())
  {
  }

  const CastHullShape* hull;                 ///< The pair's hull: the swept shape under `motion`.
  const coal::CollisionObject* held_object;  ///< Collided as its plain shape, at its start pose.
  const coal::CollisionObject* swept_object;
  const CastCollisionObjectWrapper* held;
  const CastCollisionObjectWrapper* swept;
  std::size_t held_slot;  ///< The held link's index in a ContactResult.
  std::size_t swept_slot;
  /// The world-frame motion of the swept link over its sweep with that of the held link removed: where the
  /// swept link ends up in a world in which the held one never leaves its start pose. A link that is not
  /// swept has exactly the identity for a displacement, and a product with that is exact, so two such links
  /// give the exact identity. Links that move rigidly together give the identity to rounding.
  Eigen::Isometry3d motion;
};

/// The rigid motion that takes @p link from where its sweep starts to where it is a fraction @p t of the
/// way through it: the identity at 0 and the link's whole displacement at 1. In between, the position moves
/// on a line and the orientation along the shortest arc.
static Eigen::Isometry3d displacementAt(const CastCollisionObjectWrapper& link, double t)
{
  // A link that is not swept is exactly where it started, which the interpolation below reaches only to
  // rounding.
  if (t <= 0.0 || !link.isSwept())
    return Eigen::Isometry3d::Identity();

  const Eigen::Isometry3d& whole = link.getSweepDisplacement();
  if (t >= 1.0)
    return whole;

  // The turn made by then is that share of the whole displacement's turn; the translation is the one that
  // puts the link's origin on the line between its two positions.
  const Eigen::Vector3d origin0 = link.getCollisionObjectsTransform().translation();
  const Eigen::Vector3d origin1 = link.getSweepEndTransform().translation();
  Eigen::Isometry3d part{ Eigen::Isometry3d::Identity() };
  part.linear() = Eigen::Quaterniond::Identity().slerp(t, Eigen::Quaterniond(whole.linear())).toRotationMatrix();
  part.translation() = (1.0 - t) * origin0 + t * origin1 - part.linear() * origin0;
  return part;
}

/// sweepWitnessInLinkFrame for a shape that starts @p link's sweep at @p shape_tf0 and ends it where the
/// link's displacement takes it. A link that is not swept has exactly the identity for a displacement, so
/// its shape ends exactly where it starts.
static Eigen::Vector3d pairWitnessInLinkFrame(const SweepWitness& w,
                                              const CastCollisionObjectWrapper& link,
                                              const Eigen::Isometry3d& shape_tf0)
{
  const Eigen::Isometry3d& link_tf0 = link.getCollisionObjectsTransform();
  // Only a contact at the end of the sweep reads the shape's end pose.
  if (w.cc_type != ContinuousCollisionType::CCType_Time1)
    return sweepWitnessInLinkFrame(w, link_tf0, shape_tf0, shape_tf0);

  return sweepWitnessInLinkFrame(w, link_tf0, shape_tf0, link.getSweepDisplacement() * shape_tf0);
}

/**
 * @brief Populate the continuous collision fields of a contact between two swept objects.
 *
 * The pair was collided as the held object's plain shape against one hull, so the contact has one time,
 * which both links report. Each link's local contact point follows the conventions of a single sweep; see
 * sweepWitnessInLinkFrame.
 *
 * The query ran in a world where the held link never leaves its start pose. The world points and the
 * normal are carried to where that link is at the contact time; the swept link is there with it, to the
 * accuracy of the hull.
 *
 * @param held_hint The support vertex the narrowphase ended on for the held shape
 * @param use_flat Scan every vertex for a support instead of climbing from a hint
 */
static void populatePairSweepFields(ContactResult& contact, const PairSweep& pair, int held_hint, bool use_flat)
{
  contact.cc_transform[pair.held_slot] = pair.held->getSweepEndTransform();
  contact.cc_transform[pair.swept_slot] = pair.swept->getSweepEndTransform();

  // contact.normal points from slot 0 to slot 1.
  const coal::Vec3s swept_to_held = (pair.swept_slot == 0) ? coal::Vec3s(contact.normal) : coal::Vec3s(-contact.normal);

  // The swept shape's two poses in the frame the query ran in, where the held link never leaves its start
  // pose.
  const Eigen::Isometry3d& swept_tf0 = pair.swept->getCollisionObjectsTransform();
  const Eigen::Isometry3d swept_shape_tf0 = toIsometry(pair.swept_object->getTransform());
  const Eigen::Isometry3d swept_shape_tf1 = pair.motion * swept_shape_tf0;
  const SweepWitness swept_witness = locateOnSweep(*pair.hull,
                                                   swept_shape_tf0,
                                                   swept_shape_tf1,
                                                   swept_tf0.translation(),
                                                   pair.motion * swept_tf0.translation(),
                                                   swept_to_held,
                                                   contact.nearest_points[pair.swept_slot],
                                                   use_flat);

  contact.cc_time = { swept_witness.cc_time, swept_witness.cc_time };
  contact.cc_type = { swept_witness.cc_type, swept_witness.cc_type };

  // The local point conventions are stated against each shape's own end pose in the world, not against the
  // relative one the query used.
  contact.nearest_points_local[pair.swept_slot] = pairWitnessInLinkFrame(swept_witness, *pair.swept, swept_shape_tf0);

  // The held shape is not swept, so it rests on one point throughout. A contact at the end of the sweep
  // reports that point where the link's end pose puts it, as for a swept shape.
  const Eigen::Isometry3d held_shape_tf0 = toIsometry(pair.held_object->getTransform());
  const auto* held_hull = static_cast<const CastHullShape*>(pair.held_object->collisionGeometryPtr());
  const coal::Vec3s held_pt_local = restingPoint(*held_hull->getUnderlyingShape(),
                                                 held_shape_tf0.linear().transpose() * coal::Vec3s(-swept_to_held),
                                                 held_hint,
                                                 use_flat);
  Eigen::Vector3d held_point = held_shape_tf0 * Eigen::Vector3d(held_pt_local);
  if (swept_witness.cc_type == ContinuousCollisionType::CCType_Time1)
    held_point = pair.held->getSweepDisplacement() * held_point;
  contact.nearest_points_local[pair.held_slot] = applyInverse(pair.held->getCollisionObjectsTransform(), held_point);

  const Eigen::Isometry3d carry = displacementAt(*pair.held, swept_witness.cc_time);
  contact.nearest_points[0] = carry * contact.nearest_points[0];
  contact.nearest_points[1] = carry * contact.nearest_points[1];
  contact.normal = carry.linear() * contact.normal;
}

/**
 * @brief Populate continuous collision fields (cc_time, cc_type, cc_transform) on a ContactResult.
 *
 * Uses support-function-based approach matching Bullet's calculateContinuousData:
 * finds the shape's extreme points along the contact normal at t=0 and t=1, then
 * classifies the collision time based on which pose has greater support.
 *
 * @p cow1 and @p cow2 are the wrappers of @p o1 and @p o2.
 *
 * Only kinematic objects carry CastHullShape geometry: a static link is collided through its regular
 * wrapper, and its cast wrapper joins no broadphase, so it never surfaces here as o1/o2. The dynamic_cast
 * below is what tells a swept object from the regular one it may be paired with, and the wrapper is read
 * as a cast wrapper only behind it.
 */
void populateContinuousCollisionFields(ContactResult& contact,
                                       const coal::CollisionObject* o1,
                                       const coal::CollisionObject* o2,
                                       const CollisionObjectWrapperBase& cow1,
                                       const CollisionObjectWrapperBase& cow2,
                                       bool use_flat)
{
  const std::array<const coal::CollisionObject*, 2> objects = { o1, o2 };
  const std::array<const CollisionObjectWrapperBase*, 2> cows = { &cow1, &cow2 };
  for (std::size_t i = 0; i < 2; ++i)
  {
    const CollisionObjectWrapperBase* cow = cows[i];
    if (isStatic(*cow))
      continue;

    const auto* cast_shape = dynamic_cast<const CastHullShape*>(objects[i]->collisionGeometryPtr());
    if (cast_shape == nullptr)
      continue;

    // Shape world transforms at t=0 and t=1
    const coal::Transform3s& tf_world0 = objects[i]->getTransform();
    const Eigen::Isometry3d shape_tf0 = toIsometry(tf_world0);
    const Eigen::Isometry3d shape_tf1 = toIsometry(tf_world0 * cast_shape->getCastTransform());

    // The link's pose at t=1 is the one its sweep was set with. The hulls hold it per shape and to
    // rounding; the wrapper holds it as given. Only a cast wrapper owns a CastHullShape -
    // makeCastCollisionObject is the one place a collision object is given one - so the downcast is safe
    // here.
    contact.cc_transform[i] = static_cast<const CastCollisionObjectWrapper*>(cow)->getSweepEndTransform();

    // Normal pointing from current object toward the other (matching Bullet convention).
    // contact.normal has already been remapped to original (o1, o2) order after pair normalization.
    const coal::Vec3s normal_world = (i == 0) ? coal::Vec3s(contact.normal) : coal::Vec3s(-contact.normal);

    const SweepWitness w = locateOnSweep(*cast_shape,
                                         shape_tf0,
                                         shape_tf1,
                                         contact.transform[i].translation(),
                                         contact.cc_transform[i].translation(),
                                         normal_world,
                                         contact.nearest_points[i],
                                         use_flat);
    contact.cc_time[i] = w.cc_time;
    contact.cc_type[i] = w.cc_type;
    contact.nearest_points_local[i] =
        sweepWitnessInLinkFrame(w, cow->getCollisionObjectsTransform(), shape_tf0, shape_tf1);
  }
}

int getReportedSubshapeIndex(const coal::CollisionObject* object, int coal_subshape_index)
{
  const auto* collision_object = static_cast<const CoalCollisionObjectWrapper*>(object);
  const int source_subshape_index = collision_object->getSourceSubshapeIndex();
  if (source_subshape_index >= 0)
    return source_subshape_index;

  // This backend collides against the whole tree rather than against per-voxel objects, so Coal
  // names the node that was hit instead of reporting the occupied-leaf ordinal that an expanding
  // backend reports. That value identifies the node within the tree and is stable while the node
  // lives, which is what consumers group by, but it is not an index and must not be used to look a
  // voxel up.
  //
  // Coal derives the value from the node address and can hand back a negative one, which a
  // ContactResult reads as unset. Clearing the sign bit folds it onto the non-negative half, which
  // preserves node identity for the magnitudes Coal produces. Keep the fold scoped to octrees:
  // folding an unset value instead fabricates a large index.
  //
  // The branch above must stay ahead of this one: an object that carries a real subshape index
  // reports it whatever its geometry type is.
  if (object->getNodeType() == coal::GEOM_OCTREE)
    return coal_subshape_index & 0x7FFFFFFF;

  // Coal reports Contact::NONE for a shape that has no subshapes, and -1 is how a ContactResult
  // spells unset. The two constants agree today; do not depend on that.
  return (coal_subshape_index < 0) ? -1 : coal_subshape_index;
}

/// True when a geometry is traversed as a tree of leaves rather than collided as a single convex
/// body, so that one query runs many narrowphase calls against it. Octrees, meshes and height
/// fields all traverse; only a shape does not. An unrecognised geometry is reported as traversing,
/// which costs it a warm start it may not have needed but cannot leave its leaves sharing a seed.
static bool isMultiLeaf(const coal::CollisionGeometry& geometry) { return geometry.getObjectType() != coal::OT_GEOM; }

bool pairSweepsFirst(coal::Scalar size1, coal::Scalar size2, const std::string& link1, const std::string& link2)
{
  return size1 < size2 || (!(size2 < size1) && link1 > link2);
}

/// Build the cache entry of a collision object pair. @p cow1 and @p cow2 are the wrappers of @p co1 and
/// @p co2.
static CollisionCacheEntry makeCacheEntry(const coal::CollisionObject& co1,
                                          const coal::CollisionObject& co2,
                                          const CollisionObjectWrapperBase& cow1,
                                          const CollisionObjectWrapperBase& cow2)
{
  const auto* hull1 = dynamic_cast<const CastHullShape*>(co1.collisionGeometryPtr());
  const auto* hull2 = dynamic_cast<const CastHullShape*>(co2.collisionGeometryPtr());

  const coal::CollisionGeometry* geometry1 = co1.collisionGeometryPtr();
  const coal::CollisionGeometry* geometry2 = co2.collisionGeometryPtr();
  CastHullShape* pair_hull = nullptr;
  bool sweeps_first = false;
  if (hull1 != nullptr && hull2 != nullptr)
  {
    // Two swept objects are collided as the plain shape of one against a hull of the other's motion relative
    // to it; CoalCastBVHManager states why. Which one is swept does not follow the cache key, whose order is
    // that of two addresses.
    sweeps_first = pairSweepsFirst(
        hull1->getShapeBoundRadius(), hull2->getShapeBoundRadius(), cow1.getLinkId().name(), cow2.getLinkId().name());
    pair_hull = &(sweeps_first ? hull1 : hull2)->scratchHull();
    const coal::ShapeBase* held_shape = (sweeps_first ? hull2 : hull1)->getUnderlyingShape().get();
    geometry1 = sweeps_first ? static_cast<const coal::CollisionGeometry*>(pair_hull) : held_shape;
    geometry2 = sweeps_first ? held_shape : static_cast<const coal::CollisionGeometry*>(pair_hull);
  }

  coal::CollisionRequest col_request;
  // NesterovAcceleration + DualityGap/Relative for both cast and discrete pairs.
  // PolyakAcceleration fails cast sphere-sphere contact accuracy (compared to Bullet) regardless of
  // criterion. DualityGap/Absolute with Nesterov misses collisions on CastHullShape.
  col_request.gjk_variant = coal::GJKVariant::NesterovAcceleration;
  col_request.gjk_convergence_criterion = coal::GJKConvergenceCriterion::DualityGap;
  col_request.gjk_convergence_criterion_type = coal::GJKConvergenceCriterionType::Relative;
  // Stated here rather than left to the staleness branch of CollisionCallback::collide, so that a pair's
  // guess mode does not depend on that branch having run. Coal constructs a request with a single constant
  // direction, which every leaf of a multi-leaf pair would then share.
  col_request.gjk_initial_guess = coal::BoundingVolumeGuess;

  CollisionCacheEntry entry{ std::move(col_request),
                             coal::ComputeCollision(geometry1, geometry2),
                             hull1 != nullptr || hull2 != nullptr,
                             isMultiLeaf(*geometry1) || isMultiLeaf(*geometry2) };
  entry.pair_hull = pair_hull;
  entry.sweeps_first = sweeps_first;
  entry.first_geometry = geometry1;
  return entry;
}

bool CollisionCallback::collide(coal::CollisionObject* o1, coal::CollisionObject* o2)
{
  if (cdata->done)
    return true;

  const auto* cd1 = static_cast<const CollisionObjectWrapperBase*>(o1->getUserData());
  const auto* cd2 = static_cast<const CollisionObjectWrapperBase*>(o2->getUserData());

  link_pair.assign(cd1->getLinkId(), cd2->getLinkId());

  if (!needsCollisionCheck(cd1, cd2, link_pair, cdata->validator))
    return false;

  std::size_t num_contacts = (cdata->req.contact_limit > 0) ? static_cast<std::size_t>(cdata->req.contact_limit) :
                                                              std::numeric_limits<std::size_t>::max();
  if (cdata->req.type == ContactTestType::FIRST)
    num_contacts = 1;
  const auto security_margin = cdata->collision_margin_data.getCollisionMargin(link_pair);

  // Normalize pair ordering for consistent cache lookups: Coal's broadphase
  // does not guarantee a stable (o1, o2) ordering across tree rebalances,
  // so always put the smaller pointer first to avoid duplicate cache entries.
  const bool pair_swapped = std::greater<>{}(o1, o2);
  auto* co1 = pair_swapped ? o2 : o1;
  auto* co2 = pair_swapped ? o1 : o2;
  // The wrappers in the cache key's order.
  const auto* cow1 = pair_swapped ? cd2 : cd1;
  const auto* cow2 = pair_swapped ? cd1 : cd2;
  CollisionObjectPair object_pair = std::make_pair(co1, co2);
  auto col_cache_it = cdata->collision_cache->find(object_pair);

  if (col_cache_it == cdata->collision_cache->end())
    col_cache_it = cdata->collision_cache->try_emplace(object_pair, makeCacheEntry(*co1, *co2, *cow1, *cow2)).first;

  auto& entry = col_cache_it->second;
  auto& cached_request = entry.request;

  // Drop the warm-start seed when a COW generation changes (transform or enable/disable), so the
  // next check re-derives the guess from the current bounding volumes rather than reusing one
  // aimed at the previous poses.
  if (entry.gen0 != cow1->gjk_generation_ || entry.gen1 != cow2->gjk_generation_)
  {
    // A swept pair's deepest-point set degenerates to a face whose long axis is the sweep
    // direction, so the witness EPA returns -- and with it cc_time -- is seed-dependent whatever
    // the seed; depth, normal and the witness separation are not. The seed must therefore be
    // derived from the bounding volumes, which share the geometry's symmetries: a seed taken from
    // the pair's centre-to-centre direction does not, and on a shape penetrating several
    // symmetrically placed octree voxels it yields a normal set that is not mirror-symmetric
    // either, with two voxels reporting the same normal.
    cached_request.gjk_initial_guess = coal::BoundingVolumeGuess;

    entry.gen0 = cow1->gjk_generation_;
    entry.gen1 = cow2->gjk_generation_;
  }

  cached_request.enable_contact = cdata->req.calculate_penetration;
  cached_request.num_max_contacts = num_contacts;
  cached_request.security_margin = security_margin;
  cached_request.distance_upper_bound = security_margin + cached_request.gjk_tolerance;

  // Every pair but one of two swept objects pays this test and nothing else.
  std::optional<PairSweep> pair;
  if (entry.pair_hull != nullptr)
  {
    // Both objects hold a CastHullShape, which only a cast wrapper owns: makeCastCollisionObject is the one
    // place a collision object is given one.
    pair.emplace(entry,
                 co1,
                 co2,
                 static_cast<const CastCollisionObjectWrapper*>(cow1),
                 static_cast<const CastCollisionObjectWrapper*>(cow2),
                 pair_swapped);
    writePairSweep(
        *entry.pair_hull, pair->motion, pair->swept_object->getTransform(), pair->swept->getDArcCompensation());
  }

  // Both objects sit at their start poses: a swept pair's motion is all in its hull.
  coal::CollisionResult col_result;
  entry.functor(co1->getTransform(), co2->getTransform(), cached_request, col_result);

  // Warm-start: cache the GJK/EPA result for the next call on this pair.
  // The cached separating direction (NoCollision) or penetration vector (EPA)
  // is a better seed than recomputing from geometry each time.
  // Cached guesses are only updated if gjk_initial_guess == CachedGuess. Every first
  // collision check uses BoundingVolumeGuess, so we have to update manually.
  //
  // A multi-leaf pair keeps its bounding-volume guess instead. One direction is cached per pair
  // but a query runs a separate GJK per leaf, so the value Coal returns is residue from whichever
  // leaf was visited last, and reusing it seeds every leaf of the next query from that one
  // direction. Skipping the support-function copy costs such a pair nothing: that hint is
  // warm-started in every mode, from the solver held by the cached functor.
  if (!entry.multi_leaf)
  {
    cached_request.gjk_initial_guess = coal::CachedGuess;
    cached_request.cached_gjk_guess = col_result.cached_gjk_guess;
    cached_request.cached_support_func_guess = col_result.cached_support_func_guess;
  }

  if (!col_result.isCollision())
    return false;

  // Some Coal traversal nodes (e.g., ShapeOcTreeCollisionTraversalNode) internally
  // swap arguments without compensating in the result, causing Contact o1/o2, b1/b2,
  // nearest_points, and normal to not match the (co1, co2) ordering. Detect this by
  // checking if the first contact's o1 is the geometry the functor collides as its first
  // object, which for a pair of two swept objects is not the one co1 holds.
  if (col_result.getContact(0).o1 != entry.first_geometry)
    col_result.swapObjects();

  const Eigen::Isometry3d& tf1 = cd1->getCollisionObjectsTransform();
  const Eigen::Isometry3d& tf2 = cd2->getCollisionObjectsTransform();

  // Coal result fields are in normalized (co1, co2) order; map back to original (o1, o2).
  const std::size_t idx0 = pair_swapped ? 1U : 0U;
  const std::size_t idx1 = pair_swapped ? 0U : 1U;

  // With penetration disabled, GJK early-outs without EPA, so its shared
  // warm-start seed is poorly aimed for support averaging; use the flat scan.
  const bool use_flat = !cdata->req.calculate_penetration;

  bool found = false;
  for (size_t i = 0; i < col_result.numContacts(); ++i)
  {
    const coal::Contact& coal_contact = col_result.getContact(i);
    ContactResult contact;
    contact.link_ids[0] = cd1->getLinkId();
    contact.link_ids[1] = cd2->getLinkId();
    contact.shape_id[0] = CollisionObjectWrapperBase::getShapeIndex(o1);
    contact.shape_id[1] = CollisionObjectWrapperBase::getShapeIndex(o2);
    contact.subshape_id[0] =
        getReportedSubshapeIndex(o1, static_cast<int>(pair_swapped ? coal_contact.b2 : coal_contact.b1));
    contact.subshape_id[1] =
        getReportedSubshapeIndex(o2, static_cast<int>(pair_swapped ? coal_contact.b1 : coal_contact.b2));
    contact.nearest_points[0] = coal_contact.nearest_points[idx0];
    contact.nearest_points[1] = coal_contact.nearest_points[idx1];
    contact.nearest_points_local[0] = applyInverse(tf1, contact.nearest_points[0]);
    contact.nearest_points_local[1] = applyInverse(tf2, contact.nearest_points[1]);
    contact.transform[0] = tf1;
    contact.transform[1] = tf2;
    contact.type_id[0] = cd1->getTypeID();
    contact.type_id[1] = cd2->getTypeID();
    contact.distance = coal_contact.penetration_depth;
    contact.normal = pair_swapped ? coal::Vec3s(-coal_contact.normal) : coal_contact.normal;

    if (pair)
    {
      // The functor's objects are in the cache key's order, so the held one is its second when the first is
      // swept.
      const int held_hint = col_result.cached_support_func_guess[entry.sweeps_first ? 1 : 0];
      populatePairSweepFields(contact, *pair, held_hint, use_flat);
    }
    else if (entry.is_cast)
    {
      populateContinuousCollisionFields(contact, o1, o2, *cd1, *cd2, use_flat);
    }

    if (!found)
    {
      const auto it = cdata->res->find(link_pair);
      found = (it != cdata->res->end() && !it->second.empty());
    }
    processResult(*cdata, contact, link_pair, security_margin, found);
  }

  return cdata->done;
}

CollisionObjectWrapperBase::CollisionObjectWrapperBase(tesseract::common::LinkId&& id,
                                                       const int& type_id,
                                                       CollisionShapesConst&& shapes,
                                                       tesseract::common::VectorIsometry3d&& shape_poses)
  : link_id_(std::move(id)), type_id_(type_id), shapes_(std::move(shapes)), shape_poses_(std::move(shape_poses))
{
  // Preconditions guaranteed by createCoalCollisionObject() which validates before construction.
  assert(!shapes_.empty());                       // NOLINT
  assert(!shape_poses_.empty());                  // NOLINT
  assert(!link_id_.name().empty());               // NOLINT
  assert(shapes_.size() == shape_poses_.size());  // NOLINT

  collision_objects_.reserve(shapes_.size());
  // createShapePrimitive has already bounded each geometry below, and those geometries are shared
  // between collision objects and between environments, so recomputing here would write through
  // geometry another object may be reading.
  for (std::size_t i = 0; i < shapes_.size(); ++i)  // NOLINT
  {
    if (shapes_[i]->getType() == tesseract::geometry::GeometryType::COMPOUND_MESH)
    {
      const auto& meshes = std::static_pointer_cast<const tesseract::geometry::CompoundMesh>(shapes_[i])->getMeshes();
      int subshape_index = 0;
      for (const auto& mesh : meshes)
      {
        const CollisionGeometryPtr subshape = createShapePrimitive(mesh);
        if (subshape != nullptr)
        {
          auto co = std::make_shared<CoalCollisionObjectWrapper>(subshape, /*compute_local_aabb=*/false);
          co->setUserData(this);
          co->setShapeIndex(static_cast<int>(i));
          co->setSourceShapeIndex(static_cast<int>(i));
          co->setSourceSubshapeIndex(subshape_index++);
          co->setTransform(coal::Transform3s(shape_poses_[i].rotation(), shape_poses_[i].translation()));
          co->updateAABB();
          collision_objects_.push_back(co);
        }
      }
    }
    else
    {
      const CollisionGeometryPtr subshape = createShapePrimitive(shapes_[i]);
      if (subshape != nullptr)
      {
        auto co = std::make_shared<CoalCollisionObjectWrapper>(subshape, /*compute_local_aabb=*/false);
        co->setUserData(this);
        co->setShapeIndex(static_cast<int>(i));
        co->setSourceShapeIndex(static_cast<int>(i));
        co->setTransform(coal::Transform3s(shape_poses_[i].rotation(), shape_poses_[i].translation()));
        co->updateAABB();
        collision_objects_.push_back(co);
      }
    }
  }
}

int CollisionObjectWrapperBase::getShapeIndex(const coal::CollisionObject* co)
{
  return static_cast<const CoalCollisionObjectWrapper*>(co)->getSourceShapeIndex();
}

void CollisionObjectWrapperBase::setCollisionObjectsTransform(const Eigen::Isometry3d& pose)
{
  world_pose_ = pose;
  for (auto& co : collision_objects_)
  {
    const auto& local = shape_poses_[static_cast<std::size_t>(co->getShapeIndex())];
    if (local.linear().isIdentity())
    {
      co->setTransform(coal::Transform3s(pose.linear(), pose * local.translation()));
    }
    else
    {
      auto tf = pose * local;
      co->setTransform(coal::Transform3s(tf.linear(), tf.translation()));
    }
    co->updateAABB();  // This a tesseract function that updates the aabb to take into account contact distance
  }
}

void CollisionObjectWrapperBase::setContactDistanceThreshold(double contact_distance)
{
  contact_distance_ = contact_distance;
  for (auto& co : collision_objects_)
    co->setContactDistanceThreshold(contact_distance_);
}

void CollisionObjectWrapperBase::appendCollisionObjectsRaw(std::vector<CollisionObjectRawPtr>& out) const
{
  for (const auto& co : collision_objects_)
    out.push_back(co.get());
}

std::shared_ptr<CollisionObjectWrapper> CollisionObjectWrapper::clone() const
{
  auto clone_cow = std::make_shared<CollisionObjectWrapper>();
  clone_cow->cloneFrom(*this);
  return clone_cow;
}

void CollisionObjectWrapperBase::cloneFrom(const CollisionObjectWrapper& other)
{
  link_id_ = other.link_id_;
  type_id_ = other.type_id_;
  shapes_ = other.shapes_;
  shape_poses_ = other.shape_poses_;

  collision_objects_.reserve(other.collision_objects_.size());
  for (const auto& co : other.collision_objects_)
  {
    assert(std::dynamic_pointer_cast<CoalCollisionObjectWrapper>(co) != nullptr);
    auto collObj =
        std::make_shared<CoalCollisionObjectWrapper>(*std::static_pointer_cast<CoalCollisionObjectWrapper>(co));
    collObj->setUserData(this);
    collObj->setTransform(co->getTransform());
    collObj->updateAABB();
    collision_objects_.push_back(collObj);
  }

  world_pose_ = other.world_pose_;
  contact_distance_ = other.contact_distance_;
  m_collisionFilterGroup = other.m_collisionFilterGroup;
  m_collisionFilterMask = other.m_collisionFilterMask;
  m_enabled = other.m_enabled;
}

CastCollisionObjectWrapper::CastCollisionObjectWrapper(const CollisionObjectWrapper& link, bool d_arc_compensation)
  : d_arc_compensation_(d_arc_compensation)
{
  cloneFrom(link);
}

/// Add the raw pointers of @p objects to @p ptrs for O(1) membership tests.
static void addPointers(std::unordered_set<const coal::CollisionObject*>& ptrs,
                        const std::vector<CollisionObjectPtr>& objects)
{
  ptrs.reserve(ptrs.size() + objects.size());
  for (const auto& co : objects)
    ptrs.insert(co.get());
}

/// Erase every cache entry keyed on a pointer in @p ptrs.
static void eraseCacheEntries(CollisionCacheMap& cache, const std::unordered_set<const coal::CollisionObject*>& ptrs)
{
  for (auto it = cache.begin(); it != cache.end();)
  {
    if (ptrs.count(it->first.first) != 0 || ptrs.count(it->first.second) != 0)
      it = cache.erase(it);
    else
      ++it;
  }
}

void invalidateCacheFor(CollisionCacheMap& cache, const std::vector<CollisionObjectPtr>& objects)
{
  std::unordered_set<const coal::CollisionObject*> ptrs;
  addPointers(ptrs, objects);
  eraseCacheEntries(cache, ptrs);
}

void invalidateCacheFor(CollisionCacheMap& cache,
                        const std::vector<CollisionObjectPtr>& objects,
                        const std::vector<CollisionObjectPtr>& other_objects)
{
  std::unordered_set<const coal::CollisionObject*> ptrs;
  addPointers(ptrs, objects);
  addPointers(ptrs, other_objects);
  eraseCacheEntries(cache, ptrs);
}

void unregisterObjects(const std::vector<CollisionObjectPtr>& objects, coal::BroadPhaseCollisionManager& manager)
{
  for (const auto& co : objects)
    manager.unregisterObject(co.get());
}

void removeObjects(CollisionCacheMap& cache,
                   const std::vector<CollisionObjectPtr>& objects,
                   coal::BroadPhaseCollisionManager& manager)
{
  unregisterObjects(objects, manager);
  invalidateCacheFor(cache, objects);
}

COW::Ptr createCoalCollisionObject(const tesseract::common::LinkId& id,
                                   const int& type_id,
                                   const CollisionShapesConst& shapes,
                                   const tesseract::common::VectorIsometry3d& shape_poses,
                                   bool enabled)
{
  // dont add object that does not have geometry
  if (shapes.empty() || shape_poses.empty() || (shapes.size() != shape_poses.size()))
  {
    TESSERACT_LOG_DEBUG("ignoring link {}", id.name());
    return nullptr;
  }

  auto new_cow = std::make_shared<COW>(id, type_id, shapes, shape_poses);

  new_cow->m_enabled = enabled;
  // TESSERACT_LOG_DEBUG("Created collision object for link {}", new_cow->getLinkId().name());
  return new_cow;
}

bool buildCoalCollisionObjects(const std::vector<CollisionObjectSpec>& objects, std::vector<COW::Ptr>& cows)
{
  cows.clear();
  cows.reserve(objects.size());

  std::unordered_map<tesseract::common::LinkId, std::size_t> batch_index;
  batch_index.reserve(objects.size());

  bool success{ true };
  for (const auto& obj : objects)
  {
    // Drop any earlier wrapper for this id, so the last spec naming it decides both the wrapper and the
    // position. The hole is compacted away below rather than erased here, which would be O(n) per repeat.
    const auto it = batch_index.find(obj.id);
    if (it != batch_index.end())
    {
      cows[it->second] = nullptr;
      batch_index.erase(it);
    }

    const COW::Ptr new_cow = createCoalCollisionObject(obj.id, obj.mask_id, obj.shapes, obj.shape_poses, obj.enabled);
    if (new_cow == nullptr)
    {
      success = false;
      continue;
    }

    batch_index[obj.id] = cows.size();
    cows.push_back(new_cow);
  }

  cows.erase(std::remove(cows.begin(), cows.end(), nullptr), cows.end());

  return success;
}

bool isStatic(const CollisionObjectWrapperBase& cow)
{
  return cow.m_collisionFilterGroup == CollisionFilterGroups::StaticFilter;
}

bool isKinematic(const CollisionObjectWrapperBase& cow) { return !isStatic(cow); }

void applyCollisionFilterMask(CollisionObjectWrapperBase& cow)
{
  if (isStatic(cow))
    cow.m_collisionFilterMask = CollisionFilterGroups::KinematicFilter;
  else
    cow.m_collisionFilterMask = CollisionFilterGroups::StaticFilter | CollisionFilterGroups::KinematicFilter;
}

bool applyCollisionMarginThreshold(CollisionObjectWrapperBase& cow,
                                   const tesseract::common::CollisionMarginData& margin_data)
{
  const double margin = margin_data.getMaxCollisionMargin(cow.getLinkId());
  if (margin == cow.getContactDistanceThreshold())
    return false;

  cow.setContactDistanceThreshold(margin);
  return true;
}

void updateCollisionObjectFilters(const std::unordered_set<tesseract::common::LinkId>& active_ids,
                                  const COW::Ptr& cow,
                                  const std::unique_ptr<coal::BroadPhaseCollisionManager>& static_manager,
                                  const std::unique_ptr<coal::BroadPhaseCollisionManager>& dynamic_manager)
{
  // For discrete checks we can check static to kinematic and kinematic to
  // kinematic
  if (!isLinkActive(active_ids, cow->getLinkId()))
  {
    if (isKinematic(*cow))
    {
      const std::vector<CollisionObjectPtr>& objects = cow->getCollisionObjects();
      // This link was dynamic but is now static
      for (const auto& co : objects)
        dynamic_manager->unregisterObject(co.get());

      for (const auto& co : objects)
        static_manager->registerObject(co.get());
    }
    cow->m_collisionFilterGroup = CollisionFilterGroups::StaticFilter;
  }
  else
  {
    if (isStatic(*cow))
    {
      const std::vector<CollisionObjectPtr>& objects = cow->getCollisionObjects();
      // This link was static but is now dynamic
      for (const auto& co : objects)
        static_manager->unregisterObject(co.get());

      for (const auto& co : objects)
        dynamic_manager->registerObject(co.get());
    }
    cow->m_collisionFilterGroup = CollisionFilterGroups::KinematicFilter;
  }

  applyCollisionFilterMask(*cow);
}

void updateCollisionObjectFilters(const std::unordered_set<tesseract::common::LinkId>& active_ids,
                                  const COW::Ptr& cow,
                                  CastCOW::Ptr& cast_cow,
                                  const std::unique_ptr<coal::BroadPhaseCollisionManager>& static_manager,
                                  const std::unique_ptr<coal::BroadPhaseCollisionManager>& dynamic_manager)
{
  const std::vector<CollisionObjectPtr>& reg_objects = cow->getCollisionObjects();

  if (!isLinkActive(active_ids, cow->getLinkId()))
  {
    if (isKinematic(*cow))
    {
      // This link was dynamic but is now static: unregister cast from dynamic, register raw in static.
      for (const auto& co : cast_cow->getCollisionObjects())
        dynamic_manager->unregisterObject(co.get());

      for (const auto& co : reg_objects)
        static_manager->registerObject(co.get());
    }
    cow->m_collisionFilterGroup = CollisionFilterGroups::StaticFilter;
    cast_cow->m_collisionFilterGroup = CollisionFilterGroups::StaticFilter;
  }
  else
  {
    if (isStatic(*cow))
    {
      // Static -> kinematic: build the deferred cast shapes, then swap broadphases.
      //
      // The build replaces the wrapper, dropping the deferred objects. Nothing has to invalidate the
      // narrowphase cache for them even though it is keyed on raw collision object addresses, which a
      // later allocation can reuse: a deferred wrapper is registered in no broadphase, so no contact test
      // ever reaches its objects and no entry is ever keyed on one. Registering one would break that.
      if (castCowNeedsSweptBuild(*cast_cow))
        cast_cow = makeCastCollisionObject(cow, /*build_swept=*/true, cast_cow->getDArcCompensation());

      // Set unswept at the link's pose: a static link's cast wrapper is not kept current.
      const Eigen::Isometry3d& pose = cow->getCollisionObjectsTransform();
      cast_cow->setSweep(pose, pose);

      // That write invalidates any cached GJK guess held against this wrapper: setActiveCollisionObjects
      // keeps the narrowphase cache, whose entries are re-seeded only when a generation changes, and the
      // guesses in it were formed against the pose and sweep the wrapper carried before it went static.
      cast_cow->gjk_generation_++;

      for (const auto& co : reg_objects)
        static_manager->unregisterObject(co.get());

      for (const auto& co : cast_cow->getCollisionObjects())
        dynamic_manager->registerObject(co.get());
    }
    cow->m_collisionFilterGroup = CollisionFilterGroups::KinematicFilter;
    cast_cow->m_collisionFilterGroup = CollisionFilterGroups::KinematicFilter;
  }

  applyCollisionFilterMask(*cow);
  applyCollisionFilterMask(*cast_cow);
}

bool CastCollisionObjectWrapper::setSweep(const Eigen::Isometry3d& pose1, const Eigen::Isometry3d& pose2)
{
  assert(!castCowNeedsSweptBuild(*this));

  bool changed = false;

  // A zero-length sweep is the unswept state, which every hull resolves to regardless of its local offset,
  // so it is clearSweep's business rather than a per-shape product - and the products would not reach it
  // exactly anyway, since (tf * local)^-1 * (tf * local) leaves rounding noise that defeats the equality
  // test below.
  //
  // The comparison must be exact because it stands in for that computation: whatever it accepts has to
  // produce the identity, and a relative tolerance accepts real motion far from the origin.
  if (pose1.matrix() == pose2.matrix())
  {
    for (const auto& co : collision_objects_)
      changed = static_cast<CastHullShape*>(co->collisionGeometryPtr())->clearSweep() || changed;
  }
  else
  {
    const coal::Transform3s tf1(pose1.rotation(), pose1.translation());
    const coal::Transform3s tf2(pose2.rotation(), pose2.translation());

    // Precompute rotation-angle scalars once per link (conjugation-invariant).
    DArcScalars d_arc_scalars;
    if (d_arc_compensation_)
      d_arc_scalars = computeDArcScalars(tf1.inverseTimes(tf2));

    // Update cast transforms so computeLocalAABB reflects the swept volume.
    for (const auto& co : collision_objects_)
    {
      auto* cast_shape = static_cast<CastHullShape*>(co->collisionGeometryPtr());
      assert(cast_shape != nullptr);

      // Compute per-shape relative transform accounting for local offset.
      // Each shape's world transform is link_tf * local_tf, so the relative
      // motion in the shape's local frame is:
      //   (tf1 * local_tf)^-1 * (tf2 * local_tf)
      // This matches Bullet's compound shape handling where each child gets
      // its own delta_tf = (tf1 * local_tf).inverseTimes(tf2 * local_tf).
      const auto& shape_pose = shape_poses_[static_cast<std::size_t>(co->getShapeIndex())];
      const auto local_tf = coal::Transform3s(shape_pose.rotation(), shape_pose.translation());
      const coal::Transform3s new_cast_tf = (tf1 * local_tf).inverseTimes(tf2 * local_tf);

      const auto& cur_cast_tf = cast_shape->getCastTransform();
      if (new_cast_tf == cur_cast_tf)
        continue;

      changed = true;
      if (d_arc_compensation_)
        cast_shape->setSweptSphereRadius(computeDArc(new_cast_tf, *cast_shape->getUnderlyingShape(), d_arc_scalars));
      cast_shape->updateCastTransform(new_cast_tf);
    }
  }

  // Taken ahead of the pose write: a caller may pass this wrapper's own poses, which that write changes.
  // NOLINTNEXTLINE(performance-unnecessary-copy-initialization)
  const Eigen::Isometry3d end_pose = pose2;

  // After the hulls, and even when the link has not moved: this recomputes each object's AABB from its
  // hull's local one, and a broadphase update copies that AABB rather than deriving it.
  setCollisionObjectsTransform(pose1);

  // The comparison must be exact: it stands in for products that reach the identity only to rounding,
  // and whatever it accepts is reported as no motion at all.
  swept_ = end_pose.matrix() != world_pose_.matrix();
  if (swept_)
  {
    sweep_end_pose_ = end_pose;
    sweep_displacement_current_ = false;
  }

  return changed;
}

void CastCollisionObjectWrapper::computeSweepDisplacement() const
{
  sweep_displacement_ = sweep_end_pose_ * world_pose_.inverse(Eigen::Isometry);
  sweep_displacement_inverse_ = world_pose_ * sweep_end_pose_.inverse(Eigen::Isometry);
  sweep_displacement_current_ = true;
}

CastCOW::Ptr makeCastCollisionObject(const COW::Ptr& cow, bool build_swept, bool d_arc_compensation)
{
  auto cast_cow = std::make_shared<CastCollisionObjectWrapper>(*cow, d_arc_compensation);
  // Collision objects name their wrapper through user data, which is read back as the base type.
  CollisionObjectWrapperBase* const owner = cast_cow.get();

  // A static link is collided through its regular wrapper, so its cast wrapper is a placeholder that nothing
  // reads: it joins no broadphase, and updateCollisionObjectFilters builds it at the moment the link goes
  // kinematic. Leaving the clone's own geometry in place costs nothing for a link that never goes active, and
  // is what lets a static link hold geometry Coal can collide but not sweep, such as a mesh.
  if (!build_swept)
    return cast_cow;

  // Create the vector of new collision objects
  std::vector<CollisionObjectPtr> new_collision_objects;

  // Identity transform for initial state
  coal::Transform3s identity_tf;
  identity_tf.setIdentity();

  const auto& link_tf = cast_cow->getCollisionObjectsTransform();
  const auto& current_shapes = cast_cow->getCollisionGeometries();
  const auto& current_shape_poses = cast_cow->getCollisionGeometriesTransforms();

  const auto& current_collision_objects = cast_cow->getCollisionObjects();
  new_collision_objects.reserve(current_collision_objects.size());

  CollisionShapesConst new_shapes;
  tesseract::common::VectorIsometry3d new_shape_poses;
  new_shapes.reserve(current_shapes.size());
  new_shape_poses.reserve(current_shape_poses.size());

  for (const auto& co : current_collision_objects)
  {
    const auto old_shape_index = static_cast<std::size_t>(co->getShapeIndex());
    assert(old_shape_index < current_shapes.size());
    assert(old_shape_index < current_shape_poses.size());

    auto geo = co->collisionGeometry();
    // coal::Plane is a ShapeBase with no support function: refuse it before the arm below wraps it.
    if (geo->getNodeType() == coal::GEOM_PLANE)
      throw std::runtime_error("Link '" + cow->getLinkId().name() +
                               "': I cannot continuous collision check a plane on an active link");

    auto* shape_base_ptr = dynamic_cast<coal::ShapeBase*>(geo.get());
    if (shape_base_ptr != nullptr)
    {
      // Create a cast hull shape from the original shape
      auto cast_shape = std::make_shared<CastHullShape>(std::static_pointer_cast<coal::ShapeBase>(geo), identity_tf);

      // Create a new collision object with the cast shape
      auto cast_co = std::make_shared<CoalCollisionObjectWrapper>(cast_shape, co->getTransform());
      cast_co->setShapeIndex(static_cast<int>(new_shape_poses.size()));
      cast_co->setSourceShapeIndex(static_cast<int>(old_shape_index));
      cast_co->setSourceSubshapeIndex(co->getSourceSubshapeIndex());
      cast_co->setContactDistanceThreshold(co->getContactDistanceThreshold());
      cast_co->setUserData(owner);

      // Store everything
      new_collision_objects.push_back(cast_co);
      new_shapes.push_back(current_shapes[old_shape_index]);
      new_shape_poses.push_back(current_shape_poses[old_shape_index]);
    }
    else if (auto octree_geo = std::dynamic_pointer_cast<coal::OcTree>(geo); octree_geo != nullptr)
    {
      // Expand occupied octree cells into castable box sub-shapes.
      const auto tree = octree_geo->getTree();
      assert(tree != nullptr);
      const auto& base_shape_pose = current_shape_poses[old_shape_index];
      int octree_subshape_index = 0;

      // Reserve extra capacity for the voxel expansion. tree->size() is O(1)
      // and an upper bound on the number of occupied leaves.
      const std::size_t voxel_budget = tree->size();
      new_collision_objects.reserve(new_collision_objects.size() + voxel_budget);
      new_shapes.reserve(new_shapes.size() + voxel_budget);
      new_shape_poses.reserve(new_shape_poses.size() + voxel_budget);

      // Reuse one box shape per tree depth level (all voxels at the same depth
      // have the same size), matching Bullet's managed_shapes pattern.
      std::vector<std::shared_ptr<coal::Box>> managed_boxes(tree->getTreeDepth() + 1);

      for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
      {
        if (!octree_geo->isNodeOccupied(&(*it)))
          continue;

        auto& box_shape = managed_boxes.at(it.getDepth());
        if (box_shape == nullptr)
        {
          const double size = it.getSize();
          box_shape = std::make_shared<coal::Box>(size, size, size);
          // The arc-sagitta compensation reads this box's cached aabb_center and aabb_radius, and
          // nothing else ever populates them: these boxes are built here rather than taken from the
          // geometry cache. Left at coal's defaults the compensation evaluates to NaN, which
          // propagates into every cast AABB built from the box and drops the pair in broadphase.
          // The box is function-local and unshared at this point, so this write is not visible to
          // any other object.
          box_shape->computeLocalAABB();
        }
        auto cast_shape = std::make_shared<CastHullShape>(box_shape, identity_tf);

        Eigen::Isometry3d voxel_pose = Eigen::Isometry3d::Identity();
        voxel_pose.translation() = Eigen::Vector3d(it.getX(), it.getY(), it.getZ());

        const Eigen::Isometry3d shape_pose = base_shape_pose * voxel_pose;
        const Eigen::Isometry3d world_pose = link_tf * shape_pose;

        auto cast_co = std::make_shared<CoalCollisionObjectWrapper>(
            cast_shape, coal::Transform3s(world_pose.rotation(), world_pose.translation()));
        cast_co->setShapeIndex(static_cast<int>(new_shape_poses.size()));
        cast_co->setSourceShapeIndex(static_cast<int>(old_shape_index));
        cast_co->setSourceSubshapeIndex(octree_subshape_index++);
        cast_co->setContactDistanceThreshold(co->getContactDistanceThreshold());
        cast_co->setUserData(owner);

        new_collision_objects.push_back(cast_co);
        new_shapes.push_back(current_shapes[old_shape_index]);
        new_shape_poses.push_back(shape_pose);
      }
    }
    else
    {
      throw std::runtime_error("Link '" + cow->getLinkId().name() +
                               "': I can only continuous collision check convex shapes, compound shapes made "
                               "of convex shapes, and octree boxes");
    }
  }

  // Replace the collision objects in the cast_cow with the cast versions
  cast_cow->getCollisionGeometries() = new_shapes;
  cast_cow->getCollisionGeometriesTransforms() = new_shape_poses;
  cast_cow->getCollisionObjects() = new_collision_objects;

  return cast_cow;
}

}  // namespace tesseract::collision::tesseract_collision_coal
