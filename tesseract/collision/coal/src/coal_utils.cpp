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
#include <console_bridge/console.h>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <unordered_set>
#include <utility>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_utils.h>
#include <tesseract/collision/coal/coal_collision_geometry_cache.h>
#include <tesseract/collision/coal/coal_casthullshape.h>
#include <tesseract/collision/coal/coal_collision_object_wrapper.h>
#include <tesseract/geometry/geometries.h>

namespace tesseract::collision::tesseract_collision_coal
{

void computeShapeAABB(const coal::ShapeBase& s, const coal::Transform3s& tf, coal::AABB& bv)
{
  switch (s.getNodeType())
  {
    case coal::GEOM_BOX:
      coal::computeBV<coal::AABB>(static_cast<const coal::Box&>(s), tf, bv);
      return;
    case coal::GEOM_SPHERE:
      coal::computeBV<coal::AABB>(static_cast<const coal::Sphere&>(s), tf, bv);
      return;
    case coal::GEOM_ELLIPSOID:
      coal::computeBV<coal::AABB>(static_cast<const coal::Ellipsoid&>(s), tf, bv);
      return;
    case coal::GEOM_CAPSULE:
      coal::computeBV<coal::AABB>(static_cast<const coal::Capsule&>(s), tf, bv);
      return;
    case coal::GEOM_CONE:
      coal::computeBV<coal::AABB>(static_cast<const coal::Cone&>(s), tf, bv);
      return;
    case coal::GEOM_CYLINDER:
      coal::computeBV<coal::AABB>(static_cast<const coal::Cylinder&>(s), tf, bv);
      return;
    case coal::GEOM_CONVEX32:
      coal::computeBV<coal::AABB>(static_cast<const coal::ConvexBase32&>(s), tf, bv);
      return;
    case coal::GEOM_CONVEX16:
      coal::computeBV<coal::AABB>(static_cast<const coal::ConvexBase16&>(s), tf, bv);
      return;
    case coal::GEOM_TRIANGLE:
      coal::computeBV<coal::AABB>(static_cast<const coal::TriangleP&>(s), tf, bv);
      return;
    case coal::GEOM_HALFSPACE:
      coal::computeBV<coal::AABB>(static_cast<const coal::Halfspace&>(s), tf, bv);
      return;
    case coal::GEOM_PLANE:
      coal::computeBV<coal::AABB>(static_cast<const coal::Plane&>(s), tf, bv);
      return;
    default:
      // GEOM_CUSTOM and any future types: conservative |R|*half-extents fallback.
      coal::computeBV<coal::AABB, coal::ShapeBase>(s, tf, bv);
      return;
  }
}

namespace
{
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
    std::vector<coal::Triangle> tri_indices(static_cast<size_t>(triangle_count));
    for (int i = 0; i < triangle_count; ++i)
    {
      assert(triangles[4L * i] == 3);
      tri_indices[static_cast<size_t>(i)] = coal::Triangle(static_cast<size_t>(triangles[(4 * i) + 1]),
                                                           static_cast<size_t>(triangles[(4 * i) + 2]),
                                                           static_cast<size_t>(triangles[(4 * i) + 3]));
    }

    g->beginModel();
    g->addSubModel(vertices, tri_indices);
    g->endModel();

    return g;
  }

  CONSOLE_BRIDGE_logError("The mesh is empty!");
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
    new_faces->reserve(face_count);
    for (int i = 0; i < faces.size(); ++i)
    {
      Polygon new_face;
      // First value of each face is the number of vertices
      new_face.resize(faces[i]);
      for (std::uint32_t& j : new_face)
      {
        ++i;
        j = faces[i];
      }
      new_faces->emplace_back(new_face);
    }
    assert(new_faces->size() == face_count);

    return std::make_shared<coal::Convex<Polygon>>(vertices, vertex_count, new_faces, face_count);
  }

  CONSOLE_BRIDGE_logError("The mesh is empty!");
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
      CONSOLE_BRIDGE_logError("This Coal octree sub shape type (%d) is not supported for geometry octree",
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
      CONSOLE_BRIDGE_logError("This geometric shape type (%d) is not supported using Coal yet",
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
  CoalCollisionGeometryCache::insert(geom, shape);
  return shape;
}

constexpr double COAL_SUPPORT_FUNC_TOLERANCE = 0.01;
constexpr double COAL_LENGTH_TOLERANCE = 0.001;
constexpr double COAL_EPSILON = 1e-6;

/**
 * @brief Compute the average support point for a shape along a direction.
 *
 * Matches Bullet's GetAverageSupport algorithm: for polyhedral shapes
 * (ConvexBase32), iterates all vertices, finds the maximum support value,
 * and averages all vertices within COAL_EPSILON of that maximum. This
 * handles degenerate cases where multiple vertices have equal support
 * (e.g., two symmetric vertices of a tessellated sphere), producing a
 * canonical point that's independent of vertex iteration order.
 *
 * For non-polyhedral shapes (Sphere, Capsule, etc.), falls back to coal's
 * getSupport<WithSweptSphere> which includes the shape's radius.
 */
void GetAverageSupport(const coal::ShapeBase* shape,
                       const coal::Vec3s& localNormal,
                       double& outsupport,
                       coal::Vec3s& outpt)
{
  const auto* convex = dynamic_cast<const coal::ConvexBase32*>(shape);
  if (convex != nullptr && convex->points && !convex->points->empty())
  {
    coal::Vec3s ptSum = coal::Vec3s::Zero();
    double ptCount = 0;
    double maxSupport = std::numeric_limits<double>::lowest();

    for (const auto& pt : *convex->points)
    {
      double sup = pt.dot(localNormal);
      if (sup > maxSupport + COAL_EPSILON)
      {
        ptCount = 1;
        ptSum = pt;
        maxSupport = sup;
      }
      else if (sup >= maxSupport - COAL_EPSILON)
      {
        ptCount += 1;
        ptSum += pt;
      }
    }
    outsupport = maxSupport;
    outpt = ptSum / ptCount;
  }
  else
  {
    // For primitive shapes (Sphere, etc.), use coal's standard support function.
    // WithSweptSphere mode is required so that Sphere returns radius*normalize(dir)
    // instead of zero (NoSweptSphere treats Sphere as a point + inflation).
    int hint = 0;
    outpt = coal::details::getSupport<coal::details::SupportOptions::WithSweptSphere>(shape, localNormal, hint);
    outsupport = localNormal.dot(outpt);
  }
}

bool needsCollisionCheck(const CollisionObjectWrapper* cd1,
                         const CollisionObjectWrapper* cd2,
                         const std::shared_ptr<const tesseract::common::ContactAllowedValidator>& validator,
                         bool verbose)
{
  return cd1->m_enabled && cd2->m_enabled && (cd2->m_collisionFilterGroup & cd1->m_collisionFilterMask) &&  // NOLINT
         (cd1->m_collisionFilterGroup & cd2->m_collisionFilterMask) &&                                      // NOLINT
         !isContactAllowed(cd1->getName(), cd2->getName(), validator, verbose);
}

/**
 * @brief Populate continuous collision fields (cc_time, cc_type, cc_transform) on a ContactResult.
 *
 * Uses support-function-based approach matching Bullet's calculateContinuousData:
 * finds the shape's extreme points along the contact normal at t=0 and t=1, then
 * classifies the collision time based on which pose has greater support.
 *
 * Only kinematic objects should receive CCD metadata. Static objects may still be
 * represented by CastHullShape proxies as a Coal narrowphase workaround (for example,
 * static octrees expanded to boxes), but they should keep the default CCD fields.
 */
void populateContinuousCollisionFields(ContactResult& contact,
                                       const coal::CollisionObject* o1,
                                       const coal::CollisionObject* o2,
                                       const std::array<Eigen::Isometry3d, 2>& tf_inv)
{
  const std::array<const coal::CollisionObject*, 2> objects = { o1, o2 };
  for (std::size_t i = 0; i < 2; ++i)
  {
    const auto* cow = static_cast<const CollisionObjectWrapper*>(objects[i]->getUserData());
    if (cow == nullptr || cow->m_collisionFilterGroup != CollisionFilterGroups::KinematicFilter)
      continue;

    const auto* cast_shape = dynamic_cast<const CastHullShape*>(objects[i]->collisionGeometry().get());
    if (cast_shape == nullptr)
      continue;

    const auto& ct = cast_shape->getCastTransform();

    // Shape world transforms at t=0 and t=1
    const coal::Transform3s& tf_world0 = objects[i]->getTransform();
    coal::Transform3s tf_world1 = tf_world0 * ct;

    // cc_transform = link transform at t=1
    // Recover link_tf2 from shape world transforms:
    //   shape_tf0 = link_tf1 * local_tf
    //   shape_tf1 = link_tf2 * local_tf
    //   link_tf2 = shape_tf1 * shape_tf0^-1 * link_tf1
    // This correctly handles shapes with non-identity local offsets,
    // matching Bullet's calculateContinuousData approach.
    Eigen::Isometry3d shape_tf0;
    shape_tf0.linear() = tf_world0.getRotation();
    shape_tf0.translation() = Eigen::Vector3d(tf_world0.getTranslation());
    Eigen::Isometry3d shape_tf1;
    shape_tf1.linear() = tf_world1.getRotation();
    shape_tf1.translation() = Eigen::Vector3d(tf_world1.getTranslation());
    contact.cc_transform[i] = shape_tf1 * shape_tf0.inverse() * contact.transform[i];

    // Normal pointing from current object toward the other (matching Bullet convention).
    // contact.normal has already been remapped to original (o1, o2) order after pair normalization.
    const coal::Vec3s normal_world = (i == 0) ? coal::Vec3s(contact.normal) : coal::Vec3s(-contact.normal);

    // Transform normal into local frames at t=0 and t=1
    coal::Vec3s normal_local0 = tf_world0.getRotation().transpose() * normal_world;
    coal::Vec3s normal_local1 = tf_world1.getRotation().transpose() * normal_world;

    // Get averaged support points on the underlying shape at both local normals.
    // Uses GetAverageSupport (matching Bullet's GetAverageSupport) which averages
    // tied vertices, producing canonical points independent of vertex iteration order.
    const coal::ShapeBase* underlying = cast_shape->getUnderlyingShape().get();
    coal::Vec3s pt_local0;
    double sup_local0 = 0;
    GetAverageSupport(underlying, normal_local0, sup_local0, pt_local0);

    coal::Vec3s pt_local1;
    double sup_local1 = 0;
    GetAverageSupport(underlying, normal_local1, sup_local1, pt_local1);

    // Compare world-frame supports at the LINK origin as reference center,
    // matching Bullet's compound-child treatment:
    //
    //   sup_world = sup_local + normal_world · link_center
    //
    // Using the link center (not the per-shape world center) avoids orbital-
    // motion bias: when a multi-shape link rotates, each per-shape center
    // orbits the link origin, adding a spurious translational term
    // (normal · link_R * local_offset) to the comparison that differs between
    // sub-shapes even though the link undergoes the same motion.  Bullet
    // avoids this by building compound-child cast transforms at link level
    // (no per-shape translation component); we replicate that here.
    //
    // For pure rotation: link_center0 == link_center1, so the dot-product
    // terms cancel and the comparison reduces to sup_local1 vs sup_local0,
    // unchanged from before.
    //
    // For pure translation: sup_local0 == sup_local1 (same rotation ⇒ same
    // local normal), so link_sup1 − link_sup0 = normal · link_sweep.
    // A positive value means the shape's surface advances in the normal
    // direction over the sweep → CCType_Time1, matching Bullet.
    const Eigen::Vector3d& nw(normal_world);
    const double link_sup0 = sup_local0 + nw.dot(contact.transform[i].translation());
    const double link_sup1 = sup_local1 + nw.dot(contact.cc_transform[i].translation());

    const Eigen::Isometry3d& link_tf_inv = tf_inv[i];

    if (link_sup0 - link_sup1 > COAL_SUPPORT_FUNC_TOLERANCE)
    {
      contact.cc_time[i] = 0;
      contact.cc_type[i] = ContinuousCollisionType::CCType_Time0;
      contact.nearest_points_local[i] = link_tf_inv * (shape_tf0 * Eigen::Vector3d(pt_local0));
    }
    else if (link_sup1 - link_sup0 > COAL_SUPPORT_FUNC_TOLERANCE)
    {
      contact.cc_time[i] = 1;
      contact.cc_type[i] = ContinuousCollisionType::CCType_Time1;
      // pt_local1 is the support point in the shape's local frame at t=1
      // rotation.  Map it to world via shape_tf1 (shape at end pose), then
      // to link-local.  This matches Bullet's calculateContinuousData, which
      // uses the t=1 shape transform for the CCType_Time1 branch, so that
      // transform[ki] * nearest_points_local[ki] == nearest_points[ki]
      // (the actual world-frame contact point).
      contact.nearest_points_local[i] = link_tf_inv * (shape_tf1 * Eigen::Vector3d(pt_local1));
    }
    else
    {
      contact.cc_type[i] = ContinuousCollisionType::CCType_Between;

      // Compute cc_time from the ratio of distances between the GJK witness
      // point and the surface support points at t=0 and t=1, matching
      // Schulman et al. IJRR 2014, Eq. (17):
      //   α = ||p1 - p_swept|| / (||p1 - p_swept|| + ||p0 - p_swept||)
      // where α weights p0 (i.e., cc_time = 1-α = l0c/(l0c+l1c)).
      const Eigen::Vector3d shape_ptWorld0 = shape_tf0 * Eigen::Vector3d(pt_local0);
      const Eigen::Vector3d shape_ptWorld1 = shape_tf1 * Eigen::Vector3d(pt_local1);
      const double l0c = (contact.nearest_points[i] - shape_ptWorld0).norm();
      const double l1c = (contact.nearest_points[i] - shape_ptWorld1).norm();

      if (l0c + l1c < COAL_LENGTH_TOLERANCE)
        contact.cc_time[i] = 0.5;
      else
        contact.cc_time[i] = std::clamp(l0c / (l0c + l1c), 0.0, 1.0);

      // nearest_points_local: average of the two local support points,
      // transformed to world via shape_tf0, then to link-local coordinates.
      // Matches Bullet's calculateContinuousData: (shape_ptLocal0 + shape_ptLocal1) / 2.0
      const coal::Vec3s avg_pt_local = (pt_local0 + pt_local1) / 2.0;
      contact.nearest_points_local[i] = link_tf_inv * (shape_tf0 * Eigen::Vector3d(avg_pt_local));
    }
  }
}

int getReportedSubshapeIndex(const coal::CollisionObject* object, int coal_subshape_index)
{
  const auto* collision_object = static_cast<const CoalCollisionObjectWrapper*>(object);
  const int source_subshape_index = collision_object->getSourceSubshapeIndex();
  if (source_subshape_index >= 0)
    return source_subshape_index;

  return coal_subshape_index;
}

bool CollisionCallback::collide(coal::CollisionObject* o1, coal::CollisionObject* o2)
{
  if (cdata->done)
    return true;

  const auto* cd1 = static_cast<const CollisionObjectWrapper*>(o1->getUserData());
  const auto* cd2 = static_cast<const CollisionObjectWrapper*>(o2->getUserData());

  if (!needsCollisionCheck(cd1, cd2, cdata->validator, false))
    return false;

  std::size_t num_contacts = (cdata->req.contact_limit > 0) ? static_cast<std::size_t>(cdata->req.contact_limit) :
                                                              std::numeric_limits<std::size_t>::max();
  if (cdata->req.type == ContactTestType::FIRST)
    num_contacts = 1;
  const auto security_margin = cdata->collision_margin_data.getCollisionMargin(cd1->getName(), cd2->getName());

  // Normalize pair ordering for consistent cache lookups: Coal's broadphase
  // does not guarantee a stable (o1, o2) ordering across tree rebalances,
  // so always put the smaller pointer first to avoid duplicate cache entries.
  const bool pair_swapped = std::greater<>{}(o1, o2);
  auto* co1 = pair_swapped ? o2 : o1;
  auto* co2 = pair_swapped ? o1 : o2;
  CollisionObjectPair object_pair = std::make_pair(co1, co2);
  auto col_cache_it = cdata->collision_cache->find(object_pair);

  if (col_cache_it == cdata->collision_cache->end())
  {
    const bool is_cast = dynamic_cast<const CastHullShape*>(co1->collisionGeometry().get()) != nullptr ||
                         dynamic_cast<const CastHullShape*>(co2->collisionGeometry().get()) != nullptr;

    coal::CollisionRequest col_request;
    // NesterovAcceleration + DualityGap/Relative for both cast and discrete pairs.
    // PolyakAcceleration fails cast sphere-sphere contact accuracy (compared to Bullet) regardless of
    // criterion. DualityGap/Absolute with Nesterov misses collisions on CastHullShape.
    col_request.gjk_variant = coal::GJKVariant::NesterovAcceleration;
    col_request.gjk_convergence_criterion = coal::GJKConvergenceCriterion::DualityGap;
    col_request.gjk_convergence_criterion_type = coal::GJKConvergenceCriterionType::Relative;

    auto col_functor = coal::ComputeCollision(co1->collisionGeometryPtr(), co2->collisionGeometryPtr());
    col_cache_it = cdata->collision_cache
                       ->try_emplace(object_pair, CollisionCacheEntry{ std::move(col_request), col_functor, is_cast })
                       .first;
  }

  auto& entry = col_cache_it->second;
  auto& cached_request = entry.request;

  // Re-seed GJK guess when COW generation has changed (transform or enable/disable change).
  // Deferred to here because the correct seed depends on the actual transforms.
  // Reuse cd1/cd2 from above, applying the same swap to match cache key ordering.
  const auto* cow1 = pair_swapped ? cd2 : cd1;
  const auto* cow2 = pair_swapped ? cd1 : cd2;
  if (entry.gen0 != cow1->gjk_generation_ || entry.gen1 != cow2->gjk_generation_)
  {
    // Cast pairs use center-to-center direction: BoundingVolumeGuess causes solver failure for
    // swept volumes, and DefaultGuess(1,0,0) degrades contact accuracy.
    if (entry.is_cast)
    {
      cached_request.gjk_initial_guess = coal::CachedGuess;
      coal::Vec3s guess = co1->getTransform().getTranslation() - co2->getTransform().getTranslation();
      if (guess.squaredNorm() < 1e-12)
        guess = coal::Vec3s(1, 0, 0);
      cached_request.cached_gjk_guess = guess;
    }
    else
    {
      cached_request.gjk_initial_guess = coal::BoundingVolumeGuess;
    }

    entry.gen0 = cow1->gjk_generation_;
    entry.gen1 = cow2->gjk_generation_;
  }

  cached_request.enable_contact = cdata->req.calculate_penetration;
  cached_request.num_max_contacts = num_contacts;
  cached_request.security_margin = security_margin;
  cached_request.distance_upper_bound = security_margin + cached_request.gjk_tolerance;

  coal::CollisionResult col_result;
  entry.functor(co1->getTransform(), co2->getTransform(), cached_request, col_result);

  // Warm-start: cache the GJK/EPA result for the next call on this pair.
  // The cached separating direction (NoCollision) or penetration vector (EPA)
  // is a better seed than recomputing from geometry each time.
  // Cached guesses are only updated if gjk_initial_guess == CachedGuess. Our first
  // discrete collision check uses BoundingVolumeGuess, so we have to update manually.
  cached_request.gjk_initial_guess = coal::CachedGuess;
  cached_request.cached_gjk_guess = col_result.cached_gjk_guess;
  cached_request.cached_support_func_guess = col_result.cached_support_func_guess;

  if (!col_result.isCollision())
    return false;

  // Some Coal traversal nodes (e.g., ShapeOcTreeCollisionTraversalNode) internally
  // swap arguments without compensating in the result, causing Contact o1/o2, b1/b2,
  // nearest_points, and normal to not match the (co1, co2) ordering. Detect this by
  // checking if the first contact's o1 matches co1's geometry.
  if (col_result.getContact(0).o1 != co1->collisionGeometry().get())
    col_result.swapObjects();

  auto link_pair = tesseract::common::LinkIdPair::make(tesseract::common::LinkId::fromName(cd1->getName()),
                                                       tesseract::common::LinkId::fromName(cd2->getName()));

  const Eigen::Isometry3d& tf1 = cd1->getCollisionObjectsTransform();
  const Eigen::Isometry3d& tf2 = cd2->getCollisionObjectsTransform();

  const std::array<Eigen::Isometry3d, 2> tf_inv = { tf1.inverse(), tf2.inverse() };

  // Coal result fields are in normalized (co1, co2) order; map back to original (o1, o2).
  const int idx0 = pair_swapped ? 1 : 0;
  const int idx1 = pair_swapped ? 0 : 1;

  bool found = false;
  for (size_t i = 0; i < col_result.numContacts(); ++i)
  {
    const coal::Contact& coal_contact = col_result.getContact(i);
    ContactResult contact;
    contact.link_names[0] = cd1->getName();
    contact.link_names[1] = cd2->getName();
    contact.shape_id[0] = CollisionObjectWrapper::getShapeIndex(o1);
    contact.shape_id[1] = CollisionObjectWrapper::getShapeIndex(o2);
    contact.subshape_id[0] =
        getReportedSubshapeIndex(o1, static_cast<int>(pair_swapped ? coal_contact.b2 : coal_contact.b1));
    contact.subshape_id[1] =
        getReportedSubshapeIndex(o2, static_cast<int>(pair_swapped ? coal_contact.b1 : coal_contact.b2));
    contact.nearest_points[0] = coal_contact.nearest_points[idx0];
    contact.nearest_points[1] = coal_contact.nearest_points[idx1];
    contact.nearest_points_local[0] = tf_inv[0] * contact.nearest_points[0];
    contact.nearest_points_local[1] = tf_inv[1] * contact.nearest_points[1];
    contact.transform[0] = tf1;
    contact.transform[1] = tf2;
    contact.type_id[0] = cd1->getTypeID();
    contact.type_id[1] = cd2->getTypeID();
    contact.distance = coal_contact.penetration_depth;
    contact.normal = pair_swapped ? coal::Vec3s(-coal_contact.normal) : coal_contact.normal;

    populateContinuousCollisionFields(contact, o1, o2, tf_inv);

    if (!found)
    {
      const auto it = cdata->res->find(link_pair);
      found = (it != cdata->res->end() && !it->second.empty());
    }
    processResult(*cdata, contact, link_pair, found);
  }

  return cdata->done;
}

CollisionObjectWrapper::CollisionObjectWrapper(std::string name,
                                               const int& type_id,
                                               CollisionShapesConst shapes,
                                               tesseract::common::VectorIsometry3d shape_poses)
  : name_(std::move(name)), type_id_(type_id), shapes_(std::move(shapes)), shape_poses_(std::move(shape_poses))
{
  // Preconditions guaranteed by createCoalCollisionObject() which validates before construction.
  assert(!shapes_.empty());                       // NOLINT
  assert(!shape_poses_.empty());                  // NOLINT
  assert(!name_.empty());                         // NOLINT
  assert(shapes_.size() == shape_poses_.size());  // NOLINT

  collision_geometries_.reserve(shapes_.size());
  collision_objects_.reserve(shapes_.size());
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
          collision_geometries_.push_back(subshape);
          auto co = std::make_shared<CoalCollisionObjectWrapper>(subshape);
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
        collision_geometries_.push_back(subshape);
        auto co = std::make_shared<CoalCollisionObjectWrapper>(subshape);
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

int CollisionObjectWrapper::getShapeIndex(const coal::CollisionObject* co)
{
  return static_cast<const CoalCollisionObjectWrapper*>(co)->getSourceShapeIndex();
}

void CollisionObjectWrapper::setCollisionObjectsTransform(const Eigen::Isometry3d& pose)
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

void CollisionObjectWrapper::setContactDistanceThreshold(double contact_distance)
{
  contact_distance_ = contact_distance;
  for (auto& co : collision_objects_)
    co->setContactDistanceThreshold(contact_distance_);
}

void CollisionObjectWrapper::appendCollisionObjectsRaw(std::vector<CollisionObjectRawPtr>& out) const
{
  for (const auto& co : collision_objects_)
    out.push_back(co.get());
}

std::shared_ptr<CollisionObjectWrapper> CollisionObjectWrapper::clone() const
{
  auto clone_cow = std::make_shared<CollisionObjectWrapper>();
  clone_cow->name_ = name_;
  clone_cow->type_id_ = type_id_;
  clone_cow->shapes_ = shapes_;
  clone_cow->shape_poses_ = shape_poses_;
  clone_cow->collision_geometries_ = collision_geometries_;

  clone_cow->collision_objects_.reserve(collision_objects_.size());
  for (const auto& co : collision_objects_)
  {
    assert(std::dynamic_pointer_cast<CoalCollisionObjectWrapper>(co) != nullptr);
    auto collObj =
        std::make_shared<CoalCollisionObjectWrapper>(*std::static_pointer_cast<CoalCollisionObjectWrapper>(co));
    collObj->setUserData(clone_cow.get());
    collObj->setTransform(co->getTransform());
    collObj->updateAABB();
    clone_cow->collision_objects_.push_back(collObj);
  }

  clone_cow->world_pose_ = world_pose_;
  clone_cow->m_collisionFilterGroup = m_collisionFilterGroup;
  clone_cow->m_collisionFilterMask = m_collisionFilterMask;
  clone_cow->m_enabled = m_enabled;
  return clone_cow;
}

/// Build an unordered_set of raw pointers for O(1) membership tests.
static std::unordered_set<const coal::CollisionObject*> buildPointerSet(const std::vector<CollisionObjectPtr>& objects)
{
  std::unordered_set<const coal::CollisionObject*> ptrs;
  ptrs.reserve(objects.size());
  for (const auto& co : objects)
    ptrs.insert(co.get());
  return ptrs;
}

void invalidateCacheFor(CollisionCacheMap& cache, const std::vector<CollisionObjectPtr>& objects)
{
  const auto ptrs = buildPointerSet(objects);
  for (auto it = cache.begin(); it != cache.end();)
  {
    if (ptrs.count(it->first.first) != 0 || ptrs.count(it->first.second) != 0)
      it = cache.erase(it);
    else
      ++it;
  }
}

COW::Ptr createCoalCollisionObject(const std::string& name,
                                   const int& type_id,
                                   const CollisionShapesConst& shapes,
                                   const tesseract::common::VectorIsometry3d& shape_poses,
                                   bool enabled)
{
  // dont add object that does not have geometry
  if (shapes.empty() || shape_poses.empty() || (shapes.size() != shape_poses.size()))
  {
    CONSOLE_BRIDGE_logDebug("ignoring link %s", name.c_str());
    return nullptr;
  }

  auto new_cow = std::make_shared<COW>(name, type_id, shapes, shape_poses);

  new_cow->m_enabled = enabled;
  // CONSOLE_BRIDGE_logDebug("Created collision object for link %s", new_cow->getName().c_str());
  return new_cow;
}

/** @brief Tolerance for transform comparison to avoid unnecessary BVH re-balancing */
static constexpr double kTransformEpsilon = 1e-8;

bool transformChanged(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b)
{
  return !a.translation().isApprox(b.translation(), kTransformEpsilon) ||
         !a.rotation().isApprox(b.rotation(), kTransformEpsilon);
}

void applyCollisionFilterMask(COW& cow)
{
  if (cow.m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
    cow.m_collisionFilterMask = CollisionFilterGroups::KinematicFilter;
  else
    cow.m_collisionFilterMask = CollisionFilterGroups::StaticFilter | CollisionFilterGroups::KinematicFilter;
}

void updateCollisionObjectFilters(const std::vector<std::string>& active,
                                  const COW::Ptr& cow,
                                  const std::unique_ptr<coal::BroadPhaseCollisionManager>& static_manager,
                                  const std::unique_ptr<coal::BroadPhaseCollisionManager>& dynamic_manager)
{
  // For discrete checks we can check static to kinematic and kinematic to
  // kinematic
  if (!isLinkActive(active, cow->getName()))
  {
    if (cow->m_collisionFilterGroup != CollisionFilterGroups::StaticFilter)
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
    if (cow->m_collisionFilterGroup != CollisionFilterGroups::KinematicFilter)
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

void updateCollisionObjectFilters(const std::vector<std::string>& active,
                                  const COW::Ptr& cow,
                                  COW::Ptr& cast_cow,
                                  const std::unique_ptr<coal::BroadPhaseCollisionManager>& static_manager,
                                  const std::unique_ptr<coal::BroadPhaseCollisionManager>& dynamic_manager)
{
  const std::vector<CollisionObjectPtr>& reg_objects = cow->getCollisionObjects();

  if (!isLinkActive(active, cow->getName()))
  {
    if (cow->m_collisionFilterGroup != CollisionFilterGroups::StaticFilter)
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
    if (cow->m_collisionFilterGroup != CollisionFilterGroups::KinematicFilter)
    {
      // Static -> kinematic: expand deferred octrees, then swap broadphases.
      if (castCowNeedsOctreeExpansion(cast_cow))
      {
        cast_cow = makeCastCollisionObject(cow);
        cast_cow->setContactDistanceThreshold(cow->getContactDistanceThreshold());
        cast_cow->setCollisionObjectsTransform(cow->getCollisionObjectsTransform());
      }

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

COW::Ptr makeCastCollisionObject(const COW::Ptr& cow, bool expand_octrees)
{
  auto cast_cow = cow->clone();

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
      cast_co->setUserData(cast_cow.get());

      // Store everything
      new_collision_objects.push_back(cast_co);
      new_shapes.push_back(current_shapes[old_shape_index]);
      new_shape_poses.push_back(current_shape_poses[old_shape_index]);
    }
    else
    {
      if (auto octree_geo = std::dynamic_pointer_cast<coal::OcTree>(geo); octree_geo != nullptr)
      {
        if (expand_octrees)
        {
          // Expand occupied octree cells into castable box sub-shapes.
          const auto tree = octree_geo->getTree();
          assert(tree != nullptr);
          const auto& base_shape_pose = current_shape_poses[old_shape_index];
          int octree_subshape_index = 0;

          // Reserve extra capacity for the voxel expansion.  tree->size() is O(1)
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
            cast_co->setUserData(cast_cow.get());

            new_collision_objects.push_back(cast_co);
            new_shapes.push_back(current_shapes[old_shape_index]);
            new_shape_poses.push_back(shape_pose);
          }
        }
        else
        {
          // Deferred: keep raw OcTree. Expansion happens on promotion to active.
          auto pass_co = std::make_shared<CoalCollisionObjectWrapper>(geo, co->getTransform());
          pass_co->setShapeIndex(static_cast<int>(new_shape_poses.size()));
          pass_co->setSourceShapeIndex(static_cast<int>(old_shape_index));
          pass_co->setContactDistanceThreshold(co->getContactDistanceThreshold());
          pass_co->setUserData(cast_cow.get());
          new_collision_objects.push_back(pass_co);
          new_shapes.push_back(current_shapes[old_shape_index]);
          new_shape_poses.push_back(current_shape_poses[old_shape_index]);
        }
      }
      else
      {
        throw std::runtime_error("I can only continuous collision check convex shapes, compound shapes made of "
                                 "convex shapes, and octree boxes");
      }
    }
  }

  // Replace the collision objects in the cast_cow with the cast versions
  cast_cow->getCollisionGeometries() = new_shapes;
  cast_cow->getCollisionGeometriesTransforms() = new_shape_poses;
  cast_cow->getCollisionObjects() = new_collision_objects;

  return cast_cow;
}

}  // namespace tesseract::collision::tesseract_collision_coal
