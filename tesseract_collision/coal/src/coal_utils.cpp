/**
 * @file coal_utils.cpp
 * @brief Tesseract Coal Utility Functions.
 *
 * @author Roelof Oomen, Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <coal/collision_data.h>
#include <coal/collision.h>
#include <coal/distance.h>
#include <coal/BVH/BVH_model.h>
#include <coal/shape/geometric_shapes.h>
#include <coal/shape/convex.h>
#include <coal/data_types.h>
#include <coal/octree.h>
#include <stdexcept>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/coal/coal_utils.h>
#include <tesseract_geometry/geometries.h>

namespace tesseract_collision::tesseract_collision_coal
{

namespace
{
CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Plane::ConstPtr& geom)
{
  return std::make_shared<coal::Plane>(geom->getA(), geom->getB(), geom->getC(), geom->getD());
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Box::ConstPtr& geom)
{
  return std::make_shared<coal::Box>(geom->getX(), geom->getY(), geom->getZ());
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Sphere::ConstPtr& geom)
{
  return std::make_shared<coal::Sphere>(geom->getRadius());
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Cylinder::ConstPtr& geom)
{
  return std::make_shared<coal::Cylinder>(geom->getRadius(), geom->getLength());
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Cone::ConstPtr& geom)
{
  return std::make_shared<coal::Cone>(geom->getRadius(), geom->getLength());
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Capsule::ConstPtr& geom)
{
  return std::make_shared<coal::Capsule>(geom->getRadius(), geom->getLength());
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Mesh::ConstPtr& geom)
{
  const int vertex_count = geom->getVertexCount();
  const int triangle_count = geom->getFaceCount();
  const tesseract_common::VectorVector3d& vertices = *(geom->getVertices());
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

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::ConvexMesh::ConstPtr& geom)
{
  const auto vertex_count = geom->getVertexCount();
  const auto face_count = geom->getFaceCount();
  const auto& faces = *geom->getFaces();

  if (vertex_count > 0 && face_count > 0)
  {
    auto vertices = std::const_pointer_cast<tesseract_common::VectorVector3d>(geom->getVertices());

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

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Octree::ConstPtr& geom)
{
  switch (geom->getSubType())
  {
    case tesseract_geometry::OctreeSubType::BOX:
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
}  // namespace

CollisionGeometryPtr createShapePrimitiveHelper(const CollisionShapeConstPtr& geom)
{
  switch (geom->getType())
  {
    case tesseract_geometry::GeometryType::PLANE:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Plane>(geom));
    }
    case tesseract_geometry::GeometryType::BOX:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Box>(geom));
    }
    case tesseract_geometry::GeometryType::SPHERE:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Sphere>(geom));
    }
    case tesseract_geometry::GeometryType::CYLINDER:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Cylinder>(geom));
    }
    case tesseract_geometry::GeometryType::CONE:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Cone>(geom));
    }
    case tesseract_geometry::GeometryType::CAPSULE:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Capsule>(geom));
    }
    case tesseract_geometry::GeometryType::MESH:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Mesh>(geom));
    }
    case tesseract_geometry::GeometryType::CONVEX_MESH:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::ConvexMesh>(geom));
    }
    case tesseract_geometry::GeometryType::OCTREE:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Octree>(geom));
    }
    case tesseract_geometry::GeometryType::COMPOUND_MESH:
    {
      throw std::runtime_error("CompundMesh type should not be passed to this function!");
    }
    default:
    {
      CONSOLE_BRIDGE_logError("This geometric shape type (%d) is not supported using Coal yet",
                              static_cast<int>(geom->getType()));
      return nullptr;
    }
  }
}

CollisionGeometryPtr createShapePrimitive(const CollisionShapeConstPtr& geom)
{
  // TODO: Implement cache (see FCLCollisionGeometryCache)
  // CollisionGeometryPtr shape = CoalCollisionGeometryCache::get(geom);
  // if (shape != nullptr)
  //   return shape;

  // shape = createShapePrimitiveHelper(geom);
  // CoalCollisionGeometryCache::insert(geom, shape);
  return createShapePrimitiveHelper(geom);
}

bool CollisionCallback::collide(coal::CollisionObject* o1, coal::CollisionObject* o2)
{
  if (cdata->done)
    return true;

  const auto* cd1 = static_cast<const CollisionObjectWrapper*>(o1->getUserData());
  const auto* cd2 = static_cast<const CollisionObjectWrapper*>(o2->getUserData());

  const bool needs_collision = cd1->m_enabled && cd2->m_enabled &&
                               (cd1->m_collisionFilterGroup & cd2->m_collisionFilterMask) &&  // NOLINT
                               (cd2->m_collisionFilterGroup & cd1->m_collisionFilterMask) &&  // NOLINT
                               !isContactAllowed(cd1->getName(), cd2->getName(), cdata->validator, false);

  assert(std::find(cdata->active->begin(), cdata->active->end(), cd1->getName()) != cdata->active->end() ||
         std::find(cdata->active->begin(), cdata->active->end(), cd2->getName()) != cdata->active->end());

  if (!needs_collision)
    return false;

  std::size_t num_contacts = (cdata->req.contact_limit > 0) ? static_cast<std::size_t>(cdata->req.contact_limit) :
                                                              std::numeric_limits<std::size_t>::max();
  if (cdata->req.type == ContactTestType::FIRST)
    num_contacts = 1;

  coal::CollisionResult col_result;
  coal::CollisionRequest col_request;
  col_request.num_max_contacts = num_contacts;
  col_request.enable_contact = cdata->req.calculate_penetration;
  col_request.gjk_variant = coal::GJKVariant::NesterovAcceleration;
  col_request.gjk_convergence_criterion = coal::GJKConvergenceCriterion::DualityGap;
  col_request.gjk_convergence_criterion_type = coal::GJKConvergenceCriterionType::Absolute;
  col_request.gjk_initial_guess = coal::BoundingVolumeGuess;
  // col_request.gjk_tolerance = 1e-5;
  // col_request.epa_tolerance = 1e-5;
  col_request.break_distance = cdata->collision_margin_data.getMaxCollisionMargin();
  // col_request.collision_distance_threshold // Leave at default (close to 0)
  // col_request.distance_upper_bound = // Collision margin + buffer?
  col_request.security_margin = cdata->collision_margin_data.getCollisionMargin(cd1->getName(), cd2->getName());
  coal::collide(o1, o2, col_request, col_result);

  if (col_result.isCollision())
  {
    const Eigen::Isometry3d& tf1 = cd1->getCollisionObjectsTransform();
    const Eigen::Isometry3d& tf2 = cd2->getCollisionObjectsTransform();

    for (size_t i = 0; i < col_result.numContacts(); ++i)
    {
      const coal::Contact& coal_contact = col_result.getContact(i);
      ContactResult contact;
      contact.link_names[0] = cd1->getName();
      contact.link_names[1] = cd2->getName();
      contact.shape_id[0] = CollisionObjectWrapper::getShapeIndex(o1);
      contact.shape_id[1] = CollisionObjectWrapper::getShapeIndex(o2);
      contact.subshape_id[0] = static_cast<int>(coal_contact.b1);
      contact.subshape_id[1] = static_cast<int>(coal_contact.b2);
      contact.nearest_points[0] = coal_contact.nearest_points[0];
      contact.nearest_points[1] = coal_contact.nearest_points[1];
      contact.nearest_points_local[0] = tf1.inverse() * contact.nearest_points[0];
      contact.nearest_points_local[1] = tf2.inverse() * contact.nearest_points[1];
      contact.transform[0] = tf1;
      contact.transform[1] = tf2;
      contact.type_id[0] = cd1->getTypeID();
      contact.type_id[1] = cd2->getTypeID();
      contact.distance = coal_contact.penetration_depth;
      contact.normal = coal_contact.normal;

      const ObjectPairKey pc = tesseract_common::makeOrderedLinkPair(cd1->getName(), cd2->getName());
      const auto it = cdata->res->find(pc);
      const bool found = (it != cdata->res->end() && !it->second.empty());

      processResult(*cdata, contact, pc, found);
    }
  }

  return cdata->done;
}

bool DistanceCallback::collide(coal::CollisionObject* o1, coal::CollisionObject* o2)
{
  if (cdata->done)
    return true;

  const auto* cd1 = static_cast<const CollisionObjectWrapper*>(o1->getUserData());
  const auto* cd2 = static_cast<const CollisionObjectWrapper*>(o2->getUserData());

  const bool needs_collision = cd1->m_enabled && cd2->m_enabled &&
                               (cd1->m_collisionFilterGroup & cd2->m_collisionFilterMask) &&  // NOLINT
                               (cd2->m_collisionFilterGroup & cd1->m_collisionFilterMask) &&  // NOLINT
                               !isContactAllowed(cd1->getName(), cd2->getName(), cdata->validator, false);

  assert(std::find(cdata->active->begin(), cdata->active->end(), cd1->getName()) != cdata->active->end() ||
         std::find(cdata->active->begin(), cdata->active->end(), cd2->getName()) != cdata->active->end());

  if (!needs_collision)
    return false;

  coal::DistanceResult dist_result;
  coal::DistanceRequest dist_request(true);
  dist_request.enable_signed_distance = cdata->req.calculate_penetration;
  dist_request.gjk_variant = coal::GJKVariant::NesterovAcceleration;
  dist_request.gjk_convergence_criterion = coal::GJKConvergenceCriterion::DualityGap;
  dist_request.gjk_convergence_criterion_type = coal::GJKConvergenceCriterionType::Absolute;
  dist_request.gjk_initial_guess = coal::BoundingVolumeGuess;
  // dist_request.gjk_tolerance = 1e-5;
  // dist_request.epa_tolerance = 1e-5;
  // dist_request.collision_distance_threshold // Leave at default (close to 0)
  const double d = coal::distance(o1, o2, dist_request, dist_result);

  if (d < cdata->collision_margin_data.getMaxCollisionMargin())
  {
    const Eigen::Isometry3d& tf1 = cd1->getCollisionObjectsTransform();
    const Eigen::Isometry3d& tf2 = cd2->getCollisionObjectsTransform();

    ContactResult contact;
    contact.link_names[0] = cd1->getName();
    contact.link_names[1] = cd2->getName();
    contact.shape_id[0] = CollisionObjectWrapper::getShapeIndex(o1);
    contact.shape_id[1] = CollisionObjectWrapper::getShapeIndex(o2);
    contact.subshape_id[0] = dist_result.b1;
    contact.subshape_id[1] = dist_result.b2;
    contact.nearest_points[0] = dist_result.nearest_points[0];
    contact.nearest_points[1] = dist_result.nearest_points[1];
    contact.nearest_points_local[0] = tf1.inverse() * contact.nearest_points[0];
    contact.nearest_points_local[1] = tf2.inverse() * contact.nearest_points[1];
    contact.transform[0] = tf1;
    contact.transform[1] = tf2;
    contact.type_id[0] = cd1->getTypeID();
    contact.type_id[1] = cd2->getTypeID();
    contact.distance = dist_result.min_distance;
    contact.normal = (dist_result.min_distance * (contact.nearest_points[1] - contact.nearest_points[0])).normalized();
    // contact.normal = dist_result.normal;

    const ObjectPairKey pc = tesseract_common::makeOrderedLinkPair(cd1->getName(), cd2->getName());
    const auto it = cdata->res->find(pc);
    const bool found = (it != cdata->res->end() && !it->second.empty());

    processResult(*cdata, contact, pc, found);
  }

  return cdata->done;
}

CollisionObjectWrapper::CollisionObjectWrapper(std::string name,
                                               const int& type_id,
                                               CollisionShapesConst shapes,
                                               tesseract_common::VectorIsometry3d shape_poses)
  : name_(std::move(name)), type_id_(type_id), shapes_(std::move(shapes)), shape_poses_(std::move(shape_poses))
{
  assert(!shapes_.empty());                       // NOLINT
  assert(!shape_poses_.empty());                  // NOLINT
  assert(!name_.empty());                         // NOLINT
  assert(shapes_.size() == shape_poses_.size());  // NOLINT

  m_collisionFilterGroup = CollisionFilterGroups::KinematicFilter;
  m_collisionFilterMask = CollisionFilterGroups::StaticFilter | CollisionFilterGroups::KinematicFilter;

  collision_geometries_.reserve(shapes_.size());
  collision_objects_.reserve(shapes_.size());
  collision_objects_raw_.reserve(shapes_.size());
  for (std::size_t i = 0; i < shapes_.size(); ++i)  // NOLINT
  {
    if (shapes_[i]->getType() == tesseract_geometry::GeometryType::COMPOUND_MESH)
    {
      const auto& meshes = std::static_pointer_cast<const tesseract_geometry::CompoundMesh>(shapes_[i])->getMeshes();
      for (const auto& mesh : meshes)
      {
        const CollisionGeometryPtr subshape = createShapePrimitive(mesh);
        if (subshape != nullptr)
        {
          collision_geometries_.push_back(subshape);
          auto co = std::make_shared<CoalCollisionObjectWrapper>(subshape);
          co->setUserData(this);
          co->setShapeIndex(static_cast<int>(i));
          co->setTransform(coal::Transform3s(shape_poses_[i].rotation(), shape_poses_[i].translation()));
          co->updateAABB();
          collision_objects_.push_back(co);
          collision_objects_raw_.push_back(co.get());
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
        co->setTransform(coal::Transform3s(shape_poses_[i].rotation(), shape_poses_[i].translation()));
        co->updateAABB();
        collision_objects_.push_back(co);
        collision_objects_raw_.push_back(co.get());
      }
    }
  }
}

int CollisionObjectWrapper::getShapeIndex(const coal::CollisionObject* co)
{
  return static_cast<const CoalCollisionObjectWrapper*>(co)->getShapeIndex();
}

}  // namespace tesseract_collision::tesseract_collision_coal
