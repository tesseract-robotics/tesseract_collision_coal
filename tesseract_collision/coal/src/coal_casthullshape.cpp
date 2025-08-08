/**
 * @file coal_casthullshape.cpp
 * @brief Tesseract Coal Utility Functions.
 *
 * @author Roelof Oomen
 * @date Aug 04, 2025
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
#include <coal/broadphase/broadphase_collision_manager.h>
#include <coal/collision.h>
#include <coal/distance.h>
#include <console_bridge/console.h>
#include <algorithm>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/common.h>

#include <tesseract_collision/coal/coal_collision_object_wrapper.h>
#include <tesseract_collision/coal/coal_casthullshape.h>
#include <tesseract_collision/coal/coal_casthullshape_utility.h>

#include <tesseract_geometry/conversions.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_geometry/utils.h>

namespace tesseract_collision::tesseract_collision_coal
{

void CastHullShape::computeLocalAABB()
{
  // Consistent with Coal pattern, use the external computeBV function
  // Create an identity transform since we're computing the local AABB
  coal::Transform3s tf = coal::Transform3s::Identity();

  // Create an AABB object to hold the result
  coal::AABB aabb;

  // Call the external computeBV function
  coal::computeBV<coal::AABB, CastHullShape>(*this, tf, aabb);

  // Update the shape's AABB members
  aabb_local = aabb;
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

double CastHullShape::computeVolume() const
{
  // For swept shapes, we need to compute the volume of the convex hull
  // of the original shape and its transformed position.
  // This is a complex geometric problem, so we'll use a reasonable approximation.

  // Get the base volume
  double baseVolume = shape_->computeVolume();

  // Check if transform is identity (no sweeping needed)
  coal::Vec3s translation = castTransform_.getTranslation();
  double translation_length = translation.norm();

  // Check for rotation component
  coal::Matrix3s rotation = castTransform_.getRotation();
  bool has_rotation = !rotation.isIdentity(1e-6);

  // If no transformation, return base volume
  if (translation_length < 1e-6 && !has_rotation)
  {
    return baseVolume;
  }

  // For approximation, compute the volume using the swept AABB
  // We need to compute the AABB of the swept shape
  coal::Transform3s identity = coal::Transform3s::Identity();
  coal::AABB swept_aabb;
  coal::computeBV<coal::AABB, CastHullShape>(*this, identity, swept_aabb);

  double sweptVolume = swept_aabb.volume();

  // The swept volume should be at least the base volume
  return std::max(baseVolume, sweptVolume);
}

void CastHullShape::computeSweptVertices()
{
  // Extract vertices from the underlying shape
  std::vector<coal::Vec3s> baseVertices = extractVertices(shape_.get());

  // Store both start and end positions
  points->clear();
  points->reserve(baseVertices.size() * 2);

  // Add vertices at starting position
  for (const auto& vertex : baseVertices)
  {
    points->push_back(vertex);
  }
  // Add vertices at ending position (after transform)
  for (const auto& vertex : baseVertices)
  {
    coal::Vec3s transformed_vertex = castTransform_.transform(vertex);
    points->push_back(transformed_vertex);
  }
  num_points = points->size();
}

// Extract vertices based on shape type
std::vector<coal::Vec3s> CastHullShape::extractVertices(const coal::ShapeBase* geometry) const
{
  // Try to cast to specific shape types and extract vertices
  if (const auto* box = dynamic_cast<const coal::Box*>(geometry))
    return extractVerticesFromBox(box);
  if (const auto* sphere = dynamic_cast<const coal::Sphere*>(geometry))
    return extractVerticesFromSphere(sphere);
  if (const auto* cylinder = dynamic_cast<const coal::Cylinder*>(geometry))
    return extractVerticesFromCylinder(cylinder);
  if (const auto* cone = dynamic_cast<const coal::Cone*>(geometry))
    return extractVerticesFromCone(cone);
  if (const auto* capsule = dynamic_cast<const coal::Capsule*>(geometry))
    return extractVerticesFromCapsule(capsule);
  if (const auto* convex = dynamic_cast<const coal::ConvexBase32*>(geometry))
    return extractVerticesFromConvex(convex);

  // Ellipsoid
  // Halfspace
  // Plane

  // Fallback: use AABB corners
  coal::AABB aabb = geometry->aabb_local;

  std::vector<coal::Vec3s> corners;
  corners.reserve(8);
  corners.emplace_back(aabb.min_[0], aabb.min_[1], aabb.min_[2]);
  corners.emplace_back(aabb.max_[0], aabb.min_[1], aabb.min_[2]);
  corners.emplace_back(aabb.min_[0], aabb.max_[1], aabb.min_[2]);
  corners.emplace_back(aabb.max_[0], aabb.max_[1], aabb.min_[2]);
  corners.emplace_back(aabb.min_[0], aabb.min_[1], aabb.max_[2]);
  corners.emplace_back(aabb.max_[0], aabb.min_[1], aabb.max_[2]);
  corners.emplace_back(aabb.min_[0], aabb.max_[1], aabb.max_[2]);
  corners.emplace_back(aabb.max_[0], aabb.max_[1], aabb.max_[2]);

  return corners;
}

// Extract vertices from Box
std::vector<coal::Vec3s> CastHullShape::extractVerticesFromBox(const coal::Box* box) const
{
  const coal::Vec3s& half_side = box->halfSide;

  std::vector<coal::Vec3s> corners;
  corners.reserve(8);
  corners.emplace_back(-half_side[0], -half_side[1], -half_side[2]);
  corners.emplace_back(half_side[0], -half_side[1], -half_side[2]);
  corners.emplace_back(-half_side[0], half_side[1], -half_side[2]);
  corners.emplace_back(half_side[0], half_side[1], -half_side[2]);
  corners.emplace_back(-half_side[0], -half_side[1], half_side[2]);
  corners.emplace_back(half_side[0], -half_side[1], half_side[2]);
  corners.emplace_back(-half_side[0], half_side[1], half_side[2]);
  corners.emplace_back(half_side[0], half_side[1], half_side[2]);

  return corners;
}

// Extract vertices approximating a Sphere
std::vector<coal::Vec3s> CastHullShape::extractVerticesFromSphere(const coal::Sphere* sphere, int numPoints) const
{
  auto geom = tesseract_geometry::Sphere(sphere->radius);
  auto mesh = tesseract_geometry::toTriangleMesh(geom, 0.002);

  return *mesh->getVertices();
}

// Extract vertices approximating a Cylinder
std::vector<coal::Vec3s> CastHullShape::extractVerticesFromCylinder(const coal::Cylinder* cylinder, int numPoints) const
{
  auto geom = tesseract_geometry::Cylinder(cylinder->radius, cylinder->halfLength * 2.0);
  auto mesh = tesseract_geometry::toTriangleMesh(geom, 0.002);

  return *mesh->getVertices();
}

// Extract vertices approximating a Cone
std::vector<coal::Vec3s> CastHullShape::extractVerticesFromCone(const coal::Cone* cone, int numPoints) const
{
  auto geom = tesseract_geometry::Cone(cone->radius, cone->halfLength * 2.0);
  auto mesh = tesseract_geometry::toTriangleMesh(geom, 0.002);

  return *mesh->getVertices();
}

// Extract vertices approximating a Capsule
std::vector<coal::Vec3s> CastHullShape::extractVerticesFromCapsule(const coal::Capsule* capsule, int numPoints) const
{
  auto geom = tesseract_geometry::Capsule(capsule->radius, capsule->halfLength * 2.0);
  auto mesh = tesseract_geometry::toTriangleMesh(geom, 0.002);

  return *mesh->getVertices();
}

// Extract vertices from ConvexBase
std::vector<coal::Vec3s> CastHullShape::extractVerticesFromConvex(const coal::ConvexBase32* convex) const
{
  std::vector<coal::Vec3s> vertices;
  if (!convex->points->empty())
  {
    vertices = *convex->points;
  }

  return vertices;
}

}  // namespace tesseract_collision::tesseract_collision_coal
