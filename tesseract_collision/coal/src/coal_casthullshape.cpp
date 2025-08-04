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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/common.h>

#include <tesseract_collision/coal/coal_collision_object_wrapper.h>
#include <tesseract_collision/coal/coal_casthullshape.h>
#include <tesseract_collision/coal/coal_casthullshape_utility.h>

namespace tesseract_collision::tesseract_collision_coal
{

coal::Vec3s CastHullShape::supportMapping(const coal::Vec3s& dir, bool* is_vertices_used) const
{
  // Early exit for zero direction
  if (dir.norm() < std::numeric_limits<double>::epsilon())
  {
    return { 0, 0, 0 };
  }

  // Regular support computation without optimizations
  coal::Vec3s result = computeSupportPoint(dir);

  if (is_vertices_used != nullptr)
    *is_vertices_used = true;
  return result;
}

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
  // Basic volume computation for swept shape
  double baseVolume = shape_->computeVolume();

  // Compute swept area
  shape_->computeLocalAABB();
  // Get the AABB dimensions from ShapeBase members
  coal::Vec3s min_point = shape_->aabb_center.array() - shape_->aabb_radius;
  coal::Vec3s max_point = shape_->aabb_center.array() + shape_->aabb_radius;
  coal::Vec3s dims = max_point - min_point;

  // Get translation vector
  coal::Vec3s translation = castTransform_.getTranslation();
  double translation_length = translation.norm();

  // Calculate simplified swept volume approximation
  // Just add the base volume and the volume of the swept portion
  return baseVolume + (dims[0] * dims[1] * translation_length);
}

// Implementation of public computeSweptVertices
void CastHullShape::computeSweptVertices()
{
  // Extract vertices from the underlying shape
  std::vector<coal::Vec3s> baseVertices = extractVertices(shape_.get());

  // Store both start and end positions
  swept_vertices_.clear();
  swept_vertices_.reserve(baseVertices.size() * 2);

  // Also update the points member from ConvexBase
  points->clear();
  points->reserve(baseVertices.size() * 2);

  // Add vertices at starting position
  for (const auto& vertex : baseVertices)
  {
    swept_vertices_.push_back(vertex);
    points->push_back(vertex);
  }

  // Add vertices at ending position (after transform)
  for (const auto& vertex : baseVertices)
  {
    coal::Vec3s transformed_vertex = (castTransform_ * vertex).translation();
    swept_vertices_.push_back(transformed_vertex);
    points->push_back(transformed_vertex);
  }
  num_points = points->size();
}

// Extract vertices based on shape type
std::vector<coal::Vec3s> CastHullShape::extractVertices(const coal::CollisionGeometry* geometry) const
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
  if (const auto* convex = dynamic_cast<const coal::ConvexBase32*>(geometry))
    return extractVerticesFromConvex(convex);

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
  const double radius = sphere->radius;
  std::vector<coal::Vec3s> vertices;

  // A simple approximation using points on the axes
  vertices.emplace_back(radius, 0, 0);
  vertices.emplace_back(-radius, 0, 0);
  vertices.emplace_back(0, radius, 0);
  vertices.emplace_back(0, -radius, 0);
  vertices.emplace_back(0, 0, radius);
  vertices.emplace_back(0, 0, -radius);

  // Add some points around equator
  const double step = 2.0 * M_PI / (numPoints - 6);
  for (int i = 0; i < numPoints - 6; ++i)
  {
    double angle = i * step;
    vertices.emplace_back(radius * cos(angle), radius * sin(angle), 0);
  }

  return vertices;
}

// Extract vertices approximating a Cylinder
std::vector<coal::Vec3s> CastHullShape::extractVerticesFromCylinder(const coal::Cylinder* cylinder, int numPoints) const
{
  const double radius = cylinder->radius;
  const double halfHeight = cylinder->halfLength;
  std::vector<coal::Vec3s> vertices;
  vertices.reserve(static_cast<long>(numPoints) * 2);

  // Generate points around top and bottom circles
  const double step = 2.0 * M_PI / numPoints;
  for (int i = 0; i < numPoints; ++i)
  {
    double angle = i * step;
    double x = radius * cos(angle);
    double y = radius * sin(angle);

    // Top circle point
    vertices.emplace_back(x, y, halfHeight);
    // Bottom circle point
    vertices.emplace_back(x, y, -halfHeight);
  }

  return vertices;
}

// Extract vertices approximating a Cone
std::vector<coal::Vec3s> CastHullShape::extractVerticesFromCone(const coal::Cone* cone, int numPoints) const
{
  const double radius = cone->radius;
  const double halfHeight = cone->halfLength;
  std::vector<coal::Vec3s> vertices;
  vertices.reserve(numPoints + 1);

  // Apex of the cone (assuming centered at origin with apex at +Z)
  vertices.emplace_back(0, 0, halfHeight);

  // Generate points around base circle
  const double step = 2.0 * M_PI / numPoints;
  for (int i = 0; i < numPoints; ++i)
  {
    double angle = i * step;
    double x = radius * cos(angle);
    double y = radius * sin(angle);
    vertices.emplace_back(x, y, -halfHeight);
  }

  return vertices;
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

coal::Vec3s CastHullShape::computeSupportPoint(const coal::Vec3s& dir) const
{
  // Initialize support points
  coal::Vec3s supportStart;
  coal::Vec3s supportEnd;
  coal::Vec3s supportEndLocal;

  // Variables required by getShapeSupport
  int hint = 0;  // Used primarily for ConvexBase
  coal::details::ShapeSupportData support_data;

  // Transform the direction for end support
  coal::Vec3s transformedDir = castTransformInv_.getRotation() * dir;

  // Try to cast to specific shape types and use getShapeSupport
  if (const auto* box = dynamic_cast<const coal::Box*>(shape_.get()))
  {
    coal::details::getShapeSupport<coal::details::SupportOptions::NoSweptSphere>(
        box, dir, supportStart, hint, support_data);

    coal::details::getShapeSupport<coal::details::SupportOptions::NoSweptSphere>(
        box, transformedDir, supportEndLocal, hint, support_data);
  }
  else if (const auto* sphere = dynamic_cast<const coal::Sphere*>(shape_.get()))
  {
    coal::details::getShapeSupport<coal::details::SupportOptions::NoSweptSphere>(
        sphere, dir, supportStart, hint, support_data);

    coal::details::getShapeSupport<coal::details::SupportOptions::NoSweptSphere>(
        sphere, transformedDir, supportEndLocal, hint, support_data);
  }
  else if (const auto* cylinder = dynamic_cast<const coal::Cylinder*>(shape_.get()))
  {
    coal::details::getShapeSupport<coal::details::SupportOptions::NoSweptSphere>(
        cylinder, dir, supportStart, hint, support_data);

    coal::details::getShapeSupport<coal::details::SupportOptions::NoSweptSphere>(
        cylinder, transformedDir, supportEndLocal, hint, support_data);
  }
  else if (const auto* cone = dynamic_cast<const coal::Cone*>(shape_.get()))
  {
    coal::details::getShapeSupport<coal::details::SupportOptions::NoSweptSphere>(
        cone, dir, supportStart, hint, support_data);

    coal::details::getShapeSupport<coal::details::SupportOptions::NoSweptSphere>(
        cone, transformedDir, supportEndLocal, hint, support_data);
  }
  else if (const auto* convex = dynamic_cast<const coal::ConvexBase32*>(shape_.get()))
  {
    coal::details::getShapeSupport<coal::details::SupportOptions::NoSweptSphere>(
        convex, dir, supportStart, hint, support_data);

    // Reset hint for second call (important for ConvexBase)
    hint = 0;
    coal::details::getShapeSupport<coal::details::SupportOptions::NoSweptSphere>(
        convex, transformedDir, supportEndLocal, hint, support_data);
  }
  else
  {
    // Fallback to AABB-based support for unknown shape types
    shape_->computeLocalAABB();

    coal::Vec3s min_point = shape_->aabb_center.array() - shape_->aabb_radius;
    coal::Vec3s max_point = shape_->aabb_center.array() + shape_->aabb_radius;

    supportStart = { (dir[0] >= 0) ? max_point[0] : min_point[0],
                     (dir[1] >= 0) ? max_point[1] : min_point[1],
                     (dir[2] >= 0) ? max_point[2] : min_point[2] };

    supportEndLocal = { (transformedDir[0] >= 0) ? max_point[0] : min_point[0],
                        (transformedDir[1] >= 0) ? max_point[1] : min_point[1],
                        (transformedDir[2] >= 0) ? max_point[2] : min_point[2] };
  }

  // Transform the local end support to global coordinates
  supportEnd = (castTransform_ * supportEndLocal).translation();

  // Return the point with maximum projection in the direction
  double dotStart = dir.dot(supportStart);
  double dotEnd = dir.dot(supportEnd);

  return (dotStart > dotEnd) ? supportStart : supportEnd;
}

}  // namespace tesseract_collision::tesseract_collision_coal
