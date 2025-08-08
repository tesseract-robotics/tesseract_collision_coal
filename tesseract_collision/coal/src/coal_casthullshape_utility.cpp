/**
 * @file coal_casthullshape_utility.cpp
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

#include <tesseract_collision/coal/coal_casthullshape_utility.h>

namespace coal
{

template <>
void computeBV<coal::AABB, CastHullShape>(const CastHullShape& s, const coal::Transform3s& tf, coal::AABB& bv)
{
  const auto& shape = s.getUnderlyingShape().get();
  const auto& castTransform = s.getCastTransform();

  coal::AABB bv_original;
  coal::AABB bv_cast;

  // Try to cast to specific shape types
  if (const auto* box = dynamic_cast<const coal::Box*>(shape))
  {
    coal::computeBV<coal::AABB>(*box, tf, bv_original);
    coal::computeBV<coal::AABB>(*box, tf * castTransform, bv_cast);
  }
  else if (const auto* sphere = dynamic_cast<const coal::Sphere*>(shape))
  {
    coal::computeBV<coal::AABB>(*sphere, tf, bv_original);
    coal::computeBV<coal::AABB>(*sphere, tf * castTransform, bv_cast);
  }
  else if (const auto* cylinder = dynamic_cast<const coal::Cylinder*>(shape))
  {
    coal::computeBV<coal::AABB>(*cylinder, tf, bv_original);
    coal::computeBV<coal::AABB>(*cylinder, tf * castTransform, bv_cast);
  }
  else if (const auto* cone = dynamic_cast<const coal::Cone*>(shape))
  {
    coal::computeBV<coal::AABB>(*cone, tf, bv_original);
    coal::computeBV<coal::AABB>(*cone, tf * castTransform, bv_cast);
  }
  else if (const auto* capsule = dynamic_cast<const coal::Capsule*>(shape))
  {
    coal::computeBV<coal::AABB>(*capsule, tf, bv_original);
    coal::computeBV<coal::AABB>(*capsule, tf * castTransform, bv_cast);
  }
  else if (const auto* convex = dynamic_cast<const coal::ConvexBase32*>(shape))
  {
    coal::computeBV<coal::AABB>(*convex, tf, bv_original);
    coal::computeBV<coal::AABB>(*convex, tf * castTransform, bv_cast);
  }
  else if (const auto* ellipsoid = dynamic_cast<const coal::Ellipsoid*>(shape))
  {
    coal::computeBV<coal::AABB>(*ellipsoid, tf, bv_original);
    coal::computeBV<coal::AABB>(*ellipsoid, tf * castTransform, bv_cast);
  }

  // Combine both AABBs to get the swept volume AABB
  bv = bv_original + bv_cast;
}

namespace details
{

template <int _SupportOptions>
void getShapeSupport(const CastHullShape* cast_hull_shape,
                     const Vec3s& dir,
                     Vec3s& support,
                     int& hint,
                     ShapeSupportData& support_data)
{
  const auto& shape = cast_hull_shape->getUnderlyingShape().get();
  const auto& castTransform = cast_hull_shape->getCastTransform();
  const auto& castTransformInv = cast_hull_shape->getCastTransformInverse();

  // Initialize support points
  coal::Vec3s supportStart;
  coal::Vec3s supportEnd;
  coal::Vec3s supportEndLocal;

  // Transform the direction for end support
  coal::Vec3s transformedDir = castTransformInv.getRotation() * dir;

  supportStart = coal::details::getSupport<coal::details::SupportOptions::NoSweptSphere>(shape, dir, hint);
  supportEndLocal =
      coal::details::getSupport<coal::details::SupportOptions::NoSweptSphere>(shape, transformedDir, hint);

  // Transform the local end support to global coordinates
  supportEnd = castTransform.transform(supportEndLocal);

  // Return the point with maximum projection in the direction
  double dotStart = dir.dot(supportStart);
  double dotEnd = dir.dot(supportEnd);

  support = (dotStart > dotEnd) ? supportStart : supportEnd;
}

template <int _SupportOptions>
void getShapeSupportSet(const CastHullShape* cast_hull_shape,
                        SupportSet& support_set,
                        int& hint,
                        ShapeSupportData& support_data,
                        size_t num_sampled_supports,
                        Scalar tol)
{
  // TOODO: To be implemented
}

}  // namespace details

}  // namespace coal
