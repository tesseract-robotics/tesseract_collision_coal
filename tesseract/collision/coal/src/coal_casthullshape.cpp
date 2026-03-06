/**
 * @file coal_casthullshape.cpp
 * @brief CastHullShape implementation.
 *
 * @author Roelof Oomen
 * @date Aug 04, 2025
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
#include <coal/narrowphase/support_functions.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_casthullshape.h>

namespace tesseract::collision::tesseract_collision_coal
{
CastHullShape::CastHullShape(std::shared_ptr<coal::ShapeBase> shape, const coal::Transform3s& castTransform)
  : shape_(std::move(shape))
  , castTransform_(castTransform)
  , castTransformInv_(coal::Transform3s(castTransform).inverse())
{
}

void CastHullShape::computeLocalAABB()
{
  int hint = 0;
  coal::details::ShapeSupportData data;

  for (int i = 0; i < 3; ++i)
  {
    coal::Vec3s dir = coal::Vec3s::Zero();
    coal::Vec3s s;

    dir[i] = 1;
    computeShapeSupport(dir, s, hint, data);
    aabb_local.max_[i] = s[i];

    dir[i] = -1;
    computeShapeSupport(dir, s, hint, data);
    aabb_local.min_[i] = s[i];
  }

  // Pad by CastHullShape's own swept-sphere radius (zero by default).
  // Note: the underlying shape's intrinsic radius (Sphere, Capsule, etc.) is
  // already included in the support points via WithSweptSphere mode in
  // computeShapeSupport, so no double-counting occurs here.
  const coal::Scalar r = getSweptSphereRadius();
  aabb_local.min_ -= coal::Vec3s::Constant(r);
  aabb_local.max_ += coal::Vec3s::Constant(r);

  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

// NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
CastHullShape* CastHullShape::clone() const { return new CastHullShape(shape_, castTransform_); }

double CastHullShape::computeVolume() const
{
  double baseVolume = shape_->computeVolume();

  coal::Vec3s translation = castTransform_.getTranslation();
  double translation_length = translation.norm();

  coal::Matrix3s rotation = castTransform_.getRotation();
  bool has_rotation = !rotation.isIdentity(1e-6);

  if (translation_length < 1e-6 && !has_rotation)
    return baseVolume;

  // Compute a conservative AABB via the support function and use its volume
  int hint = 0;
  coal::details::ShapeSupportData data;
  coal::AABB swept_aabb;

  for (int i = 0; i < 3; ++i)
  {
    coal::Vec3s dir = coal::Vec3s::Zero();
    coal::Vec3s s;

    dir[i] = 1;
    computeShapeSupport(dir, s, hint, data);
    swept_aabb.max_[i] = s[i];

    dir[i] = -1;
    computeShapeSupport(dir, s, hint, data);
    swept_aabb.min_[i] = s[i];
  }

  return std::max(baseVolume, swept_aabb.volume());
}

bool CastHullShape::isEqual(const coal::CollisionGeometry& _other) const
{
  const auto* other_ptr = dynamic_cast<const CastHullShape*>(&_other);
  if (other_ptr == nullptr)
    return false;

  return shape_ == other_ptr->shape_ && castTransform_ == other_ptr->castTransform_;
}

void CastHullShape::updateCastTransform(const coal::Transform3s& castTransform)
{
  castTransform_ = castTransform;
  castTransformInv_ = coal::Transform3s(castTransform).inverse();
  computeLocalAABB();
}

void CastHullShape::computeShapeSupport(const coal::Vec3s& dir,
                                        coal::Vec3s& support,
                                        int& hint,
                                        coal::details::ShapeSupportData& /*data*/) const
{
  // Support at pose 0 (shape in its local frame, identity transform).
  // Use WithSweptSphere so that shapes with intrinsic radii (Sphere, Capsule)
  // include that radius in the support point — necessary for correct swept-hull
  // geometry and matching the behavior of the former castHullGetSupportFunc.
  const coal::Vec3s s0 =
      coal::details::getSupport<coal::details::SupportOptions::WithSweptSphere>(shape_.get(), dir, hint);

  // Support at pose 1 (shape at castTransform_).
  // Rotate the query direction into the local frame of pose 1, compute support,
  // then transform the result back to the local frame of pose 0.
  const coal::Vec3s dir_local1 = castTransformInv_.getRotation() * dir;
  const coal::Vec3s s1_local =
      coal::details::getSupport<coal::details::SupportOptions::WithSweptSphere>(shape_.get(), dir_local1, hint);
  const coal::Vec3s s1 = castTransform_.transform(s1_local);

  // Return the support of the convex hull of both poses (Schulman et al. 2013).
  // When projections are equal, favour pose 1 — matching Bullet's
  // btCastHullShape::localGetSupportingVertex (strict ">").
  support = (dir.dot(s0) > dir.dot(s1)) ? s0 : s1;
}

}  // namespace tesseract::collision::tesseract_collision_coal
