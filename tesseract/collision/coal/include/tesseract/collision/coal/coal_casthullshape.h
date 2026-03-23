/**
 * @file coal_casthullshape.h
 * @brief CastHullShape: a lightweight wrapper for continuous collision detection.
 *
 * Analogous to Bullet's btCastHullShape, this wraps an underlying coal::ShapeBase
 * and a cast transform representing the relative motion from pose t=0 to t=1.
 * The narrowphase uses the Schulman support function which defers to the underlying
 * shape's exact support, so no convex hull or vertex tessellation is needed.
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

#ifndef TESSERACT_COLLISION_COAL_CASTHULLSHAPE_H
#define TESSERACT_COLLISION_COAL_CASTHULLSHAPE_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <coal/narrowphase/minkowski_difference.h>
#include <coal/shape/geometric_shapes.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/types.h>
#include <tesseract/collision/common.h>
#include <tesseract/collision/coal/coal_collision_object_wrapper.h>

namespace tesseract::collision::tesseract_collision_coal
{
/**
 * @brief A lightweight swept-shape wrapper for continuous collision detection.
 *
 * Wraps an underlying coal::ShapeBase and a cast transform (relative motion from
 * t=0 to t=1). The narrowphase uses the Schulman support function which queries
 * the underlying shape's exact support at both poses, so no convex hull vertices
 * are materialized. This mirrors Bullet's btCastHullShape design.
 */
class CastHullShape : public coal::ShapeBase
{
public:
  CastHullShape(std::shared_ptr<coal::ShapeBase> shape, const coal::Transform3s& castTransform);

  void computeLocalAABB() override;

  // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
  CastHullShape* clone() const override;

  coal::NODE_TYPE getNodeType() const override { return coal::GEOM_CUSTOM; }

  /// @brief Delegate to the underlying shape via shape_traits lookup.
  bool needNesterovNormalizeHeuristic() const override
  {
    return coal::details::getNormalizeSupportDirection(shape_.get());
  }

  double computeVolume() const override;

  bool isEqual(const coal::CollisionGeometry& _other) const override;

  void updateCastTransform(const coal::Transform3s& castTransform);

  void computeShapeSupport(const coal::Vec3s& dir,
                           coal::Vec3s& support,
                           int& hint,
                           coal::details::ShapeSupportData& data) const override;

  const std::shared_ptr<coal::ShapeBase>& getUnderlyingShape() const { return shape_; }

  const coal::Transform3s& getCastTransform() const { return castTransform_; }

  const coal::Transform3s& getCastTransformInverse() const { return castTransformInv_; }

private:
  std::shared_ptr<coal::ShapeBase> shape_;
  coal::Transform3s castTransform_;
  coal::Transform3s castTransformInv_;

  /// Separate support function vertex hints for pose 0 and pose 1.
  /// Each getSupport call uses hill-climbing from the hint, so sharing a single
  /// hint between the two poses (which query different directions) would cause
  /// each to corrupt the other's warm-start.
  /// @note Not thread-safe: each thread must use its own CastHullShape instance.
  /// The collision managers ensure this via per-thread clone().
  mutable int hint0_{ 0 };
  mutable int hint1_{ 0 };
};

}  // namespace tesseract::collision::tesseract_collision_coal
#endif  // TESSERACT_COLLISION_COAL_CASTHULLSHAPE_H
