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
 * the underlying shape's exact support at both poses, or once where the cast
 * transform is the identity, so no convex hull vertices are materialized. This
 * mirrors Bullet's btCastHullShape design.
 */
class CastHullShape : public coal::ShapeBase
{
public:
  CastHullShape(std::shared_ptr<coal::ShapeBase> shape, const coal::Transform3s& castTransform);

  /// @brief Copy everything but the scratch hull: the copy has none until its own scratchHull() makes it.
  CastHullShape(const CastHullShape& other);
  ~CastHullShape() override = default;
  CastHullShape& operator=(const CastHullShape&) = delete;
  CastHullShape(CastHullShape&&) = delete;
  CastHullShape& operator=(CastHullShape&&) = delete;

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

  /// @brief Return this hull to the state of a shape that has not been swept: identity cast transform and
  /// no swept-sphere inflation. computeLocalAABB reads both, so clearing one without the other leaves a
  /// hull whose bounds still describe a sweep. Returns whether anything changed.
  bool clearSweep();

  void computeShapeSupport(const coal::Vec3s& dir,
                           coal::Vec3s& support,
                           int& hint,
                           coal::details::ShapeSupportData& data) const override;

  const std::shared_ptr<coal::ShapeBase>& getUnderlyingShape() const { return shape_; }

  const coal::Transform3s& getCastTransform() const { return castTransform_; }

  /// @brief Whether the cast transform is exactly the identity: the hull holds one pose.
  bool isCastIdentity() const { return cast_is_identity_; }

  /// @brief Half the diagonal of the wrapped shape's own bounding box, its swept-sphere radius left out: a
  /// measure of the shape's size that no sweep changes. Fixed when the hull is made.
  coal::Scalar getShapeBoundRadius() const { return 0.5 * (wrapped_aabb_.max_ - wrapped_aabb_.min_).norm(); }

  /// @brief A second hull over the same shape, for a caller that needs the shape swept by a motion other
  /// than this hull's own. The same object on every call, made on the first, and unswept then: it takes
  /// neither this hull's cast transform nor its swept-sphere radius. This class never reads or writes its
  /// sweep afterwards: it holds whatever the caller wrote last, so a caller must write the sweep it wants
  /// before each use. A copy of this hull does not carry it.
  CastHullShape& scratchHull() const;

  /// @brief Accessors for the GJK sweep's mutable vertex hints and
  /// ShapeSupportData. After GJK converges, the hint vertex and its last_dir are
  /// high-quality starting points for support queries along related directions
  /// (e.g. the contact normal), so callers may read them to seed their own
  /// queries. Callers should climb on their own scratch rather than mutating
  /// these, so they do not perturb the sweep's warm-start chain. While the cast
  /// transform is exactly the identity, queries run on the pose-1 hint and data
  /// alone and the pose-0 ones are stale: seed from the pose-1 ones. Writing a
  /// sweep to such a hull copies the pose-1 hint and last_dir to pose 0.
  int& getHint0() const { return hint0_; }
  int& getHint1() const { return hint1_; }
  coal::details::ShapeSupportData& getSupportData0() const { return support_data0_; }
  coal::details::ShapeSupportData& getSupportData1() const { return support_data1_; }

private:
  /// @brief Support of the two-pose convex hull along @p dir, hill-climbing
  /// from the caller's per-pose hint/data pairs. The virtual override forwards
  /// the mutable warm-start members; state-neutral queries pass local scratch.
  void computeShapeSupport(const coal::Vec3s& dir,
                           coal::Vec3s& support,
                           int& hint0,
                           int& hint1,
                           coal::details::ShapeSupportData& data0,
                           coal::details::ShapeSupportData& data1) const;

  std::shared_ptr<coal::ShapeBase> shape_;
  coal::Transform3s castTransform_;
  /// Whether castTransform_ is exactly the identity. Every writer of castTransform_ keeps it current.
  bool cast_is_identity_;

  /// See scratchHull(). Mutable for the same reason as the support hints below: made lazily, per
  /// instance, and never shared between threads.
  mutable std::unique_ptr<CastHullShape> scratch_hull_;

  /// @brief The wrapped shape's local AABB with its swept-sphere radius removed, captured at
  /// construction. computeLocalAABB re-inflates it by the shape's live radius rather than
  /// reading shape_->aabb_local, which coal refreshes only on computeLocalAABB() and which
  /// setSweptSphereRadius() leaves stale -- a stale one would make the cast bound smaller
  /// than the shape, and the broadphase would drop the pair. This bound needs no refreshing
  /// because it does not depend on the radius at all.
  coal::AABB wrapped_aabb_;

  /// Separate support function vertex hints and data for pose 0 and pose 1.
  /// Each getSupport call uses hill-climbing from the hint, so sharing a single
  /// hint between the two poses (which query different directions) would cause
  /// each to corrupt the other's warm-start. The ShapeSupportData holds the
  /// visited-vertex buffer reused across calls to avoid per-call allocation.
  /// @note Not thread-safe: each thread must use its own CastHullShape instance.
  /// The collision managers ensure this via per-thread clone().
  mutable int hint0_{ 0 };
  mutable int hint1_{ 0 };
  mutable coal::details::ShapeSupportData support_data0_;
  mutable coal::details::ShapeSupportData support_data1_;
};

}  // namespace tesseract::collision::tesseract_collision_coal
#endif  // TESSERACT_COLLISION_COAL_CASTHULLSHAPE_H
