/**
 * @file coal_d_arc.cpp
 * @brief Arc-sagitta compensation for swept shapes: the screw axis of a cast transform and the sagitta
 * of a shape's arc about it, evaluated without trigonometric calls.
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
#include <algorithm>
#include <cassert>
#include <cmath>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_d_arc.h>

namespace tesseract::collision::tesseract_collision_coal
{
/// Rotation axis of `R`, read from its symmetric part:
/// `R + R^T == 2*cos(phi)*I + 2*(1 - cos(phi))*k*k^T`. Used past 120 degrees, where the
/// skew-symmetric part -- which is `2*sin(phi)*k` and gives the axis directly below that -- shrinks
/// toward zero as phi approaches pi, taking the precision of its direction with it, and is exactly
/// zero at a half turn. The two extractions are well conditioned across a wide overlap, so the
/// handoff is not a cliff.
static coal::Vec3s screwAxisFromRotation(const coal::Matrix3s& R, double cos_phi)
{
  const coal::Matrix3s outer = (R + R.transpose()) * 0.5 - cos_phi * coal::Matrix3s::Identity();

  // Columns of k*k^T are k scaled by each k_i, so the largest-diagonal column is the best determined.
  // It cannot vanish: some k_i^2 is at least 1/3, and this extraction runs only past the handoff,
  // where 1 - cos_phi is at least 1.5, so `col` has norm at least 1.5/sqrt(3). The guard below holds
  // only for a caller that ignores that precondition.
  Eigen::Index j = 0;
  outer.diagonal().maxCoeff(&j);
  const coal::Vec3s col = outer.col(j);
  const double n = col.norm();
  if (n == 0.0)
    return coal::Vec3s::UnitX();  // LCOV_EXCL_LINE

  // k*k^T loses the sign. Where the skew part still carries one, agree with it; where it does not,
  // cot(phi/2) has gone to zero with it and nothing downstream can tell the two apart.
  const coal::Vec3s k = col / n;
  const double sign = (R(2, 1) - R(1, 2)) * k.x() + (R(0, 2) - R(2, 0)) * k.y() + (R(1, 0) - R(0, 1)) * k.z();
  return (sign < 0.0) ? coal::Vec3s(-k) : k;
}

DArcScalars computeDArcScalars(const coal::Transform3s& link_cast_tf)
{
  const coal::Matrix3s& R = link_cast_tf.getRotation();
  const double cos_phi = std::clamp((R.trace() - 1.0) * 0.5, -1.0, 1.0);
  const double one_plus_cos = 1.0 + cos_phi;
  if (one_plus_cos > 2.0 - 1e-14)
    return {};
  const double cos_half = std::sqrt(one_plus_cos * 0.5);
  // Singular only at phi = 0, which the early return above already excludes. sin_half is not needed
  // on the fast path at all: the raw skew vector carries the 2*sin(phi) scale, and this factor
  // absorbs both that and cot(phi/2).
  return { 1.0 - cos_half, 1.0 / (2.0 * (1.0 - cos_phi)), cos_phi };
}

double computeDArc(const coal::Transform3s& cast_tf, const coal::ShapeBase& shape, const DArcScalars& s)
{
  // The sagitta reads the shape's cached bound, which wrapping it does not populate. An unbounded
  // shape carries radius -1, reaching the broadphase as a NaN AABB: pairs drop unobserved.
  assert(shape.aabb_radius >= 0.0);

  if (s.sagitta_factor == 0.0)
    return 0.0;

  // Screw axis: closest point to the origin in the shape-local frame.
  // c = t_perp/2 + (k x t_perp) * cos(phi/2) / (2*sin(phi/2))
  const coal::Matrix3s& R = cast_tf.getRotation();
  const coal::Vec3s& t = cast_tf.getTranslation();

  if (s.cos_phi > -0.5)
  {
    // The skew part is used unnormalised: it appears only inside projections, which are scale-free
    // once divided by |w|^2, and scaled by inv_4sin2_half, which absorbs both the 2*sin(phi) and
    // the cot(phi/2). What keeps 1/|w|^2 finite is computeDArcScalars' early return, which rejects
    // phi below ~1.4e-7 rad -- |w| = 2*sin(phi) is not otherwise bounded away from zero here.
    const coal::Vec3s w(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
    const double inv_ww = 1.0 / w.squaredNorm();
    const coal::Vec3s t_perp = t - (t.dot(w) * inv_ww) * w;
    const coal::Vec3s c = 0.5 * t_perp + 0.5 * s.inv_4sin2_half * w.cross(t_perp);
    const coal::Vec3s pc = shape.aabb_center - c;
    const double dist_to_axis = (pc - (pc.dot(w) * inv_ww) * w).norm();
    return (dist_to_axis + shape.aabb_radius) * s.sagitta_factor;
  }

  // Past the handoff the skew part shrinks toward zero as phi approaches pi and its direction
  // degrades with it, reaching exactly zero at a half turn -- where a 1/(2*sin(phi)) scale would be
  // +inf and the product 0 * inf a NaN, which coal's `radius < 0` validation passes straight
  // through. Read the symmetric part instead, and recover cot(phi/2) here rather than carrying it,
  // since this branch is rare.
  const coal::Vec3s k = screwAxisFromRotation(R, s.cos_phi);
  const double sin_half = std::sqrt((1.0 - s.cos_phi) * 0.5);
  const double cos_half = 1.0 - s.sagitta_factor;
  const coal::Vec3s t_perp = t - t.dot(k) * k;
  const coal::Vec3s c = 0.5 * t_perp + 0.5 * (cos_half / sin_half) * k.cross(t_perp);
  const coal::Vec3s pc = shape.aabb_center - c;
  const double dist_to_axis = (pc - pc.dot(k) * k).norm();
  return (dist_to_axis + shape.aabb_radius) * s.sagitta_factor;
}

}  // namespace tesseract::collision::tesseract_collision_coal
