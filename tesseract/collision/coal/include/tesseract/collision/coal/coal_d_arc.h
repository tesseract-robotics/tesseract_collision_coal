/**
 * @file coal_d_arc.h
 * @brief Arc-sagitta compensation for swept shapes.
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

#ifndef TESSERACT_COLLISION_COAL_D_ARC_H
#define TESSERACT_COLLISION_COAL_D_ARC_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <coal/math/transform.h>
#include <coal/shape/geometric_shapes.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract::collision::tesseract_collision_coal
{
/// Precomputed rotation-angle scalars for d_arc computation.
/// These depend only on the rotation angle, which is conjugation-invariant
/// and therefore identical for all shapes on the same link — computed once
/// from the link-level relative rotation before the per-shape loop. The same invariance is what
/// makes `cos_phi` safe as a branch selector: a per-shape conjugation preserves the trace, so every
/// shape on a link lands on the same side of the axis-extraction handoff as the link itself.
struct DArcScalars
{
  double sagitta_factor{ 0.0 };  ///< 1 - cos(phi/2); zero means negligible rotation.
  double inv_4sin2_half{ 0.0 };  ///< 1 / (2*(1 - cos_phi)) == 1 / (4*sin^2(phi/2)); scales the raw skew vector.
  double cos_phi{ 0.0 };         ///< (trace(R) - 1) / 2; selects the axis-extraction branch.
};

/// Compute the rotation-angle scalars from a link-level cast transform.
/// All trig is avoided via half-angle identities on the rotation matrix trace.
/// Returns zero-initialized scalars when the rotation is negligible (phi < ~1e-7 rad).
DArcScalars computeDArcScalars(const coal::Transform3s& link_cast_tf);

/// Compute the arc-chord sagitta (d_arc) for a single shape, given precomputed scalars.
/// d_arc = r_max * (1 - cos(phi/2)), where r_max is the maximum distance from any
/// point on the shape's bounding sphere to the screw axis.
/// @param cast_tf The shape's motion over the sweep, in the shape's own frame
/// @param shape The shape, read for its bounding sphere
/// @param s The result of computeDArcScalars for a cast transform with the same rotation angle as
/// @p cast_tf: the link-level one, or @p cast_tf itself
/// @pre `shape.aabb_radius >= 0`: the shape's local bound has been computed. Checked by an assert,
/// which a release build drops.
double computeDArc(const coal::Transform3s& cast_tf, const coal::ShapeBase& shape, const DArcScalars& s);

}  // namespace tesseract::collision::tesseract_collision_coal
#endif  // TESSERACT_COLLISION_COAL_D_ARC_H
