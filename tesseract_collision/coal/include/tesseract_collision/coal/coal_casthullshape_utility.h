/**
 * @file coal_casthullshape_utility.h
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

#ifndef TESSERACT_COLLISION_COAL_CASTHULLSHAPE_UTILITY_H
#define TESSERACT_COLLISION_COAL_CASTHULLSHAPE_UTILITY_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <coal/shape/geometric_shapes_utility.h>
#include <coal/broadphase/broadphase_collision_manager.h>
#include <coal/collision.h>
#include <coal/distance.h>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/common.h>
#include <tesseract_collision/coal/coal_collision_object_wrapper.h>

#include <tesseract_collision/coal/coal_casthullshape.h>

namespace coal
{

using tesseract_collision::tesseract_collision_coal::CastHullShape;

template <>
void computeBV<coal::AABB, CastHullShape>(const CastHullShape& s, const coal::Transform3s& tf, coal::AABB& bv);

namespace details
{

/// @brief CastHullShape support function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>  // NOLINT(bugprone-reserved-identifier)
void getShapeSupport(const CastHullShape* shape,
                     const Vec3s& dir,
                     Vec3s& support,
                     int& hint,
                     ShapeSupportData& /*unused*/);

template <int _SupportOptions>  // NOLINT(bugprone-reserved-identifier)
void getShapeSupportSet(const CastHullShape* cast_hull_shape,
                        SupportSet& support_set,
                        int& hint,
                        ShapeSupportData& support_data,
                        size_t num_sampled_supports,
                        Scalar tol);

}  // namespace details

}  // namespace coal
#endif  // TESSERACT_COLLISION_COAL_CASTHULLSHAPE_UTILITY_H
