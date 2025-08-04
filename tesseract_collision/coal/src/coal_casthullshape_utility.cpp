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
  // We need to access the shape's swept vertices
  // First, ensure swept vertices are up-to-date
  const_cast<CastHullShape&>(s).computeSweptVertices();

  // Get the swept vertices
  const auto& swept_vertices = s.getSweptVertices();

  if (swept_vertices.empty())
  {
    // Handle empty case - set to zero-sized AABB at transform origin
    const coal::Vec3s& origin = tf.getTranslation();
    bv.min_ = origin;
    bv.max_ = origin;
    return;
  }

  // Initialize min/max with the transformed first vertex
  coal::Vec3s tmin = (tf * swept_vertices[0]).translation();
  coal::Vec3s tmax = tmin;

  // Find the AABB that encloses all transformed swept vertices
  for (size_t i = 1; i < swept_vertices.size(); ++i)
  {
    coal::Vec3s transformed_vertex = (tf * swept_vertices[i]).translation();
    tmin = tmin.cwiseMin(transformed_vertex);
    tmax = tmax.cwiseMax(transformed_vertex);
  }

  // Set the output bounding volume
  bv.min_ = tmin;
  bv.max_ = tmax;
}

}  // namespace coal
