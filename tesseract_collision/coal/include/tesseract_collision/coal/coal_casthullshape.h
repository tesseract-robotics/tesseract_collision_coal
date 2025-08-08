/**
 * @file coal_casthullshape.h
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

#ifndef TESSERACT_COLLISION_COAL_CASTHULLSHAPE_H
#define TESSERACT_COLLISION_COAL_CASTHULLSHAPE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <utility>
#include <console_bridge/console.h>
#include <coal/shape/geometric_shapes.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/common.h>
#include <tesseract_collision/coal/coal_collision_object_wrapper.h>

namespace tesseract_collision::tesseract_collision_coal
{

class CastHullShape : public coal::ConvexBase32
{
public:
  CastHullShape(std::shared_ptr<coal::ShapeBase> shape, const coal::Transform3s& castTransform)
    : shape_(std::move(shape))
    , castTransform_(castTransform)
    , castTransformInv_(coal::Transform3s(castTransform).inverse())
  {
    // Make sure points member from ConvexBase is properly initialized
    if (!points)
      points = std::make_shared<std::vector<coal::Vec3s>>();

    // Compute swept vertices
    computeSweptVertices();
  }

  void computeLocalAABB() override;

  // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
  coal::ConvexBase32* clone() const override { return new CastHullShape(*this); }

  double computeVolume() const override;

  bool isEqual(const coal::CollisionGeometry& _other) const override
  {
    const auto* other_ptr = dynamic_cast<const CastHullShape*>(&_other);
    if (other_ptr == nullptr)
      return false;
    const CastHullShape& other = *other_ptr;

    return shape_ == other.shape_ && castTransform_ == other.castTransform_;
  }

  void updateCastTransform(const coal::Transform3s& castTransform)
  {
    castTransform_ = castTransform;
    castTransformInv_ = coal::Transform3s(castTransform).inverse();
    computeSweptVertices();
  }

  // Add these methods to the CastHullShape class declaration
  const std::shared_ptr<coal::ShapeBase>& getUnderlyingShape() const { return shape_; }

  const coal::Transform3s& getCastTransform() const { return castTransform_; }

  const coal::Transform3s& getCastTransformInverse() const { return castTransformInv_; }

  void computeSweptVertices();

private:
  std::shared_ptr<coal::ShapeBase> shape_;
  coal::Transform3s castTransform_;
  coal::Transform3s castTransformInv_;

  // Helper methods to extract vertices based on shape type
  std::vector<coal::Vec3s> extractVertices(const coal::ShapeBase* geometry) const;
  std::vector<coal::Vec3s> extractVerticesFromBox(const coal::Box* box) const;
  std::vector<coal::Vec3s> extractVerticesFromSphere(const coal::Sphere* sphere, int numPoints = 8) const;
  std::vector<coal::Vec3s> extractVerticesFromCylinder(const coal::Cylinder* cylinder, int numPoints = 8) const;
  std::vector<coal::Vec3s> extractVerticesFromCone(const coal::Cone* cone, int numPoints = 8) const;
  std::vector<coal::Vec3s> extractVerticesFromCapsule(const coal::Capsule* capsule, int numPoints = 8) const;
  std::vector<coal::Vec3s> extractVerticesFromConvex(const coal::ConvexBase32* convex) const;
};

}  // namespace tesseract_collision::tesseract_collision_coal
#endif  // TESSERACT_COLLISION_COAL_CASTHULLSHAPE_H
