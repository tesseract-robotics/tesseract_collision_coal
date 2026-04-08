/**
 * @file coal_collision_object_wrapper.h
 * @brief Collision Object Wrapper to modify AABB with contact distance threshold
 *
 * @author Levi Armstrong
 * @date April 14, 2020
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_COLLISION_COAL_COLLISION_OBJECT_WRAPPER_H
#define TESSERACT_COLLISION_COAL_COLLISION_OBJECT_WRAPPER_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <coal/collision_object.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract::collision::tesseract_collision_coal
{
/**
 * @brief This is a wrapper around Coal Collision Object Class which allows you to expand the AABB by the contact
 * dist.
 *
 * This significantly improves performance when making distance requests if performing a contact tests type FIRST.
 */
class CoalCollisionObjectWrapper : public coal::CollisionObject
{
public:
  using coal::CollisionObject::CollisionObject;

  /**
   * @brief Set the collision objects contact distance threshold.
   *
   * This automatically calls updateAABB() which increases the AABB by the contact distance.
   * @param contact_distance The contact distance threshold.
   */
  void setContactDistanceThreshold(double contact_distance);

  /**
   * @brief Get the collision objects contact distance threshold.
   * @return The contact distance threshold.
   */
  double getContactDistanceThreshold() const;

  /**
   * @brief Update the internal AABB. This must be called instead of the base class computeAABB().
   *
   * After setting the collision objects transform this must be called.
   */
  void updateAABB();

  /**
   * @brief Set the shape index. This is the geometries index in the urdf.
   * @param index The index
   */
  void setShapeIndex(int index);

  /**
   * @brief Get the shape index. This is the geometries index in the urdf.
   * @return The shape index
   */
  int getShapeIndex() const;

  /**
   * @brief Set the source geometry index used for contact result reporting.
   *
   * This may differ from shape_index_ when an input geometry is expanded into
   * multiple collision proxies (e.g., octree voxels).
   * @param index The source geometry index in the original link geometry list.
   */
  void setSourceShapeIndex(int index);

  /**
   * @brief Get the source geometry index for contact result reporting.
   *
   * Falls back to shape_index_ when an explicit source index has not been set.
   * @return The source geometry index.
   */
  int getSourceShapeIndex() const;

  /**
   * @brief Set the source primitive index used for contact subshape reporting.
   *
   * This is used when one input geometry is expanded into multiple collision
   * proxies and Coal's native contact primitive id no longer refers to the
   * original geometry primitive identity.
   * @param index The source primitive index in the original geometry.
   */
  void setSourceSubshapeIndex(int index);

  /**
   * @brief Get the source primitive index for contact subshape reporting.
   *
   * Returns -1 when no explicit source primitive mapping exists.
   * @return The source primitive index or -1.
   */
  int getSourceSubshapeIndex() const;

protected:
  double contact_distance_{ 0 }; /**< @brief The contact distance threshold. */

  /** @brief The shape index, which is the geometries index in the urdf. */
  int shape_index_{ -1 };

  /** @brief The source geometry index used when reporting contact shape ids. */
  int source_shape_index_{ -1 };

  /** @brief The source primitive index used when reporting contact subshape ids. */
  int source_subshape_index_{ -1 };
};

}  // namespace tesseract::collision::tesseract_collision_coal

#endif  // TESSERACT_COLLISION_COAL_COLLISION_OBJECT_WRAPPER_H
