/**
 * @file coal_factories.h
 * @brief Factories for loading Coal contact managers as plugins
 *
 * @author Roelof Oomen, Levi Armstrong
 * @date October 25, 2021
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_COLLISION_COAL_FACTORIES_H
#define TESSERACT_COLLISION_COAL_FACTORIES_H

#include <tesseract/collision/contact_managers_plugin_factory.h>
#include <boost_plugin_loader/macros.h>

namespace tesseract::collision::tesseract_collision_coal
{
/**
 * @brief Factory for discrete (single-pose) collision checking.
 * @details
 * This factory takes no configuration.
 *
 * Example Yaml Config:
 *
 *    plugins:
 *      CoalDiscreteBVHManager:
 *        class: CoalDiscreteBVHManagerFactory
 */
class CoalDiscreteBVHManagerFactory : public DiscreteContactManagerFactory
{
public:
  tesseract::common::PropertyTree schema() const override;

protected:
  std::unique_ptr<DiscreteContactManager>
  createImpl(const std::string& name, const tesseract::common::PropertyTree& config) const override final;
};

/**
 * @brief Factory for continuous (swept/cast) collision checking.
 * @details
 * The config and its parameters shown below are optional.
 * The values shown below are the defaults that will be used.
 *
 * Example Yaml Config:
 *
 *    plugins:
 *      CoalCastBVHManager:
 *        class: CoalCastBVHManagerFactory
 *        config:
 *          d_arc_compensation: false
 *          relative_cast: true
 *
 * `d_arc_compensation` pads every swept hull by the arc sagitta of its rotation: a moving link's hull by
 * that of the link's turn in the world, and under relative cast the hull of a pair of two moving links by
 * that of their relative turn. See kDefaultDArcCompensation.
 *
 * `relative_cast` collides a pair of two moving links as one link's shape against the other's motion
 * relative to it. `false` collides the two links' own swept hulls instead, which reports a contact wherever
 * the links pass through the same space, at whatever times. Neither is conservative throughout: with
 * `d_arc_compensation` on, `false` is conservative for two links each moved by a single joint, which `true`
 * is not, and CoalCastBVHManager states where each reads too far. Two entries of this factory with different
 * configs give both, selected by name. See kDefaultRelativeCast.
 */
class CoalCastBVHManagerFactory : public ContinuousContactManagerFactory
{
public:
  tesseract::common::PropertyTree schema() const override;

protected:
  std::unique_ptr<ContinuousContactManager>
  createImpl(const std::string& name, const tesseract::common::PropertyTree& config) const override final;
};

PLUGIN_ANCHOR_DECL(CoalFactoriesAnchor)

}  // namespace tesseract::collision::tesseract_collision_coal
#endif  // TESSERACT_COLLISION_COAL_FACTORIES_H
