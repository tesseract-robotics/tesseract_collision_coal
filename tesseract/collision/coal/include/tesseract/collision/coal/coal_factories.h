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
 * @brief The yaml config for each of the factories below is the same.
 * @details
 * The config and its parameters shown below are optional.
 * The values shown below are the defaults that will be used.
 *
 * Example Yaml Config:
 *
 *    plugins:
 *      CoalDiscreteBVHManager:
 *        class: CoalDiscreteBVHManagerFactory
 *        config:
 *          gjk_guess_threshold: 0.005
 */
class CoalDiscreteBVHManagerFactory : public DiscreteContactManagerFactory
{
public:
  std::unique_ptr<DiscreteContactManager> create(const std::string& name,
                                                 const YAML::Node& config) const override final;
};

class CoalCastBVHManagerFactory : public ContinuousContactManagerFactory
{
public:
  std::unique_ptr<ContinuousContactManager> create(const std::string& name,
                                                   const YAML::Node& config) const override final;
};

PLUGIN_ANCHOR_DECL(CoalFactoriesAnchor)

}  // namespace tesseract::collision::tesseract_collision_coal
#endif  // TESSERACT_COLLISION_COAL_FACTORIES_H
