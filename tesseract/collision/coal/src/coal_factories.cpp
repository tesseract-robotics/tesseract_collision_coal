/**
 * @file coal_factories.cpp
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

#include <tesseract/collision/coal/coal_factories.h>
#include <tesseract/collision/coal/coal_discrete_managers.h>
#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/collision/coal/coal_utils.h>
#include <tesseract/collision/discrete_contact_manager.h>

namespace tesseract::collision::tesseract_collision_coal
{
template <typename T>
T getConfigValue(const YAML::Node& config, const char* key, T default_value)
{
  if (!config.IsNull())
    if (YAML::Node n = config[key])
      return n.as<T>();
  return default_value;
}

std::unique_ptr<tesseract::collision::DiscreteContactManager>
CoalDiscreteBVHManagerFactory::create(const std::string& name, const YAML::Node& config) const
{
  return std::make_unique<CoalDiscreteBVHManager>(
      name, getConfigValue(config, "gjk_guess_threshold", kDefaultGJKGuessThreshold));
}

std::unique_ptr<tesseract::collision::ContinuousContactManager>
CoalCastBVHManagerFactory::create(const std::string& name, const YAML::Node& config) const
{
  return std::make_unique<CoalCastBVHManager>(name,
                                              getConfigValue(config, "gjk_guess_threshold", kDefaultGJKGuessThreshold),
                                              getConfigValue(config, "d_arc_compensation", kDefaultDArcCompensation));
}

PLUGIN_ANCHOR_IMPL(CoalFactoriesAnchor)  // LCOV_EXCL_LINE

}  // namespace tesseract::collision::tesseract_collision_coal

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_DISCRETE_MANAGER_PLUGIN(tesseract::collision::tesseract_collision_coal::CoalDiscreteBVHManagerFactory,
                                      CoalDiscreteBVHManagerFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_CONTINUOUS_MANAGER_PLUGIN(tesseract::collision::tesseract_collision_coal::CoalCastBVHManagerFactory,
                                        CoalCastBVHManagerFactory);
