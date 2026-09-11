/**
 * @file contact_managers_factory_unit.cpp
 * @brief Tesseract collision contact managers factory test
 *
 * @author Levi Armstrong
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
#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <cstdlib>

#include <tesseract/collision/contact_managers_plugin_factory.h>
#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/collision/bullet/bullet_factories.h>
#include <tesseract/collision/coal/coal_factories.h>
#include <tesseract/common/yaml_utils.h>
#include <tesseract/common/resource_locator.h>
#include <boost_plugin_loader/utils.h>

using namespace tesseract::collision;

TEST(TesseractContactManagersFactoryUnit, StaticLoadPlugin)  // NOLINT
{
  // Workaround: Bullet's anchor symbol is unlinkable while tesseract_collision_coal is developed
  // outside mainline tesseract, because Bullet's factories target is installed but not exported
  // from the tesseract package. Once Coal moves in-tree, this should use the same anchor-based
  // registration as Coal's below.
  constexpr const char* env_name = "TESSERACT_CONTACT_MANAGERS_PLUGINS";
  std::string env_value = "tesseract_collision_bullet_factories";
  const char* existing_env = std::getenv(env_name);  // NOLINT(concurrency-mt-unsafe)
  if (existing_env != nullptr && existing_env[0] != '\0')
    env_value = std::string(existing_env) + ":" + env_value;
  setenv(env_name, env_value.c_str(), 1);  // NOLINT(concurrency-mt-unsafe)

  // Coal's factories library is a target of this project, so its anchor symbol can be linked
  // directly; Bullet's and FCL's live in tesseract and are installed but not exported, so they
  // cannot. Referencing the anchor keeps the factories library a link dependency, and this call
  // puts its resolved path on the loader's search list, which is what makes
  // CoalDiscreteBVHManagerFactory and CoalCastBVHManagerFactory resolvable below.
  boost_plugin_loader::addSymbolLibraryToSearchLibrariesEnv(
      tesseract::collision::tesseract_collision_coal::CoalFactoriesAnchor(), env_name);

  std::string config = R"(contact_manager_plugins:
                            search_paths:
                              - /usr/local/lib
                            search_libraries:
                              - tesseract_collision_bullet_factories_not_there
                              - tesseract_collision_fcl_factories_not_there
                              - tesseract_collision_coal_coal_factories_not_there
                            discrete_plugins:
                              default: BulletDiscreteBVHManager
                              plugins:
                                BulletDiscreteBVHManager:
                                  class: BulletDiscreteBVHManagerFactory
                                BulletDiscreteSimpleManager:
                                  class: BulletDiscreteSimpleManagerFactory
                                FCLDiscreteBVHManager:
                                  class: FCLDiscreteBVHManagerFactory
                                CoalDiscreteBVHManager:
                                  class: CoalDiscreteBVHManagerFactory
                            continuous_plugins:
                              default: BulletCastBVHManager
                              plugins:
                                BulletCastBVHManager:
                                  class: BulletCastBVHManagerFactory
                                BulletCastSimpleManager:
                                  class: BulletCastSimpleManagerFactory
                                CoalCastBVHManager:
                                  class: CoalCastBVHManagerFactory)";

  tesseract::common::GeneralResourceLocator locator;
  ContactManagersPluginFactory factory(config, locator);
  factory.clearSearchLibraries();
  factory.clearSearchPaths();
  YAML::Node plugin_config = tesseract::common::loadYamlString(config, locator);

  DiscreteContactManager::UPtr cm = factory.createDiscreteContactManager("BulletDiscreteBVHManager");
  EXPECT_TRUE(cm != nullptr);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
