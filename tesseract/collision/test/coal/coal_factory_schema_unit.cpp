/**
 * @file coal_factory_schema_unit.cpp
 * @brief Verify the schemas the Coal contact manager factories register
 *
 * @author Roelof Oomen
 * @date September 11, 2026
 *
 * @copyright Copyright (c) 2026, Roelof Oomen
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
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/property_tree.h>
#include <tesseract/common/yaml_extensions.h>
#include <tesseract/collision/coal/coal_factories.h>
#include <tesseract/collision/coal/coal_utils.h>

using namespace tesseract::common;

// The registration macros run at static initialization of the factories library; the anchor keeps
// that library linked so they run at all.
static const auto& coal_anchor = tesseract::collision::tesseract_collision_coal::CoalFactoriesAnchor();

namespace
{
std::string joinErrors(const std::vector<std::string>& errors)
{
  std::string msg;
  for (const auto& e : errors)
    msg += e + "\n";
  return msg;
}
}  // namespace

TEST(CoalFactorySchemaUnit, DiscreteAcceptsNoConfig)  // NOLINT
{
  auto schema = YAML::convert<ContactManagersPluginInfo>::schema();

  YAML::Node config = YAML::Load(R"(
    discrete_plugins:
      default: CoalDiscreteBVHManager
      plugins:
        CoalDiscreteBVHManager:
          class: CoalDiscreteBVHManagerFactory
  )");

  schema.mergeConfig(config);
  auto errors = schema.validate();
  EXPECT_TRUE(errors.empty()) << joinErrors(errors);
}

TEST(CoalFactorySchemaUnit, CastAcceptsDArcCompensation)  // NOLINT
{
  auto schema = YAML::convert<ContactManagersPluginInfo>::schema();

  YAML::Node config = YAML::Load(R"(
    continuous_plugins:
      default: CoalCastBVHManager
      plugins:
        CoalCastBVHManager:
          class: CoalCastBVHManagerFactory
          config:
            d_arc_compensation: true
  )");

  schema.mergeConfig(config);
  auto errors = schema.validate();
  EXPECT_TRUE(errors.empty()) << joinErrors(errors);
}

TEST(CoalFactorySchemaUnit, CastAcceptsOmittedConfig)  // NOLINT
{
  auto schema = YAML::convert<ContactManagersPluginInfo>::schema();

  YAML::Node config = YAML::Load(R"(
    continuous_plugins:
      default: CoalCastBVHManager
      plugins:
        CoalCastBVHManager:
          class: CoalCastBVHManagerFactory
  )");

  schema.mergeConfig(config);
  auto errors = schema.validate();
  EXPECT_TRUE(errors.empty()) << joinErrors(errors);
}

TEST(CoalFactorySchemaUnit, CastRejectsNonBooleanDArcCompensation)  // NOLINT
{
  auto schema = YAML::convert<ContactManagersPluginInfo>::schema();

  // The cast config expects a boolean for d_arc_compensation, not a string
  YAML::Node config = YAML::Load(R"(
    continuous_plugins:
      default: CoalCastBVHManager
      plugins:
        CoalCastBVHManager:
          class: CoalCastBVHManagerFactory
          config:
            d_arc_compensation: not_a_bool
  )");

  schema.mergeConfig(config);
  auto errors = schema.validate();
  ASSERT_FALSE(errors.empty());
  // Naming the key proves the rejection came from the type check, not from the factory being unregistered
  const std::string message = joinErrors(errors);
  EXPECT_NE(message.find("config: d_arc_compensation"), std::string::npos) << message;
}

TEST(CoalFactorySchemaUnit, CastRejectsUnknownConfigKey)  // NOLINT
{
  auto schema = YAML::convert<ContactManagersPluginInfo>::schema();

  // The cast config declares only d_arc_compensation, so an undeclared key is rejected
  YAML::Node config = YAML::Load(R"(
    continuous_plugins:
      default: CoalCastBVHManager
      plugins:
        CoalCastBVHManager:
          class: CoalCastBVHManagerFactory
          config:
            not_a_real_key: true
  )");

  schema.mergeConfig(config);
  auto errors = schema.validate();
  ASSERT_FALSE(errors.empty());
  // Naming the key proves the rejection came from the extra-property check, not from the factory
  // being unregistered
  const std::string message = joinErrors(errors);
  EXPECT_NE(message.find("config: not_a_real_key"), std::string::npos) << message;
}

TEST(CoalFactorySchemaUnit, CastFactorySchemaDeclaresDArcCompensation)  // NOLINT
{
  // Validation goes through the registered schema function, so the factory's own schema() is the
  // copy tooling reads. The two must not drift.
  const tesseract::collision::tesseract_collision_coal::CoalCastBVHManagerFactory factory;
  const PropertyTree schema = factory.schema();

  ASSERT_EQ(schema.size(), 1U);
  const auto default_value = schema.at("d_arc_compensation").getAttribute(property_attribute::DEFAULT);
  ASSERT_TRUE(default_value.has_value());
  // ASSERT_TRUE returns on failure, but bugprone-unchecked-optional-access cannot see that through the macro
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  EXPECT_EQ(default_value->as<bool>(), tesseract::collision::tesseract_collision_coal::kDefaultDArcCompensation);
}

TEST(CoalFactorySchemaUnit, DiscreteFactorySchemaDeclaresNoKeys)  // NOLINT
{
  // The discrete manager takes no configuration. A childless schema is also why it accepts any
  // config block unchecked, unlike its cast sibling.
  const tesseract::collision::tesseract_collision_coal::CoalDiscreteBVHManagerFactory factory;
  EXPECT_TRUE(factory.schema().empty());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
