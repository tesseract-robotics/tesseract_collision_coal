#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <algorithm>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/collision/common.h>
#include <tesseract/collision/test_suite/collision_link_fixtures.hpp>

using namespace tesseract::collision;
using namespace tesseract_collision_coal;

/**
 * @brief Removal unregisters whatever the add path registered, for a static and for a kinematic link.
 *
 * A link is registered through its regular wrapper when static and through its cast wrapper otherwise,
 * so removal must decide on the same question. A link that survives in a broadphase after removal is
 * reached through a wrapper the maps no longer own.
 */
class FilterGroupRemovalUnit : public ::testing::Test
{
protected:
  void SetUp() override
  {
    for (const auto& link : { std::string("probe"), std::string("obstacle"), std::string("witness") })
      test_suite::detail::addSphereLink(checker_, link, 0.5);
    checker_.setDefaultCollisionMargin(0.0);
  }

  /** @brief Sweep every active link in place and report the contacts. */
  ContactResultMap contacts()
  {
    const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    checker_.setCollisionObjectsTransform("probe", pose, pose);

    ContactResultMap result;
    checker_.contactTest(result, ContactRequest(ContactTestType::ALL));
    return result;
  }

  /**
   * @brief True when any reported pair names @p link and carries results.
   *
   * A ContactResultMap keeps a key whose results were emptied, so a key alone is not a contact.
   */
  static bool mentions(const ContactResultMap& result, const tesseract::common::LinkId& link)
  {
    return std::any_of(result.begin(), result.end(), [&link](const auto& pair) {
      return !pair.second.empty() && (pair.first.first() == link || pair.first.second() == link);
    });
  }

  CoalCastBVHManager checker_;
};

TEST_F(FilterGroupRemovalUnit, RemovingAKinematicLinkUnregistersIt)  // NOLINT
{
  checker_.setActiveCollisionObjects({ "probe", "obstacle" });

  // All three spheres sit at the origin, so the removed link is reported unless it is unregistered.
  ASSERT_TRUE(mentions(contacts(), tesseract::common::LinkId("obstacle")));

  ASSERT_TRUE(checker_.removeCollisionObject("obstacle"));

  const auto& cast_map = checker_.getCastCollisionObjectMap();
  EXPECT_TRUE(cast_map.find("obstacle") == cast_map.end());
  EXPECT_FALSE(mentions(contacts(), tesseract::common::LinkId("obstacle")));
}

TEST_F(FilterGroupRemovalUnit, RemovingAStaticLinkUnregistersIt)  // NOLINT
{
  // "witness" is never named active, so it is registered through its regular wrapper in the static tree.
  checker_.setActiveCollisionObjects({ "probe" });

  ASSERT_TRUE(mentions(contacts(), tesseract::common::LinkId("witness")));

  ASSERT_TRUE(checker_.removeCollisionObject("witness"));

  EXPECT_FALSE(mentions(contacts(), tesseract::common::LinkId("witness")));
}

TEST_F(FilterGroupRemovalUnit, RemovingALinkLeavesItsNeighboursRegistered)  // NOLINT
{
  // The control: removal must unregister the link named and nothing else.
  checker_.setActiveCollisionObjects({ "probe" });

  ASSERT_TRUE(checker_.removeCollisionObject("witness"));

  EXPECT_TRUE(mentions(contacts(), tesseract::common::LinkId("obstacle")));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
