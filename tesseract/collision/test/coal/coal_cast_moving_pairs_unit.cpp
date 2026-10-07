#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <coal/shape/geometric_shapes.h>
#include <yaml-cpp/yaml.h>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <tuple>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/coal/coal_cast_managers.h>
#include <tesseract/collision/coal/coal_casthullshape.h>
#include <tesseract/collision/coal/coal_factories.h>
#include <tesseract/collision/coal/coal_utils.h>
#include <tesseract/geometry/geometries.h>

using namespace tesseract::collision;
using tesseract::collision::tesseract_collision_coal::CoalCastBVHManager;
using tesseract::collision::tesseract_collision_coal::CoalCastBVHManagerFactory;
using tesseract::common::LinkId;

namespace
{
constexpr double BOX = 0.2;

Eigen::Isometry3d at(double x, double y = 0.0, double z = 0.0)
{
  Eigen::Isometry3d tf{ Eigen::Isometry3d::Identity() };
  tf.translation() = Eigen::Vector3d(x, y, z);
  return tf;
}

void addShape(ContinuousContactManager& checker,
              const std::string& link,
              const CollisionShapeConstPtr& shape,
              const Eigen::Isometry3d& shape_pose = Eigen::Isometry3d::Identity())
{
  const CollisionShapesConst shapes{ shape };
  const tesseract::common::VectorIsometry3d poses{ shape_pose };
  checker.addCollisionObject(LinkId(link), 0, shapes, poses, true);
}

void addBox(CoalCastBVHManager& checker,
            const std::string& link,
            const Eigen::Isometry3d& shape_pose = Eigen::Isometry3d::Identity())
{
  addShape(checker, link, std::make_shared<tesseract::geometry::Box>(BOX, BOX, BOX), shape_pose);
}

constexpr double GAP = 0.05;

/// A turn by @p angle about the line through @p centre along @p axis.
Eigen::Isometry3d turnAbout(const Eigen::Vector3d& centre, const Eigen::Vector3d& axis, double angle)
{
  Eigen::Isometry3d tf{ Eigen::Isometry3d::Identity() };
  tf.linear() = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  tf.translation() = centre - tf.linear() * centre;
  return tf;
}

/// A turn by @p angle about the vertical axis through @p centre.
Eigen::Isometry3d turnAbout(const Eigen::Vector3d& centre, double angle)
{
  return turnAbout(centre, Eigen::Vector3d::UnitZ(), angle);
}

ContactResultVector contacts(ContinuousContactManager& checker,
                             const ContactRequest& request = ContactRequest(ContactTestType::ALL))
{
  ContactResultMap result;
  checker.contactTest(result, request);
  ContactResultVector flat;
  result.flattenMoveResults(flat);
  return flat;
}

/// The one contact @p checker is expected to find; a contact with no data where it finds another number.
ContactResult onlyContact(ContinuousContactManager& checker,
                          const ContactRequest& request = ContactRequest(ContactTestType::ALL))
{
  const ContactResultVector found = contacts(checker, request);
  EXPECT_EQ(found.size(), 1U);
  return (found.size() == 1U) ? found[0] : ContactResult();
}

/// The slot @p link occupies in @p contact.
std::size_t slotOf(const ContactResult& contact, const std::string& link)
{
  return (contact.link_ids[0].name() == link) ? 0U : 1U;
}

Eigen::Isometry3d aStart() { return at(0.0); }
Eigen::Isometry3d bStart() { return at(BOX + GAP); }

/// The box of addBox as a convex mesh, the form a robot's links usually take.
void addConvexBox(CoalCastBVHManager& checker, const std::string& link)
{
  const double h = 0.5 * BOX;
  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
  vertices->emplace_back(-h, -h, -h);
  vertices->emplace_back(h, -h, -h);
  vertices->emplace_back(h, h, -h);
  vertices->emplace_back(-h, h, -h);
  vertices->emplace_back(-h, -h, h);
  vertices->emplace_back(h, -h, h);
  vertices->emplace_back(h, h, h);
  vertices->emplace_back(-h, h, h);
  // Twelve outward triangles, each prefixed by its vertex count.
  auto faces = std::make_shared<Eigen::VectorXi>(48);
  *faces << 3, 0, 2, 1, 3, 0, 3, 2, 3, 4, 5, 6, 3, 4, 6, 7, 3, 0, 1, 5, 3, 0, 5, 4, 3, 1, 2, 6, 3, 1, 6, 5, 3, 2, 3, 7,
      3, 2, 7, 6, 3, 3, 0, 4, 3, 3, 4, 7;
  addShape(checker, link, std::make_shared<tesseract::geometry::ConvexMesh>(vertices, faces));
}

/// Boxes "a" and "b", GAP apart along x, both active; as convex meshes with @p convex.
void addGappedBoxes(CoalCastBVHManager& checker, double margin, bool convex = false)
{
  for (const std::string link : { "a", "b" })
  {
    if (convex)
      addConvexBox(checker, link);
    else
      addBox(checker, link);
  }
  checker.setActiveCollisionObjects({ LinkId("a"), LinkId("b") });
  checker.setDefaultCollisionMargin(margin);
}

/// Sweep both boxes of addGappedBoxes through the same @p motion.
void sweepTogether(ContinuousContactManager& checker, const Eigen::Isometry3d& motion)
{
  checker.setCollisionObjectsTransform("a", aStart(), motion * aStart());
  checker.setCollisionObjectsTransform("b", bStart(), motion * bStart());
}

/// Spheres "p" and "q" of radius 0.25 on crossing paths, 0.4 apart at the crossing, which they pass at
/// different times. Their centres are nearest at t = 0.44: p at (-0.2, 0.16, 0), q at (0.2, 0, -0.12).
void addCrossingSpheres(ContinuousContactManager& checker)
{
  const auto sphere = std::make_shared<tesseract::geometry::Sphere>(0.25);
  addShape(checker, "p", sphere);
  addShape(checker, "q", sphere);
  checker.setActiveCollisionObjects({ LinkId("p"), LinkId("q") });
  checker.setDefaultCollisionMargin(0.0);
}

void sweepCrossingSpheres(ContinuousContactManager& checker)
{
  checker.setCollisionObjectsTransform("p", at(-0.2, -0.5, 0.0), at(-0.2, 1.0, 0.0));
  checker.setCollisionObjectsTransform("q", at(0.2, 0.0, -1.0), at(0.2, 0.0, 1.0));
}

constexpr double CROSSING_TIME = 0.44;
const Eigen::Vector3d P_AT_CROSSING(-0.2, 0.16, 0.0);
const Eigen::Vector3d Q_AT_CROSSING(0.2, 0.0, -0.12);

/// "b" closes in on "a" and turns as it comes, while both are carried from @p carry0 to @p carry1.
ContactResult approachCarried(const Eigen::Isometry3d& carry0, const Eigen::Isometry3d& carry1)
{
  CoalCastBVHManager checker;
  addGappedBoxes(checker, 0.1);
  const Eigen::Isometry3d b_end = at(0.22, 0.03) * turnAbout(Eigen::Vector3d::Zero(), 0.3);
  checker.setCollisionObjectsTransform("a", carry0 * aStart(), carry1 * aStart());
  checker.setCollisionObjectsTransform("b", carry0 * at(0.6), carry1 * b_end);
  return onlyContact(checker);
}

/// A frame that is neither at the origin nor aligned with its axes, as a link's is.
Eigen::Isometry3d generalFrame()
{
  return at(0.3, -0.7, 0.45) * Eigen::AngleAxisd(0.83, Eigen::Vector3d(1.0, 2.0, 3.0).normalized());
}

/// The collision object of shape @p shape_index of @p link's cast wrapper: what the narrowphase keys a pair on.
const coal::CollisionObject* castObject(const CoalCastBVHManager& checker,
                                        const std::string& link,
                                        std::size_t shape_index = 0)
{
  const auto* cast = checker.getCastCollisionObject(LinkId(link));
  return (cast == nullptr || cast->getCollisionObjects().size() <= shape_index) ?
             nullptr :
             cast->getCollisionObjects()[shape_index].get();
}

/// The hull a pair that sweeps shape @p shape_index of @p link is collided through. It holds the sweep the
/// last query of such a pair wrote, and no sweep if there has been none.
const tesseract_collision_coal::CastHullShape* pairHull(const CoalCastBVHManager& checker,
                                                        const std::string& link,
                                                        std::size_t shape_index = 0)
{
  const coal::CollisionObject* object = castObject(checker, link, shape_index);
  if (object == nullptr)
    return nullptr;
  const auto* hull = dynamic_cast<const tesseract_collision_coal::CastHullShape*>(object->collisionGeometryPtr());
  return (hull == nullptr) ? nullptr : &hull->scratchHull();
}

/// Half the diagonal of the box of addBox: how far a corner is from its centre.
const double CORNER_REACH = 0.5 * std::sqrt(3.0) * BOX;

/// The pose in its link of a box set off from the link's origin by @p offset and turned so that one corner
/// points along the link's x axis (@p along_x = 1) or against it (-1).
Eigen::Isometry3d cornerOnX(const Eigen::Vector3d& offset, double along_x)
{
  Eigen::Isometry3d pose{ Eigen::Isometry3d::Identity() };
  pose.linear() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::Ones(), along_x * Eigen::Vector3d::UnitX())
                      .toRotationMatrix();
  pose.translation() = offset;
  return pose;
}

const Eigen::Vector3d STILL_BOX_OFFSET(0.03, 0.1, 0.02);
const Eigen::Vector3d MOVING_BOX_OFFSET(-0.02, 0.05, 0.01);
/// Where "m" is when it is beside the still link, in the frame of that link.
const Eigen::Vector3d M_BESIDE(0.476, 0.065, 0.0);
/// From the still box's corner to the corner of m's box that faces it, with m at M_BESIDE.
const Eigen::Vector3d CORNER_TO_CORNER = (M_BESIDE + MOVING_BOX_OFFSET - CORNER_REACH * Eigen::Vector3d::UnitX()) -
                                         (STILL_BOX_OFFSET + CORNER_REACH * Eigen::Vector3d::UnitX());

/// "m" travels in a straight line, without turning, between a pose far from a box that holds still and
/// M_BESIDE, towards it or away. Each box is offset and turned in its link so that a corner faces the other
/// box, and both links sit in a general frame. The still box is static, or an active link that does not move.
ContactResult cornerToCorner(const std::string& still_link, bool still_link_active, bool approaching)
{
  CoalCastBVHManager checker;
  addBox(checker, still_link, cornerOnX(STILL_BOX_OFFSET, 1.0));
  addBox(checker, "m", cornerOnX(MOVING_BOX_OFFSET, -1.0));
  if (still_link_active)
    checker.setActiveCollisionObjects({ LinkId(still_link), LinkId("m") });
  else
    checker.setActiveCollisionObjects({ LinkId("m") });
  checker.setDefaultCollisionMargin(0.2);
  const Eigen::Isometry3d frame = generalFrame();
  const Eigen::Isometry3d apart = frame * at(0.9, 0.3, 0.2);
  const Eigen::Isometry3d beside = frame * at(M_BESIDE.x(), M_BESIDE.y(), M_BESIDE.z());
  checker.setCollisionObjectsTransform(LinkId(still_link), frame);
  checker.setCollisionObjectsTransform("m", approaching ? apart : beside, approaching ? beside : apart);
  return onlyContact(checker);
}
}  // namespace

TEST(CoalCastMovingPairsUnit, CastWrapperRecordsWhereItsSweepEnds)  // NOLINT
{
  CoalCastBVHManager checker;
  addBox(checker, "a");
  checker.setActiveCollisionObjects({ LinkId("a") });

  checker.setCollisionObjectsTransform("a", at(1.0), at(2.0));
  const auto* cast = checker.getCastCollisionObject("a");
  ASSERT_NE(cast, nullptr);
  EXPECT_TRUE(cast->getCollisionObjectsTransform().isApprox(at(1.0)));
  EXPECT_TRUE(cast->getSweepEndTransform().isApprox(at(2.0)));
  EXPECT_TRUE(cast->getSweepDisplacement().isApprox(at(1.0)));
  EXPECT_TRUE(cast->getSweepDisplacementInverse().isApprox(at(-1.0)));

  // A sweep that goes nowhere is no displacement at all, not one to rounding.
  const Eigen::Isometry3d turned = generalFrame();
  checker.setCollisionObjectsTransform("a", turned, turned);
  EXPECT_TRUE(cast->getSweepDisplacement().matrix() == Eigen::Isometry3d::Identity().matrix());
  EXPECT_TRUE(cast->getSweepDisplacementInverse().matrix() == Eigen::Isometry3d::Identity().matrix());

  // A pose set without a sweep leaves none behind.
  checker.setCollisionObjectsTransform("a", at(1.0), at(2.0));
  checker.setCollisionObjectsTransform("a", at(3.0));
  cast = checker.getCastCollisionObject("a");
  ASSERT_NE(cast, nullptr);
  EXPECT_TRUE(cast->getSweepEndTransform().isApprox(at(3.0)));
  EXPECT_TRUE(cast->getSweepDisplacement().matrix() == Eigen::Isometry3d::Identity().matrix());
}

TEST(CoalCastMovingPairsUnit, SweepSetFromTheWrappersOwnPosesIsKept)  // NOLINT
{
  CoalCastBVHManager checker;
  addBox(checker, "a");
  checker.setActiveCollisionObjects({ LinkId("a") });
  checker.setCollisionObjectsTransform("a", at(1.0), at(2.0));
  const auto* cast = checker.getCastCollisionObject("a");
  ASSERT_NE(cast, nullptr);

  // The sweep run backwards, each pose read from the wrapper the call writes.
  checker.setCollisionObjectsTransform("a", cast->getSweepEndTransform(), cast->getCollisionObjectsTransform());
  EXPECT_TRUE(cast->isSwept());
  EXPECT_TRUE(cast->getCollisionObjectsTransform().isApprox(at(2.0)));
  EXPECT_TRUE(cast->getSweepEndTransform().isApprox(at(1.0)));
  EXPECT_TRUE(cast->getSweepDisplacement().isApprox(at(-1.0)));
}

TEST(CoalCastMovingPairsUnit, LinksTranslatingTogetherReportNoContact)  // NOLINT
{
  CoalCastBVHManager checker;
  addGappedBoxes(checker, 0.0);
  // Six times the gap, along the axis joining them: each box ends up where the other one was.
  sweepTogether(checker, at(0.3));
  EXPECT_TRUE(contacts(checker).empty());
}

TEST(CoalCastMovingPairsUnit, ConvexMeshLinksRotatingTogetherKeepTheirGap)  // NOLINT
{
  CoalCastBVHManager checker;
  addGappedBoxes(checker, 0.1, /*convex=*/true);
  sweepTogether(checker, turnAbout(Eigen::Vector3d(0.0, -2.0, 0.0), 0.4));
  EXPECT_NEAR(onlyContact(checker).distance, GAP, 1e-5);
}

TEST(CoalCastMovingPairsUnit, DistanceRespondsToEachEndPoseAsTheFieldsSay)  // NOLINT
{
  // What an optimiser reads off a contact: moving a link's start pose along the normal changes the
  // distance by (1 - cc_time) of that move and its end pose by cc_time, with opposite signs for the two
  // links.
  auto contactFor = [](const std::array<Eigen::Isometry3d, 4>& poses) {
    CoalCastBVHManager checker;
    addCrossingSpheres(checker);
    // The spheres stay apart, inside the margin: a distance is resolved more finely than a penetration
    // depth, and the slopes below are differences of it.
    checker.setDefaultCollisionMargin(0.3);
    checker.setCollisionObjectsTransform("p", poses[0], poses[1]);
    checker.setCollisionObjectsTransform("q", poses[2], poses[3]);
    return onlyContact(checker);
  };

  // p's start and end, then q's: the crossing of addCrossingSpheres, 0.6 apart instead of 0.4.
  const std::array<Eigen::Isometry3d, 4> poses{
    at(-0.3, -0.5, 0.0), at(-0.3, 1.0, 0.0), at(0.3, 0.0, -1.0), at(0.3, 0.0, 1.0)
  };
  const ContactResult contact = contactFor(poses);
  EXPECT_GT(contact.distance, 0.05);
  const std::size_t p = slotOf(contact, "p");
  const std::size_t q = 1U - p;
  // The normal points from slot 0 to slot 1: moving the link in slot 1 along it opens the gap.
  const std::array<double, 2> sign{ -1.0, 1.0 };
  const std::array<double, 4> share{ sign[p] * (1.0 - contact.cc_time[p]),
                                     sign[p] * contact.cc_time[p],
                                     sign[q] * (1.0 - contact.cc_time[q]),
                                     sign[q] * contact.cc_time[q] };

  const double step = 5e-3;
  for (std::size_t pose = 0; pose < 4; ++pose)
  {
    for (Eigen::Index axis = 0; axis < 3; ++axis)
    {
      std::array<Eigen::Isometry3d, 4> ahead = poses;
      std::array<Eigen::Isometry3d, 4> behind = poses;
      ahead[pose].translation()[axis] += step;
      behind[pose].translation()[axis] -= step;
      const double slope = (contactFor(ahead).distance - contactFor(behind).distance) / (2.0 * step);
      EXPECT_NEAR(slope, share[pose] * contact.normal[axis], 2e-3) << "pose " << pose << ", axis " << axis;
    }
  }
}

TEST(CoalCastMovingPairsUnit, CommonMotionLeavesTheResultUnchanged)  // NOLINT
{
  const Eigen::Isometry3d still = Eigen::Isometry3d::Identity();
  const ContactResult reference = approachCarried(still, still);

  const Eigen::Isometry3d carry0 = turnAbout(Eigen::Vector3d(1.0, 1.0, 0.0), 0.7) * at(0.3, -0.2, 0.1);
  const Eigen::Isometry3d carry1 = turnAbout(Eigen::Vector3d(-1.0, 0.5, 0.0), -0.4) * at(-0.2, 0.4, 0.3);
  const ContactResult carried = approachCarried(carry0, carry1);

  EXPECT_LT(reference.distance, 0.0);  // the scenario is a real contact, not an empty comparison
  EXPECT_NEAR(carried.distance, reference.distance, 1e-5);
  EXPECT_NEAR(carried.cc_time[0], reference.cc_time[0], 1e-4);
  EXPECT_EQ(carried.cc_type[0], reference.cc_type[0]);
}

namespace
{
/// Link @p turning, a box of side @p turning_box, turns in place beside link @p still, a box of side
/// @p still_box that holds still. Expects the distance that sweeping @p turning gives, which is the true
/// one: swept by its own turn it is nearest at the end of it, where a corner has come round to face the
/// still box. Were the still link swept about the turning one instead, the straight line between its two
/// poses would cut inside its arc and read nearer.
///
/// The narrowphase keys a pair on its two collision objects in an order that the order the links are added
/// in can change, and which the choice of swept link must not follow. So the scene is built with the links
/// added either way round, and both builds must give the one answer.
void expectTurningLinkSwept(const std::string& still, double still_box, const std::string& turning, double turning_box)
{
  const double angle = 0.6;
  const double apart = 0.3;
  for (const bool still_added_first : { true, false })
  {
    SCOPED_TRACE(still_added_first ? "still link added first" : "turning link added first");
    CoalCastBVHManager checker;
    const auto still_shape = std::make_shared<tesseract::geometry::Box>(still_box, still_box, still_box);
    const auto turning_shape = std::make_shared<tesseract::geometry::Box>(turning_box, turning_box, turning_box);
    if (still_added_first)
      addShape(checker, still, still_shape);
    addShape(checker, turning, turning_shape);
    if (!still_added_first)
      addShape(checker, still, still_shape);
    checker.setActiveCollisionObjects({ LinkId(still), LinkId(turning) });
    checker.setDefaultCollisionMargin(0.2);
    const Eigen::Isometry3d turning_start = at(apart);
    checker.setCollisionObjectsTransform(LinkId(still), at(0.0), at(0.0));
    checker.setCollisionObjectsTransform(
        LinkId(turning), turning_start, turning_start * turnAbout(Eigen::Vector3d::Zero(), angle));
    EXPECT_NEAR(onlyContact(checker).distance,
                apart - (0.5 * still_box) - (0.5 * turning_box * (std::cos(angle) + std::sin(angle))),
                1e-5);
  }
}
}  // namespace

TEST(CoalCastMovingPairsUnit, SweptShapeIsChosenBySizeThenByNameWhicheverIsGivenFirst)  // NOLINT
{
  using tesseract_collision_coal::pairSweepsFirst;

  // The smaller shape is swept, whatever the names of the links.
  EXPECT_TRUE(pairSweepsFirst(0.1, 0.2, "a", "b"));
  EXPECT_FALSE(pairSweepsFirst(0.2, 0.1, "b", "a"));
  EXPECT_TRUE(pairSweepsFirst(0.1, 0.2, "b", "a"));
  EXPECT_FALSE(pairSweepsFirst(0.2, 0.1, "a", "b"));

  // Between shapes of one size, that of the link whose name sorts last.
  EXPECT_FALSE(pairSweepsFirst(0.1, 0.1, "a", "b"));
  EXPECT_TRUE(pairSweepsFirst(0.1, 0.1, "b", "a"));
}

TEST(CoalCastMovingPairsUnit, SmallerShapeIsSwept)  // NOLINT
{
  // The turning box is the smaller one and its link's name sorts first, so only its size can make it the
  // swept link.
  expectTurningLinkSwept("b", BOX, "a", 0.5 * BOX);
}

TEST(CoalCastMovingPairsUnit, OfEqualShapesTheLinkWhoseNameSortsLastIsSwept)  // NOLINT
{
  expectTurningLinkSwept("a", BOX, "b", BOX);
}

TEST(CoalCastMovingPairsUnit, LinkIsHeldForOneOfItsShapesAndSweptForAnother)  // NOLINT
{
  // k carries a box larger than m's and one smaller, so against m it is held for the first and swept for
  // the second within one check. m closes in on both along k's x axis while a common motion carries the
  // two links: a relative translation, which the hull holds exactly whichever shape it sweeps.
  const double big_box = 2.0 * BOX;
  const double small_box = 0.5 * BOX;
  const double gap_to_big = 0.03;
  // The small box's face towards m lies this far behind the big one's.
  const double set_back = 0.05;
  CoalCastBVHManager checker;
  const CollisionShapesConst k_shapes{ std::make_shared<tesseract::geometry::Box>(big_box, big_box, big_box),
                                       std::make_shared<tesseract::geometry::Box>(small_box, small_box, small_box) };
  const tesseract::common::VectorIsometry3d k_poses{ at(0.0),
                                                     at((0.5 * big_box) - set_back - (0.5 * small_box), 0.35) };
  checker.addCollisionObject(LinkId("k"), 0, k_shapes, k_poses, true);
  addBox(checker, "m");
  checker.setActiveCollisionObjects({ LinkId("k"), LinkId("m") });
  checker.setDefaultCollisionMargin(0.1);

  // m overlaps the big box and the small one in y, and stops gap_to_big short of the big one's face.
  const double m_y = 0.25;
  const Eigen::Isometry3d k_start = generalFrame();
  const Eigen::Isometry3d carry = turnAbout(Eigen::Vector3d(0.4, -0.2, 0.3), Eigen::Vector3d(2.0, -1.0, 2.0), 0.4);
  checker.setCollisionObjectsTransform("k", k_start, carry * k_start);
  const double m_start_x = 0.8;
  const double m_end_x = (0.5 * big_box) + gap_to_big + (0.5 * BOX);
  checker.setCollisionObjectsTransform("m", k_start * at(m_start_x, m_y), carry * k_start * at(m_end_x, m_y));

  const ContactResultVector found = contacts(checker);
  ASSERT_EQ(found.size(), 2U);
  for (const ContactResult& contact : found)
  {
    const bool against_big = contact.shape_id[slotOf(contact, "k")] == 0;
    SCOPED_TRACE(against_big ? "m against k's big box" : "m against k's small box");
    EXPECT_NEAR(contact.distance, against_big ? gap_to_big : gap_to_big + set_back, 1e-5);
    EXPECT_EQ(contact.cc_time[0], contact.cc_time[1]);
    for (std::size_t i = 0; i < 2; ++i)
      EXPECT_EQ(contact.cc_type[i], ContinuousCollisionType::CCType_Time1);
  }

  // Each pair wrote its sweep on the hull of its smaller shape: m's against the big box, the small box's
  // against m, each the travel between the two links as its own shape sees it. Nothing swept the big box.
  const auto* m_hull = pairHull(checker, "m");
  const auto* big_hull = pairHull(checker, "k", 0);
  const auto* small_hull = pairHull(checker, "k", 1);
  ASSERT_NE(m_hull, nullptr);
  ASSERT_NE(big_hull, nullptr);
  ASSERT_NE(small_hull, nullptr);
  const Eigen::Vector3d travel(m_end_x - m_start_x, 0.0, 0.0);
  EXPECT_TRUE(m_hull->getCastTransform().getTranslation().isApprox(travel, 1e-9));
  EXPECT_TRUE(small_hull->getCastTransform().getTranslation().isApprox(-travel, 1e-9));
  EXPECT_TRUE(big_hull->getCastTransform() == coal::Transform3s());
}

TEST(CoalCastMovingPairsUnit, PairSurvivesALinkGoingStaticAndBack)  // NOLINT
{
  CoalCastBVHManager checker;
  addGappedBoxes(checker, 0.0);
  sweepTogether(checker, at(0.3));
  EXPECT_TRUE(contacts(checker).empty());

  // b static: a sweeps into it, twice the gap.
  checker.setActiveCollisionObjects({ LinkId("a") });
  checker.setCollisionObjectsTransform("b", bStart());
  checker.setCollisionObjectsTransform("a", aStart(), at(2.0 * GAP));
  EXPECT_NEAR(onlyContact(checker).distance, -GAP, 1e-5);

  // Both active again: moving together is free, and the same approach collides as it did before.
  checker.setActiveCollisionObjects({ LinkId("a"), LinkId("b") });
  sweepTogether(checker, at(0.3));
  EXPECT_TRUE(contacts(checker).empty());

  checker.setCollisionObjectsTransform("b", bStart(), bStart());
  checker.setCollisionObjectsTransform("a", aStart(), at(2.0 * GAP));
  EXPECT_NEAR(onlyContact(checker).distance, -GAP, 1e-5);
}

TEST(CoalCastMovingPairsUnit, OffsetShapesMovingTogetherMatchTheUnsweptCheck)  // NOLINT
{
  // Two shapes on one link and a shape turned and offset in the other: the sweep is per shape, in each
  // shape's own frame.
  auto setup = [](CoalCastBVHManager& checker) {
    const auto box = std::make_shared<tesseract::geometry::Box>(BOX, BOX, BOX);
    const CollisionShapesConst two{ box, box };
    const tesseract::common::VectorIsometry3d two_poses{ at(0.0), at(0.0, 0.3) };
    checker.addCollisionObject(LinkId("a"), 0, two, two_poses, true);
    addBox(checker, "b", at(0.03, 0.1, 0.02) * turnAbout(Eigen::Vector3d::Zero(), 0.5));
    checker.setActiveCollisionObjects({ LinkId("a"), LinkId("b") });
    checker.setDefaultCollisionMargin(0.2);
  };
  auto distance_by_shape_of_a = [](const ContactResultVector& found) {
    std::map<int, double> distances;
    for (const auto& contact : found)
      distances[contact.shape_id[slotOf(contact, "a")]] = contact.distance;
    return distances;
  };

  CoalCastBVHManager unswept;
  setup(unswept);
  unswept.setCollisionObjectsTransform("a", aStart());
  unswept.setCollisionObjectsTransform("b", bStart());
  const std::map<int, double> expected = distance_by_shape_of_a(contacts(unswept));
  ASSERT_EQ(expected.size(), 2U);

  CoalCastBVHManager swept;
  setup(swept);
  sweepTogether(swept, turnAbout(Eigen::Vector3d(0.5, -2.0, 0.0), 0.4) * at(0.1, 0.0, 0.2));
  const std::map<int, double> found = distance_by_shape_of_a(contacts(swept));
  ASSERT_EQ(found.size(), 2U);
  for (const auto& [shape, distance] : expected)
    EXPECT_NEAR(found.at(shape), distance, 1e-5) << "shape " << shape << " of a";
}

TEST(CoalCastMovingPairsUnit, ClonedManagerAnswersOnItsOwn)  // NOLINT
{
  CoalCastBVHManager checker;
  addCrossingSpheres(checker);
  sweepCrossingSpheres(checker);
  const ContactResult original = onlyContact(checker);

  const ContinuousContactManager::UPtr clone = checker.clone();
  sweepCrossingSpheres(*clone);
  const ContactResult cloned = onlyContact(*clone);
  EXPECT_NEAR(cloned.distance, original.distance, 1e-6);
  EXPECT_NEAR(cloned.cc_time[0], original.cc_time[0], 1e-6);

  // Parking the clone's spheres at their start poses does not disturb the original's sweep.
  clone->setCollisionObjectsTransform("p", at(-0.2, -0.5, 0.0));
  clone->setCollisionObjectsTransform("q", at(0.2, 0.0, -1.0));
  EXPECT_TRUE(contacts(*clone).empty());
  EXPECT_NEAR(onlyContact(checker).distance, original.distance, 1e-6);
}

TEST(CoalCastMovingPairsUnit, RelativeCastSettingReachesTheManagerAndItsClone)  // NOLINT
{
  // The spheres' hulls overlap by 0.1 where their paths cross, and the spheres themselves are nearest at
  // CROSSING_TIME. They only translate, so arc compensation adds nothing to either reading.
  const double hull_against_hull = -0.1;
  const double at_one_time = (Q_AT_CROSSING - P_AT_CROSSING).norm() - 0.5;

  const CoalCastBVHManagerFactory factory;
  YAML::Node config;
  config["d_arc_compensation"] = true;
  config["relative_cast"] = false;
  const ContinuousContactManager::UPtr separate = factory.create("separate", config);
  addCrossingSpheres(*separate);
  sweepCrossingSpheres(*separate);
  EXPECT_NEAR(onlyContact(*separate).distance, hull_against_hull, 1e-4);

  const ContinuousContactManager::UPtr clone = separate->clone();
  sweepCrossingSpheres(*clone);
  EXPECT_NEAR(onlyContact(*clone).distance, hull_against_hull, 1e-4);

  const ContinuousContactManager::UPtr by_default = factory.create("default", YAML::Node());
  addCrossingSpheres(*by_default);
  sweepCrossingSpheres(*by_default);
  EXPECT_NEAR(onlyContact(*by_default).distance, at_one_time, 1e-4);
}

TEST(CoalCastMovingPairsUnit, EveryRequestKindSeesTheSamePair)  // NOLINT
{
  const ContactRequest first(ContactTestType::FIRST);
  ContactRequest verdict_only(ContactTestType::FIRST);
  verdict_only.calculate_penetration = false;
  verdict_only.calculate_distance = false;
  const ContactRequest closest(ContactTestType::CLOSEST);

  for (const ContactRequest& request : { first, verdict_only, closest })
  {
    SCOPED_TRACE((static_cast<int>(request.type) * 10) + static_cast<int>(request.calculate_penetration));

    CoalCastBVHManager together;
    addGappedBoxes(together, 0.0);
    sweepTogether(together, at(0.3));
    EXPECT_TRUE(contacts(together, request).empty());

    CoalCastBVHManager crossing;
    addCrossingSpheres(crossing);
    sweepCrossingSpheres(crossing);
    const ContactResult found = onlyContact(crossing, request);
    // Without penetration the narrowphase stops at the verdict, and the contact carries no normal to
    // locate a time with.
    if (request.calculate_penetration)
    {
      EXPECT_EQ(found.cc_time[0], found.cc_time[1]);
      EXPECT_EQ(found.cc_type[0], found.cc_type[1]);
    }
  }
}

TEST(CoalCastMovingPairsUnit, EachPairOfThreeLinksGetsItsOwnSweep)  // NOLINT
{
  // The boxes are equal, so c, whose name sorts last, is the swept link of two pairs at once, moving with a
  // and coming up beside b: its sweep differs from one pair to the next within a single check.
  CoalCastBVHManager checker;
  addBox(checker, "a");
  addBox(checker, "b");
  addBox(checker, "c");
  checker.setActiveCollisionObjects({ LinkId("a"), LinkId("b"), LinkId("c") });
  checker.setDefaultCollisionMargin(0.1);

  const double travel = 0.3;
  const Eigen::Isometry3d c_start = at(BOX + GAP);
  // b waits beside where c ends up, GAP away from it in y.
  const Eigen::Isometry3d b_pose = at(BOX + GAP + travel, BOX + GAP);
  checker.setCollisionObjectsTransform("a", aStart(), at(travel) * aStart());
  checker.setCollisionObjectsTransform("c", c_start, at(travel) * c_start);
  checker.setCollisionObjectsTransform("b", b_pose, b_pose);

  std::map<std::string, double> distance;
  for (const auto& contact : contacts(checker))
  {
    const std::string& first = contact.link_ids[0].name();
    const std::string& second = contact.link_ids[1].name();
    distance[(first < second) ? first + second : second + first] = contact.distance;
  }
  ASSERT_EQ(distance.size(), 3U);
  EXPECT_NEAR(distance.at("ac"), GAP, 1e-5);                   // moving together
  EXPECT_NEAR(distance.at("bc"), GAP, 1e-5);                   // c ends beside b
  EXPECT_NEAR(distance.at("ab"), std::sqrt(2.0) * GAP, 1e-5);  // a ends GAP short of b in x and in y
}

TEST(CoalCastMovingPairsUnit, SweepSetOnADisabledLinkIsCheckedOnceEnabled)  // NOLINT
{
  CoalCastBVHManager checker;
  addBox(checker, "s");
  addBox(checker, "m");
  // Further active links, out of reach, so that m is not the whole of the broadphase.
  for (const std::string far_link : { "f1", "f2", "f3" })
    addBox(checker, far_link);
  checker.setActiveCollisionObjects({ LinkId("m"), LinkId("f1"), LinkId("f2"), LinkId("f3") });
  checker.setDefaultCollisionMargin(0.1);
  checker.setCollisionObjectsTransform("s", at(0.0));
  checker.setCollisionObjectsTransform("f1", at(50.0), at(51.0));
  checker.setCollisionObjectsTransform("f2", at(50.0, 50.0), at(51.0, 50.0));
  checker.setCollisionObjectsTransform("f3", at(50.0, 0.0, 50.0), at(51.0, 0.0, 50.0));

  // Far from the static box and unswept, m touches nothing.
  checker.setCollisionObjectsTransform("m", at(5.0));
  ASSERT_TRUE(contacts(checker).empty());

  // A sweep set while m is disabled is the one it is checked along once enabled. It ends beside the box,
  // where neither its start pose nor the pose m held before puts it.
  checker.disableCollisionObject("m");
  checker.setCollisionObjectsTransform("m", at(0.8), at(BOX + GAP));
  EXPECT_TRUE(contacts(checker).empty());
  checker.enableCollisionObject("m");
  const auto* cast = checker.getCastCollisionObject("m");
  ASSERT_NE(cast, nullptr);
  EXPECT_TRUE(cast->isSwept());
  EXPECT_TRUE(cast->getCollisionObjectsTransform().isApprox(at(0.8)));
  EXPECT_TRUE(cast->getSweepEndTransform().isApprox(at(BOX + GAP)));
  const ContactResult contact = onlyContact(checker);
  const std::size_t m = slotOf(contact, "m");
  EXPECT_NEAR(contact.distance, GAP, 1e-5);
  EXPECT_TRUE(contact.cc_transform[m].isApprox(at(BOX + GAP)));

  // It also replaces the sweep m held: set far from the box while disabled, m no longer reaches it.
  checker.disableCollisionObject("m");
  checker.setCollisionObjectsTransform("m", at(5.0), at(5.5));
  checker.enableCollisionObject("m");
  EXPECT_TRUE(contacts(checker).empty());
}

TEST(CoalCastMovingPairsUnit, ShapeBoundRadiusMeasuresTheWrappedShapeAlone)  // NOLINT
{
  using tesseract::collision::tesseract_collision_coal::CastHullShape;
  const coal::Transform3s unswept;

  // A box measures half its diagonal.
  auto box = std::make_shared<coal::Box>(0.2, 0.4, 0.6);
  CastHullShape box_hull(box, unswept);
  EXPECT_NEAR(box_hull.getShapeBoundRadius(), 0.5 * Eigen::Vector3d(0.2, 0.4, 0.6).norm(), 1e-12);

  // A sphere's own radius is part of the shape: it measures half the diagonal of the cube around it.
  const CastHullShape sphere_hull(std::make_shared<coal::Sphere>(0.25), unswept);
  EXPECT_NEAR(sphere_hull.getShapeBoundRadius(), 0.25 * std::sqrt(3.0), 1e-12);

  // Neither a sweep nor a padding of the hull changes it, and the hull's scratch hull and its clone report
  // the same.
  const double before = box_hull.getShapeBoundRadius();
  coal::Transform3s sweep;
  sweep.setTranslation(coal::Vec3s(0.3, 0.0, 0.0));
  box_hull.setSweptSphereRadius(0.01);
  box_hull.updateCastTransform(sweep);
  EXPECT_EQ(box_hull.getShapeBoundRadius(), before);
  EXPECT_EQ(box_hull.scratchHull().getShapeBoundRadius(), before);
  const std::unique_ptr<CastHullShape> clone(box_hull.clone());
  EXPECT_EQ(clone->getShapeBoundRadius(), before);
}

TEST(CoalCastMovingPairsUnit, ScratchHullIsOnePerHullAndNotCopied)  // NOLINT
{
  using tesseract::collision::tesseract_collision_coal::CastHullShape;
  auto box = std::make_shared<coal::Box>(BOX, BOX, BOX);
  box->computeLocalAABB();
  coal::Transform3s own_sweep;
  own_sweep.setTranslation(coal::Vec3s(0.0, 0.5, 0.0));
  CastHullShape hull(box, own_sweep);
  hull.setSweptSphereRadius(0.01);
  hull.computeLocalAABB();

  CastHullShape& scratch = hull.scratchHull();
  EXPECT_EQ(&hull.scratchHull(), &scratch);
  EXPECT_NE(&scratch, &hull);
  EXPECT_EQ(scratch.getUnderlyingShape(), hull.getUnderlyingShape());

  // It starts unswept, whatever sweep the hull it belongs to holds.
  EXPECT_TRUE(scratch.getCastTransform() == coal::Transform3s());
  EXPECT_EQ(scratch.getSweptSphereRadius(), 0.0);

  // Sweeping the scratch hull leaves the hull it belongs to alone.
  coal::Transform3s sweep;
  sweep.setTranslation(coal::Vec3s(0.3, 0.0, 0.0));
  scratch.updateCastTransform(sweep);
  EXPECT_TRUE(hull.getCastTransform() == own_sweep);
  EXPECT_EQ(hull.getSweptSphereRadius(), 0.01);

  // A copy starts with a scratch hull of its own.
  const std::unique_ptr<CastHullShape> copy(hull.clone());
  EXPECT_NE(&copy->scratchHull(), &scratch);
}

TEST(CoalCastMovingPairsUnit, NormalTurnsWithLinksThatRotateTogether)  // NOLINT
{
  CoalCastBVHManager checker;
  addGappedBoxes(checker, 0.1);
  const double angle = 0.4;
  sweepTogether(checker, turnAbout(Eigen::Vector3d(0.0, -2.0, 0.0), angle));
  const ContactResult contact = onlyContact(checker);
  EXPECT_NEAR(contact.distance, GAP, 1e-5);

  // Links that move together are as near all through the sweep, so the contact is reported at its middle,
  // where they have made half the turn.
  EXPECT_NEAR(contact.cc_time[0], 0.5, 1e-9);
  const Eigen::Vector3d a_to_b = Eigen::AngleAxisd(0.5 * angle, Eigen::Vector3d::UnitZ()) * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d slot0_to_slot1 = (slotOf(contact, "a") == 0U) ? a_to_b : Eigen::Vector3d(-a_to_b);
  EXPECT_LT((contact.normal - slot0_to_slot1).norm(), 1e-6);
  EXPECT_NEAR((contact.nearest_points[1] - contact.nearest_points[0]).dot(contact.normal), contact.distance, 1e-6);
}

TEST(CoalCastMovingPairsUnit, ContactAtTheEndOfTheSweepLiesOnTheEndPoses)  // NOLINT
{
  CoalCastBVHManager checker;
  addGappedBoxes(checker, 0.1);
  // a slides sideways while b closes in from afar: they are nearest at the end, GAP apart.
  const Eigen::Isometry3d a_end = at(0.0, 0.3);
  const Eigen::Isometry3d b_end = at(BOX + GAP, 0.3);
  checker.setCollisionObjectsTransform("a", aStart(), a_end);
  checker.setCollisionObjectsTransform("b", at(0.6), b_end);
  const ContactResult contact = onlyContact(checker);

  EXPECT_NEAR(contact.distance, GAP, 1e-5);
  for (std::size_t i = 0; i < 2; ++i)
  {
    EXPECT_EQ(contact.cc_type[i], ContinuousCollisionType::CCType_Time1);
    EXPECT_NEAR(contact.cc_time[i], 1.0, 1e-9);
  }

  // The world points lie on the faces that face each other, with the boxes at their end poses.
  const std::size_t a = slotOf(contact, "a");
  const std::size_t b = 1U - a;
  const Eigen::Vector3d on_a = a_end.inverse() * contact.nearest_points[a];
  const Eigen::Vector3d on_b = b_end.inverse() * contact.nearest_points[b];
  EXPECT_NEAR(on_a.x(), 0.5 * BOX, 1e-5);
  EXPECT_LE(std::abs(on_a.y()), (0.5 * BOX) + 1e-9);
  EXPECT_NEAR(on_b.x(), -0.5 * BOX, 1e-5);
  EXPECT_LE(std::abs(on_b.y()), (0.5 * BOX) + 1e-9);

  // A contact pinned to the end reports the end-pose point through the start pose: the centre of each
  // facing face, once the start pose is applied and the end pose taken back out.
  const Eigen::Vector3d face_of_a = a_end.inverse() * (contact.transform[a] * contact.nearest_points_local[a]);
  const Eigen::Vector3d face_of_b = b_end.inverse() * (contact.transform[b] * contact.nearest_points_local[b]);
  EXPECT_LT((face_of_a - Eigen::Vector3d(0.5 * BOX, 0.0, 0.0)).norm(), 1e-6);
  EXPECT_LT((face_of_b - Eigen::Vector3d(-0.5 * BOX, 0.0, 0.0)).norm(), 1e-6);
}

TEST(CoalCastMovingPairsUnit, ArcCompensationFollowsTheRelativeTurn)  // NOLINT
{
  // b turns in place beside a, and of the two equal boxes it is the swept one, its name sorting last. The
  // hull of b's two poses cuts the arc its corners travel; the compensation pads it by the sagitta of that
  // arc.
  const double angle = 0.6;
  auto distance = [angle](bool d_arc_compensation, const Eigen::Isometry3d& carry) {
    CoalCastBVHManager checker("test", d_arc_compensation);
    addGappedBoxes(checker, 0.2);
    const Eigen::Isometry3d b_start = at(0.3);
    checker.setCollisionObjectsTransform("a", aStart(), carry * aStart());
    checker.setCollisionObjectsTransform("b", b_start, carry * b_start * turnAbout(Eigen::Vector3d::Zero(), angle));
    return onlyContact(checker).distance;
  };

  const Eigen::Isometry3d still = Eigen::Isometry3d::Identity();
  // The turn's axis passes through b's centre, and its corners are half a diagonal away from it.
  const double sagitta = CORNER_REACH * (1.0 - std::cos(0.5 * angle));
  EXPECT_NEAR(distance(true, still), distance(false, still) - sagitta, 1e-5);

  // Carrying both links along pads nothing more: only the relative turn is compensated.
  const Eigen::Isometry3d carry = turnAbout(Eigen::Vector3d(0.0, -2.0, 0.0), 0.4);
  EXPECT_NEAR(distance(true, carry), distance(true, still), 1e-5);
}

TEST(CoalCastMovingPairsUnit, LinksTurningTogetherFromGeneralPosesAreCollidedUnswept)  // NOLINT
{
  // From poses like a real link's, a shared turn leaves a relative motion that is the identity only to
  // rounding, and the rounding grows with the distance from the origin. The pair is collided through an
  // unswept hull all the same, with arc compensation or without and whatever sweep the hull held before,
  // and reads the gap, the middle of the sweep and the turned normal.
  const Eigen::Vector3d axis = Eigen::Vector3d(2.0, -1.0, 2.0).normalized();
  const double angle = 0.4;
  const Eigen::Isometry3d motion = turnAbout(Eigen::Vector3d(0.3, -2.0, 0.4), axis, angle) * at(0.1, 0.2, 0.3);

  for (const auto& [distant, convex, d_arc_compensation] : { std::tuple{ false, false, true },
                                                             std::tuple{ false, true, true },
                                                             std::tuple{ true, false, true },
                                                             std::tuple{ false, false, false } })
  {
    SCOPED_TRACE(std::string(distant ? "far from the origin, " : "near the origin, ") +
                 (convex ? "convex meshes" : "boxes") + (d_arc_compensation ? "" : ", no arc compensation"));
    const Eigen::Isometry3d frame = distant ? at(120.0, -80.0, 40.0) * generalFrame() : generalFrame();
    CoalCastBVHManager checker("test", d_arc_compensation);
    addGappedBoxes(checker, 0.1, convex);

    // First b makes the turn alone. The shapes are equal and b sorts last, so it is the swept link: its
    // scratch hull is left holding that sweep and, with compensation, the radius that covers its arc.
    checker.setCollisionObjectsTransform("a", frame * aStart(), frame * aStart());
    checker.setCollisionObjectsTransform("b", frame * bStart(), motion * frame * bStart());
    contacts(checker);
    const auto* hull = pairHull(checker, "b");
    ASSERT_NE(hull, nullptr);
    ASSERT_FALSE(hull->getCastTransform() == coal::Transform3s());
    ASSERT_EQ(hull->getSweptSphereRadius() > 0.0, d_arc_compensation);

    checker.setCollisionObjectsTransform("a", frame * aStart(), motion * frame * aStart());
    checker.setCollisionObjectsTransform("b", frame * bStart(), motion * frame * bStart());
    const ContactResult contact = onlyContact(checker);

    EXPECT_NEAR(contact.distance, GAP, 1e-5);
    EXPECT_TRUE(hull->getCastTransform() == coal::Transform3s()) << "the pair was collided through a sweep";
    EXPECT_EQ(hull->getSweptSphereRadius(), 0.0);

    // As near all through the sweep, the links are reported at its middle, having made half the turn.
    EXPECT_NEAR(contact.cc_time[0], 0.5, 1e-6);
    EXPECT_NEAR(contact.cc_time[1], 0.5, 1e-6);
    const Eigen::Vector3d a_to_b = Eigen::AngleAxisd(0.5 * angle, axis) * (frame.linear() * Eigen::Vector3d::UnitX());
    const Eigen::Vector3d slot0_to_slot1 = (slotOf(contact, "a") == 0U) ? a_to_b : Eigen::Vector3d(-a_to_b);
    EXPECT_LT((contact.normal - slot0_to_slot1).norm(), 1e-6);

    // The hull holds one pose, so the contact lies between the ends of the sweep, and each link reports the
    // centre of the face that faces the other.
    EXPECT_EQ(contact.cc_type[0], ContinuousCollisionType::CCType_Between);
    EXPECT_EQ(contact.cc_type[1], ContinuousCollisionType::CCType_Between);
    const std::size_t a = slotOf(contact, "a");
    EXPECT_LT((contact.nearest_points_local[a] - Eigen::Vector3d(0.5 * BOX, 0.0, 0.0)).norm(), 1e-5);
    EXPECT_LT((contact.nearest_points_local[1U - a] - Eigen::Vector3d(-0.5 * BOX, 0.0, 0.0)).norm(), 1e-5);
  }
}

TEST(CoalCastMovingPairsUnit, LinkThatIsNotSweptReportsItsContactAtTheMiddle)  // NOLINT
{
  // An active link set to one pose is not swept. Its contact with a static obstacle is reported between the
  // ends of the sweep, at the middle, at the centre of the face that faces the obstacle.
  CoalCastBVHManager checker;
  addBox(checker, "a");
  addBox(checker, "s");
  checker.setActiveCollisionObjects({ LinkId("a") });
  checker.setDefaultCollisionMargin(0.1);
  const Eigen::Isometry3d frame = generalFrame();
  checker.setCollisionObjectsTransform("a", frame * aStart());
  checker.setCollisionObjectsTransform("s", frame * bStart());
  const ContactResult contact = onlyContact(checker);
  const std::size_t a = slotOf(contact, "a");

  EXPECT_NEAR(contact.distance, GAP, 1e-5);
  EXPECT_EQ(contact.cc_type[a], ContinuousCollisionType::CCType_Between);
  EXPECT_EQ(contact.cc_time[a], 0.5);
  EXPECT_LT((contact.nearest_points_local[a] - Eigen::Vector3d(0.5 * BOX, 0.0, 0.0)).norm(), 1e-5);
}

TEST(CoalCastMovingPairsUnit, RelativeMotionAboveRoundingIsSwept)  // NOLINT
{
  // Each pair is swept by a motion with no turn worth the name, so arc compensation pads it by nothing.
  const auto expect_swept = [](const std::string& what,
                               const Eigen::Isometry3d& a_start,
                               const Eigen::Isometry3d& a_end,
                               const Eigen::Isometry3d& b_start,
                               const Eigen::Isometry3d& b_end) {
    SCOPED_TRACE(what);
    CoalCastBVHManager checker("test", /*d_arc_compensation=*/true);
    addGappedBoxes(checker, 0.1);
    checker.setCollisionObjectsTransform("a", a_start, a_end);
    checker.setCollisionObjectsTransform("b", b_start, b_end);
    EXPECT_NEAR(onlyContact(checker).distance, GAP, 1e-5);

    const auto* hull = pairHull(checker, "b");
    ASSERT_NE(hull, nullptr);
    EXPECT_FALSE(hull->getCastTransform() == coal::Transform3s()) << "the pair's motion was discarded";
    EXPECT_EQ(hull->getSweptSphereRadius(), 0.0);
  };

  // b ends 1e-9 from where the shared motion would put it, which is far above rounding.
  const Eigen::Isometry3d frame = generalFrame();
  const Eigen::Isometry3d motion =
      turnAbout(Eigen::Vector3d(0.3, -2.0, 0.4), Eigen::Vector3d(2.0, -1.0, 2.0).normalized(), 0.4) * at(0.1, 0.2, 0.3);
  expect_swept("a step aside",
               frame * aStart(),
               motion * frame * aStart(),
               frame * bStart(),
               at(0.0, 1e-9) * motion * frame * bStart());

  // b turns by 1e-9 about its own centre while a holds still. That carries its shape nowhere: the turn alone
  // counts.
  expect_swept("a slight turn in place",
               frame * aStart(),
               frame * aStart(),
               frame * bStart(),
               frame * bStart() * turnAbout(Eigen::Vector3d::Zero(), 1e-9));

  // Far from the origin, b turns about the origin by an angle that alone would count as none, while a holds
  // still. The turn carries b's shape 7e-9 along, and that is what counts.
  const Eigen::Isometry3d distant = at(120.0, -80.0, 40.0) * generalFrame();
  expect_swept("a slight turn about a distant centre",
               distant * aStart(),
               distant * aStart(),
               distant * bStart(),
               turnAbout(Eigen::Vector3d::Zero(), 5e-11) * distant * bStart());
}

TEST(CoalCastMovingPairsUnit, PairWithAStillLinkReportsWhatAStaticPartnerDoes)  // NOLINT
{
  // An active link that holds still is the same obstacle as a static one, and the static path is the
  // reference for every field of the contact. m does not turn, so sweeping either link by the relative
  // motion covers the same set: whether the still link is the held one or the swept one must not show
  // either. The boxes are equal, so the names decide: "c" sorts before "m" and is held, "z" after it and is
  // swept.
  //
  // The two runs solve the same problem from cast transforms that agree to rounding, and take most points
  // from the same box corners; only the static link's point is the narrowphase witness itself. So they agree
  // to the narrowphase's convergence tolerance, 1e-6, in a scene whose lengths are 0.1 to 1.
  const double tolerance = 1e-6;
  for (const bool approaching : { true, false })
  {
    for (const std::string still_link : { "c", "z" })
    {
      SCOPED_TRACE(still_link + (approaching ? ", approaching" : ", moving away"));
      const ContactResult as_static = cornerToCorner(still_link, false, approaching);
      const ContactResult as_active = cornerToCorner(still_link, true, approaching);
      const std::size_t m_static = slotOf(as_static, "m");
      const std::size_t m_active = slotOf(as_active, "m");

      // The reference is the scene intended: corner to corner, with m nearest at the pose beside the still
      // link, which is where its sweep ends or starts.
      EXPECT_NEAR(as_static.distance, CORNER_TO_CORNER.norm(), 1e-5);
      EXPECT_EQ(as_static.cc_type[m_static],
                approaching ? ContinuousCollisionType::CCType_Time1 : ContinuousCollisionType::CCType_Time0);

      EXPECT_NEAR(as_active.distance, as_static.distance, tolerance);
      EXPECT_EQ(as_active.cc_type[m_active], as_static.cc_type[m_static]);
      EXPECT_NEAR(as_active.cc_time[m_active], as_static.cc_time[m_static], tolerance);
      // The pair has one time, which the still link reports too.
      EXPECT_EQ(as_active.cc_type[1U - m_active], as_active.cc_type[m_active]);
      EXPECT_EQ(as_active.cc_time[1U - m_active], as_active.cc_time[m_active]);

      EXPECT_LT((as_active.nearest_points_local[m_active] - as_static.nearest_points_local[m_static]).norm(),
                tolerance);
      EXPECT_LT((as_active.nearest_points_local[1U - m_active] - as_static.nearest_points_local[1U - m_static]).norm(),
                tolerance);

      EXPECT_LT((as_active.nearest_points[m_active] - as_static.nearest_points[m_static]).norm(), tolerance);
      EXPECT_LT((as_active.nearest_points[1U - m_active] - as_static.nearest_points[1U - m_static]).norm(), tolerance);
      // The normal points from slot 0 to slot 1, so it turns round with the slots.
      const double same_slots = (m_active == m_static) ? 1.0 : -1.0;
      EXPECT_LT((same_slots * as_active.normal - as_static.normal).norm(), tolerance);
    }
  }
}

TEST(CoalCastMovingPairsUnit, ContactIsCarriedByTheTimeItsHeldLinkHasTravelled)  // NOLINT
{
  // The crossing spheres, with one turn added to the end pose of both. A motion both links share leaves
  // their relative motion alone, so they still meet at CROSSING_TIME, as deep. Of the two equal spheres p,
  // whose name sorts first, is held, and the contact is reported where p is by then: that fraction of the
  // way along its path, having made that fraction of the turn. A contact at the middle of a sweep cannot
  // tell that fraction from the rest of it; this one can.
  const double angle = 1.0;
  const Eigen::Isometry3d turn = turnAbout(Eigen::Vector3d(0.5, -0.3, 0.0), angle);
  const Eigen::Isometry3d p_start = at(-0.2, -0.5, 0.0);
  const Eigen::Isometry3d p_end = turn * at(-0.2, 1.0, 0.0);

  CoalCastBVHManager checker;
  addCrossingSpheres(checker);
  checker.setCollisionObjectsTransform("p", p_start, p_end);
  checker.setCollisionObjectsTransform("q", at(0.2, 0.0, -1.0), turn * at(0.2, 0.0, 1.0));
  const ContactResult contact = onlyContact(checker);
  const std::size_t p = slotOf(contact, "p");
  const std::size_t q = 1U - p;

  const Eigen::Vector3d p_to_q = Q_AT_CROSSING - P_AT_CROSSING;
  EXPECT_NEAR(contact.distance, p_to_q.norm() - 0.5, 1e-4);
  EXPECT_NEAR(contact.cc_time[p], CROSSING_TIME, 1e-3);
  EXPECT_NEAR(contact.cc_time[q], CROSSING_TIME, 1e-3);

  // Where p is at the crossing, and what its share of the turn does to a direction.
  const Eigen::Vector3d p_centre = (1.0 - CROSSING_TIME) * p_start.translation() + CROSSING_TIME * p_end.translation();
  const Eigen::Matrix3d turned = Eigen::AngleAxisd(CROSSING_TIME * angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d towards_q = turned * p_to_q.normalized();
  EXPECT_LT((contact.nearest_points[p] - (p_centre + 0.25 * towards_q)).norm(), 1e-3);
  EXPECT_LT((contact.nearest_points[q] - (p_centre + turned * p_to_q - 0.25 * towards_q)).norm(), 1e-3);
  // The normal points from the link in slot 0 to the link in slot 1.
  const Eigen::Vector3d slot0_to_slot1 = (p == 0U) ? towards_q : Eigen::Vector3d(-towards_q);
  EXPECT_LT((contact.normal - slot0_to_slot1).norm(), 1e-3);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
