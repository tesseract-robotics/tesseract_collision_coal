#ifndef TESSERACT_COLLISION_COLLISION_LINK_FIXTURES_HPP
#define TESSERACT_COLLISION_COLLISION_LINK_FIXTURES_HPP

#include <string>

#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/geometry/geometries.h>

namespace tesseract::collision::test_suite::detail
{
/// Add a single box shape on a link.
inline void addBoxLink(ContinuousContactManager& checker,
                       const std::string& link_name,
                       const Eigen::Vector3d& half_extents,
                       const Eigen::Isometry3d& local_pose = Eigen::Isometry3d::Identity())
{
  CollisionShapePtr box =
      std::make_shared<tesseract::geometry::Box>(half_extents.x() * 2, half_extents.y() * 2, half_extents.z() * 2);
  CollisionShapesConst shapes;
  tesseract::common::VectorIsometry3d poses;
  shapes.push_back(box);
  poses.push_back(local_pose);
  checker.addCollisionObject(link_name, 0, shapes, poses);
}
}  // namespace tesseract::collision::test_suite::detail

#endif  // TESSERACT_COLLISION_COLLISION_LINK_FIXTURES_HPP
