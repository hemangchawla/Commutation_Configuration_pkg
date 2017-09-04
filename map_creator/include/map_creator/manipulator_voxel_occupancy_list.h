#ifndef MANIPULATOR_VOXEL_OCCUPANCY_LIST_H
#define MANIPULATOR_VOXEL_OCCUPANCY_LIST_H

// C++
#include <chrono>

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit_msgs/RobotState.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/planning_interface/planning_interface.h>

// Octomap
#include <octomap/octomap.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

// FCL
#include <fcl/octree.h>
#include <fcl/traversal/traversal_node_octree.h>
#include <fcl/collision.h>
#include <fcl/broadphase/broadphase.h>

class manipulator_voxel_occupancy_list
{
public:
  /** @brief Constructor */
  manipulator_voxel_occupancy_list(octomap::OcTree*& octomap_tree, const collision_detection::CollisionRobotConstPtr& collision_robot_);
  /** @brief Destructor */
  ~manipulator_voxel_occupancy_list();
  /** @brief Mainfuction to compute centers of occupied voxels for a given robot state */
  void getOccupiedVoxels(robot_state::RobotStatePtr state_, std::vector<std::vector<double> >& occ_list);

private:

  /** @brief Generate the collision object boxes from the octomap */
  void generateBoxesFromOctomap(octomap::OcTree*& tree);

  std::vector<fcl::CollisionObject*> octree_boxes;
  fcl::BroadPhaseCollisionManager* manager_;
  fcl::CollisionRequest req;
  fcl::CollisionResult res;
  const collision_detection::CollisionRobotFCL* robot_fcl_ptr_;
};

#endif // MANIPULATOR_VOXEL_OCCUPANCY_LIST_H
