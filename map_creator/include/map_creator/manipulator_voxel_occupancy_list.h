#ifndef MANIPULATOR_VOXEL_OCCUPANCY_LIST_H
#define MANIPULATOR_VOXEL_OCCUPANCY_LIST_H

/*
 * Author: Hemang Chawla
 */

// Basic
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

class ManipulatorVoxelOccupancy
{
public:
  /**
   * @brief ManipulatorVoxelOccupancy Represents the occupancy object of a manipulator in a given octomap
   * @param octomap_tree The octree containing the world scene ie the 3D obstacles
   * @param collision_robot The collision model of the robot
   */
  ManipulatorVoxelOccupancy(octomap::OcTree*& octomap_tree,
                            const collision_detection::CollisionRobotConstPtr& collision_robot);
  /** @brief Destructor */
  ~ManipulatorVoxelOccupancy();
  /**
   * @brief getOccupiedVoxels Main fuction to compute centers of occupied voxels for a given robot state
   * @param state Ptr to the state of the robotic arm
   * @param occ_list Output the list of occupied voxel centers
   */
  void getOccupiedVoxels(robot_state::RobotStatePtr state, std::vector<std::vector<double> >& occ_list);

private:
  /**
   * @brief generateBoxesFromOctomap Generate the collision object boxes from the octomap during initialization
   * @param tree Octree of Octomap
   */
  void generateBoxesFromOctomap(octomap::OcTree*& tree);

  /**
   * @brief octree_boxes_ The collision objects created out of the octree boxes
   */
  std::vector<fcl::CollisionObject*> octree_boxes_;
  /**
   * @brief manager_ Collision manager for broadphase collision checeking
   */
  fcl::BroadPhaseCollisionManager* manager_;
  /**
   * @brief req_ Collision check request
   */
  fcl::CollisionRequest req_;
  /**
   * @brief res_ Collision check response
   */
  fcl::CollisionResult res_;
  /**
   * @brief robot_fcl_ptr_ The pointer to collision robot object
   */
  const collision_detection::CollisionRobotFCL* robot_fcl_ptr_;
};

#endif  // MANIPULATOR_VOXEL_OCCUPANCY_LIST_H
