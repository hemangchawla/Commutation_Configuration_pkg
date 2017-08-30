#ifndef MANIPULATOR_VOXEL_OCCUPANCY_LIST_H
#define MANIPULATOR_VOXEL_OCCUPANCY_LIST_H

// C++
#include <chrono>

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_robot.h>
#include <moveit/planning_interface/planning_interface.h>


// Octomap
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/AbstractOcTree.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

// FCL
#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
#include "fcl/math/transform.h"
//#include "test_fcl_utility.h"
//#include "fcl_resources/config.h"
//#include <boost/filesystem.hpp>


//Map Creator
//#include <map_creator/WorkSpace.h>

class manipulator_voxel_occupancy_list
{
public:
  manipulator_voxel_occupancy_list(octomap::OcTree*& octomap_tree);
  ~manipulator_voxel_occupancy_list();
  void getOccupiedVoxels(robot_state::RobotStatePtr state, std::vector<std::vector<double>>& occ_list);
private:
  void generateBoxesFromOctomap(octomap::OcTree*& tree);
  bool checkVoxelOccupancy(std::vector<double>& voxel_center, double& voxel_size);

  std::vector<fcl::CollisionObject*> octree_boxes;


};

#endif // MANIPULATOR_VOXEL_OCCUPANCY_LIST_H
