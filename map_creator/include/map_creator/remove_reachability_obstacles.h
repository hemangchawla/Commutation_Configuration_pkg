#ifndef REMOVE_REACHABILITY_OBSTACLES_H
#define REMOVE_REACHABILITY_OBSTACLES_H

// C++
#include <chrono>

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit_msgs/GetPlanningScene.h>

// Octomap
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/AbstractOcTree.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

//Map Creator
#include <map_creator/WorkSpace.h>

#define SPIN_RATE 10

class remove_obstacles_reachability
{
public:
  remove_obstacles_reachability();
  ~remove_obstacles_reachability();
  void spin();

private:
  // Functions

  void readMap(const map_creator::WorkSpace msg); // PARTIAL
  // Reads the reachability map and stores in global variable
  void createObstaclesPointCloud(octomap::OcTree& tree, pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_vertices); // DONE
  // Manipulates the read collision octomap to create the simplified coliision point cloud
  void createFilteredReachability(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& search_tree, map_creator::WorkSpace& filtered_map, map_creator::WorkSpace& colliding_map); // ONGOING
  // Filters reachability map using obstacle information

  ros::NodeHandle nh;
  ros::Subscriber Subscriber_reachability;
  ros::Publisher Publisher_filtered_reachability;
  ros::Publisher Publisher_colliding_reachability;
  ros::ServiceClient Client_get_planning_scene;
  moveit_msgs::GetPlanningScene scene_srv;
  map_creator::WorkSpace reachability_map;
  double reachability_resolution;
};

#endif // REMOVE_REACHABILITY_OBSTACLES_H
