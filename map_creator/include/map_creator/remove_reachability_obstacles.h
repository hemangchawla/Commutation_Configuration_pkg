#ifndef REMOVE_REACHABILITY_OBSTACLES_H
#define REMOVE_REACHABILITY_OBSTACLES_H

/*
 * Author: Hemang Chawla
 */

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

// Map Creator
#include <map_creator/WorkSpace.h>

#define SPIN_RATE 10

class FilteredReachability
{
public:
  /**
   * @brief The filterType enum The filtering of reachability voxels can be done based on the three modes of collision
   * check. Default is Conservative CIRCUMSCRIBED_RADIUS
   */
  enum FilterType
  {
    VOXEL,
    INSCRIBED_SPHERE,
    CIRCUMSCRIBED_SPHERE  // Conservative, Default
  };
  /**
   * @brief FilteredReachability The constructor to create a Filtered Reachability objec
   */
  FilteredReachability();
  ~FilteredReachability();
  /**
   * @brief spin Spin the ros node.
   * @param filter_type The type of FilteredReachability::FilterType to be used for direct filtering of reachability
   * voxels in collision
   * @todo convert to action server
   */
  void spin(FilterType filter_type);

private:
  // Functions
  /**
   * @brief readMap Reads the reachability map from topic and stores it
   * @param msg The published map as subscribed on topic
   */
  void readMap(const map_creator::WorkSpace msg);
  /**
   * @brief createObstaclesPointCloud Manipulates the read collision octomap to create the simplified coliision point
   * cloud
   * @param tree The obstacle tree
   * @param obstacle_vertices The simplified vertices representation of the obstacles
   * @todo compare with AABB collision check in terms of speed
   */
  void createObstaclesPointCloud(octomap::OcTree& tree, pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_vertices);

  /**
   * @brief readPlanningScene Read current state of the planning scene to be used for filtering voxels
   * @param msg The planning scene topic published by moveit
   */
  void readPlanningScene(const moveit_msgs::PlanningScene msg);

  /**
   * @brief createFilteredReachability Filters reachability map using obstacle information
   * @param type The type of filter to be used to remove the reachability voxels in direct collision
   * @param search_tree The obstacle octree for collision check
   * @param filtered_map The output filtered reachability map
   * @param colliding_map The output for colliding portion of reachability map
   */
  void createFilteredReachability(FilterType type, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& search_tree,
                                  map_creator::WorkSpace& filtered_map, map_creator::WorkSpace& colliding_map);

  // Variables
  /**
   * @brief nh_ Main node handle
   */
  ros::NodeHandle nh_;
  /**
   * @brief Subscriber_reachability_ Subscriber to the reachability or inverse reachability map
   */
  ros::Subscriber Subscriber_reachability_;
  /**
   * @brief Subscriber_planning_scene_ Subscriber to the moveit planning scene
   */
  ros::Subscriber Subscriber_planning_scene_;
  /**
   * @brief Publisher_filtered_reachability_ Publisher of the filtered reachability map
   */
  ros::Publisher Publisher_filtered_reachability_;
  /**
   * @brief Publisher_colliding_reachability_ Publisher of the directly colliding reachability map that was removed
   */
  ros::Publisher Publisher_colliding_reachability_;

  map_creator::WorkSpace reachability_map_;
  octomap::OcTree* collision_octree_;
  double reachability_resolution_;
  bool map_rcvd_;
  bool scene_rcvd_;
};

#endif  // REMOVE_REACHABILITY_OBSTACLES_H
