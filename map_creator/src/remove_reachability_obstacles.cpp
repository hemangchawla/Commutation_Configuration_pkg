/* AUTHOR: Hemang Chawla */

#include <map_creator/remove_reachability_obstacles.h>

/**
 * DESCRIPTION:
 * This program computes the voxels in collision with the reachability map of a robot.
 * A new topic is hence published which removes the reachability spheres in collision
 */

/**
 * APPROACH:
 *
 * - Create the reachability centers list. This list will be used to remove the voxels occupied by the map
 * - Use moveit service to get the octomap
 * - Create the center plus vertices point cloud
 * - Use this obstacle point cloud to filter the reachability octree
 * - Publish new reachability topic
 *
 */
FilteredReachability::FilteredReachability()
{
  // Initiating and connecting to MoveIt! Octomap planning scene

  // Note: The client method is slower
  //  Client_get_planning_scene = nh_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene", true);
  //  scene_srv.request.components.components = scene_srv.request.components.OCTOMAP;

  // Listen to relavent topics:
  scene_rcvd_ = false;
  Subscriber_planning_scene_ =
      nh_.subscribe("/move_group/monitored_planning_scene", 1, &FilteredReachability::readPlanningScene, this);
  map_rcvd_ = false;
  Subscriber_reachability_ = nh_.subscribe("/reachability_map", 1, &FilteredReachability::readMap, this);
  // reachability gives use the centers of the voxels we will be searching
  Publisher_filtered_reachability_ = nh_.advertise<map_creator::WorkSpace>("/reachability_map_filtered", 0);
  Publisher_colliding_reachability_ = nh_.advertise<map_creator::WorkSpace>("/reachability_map_colliding", 0);

  /** @todo
   * if (dynamic scene)
   *  The new reachability map will be published at the rate at which the scene is receied.
   * else
   *  The scene is published at selected rate
   */
}

FilteredReachability::~FilteredReachability()
{
  ROS_INFO("Shutting Down remove_reachability_obstacles");
  ros::shutdown();
}

void FilteredReachability::readPlanningScene(const moveit_msgs::PlanningScene scene_msg)
{
  ROS_INFO("Planning_scene received");

  if (scene_msg.world.octomap.octomap.data.size() == 0)
  {
    ROS_ERROR("No collision octomap nodes found! Nothing to filter!");
    ros::shutdown();
  }
  octomap_msgs::OctomapWithPose octomap_pose = scene_msg.world.octomap;
  octomap_msgs::Octomap octomap = octomap_pose.octomap;
  octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(octomap);
  collision_octree_ = (octomap::OcTree*)abstract_tree;
  scene_rcvd_ = true;
}

void FilteredReachability::readMap(const map_creator::WorkSpace msg)
{
  // This function stores the incoming map
  ROS_INFO_STREAM("Reachability Map Received! Number of reachability spheres: " << msg.WsSpheres.size());
  reachability_map_ = msg;
  reachability_resolution_ = msg.resolution;
  map_rcvd_ = true;
  /// The reachability map has to be read only once
  Subscriber_reachability_.shutdown();
}

void FilteredReachability::createObstaclesPointCloud(octomap::OcTree& tree,
                                                     pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_vertices)
{
  unsigned int max_depth = tree.getTreeDepth();

  if (!obstacle_vertices->empty())
  {
    obstacle_vertices->clear();
  }
  // Expand collapsed occupied nodes until all occupied leaves are at maximum depth
  std::vector<octomap::OcTreeNode*> collapsed_occ_nodes;

  do
  {
    collapsed_occ_nodes.clear();

    for (octomap::OcTree::iterator it = tree.begin(); it != tree.end(); ++it)
    {
      if (tree.isNodeOccupied(*it) && it.getDepth() < max_depth)
      {
        collapsed_occ_nodes.push_back(&(*it));
      }
    }

    for (std::vector<octomap::OcTreeNode*>::iterator it = collapsed_occ_nodes.begin(); it != collapsed_occ_nodes.end();
         ++it)
    {
      tree.expandNode(*it);
    }
  } while (collapsed_occ_nodes.size() > 0);
  // Create set of occupied voxel centers (and vertices) at max depth
  std::set<std::vector<double> > points_set;

  for (octomap::OcTree::leaf_iterator it = tree.begin_leafs(max_depth), end = tree.end_leafs(); it != end; ++it)
  {
    // Only if node is occupied
    if (tree.isNodeOccupied(*it))
    {
      double voxel_size = it.getSize();

      /// The voxel is converted into limited set of points that are be used for collision checking
      /// The points are the vertices of the box
      for (int dx = -1; dx <= 1; dx = dx + 2)
      {
        for (int dy = -1; dy <= 1; dy = dy + 2)
        {
          for (int dz = -1; dz <= 1; dz = dz + 2)
          {
            std::vector<double> point;
            point.push_back(it.getX() + (dx * voxel_size / 2));
            point.push_back(it.getY() + (dy * voxel_size / 2));
            point.push_back(it.getZ() + (dz * voxel_size / 2));
            // Store points in a set to avoid repitition
            points_set.insert(point);
          }
        }
      }
    }
  }
  ROS_INFO_STREAM("Number of vertices in obstacle point cloud: " << points_set.size());

  // Push points to cloud
  for (std::set<std::vector<double> >::iterator it = points_set.begin(); it != points_set.end(); ++it)
  {
    pcl::PointXYZ cloud_point;
    cloud_point.x = (*it)[0];
    cloud_point.y = (*it)[1];
    cloud_point.z = (*it)[2];
    obstacle_vertices->push_back(cloud_point);
  }
}

void FilteredReachability::createFilteredReachability(FilteredReachability::FilterType type,
                                                      pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& search_tree,
                                                      map_creator::WorkSpace& filtered_map,
                                                      map_creator::WorkSpace& colliding_map)
{
  filtered_map.header = reachability_map_.header;
  filtered_map.resolution = reachability_map_.resolution;
  colliding_map.header = reachability_map_.header;
  colliding_map.resolution = reachability_map_.resolution;

  double circumscribe_reachability_radius = sqrt(3) * reachability_resolution_ / 2.0;
  double inscribe_reachability_radius = reachability_resolution_ / 2.0;

  for (int i = 0; i < reachability_map_.WsSpheres.size(); ++i)
  {
    geometry_msgs::Point32 voxel_center = reachability_map_.WsSpheres[i].point;
    pcl::PointXYZ search_point;
    search_point.x = voxel_center.x;
    search_point.y = voxel_center.y;
    search_point.z = voxel_center.z;

    std::vector<int> point_idx_vec;
    std::vector<float> point_sqrd_dis;

    /// Based on the selected filter type the collision is checked by checking for presence of any of the veretices of
    /// the voxels within the search tree
    switch (type)
    {
      case FilteredReachability::VOXEL:
      {
        search_tree.voxelSearch(search_point, point_idx_vec);
        break;
      }
      case FilteredReachability::INSCRIBED_SPHERE:
      {
        search_tree.radiusSearch(search_point, inscribe_reachability_radius, point_idx_vec, point_sqrd_dis);
        break;
      }
      case FilteredReachability::CIRCUMSCRIBED_SPHERE:
      {
        search_tree.radiusSearch(search_point, circumscribe_reachability_radius, point_idx_vec, point_sqrd_dis);
        break;
      }
    }

    // If the reachability voxel has no collision points inside it
    if (point_idx_vec.size() == 0)
    {
      filtered_map.WsSpheres.push_back(reachability_map_.WsSpheres[i]);
    }
    else
    {
      colliding_map.WsSpheres.push_back(reachability_map_.WsSpheres[i]);
    }
  }
  ROS_INFO("Reachability Map Filtered!");
  ROS_INFO_STREAM("Number of colliding voxels: " << colliding_map.WsSpheres.size());
  ROS_INFO_STREAM("Number of spheres remaining: " << filtered_map.WsSpheres.size());
}

void FilteredReachability::spin(FilterType filter_type)
{
  ros::Rate loop_rate(SPIN_RATE);
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // TODO: Dynamic Map

  while (ros::ok())
  {
    if (scene_rcvd_ && obstacles_cloud->size() == 0)
    {
      // Create obstacle point cloud
      ROS_INFO("Creating obstacles point cloud");
      createObstaclesPointCloud(*collision_octree_, obstacles_cloud);
    }
    else if (!scene_rcvd_)
    {
      ROS_WARN("Awaiting planning scene!");
    }

    if (!map_rcvd_)
    {
      ROS_WARN("Awating reachability_map");
    }

    if (map_rcvd_ && obstacles_cloud->size() != 0)
    {
      std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
      // Obstacle tree will be searched for neighbors
      pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> obstacles_tree(reachability_resolution_);
      obstacles_tree.setInputCloud(obstacles_cloud);
      obstacles_tree.addPointsFromInputCloud();

      // Create filtered reachability map
      map_creator::WorkSpace filtered_map;
      map_creator::WorkSpace colliding_map;
      createFilteredReachability(filter_type, obstacles_tree, filtered_map, colliding_map);

      std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
      std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
      ROS_INFO_STREAM("Time requierd to process map: " << ms.count() << "ms\n");

      // Publish filtered reachability map
      Publisher_filtered_reachability_.publish(filtered_map);
      Publisher_colliding_reachability_.publish(colliding_map);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void FilteredReachability::setFilterType(std::string input, FilterType& filter_type)
{
  if (input.compare("voxel") == 0)
  {
    ROS_INFO("Setting filter type to VOXEL");
    filter_type = FilteredReachability::VOXEL;
  }
  else if (input.compare("circumscribe") == 0)
  {
    ROS_INFO("Setting filter type to CIRCUMSCRIBED SPHERE");
    filter_type = FilteredReachability::CIRCUMSCRIBED_SPHERE;
  }
  else if (input.compare("inscribe") == 0)
  {
    ROS_INFO("Setting filter type to INSCRIBED SPHERE");
    filter_type = FilteredReachability::INSCRIBED_SPHERE;
  }
  else
  {
    ROS_ERROR_STREAM("Invalid filtering type " << input << " receievd. Shutting Down!");
    ros::shutdown();
  }
}

int main(int argc, char** argv)
{
  FilteredReachability::FilterType filter_type;

  if (argc == 1)
  {
    ROS_WARN("No filter type provided. Defaulting to CIRCUMSCRIBED SPHERE!");
    filter_type = FilteredReachability::CIRCUMSCRIBED_SPHERE;
  }
  else
  {
    std::string input = argv[1];
    std::transform(input.begin(), input.end(), input.begin(), ::tolower);

    setFilterType(input, filter_type);
  }

  ros::init(argc, argv, "remove_reachability_obstacles");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  FilteredReachability fr;
  fr.spin(filter_type);
}
