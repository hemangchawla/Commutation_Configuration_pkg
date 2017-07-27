/* AUTHOR: Hemang Chawla */

/*
 * DESCRIPTION:
 * This program computes the voxels in collision with the reachability map of a robot.
 * A new topic is hence published which removes the reachability spheres in collision
 */

/*
 * APPROACH:
 *
 * - Create the reachability centers list. This list will be used to remove the voxels occupied by the map
 * - Use moveit service to get the octomap
 * - Create the center plus vertices point cloud
 * - Use this obstacle point cloud to filter the reachability octree
 * - Publish new reachability topic
 *
 */

#include <map_creator/remove_reachability_obstacles.h>

remove_obstacles_reachability::remove_obstacles_reachability()
{
  // Initiating and connecting to MoveIt! Octomap planning scene
  Client_get_planning_scene = nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene",true);
  scene_srv.request.components.components = scene_srv.request.components.OCTOMAP;
  // Listen to relavent topics:
  Subscriber_reachability = nh.subscribe("/reachability_map", 1, &remove_obstacles_reachability::readMap, this);
  // reachability gives use the centers of the voxels we will be searching
  Publisher_filtered_reachability = nh.advertise<map_creator::WorkSpace>("/reachability_map_filtered", 0);
  Publisher_colliding_reachability = nh.advertise<map_creator::WorkSpace>("/reachability_map_colliding", 0);

  /* TODO:
   * if (dynamic scene)
   *  The new map will be published at the rate at which the scene is receied.
   * else
   *  The scene is published at selected reate
   */
}

remove_obstacles_reachability::~remove_obstacles_reachability()
{
  ROS_INFO("Shutting Down remove_reachability_obstacles");
  ros::shutdown();
}

void remove_obstacles_reachability::readMap(const map_creator::WorkSpace msg)
{
  // This function stores the incoming map
  ROS_INFO_STREAM( "Reachability Map Received! Number of reachability spheres: "<<msg.WsSpheres.size() );
  reachability_map = msg;
  reachability_resolution = msg.resolution;
}

void remove_obstacles_reachability::createObstaclesPointCloud(octomap::OcTree& tree,   pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_vertices)
{
  unsigned int max_depth = tree.getTreeDepth();

  if( !obstacle_vertices->empty() )
  {
    obstacle_vertices->clear();
  }
  // Expand collapsed occupied nodes until all occupied leaves are at maximum depth
  std::vector<octomap::OcTreeNode*> collapsed_occ_nodes;

  do
  {
    collapsed_occ_nodes.clear();

    for(octomap::OcTree::iterator it = tree.begin(); it != tree.end(); ++it)
    {
      if(tree.isNodeOccupied(*it) && it.getDepth() < max_depth)
      {
        collapsed_occ_nodes.push_back( &(*it) );
      }
    }

    for(std::vector<octomap::OcTreeNode*>::iterator it = collapsed_occ_nodes.begin(); it != collapsed_occ_nodes.end(); ++it)
    {
      tree.expandNode(*it);
    }
  }
  while(collapsed_occ_nodes.size() > 0);
  // Create set of occupied voxel centers (and vertices) at max depth
  std::set<std::vector<double> > points_set;

  for(octomap::OcTree::leaf_iterator it = tree.begin_leafs(max_depth), end = tree.end_leafs(); it != end; ++it)
  {
    // Only if node is occupied
    if( tree.isNodeOccupied(*it) )
    {
      double voxel_size = it.getSize();

      for(int dx = -1; dx <= 1; ++dx)
      {
        for(int dy = -1; dy <= 1; ++dy)
        {
          for(int dz = -1; dz <= 1; ++dz)
          {
            std::vector<double> point;
            point.push_back( it.getX()+(dx*voxel_size/2) );
            point.push_back( it.getY()+(dy*voxel_size/2) );
            point.push_back( it.getZ()+(dz*voxel_size/2) );
            // Store points in a set to avoid repitition
            points_set.insert(point);
          }
        }
      }
    }
  }
  ROS_INFO_STREAM( "Number of vertices in obstacle point cloud: "<<points_set.size() );

  // Push points to cloud
  for(std::set<std::vector<double> >::iterator it = points_set.begin(); it != points_set.end(); ++it)
  {
    pcl::PointXYZ cloud_point;
    cloud_point.x = (*it)[0];
    cloud_point.y = (*it)[1];
    cloud_point.z = (*it)[2];
    obstacle_vertices->push_back(cloud_point);
  }
}

void remove_obstacles_reachability::createFilteredReachability(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& search_tree,
                                                               map_creator::WorkSpace& filtered_map,
                                                               map_creator::WorkSpace& colliding_map)
{

  filtered_map.header = reachability_map.header;
  filtered_map.resolution = reachability_map.resolution;
  colliding_map.header = reachability_map.header;
  colliding_map.resolution = reachability_map.resolution;

  for(int i = 0; i < reachability_map.WsSpheres.size(); ++i)
  {
    geometry_msgs::Point32 voxel_center = reachability_map.WsSpheres[i].point;
    pcl::PointXYZ search_point;
    search_point.x = voxel_center.x;
    search_point.y = voxel_center.y;
    search_point.z = voxel_center.z;

    std::vector<int> point_idx_vec;
    std::vector<float> point_sqrd_dis;
//    search_tree.voxelSearch(search_point, point_idx_vec);
    search_tree.radiusSearch(search_point, sqrt(3)*reachability_resolution/2.0, point_idx_vec, point_sqrd_dis);
    // If the reachability voxel has no collision points inside it
    if(point_idx_vec.size() == 0)
    {
      filtered_map.WsSpheres.push_back(reachability_map.WsSpheres[i]);
    }
    else
    {
      colliding_map.WsSpheres.push_back(reachability_map.WsSpheres[i]);
    }
  }
  ROS_INFO("Reachability Map Filtered!");
  ROS_INFO_STREAM("Number of colliding voxels: " << colliding_map.WsSpheres.size());
  ROS_INFO_STREAM("Number of spheres remaining: "<< filtered_map.WsSpheres.size());

}

void remove_obstacles_reachability::spin()
{
  ros::Rate loop_rate(SPIN_RATE);

  // Static scene (Else the speed of publishing is the limited by the rate of service call)
  octomap::OcTree* collision_octree;
  if(Client_get_planning_scene.call(scene_srv) )
  {
    ROS_INFO("Planning_scene received");
    moveit_msgs::PlanningScene scene_msg = scene_srv.response.scene;
    octomap_msgs::OctomapWithPose octomap_pose = scene_msg.world.octomap;
    octomap_msgs::Octomap octomap = octomap_pose.octomap;
    octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(octomap);
    collision_octree = (octomap::OcTree*)abstract_tree;
  }
  else
  {
    ROS_WARN("Failed to call service /get_planning_scene");
    nh.shutdown();
  }

  // Create obstacle point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles_cloud(new pcl::PointCloud <pcl::PointXYZ>);
  createObstaclesPointCloud(*collision_octree, obstacles_cloud);

  // TODO: Dynamic Map
  // The above lines will be part of the loop


  while(ros::ok())
  {
    std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
    // Obstacle tree will be searched for neighbors
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> obstacles_tree(reachability_resolution);
    obstacles_tree.setInputCloud(obstacles_cloud);
    obstacles_tree.addPointsFromInputCloud();

    // Create filtered reachability map
    map_creator::WorkSpace filtered_map;
    map_creator::WorkSpace colliding_map;
    createFilteredReachability(obstacles_tree, filtered_map, colliding_map);

    std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds ms = std::chrono::duration_cast <std::chrono::milliseconds>(t_end-t_start);
    ROS_INFO_STREAM("Time requierd to process map: "<<ms.count()<<"ms\n");

    // Publish filtered reachability map
    Publisher_filtered_reachability.publish(filtered_map);
    Publisher_colliding_reachability.publish(colliding_map);
    loop_rate.sleep();

  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "remove_reachability_obstacles");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  remove_obstacles_reachability rm;
  rm.spin();
}
