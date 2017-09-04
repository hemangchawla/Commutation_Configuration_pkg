#include <ros/ros.h>
#include <map_creator/manipulator_voxel_occupancy_list.h>
#include <octomap/octomap.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>
#include <map_creator/sphere_discretization.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_state_voxels");
  ros::NodeHandle nh;

  // Construct octree of reachability voxels (all possible)
  if( ros::ok() )
  {
    float reach_size = 1.5;
    float resolution = 0.08;

    if(argc == 2)
    {
      resolution = atof(argv[1]);
    }
    octomap::point3d origin = octomap::point3d(0, 0, 0);
    sphere_discretization::SphereDiscretization sd;
    octomap::OcTree *tree = sd.generateBoxTree(origin, reach_size, resolution);

    // Load MoveIt robot and scene
    robot_model_loader::RobotModelLoader rml("robot_description");
    robot_model::RobotModelPtr kinematic_model = rml.getModel();
    planning_scene::PlanningScene scene(kinematic_model);
    robot_state::RobotStatePtr kinematic_state( new robot_state::RobotState(kinematic_model) );

    // Set state to random positions
    kinematic_state->setToRandomPositions();

    //  Find voxels the state occupies
    manipulator_voxel_occupancy_list mvol( tree, scene.getCollisionRobot() );

    std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<double> > occ_list;
    occ_list.clear();
    mvol.getOccupiedVoxels(kinematic_state, occ_list);
    std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds ms = std::chrono::duration_cast <std::chrono::milliseconds>(t_end-t_start);
    ROS_INFO_STREAM("Time requierd to compute occupied voxels: "<<ms.count()<<"ms\n");

    // Show state
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
    visual_tools_.reset( new moveit_visual_tools::MoveItVisualTools("base_link", "occupied_voxels") );
    visual_tools_->deleteAllMarkers();
    visual_tools_->loadMarkerPub();
    visual_tools_->loadRobotStatePub("test_state");

    visual_tools_->publishRobotState(kinematic_state);
    visual_tools_->trigger();

    ROS_INFO_STREAM( "Number of voxels in state:"<<occ_list.size() );

    for(int i = 0; i < occ_list.size(); ++i)
    {
      geometry_msgs::Pose p;
      p.position.x = occ_list[i][0];
      p.position.y = occ_list[i][1];
      p.position.z = occ_list[i][2];
      p.orientation.x = 0;
      p.orientation.y = 0;
      p.orientation.z = 0;
      p.orientation.w = 0;
      visual_tools_->publishCuboid(p, resolution, resolution, resolution, rviz_visual_tools::RED);
    }
    visual_tools_->trigger();
    ros::Rate(1).sleep();
    ros::spinOnce();
  }
  return 0;
}
