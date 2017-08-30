//#include  <map_creator/manipulator_voxel_occupancy_list.h>
#include "../include/map_creator/manipulator_voxel_occupancy_list.h"

manipulator_voxel_occupancy_list::manipulator_voxel_occupancy_list(octomap::OcTree*& tree)
{

  // The set of voxels we are considering will be converted to array of boxes collision object once
  generateBoxesFromOctomap(tree);
}

manipulator_voxel_occupancy_list::~manipulator_voxel_occupancy_list()
{

}

void manipulator_voxel_occupancy_list::getOccupiedVoxels(robot_state::RobotStatePtr state,std::vector<std::vector<double>>& occ_list)
{
//  const collision_detection::CollisionRobot robot;
////  collision_detection::CollisionWorldFCL::checkRobotCollision();
//  const collision_detection::CollisionRobotFCL& robot_fcl = dynamic_cast<const collision_detection::CollisionRobotFCL&>(robot);
//  collision_detection::FCLObject fcl_obj;
//  robot_fcl.constructFCLObject(state, fcl_obj);

//  // fcl_obj is the one we consider
//  std::unique_ptr<fcl::BroadPhaseCollisionManager> manager_;


//  collision_detection::CollisionData cd(&req, &res, acm);
//  cd.enableGroup(robot.getRobotModel());
//  for (std::size_t i = 0; !cd.done_ && i < fcl_obj.collision_objects_.size(); ++i)
//    manager_->collide(fcl_obj.collision_objects_[i].get(), &cd, &fcl::CollisionCallBack);

}

void manipulator_voxel_occupancy_list::generateBoxesFromOctomap(octomap::OcTree*& octomap_tree)
{

  fcl::OcTree* fcl_tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(octomap_tree));
  std::vector<std::array<fcl::FCL_REAL, 6> > boxes_ = fcl_tree->toBoxes();

  for(std::size_t i = 0; i < boxes_.size(); ++i)
  {
    // Center of box
    fcl::FCL_REAL x = boxes_[i][0];
    fcl::FCL_REAL y = boxes_[i][1];
    fcl::FCL_REAL z = boxes_[i][2];
    // Size of box
    fcl::FCL_REAL size = boxes_[i][3];


    fcl::FCL_REAL cost = boxes_[i][4];
    fcl::FCL_REAL threshold = boxes_[i][5];

    fcl::Box* box = new fcl::Box(size, size, size);
    box->cost_density = cost;
    box->threshold_occupied = threshold;
    fcl::CollisionObject* obj = new fcl::CollisionObject(std::shared_ptr<fcl::CollisionGeometry>(box), fcl::Transform3f(fcl::Vec3f(x, y, z)));
    octree_boxes.push_back(obj);
  }

}

bool manipulator_voxel_occupancy_list::checkVoxelOccupancy(std::vector<double> &voxel_center, double &voxel_size)
{

}


////    std::vector<fcl::CollisionObject*> env;

////    fcl::FCL_REAL extents[] = {-1,1,-1,1,-1,1};
////    std::vector<fcl::Transform3f> transforms;
////    fcl::Transform3f tf;
////    // tf is the location of the box
////    fcl::Box* box = new fcl::Box(1,1,1); // sizes of x y and z
////    // This has to be the size of the reach map voxels

////    env.push_back(new fcl::CollisionObject(std::shared_ptr<fcl::CollisionGeometry>(box),tf));


