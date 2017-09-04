#include  <map_creator/manipulator_voxel_occupancy_list.h>

manipulator_voxel_occupancy_list::manipulator_voxel_occupancy_list(octomap::OcTree*& tree, const collision_detection::CollisionRobotConstPtr& collision_robot_)
{
  // The set of voxels we are considering will be converted to array of boxes collision object once
  generateBoxesFromOctomap(tree);

  // Collision Robot pointer for FCL
  robot_fcl_ptr_ = &dynamic_cast<const collision_detection::CollisionRobotFCL&>(*collision_robot_);

  // Collision Manager
  manager_ = new fcl::DynamicAABBTreeCollisionManager();
}

manipulator_voxel_occupancy_list::~manipulator_voxel_occupancy_list()
{
}

void manipulator_voxel_occupancy_list::getOccupiedVoxels(robot_state::RobotStatePtr state_,  std::vector<std::vector<double> >& occ_list)
{
  // From the state a set of robot collision object has to be constructed
  collision_detection::FCLObject fcl_obj;

  // Construct collision objects for the arm

  state_->updateCollisionBodyTransforms();
  robot_fcl_ptr_->constructFCLObject(*state_, fcl_obj);

  // Arm is to be checked against all boxes in the list for collision

  // Setup collision manager with arm links
  // TODO: broadphase manager vs double for loops

  fcl_obj.registerTo(manager_);
  std::vector<fcl::CollisionObject*> arm_objs;
  manager_->getObjects(arm_objs);

//   // iterate over octree_boxes and check for collision with robot
  for(size_t i = 0; i < octree_boxes.size(); ++i)
  {
    for(size_t j = 0; j < arm_objs.size(); ++j)
    {
      res.clear();
      fcl::collide(arm_objs[j], octree_boxes[i], req, res);

      if( res.isCollision() )
      {
        std::vector<double> center;
        center.push_back(octree_boxes[i]->getTranslation().data[0]);
        center.push_back(octree_boxes[i]->getTranslation().data[1]);
        center.push_back(octree_boxes[i]->getTranslation().data[2]);
        occ_list.push_back(center);
        break;
      }
    }
  }
  fcl_obj.unregisterFrom(manager_);
  return;
}

void manipulator_voxel_occupancy_list::generateBoxesFromOctomap(octomap::OcTree*& octomap_tree)
{
  fcl::OcTree* fcl_tree = new fcl::OcTree( std::shared_ptr<const octomap::OcTree>(octomap_tree) );
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
    fcl::CollisionObject* obj = new fcl::CollisionObject( std::shared_ptr<fcl::CollisionGeometry>(box), fcl::Transform3f( fcl::Vec3f(x, y, z) ) );
    octree_boxes.push_back(obj);
  }
}
