cmake_minimum_required(VERSION 2.8.3)
project(map_creator)

add_compile_options(-std=c++11)
add_compile_options(-std=gnu++11)
add_compile_options(-fext-numeric-literals)

find_package(catkin REQUIRED COMPONENTS
  visualization_msgs
  roscpp
  message_generation
  geometry_msgs
  std_msgs
  tf2
  pcl_ros
  moveit_ros_planning_interface
  moveit_core
  interactive_markers
  moveit_ros_perception
)

find_package(octomap REQUIRED)

add_message_files(
 FILES
 capShape.msg
 capability.msg
 WsSphere.msg
 WorkSpace.msg
 
)

generate_messages(
 DEPENDENCIES
 geometry_msgs std_msgs
)

catkin_package(

  INCLUDE_DIRS
    include
  LIBRARIES
    sphere_discretization
    kinematics
    hdf5_dataset
    roscpp
  CATKIN_DEPENDS message_runtime 
)
set(HDF5_INCLUDE_DIR "/usr/include/hdf5/serial" "/usr/lib/x86_64-linux-gnu/hdf5/serial/include")
include_directories(include ${catkin_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${HDF5_INCLUDE_DIR} ${PCL_INCLUDE_DIR})

add_library(sphere_discretization  src/sphere_discretization.cpp)
add_library(kinematics  src/kinematics.cpp)
add_library(hdf5_dataset  src/hdf5_dataset.cpp)

target_link_libraries(sphere_discretization  ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARY_DIRS})
target_link_libraries(kinematics  ${catkin_LIBRARIES} -llapack)
target_link_libraries(hdf5_dataset  ${catkin_LIBRARIES} -lhdf5_serial -lhdf5_cpp)

install(TARGETS sphere_discretization kinematics hdf5_dataset
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_library(manipulator_voxel_occupancy_list src/manipulator_voxel_occupancy_list.cpp)
add_dependencies(manipulator_voxel_occupancy_list ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(manipulator_voxel_occupancy_list ${catkin_LIBRARIES})

add_executable(create_reachability_map src/create_reachability_map.cpp)
add_dependencies(create_reachability_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(create_reachability_map sphere_discretization kinematics  hdf5_dataset ${catkin_LIBRARIES})

add_executable(create_inverse_reachability_map src/create_inverse_reachability_map.cpp)
add_dependencies(create_inverse_reachability_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(create_inverse_reachability_map sphere_discretization kinematics  hdf5_dataset ${catkin_LIBRARIES})

add_executable(create_capability_map src/create_capability_map.cpp)
add_dependencies(create_capability_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(create_capability_map sphere_discretization kinematics hdf5_dataset ${catkin_LIBRARIES})

add_executable(load_reachability_map src/load_reachability_map.cpp)
add_dependencies(load_reachability_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(load_reachability_map  hdf5_dataset ${catkin_LIBRARIES})

add_executable(load_capability_map src/load_capability_map.cpp)
add_dependencies(load_capability_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(load_capability_map  hdf5_dataset ${catkin_LIBRARIES})

add_executable(remove_reachability_obstacles src/remove_reachability_obstacles.cpp)
add_dependencies(remove_reachability_obstacles ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(remove_reachability_obstacles sphere_discretization hdf5_dataset ${catkin_LIBRARIES})



if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(utest test/utest.cpp)
  target_link_libraries(utest sphere_discretization ${catkin_LIBRARIES})
endif()
