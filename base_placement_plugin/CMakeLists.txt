cmake_minimum_required(VERSION 2.8.3)

project(base_placement_plugin)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  pluginlib
  roscpp
  rviz
  visualization_msgs
  tf
  tf_conversions
  map_creator
  pcl_ros
)


if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
  macro(qt_wrap_cpp)
    qt5_wrap_cpp(${ARGN})
  endmacro()
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)
 

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	interactive_markers
  	pluginlib
  	roscpp
  #rqt_gui
  	rviz
  #rqt_gui_cpp
  	visualization_msgs
  	map_creator
)


include_directories(include ${catkin_INCLUDE_DIRS})

set(base_placement_plugin_HDRS
  include/base_placement_plugin/add_way_point.h
  include/base_placement_plugin/point_tree_item.h
  include/base_placement_plugin/point_tree_model.h
  include/base_placement_plugin/widgets/base_placement_widget.h
  include/base_placement_plugin/place_base.h
  
)

qt_wrap_cpp(base_placement_plugin_MOCS ${base_placement_plugin_HDRS})

set(base_placement_plugin_SRCS
  src/add_way_point.cpp
  src/point_tree_item.cpp
  src/point_tree_model.cpp
  src/widgets/base_placement_widget.cpp
  src/place_base.cpp
  ${base_placement_plugin_MOCS}
  )


set(base_placement_plugin_UIS
  src/widgets/base_placement_widget.ui
)



qt_wrap_ui(UI_GENERATED_HEADERS ${base_placement_plugin_UIS})



add_library(${PROJECT_NAME} ${base_placement_plugin_SRCS} ${base_placement_plugin_UIS} ${UI_GENERATED_HEADERS}) 

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES} yaml-cpp )




find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
