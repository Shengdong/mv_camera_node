cmake_minimum_required (VERSION 2.6)
project(mv_camera_node)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


include_directories(${catkin_INCLUDE_DIRS}
                    /opt/mvIMPACT_Acquire
                    /usr/local/include
                    )

find_package(catkin REQUIRED COMPONENTS
             cmake_modules
             roscpp
             sensor_msgs
             image_transport
             cv_bridge
            )

find_package(mvIMPACT REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
  LIBRARIES mv_camera
  CATKIN_DEPENDS roscpp sensor_msgs
  DEPENDS Eigen3 BOOST OPENCV2
)

###########
## Build ##
###########

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(mv_camera
  src/mvCamInterface.cpp
)
target_link_libraries(mv_camera
  ${catkin_LIBRARIES}
  ${mvIMPACT_LIBRARY} 
  ${OpenCV_LIBS}
)

add_executable(mv_camera_node src/mv_camera_node.cpp)
target_link_libraries(mv_camera_node
  mv_camera
  ${catkin_LIBRARIES}
  ${mvIMPACT_LIBRARY} 
  ${OpenCV_LIBS}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS mv_camera mv_camera_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/mv_camera_node
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


