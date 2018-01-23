cmake_minimum_required (VERSION 2.6)
project(mv_camera_node)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


include_directories(${catkin_INCLUDE_DIRS}
                    /opt/mvIMPACT_Acquire
                    /usr/local/include
                    )


find_package(mvIMPACT REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})



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


