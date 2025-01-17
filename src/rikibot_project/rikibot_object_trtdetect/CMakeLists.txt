cmake_minimum_required(VERSION 2.8.3)
project(rikibot_object_trtdetect)

# locate dependencies
find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  sensor_msgs
  cv_bridge
  vision_msgs
  std_msgs
)

find_package(jetson-utils REQUIRED)
find_package(jetson-inference REQUIRED)

find_package(CUDA REQUIRED)
find_package(OpenCV 3 REQUIRED)
#find_package(Qt4 REQUIRED)

#include(${QT_USE_FILE})
#add_definitions(${QT_DEFINITIONS})


# define catkin package
catkin_package(
    CATKIN_DEPENDS roscpp image_transport sensor_msgs  cv_bridge
)

# include paths
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

# enable c++11 (TensorRT requirement)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


add_executable(rikibot_object_trtdetect src/node_detectnet.cpp src/image_converter.cpp)
target_link_libraries(rikibot_object_trtdetect ${catkin_LIBRARIES} jetson-inference)



#if(catkin_EXPORTED_LIBRARIES)
#    add_dependencies(ros_deep_learning_nodelets ${catkin_EXPORTED_LIBRARIES})
#endif()

