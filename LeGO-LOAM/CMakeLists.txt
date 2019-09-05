cmake_minimum_required(VERSION 2.8.3)
project(lego_loam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
  tf
  tf2_ros
  roscpp
  rospy
  cv_bridge
  image_transport

  pcl_ros
  pcl_conversions

  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cloud_msgs
  # octomap_msgs
  # octomap_ros
  visualization_msgs
)

find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
# find_package(octomap REQUIRED QUIET)
find_package(Eigen3 REQUIRED QUIET)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cloud_msgs
  DEPENDS PCL 
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  # ${OCTOMAP_INCLUIDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
	${GTSAM_LIBRARY_DIRS}
)



add_executable(imageProjection src/imageProjection.cpp)
add_dependencies(imageProjection ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(featureAssociation src/featureAssociation.cpp)
add_dependencies(featureAssociation ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(featureAssociation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(mapOptmization src/mapOptmization.cpp)
target_link_libraries(mapOptmization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

add_executable(transformFusion src/transformFusion.cpp)
target_link_libraries(transformFusion ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(lidar_imu_sub src/lidar_imu.cpp)
target_link_libraries(lidar_imu_sub ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(VIOAssociation src/vio.cpp)
add_dependencies(VIOAssociation ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(VIOAssociation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# add_executable(globaloctomapping  src/globaloctomapping.cpp )
# target_link_libraries(globaloctomapping  ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(pcl2pcd src/pcl2pcd.cpp)
target_link_libraries(pcl2pcd ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(normal_compute src/normal_compute.cpp)
add_dependencies(normal_compute ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(normal_compute ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(normal_compute_ros src/normal_compute_ros.cpp)
add_dependencies(normal_compute_ros ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(normal_compute_ros ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(seg_and_cluster src/SegAndCluster.cpp)
add_dependencies(seg_and_cluster ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(seg_and_cluster ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
