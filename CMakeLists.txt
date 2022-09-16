cmake_minimum_required(VERSION 3.2)
project(LIO_Localization)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(catkin REQUIRED COMPONENTS
  	     message_generation
  	     geometry_msgs
  	     nav_msgs
  	     sensor_msgs
  	     roscpp
  	     rospy
  	     std_msgs
  	     tf
  	     eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(Ceres REQUIRED PATHS /home/cc/workspace/3rdparty/ceres_1.14)
# add third part conf
set(3RDPARTY_DIR ${PROJECT_SOURCE_DIR}/../../../workspace/3rdparty)

set(GTSAM_INCLUDE_DIR /home/cc/workspace/3rdparty/gtsam4.2a0/include)
set(GTSAM_LIBRARY_DIRS /home/cc/workspace/3rdparty/gtsam4.2a0/lib)
file(GLOB GTSAM_LIBRARIES ${GTSAM_LIBRARY_DIRS}/*.a ${GTSAM_LIBRARY_DIRS}/*.so*)


message(STATUS "Using catkin version ${catkin_VERSION}")
message(STATUS "Using Ceres version ${CERES_VERSION}")

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)


generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # ${SUITESPARSE_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${CERES_LIBRARY_DIRS}
)

###########
## Build ##
###########

add_executable(${PROJECT_NAME}_featureExtract src/lio/featureExtract.cpp)
target_link_libraries(${PROJECT_NAME}_featureExtract ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME}_poseEstimate 
              src/lio/PoseEstimation.cpp 
              src/lio/Estimator.cpp 
              src/lio/IMUIntegrator.cpp
              src/lio/ceresfunc.cpp 
		          src/lio/Map_Manager.cpp)
target_link_libraries(${PROJECT_NAME}_poseEstimate 
                      ${catkin_LIBRARIES}  
                      ${PCL_LIBRARIES} 
                      ${OpenCV_LIBRARIES} 
                      ${CERES_LIBRARIES} )
					  
add_executable(${PROJECT_NAME}_maplocalization
              src/loc/map_location.cpp 
              src/lio/Estimator.cpp 
              src/lio/IMUIntegrator.cpp
              src/lio/ceresfunc.cpp 
		          src/lio/Map_Manager.cpp)
target_link_libraries(${PROJECT_NAME}_maplocalization 
                      ${catkin_LIBRARIES}  
                      ${PCL_LIBRARIES} 
                      ${OpenCV_LIBRARIES} 
                      ${CERES_LIBRARIES} )