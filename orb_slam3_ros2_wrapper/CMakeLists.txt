cmake_minimum_required(VERSION 3.5)
project(orb_slam3_ros2_wrapper LANGUAGES C CXX)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
# set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# You should set the PYTHONPATH to your own python site-packages path
set(ENV{PYTHONPATH} "/opt/ros/jazzy/lib/python3.12/site-packages/")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(slam_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_srvs REQUIRED)
find_package(MPI REQUIRED C)


option(ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA "Enable CUDA support if available" OFF)

if(ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA)
  include(CheckLanguage)
  enable_language(CUDA)
  message(WARNING "CUDA support requested for ORB_SLAM3_ROS2_WRAPPER")
endif()

# This fork grafts the COVINS agent communicator into ORB-SLAM3, which makes
# COVINS a PUBLIC dependency of ORB-SLAM3's headers: ImuTypes.h does
# #include <covins/covins_base/typedefs_base.hpp>, and that in turn includes
# <cereal/cereal.hpp>. So anything including ORB-SLAM3 headers -- this wrapper
# included -- needs the COVINS install tree on its include path.
find_path(COVINS_INCLUDE_DIR
  NAMES covins/covins_base/typedefs_base.hpp
  PATHS $ENV{COVINS_PREFIX}/include $ENV{HOME}/.local/include
        /usr/local/include /usr/include
)
if(NOT COVINS_INCLUDE_DIR)
  message(FATAL_ERROR
    "COVINS headers not found (expected covins/covins_base/typedefs_base.hpp "
    "under $ENV{HOME}/.local/include). ORB-SLAM3's public headers include them, "
    "so this package cannot compile without them. Build and install "
    "covins/covins_comm with -DBUILD_LIBRARY=ON -DUSE_CATKIN=OFF first.")
endif()
message(STATUS "orb_slam3_ros2_wrapper COVINS_INCLUDE_DIR = ${COVINS_INCLUDE_DIR}")

include_directories(
  include
  ${COVINS_INCLUDE_DIR}
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  # Sophus. Deliberately the copy vendored in the ORB-SLAM3 tree we link
  # against, NOT ros-<distro>-sophus:
  #   * find_package(Sophus) above does not put its headers on the include path
  #     (ros-jazzy-sophus installs a flat /opt/ros/jazzy/include/sophus, which
  #     is not a default include dir), so <sophus/se3.hpp> fails without this.
  #   * Sophus::SE3f crosses the wrapper <-> libORB_SLAM3.so <-> libcovins_comm.so
  #     boundaries, so all three must instantiate the same Sophus templates.
  #     Mixing versions is an ODR violation waiting to happen.
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
)

if(ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA)
  include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
endif()

link_directories(
  include
)

set(dependencies 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  message_filters 
  ORB_SLAM3 
  Pangolin 
  tf2_ros 
  tf2_eigen 
  slam_msgs 
  pcl_ros 
  pcl_conversions 
  PCL 
  nav_msgs 
  std_srvs
)

add_library(orb_slam3_ros2_wrapper_common SHARED
  src/type_conversion.cpp
  src/orb_slam3_interface.cpp
  src/time_profiler.cpp
  src/slam_node_base.cpp
)
ament_target_dependencies(orb_slam3_ros2_wrapper_common ${dependencies})
if(ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA)
  target_compile_definitions(orb_slam3_ros2_wrapper_common PRIVATE ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA=1)
endif()


add_executable(rgbd
  src/rgbd/rgbd-slam-node.cpp
  src/rgbd/rgbd.cpp
)
ament_target_dependencies(rgbd ${dependencies})
target_link_libraries(rgbd orb_slam3_ros2_wrapper_common ${PCL_LIBRARIES})
if(ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA)
  set_target_properties(rgbd PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
  target_compile_definitions(rgbd PRIVATE ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA=1)
endif()


add_executable(rgbd_imu
  src/rgbd-imu/rgbd-imu-slam-node.cpp
  src/rgbd-imu/rgbd_imu.cpp
)
ament_target_dependencies(rgbd_imu ${dependencies})
target_link_libraries(rgbd_imu orb_slam3_ros2_wrapper_common ${PCL_LIBRARIES})
if(ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA)
  set_target_properties(rgbd_imu PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
  target_compile_definitions(rgbd_imu PRIVATE ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA=1)
endif()


add_executable(mono_imu
  src/mono-imu/mono-imu-slam-node.cpp
  src/mono-imu/mono_imu.cpp
)
ament_target_dependencies(mono_imu ${dependencies})
target_link_libraries(mono_imu orb_slam3_ros2_wrapper_common ${PCL_LIBRARIES})
if(ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA)
  set_target_properties(mono_imu PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
  target_compile_definitions(mono_imu PRIVATE ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA=1)
endif()


add_executable(stereo
  src/stereo/stereo-slam-node.cpp
  src/stereo/stereo.cpp
)
ament_target_dependencies(stereo ${dependencies})
target_link_libraries(stereo orb_slam3_ros2_wrapper_common ${PCL_LIBRARIES})
if(ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA)
  set_target_properties(stereo PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
  target_compile_definitions(stereo PRIVATE ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA=1)
endif()


# Libraries and executables must go to DIFFERENT destinations.
#
# ament puts <prefix>/lib on the runtime library path, but NOT
# <prefix>/lib/<project>, which is where `ros2 run` looks for executables.
# Installing the shared library alongside the executables (as a single
# install(TARGETS ...) DESTINATION lib/${PROJECT_NAME} does) leaves it off the
# loader path, and the node dies at startup with:
#
#   rgbd: error while loading shared libraries:
#   liborb_slam3_ros2_wrapper_common.so: cannot open shared object file
#
# So: library -> lib/, executables -> lib/${PROJECT_NAME}.
install(TARGETS orb_slam3_ros2_wrapper_common
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS rgbd rgbd_imu mono_imu stereo
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params
DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/sync_repub.py
  scripts/monitor_cpu_ram.sh
  scripts/get_landmarks_for_pose.py
  scripts/viz_keyframes.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(gtest)
  find_package(ament_cmake_gtest REQUIRED)
  ament_find_gtest()
  include_directories(${GTEST_INCLUDE_DIRS})
  link_directories(${GTEST_LIBRARY_DIRS})
  
  ament_add_gtest(typeconversionTests tests/typeConversionsTests.cpp src/type_conversion.cpp)
  ament_target_dependencies(typeconversionTests rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin tf2_ros tf2_eigen slam_msgs pcl_ros pcl_conversions PCL nav_msgs)
endif()

ament_package()
