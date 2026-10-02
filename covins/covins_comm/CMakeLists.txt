cmake_minimum_required(VERSION 2.8.3)
project(covins_comm)

# covins_comm is built in two different ways:
#
#   BUILD_LIBRARY=ON, USE_CATKIN=OFF  -> plain CMake shared library, installed to
#       a prefix and consumed by the ROS 2 ORB-SLAM3 frontend (which links
#       libcovins_comm.so directly).
#   (default)                         -> catkin/catkin_simple package, consumed
#       by covins_backend in the ROS 1 workspace.
#
# The catkin_simple lookup must stay enabled for the second case, otherwise the
# cs_add_library()/cs_install()/cs_export() calls below are unresolvable CMake
# commands and the ROS 1 backend build fails at configure time.
if(NOT BUILD_LIBRARY)
  find_package(catkin_simple REQUIRED)
  catkin_simple(ALL_DEPS_REQUIRED)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-pthread -pipe -Wall -Wextra -pedantic -Wno-unused-parameter -Wno-unused-function -Wno-vla)
add_definitions(-DEIGEN_MAX_ALIGN_BYTES=16 -DEIGEN_MAX_STATIC_ALIGN_BYTES=16)

IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF()

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -march=native")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# The local utils_base.{hpp,cpp} changes replaced the cv::Mat overloads of
# ToEigenMat44d()/ToEigenVec3d() with Sophus::SE3f / Eigen::Vector3f ones
# (ORB-SLAM3 v1.0 switched its pose type from cv::Mat to Sophus::SE3f), so
# covins_comm now needs Sophus headers to compile at all.
#
# Prefer the Sophus vendored inside the ORB-SLAM3 tree we link against: these
# overloads are called across the covins_comm <-> ORB_SLAM3 boundary, and two
# different Sophus versions would mean two different mangled signatures.
find_path(COVINS_SOPHUS_INCLUDE_DIR
    NAMES sophus/se3.hpp
    PATHS
        ${SOPHUS_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/../orb_slam3/Thirdparty/Sophus
        ${PROJECT_SOURCE_DIR}/../../ORB_SLAM3/Thirdparty/Sophus
        /usr/local/include
        /usr/include
    NO_DEFAULT_PATH
)
if(NOT COVINS_SOPHUS_INCLUDE_DIR)
    message(FATAL_ERROR
        "Sophus headers (sophus/se3.hpp) not found. covins_comm requires them "
        "because utils_base.hpp uses Sophus::SE3f. Pass -DSOPHUS_INCLUDE_DIR=...")
endif()
message(STATUS "covins_comm Sophus include dir = ${COVINS_SOPHUS_INCLUDE_DIR}")

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/covins
    ${PROJECT_SOURCE_DIR}/thirdparty
    ${COVINS_SOPHUS_INCLUDE_DIR}
)

set(EXTERNAL_LIBRARIES
  pthread
)

set(COMM_SOURCE_FILES
    src/covins_base/communicator_base.cpp
    src/covins_base/config_comm.cpp
    src/covins_base/utils_base.cpp

    src/covins_base/msgs/msg_keyframe.cpp
    src/covins_base/msgs/msg_landmark.cpp
)

set(COMM_HEADER_FILES
    include/covins/covins_base/communicator_base.hpp
    include/covins/covins_base/config_comm.hpp
    include/covins/covins_base/typedefs_base.hpp
    include/covins/covins_base/utils_base.hpp

    include/covins/covins_base/msgs/msg_keyframe.hpp
    include/covins/covins_base/msgs/msg_landmark.hpp
)

if (NOT BUILD_LIBRARY)

    cs_add_library(covins_comm ${COMM_SOURCE_FILES} ${COMM_HEADER_FILES})
    target_link_libraries(covins_comm ${catkin_LIBRARIES} ${EXTERNAL_LIBRARIES})

    cs_install()
    cs_export()

else()
    if (NOT USE_CATKIN)
        include_directories(${CMAKE_SOURCE_DIR}/thirdparty/cereal)
        # create library
        message("${PROJECT_SOURCE_DIR}/lib")
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

        add_library(${PROJECT_NAME} SHARED ${COMM_SOURCE_FILES} ${COMM_HEADER_FILES})
        option(BUILD_SHARED_LIBS "Build shared library" ON)

        target_link_libraries(${PROJECT_NAME} Eigen3::Eigen  ${OpenCV_LIBS} pthread)

        install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
      )
      
      install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
        DESTINATION include
      )

      # cereal is a PUBLIC dependency of the installed headers, not just an
      # internal build detail: typedefs_base.hpp (pulled in by every covins
      # header) does #include <cereal/cereal.hpp>. Without this, anything
      # consuming the installed tree -- i.e. ORB-SLAM3 -- fails with
      # "fatal error: cereal/cereal.hpp: No such file or directory".
      # thirdparty/ is the include root, so cereal/ lands at <prefix>/include/cereal/.
      install(DIRECTORY ${PROJECT_SOURCE_DIR}/thirdparty/
        DESTINATION include
      )

      # config_comm.hpp resolves its runtime YAML path at COMPILE time from
      # __FILE__, by stripping everything from "include/covins" onwards and
      # appending "config/config_comm.yaml". For installed headers at
      # <prefix>/include/covins/..., that means <prefix>/config/config_comm.yaml.
      # Install the YAML there, otherwise every covins_params value silently
      # initialises from a missing file.
      install(DIRECTORY ${PROJECT_SOURCE_DIR}/config/
        DESTINATION config
      )
    endif()

endif()
