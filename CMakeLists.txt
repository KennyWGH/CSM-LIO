cmake_minimum_required(VERSION 3.2)
project(csm_lio)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## 添加版本号
set(CSMLIO_MAJOR_VERSION 1)
set(CSMLIO_MINOR_VERSION 0)
set(CSMLIO_PATCH_VERSION 0)
set(CSMLIO_VERSION ${CSMLIO_MAJOR_VERSION}.${CSMLIO_MINOR_VERSION}.${CSMLIO_PATCH_VERSION}) # 库版本号
set(CSMLIO_SOVERSION ${CSMLIO_MAJOR_VERSION}.${CSMLIO_MINOR_VERSION}.${CSMLIO_PATCH_VERSION}) # API版本号

## Initialize project, see .cmake script for details.
## 初始化一些配置信息，包括{CMAKE_MODULE_PATH、BUILD_TYPE,、编译器选项}，查阅脚本获取详细信息。
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/functions.cmake")
initialize_this_project()
message(STATUS "######### CMAKE_BUILD_TYPE : " ${CMAKE_BUILD_TYPE})
message(STATUS "######### CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})
message(STATUS "######### CXX_COMPILE_FLAGS: " ${CXX_COMPILE_FLAGS})


## ##################################
## Q: CMAKE_MODULE_PATH 定义了什么？
## Q: find_package()做了什么？
## ##################################


## #########################################################################
## #################         构建 CSM_LIO CORE 部分         #################
## #########################################################################

find_package(absl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread iostreams)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
message(STATUS "######### absl_FOUND        : " ${absl_FOUND})
message(STATUS "######### absl_INCLUDE_DIR  : " ${absl_INCLUDE_DIR})
message(STATUS "######### absl_LIBRARIES    : " ${abslLIBRARIES})
message(STATUS "######### EIGEN3_INCLUDE_DIR: " ${EIGEN3_INCLUDE_DIR})
message(STATUS "######### EIGEN3_LIBRARIES  : " ${EIGEN3_LIBRARIES})
message(STATUS "######### Boost_INCLUDE_DIRS: " ${Boost_INCLUDE_DIRS})
message(STATUS "######### Boost_LIBRARY_DIRS: " ${Boost_LIBRARY_DIRS})
message(STATUS "######### Boost_LIBRARIES   : " ${Boost_LIBRARIES})
if(NOT absl_FOUND)
  message(FATAL_ERROR "absl not found.")
endif()

find_package(Protobuf 3.0.0 REQUIRED)

include(FindPkgConfig)
PKG_SEARCH_MODULE(CAIRO REQUIRED cairo>=1.12.16)

find_package(yaml-cpp REQUIRED)
message(STATUS "######### Is yaml-cpp found   : " yaml-cpp_FOUND)
message(STATUS "######### YAML_CPP_INCLUDE_DIR: " ${YAML_CPP_INCLUDE_DIR})
message(STATUS "######### YAML_CPP_LIBRARIES  : " ${YAML_CPP_LIBRARIES})

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# 打印查阅一些必要的路径信息。
message(STATUS "## ############################################################# ##")
message(STATUS "## CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "## PROJECT_SOURCE_DIR      : ${PROJECT_SOURCE_DIR}")
string(REGEX REPLACE "(.*)/(.*)" "\\1" CMAKE_PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "## CMAKE_PARENT_DIR        : ${CMAKE_PARENT_DIR}")
# set(CMAKE_INSTALL_PREFIX ${CMAKE_PARENT_DIR}/cartographer_ros/cartographer)
message(STATUS "## CMAKE_INSTALL_PREFIX    : ${CMAKE_INSTALL_PREFIX}")
message(STATUS "## ############################################################# ##")

## 安装文件，主要是配置文件。
## 有哪些文件需要安装呢？ TODO

## 捕获所有源文件。
## 使用GLOB捕获源文件的一个缺点是：新增或移除源文件时，cmake无法感知这种改变，也就不会regenerate，
## 解决方法是在初始化项目的宏中手动设置regenerate条件。
file(GLOB_RECURSE ALL_HDRS "*.h")
file(GLOB_RECURSE ALL_SRCS "*.cc")
file(GLOB_RECURSE ALL_CORE_HDRS "infinityslam/*.h")
file(GLOB_RECURSE ALL_CORE_SRCS "infinityslam/*.cc")
file(GLOB_RECURSE ALL_WRAPPER_HDRS "ros_app/*.h")
file(GLOB_RECURSE ALL_WRAPPER_SRCS "ros_app/*.cc")
file(GLOB_RECURSE ALL_EXECUTABLES "*_main.cc")
file(GLOB_RECURSE ALL_TESTS "*_test.cc")
# message(STATUS "######### ALL_EXECUTABLES: " ${ALL_EXECUTABLES})
# message(STATUS "######### ALL_TESTS: " ${ALL_TESTS})
# message(STATUS "######### ALL_CORE_HDRS: " ${ALL_CORE_HDRS})
list(REMOVE_ITEM ALL_SRCS ${ALL_EXECUTABLES} ${ALL_TESTS})
list(REMOVE_ITEM ALL_CORE_SRCS ${ALL_EXECUTABLES} ${ALL_TESTS})

## 捕获需要安装的头文件（作为库接口对外可见）——我不需要。
## 捕获和逐个编译.proto文件。 ——我不需要。

## 构建项目
set(PROJECT_CORE "${PROJECT_NAME}_core")
add_library(${PROJECT_CORE} STATIC ${ALL_CORE_HDRS} ${ALL_CORE_SRCS})

target_include_directories(${PROJECT_CORE} SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}")
target_link_libraries(${PROJECT_CORE} PUBLIC ${EIGEN3_LIBRARIES})

target_include_directories(${PROJECT_CORE} SYSTEM PUBLIC
  "${CERES_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_CORE} PUBLIC ${CERES_LIBRARIES})

target_include_directories(${PROJECT_CORE} SYSTEM PUBLIC
  "${YAML_CPP_INCLUDE_DIR}")
target_link_libraries(${PROJECT_CORE} PUBLIC ${YAML_CPP_LIBRARIES})

target_include_directories(${PROJECT_CORE} SYSTEM PUBLIC
  "${Boost_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_CORE} PUBLIC ${Boost_LIBRARIES})

set(GLOG_LIBRARY glog)
target_link_libraries(${PROJECT_CORE} PUBLIC ${GLOG_LIBRARY})
target_link_libraries(${PROJECT_CORE} PUBLIC gflags)

if("${CAIRO_INCLUDE_DIRS}")
  target_include_directories(${PROJECT_CORE} SYSTEM PUBLIC
    "${CAIRO_INCLUDE_DIRS}")
endif()
target_link_libraries(${PROJECT_CORE} PUBLIC ${CAIRO_LIBRARIES})

target_include_directories(${PROJECT_CORE} SYSTEM PUBLIC
  ${PROTOBUF_INCLUDE_DIR})
# TODO(hrapp): This should not explicitly list pthread and use
# PROTOBUF_LIBRARIES, but that failed on first try.
target_link_libraries(${PROJECT_CORE} PUBLIC ${PROTOBUF_LIBRARY} 
  absl::algorithm
  absl::base
  absl::debugging
  absl::flat_hash_map
  absl::memory
  absl::meta
  absl::numeric
  absl::str_format
  absl::strings
  absl::synchronization
  absl::time
  absl::utility 
)

set(CORE_COMPILE_FLAGS "${CORE_COMPILE_FLAGS} ${CXX_COMPILE_FLAGS}")
message(STATUS "######### CORE_COMPILE_FLAGS: ${CORE_COMPILE_FLAGS}")
set_target_properties(${PROJECT_CORE} PROPERTIES
  COMPILE_FLAGS ${CORE_COMPILE_FLAGS})

## done.


## #########################################################################
## #################        构建 ROS_APPLICATION 部分       #################
## #########################################################################

## Find catkin macros and libraries and other dependices through CMmake's conventions
## catkin是ROS下的构建工具，同时貌似也是众多ros基础package的总称，我们要用到这些。 
## catkin_make 如果更名为 ros_make（just like cmake），岂不是更方便小白理解？（20200312）
set(CATKIN_DEPENDENCIES
  roscpp
  roslib
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  # message_runtime
  # rosbag
  tf2
  tf2_eigen
  tf2_ros
  urdf
  visualization_msgs
  # tf
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDENCIES})
# 该命令输出：Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy

message(STATUS "######### catkin_INCLUDE_DIRS: " ${catkin_INCLUDE_DIRS})
message(STATUS "######### catkin_LIBRARY_DIRS: " ${catkin_LIBRARY_DIRS})
# message(STATUS "######### catkin_LIBRARIES: " ${catkin_LIBRARIES})

# urdfdom_headers是一个ROS包，我们需要借助它来读取和解析urdf文件；
# 在ubuntu16/18等不是很老的发行版对应的ROS版本中，Version肯定是大于0.4.1的。
find_package(urdfdom_headers REQUIRED)
if(DEFINED urdfdom_headers_VERSION)
  if(${urdfdom_headers_VERSION} GREATER 0.4.1)
    add_definitions(-DURDFDOM_HEADERS_HAS_SHARED_PTR_DEFS)
    message(STATUS "######### urdfdom_headers > 0.4.1 was found.")
  endif()
endif()
message(STATUS "######### urdfdom_headers_VERSION     : " ${urdfdom_headers_VERSION})
message(STATUS "######### urdfdom_headers_INCLUDE_DIRS: " ${urdfdom_headers_INCLUDE_DIRS})
message(STATUS "######### urdfdom_headers_LIBRARIES   : " ${urdfdom_headers_LIBRARIES})
include_directories(
  ${urdfdom_headers_INCLUDE_DIRS}
)

## #### Find extra dependencies for ROS_APP. ####
find_package(PCL REQUIRED COMPONENTS common)
# find_package(PCL 1.8 REQUIRED)
message(STATUS "######### PCL_VERSION     : " ${PCL_VERSION})
message(STATUS "######### PCL_INCLUDE_DIRS: " ${PCL_INCLUDE_DIRS})
message(STATUS "######### PCL_LIBRARY_DIRS: " ${PCL_LIBRARY_DIRS})
message(STATUS "######### PCL_LIBRARIES   : " ${PCL_LIBRARIES})
if(NOT PCL_FOUND)
  # message(FATAL_ERROR "PCL >= 1.8 not found.")
  message(FATAL_ERROR "PCL not found.")
endif()

## catkin_package的声明必须放在任何add_library和add_executable之前，
## 否则库文件/或可执行文件可能不会安装到devel目录下，导致roslaunch/
## 或rosrun找不到可执行文件。
# ## catkin specific configuration ##
# ## 声明该项目为一个 catkin package
# ## wgh-- 当作者发布了自己的catkin包，供别人调用时，作者必须设置好
# ## wgh-- 自己的头文件，库文件，以及自己依赖的其他catkin包或系统依赖项。
# ## wgh-- catkin_package()正是服务于这个目的的。
# ## wgh-- 第1、2行制定了所发布包的头文件和库文件；
# ## wgh-- 第3行指定了所发布包依赖的其它catkin包；
# ## wgh-- 第4行指定了所发布包依赖的其它（如操作系统中的）依赖项。
# catkin_package(
# #  INCLUDE_DIRS include
# #  LIBRARIES csm_lio
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
# #  DEPENDS system_lib
# )

catkin_package(
  CATKIN_DEPENDS
    ${CATKIN_DEPENDENCIES}
  DEPENDS
    # TODO(damonkohler): This should be here but causes Catkin to abort because
    # protobuf specifies a library '-lpthread' instead of just 'pthread'.
    # CARTOGRAPHER
    PCL
    EIGEN3
    Boost
    urdfdom_headers
  INCLUDE_DIRS "."
  # LIBRARIES ${PROJECT_CORE} ${PROJECT_ROS_APP_NODE}
)

set(ROS_APP_SRCS 
  ros_app/src/ros_app_node_main.cc
  ros_app/src/ros_wrapper.cc
  ros_app/src/ros_wrapper_options.cc
  ros_app/src/time_conversion.cc
  ros_app/src/msg_conversion.cc
  ros_app/src/node_constants.cc
  ros_app/src/trajectory_options.cc
  ros_app/src/node_options.cc
  ros_app/src/ros_log_sink.cc
  ros_app/src/urdf_reader.cc
  ros_app/src/tf_bridge.cc
  # ros_app/src/
  # ros_app/src/
  # ros_app/src/
  # ros_app/src/
  # ros_app/src/
  # ros_app/src/
)

set(PROJECT_ROS_APP_NODE "${PROJECT_NAME}_ros_app") #可执行文件名应与ROS节点名相同 
add_executable(${PROJECT_ROS_APP_NODE} ${ROS_APP_SRCS})

target_link_libraries(${PROJECT_ROS_APP_NODE} PUBLIC ${PROJECT_CORE})

# yaml-cpp
target_include_directories(${PROJECT_ROS_APP_NODE} SYSTEM PUBLIC ${YAML_CPP_INCLUDE_DIR})

# PCL
target_include_directories(${PROJECT_ROS_APP_NODE} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_ROS_APP_NODE} PUBLIC ${PCL_LIBRARIES})
set(BLACKLISTED_PCL_DEFINITIONS " -march=native -msse4.2 -mfpmath=sse ")
set(ROS_APP_COMPILE_FLAGS "${CXX_COMPILE_FLAGS}")
foreach(DEFINITION ${PCL_DEFINITIONS})
  list (FIND BLACKLISTED_PCL_DEFINITIONS "${DEFINITION}" DEFINITIONS_INDEX)
  if (${DEFINITIONS_INDEX} GREATER -1)
    continue()
  endif()
  set(ROS_APP_COMPILE_FLAGS "${ROS_APP_COMPILE_FLAGS} ${DEFINITION}")
endforeach()

# Eigen
target_include_directories(${PROJECT_ROS_APP_NODE} SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}")

# Boost
target_include_directories(${PROJECT_ROS_APP_NODE} SYSTEM PUBLIC
  "${Boost_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_ROS_APP_NODE} PUBLIC ${Boost_LIBRARIES})

# Catkin
target_include_directories(${PROJECT_ROS_APP_NODE} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_ROS_APP_NODE} PUBLIC ${catkin_LIBRARIES})
# add_dependencies(${PROJECT_ROS_APP_NODE} ${catkin_EXPORTED_TARGETS})

# # Add the binary directory first, so that port.h is included after it has
# # been generated.
# target_include_directories(${PROJECT_ROS_APP_NODE} PUBLIC
#     $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
#     $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
#     $<INSTALL_INTERFACE:include>
# )

message(STATUS "######### CXX_COMPILE_FLAGS    : ${CXX_COMPILE_FLAGS}") 
message(STATUS "######### ROS_APP_COMPILE_FLAGS: ${ROS_APP_COMPILE_FLAGS}")
set_target_properties(${PROJECT_ROS_APP_NODE} PROPERTIES
  COMPILE_FLAGS ${ROS_APP_COMPILE_FLAGS})


## #########################################################################
## #################    构建 Online Dynamic Remover 部分    #################
## #########################################################################

# # TODO: 如果找到了文件夹和CMakeLists文件，才执行编译。

# set(DYNC_REMOVER_SRCS 
#   online_dynamic_remover/online_dynamic_remover.cpp
#   online_dynamic_remover/points_batch.cc
# )

# set(DYNC_REMOVER_HDRS 
#   online_dynamic_remover/online_dynamic_remover.h
#   online_dynamic_remover/points_batch.h
#   online_dynamic_remover/incremental_cube.h
# )

# set(PROJECT_DYNC_REMOVER_NODE "online_dynamic_removal_and_mapping") #可执行文件名应与ROS节点名相同 

# add_library(${PROJECT_DYNC_REMOVER_NODE} STATIC ${DYNC_REMOVER_HDRS} ${DYNC_REMOVER_SRCS})

# target_link_libraries(${PROJECT_DYNC_REMOVER_NODE} PUBLIC ${PROJECT_CORE})

# # yaml-cpp
# target_include_directories(${PROJECT_ROS_APP_NODE} SYSTEM PUBLIC ${YAML_CPP_INCLUDE_DIR})

# # PCL
# target_include_directories(${PROJECT_DYNC_REMOVER_NODE} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
# target_link_libraries(${PROJECT_DYNC_REMOVER_NODE} PUBLIC ${PCL_LIBRARIES})

# # Eigen
# target_include_directories(${PROJECT_DYNC_REMOVER_NODE} SYSTEM PUBLIC
#   "${EIGEN3_INCLUDE_DIR}")

# # Boost
# target_include_directories(${PROJECT_DYNC_REMOVER_NODE} SYSTEM PUBLIC
#   "${Boost_INCLUDE_DIRS}")
# target_link_libraries(${PROJECT_DYNC_REMOVER_NODE} PUBLIC ${Boost_LIBRARIES})

# # Catkin
# target_include_directories(${PROJECT_DYNC_REMOVER_NODE} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
# target_link_libraries(${PROJECT_DYNC_REMOVER_NODE} PUBLIC ${catkin_LIBRARIES})

# set(DYNC_REMOVER_COMPILE_FLAGS "${CORE_COMPILE_FLAGS}")
# message(STATUS "######### DYNC_REMOVER_COMPILE_FLAGS: ${DYNC_REMOVER_COMPILE_FLAGS}")
# set_target_properties(${PROJECT_DYNC_REMOVER_NODE} PROPERTIES
#   COMPILE_FLAGS ${DYNC_REMOVER_COMPILE_FLAGS})


