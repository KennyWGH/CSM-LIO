/**
 * Copyright 2023 WANG Guanhua (wangguanhua999@gmail.com)
*/

#include <cmath>
#include <ctime>
#include <string>
#include <deque>
#include <vector>
#include <unordered_map>
#include <iomanip>
#include <thread>
#include <chrono>
#include <sstream>

#include <boost/format.hpp>
#include <boost/make_unique.hpp>
#include <Eigen/Core>
#include <glog/logging.h>
#include <gflags/gflags.h>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/transform_listener.h"
#include <pcl_conversions/pcl_conversions.h>

#include "ros_app/src/node_constants.h"
#include "ros_app/src/trajectory_options.h"
#include "ros_app/src/node_options.h"
#include "ros_app/src/ros_log_sink.h"
#include "ros_app/src/ros_wrapper.h"

#include "absl/memory/memory.h"
#include "csmlio/lio/csm_lidar_inertial_odometry.h"


DEFINE_string(configuration_directory, "",
              "The directory in which configuration files are searched.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");


namespace ros_app {
namespace {

void Run()
{
    constexpr double kTfBufferCacheTimeInSeconds = 10.;
    tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
    tf2_ros::TransformListener tf(tf_buffer);
    NodeOptions node_options;
    TrajectoryOptions trajectory_options;
    std::tie(node_options, trajectory_options) =
        LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

    Node node(node_options, trajectory_options, &tf_buffer);
    node.node_handle()->getNamespace();
    LOG(INFO) << "## Report ros::NodeHandle::getNamespace(): " 
        << node.node_handle()->getNamespace() << " .";
    node.Start();

    ::ros::spin();

    // 保存必要的SLAM文件。
    // node.FinishAllTrajectories();
    // node.RunFinalOptimization();

}

void ConvertSO3ToRPY()
{
//    data: [ 2.67949e-08, -1,  0, 0,
//            1,  2.67949e-08,  0, 0,
//            0,  0,  1, -0.28, 
//            0., 0., 0., 1 ] 


    Eigen::Matrix3d so3;
    so3(0,0) = 2.67949e-08; so3(0,1) = -1;          so3(0,2) = 0;
    so3(1,0) = 1;           so3(1,1) = 2.67949e-08; so3(1,2) = 0;
    so3(2,0) = 0;           so3(2,1) = 0;           so3(2,2) = 1;
    LOG(INFO) << "Calculating transform ... \n" 
        << "From so3 to roll/pitch/yaw: \n" 
        << so3.eulerAngles(0,1,2) << " \n"
        << "From so3.inv to roll/pitch/yaw: \n" 
        << so3.inverse().eulerAngles(0,1,2);
}

} // namespace
} // namespace ros_app


int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    CHECK(!FLAGS_configuration_directory.empty())
        << "-configuration_directory is missing.";
    CHECK(!FLAGS_configuration_basename.empty())
        << "-configuration_basename is missing.";

    ::ros::init(argc, argv, "csm_lio_ros_app");
    ::ros::start();

    ros_app::ScopedRosLogSink ros_log_sink;
    // ros_app::ConvertSO3ToRPY();
    ros_app::Run();
    ::ros::shutdown();
    return 0;
}