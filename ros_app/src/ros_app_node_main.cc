/**
 * Copyright 2022 WANG Guanhua (wangxxx@gmail.com)
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
#include "ros_app/src/ros_wrapper.h"
#include "ros_app/src/ros_wrapper_options.h"
#include "ros_app/src/ros_log_sink.h"
#include "ros_app/src/tmp_test.h"

#include "infinityslam/csmlio/csm_lidar_inertial_odometry.h"


DEFINE_string(ros_config_file, "./src/CSM-LIO/config/ros_config_file.yaml", 
    "The configuration file from which ros-wrapper options are loaded.");
DEFINE_string(slam_config_file, "./src/CSM-LIO/config/slam_config_file.yaml", 
    "The configuration file from which slam options are loaded.");

namespace ros_app {
namespace {

void Run()
{
    constexpr double kTfBufferCacheTimeInSeconds = 10.;
    tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
    tf2_ros::TransformListener tf(tf_buffer);

    // LoadRosWrapperOptions(FLAGS_ros_config_file, /*logging=*/true); // TODO: debug yaml reading error.
    RosWrapperOptions ros_wrapper_options = ReadRosWrapperOptions();

    Node node(ros_wrapper_options, &tf_buffer);

    LOG(INFO) << "## Report ros::NodeHandle::getNamespace(): " 
        << node.node_handle()->getNamespace() << " .";
    node.Start();

    ::ros::spin();

    // 保存必要的SLAM文件。
    // node.FinishAllTrajectories();
    // node.RunFinalOptimization();

}

} // namespace
} // namespace ros_app


int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    CHECK(!FLAGS_ros_config_file.empty())
        << "-ros_config_file is missing.";
    CHECK(!FLAGS_slam_config_file.empty())
        << "-slam_config_file is missing.";

    ::ros::init(argc, argv, "csm_lio_ros_app");
    ::ros::start();

    ros_app::ScopedRosLogSink ros_log_sink;
    // ros_app::ConvertSO3ToRPY();
    ros_app::Run();
    ::ros::shutdown();
    return 0;
}