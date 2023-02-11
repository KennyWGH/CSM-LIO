/**
 * Copyright 2016 The Cartographer Authors
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#ifndef ROS_APP_TIME_CONVERSION_H
#define ROS_APP_TIME_CONVERSION_H

#include "infinityslam/common/time.h"
#include "ros/ros.h"

namespace ros_app {

::ros::Time ToRos(::infinityslam::common::Time time);

::infinityslam::common::Time FromRos(const ::ros::Time& time);

}  // namespace ros_app

#endif  // ROS_APP_TIME_CONVERSION_H
