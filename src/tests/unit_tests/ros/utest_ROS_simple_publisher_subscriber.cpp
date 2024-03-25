// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// Test for a simple publisher and subscriber using the Chrono::ROS interface
//
// =============================================================================

#include "gtest/gtest.h"

#include "chrono_ros/ChROSInterface.h"

#include "std_msgs/msg/int64.hpp"

TEST(ChROSInterface, simple_publisher_subscriber) {
    // Create the ROS interface
    auto interface = chrono::ros::ChROSInterface("chrono_ros_node");

    // Initialize the ROS interface
    interface.Initialize();

    // Create a publisher
    auto publisher = interface.GetNode()->create_publisher<std_msgs::msg::Int64>("test_topic", 1);

    // Create a subscriber
    int64_t data;
    auto subscriber = interface.GetNode()->create_subscription<std_msgs::msg::Int64>(
        "test_topic", 1, [&data](const std_msgs::msg::Int64::SharedPtr msg) { data = msg->data; });

    // publish a message
    std_msgs::msg::Int64 msg;
    msg.data = 42;
    publisher->publish(msg);

    // spin for a bit
    interface.SpinSome();

    // check that the subscriber received the message
    EXPECT_EQ(data, msg.data);
}