// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Patrick Chen
// =============================================================================
//
// End-to-end integration tests of the schema-driven Chrono::ROS bridge.
//
// This test binary deliberately links BOTH rclcpp (to act as the "real ROS"
// peer) and Chrono_ros (which contains no ROS symbols) - their coexistence in
// one process is itself part of what the new architecture guarantees. The
// bridge under test still does its ROS work in the chrono_ros_node
// subprocess, exactly as in production.
//
// Requires a sourced ROS 2 environment at runtime (std_msgs, geometry_msgs,
// sensor_msgs installed - all standard).
//
// =============================================================================

#include "gtest/gtest.h"

#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSManager.h"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int64.hpp>

#include <chrono>
#include <thread>

using namespace chrono::ros;
using namespace std::chrono_literals;

namespace {

void EnsureRclcppInit() {
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
}

/// Pump both the bridge and a test-side rclcpp node until 'done' returns true
/// or the wall-clock timeout expires. Returns the final value of done().
template <typename DonePredicate>
bool PumpUntil(ChROSManager& manager, rclcpp::Node::SharedPtr test_node, DonePredicate done, double timeout_s = 20.0) {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(test_node);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    double sim_time = 0;
    const double step = 1e-2;
    while (!done()) {
        if (std::chrono::steady_clock::now() > deadline) {
            executor.remove_node(test_node);
            return false;
        }
        EXPECT_TRUE(manager.Update(sim_time, step));
        sim_time += step;
        executor.spin_some();
        std::this_thread::sleep_for(2ms);
    }
    executor.remove_node(test_node);
    return true;
}

}  // namespace

// -----------------------------------------------------------------------------
// Chrono -> ROS: a message built by field name arrives as a typed ROS message.
TEST(ChROSBridge, publish_reaches_ros) {
    EnsureRclcppInit();

    ChROSManager manager("utest_pub_bridge");
    manager.Initialize();
    auto publisher = manager.GetBridge()->CreatePublisher("/utest/int", "std_msgs/msg/Int64");

    auto test_node = std::make_shared<rclcpp::Node>("utest_pub_peer");
    int64_t received = 0;
    int received_count = 0;
    auto subscription = test_node->create_subscription<std_msgs::msg::Int64>(
        "/utest/int", 10, [&](const std_msgs::msg::Int64::SharedPtr msg) {
            received = msg->data;
            received_count++;
        });

    const bool ok = PumpUntil(manager, test_node, [&] {
        auto message = publisher->NewMessage();
        message.SetInt("data", 42);
        publisher->Publish(message);
        return received_count > 0;
    });

    ASSERT_TRUE(ok) << "ROS peer never received the bridged message";
    EXPECT_EQ(received, 42);

    // Matched-endpoint reporting (CHANNEL_INFO) is eventually-consistent: it
    // depends on DDS discovery completing and the node's periodic poll, which
    // can lag first delivery. Pump until it reflects our peer (with timeout).
    const bool matched = PumpUntil(manager, test_node,
                                   [&] { return publisher->GetSubscriptionCount() >= 1; }, 5.0);
    EXPECT_TRUE(matched) << "publisher never reported its matched subscriber";
}

// -----------------------------------------------------------------------------
// ROS -> Chrono: a typed ROS message arrives in a field-addressed callback on
// the simulation thread.
TEST(ChROSBridge, ros_publish_reaches_bridge_callback) {
    EnsureRclcppInit();

    ChROSManager manager("utest_sub_bridge");
    manager.Initialize();

    double received_x = 0;
    int received_count = 0;
    auto subscription = manager.GetBridge()->CreateSubscription(
        "/utest/twist", "geometry_msgs/msg/Twist", [&](const ChROSMessageView& message) {
            received_x = message.GetDouble("linear.x");
            received_count++;
        });

    auto test_node = std::make_shared<rclcpp::Node>("utest_sub_peer");
    auto publisher = test_node->create_publisher<geometry_msgs::msg::Twist>("/utest/twist", 10);

    const bool ok = PumpUntil(manager, test_node, [&] {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 1.5;
        publisher->publish(msg);
        return received_count > 0;
    });

    ASSERT_TRUE(ok) << "bridge callback never fired";
    EXPECT_DOUBLE_EQ(received_x, 1.5);
    EXPECT_EQ(subscription->GetReceivedCount(), static_cast<uint64_t>(received_count));
}

// -----------------------------------------------------------------------------
// Big-sensor shape: an Image (nested header + string + bulk blob) round-trips
// pixel-exact through the schema-driven serializer and the real rmw.
TEST(ChROSBridge, image_roundtrip_pixel_exact) {
    EnsureRclcppInit();

    ChROSManager manager("utest_image_bridge");
    manager.Initialize();
    // CreatePublisher implies the VALIDATE_TYPE handshake: reaching this line
    // means the CDR encoding of sensor_msgs/msg/Image byte-matched the rmw.
    auto publisher = manager.GetBridge()->CreatePublisher("/utest/image", "sensor_msgs/msg/Image",
                                                          ChROSQoS::SensorData());

    const uint32_t width = 64, height = 48;
    std::vector<uint8_t> pixels(width * height * 4);
    for (size_t i = 0; i < pixels.size(); i++) {
        pixels[i] = static_cast<uint8_t>(i * 13);
    }

    auto test_node = std::make_shared<rclcpp::Node>("utest_image_peer");
    sensor_msgs::msg::Image received;
    int received_count = 0;
    auto subscription = test_node->create_subscription<sensor_msgs::msg::Image>(
        "/utest/image", rclcpp::SensorDataQoS(), [&](const sensor_msgs::msg::Image::SharedPtr msg) {
            received = *msg;
            received_count++;
        });

    const bool ok = PumpUntil(manager, test_node, [&] {
        auto message = publisher->NewMessage();
        message.SetTime("header.stamp", 1.25);
        message.SetString("header.frame_id", "cam");
        message.SetInt("height", height);
        message.SetInt("width", width);
        message.SetString("encoding", "rgba8");
        message.SetInt("step", width * 4);
        message.SetBlob("data", pixels.data(), pixels.size());
        publisher->Publish(message);
        return received_count > 0;
    });

    ASSERT_TRUE(ok) << "image never arrived";
    EXPECT_EQ(received.header.frame_id, "cam");
    EXPECT_EQ(received.header.stamp.sec, 1);
    EXPECT_EQ(received.header.stamp.nanosec, 250000000u);
    EXPECT_EQ(received.height, height);
    EXPECT_EQ(received.width, width);
    EXPECT_EQ(received.encoding, "rgba8");
    ASSERT_EQ(received.data.size(), pixels.size());
    EXPECT_EQ(received.data, pixels);
}

// -----------------------------------------------------------------------------
// Type discovery and fail-fast diagnostics.
TEST(ChROSBridge, describe_type_and_errors) {
    EnsureRclcppInit();

    ChROSManager manager("utest_meta_bridge");
    manager.Initialize();
    auto bridge = manager.GetBridge();

    const std::string description = bridge->DescribeType("sensor_msgs/msg/NavSatFix");
    EXPECT_NE(description.find("latitude"), std::string::npos);
    EXPECT_NE(description.find("float64[9] position_covariance"), std::string::npos);

    // Unknown package: actionable error, not a hang or a crash.
    EXPECT_THROW(bridge->CreatePublisher("/utest/nope", "definitely_not_a_pkg/msg/Nope"), std::runtime_error);

    // Bad field name on a real type: lists the valid fields.
    auto publisher = bridge->CreatePublisher("/utest/fix", "sensor_msgs/msg/NavSatFix");
    auto message = publisher->NewMessage();
    try {
        message.SetDouble("lattitude", 43.07);  // typo on purpose
        FAIL() << "expected FieldError";
    } catch (const std::exception& e) {
        EXPECT_NE(std::string(e.what()).find("latitude"), std::string::npos)
            << "error message should list available fields";
    }
}
