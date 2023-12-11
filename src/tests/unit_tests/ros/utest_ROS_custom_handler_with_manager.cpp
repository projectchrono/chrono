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

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/ChROSHandler.h"

#include "std_msgs/msg/int64.hpp"

#include <memory>

class CustomPublisherHandler : public chrono::ros::ChROSHandler {
  public:
    CustomPublisherHandler(const std::string& topic, int64_t data) : ChROSHandler(0), m_topic(topic), m_data(data) {}

    virtual bool Initialize(std::shared_ptr<chrono::ros::ChROSInterface> interface) override {
        m_publisher = interface->GetNode()->create_publisher<std_msgs::msg::Int64>(m_topic, 1);
        return true;
    }

    virtual void Tick(double time) override {
        std_msgs::msg::Int64 msg;
        msg.data = m_data;
        m_publisher->publish(msg);
    }

    int64_t GetData() { return m_data; }

  private:
    std::string m_topic;
    int64_t m_data;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int64>> m_publisher;
};

class CustomSubscriberHandler : public chrono::ros::ChROSHandler {
  public:
    CustomSubscriberHandler(const std::string& topic) : ChROSHandler(0), m_topic(topic) {}

    virtual bool Initialize(std::shared_ptr<chrono::ros::ChROSInterface> interface) override {
        m_subscriber = interface->GetNode()->create_subscription<std_msgs::msg::Int64>(
            m_topic, 1, [this](const std_msgs::msg::Int64::SharedPtr msg) { this->m_data = msg->data; });
        return true;
    }

    virtual void Tick(double time) override {}

    int64_t GetData() { return m_data; }

  private:
    std::string m_topic;
    int64_t m_data;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int64>> m_subscriber;
};

TEST(ChROSHandler, custom_handler_with_manager) {
    // Create the ROS manager
    auto manager = chrono::ros::ChROSManager();

    // Create the custom handlers
    auto publisher_handler = std::make_shared<CustomPublisherHandler>("test_topic", 42);
    manager.RegisterHandler(publisher_handler);
    auto subscriber_handler = std::make_shared<CustomSubscriberHandler>("test_topic");
    manager.RegisterHandler(subscriber_handler);

    // Initialize the ROS manager
    manager.Initialize();

    // Update once
    manager.Update(0, 0);

    // check that the subscriber received the message
    EXPECT_EQ(publisher_handler->GetData(), subscriber_handler->GetData());
}