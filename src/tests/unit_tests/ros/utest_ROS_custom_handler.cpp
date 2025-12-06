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

    virtual std::vector<uint8_t> GetSerializedData(double time) override {
        std::vector<uint8_t> data(sizeof(int64_t));
        std::memcpy(data.data(), &m_data, sizeof(int64_t));
        return data;
    }

    virtual void PublishFromSerialized(const std::vector<uint8_t>& data, 
                                       std::shared_ptr<chrono::ros::ChROSInterface> interface) override {
        if (data.size() != sizeof(int64_t)) return;
        int64_t val;
        std::memcpy(&val, data.data(), sizeof(int64_t));
        
        std_msgs::msg::Int64 msg;
        msg.data = val;
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
    
    virtual bool IsPublisher() const override { return false; }

    int64_t GetData() { return m_data; }

  private:
    std::string m_topic;
    int64_t m_data;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int64>> m_subscriber;
};

TEST(ChROSHandler, custom_handler) {
    // Create the ROS interface
    auto interface = std::make_shared<chrono::ros::ChROSInterface>("chrono_ros_node");

    // Create the custom handlers
    auto publisher_handler = CustomPublisherHandler("test_topic", 42);
    auto subscriber_handler = CustomSubscriberHandler("test_topic");

    // Initialize the ROS interface and handlers
    interface->Initialize();
    publisher_handler.Initialize(interface);
    subscriber_handler.Initialize(interface);

    // publish a message
    publisher_handler.Tick(0);

    // spin for a bit
    interface->SpinSome();

    // check that the subscriber received the message
    EXPECT_EQ(publisher_handler.GetData(), subscriber_handler.GetData());
}