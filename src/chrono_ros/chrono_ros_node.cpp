// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// Standalone ROS node process for chrono_ros IPC communication
// This process runs in isolation from VSG symbols to avoid conflicts
//
// =============================================================================

#include <iostream>
#include <string>
#include <memory>
#include <unordered_map>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include "chrono_ros/ipc/ChROSIPCChannel.h"
#include "chrono_ros/ipc/ChROSIPCMessage.h"

using namespace chrono::ros::ipc;

/// ROS node that receives data via IPC and publishes to ROS topics
class ChronoROSNode {
public:
    ChronoROSNode(const std::string& node_name, const std::string& channel_name)
        : m_node_name(node_name), m_channel_name(channel_name) {
        
        // Initialize ROS
        rclcpp::init(0, nullptr);
        m_node = std::make_shared<rclcpp::Node>(node_name);
        m_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        m_executor->add_node(m_node);
        
        // Connect to IPC channel
        RCLCPP_INFO(m_node->get_logger(), "Connecting to IPC channel: %s", channel_name.c_str());
        try {
            m_channel = IPCChannel::ConnectToChannel(channel_name);
            RCLCPP_INFO(m_node->get_logger(), "IPC channel connected successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to connect to IPC channel: %s", e.what());
            throw;
        }
        
        // Initialize TF broadcaster
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(m_node);
        
        RCLCPP_INFO(m_node->get_logger(), "Chrono ROS node initialized: %s", node_name.c_str());
    }
    
    ~ChronoROSNode() {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }
    
    /// Main execution loop
    void Run() {
        Message message;
        RCLCPP_INFO(m_node->get_logger(), "Starting main execution loop");
        
        while (rclcpp::ok()) {
            // Process ROS callbacks
            m_executor->spin_some(std::chrono::milliseconds(1));
            
            // Process IPC messages
            static int debug_counter = 0;
            if (++debug_counter % 1000 == 0) {  // Every 1000 iterations
                RCLCPP_INFO(m_node->get_logger(), "Checking for IPC messages... (check #%d)", debug_counter);
            }
            
            while (m_channel->ReceiveMessage(message)) {
                RCLCPP_INFO(m_node->get_logger(), "Received IPC message type: %d", 
                           static_cast<int>(message.header.type));
                ProcessMessage(message);
                
                if (message.header.type == MessageType::SHUTDOWN) {
                    RCLCPP_INFO(m_node->get_logger(), "Received shutdown message");
                    return;
                }
            }
            
            // Small delay to prevent busy waiting
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

private:
    void ProcessMessage(const Message& message) {
        switch (message.header.type) {
            case MessageType::CLOCK_DATA:
                ProcessClockMessage(message);
                break;
                
            case MessageType::BODY_DATA:
                ProcessBodyMessage(message);
                break;
                
            default:
                RCLCPP_WARN(m_node->get_logger(), "Unknown message type: %d", 
                           static_cast<int>(message.header.type));
                break;
        }
    }
    
    void ProcessClockMessage(const Message& message) {
        const auto* clock_data = message.GetPayload<ClockData>();
        
        // Create or get clock publisher
        if (m_clock_publisher == nullptr) {
            m_clock_publisher = m_node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
            RCLCPP_INFO(m_node->get_logger(), "Created clock publisher");
        }
        
        // Publish clock message
        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock.sec = static_cast<int32_t>(clock_data->time_seconds);
        clock_msg.clock.nanosec = static_cast<uint32_t>((clock_data->time_seconds - clock_msg.clock.sec) * 1e9);
        
        RCLCPP_INFO(m_node->get_logger(), "Publishing clock: %.3fs", clock_data->time_seconds);
        
        m_clock_publisher->publish(clock_msg);
        RCLCPP_INFO(m_node->get_logger(), "Clock message published successfully");
    }
    
    void ProcessBodyMessage(const Message& message) {
        const auto* body_data = message.GetPayload<BodyData>();
        
        RCLCPP_INFO(m_node->get_logger(), "Processing body data for: %s", body_data->body_name);
        
        // Create publishers for this body (using a map for dynamic creation)
        std::string body_key = std::string(body_data->body_name);
        
        // Create pose publisher
        auto pose_topic = std::string(body_data->topic_prefix) + "pose";
        if (m_body_pose_publishers.find(body_key) == m_body_pose_publishers.end()) {
            m_body_pose_publishers[body_key] = 
                m_node->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);
            RCLCPP_INFO(m_node->get_logger(), "Created body pose publisher: %s", pose_topic.c_str());
        }
        
        // Create twist publisher  
        auto twist_topic = std::string(body_data->topic_prefix) + "twist";
        if (m_body_twist_publishers.find(body_key) == m_body_twist_publishers.end()) {
            m_body_twist_publishers[body_key] = 
                m_node->create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic, 10);
            RCLCPP_INFO(m_node->get_logger(), "Created body twist publisher: %s", twist_topic.c_str());
        }
        
        // Create accel publisher
        auto accel_topic = std::string(body_data->topic_prefix) + "accel"; 
        if (m_body_accel_publishers.find(body_key) == m_body_accel_publishers.end()) {
            m_body_accel_publishers[body_key] = 
                m_node->create_publisher<geometry_msgs::msg::AccelStamped>(accel_topic, 10);
            RCLCPP_INFO(m_node->get_logger(), "Created body accel publisher: %s", accel_topic.c_str());
        }
        
        // Create and publish pose message
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = m_node->get_clock()->now();
        pose_msg.header.frame_id = "world";
        pose_msg.pose.position.x = body_data->pos_x;
        pose_msg.pose.position.y = body_data->pos_y;
        pose_msg.pose.position.z = body_data->pos_z;
        pose_msg.pose.orientation.w = body_data->rot_w;
        pose_msg.pose.orientation.x = body_data->rot_x;
        pose_msg.pose.orientation.y = body_data->rot_y;
        pose_msg.pose.orientation.z = body_data->rot_z;
        m_body_pose_publishers[body_key]->publish(pose_msg);
        
        // Create and publish twist message
        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.stamp = m_node->get_clock()->now();
        twist_msg.header.frame_id = "world";
        twist_msg.twist.linear.x = body_data->lin_vel_x;
        twist_msg.twist.linear.y = body_data->lin_vel_y;
        twist_msg.twist.linear.z = body_data->lin_vel_z;
        twist_msg.twist.angular.x = body_data->ang_vel_x;
        twist_msg.twist.angular.y = body_data->ang_vel_y;
        twist_msg.twist.angular.z = body_data->ang_vel_z;
        m_body_twist_publishers[body_key]->publish(twist_msg);
        
        // Create and publish accel message
        geometry_msgs::msg::AccelStamped accel_msg;
        accel_msg.header.stamp = m_node->get_clock()->now(); 
        accel_msg.header.frame_id = "world";
        accel_msg.accel.linear.x = body_data->lin_acc_x;
        accel_msg.accel.linear.y = body_data->lin_acc_y;
        accel_msg.accel.linear.z = body_data->lin_acc_z;
        accel_msg.accel.angular.x = body_data->ang_acc_x;
        accel_msg.accel.angular.y = body_data->ang_acc_y;
        accel_msg.accel.angular.z = body_data->ang_acc_z;
        m_body_accel_publishers[body_key]->publish(accel_msg);
        
        RCLCPP_INFO(m_node->get_logger(), "Published body data for: %s", body_data->body_name);
    }

private:
    std::string m_node_name;
    std::string m_channel_name;
    
    std::shared_ptr<rclcpp::Node> m_node;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;
    std::unique_ptr<IPCChannel> m_channel;
    
    // Publishers
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr m_clock_publisher;
    
    // Body publishers (dynamic creation based on body names)
    std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> m_body_pose_publishers;
    std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr> m_body_twist_publishers;
    std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr> m_body_accel_publishers;
    
    // TF broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
};

int main(int argc, char* argv[]) {
    std::cout << "=== SUBPROCESS STARTING ===" << std::endl;
    std::cout << "Arguments received: " << argc << std::endl;
    for (int i = 0; i < argc; ++i) {
        std::cout << "  argv[" << i << "] = " << argv[i] << std::endl;
    }
    
    std::string node_name = "chrono_ros_node";
    std::string channel_name = "chrono_ros_ipc";
    
    // Parse command line arguments
    for (int i = 1; i < argc - 1; ++i) {
        std::string arg = argv[i];
        if (arg == "--node-name") {
            node_name = argv[++i];
        } else if (arg == "--channel-name") {
            channel_name = argv[++i];
        }
    }
    
    std::cout << "Parsed node_name: " << node_name << std::endl;
    std::cout << "Parsed channel_name: " << channel_name << std::endl;
    
    try {
        std::cout << "Creating ChronoROSNode..." << std::endl;
        ChronoROSNode node(node_name, channel_name);
        std::cout << "Starting node.Run()..." << std::endl;
        node.Run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}