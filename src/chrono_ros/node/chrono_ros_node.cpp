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
// The chrono_ros bridge node: a fixed, message-agnostic subprocess.
//
// It owns the ROS side of the bridge and is intentionally dumb: it answers
// type-schema queries (DescribeType/ValidateType), creates generic
// publishers/subscriptions on Advertise, and pumps opaque serialized message
// bytes between the IPC channel and the ROS graph. It contains no
// per-message-type code and no Chrono code, and never needs to be modified
// to support new topics or message packages (CLAUDE.md sections 3-5).
//
// =============================================================================

#include "chrono_ros/core/ChROSControl.h"
#include "chrono_ros/core/ChROSFrame.h"
#include "chrono_ros/core/transport/ChROSChannel.h"
#include "chrono_ros/node/ChROSNodeTypeSupport.h"

#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#ifndef _WIN32
    #include <signal.h>
    #include <sys/prctl.h>  // PR_SET_PDEATHSIG
    #include <unistd.h>
#else
    #include <process.h>
#endif

using namespace chrono::ros;

namespace {

core::QoSSpec ToQoSSpec(const core::AdvertisePayload& payload) {
    return payload.qos;
}

rclcpp::QoS ToRclcppQoS(const core::QoSSpec& spec) {
    rclcpp::QoS qos(spec.depth);
    if (spec.reliability == core::Reliability::BestEffort) {
        qos.best_effort();
    } else {
        qos.reliable();
    }
    if (spec.durability == core::Durability::TransientLocal) {
        qos.transient_local();
    } else {
        qos.durability_volatile();
    }
    return qos;
}

class BridgeNode {
  public:
    BridgeNode(const std::string& node_name, const std::string& channel_name) {
#ifndef _WIN32
        // Die with the simulation process so no orphan ROS nodes accumulate.
        prctl(PR_SET_PDEATHSIG, SIGTERM);
#endif
        m_node = std::make_shared<rclcpp::Node>(node_name);
        m_executor.add_node(m_node);

        RCLCPP_INFO(m_node->get_logger(), "attaching to IPC channel '%s'", channel_name.c_str());
        m_channel = core::Channel::Open(channel_name);

        SendHello(node_name);
        RCLCPP_INFO(m_node->get_logger(), "bridge node ready");
    }

    void Run() {
        std::vector<uint8_t> payload;
        core::FrameHeader header;
        auto last_endpoint_poll = std::chrono::steady_clock::now();

        while (rclcpp::ok()) {
            // Drain the IPC channel first so bulk sensor frames do not pile up
            // behind ROS callback work (bounded per cycle to keep callbacks
            // responsive under sustained load).
            int drained = 0;
            while (drained < 128 && m_channel->ReceiveFrame(header, payload)) {
                drained++;
                if (!Dispatch(header, payload)) {
                    return;  // Shutdown frame
                }
            }

            m_executor.spin_some(std::chrono::microseconds(200));

            const auto now = std::chrono::steady_clock::now();
            if (now - last_endpoint_poll > std::chrono::milliseconds(200)) {
                last_endpoint_poll = now;
                PollMatchedEndpoints();
            }

            if (drained == 0) {
                // Idle: yield briefly instead of busy-spinning.
                std::this_thread::sleep_for(std::chrono::microseconds(50));
            }
        }
    }

  private:
    void SendHello(const std::string& node_name) {
        core::HelloPayload hello;
        hello.protocol_version = core::kProtocolVersion;
#ifndef _WIN32
        hello.pid = static_cast<uint32_t>(getpid());
#else
        hello.pid = static_cast<uint32_t>(_getpid());
#endif
        const char* distro = std::getenv("ROS_DISTRO");
        hello.ros_distro = distro ? distro : "unknown";
        hello.rmw_id = rmw_get_implementation_identifier();
        hello.node_name = node_name;

        std::vector<uint8_t> buffer;
        hello.Encode(buffer);
        m_channel->SendFrame(core::FrameKind::Hello, 0, 0, buffer.data(), buffer.size());
    }

    /// Returns false only on a Shutdown frame.
    bool Dispatch(const core::FrameHeader& header, const std::vector<uint8_t>& payload) {
        switch (static_cast<core::FrameKind>(header.kind)) {
            case core::FrameKind::DescribeType:
                OnDescribeType(payload);
                return true;
            case core::FrameKind::ValidateType:
                OnValidateType(payload);
                return true;
            case core::FrameKind::Advertise:
                OnAdvertise(header.channel_id, payload);
                return true;
            case core::FrameKind::Publish:
                OnPublish(header.channel_id, payload);
                return true;
            case core::FrameKind::Unadvertise:
                m_publishers.erase(header.channel_id);
                m_subscriptions.erase(header.channel_id);
                m_matched_counts.erase(header.channel_id);
                return true;
            case core::FrameKind::Shutdown:
                RCLCPP_INFO(m_node->get_logger(), "shutdown requested by simulation");
                return false;
            default:
                RCLCPP_WARN(m_node->get_logger(), "unexpected frame kind %u from simulation",
                            static_cast<unsigned>(header.kind));
                return true;
        }
    }

    void OnDescribeType(const std::vector<uint8_t>& payload) {
        const auto request = core::DescribeTypePayload::Decode(payload.data(), payload.size());

        core::TypeSchemaPayload reply;
        reply.type_name = request.type_name;
        try {
            const core::Schema schema = node::BuildSchema(request.type_name);
            reply.schema_blob = schema.EncodeBlob();
            reply.ok = true;
        } catch (const std::exception& e) {
            reply.ok = false;
            reply.error = e.what();
            RCLCPP_ERROR(m_node->get_logger(), "DescribeType('%s') failed: %s", request.type_name.c_str(), e.what());
        }

        std::vector<uint8_t> buffer;
        reply.Encode(buffer);
        m_channel->SendFrame(core::FrameKind::TypeSchema, 0, 0, buffer.data(), buffer.size());
    }

    void OnValidateType(const std::vector<uint8_t>& payload) {
        const auto request = core::ValidateTypePayload::Decode(payload.data(), payload.size());
        const core::ValidateResultPayload result = node::ValidateSample(request.type_name, request.sample_cdr);
        if (!result.ok) {
            RCLCPP_ERROR(m_node->get_logger(), "ValidateType('%s') failed: %s", request.type_name.c_str(),
                         result.error.c_str());
        }
        std::vector<uint8_t> buffer;
        result.Encode(buffer);
        m_channel->SendFrame(core::FrameKind::ValidateResult, 0, 0, buffer.data(), buffer.size());
    }

    void OnAdvertise(uint32_t channel_id, const std::vector<uint8_t>& payload) {
        const auto request = core::AdvertisePayload::Decode(payload.data(), payload.size());

        core::AdvertiseAckPayload ack;
        try {
            const rclcpp::QoS qos = ToRclcppQoS(ToQoSSpec(request));
            if (request.direction == core::AdvertisePayload::Direction::Publish) {
                auto publisher = m_node->create_generic_publisher(request.topic, request.type_name, qos);
                m_publishers[channel_id] = publisher;
                RCLCPP_INFO(m_node->get_logger(), "publishing '%s' [%s] (channel %u)", request.topic.c_str(),
                            request.type_name.c_str(), channel_id);
            } else {
                auto subscription = m_node->create_generic_subscription(
                    request.topic, request.type_name, qos,
                    [this, channel_id](std::shared_ptr<rclcpp::SerializedMessage> message) {
                        OnRosMessage(channel_id, *message);
                    });
                m_subscriptions[channel_id] = subscription;
                RCLCPP_INFO(m_node->get_logger(), "subscribed to '%s' [%s] (channel %u)", request.topic.c_str(),
                            request.type_name.c_str(), channel_id);
            }
            ack.ok = true;
        } catch (const std::exception& e) {
            ack.ok = false;
            ack.error = e.what();
            RCLCPP_ERROR(m_node->get_logger(), "Advertise('%s' [%s]) failed: %s", request.topic.c_str(),
                         request.type_name.c_str(), e.what());
        }

        std::vector<uint8_t> buffer;
        ack.Encode(buffer);
        m_channel->SendFrame(core::FrameKind::AdvertiseAck, channel_id, 0, buffer.data(), buffer.size());
    }

    void OnPublish(uint32_t channel_id, const std::vector<uint8_t>& payload) {
        auto it = m_publishers.find(channel_id);
        if (it == m_publishers.end()) {
            RCLCPP_WARN_THROTTLE(m_node->get_logger(), *m_node->get_clock(), 5000,
                                 "Publish frame for unknown channel %u", channel_id);
            return;
        }
        // Hand the bytes to rmw verbatim; this node never parses message content.
        m_scratch.reserve(payload.size());
        auto& rcl_message = m_scratch.get_rcl_serialized_message();
        std::memcpy(rcl_message.buffer, payload.data(), payload.size());
        rcl_message.buffer_length = payload.size();
        it->second->publish(m_scratch);
    }

    void OnRosMessage(uint32_t channel_id, const rclcpp::SerializedMessage& message) {
        const auto& rcl_message = message.get_rcl_serialized_message();
        if (!m_channel->SendFrame(core::FrameKind::Received, channel_id, 0, rcl_message.buffer,
                                  rcl_message.buffer_length)) {
            // Return ring momentarily full: drop and account. Command-style
            // topics are low-rate, so this indicates a stalled simulation.
            m_dropped_inbound++;
            RCLCPP_WARN_THROTTLE(m_node->get_logger(), *m_node->get_clock(), 5000,
                                 "IPC return path full; dropped %lu inbound message(s)",
                                 static_cast<unsigned long>(m_dropped_inbound));
        }
    }

    void PollMatchedEndpoints() {
        for (const auto& [channel_id, publisher] : m_publishers) {
            ReportMatched(channel_id, publisher->get_subscription_count());
        }
        for (const auto& [channel_id, subscription] : m_subscriptions) {
            ReportMatched(channel_id, subscription->get_publisher_count());
        }
    }

    void ReportMatched(uint32_t channel_id, size_t count) {
        auto it = m_matched_counts.find(channel_id);
        if (it != m_matched_counts.end() && it->second == count) {
            return;
        }
        m_matched_counts[channel_id] = count;
        core::ChannelInfoPayload info;
        info.matched_endpoints = static_cast<uint32_t>(count);
        std::vector<uint8_t> buffer;
        info.Encode(buffer);
        m_channel->SendFrame(core::FrameKind::ChannelInfo, channel_id, 0, buffer.data(), buffer.size());
    }

    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::executors::SingleThreadedExecutor m_executor;
    std::unique_ptr<core::Channel> m_channel;

    std::unordered_map<uint32_t, std::shared_ptr<rclcpp::GenericPublisher>> m_publishers;
    std::unordered_map<uint32_t, std::shared_ptr<rclcpp::GenericSubscription>> m_subscriptions;
    std::unordered_map<uint32_t, size_t> m_matched_counts;

    rclcpp::SerializedMessage m_scratch;
    uint64_t m_dropped_inbound = 0;
};

}  // namespace

int main(int argc, char* argv[]) {
    std::string node_name = "chrono_ros_node";
    std::string channel_name;

    for (int i = 1; i + 1 < argc; i++) {
        const std::string arg = argv[i];
        if (arg == "--node-name") {
            node_name = argv[++i];
        } else if (arg == "--channel-name") {
            channel_name = argv[++i];
        }
    }
    if (channel_name.empty()) {
        std::cerr << "chrono_ros_node is the internal bridge subprocess of Chrono::ROS and is not meant to be\n"
                     "launched manually. Required arguments: --channel-name <shm name> [--node-name <name>]\n";
        return 2;
    }

    rclcpp::init(argc, argv);
    int status = 0;
    try {
        BridgeNode bridge(node_name, channel_name);
        bridge.Run();
    } catch (const std::exception& e) {
        std::cerr << "[chrono_ros_node] fatal: " << e.what() << std::endl;
        status = 1;
    }
    rclcpp::shutdown();
    return status;
}
