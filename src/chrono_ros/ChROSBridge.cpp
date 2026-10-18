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

#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSNodeProcess.h"
#include "chrono_ros/core/ChROSCdr.h"
#include "chrono_ros/core/ChROSControl.h"
#include "chrono_ros/core/transport/ChROSChannel.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>
#include <thread>

#ifndef _WIN32
    #include <unistd.h>
#else
    #include <process.h>
#endif

namespace chrono {
namespace ros {

namespace {

constexpr double kStartupTimeout = 30.0;  ///< node launch + rclcpp::init (slow on cold DDS discovery)
constexpr double kRequestTimeout = 15.0;  ///< per control-plane round trip (includes typesupport dlopen)

uint32_t CurrentPid() {
#ifndef _WIN32
    return static_cast<uint32_t>(getpid());
#else
    return static_cast<uint32_t>(_getpid());
#endif
}

bool SchemaContainsWString(const core::Schema& schema) {
    for (size_t t = 0; t < schema.TypeCount(); t++) {
        for (const auto& field : schema.At(static_cast<int32_t>(t)).fields) {
            if (field.kind == core::FieldKind::WString) {
                return true;
            }
        }
    }
    return false;
}

/// Fill every field of a message deterministically, exercising each
/// alignment/sequence path of the sim-side serializer. Used to build the
/// VALIDATE_TYPE sample.
void FillSampleMessage(core::MessageBuilder& builder, const core::Schema& schema, int32_t type_index, uint32_t& seed) {
    for (const auto& field : schema.At(type_index).fields) {
        seed++;
        const uint32_t small = seed % 100;  // fits every integer width

        switch (field.array_kind) {
            case core::ArrayKind::None: {
                switch (field.kind) {
                    case core::FieldKind::Bool:
                        builder.SetBool(field.name, (seed & 1) != 0);
                        break;
                    case core::FieldKind::String:
                        builder.SetString(field.name, "chrono" + std::to_string(seed));
                        break;
                    case core::FieldKind::Message: {
                        core::MessageBuilder& nested = builder.GetNested(field.name);
                        FillSampleMessage(nested, schema, field.nested_type, seed);
                        break;
                    }
                    case core::FieldKind::Float32:
                    case core::FieldKind::Float64:
                        builder.SetDouble(field.name, static_cast<double>(small) + 0.5);
                        break;
                    case core::FieldKind::Int8:
                    case core::FieldKind::Int16:
                    case core::FieldKind::Int32:
                    case core::FieldKind::Int64:
                        builder.SetInt(field.name, -static_cast<int64_t>(small));
                        break;
                    default:  // unsigned integers, char, octet
                        builder.SetInt(field.name, small);
                        break;
                }
                break;
            }
            case core::ArrayKind::FixedArray:
            case core::ArrayKind::BoundedSequence:
            case core::ArrayKind::UnboundedSequence: {
                size_t count = 3;
                if (field.array_kind == core::ArrayKind::FixedArray) {
                    count = field.array_size;
                } else if (field.array_kind == core::ArrayKind::BoundedSequence) {
                    count = std::min<size_t>(count, field.array_size);
                }

                if (field.kind == core::FieldKind::Message) {
                    for (size_t i = 0; i < count; i++) {
                        core::MessageBuilder& element = builder.AppendMessage(field.name);
                        FillSampleMessage(element, schema, field.nested_type, seed);
                    }
                } else if (field.kind == core::FieldKind::String) {
                    std::vector<std::string> strings;
                    for (size_t i = 0; i < count; i++) {
                        strings.push_back("s" + std::to_string(seed + i));
                    }
                    builder.SetStringArray(field.name, strings);
                } else {
                    const size_t element_size = core::PrimitiveSize(field.kind);
                    std::vector<uint8_t> bytes(count * element_size);
                    for (size_t i = 0; i < bytes.size(); i++) {
                        // Bool elements must be strictly 0/1 to round-trip.
                        bytes[i] = field.kind == core::FieldKind::Bool ? static_cast<uint8_t>((seed + i) & 1)
                                                                       : static_cast<uint8_t>(seed + i);
                    }
                    builder.SetBlobCopy(field.name, bytes.data(), count);
                }
                break;
            }
        }
    }
}

std::string FormatValidationFailure(const core::ValidateResultPayload& result) {
    std::ostringstream out;
    out << "wire-format validation failed for '" << result.type_name << "': " << result.error;
    if (!result.expected_window.empty() || !result.actual_window.empty()) {
        out << "\n  rmw bytes around offset " << result.first_diff_offset << ":";
        for (uint8_t b : result.expected_window) {
            char hex[8];
            std::snprintf(hex, sizeof(hex), " %02X", b);
            out << hex;
        }
        out << "\n  sim bytes around offset " << result.first_diff_offset << ":";
        for (uint8_t b : result.actual_window) {
            char hex[8];
            std::snprintf(hex, sizeof(hex), " %02X", b);
            out << hex;
        }
    }
    out << "\nThis indicates a CDR encoding discrepancy between Chrono::ROS and the rmw in use; please report it "
           "with the message type and ROS distro.";
    return out.str();
}

}  // namespace

// ----------------------------------------------------------------------------
// lifecycle
// ----------------------------------------------------------------------------

ChROSBridge::ChROSBridge(const std::string& node_name)
    : m_node_name(node_name),
      m_capacity_sim_to_node(core::Channel::Config().sim_to_node_capacity),
      m_capacity_node_to_sim(core::Channel::Config().node_to_sim_capacity),
      m_process(new ChROSNodeProcess()) {}

ChROSBridge::~ChROSBridge() {
    Shutdown();
}

void ChROSBridge::SetChannelCapacity(size_t sim_to_node_bytes, size_t node_to_sim_bytes) {
    if (m_initialized) {
        throw std::runtime_error("Chrono::ROS: SetChannelCapacity must be called before Initialize()");
    }
    m_capacity_sim_to_node = sim_to_node_bytes;
    m_capacity_node_to_sim = node_to_sim_bytes;
}

void ChROSBridge::Initialize() {
    if (m_initialized) {
        return;
    }
    if (!core::IsHostLittleEndian()) {
        throw std::runtime_error("Chrono::ROS supports little-endian hosts only");
    }

    static uint32_t s_instance_counter = 0;
    const std::string channel_name =
        "chrono_ros_" + std::to_string(CurrentPid()) + "_" + std::to_string(s_instance_counter++);

    core::Channel::Config config;
    config.sim_to_node_capacity = m_capacity_sim_to_node;
    config.node_to_sim_capacity = m_capacity_node_to_sim;
    m_channel = core::Channel::Create(channel_name, config);

    m_process->Launch(m_node_name, channel_name);
    m_initialized = true;  // AwaitFrame requires it; rolled back on failure below

    try {
        std::vector<uint8_t> payload;
        AwaitFrame(static_cast<uint16_t>(core::FrameKind::Hello), 0, payload, kStartupTimeout,
                   "bridge node startup handshake");
        const auto hello = core::HelloPayload::Decode(payload.data(), payload.size());
        if (hello.protocol_version != core::kProtocolVersion) {
            throw std::runtime_error("protocol version mismatch: chrono_ros_node speaks v" +
                                     std::to_string(hello.protocol_version) + ", this library v" +
                                     std::to_string(core::kProtocolVersion) +
                                     " (simulation and node must come from the same build)");
        }
        std::cout << "[Chrono::ROS] bridge connected: node '" << hello.node_name << "' (pid " << hello.pid
                  << "), ROS distro '" << hello.ros_distro << "', rmw '" << hello.rmw_id << "'" << std::endl;
    } catch (...) {
        m_initialized = false;
        m_process->Terminate();
        m_channel.reset();
        throw;
    }
}

bool ChROSBridge::IsNodeAlive() const {
    return m_initialized && m_process->IsRunning();
}

void ChROSBridge::Shutdown() {
    if (!m_initialized) {
        return;
    }
    m_initialized = false;
    if (m_channel) {
        try {
            m_channel->SendFrame(core::FrameKind::Shutdown, 0, m_sim_time_ns);
        } catch (const std::exception&) {
            // Best effort; the process is terminated below regardless.
        }
    }
    m_process->Terminate();
    m_channel.reset();
    m_subscriptions.clear();
    m_matched_counts.clear();
    if (m_dropped_outbound > 0) {
        std::cerr << "[Chrono::ROS] note: " << m_dropped_outbound
                  << " outbound message(s) were dropped during the run because the IPC channel was full" << std::endl;
    }
}

void ChROSBridge::RequireInitialized(const char* what) const {
    if (!m_initialized) {
        throw std::runtime_error(std::string("Chrono::ROS: ") + what +
                                 " requires an initialized bridge (call ChROSManager::Initialize() first)");
    }
}

// ----------------------------------------------------------------------------
// control plane
// ----------------------------------------------------------------------------

core::FrameHeader ChROSBridge::AwaitFrame(uint16_t kind,
                                          uint32_t channel_id,
                                          std::vector<uint8_t>& payload,
                                          double timeout_seconds,
                                          const char* what) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_seconds);
    core::FrameHeader header;

    while (true) {
        while (m_channel->ReceiveFrame(header, payload)) {
            if (header.kind == kind && (channel_id == 0 || header.channel_id == channel_id)) {
                return header;
            }
            DispatchFrame(header, payload);
        }
        if (!m_process->IsRunning()) {
            throw std::runtime_error(std::string("Chrono::ROS: chrono_ros_node exited during ") + what +
                                     ". Common causes: the ROS 2 environment is not sourced in this shell "
                                     "('source /opt/ros/<distro>/setup.bash' plus any workspace overlays), or the "
                                     "executable '" +
                                     m_process->GetExecutablePath() + "' is missing its libraries.");
        }
        if (std::chrono::steady_clock::now() > deadline) {
            throw std::runtime_error(std::string("Chrono::ROS: timed out waiting for ") + what);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
}

void ChROSBridge::DispatchFrame(const core::FrameHeader& header, std::vector<uint8_t>& payload) {
    switch (static_cast<core::FrameKind>(header.kind)) {
        case core::FrameKind::Received: {
            auto it = m_subscriptions.find(header.channel_id);
            if (it == m_subscriptions.end()) {
                return;  // unadvertised meanwhile; benign race
            }
            auto subscription = it->second.lock();
            if (!subscription) {
                m_subscriptions.erase(it);
                return;
            }
            auto data = std::make_shared<const std::vector<uint8_t>>(std::move(payload));
            payload = std::vector<uint8_t>();  // moved-from; restore a valid buffer
            const ChROSMessageView view(subscription->m_schema, data);
            subscription->m_received_count++;
            subscription->m_callback->OnMessage(view);
            return;
        }
        case core::FrameKind::ChannelInfo: {
            const auto info = core::ChannelInfoPayload::Decode(payload.data(), payload.size());
            m_matched_counts[header.channel_id] = info.matched_endpoints;
            return;
        }
        default:
            std::cerr << "[Chrono::ROS] unexpected frame '" << core::FrameKindName(static_cast<core::FrameKind>(header.kind))
                      << "' from bridge node (ignored)" << std::endl;
            return;
    }
}

void ChROSBridge::ProcessIncoming() {
    RequireInitialized("ProcessIncoming");
    core::FrameHeader header;
    // Bounded drain so a message flood cannot starve the simulation step.
    for (int i = 0; i < 1024 && m_channel->ReceiveFrame(header, m_receive_scratch); i++) {
        DispatchFrame(header, m_receive_scratch);
    }
}

void ChROSBridge::SetSimTime(double time_seconds) {
    m_sim_time_ns = time_seconds <= 0 ? 0 : static_cast<uint64_t>(time_seconds * 1e9);
}

// ----------------------------------------------------------------------------
// schemas and validation
// ----------------------------------------------------------------------------

std::shared_ptr<const core::Schema> ChROSBridge::GetOrFetchSchema(const std::string& type_name) {
    RequireInitialized("type lookup");
    auto it = m_schemas.find(type_name);
    if (it != m_schemas.end()) {
        return it->second;
    }

    core::DescribeTypePayload request;
    request.type_name = type_name;
    std::vector<uint8_t> buffer;
    request.Encode(buffer);
    if (!m_channel->SendFrame(core::FrameKind::DescribeType, 0, m_sim_time_ns, buffer.data(), buffer.size())) {
        throw std::runtime_error("Chrono::ROS: IPC channel full while requesting the schema of '" + type_name + "'");
    }

    std::vector<uint8_t> payload;
    AwaitFrame(static_cast<uint16_t>(core::FrameKind::TypeSchema), 0, payload, kRequestTimeout,
               ("schema of '" + type_name + "'").c_str());
    const auto reply = core::TypeSchemaPayload::Decode(payload.data(), payload.size());
    if (!reply.ok) {
        throw std::runtime_error("Chrono::ROS: " + reply.error);
    }

    auto schema = core::Schema::DecodeBlob(reply.schema_blob.data(), reply.schema_blob.size());
    if (SchemaContainsWString(*schema)) {
        throw std::runtime_error("Chrono::ROS: message type '" + type_name +
                                 "' contains a wstring field, which is not supported (no common message uses "
                                 "wstring; choose a different type)");
    }
    m_schemas.emplace(type_name, schema);
    return schema;
}

void ChROSBridge::ValidateTypeOnce(const std::string& type_name,
                                   const std::shared_ptr<const core::Schema>& schema) {
    if (m_validated_types.count(type_name) != 0) {
        return;
    }

    core::MessageBuilder sample(schema);
    uint32_t seed = 0;
    FillSampleMessage(sample, *schema, schema->RootIndex(), seed);

    core::ValidateTypePayload request;
    request.type_name = type_name;
    sample.SerializeTo(request.sample_cdr);

    std::vector<uint8_t> buffer;
    request.Encode(buffer);
    if (!m_channel->SendFrame(core::FrameKind::ValidateType, 0, m_sim_time_ns, buffer.data(), buffer.size())) {
        throw std::runtime_error("Chrono::ROS: IPC channel full while validating '" + type_name + "'");
    }

    std::vector<uint8_t> payload;
    AwaitFrame(static_cast<uint16_t>(core::FrameKind::ValidateResult), 0, payload, kRequestTimeout,
               ("wire-format validation of '" + type_name + "'").c_str());
    const auto result = core::ValidateResultPayload::Decode(payload.data(), payload.size());
    if (!result.ok) {
        throw std::runtime_error("Chrono::ROS: " + FormatValidationFailure(result));
    }
    m_validated_types.insert(type_name);
}

std::string ChROSBridge::DescribeType(const std::string& type_name) {
    return GetOrFetchSchema(type_name)->ToString();
}

// ----------------------------------------------------------------------------
// topic API
// ----------------------------------------------------------------------------

uint32_t ChROSBridge::Advertise(const std::string& topic,
                                const std::string& type_name,
                                const ChROSQoS& qos,
                                bool subscribe) {
    const uint32_t channel_id = m_next_channel_id++;

    core::AdvertisePayload request;
    request.direction =
        subscribe ? core::AdvertisePayload::Direction::Subscribe : core::AdvertisePayload::Direction::Publish;
    request.topic = topic;
    request.type_name = type_name;
    request.qos = qos;

    std::vector<uint8_t> buffer;
    request.Encode(buffer);
    if (!m_channel->SendFrame(core::FrameKind::Advertise, channel_id, m_sim_time_ns, buffer.data(), buffer.size())) {
        throw std::runtime_error("Chrono::ROS: IPC channel full while advertising '" + topic + "'");
    }

    std::vector<uint8_t> payload;
    AwaitFrame(static_cast<uint16_t>(core::FrameKind::AdvertiseAck), channel_id, payload, kRequestTimeout,
               ("advertisement of '" + topic + "'").c_str());
    const auto ack = core::AdvertiseAckPayload::Decode(payload.data(), payload.size());
    if (!ack.ok) {
        throw std::runtime_error("Chrono::ROS: failed to advertise '" + topic + "' [" + type_name +
                                 "]: " + ack.error);
    }
    return channel_id;
}

std::shared_ptr<ChROSPublisher> ChROSBridge::CreatePublisher(const std::string& topic,
                                                             const std::string& type_name,
                                                             const ChROSQoS& qos) {
    RequireInitialized("CreatePublisher");
    auto schema = GetOrFetchSchema(type_name);
    ValidateTypeOnce(type_name, schema);
    const uint32_t channel_id = Advertise(topic, type_name, qos, /*subscribe=*/false);
    return std::shared_ptr<ChROSPublisher>(
        new ChROSPublisher(shared_from_this(), channel_id, schema, topic, type_name));
}

std::shared_ptr<ChROSSubscription> ChROSBridge::CreateSubscription(
    const std::string& topic,
    const std::string& type_name,
    std::shared_ptr<ChROSSubscriptionCallback> callback,
    const ChROSQoS& qos) {
    RequireInitialized("CreateSubscription");
    if (!callback) {
        throw std::runtime_error("Chrono::ROS: CreateSubscription requires a callback");
    }
    auto schema = GetOrFetchSchema(type_name);
    // Validation also covers subscriptions: the same schema decodes inbound CDR.
    ValidateTypeOnce(type_name, schema);
    const uint32_t channel_id = Advertise(topic, type_name, qos, /*subscribe=*/true);
    auto subscription = std::shared_ptr<ChROSSubscription>(
        new ChROSSubscription(shared_from_this(), channel_id, schema, topic, type_name, std::move(callback)));
    m_subscriptions[channel_id] = subscription;
    return subscription;
}

namespace {
class FunctionCallback : public ChROSSubscriptionCallback {
  public:
    explicit FunctionCallback(std::function<void(const ChROSMessageView&)> fn) : m_fn(std::move(fn)) {}
    void OnMessage(const ChROSMessageView& message) override { m_fn(message); }

  private:
    std::function<void(const ChROSMessageView&)> m_fn;
};
}  // namespace

std::shared_ptr<ChROSSubscription> ChROSBridge::CreateSubscription(
    const std::string& topic,
    const std::string& type_name,
    std::function<void(const ChROSMessageView&)> callback,
    const ChROSQoS& qos) {
    return CreateSubscription(topic, type_name, std::make_shared<FunctionCallback>(std::move(callback)), qos);
}

// ----------------------------------------------------------------------------
// handle backends
// ----------------------------------------------------------------------------

bool ChROSBridge::PublishSerialized(uint32_t channel_id,
                                    const ChROSMessage& message,
                                    const std::shared_ptr<const core::Schema>& expected_schema) {
    RequireInitialized("Publish");
    if (&message.m_root->GetSchema() != expected_schema.get()) {
        throw core::FieldError("message published on the wrong publisher: it was created for type '" +
                               message.m_root->GetSchema().Root().name + "', this publisher sends '" +
                               expected_schema->Root().name + "'");
    }

    m_serialize_scratch.clear();
    message.m_root->SerializeTo(m_serialize_scratch);

    const bool sent = m_channel->SendFrame(core::FrameKind::Publish, channel_id, m_sim_time_ns,
                                           m_serialize_scratch.data(), m_serialize_scratch.size());
    if (!sent) {
        m_dropped_outbound++;
        // Log on a decade scale to avoid flooding.
        if (m_dropped_outbound == 1 || m_dropped_outbound == 100 || m_dropped_outbound % 10000 == 0) {
            std::cerr << "[Chrono::ROS] outbound IPC channel full; dropped " << m_dropped_outbound
                      << " message(s) so far. If this persists, the bridge node is falling behind or the channel "
                         "capacity is too small (ChROSBridge::SetChannelCapacity)."
                      << std::endl;
        }
    }
    return sent;
}

void ChROSBridge::Unadvertise(uint32_t channel_id) {
    m_subscriptions.erase(channel_id);
    m_matched_counts.erase(channel_id);
    if (m_initialized && m_channel) {
        m_channel->SendFrame(core::FrameKind::Unadvertise, channel_id, m_sim_time_ns);
    }
}

size_t ChROSBridge::GetMatchedCount(uint32_t channel_id) const {
    auto it = m_matched_counts.find(channel_id);
    return it == m_matched_counts.end() ? 0 : it->second;
}

bool ChROSBridge::WaitForMessageOn(ChROSSubscription& subscription, double timeout_seconds) {
    RequireInitialized("WaitForMessage");
    const uint64_t initial_count = subscription.GetReceivedCount();
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(std::max(timeout_seconds, 0.0));

    while (true) {
        ProcessIncoming();
        if (subscription.GetReceivedCount() > initial_count) {
            return true;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            return false;
        }
        if (!m_process->IsRunning()) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
}

// ----------------------------------------------------------------------------
// ChROSPublisher / ChROSSubscription
// ----------------------------------------------------------------------------

ChROSPublisher::ChROSPublisher(std::shared_ptr<ChROSBridge> bridge,
                               uint32_t channel_id,
                               std::shared_ptr<const core::Schema> schema,
                               std::string topic,
                               std::string type_name)
    : m_bridge(std::move(bridge)),
      m_channel_id(channel_id),
      m_schema(std::move(schema)),
      m_topic(std::move(topic)),
      m_type_name(std::move(type_name)) {}

ChROSPublisher::~ChROSPublisher() {
    m_bridge->Unadvertise(m_channel_id);
}

ChROSMessage ChROSPublisher::NewMessage() const {
    return ChROSMessage(std::make_shared<core::MessageBuilder>(m_schema));
}

bool ChROSPublisher::Publish(const ChROSMessage& message) {
    return m_bridge->PublishSerialized(m_channel_id, message, m_schema);
}

size_t ChROSPublisher::GetSubscriptionCount() const {
    return m_bridge->GetMatchedCount(m_channel_id);
}

std::string ChROSPublisher::DescribeType() const {
    return m_schema->ToString();
}

ChROSSubscription::ChROSSubscription(std::shared_ptr<ChROSBridge> bridge,
                                     uint32_t channel_id,
                                     std::shared_ptr<const core::Schema> schema,
                                     std::string topic,
                                     std::string type_name,
                                     std::shared_ptr<ChROSSubscriptionCallback> callback)
    : m_bridge(std::move(bridge)),
      m_channel_id(channel_id),
      m_schema(std::move(schema)),
      m_topic(std::move(topic)),
      m_type_name(std::move(type_name)),
      m_callback(std::move(callback)) {}

ChROSSubscription::~ChROSSubscription() {
    m_bridge->Unadvertise(m_channel_id);
}

size_t ChROSSubscription::GetPublisherCount() const {
    return m_bridge->GetMatchedCount(m_channel_id);
}

bool ChROSSubscription::WaitForMessage(double timeout_seconds) {
    return m_bridge->WaitForMessageOn(*this, timeout_seconds);
}

std::string ChROSSubscription::DescribeType() const {
    return m_schema->ToString();
}

}  // namespace ros
}  // namespace chrono
