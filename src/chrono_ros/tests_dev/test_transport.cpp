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
// Dev-time tests for the IPC transport: ring buffer semantics (wraparound,
// atomic two-chunk publication), shared-memory channel round trips in both
// directions, oversized-frame diagnostics, control payload codecs, and a
// threaded producer/consumer stress run.
//
// =============================================================================

#include "chrono_ros/core/ChROSCdr.h"
#include "chrono_ros/core/ChROSControl.h"
#include "chrono_ros/core/transport/ChROSChannel.h"
#include "chrono_ros/core/transport/ChROSRingBuffer.h"
#include "ChROSTestUtils.h"

#include <atomic>
#include <cstring>
#include <random>
#include <thread>

using namespace chrono::ros::core;

namespace {

/// Unique-ish shm names per test run so parallel/failed runs do not collide.
std::string TestShmName(const char* tag) {
    static std::mt19937_64 rng(std::random_device{}());
    return std::string("chros_test_") + tag + "_" + std::to_string(rng() & 0xFFFFFF);
}

struct HeapRing {
    std::atomic<uint64_t> head{0};
    std::atomic<uint64_t> tail{0};
    std::vector<uint8_t> data;
    RingBuffer ring;

    explicit HeapRing(size_t capacity) : data(capacity), ring(&head, &tail, data.data(), capacity) {}
};

}  // namespace

// ----------------------------------------------------------------------------
// RingBuffer
// ----------------------------------------------------------------------------

TEST(ring_basic_write_read) {
    HeapRing r(1024);
    const char msg[] = "hello";
    CHECK(r.ring.Write(msg, sizeof(msg)));
    CHECK_EQ(r.ring.Available(), sizeof(msg));
    char out[sizeof(msg)];
    CHECK(r.ring.Read(out, sizeof(out)));
    CHECK(std::strcmp(out, "hello") == 0);
    CHECK_EQ(r.ring.Available(), size_t(0));
}

TEST(ring_rejects_oversize_and_partial) {
    HeapRing r(64);
    std::vector<uint8_t> big(65, 1);
    CHECK(!r.ring.Write(big.data(), big.size()));
    std::vector<uint8_t> half(40, 2);
    CHECK(r.ring.Write(half.data(), half.size()));
    CHECK(!r.ring.Write(half.data(), half.size()));  // only 24 bytes left
    CHECK_EQ(r.ring.Space(), size_t(24));
}

TEST(ring_wraparound_preserves_data) {
    HeapRing r(256);
    std::mt19937 rng(42);
    uint8_t out[64];
    for (int round = 0; round < 1000; round++) {
        const size_t size = 1 + rng() % 64;
        std::vector<uint8_t> chunk(size);
        for (auto& b : chunk) {
            b = static_cast<uint8_t>(rng());
        }
        CHECK(r.ring.Write(chunk.data(), chunk.size()));
        CHECK(r.ring.Read(out, size));
        CHECK(std::memcmp(out, chunk.data(), size) == 0);
    }
}

TEST(ring_two_chunk_write_is_contiguous) {
    HeapRing r(128);
    const uint32_t a = 0xAABBCCDD;
    const uint8_t b[] = {1, 2, 3};
    CHECK(r.ring.Write(&a, sizeof(a), b, sizeof(b)));
    uint8_t out[7];
    CHECK(r.ring.Read(out, sizeof(out)));
    CHECK(std::memcmp(out, &a, 4) == 0);
    CHECK(std::memcmp(out + 4, b, 3) == 0);
}

TEST(ring_peek_does_not_consume) {
    HeapRing r(128);
    const uint64_t v = 99;
    CHECK(r.ring.Write(&v, sizeof(v)));
    uint64_t peeked = 0;
    CHECK(r.ring.Peek(&peeked, sizeof(peeked)));
    CHECK_EQ(peeked, v);
    CHECK_EQ(r.ring.Available(), sizeof(v));
    r.ring.Consume(sizeof(v));
    CHECK_EQ(r.ring.Available(), size_t(0));
}

TEST(ring_threaded_stress) {
    HeapRing r(1 << 12);
    constexpr int kMessages = 20000;
    std::atomic<bool> failed{false};

    std::thread producer([&] {
        std::mt19937 rng(7);
        for (int i = 0; i < kMessages; i++) {
            uint32_t size = 4 + rng() % 60;
            std::vector<uint8_t> chunk(size);
            std::memcpy(chunk.data(), &i, 4);
            for (size_t j = 4; j < size; j++) {
                chunk[j] = static_cast<uint8_t>(i + j);
            }
            while (!r.ring.Write(&size, 4, chunk.data(), size)) {
                std::this_thread::yield();
            }
        }
    });
    std::thread consumer([&] {
        for (int i = 0; i < kMessages; i++) {
            uint32_t size = 0;
            while (!r.ring.Read(&size, 4)) {
                std::this_thread::yield();
            }
            std::vector<uint8_t> chunk(size);
            while (!r.ring.Read(chunk.data(), size)) {
                std::this_thread::yield();
            }
            int seq = -1;
            std::memcpy(&seq, chunk.data(), 4);
            if (seq != i) {
                failed = true;
                return;
            }
            for (size_t j = 4; j < size; j++) {
                if (chunk[j] != static_cast<uint8_t>(i + j)) {
                    failed = true;
                    return;
                }
            }
        }
    });
    producer.join();
    consumer.join();
    CHECK(!failed);
}

// ----------------------------------------------------------------------------
// Channel over real shared memory
// ----------------------------------------------------------------------------

TEST(channel_bidirectional_roundtrip) {
    const std::string name = TestShmName("rt");
    Channel::Config config;
    config.sim_to_node_capacity = 1 << 20;
    config.node_to_sim_capacity = 1 << 20;
    auto sim = Channel::Create(name, config);
    CHECK(!sim->IsPeerAttached());
    auto node = Channel::Open(name);
    CHECK(sim->IsPeerAttached());

    // sim -> node
    const char payload[] = "sensor bytes";
    CHECK(sim->SendFrame(FrameKind::Publish, 3, 1234567, payload, sizeof(payload)));
    FrameHeader header;
    std::vector<uint8_t> received;
    CHECK(node->ReceiveFrame(header, received));
    CHECK_EQ(header.kind, static_cast<uint16_t>(FrameKind::Publish));
    CHECK_EQ(header.channel_id, 3u);
    CHECK_EQ(header.sim_time_ns, 1234567u);
    CHECK_EQ(received.size(), sizeof(payload));
    CHECK(std::memcmp(received.data(), payload, sizeof(payload)) == 0);

    // node -> sim
    CHECK(node->SendFrame(FrameKind::Received, 8, 0, payload, 5));
    CHECK(sim->ReceiveFrame(header, received));
    CHECK_EQ(header.kind, static_cast<uint16_t>(FrameKind::Received));
    CHECK_EQ(received.size(), size_t(5));

    // empty
    CHECK(!sim->ReceiveFrame(header, received));
    CHECK(!node->ReceiveFrame(header, received));

    // payload-less frame
    CHECK(sim->SendFrame(FrameKind::Shutdown, 0, 0));
    CHECK(node->ReceiveFrame(header, received));
    CHECK_EQ(header.kind, static_cast<uint16_t>(FrameKind::Shutdown));
    CHECK(received.empty());
}

TEST(channel_large_frame) {
    const std::string name = TestShmName("big");
    Channel::Config config;
    config.sim_to_node_capacity = 8u << 20;
    config.node_to_sim_capacity = 1u << 20;
    auto sim = Channel::Create(name, config);
    auto node = Channel::Open(name);

    std::vector<uint8_t> image(5u << 20);  // 5 MiB "camera frame"
    for (size_t i = 0; i < image.size(); i++) {
        image[i] = static_cast<uint8_t>(i * 31);
    }
    CHECK(sim->SendFrame(FrameKind::Publish, 1, 0, image.data(), image.size()));

    FrameHeader header;
    std::vector<uint8_t> received;
    CHECK(node->ReceiveFrame(header, received));
    CHECK(received == image);
}

TEST(channel_oversized_frame_reports_capacity) {
    const std::string name = TestShmName("over");
    Channel::Config config;
    config.sim_to_node_capacity = 1 << 20;
    config.node_to_sim_capacity = 1 << 20;
    auto sim = Channel::Create(name, config);
    std::vector<uint8_t> huge(2u << 20);
    CHECK_THROWS(sim->SendFrame(FrameKind::Publish, 1, 0, huge.data(), huge.size()), ChannelError,
                 "increase the channel capacity");
}

TEST(channel_full_returns_false_then_recovers) {
    const std::string name = TestShmName("full");
    Channel::Config config;
    config.sim_to_node_capacity = 64 * 1024;  // minimum
    config.node_to_sim_capacity = 64 * 1024;
    auto sim = Channel::Create(name, config);
    auto node = Channel::Open(name);

    std::vector<uint8_t> chunk(20 * 1024, 0x55);
    int queued = 0;
    while (sim->SendFrame(FrameKind::Publish, 1, 0, chunk.data(), chunk.size())) {
        queued++;
        CHECK(queued < 100);  // must eventually report full
    }
    CHECK(queued >= 2);

    FrameHeader header;
    std::vector<uint8_t> received;
    CHECK(node->ReceiveFrame(header, received));  // drain one
    CHECK(sim->SendFrame(FrameKind::Publish, 1, 0, chunk.data(), chunk.size()));  // space again
}

TEST(channel_open_missing_fails) {
    bool threw = false;
    try {
        Channel::Open(TestShmName("missing"));
    } catch (const std::exception&) {
        threw = true;
    }
    CHECK(threw);
}

TEST(channel_wraparound_stress_with_frames) {
    const std::string name = TestShmName("wrap");
    Channel::Config config;
    config.sim_to_node_capacity = 64 * 1024;
    config.node_to_sim_capacity = 64 * 1024;
    auto sim = Channel::Create(name, config);
    auto node = Channel::Open(name);

    std::mt19937 rng(123);
    FrameHeader header;
    std::vector<uint8_t> received;
    for (uint32_t i = 0; i < 5000; i++) {
        std::vector<uint8_t> payload(1 + rng() % 3000);
        for (auto& b : payload) {
            b = static_cast<uint8_t>(rng());
        }
        while (!sim->SendFrame(FrameKind::Publish, i, i, payload.data(), payload.size())) {
            CHECK(node->ReceiveFrame(header, received));
        }
        if (i % 3 == 0) {
            if (node->ReceiveFrame(header, received)) {
                CHECK_EQ(header.channel_id, header.sim_time_ns);  // we set them equal
            }
        }
    }
    while (node->ReceiveFrame(header, received)) {
    }
}

// ----------------------------------------------------------------------------
// Control payload codecs
// ----------------------------------------------------------------------------

TEST(control_payload_roundtrips) {
    {
        HelloPayload p;
        p.protocol_version = kProtocolVersion;
        p.pid = 4242;
        p.ros_distro = "lyrical";
        p.rmw_id = "rmw_fastrtps_cpp";
        p.node_name = "chrono_ros_node";
        std::vector<uint8_t> buffer;
        p.Encode(buffer);
        const HelloPayload q = HelloPayload::Decode(buffer.data(), buffer.size());
        CHECK_EQ(q.protocol_version, p.protocol_version);
        CHECK_EQ(q.pid, p.pid);
        CHECK_EQ(q.ros_distro, p.ros_distro);
        CHECK_EQ(q.rmw_id, p.rmw_id);
        CHECK_EQ(q.node_name, p.node_name);
    }
    {
        TypeSchemaPayload p;
        p.type_name = "sensor_msgs/msg/Image";
        p.ok = true;
        p.schema_blob = {9, 8, 7};
        std::vector<uint8_t> buffer;
        p.Encode(buffer);
        const auto q = TypeSchemaPayload::Decode(buffer.data(), buffer.size());
        CHECK_EQ(q.type_name, p.type_name);
        CHECK_EQ(q.ok, true);
        CHECK(q.schema_blob == p.schema_blob);
    }
    {
        AdvertisePayload p;
        p.direction = AdvertisePayload::Direction::Subscribe;
        p.topic = "/cmd_vel";
        p.type_name = "geometry_msgs/msg/Twist";
        p.qos = QoSSpec::SensorData();
        std::vector<uint8_t> buffer;
        p.Encode(buffer);
        const auto q = AdvertisePayload::Decode(buffer.data(), buffer.size());
        CHECK(q.direction == AdvertisePayload::Direction::Subscribe);
        CHECK_EQ(q.topic, p.topic);
        CHECK_EQ(q.type_name, p.type_name);
        CHECK(q.qos.reliability == Reliability::BestEffort);
        CHECK_EQ(q.qos.depth, 5u);
    }
    {
        ValidateResultPayload p;
        p.type_name = "x/msg/Y";
        p.ok = false;
        p.first_diff_offset = 17;
        p.expected_window = {1, 2};
        p.actual_window = {3, 4};
        std::vector<uint8_t> buffer;
        p.Encode(buffer);
        const auto q = ValidateResultPayload::Decode(buffer.data(), buffer.size());
        CHECK_EQ(q.first_diff_offset, 17u);
        CHECK(q.expected_window == p.expected_window);
        CHECK(q.actual_window == p.actual_window);
    }
    {
        ChannelInfoPayload p;
        p.matched_endpoints = 3;
        std::vector<uint8_t> buffer;
        p.Encode(buffer);
        CHECK_EQ(ChannelInfoPayload::Decode(buffer.data(), buffer.size()).matched_endpoints, 3u);
    }
}

TEST(advertise_decode_rejects_garbage) {
    AdvertisePayload p;
    p.topic = "/t";
    p.type_name = "a/msg/B";
    std::vector<uint8_t> buffer;
    p.Encode(buffer);
    buffer[0] = 9;  // invalid direction
    CHECK_THROWS(AdvertisePayload::Decode(buffer.data(), buffer.size()), CdrError, "direction");
}

int main() {
    return chros_test::RunAll("transport");
}
