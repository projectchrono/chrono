// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Unit tests for deterministic per-stream RNG seeding in Chrono::Sensor.
//
// WHAT THESE TESTS PROTECT.
// cuRAND separates generators within one buffer by its subsequence parameter, and Chrono already
// spends that parameter on the per-pixel index. So the ONLY thing distinguishing one sensor's random
// numbers from another's is the seed. Handing one fixed seed to several curand_init calls therefore
// makes them the same stream: measured on this code before the fix, two 64x64 buffers produced 4096
// identical draws out of 4096, and a 640x360 buffer was a bit-exact prefix of a 1280x720 one.
//
// The fix derives a per-buffer seed from a stream key. These tests check the property the key must
// have, which is INJECTIVITY: distinct (manager, sensor, filter, purpose) tuples must never map to
// the same seed. A key that is "usually different" is not enough, because a collision does not fail
// loudly; it silently correlates two sensors and quietly invalidates whatever study used them.
//
// WHY THESE RUN WITHOUT A GPU. They test the seed algebra and the assignment of identity, not
// cuRAND itself. That split is deliberate: this file then runs in configurations built without
// OptiX, where most of the sensor suite is skipped. The end-to-end checks that two rendered noisy
// images actually differ live in utest_SEN_camera_convergence.cpp, which needs the render pipeline.
//
// =============================================================================

#include <set>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_sensor/sensors/ChNoiseModel.h"

using namespace chrono;
using namespace chrono::sensor;

namespace {

// A filter that does nothing except record what seed it would have used, so a test can inspect the
// exact value a real stochastic filter would hand to init_cuda_rng. It goes through the same
// protected accessor and the same public helper as ChFilterCameraNoise, which is the point: a test
// that recomputed the seed itself would pass even if every call site were wired up wrongly.
class SeedProbeFilter : public ChFilter {
  public:
    SeedProbeFilter(RngUsage usage, std::string name = "SeedProbe") : ChFilter(name), m_usage(usage) {}

    void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) override {
        m_stream_index = GetRngStreamIndex();
        m_seed = ChSensorManager::GetDeterministicSeed(pSensor, m_usage, m_stream_index);
        m_initialized = true;
    }

    void Apply() override {}

    unsigned long long Seed() const { return m_seed; }
    unsigned int StreamIndex() const { return m_stream_index; }
    bool Initialized() const { return m_initialized; }

  private:
    RngUsage m_usage;
    unsigned long long m_seed = 0;
    unsigned int m_stream_index = CH_SENSOR_UNASSIGNED_RNG_ID;
    bool m_initialized = false;
};

/// Smallest sensor that can be registered without a GPU: a dynamic sensor goes to the dynamics
/// manager rather than to an OptiX engine, and its filters are still initialized during AddSensor.
std::shared_ptr<ChAccelerometerSensor> MakeDynamicSensor(std::shared_ptr<ChBody> body) {
    return chrono_types::make_shared<ChAccelerometerSensor>(body, 10.f, ChFrame<double>(),
                                                            chrono_types::make_shared<ChNoiseNone>());
}

/// A sensor that assembles its own filter chain by writing to the protected m_filters directly,
/// WITHOUT calling PushFilter.
///
/// This is not a contrived shape. ChPhysCameraSensor's constructor does exactly this for its
/// seven-stage pipeline, and one of those stages owns a cuRAND buffer. Relying on PushFilter alone to
/// hand out stream indices therefore left that stage with no identity, and it threw when it asked for
/// a seed, hanging the phys-camera render test. Every test in this file passed while that was broken,
/// because they all attach filters the normal way; this class is what closes that gap.
class SelfAssemblingSensor : public ChAccelerometerSensor {
  public:
    SelfAssemblingSensor(std::shared_ptr<ChBody> body, std::shared_ptr<ChFilter> internal_filter)
        : ChAccelerometerSensor(body, 10.f, ChFrame<double>(), chrono_types::make_shared<ChNoiseNone>()) {
        m_filters.push_back(internal_filter);  // deliberately NOT PushFilter
    }
};

// RAII guard for the process-global fixed seed.
//
// Needed because the seed is process-global and a GoogleTest ASSERT_* aborts the test body: a fatal
// assertion between SetRandomSeed and ClearRandomSeed would leave the fixed seed switched on and
// silently change every later test in the same binary. A reviewer caught exactly that pattern here.
// Setting the seed through this guard makes the cleanup unconditional.
class FixedSeedGuard {
  public:
    /// seed >= 0 pins the base seed; seed < 0 means "leave seeding cleared", matching the
    /// convention the render helpers in this file already use.
    explicit FixedSeedGuard(long long seed) {
        if (seed >= 0)
            ChSensorManager::SetRandomSeed((unsigned int)seed);
        else
            ChSensorManager::ClearRandomSeed();
    }
    ~FixedSeedGuard() { ChSensorManager::ClearRandomSeed(); }
    FixedSeedGuard(const FixedSeedGuard&) = delete;
    FixedSeedGuard& operator=(const FixedSeedGuard&) = delete;
};

struct Scene {
    ChSystemNSC sys;
    std::shared_ptr<ChBody> body;

    Scene() {
        body = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
        body->SetFixed(true);
        sys.Add(body);
    }
};

}  // namespace

// -----------------------------------------------------------------------------
// The stream key must be injective. These tests need no sensors at all.
// -----------------------------------------------------------------------------

// Grok's binding requirement from the design audit, verbatim in intent: distinct ids for the first
// 10k ordinals crossed with 20 purposes. Note this deliberately uses 20 usage values, more than the
// 10 constants RngUsage currently defines, so the field keeps room for future stochastic filters.
TEST(ChSensorRngStreams, stream_id_injective_over_10k_ordinals_x_20_usages) {
    const unsigned int kOrdinals = 10000;
    const unsigned int kUsages = 20;
    ASSERT_LE(kUsages, 1u << CH_RNG_USAGE_BITS) << "the usage field is too narrow for this test";

    std::set<unsigned long long> seen;
    for (unsigned int ordinal = 0; ordinal < kOrdinals; ++ordinal) {
        for (unsigned int u = 0; u < kUsages; ++u) {
            unsigned long long id = ChSensorManager::MakeRngStreamId(0, ordinal, 0, (RngUsage)u);
            ASSERT_TRUE(seen.insert(id).second) << "collision at ordinal " << ordinal << " usage " << u;
        }
    }
    ASSERT_EQ(seen.size(), (size_t)kOrdinals * kUsages);
}

// Injectivity has to hold when every field varies at once, not only along one axis. A packing that
// is injective in each field separately can still collide across fields, which is exactly how the
// unconstrained additive formula the design review started from was broken.
TEST(ChSensorRngStreams, stream_id_injective_across_all_four_fields) {
    std::set<unsigned long long> seen;
    size_t count = 0;
    for (unsigned int mgr = 0; mgr < 7; ++mgr) {
        for (unsigned int ordinal = 0; ordinal < 13; ++ordinal) {
            for (unsigned int filt = 0; filt < 11; ++filt) {
                for (unsigned int u = 0; u < 10; ++u) {
                    unsigned long long id = ChSensorManager::MakeRngStreamId(mgr, ordinal, filt, (RngUsage)u);
                    ASSERT_TRUE(seen.insert(id).second) << "collision at (" << mgr << "," << ordinal << ","
                                                        << filt << "," << u << ")";
                    ++count;
                }
            }
        }
    }
    ASSERT_EQ(seen.size(), count);
}

// A field that overflows its bit width must throw rather than wrap. Wrapping would map two different
// sensors onto one stream, which is the defect, and it would do so silently.
TEST(ChSensorRngStreams, stream_id_rejects_out_of_range_fields) {
    const unsigned int max_usage = 1u << CH_RNG_USAGE_BITS;
    const unsigned int max_filter = 1u << CH_RNG_FILTER_BITS;
    const unsigned int max_sensor = 1u << CH_RNG_SENSOR_BITS;
    const unsigned int max_manager = 1u << CH_RNG_MANAGER_BITS;

    // Largest in-range key in every field must be accepted.
    EXPECT_NO_THROW(ChSensorManager::MakeRngStreamId(max_manager - 1, max_sensor - 1, max_filter - 1,
                                                     (RngUsage)(max_usage - 1)));

    EXPECT_THROW(ChSensorManager::MakeRngStreamId(0, 0, 0, (RngUsage)max_usage), std::runtime_error);
    EXPECT_THROW(ChSensorManager::MakeRngStreamId(0, 0, max_filter, RngUsage::CameraNoiseConstNormal),
                 std::runtime_error);
    EXPECT_THROW(ChSensorManager::MakeRngStreamId(0, max_sensor, 0, RngUsage::CameraNoiseConstNormal),
                 std::runtime_error);
    EXPECT_THROW(ChSensorManager::MakeRngStreamId(max_manager, 0, 0, RngUsage::CameraNoiseConstNormal),
                 std::runtime_error);
}

// Adding the base seed must not undo the packing. This is the wraparound case: at base seed
// 0xFFFFFFFF the sum crosses 2^32, and it would cross 2^64 for a large enough stream id. Addition of
// a constant is a bijection on the 64-bit ring, so distinct ids stay distinct; this test is here so
// that a future change to a non-bijective combining step fails loudly.
TEST(ChSensorRngStreams, base_seed_addition_preserves_distinctness) {
    for (unsigned int base : {0u, 1u, 12345u, 0x7FFFFFFFu, 0xFFFFFFFFu}) {
        FixedSeedGuard seed_guard(base);
        std::set<unsigned long long> seen;
        for (unsigned int ordinal = 0; ordinal < 64; ++ordinal) {
            for (unsigned int u = 0; u < 10; ++u) {
                unsigned long long seed =
                    (unsigned long long)base + ChSensorManager::MakeRngStreamId(0, ordinal, 0, (RngUsage)u);
                ASSERT_TRUE(seen.insert(seed).second) << "collision at base " << base << " ordinal " << ordinal;
            }
        }
    }
}

// -----------------------------------------------------------------------------
// Identity assignment, which requires a manager and a registered sensor.
// -----------------------------------------------------------------------------

TEST(ChSensorRngStreams, add_sensor_assigns_consecutive_ordinals) {
    Scene scene;
    auto manager = chrono_types::make_shared<ChSensorManager>(&scene.sys);

    std::vector<std::shared_ptr<ChAccelerometerSensor>> sensors;
    for (int i = 0; i < 4; ++i) {
        auto s = MakeDynamicSensor(scene.body);
        manager->AddSensor(s);
        sensors.push_back(s);
    }
    for (unsigned int i = 0; i < sensors.size(); ++i) {
        EXPECT_EQ(sensors[i]->GetRngSensorOrdinal(), i);
        EXPECT_NE(sensors[i]->GetRngManagerId(), CH_SENSOR_UNASSIGNED_RNG_ID);
    }
    // All sensors of one manager share its id; that is what makes the ordinal meaningful.
    for (auto& s : sensors)
        EXPECT_EQ(s->GetRngManagerId(), sensors[0]->GetRngManagerId());
}

// The defect this whole change exists to fix, checked at the seed level: two sensors under one fixed
// base seed must not receive the same seed for the same purpose.
TEST(ChSensorRngStreams, two_sensors_same_purpose_get_different_seeds) {
    Scene scene;
    FixedSeedGuard seed_guard(4242);
    auto manager = chrono_types::make_shared<ChSensorManager>(&scene.sys);

    auto s1 = MakeDynamicSensor(scene.body);
    auto p1 = chrono_types::make_shared<SeedProbeFilter>(RngUsage::CameraNoiseConstNormal);
    s1->PushFilter(p1);
    manager->AddSensor(s1);

    auto s2 = MakeDynamicSensor(scene.body);
    auto p2 = chrono_types::make_shared<SeedProbeFilter>(RngUsage::CameraNoiseConstNormal);
    s2->PushFilter(p2);
    manager->AddSensor(s2);

    ASSERT_TRUE(p1->Initialized());
    ASSERT_TRUE(p2->Initialized());
    EXPECT_NE(p1->Seed(), p2->Seed());
}

// One sensor, two purposes. Before the fix both read the same global seed.
TEST(ChSensorRngStreams, one_sensor_two_purposes_get_different_seeds) {
    Scene scene;
    FixedSeedGuard seed_guard(4242);
    auto manager = chrono_types::make_shared<ChSensorManager>(&scene.sys);

    auto s = MakeDynamicSensor(scene.body);
    auto pa = chrono_types::make_shared<SeedProbeFilter>(RngUsage::CameraNoiseConstNormal, "a");
    auto pb = chrono_types::make_shared<SeedProbeFilter>(RngUsage::LidarNoiseXYZI, "b");
    s->PushFilter(pa);
    s->PushFilter(pb);
    manager->AddSensor(s);

    ASSERT_TRUE(pa->Initialized());
    ASSERT_TRUE(pb->Initialized());
    EXPECT_NE(pa->Seed(), pb->Seed());
}

// The case the design review split over, and the reason the key carries a filter index at all.
// ChSensor::PushFilter applies no type or duplicate check, so two filters of the same kind can sit
// on one sensor, each owning its own cuRAND buffer. Without the index they would answer to the same
// (sensor, purpose) pair and draw identical numbers.
TEST(ChSensorRngStreams, duplicate_same_purpose_filters_get_different_seeds) {
    Scene scene;
    FixedSeedGuard seed_guard(4242);
    auto manager = chrono_types::make_shared<ChSensorManager>(&scene.sys);

    auto s = MakeDynamicSensor(scene.body);
    auto p1 = chrono_types::make_shared<SeedProbeFilter>(RngUsage::CameraNoiseConstNormal, "dup1");
    auto p2 = chrono_types::make_shared<SeedProbeFilter>(RngUsage::CameraNoiseConstNormal, "dup2");
    s->PushFilter(p1);
    s->PushFilter(p2);
    manager->AddSensor(s);

    ASSERT_TRUE(p1->Initialized());
    ASSERT_TRUE(p2->Initialized());
    EXPECT_NE(p1->StreamIndex(), p2->StreamIndex());
    EXPECT_NE(p1->Seed(), p2->Seed());
}

// The index is attach order, not list position. ChOptixEngine prepends its render filter with
// PushFilterFront after the user's filters are already attached, so a positional index would
// renumber filters that had already been stamped.
TEST(ChSensorRngStreams, filter_stream_index_is_attach_order_not_position) {
    Scene scene;
    FixedSeedGuard seed_guard(555);
    auto manager = chrono_types::make_shared<ChSensorManager>(&scene.sys);
    auto s = MakeDynamicSensor(scene.body);

    auto first = chrono_types::make_shared<SeedProbeFilter>(RngUsage::CameraNoiseConstNormal, "first");
    s->PushFilter(first);
    auto prepended = chrono_types::make_shared<SeedProbeFilter>(RngUsage::CameraNoisePixDep, "prepended");
    s->PushFilterFront(prepended);

    // The index is only observable once the filter initializes, which is what AddSensor triggers.
    manager->AddSensor(s);
    ASSERT_TRUE(first->Initialized());
    ASSERT_TRUE(prepended->Initialized());

    // Attached second, so it holds the higher index even though it now sits earlier in the list.
    unsigned int i_first = first->StreamIndex();
    unsigned int i_prep = prepended->StreamIndex();
    EXPECT_NE(i_first, CH_SENSOR_UNASSIGNED_RNG_ID);
    EXPECT_NE(i_prep, CH_SENSOR_UNASSIGNED_RNG_ID);
    EXPECT_LT(i_first, i_prep) << "the prepended filter must not have been renumbered ahead of the first";

    // And the list order really is prepended-then-first, so the indices are not list positions.
    auto filters = s->GetFilterList();
    ASSERT_GE(filters.size(), (size_t)2);
    EXPECT_EQ(filters.front().get(), prepended.get());
}

// A filter that reached m_filters without going through PushFilter must still get an identity.
//
// This is the regression test for a defect this change introduced and that the rest of this file did
// not catch: ChPhysCameraSensor fills m_filters directly in its constructor, so its shot-noise stage
// had no stream index and threw when it asked for a seed. The fix stamps pending indices in
// AddSensor, which every sensor must pass through, rather than trusting PushFilter to be the only
// entry point into a protected member.
TEST(ChSensorRngStreams, filter_added_without_PushFilter_still_gets_an_identity) {
    Scene scene;
    FixedSeedGuard seed_guard(8675309);
    auto manager = chrono_types::make_shared<ChSensorManager>(&scene.sys);

    auto internal = chrono_types::make_shared<SeedProbeFilter>(RngUsage::PhysCameraShotNoise, "internal");
    auto s = chrono_types::make_shared<SelfAssemblingSensor>(scene.body, internal);
    ASSERT_EQ(internal->StreamIndex(), CH_SENSOR_UNASSIGNED_RNG_ID) << "precondition: not stamped yet";

    // Must not throw. Before the fix this threw std::runtime_error from GetDeterministicSeed.
    ASSERT_NO_THROW(manager->AddSensor(s));

    ASSERT_TRUE(internal->Initialized()) << "the internally-added filter was never initialized";
    EXPECT_NE(internal->StreamIndex(), CH_SENSOR_UNASSIGNED_RNG_ID)
        << "a filter that bypassed PushFilter got no RNG stream index, so it cannot be seeded";
    EXPECT_NE(internal->Seed(), 0ull);
}

// Two such sensors must still get different streams, which is the property the identity exists for.
TEST(ChSensorRngStreams, two_self_assembling_sensors_get_different_seeds) {
    Scene scene;
    FixedSeedGuard seed_guard(8675309);
    auto manager = chrono_types::make_shared<ChSensorManager>(&scene.sys);

    auto p1 = chrono_types::make_shared<SeedProbeFilter>(RngUsage::PhysCameraShotNoise, "i1");
    auto p2 = chrono_types::make_shared<SeedProbeFilter>(RngUsage::PhysCameraShotNoise, "i2");
    manager->AddSensor(chrono_types::make_shared<SelfAssemblingSensor>(scene.body, p1));
    manager->AddSensor(chrono_types::make_shared<SelfAssemblingSensor>(scene.body, p2));

    ASSERT_TRUE(p1->Initialized());
    ASSERT_TRUE(p2->Initialized());
    EXPECT_NE(p1->Seed(), p2->Seed());
}

// Two coexisting managers each start their ordinals at zero while the base seed is process-global.
// Without a manager field in the key, their first sensors would share a stream. No in-tree code
// builds two managers at once, but the public API permits it, which is why this is tested rather
// than documented.
TEST(ChSensorRngStreams, two_managers_same_base_seed_do_not_collide) {
    Scene scene;
    FixedSeedGuard seed_guard(777);

    auto mgr_a = chrono_types::make_shared<ChSensorManager>(&scene.sys);
    auto mgr_b = chrono_types::make_shared<ChSensorManager>(&scene.sys);

    auto sa = MakeDynamicSensor(scene.body);
    auto pa = chrono_types::make_shared<SeedProbeFilter>(RngUsage::CameraNoiseConstNormal);
    sa->PushFilter(pa);
    mgr_a->AddSensor(sa);

    auto sb = MakeDynamicSensor(scene.body);
    auto pb = chrono_types::make_shared<SeedProbeFilter>(RngUsage::CameraNoiseConstNormal);
    sb->PushFilter(pb);
    mgr_b->AddSensor(sb);

    // Same ordinal in each manager, which is the situation that used to collide.
    ASSERT_EQ(sa->GetRngSensorOrdinal(), sb->GetRngSensorOrdinal());
    ASSERT_NE(sa->GetRngManagerId(), sb->GetRngManagerId());
    EXPECT_NE(pa->Seed(), pb->Seed());
}

// Reproducibility, and the reason the manager id is a reusable slot rather than a counter.
//
// Building the same scene twice IN ONE PROCESS must give identical seeds, because that is what the
// existing byte-exact render tests do: render, tear down, render again, compare images. A monotonic
// manager counter passes every other test in this file and fails this one, which is how the counter
// was caught during development. Sequential managers must land on the same slot.
TEST(ChSensorRngStreams, same_scene_built_twice_in_one_process_gives_identical_seeds) {
    std::vector<unsigned long long> run_seeds[2];
    for (int run = 0; run < 2; ++run) {
        Scene scene;
        FixedSeedGuard seed_guard(31337);
        {
            auto manager = chrono_types::make_shared<ChSensorManager>(&scene.sys);
            for (int i = 0; i < 3; ++i) {
                auto s = MakeDynamicSensor(scene.body);
                auto p = chrono_types::make_shared<SeedProbeFilter>(RngUsage::CameraNoiseConstNormal);
                s->PushFilter(p);
                manager->AddSensor(s);
                run_seeds[run].push_back(p->Seed());
            }
        }  // manager destroyed here, releasing its slot for the next run
    }

    ASSERT_EQ(run_seeds[0].size(), run_seeds[1].size());
    for (size_t i = 0; i < run_seeds[0].size(); ++i)
        EXPECT_EQ(run_seeds[0][i], run_seeds[1][i]) << "sensor " << i << " drew a different stream on the "
                                                       "second build of an identical scene";

    // And within a run the three sensors still differ from each other.
    std::set<unsigned long long> uniq(run_seeds[0].begin(), run_seeds[0].end());
    EXPECT_EQ(uniq.size(), run_seeds[0].size());
}

// The flip side of slot reuse: while two managers are alive their ids must differ, and once one is
// gone its slot is available again. Stated as a test because the two halves pull in opposite
// directions and a change that fixes one can silently break the other.
TEST(ChSensorRngStreams, manager_slots_are_distinct_while_alive_and_reused_after) {
    Scene scene;
    unsigned int first_id;
    {
        auto a = chrono_types::make_shared<ChSensorManager>(&scene.sys);
        auto sa = MakeDynamicSensor(scene.body);
        a->AddSensor(sa);
        first_id = sa->GetRngManagerId();

        auto b = chrono_types::make_shared<ChSensorManager>(&scene.sys);
        auto sb = MakeDynamicSensor(scene.body);
        b->AddSensor(sb);
        EXPECT_NE(sb->GetRngManagerId(), first_id) << "two live managers share an id";
    }
    auto c = chrono_types::make_shared<ChSensorManager>(&scene.sys);
    auto sc = MakeDynamicSensor(scene.body);
    c->AddSensor(sc);
    EXPECT_EQ(sc->GetRngManagerId(), first_id) << "a freed slot was not reused, so sequential managers "
                                                  "drift and reproducibility is lost";
}

// Default behavior must be unchanged: with no fixed seed every call reads the clock afresh, so
// nothing is reproducible unless the user asks for it.
TEST(ChSensorRngStreams, no_fixed_seed_still_varies_per_call) {
    Scene scene;
    ASSERT_FALSE(ChSensorManager::HasRandomSeed());

    auto manager = chrono_types::make_shared<ChSensorManager>(&scene.sys);
    auto s = MakeDynamicSensor(scene.body);
    manager->AddSensor(s);

    std::set<unsigned long long> seeds;
    for (int i = 0; i < 8; ++i)
        seeds.insert(ChSensorManager::GetDeterministicSeed(s, RngUsage::CameraNoiseConstNormal, 0));

    // A clock with coarse resolution could legitimately repeat a value on consecutive calls, so this
    // asserts "not all the same" rather than "all distinct". A fixed seed would give exactly one.
    EXPECT_GT(seeds.size(), (size_t)1) << "the no-fixed-seed path looks constant, which would make the "
                                          "default behavior deterministic without the user asking";
}

// Seeding from the unassigned sentinel would give every unregistered sensor the same stream, so it
// throws instead. In the shipped tree this is unreachable, because the only code that initializes
// filters runs inside AddSensor after the ordinal is assigned; it guards a future caller.
TEST(ChSensorRngStreams, unregistered_sensor_throws) {
    Scene scene;
    FixedSeedGuard seed_guard(99);

    auto orphan = MakeDynamicSensor(scene.body);  // never handed to a manager
    ASSERT_EQ(orphan->GetRngSensorOrdinal(), CH_SENSOR_UNASSIGNED_RNG_ID);
    EXPECT_THROW(ChSensorManager::GetDeterministicSeed(orphan, RngUsage::CameraNoiseConstNormal, 0),
                 std::runtime_error);

    // A registered sensor with an unattached filter is equally unsafe.
    auto registered = MakeDynamicSensor(scene.body);
    auto manager = chrono_types::make_shared<ChSensorManager>(&scene.sys);
    manager->AddSensor(registered);
    EXPECT_THROW(ChSensorManager::GetDeterministicSeed(registered, RngUsage::CameraNoiseConstNormal,
                                                       CH_SENSOR_UNASSIGNED_RNG_ID),
                 std::runtime_error);

    EXPECT_THROW(ChSensorManager::GetDeterministicSeed(nullptr, RngUsage::CameraNoiseConstNormal, 0),
                 std::runtime_error);
}

// -----------------------------------------------------------------------------
// Statistical oracle and topology fuzzer.
//
// Every test above checks a property somebody thought to name. These two exist to catch the ones
// nobody did, which is how both the original defect and the design review's first proposed fix got
// through: each was a collision in a case no acceptance criterion mentioned.
// -----------------------------------------------------------------------------

// Build randomized sensor topologies and demand that every derived seed in the whole scene is
// distinct. The generator does not know which combinations are dangerous, which is the point.
TEST(ChSensorRngStreams, topology_fuzzer_finds_no_colliding_streams) {
    // Fixed generator seed: a fuzzer that cannot be replayed is not much use in a regression suite.
    unsigned int rng = 20260731u;
    auto next = [&rng](unsigned int n) {
        rng = rng * 1664525u + 1013904223u;  // Numerical Recipes LCG, adequate for choosing shapes
        return (rng >> 16) % n;
    };

    const RngUsage kUsages[] = {RngUsage::CameraNoiseConstNormal, RngUsage::CameraNoisePixDep,
                                RngUsage::LidarNoiseXYZI,         RngUsage::PhysCameraShotNoise,
                                RngUsage::OptixCameraRaygen,      RngUsage::OptixSegmentationRaygen};
    const unsigned int kNumUsages = sizeof(kUsages) / sizeof(kUsages[0]);

    for (int trial = 0; trial < 40; ++trial) {
        Scene scene;
        FixedSeedGuard seed_guard(1000u + trial);

        const unsigned int n_managers = 1 + next(3);
        std::vector<std::shared_ptr<ChSensorManager>> managers;
        for (unsigned int m = 0; m < n_managers; ++m)
            managers.push_back(chrono_types::make_shared<ChSensorManager>(&scene.sys));

        std::vector<std::shared_ptr<SeedProbeFilter>> probes;
        const unsigned int n_sensors = 1 + next(6);
        for (unsigned int i = 0; i < n_sensors; ++i) {
            auto s = MakeDynamicSensor(scene.body);
            const unsigned int n_filters = 1 + next(4);
            for (unsigned int f = 0; f < n_filters; ++f) {
                // Usage repeats on purpose, so duplicate same-purpose filters do occur.
                auto p = chrono_types::make_shared<SeedProbeFilter>(kUsages[next(kNumUsages)]);
                probes.push_back(p);
                if (next(2))
                    s->PushFilter(p);
                else
                    s->PushFilterFront(p);
            }
            managers[next(n_managers)]->AddSensor(s);
        }

        std::set<unsigned long long> seeds;
        size_t initialized = 0;
        for (auto& p : probes) {
            if (!p->Initialized())
                continue;  // a filter on a sensor its manager rejected
            ++initialized;
            ASSERT_TRUE(seeds.insert(p->Seed()).second)
                << "trial " << trial << ": two RNG streams in one scene got the same seed";
        }
        ASSERT_GT(initialized, (size_t)0) << "trial " << trial << " initialized nothing, so it proved nothing";
    }
}

// A birthday-bound check on the derived seeds themselves. With N streams drawn from a 64-bit space,
// even one exact collision is overwhelming evidence of structure rather than chance: the expected
// number of collisions among 100k values in 2^64 is about 2.7e-10. So the threshold is zero, and any
// hit means the key is not injective somewhere the named tests do not look.
TEST(ChSensorRngStreams, derived_seeds_show_no_collisions_at_birthday_bound) {
    FixedSeedGuard seed_guard(0xABCDEF01u);
    std::set<unsigned long long> seeds;
    size_t n = 0;
    for (unsigned int mgr = 0; mgr < 5; ++mgr) {
        for (unsigned int ordinal = 0; ordinal < 200; ++ordinal) {
            for (unsigned int filt = 0; filt < 10; ++filt) {
                for (unsigned int u = 1; u < 10; ++u) {
                    unsigned long long seed = 0xABCDEF01ull +
                                              ChSensorManager::MakeRngStreamId(mgr, ordinal, filt, (RngUsage)u);
                    ASSERT_TRUE(seeds.insert(seed).second)
                        << "collision after " << n << " streams at (" << mgr << "," << ordinal << "," << filt
                        << "," << u << ")";
                    ++n;
                }
            }
        }
    }
    ASSERT_EQ(seeds.size(), n);
    std::cout << "checked " << n << " derived seeds, 0 collisions" << std::endl;
}

// -----------------------------------------------------------------------------
// Tests added in response to audit 2. Each names the finding it closes, so that a later reader can
// tell which of these exist because somebody caught something rather than because they seemed nice.
// -----------------------------------------------------------------------------

// AUDIT-2 FINDING A3. The identity is a precondition of the call and must be checked on EVERY call,
// not only when a fixed seed happens to be set.
//
// The original code returned the clock value first and validated afterwards, so a filter with no
// identity was diagnosed only once a user turned reproducibility on. That is the worst possible
// timing: the mistake is made during ordinary development and surfaces later, in someone else's run,
// far from its cause.
TEST(ChSensorRngStreams, unregistered_sensor_throws_even_with_no_fixed_seed) {
    Scene scene;
    ASSERT_FALSE(ChSensorManager::HasRandomSeed()) << "precondition: no fixed seed in force";

    auto orphan = MakeDynamicSensor(scene.body);  // never handed to a manager
    EXPECT_THROW(ChSensorManager::GetDeterministicSeed(orphan, RngUsage::CameraNoiseConstNormal, 0),
                 std::runtime_error)
        << "an unregistered sensor was silently given a clock seed because no fixed seed was set";

    EXPECT_THROW(ChSensorManager::GetDeterministicSeed(nullptr, RngUsage::CameraNoiseConstNormal, 0),
                 std::runtime_error);

    auto registered = MakeDynamicSensor(scene.body);
    auto manager = chrono_types::make_shared<ChSensorManager>(&scene.sys);
    manager->AddSensor(registered);
    EXPECT_THROW(ChSensorManager::GetDeterministicSeed(registered, RngUsage::CameraNoiseConstNormal,
                                                       CH_SENSOR_UNASSIGNED_RNG_ID),
                 std::runtime_error)
        << "a filter with no stream index was silently given a clock seed";

    // And the valid case on the same path still returns a clock-derived value rather than throwing.
    EXPECT_NO_THROW(ChSensorManager::GetDeterministicSeed(registered, RngUsage::CameraNoiseConstNormal, 0));
}

// AUDIT-2 FINDING A5. RngUsage is documented as a closed set, so the production path must enforce it.
// MakeRngStreamId is deliberately laxer, because the usage FIELD is wider than the enum so that
// injectivity can be proved over more purposes than exist yet; that tolerance must not leak into
// seeding real buffers.
TEST(ChSensorRngStreams, production_path_rejects_undeclared_or_placeholder_usage) {
    Scene scene;
    FixedSeedGuard seed_guard(4242);
    auto manager = chrono_types::make_shared<ChSensorManager>(&scene.sys);
    auto s = MakeDynamicSensor(scene.body);
    manager->AddSensor(s);

    // Unknown is a placeholder, not a stream identity.
    EXPECT_THROW(ChSensorManager::GetDeterministicSeed(s, RngUsage::Unknown, 0), std::runtime_error);

    // A value cast into an undeclared slot, still inside the 5-bit field, must also be refused.
    const unsigned int past_end = (unsigned int)RngUsage::Count;
    ASSERT_LT(past_end, 1u << CH_RNG_USAGE_BITS) << "this test needs a slot the field allows but the "
                                                    "enum does not declare";
    EXPECT_THROW(ChSensorManager::GetDeterministicSeed(s, (RngUsage)past_end, 0), std::runtime_error);

    // Every declared purpose is still accepted.
    for (unsigned int u = 1; u < past_end; ++u)
        EXPECT_NO_THROW(ChSensorManager::GetDeterministicSeed(s, (RngUsage)u, 0))
            << "declared usage " << u << " was rejected";

    // The raw packer stays permissive on purpose, which is what lets the injectivity tests above
    // cover 20 purposes.
    EXPECT_NO_THROW(ChSensorManager::MakeRngStreamId(0, 0, 0, (RngUsage)past_end));
}

// AUDIT-2 FINDING C3. The sentinel throw was only ever exercised by calling the helper directly.
// This forces it out of a REAL registration, through ChDynamicsManager::AssignSensor and
// ChSensorManager::AddSensor, and asserts it reaches the caller.
//
// Only the dynamic-sensor path can be asserted this way. The OptiX path catches the exception and
// calls exit(1), so a test that triggered it would kill the test process rather than fail. That
// asymmetry is real and is recorded in DEVIATIONS D14 and D16 rather than papered over here.
namespace {

// A filter that asks for a seed with a deliberately unassigned stream index, imitating a filter that
// reached the list without being stamped.
class ThrowingProbeFilter : public ChFilter {
  public:
    ThrowingProbeFilter() : ChFilter("ThrowingProbe") {}
    void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) override {
        ChSensorManager::GetDeterministicSeed(pSensor, RngUsage::CameraNoiseConstNormal,
                                             CH_SENSOR_UNASSIGNED_RNG_ID);
    }
    void Apply() override {}
};

}  // namespace

TEST(ChSensorRngStreams, sentinel_throw_reaches_the_caller_through_AddSensor) {
    Scene scene;
    FixedSeedGuard seed_guard(1234);
    auto manager = chrono_types::make_shared<ChSensorManager>(&scene.sys);

    auto s = MakeDynamicSensor(scene.body);
    s->PushFilter(chrono_types::make_shared<ThrowingProbeFilter>());

    EXPECT_THROW(manager->AddSensor(s), std::runtime_error)
        << "the seeding precondition failure did not reach the caller of AddSensor; on the OptiX path "
           "this same condition once produced an indefinite hang (DEVIATIONS D16)";
}

// AUDIT-2 FINDING A2. The fixed-seed state is process-global, so the pair must be read together.
// This does not prove thread safety, which needs a race detector, but it does pin the invariant that
// a cleared seed is never observed as "fixed with value zero".
TEST(ChSensorRngStreams, clearing_the_seed_is_observed_atomically) {
    Scene scene;
    auto manager = chrono_types::make_shared<ChSensorManager>(&scene.sys);
    auto s = MakeDynamicSensor(scene.body);
    manager->AddSensor(s);

    // The guard still provides the cleanup-on-fatal-assertion safety, but this test also has to call
    // ClearRandomSeed explicitly MID-TEST, because the transition from "fixed seed of value zero" to
    // "no fixed seed" is the thing under test. That is the one legitimate direct use of the raw API in
    // this file; everywhere else the guard owns it.
    FixedSeedGuard seed_guard(0);  // a fixed seed whose VALUE is zero
    ASSERT_TRUE(ChSensorManager::HasRandomSeed());
    const unsigned long long fixed_zero =
        ChSensorManager::GetDeterministicSeed(s, RngUsage::CameraNoiseConstNormal, 0);
    // With base seed 0 the derived seed is exactly the stream id, so it is small and deterministic.
    EXPECT_EQ(fixed_zero, ChSensorManager::MakeRngStreamId(s->GetRngManagerId(), s->GetRngSensorOrdinal(),
                                                           0, RngUsage::CameraNoiseConstNormal));

    ChSensorManager::ClearRandomSeed();
    ASSERT_FALSE(ChSensorManager::HasRandomSeed());
    // Now the same call must be clock-derived, which is enormous next to a stream id, so the two
    // states are distinguishable rather than both looking like "seed 0".
    const unsigned long long cleared =
        ChSensorManager::GetDeterministicSeed(s, RngUsage::CameraNoiseConstNormal, 0);
    EXPECT_GT(cleared, 1ull << 40) << "after ClearRandomSeed the derivation still looks like a fixed "
                                      "base seed of zero rather than a clock reading";
}
