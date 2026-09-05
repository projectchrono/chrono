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
// Authors: Kyle Sha
// =============================================================================
//
// The backend-neutral Modify*Light overloads: does redefining a light from a set of parameters
// produce the same light that adding one from those parameters produces?
//
// That question is the whole contract, and asking it this way needs no knowledge of the light
// struct's fields. Add a light from parameters A, add a second from parameters B, then modify the
// first to B and require the two to become identical. A field the modify path forgets to write
// fails immediately, and the test does not have to be updated when a field is added.
//
// Comparing bytes is sound here because both lights reach the comparison through the same
// construction helper, which zero-initializes the struct before filling it, so padding is equal
// whenever the members are.
//
// Runs against whichever backend the build selected: the scene classes are different types with
// different light structs, and the test is written against neither.
//
// =============================================================================

#include "gtest/gtest.h"

#include <cstring>
#include <functional>
#include <vector>

#include "chrono/core/ChVector3.h"
#include "chrono/assets/ChColor.h"

#include "chrono_sensor/ChConfigSensor.h"

#ifdef CHRONO_HAS_OPTIX
    #include "chrono_sensor/optix/ChOptixScene.h"
using SceneType = chrono::sensor::ChOptixScene;
#elif defined(CHRONO_HAS_METAL_RT)
    #include "chrono_sensor/metal/ChMetalRTScene.h"
using SceneType = chrono::sensor::ChMetalRTScene;
#elif defined(CHRONO_HAS_VULKAN_RT)
    #include "chrono_sensor/vulkan/ChVulkanRTScene.h"
using SceneType = chrono::sensor::ChVulkanRTScene;
#endif

using namespace chrono;

#if defined(CHRONO_HAS_OPTIX) || defined(CHRONO_HAS_METAL_RT) || defined(CHRONO_HAS_VULKAN_RT)

namespace {

// Two distinct parameter sets per light type. They differ in every argument, so a modify path that
// drops one is caught rather than passing because the two happened to agree there.
struct LightPair {
    const char* name;
    std::function<unsigned int(SceneType&)> add_a;
    std::function<unsigned int(SceneType&)> add_b;
    std::function<void(SceneType&, unsigned int)> modify_a_to_b;
};

template <typename L>
::testing::AssertionResult SameLight(const L& lhs, const L& rhs, const char* which) {
    if (std::memcmp(&lhs, &rhs, sizeof(L)) == 0)
        return ::testing::AssertionSuccess();
    return ::testing::AssertionFailure() << which << ": Modify*Light did not reproduce what Add*Light builds from the same "
                                         << "parameters, so at least one field is not written by the modify path";
}

}  // namespace

TEST(ChSensorSceneLights, modify_reproduces_add_for_every_light_type) {
    const std::vector<LightPair> cases = {
        {"point", [](SceneType& s) { return s.AddPointLight(ChVector3f(1, 2, 3), ChColor(0.2f, 0.3f, 0.4f), 10.f, true); },
         [](SceneType& s) { return s.AddPointLight(ChVector3f(-4, 5, 6), ChColor(0.7f, 0.6f, 0.5f), 25.f, false); },
         [](SceneType& s, unsigned int i) { s.ModifyPointLight(i, ChVector3f(-4, 5, 6), ChColor(0.7f, 0.6f, 0.5f), 25.f, false); }},
        {"directional", [](SceneType& s) { return s.AddDirectionalLight(ChColor(0.1f, 0.2f, 0.3f), 0.4f, 0.5f); },
         [](SceneType& s) { return s.AddDirectionalLight(ChColor(0.9f, 0.8f, 0.7f), 1.1f, 2.2f); },
         [](SceneType& s, unsigned int i) { s.ModifyDirectionalLight(i, ChColor(0.9f, 0.8f, 0.7f), 1.1f, 2.2f); }},
        {"spot", [](SceneType& s) { return s.AddSpotLight(ChVector3f(1, 1, 1), ChColor(0.2f, 0.2f, 0.2f), 8.f, ChVector3f(0, 0, -1), 0.3f, 0.6f, true); },
         [](SceneType& s) { return s.AddSpotLight(ChVector3f(2, -3, 4), ChColor(0.5f, 0.4f, 0.3f), 30.f, ChVector3f(1, 0, 0), 0.15f, 0.9f, false); },
         [](SceneType& s, unsigned int i) { s.ModifySpotLight(i, ChVector3f(2, -3, 4), ChColor(0.5f, 0.4f, 0.3f), 30.f, ChVector3f(1, 0, 0), 0.15f, 0.9f, false); }},
        {"rectangle", [](SceneType& s) { return s.AddRectangleLight(ChVector3f(0, 0, 5), ChColor(0.3f, 0.3f, 0.3f), 12.f, ChVector3f(1, 0, 0), ChVector3f(0, 1, 0), true); },
         [](SceneType& s) { return s.AddRectangleLight(ChVector3f(-1, 2, 9), ChColor(0.8f, 0.1f, 0.6f), 40.f, ChVector3f(0, 2, 0), ChVector3f(0, 0, 3), false); },
         [](SceneType& s, unsigned int i) { s.ModifyRectangleLight(i, ChVector3f(-1, 2, 9), ChColor(0.8f, 0.1f, 0.6f), 40.f, ChVector3f(0, 2, 0), ChVector3f(0, 0, 3), false); }},
        {"disk", [](SceneType& s) { return s.AddDiskLight(ChVector3f(3, 3, 3), ChColor(0.4f, 0.4f, 0.4f), 15.f, ChVector3f(0, 0, -1), 0.5f, true); },
         [](SceneType& s) { return s.AddDiskLight(ChVector3f(-2, 7, 1), ChColor(0.2f, 0.9f, 0.5f), 33.f, ChVector3f(0, 1, 0), 1.75f, false); },
         [](SceneType& s, unsigned int i) { s.ModifyDiskLight(i, ChVector3f(-2, 7, 1), ChColor(0.2f, 0.9f, 0.5f), 33.f, ChVector3f(0, 1, 0), 1.75f, false); }},
    };

    for (const auto& c : cases) {
        SceneType scene;
        const unsigned int a = c.add_a(scene);
        const unsigned int b = c.add_b(scene);
        ASSERT_NE(a, b) << c.name << ": the two lights must be distinct entries";

        // Sanity: A and B differ before the modify, or the comparison afterwards proves nothing.
        const auto before = scene.GetLights();
        ASSERT_EQ(before.size(), 2u) << c.name;
        ASSERT_NE(0, std::memcmp(&before[a], &before[b], sizeof(before[0]))) << c.name << ": the two parameter sets produce identical lights, so this case cannot detect a "
                                                                             << "modify path that writes nothing";

        c.modify_a_to_b(scene, a);

        const auto after = scene.GetLights();
        ASSERT_EQ(after.size(), 2u) << c.name << ": modifying a light must not add or remove one";
        EXPECT_TRUE(SameLight(after[a], after[b], c.name));
    }
}

TEST(ChSensorSceneLights, modify_out_of_range_is_ignored) {
    SceneType scene;
    const unsigned int id = scene.AddPointLight(ChVector3f(1, 2, 3), ChColor(0.5f, 0.5f, 0.5f), 10.f, true);
    const auto before = scene.GetLights();

    // Out of range must not write past the end, and must not resize the list.
    scene.ModifyPointLight(id + 7, ChVector3f(9, 9, 9), ChColor(1, 1, 1), 99.f, false);

    const auto after = scene.GetLights();
    ASSERT_EQ(after.size(), before.size());
    EXPECT_EQ(0, std::memcmp(&after[id], &before[id], sizeof(after[0]))) << "an out-of-range Modify*Light changed an existing light";
}

#endif  // any render backend
