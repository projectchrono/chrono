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
// How two lights are compared depends on the backend's light type. Only a trivially copyable
// struct can be compared by its object representation; the Vulkan light holds a std::string and
// the Metal one holds ChVector3f and ChColor, so both get a field-wise comparison instead.
//
// Runs against whichever backend the build selected: the scene classes are different types with
// different light structs, and the test is written against neither.
//
// =============================================================================

#include "gtest/gtest.h"

#include <cstring>
#include <type_traits>
#include <functional>
#include <vector>

#include "chrono/core/ChVector3.h"
#include "chrono/assets/ChColor.h"

#include "chrono_sensor/ChConfigSensor.h"

#ifdef CHRONO_HAS_OPTIX
    #include "chrono_sensor/optix/ChOptixScene.h"
#endif
#ifdef CHRONO_HAS_METAL_RT
    #include "chrono_sensor/metal/ChMetalRTScene.h"
#endif
#ifdef CHRONO_HAS_VULKAN_RT
    #include "chrono_sensor/vulkan/ChVulkanRTScene.h"
#endif

using namespace chrono;

#if defined(CHRONO_HAS_OPTIX) || defined(CHRONO_HAS_METAL_RT) || defined(CHRONO_HAS_VULKAN_RT)

namespace {

// Two distinct parameter sets per light type. They differ in every argument, so a modify path that
// drops one is caught rather than passing because the two happened to agree there.
template <typename SceneType>
struct LightPair {
    const char* name;
    std::function<unsigned int(SceneType&)> add_a;
    std::function<unsigned int(SceneType&)> add_b;
    std::function<void(SceneType&, unsigned int)> modify_a_to_b;
};

// Comparing two lights. Byte comparison is only valid for a trivially copyable type, which the
// OptiX light struct is and the other two are not: the Vulkan one holds a std::string, the Metal
// one holds ChVector3f and ChColor. Those get a field-wise comparison here rather than in the
// backend headers, since comparing lights is something only this test needs.
template <typename L>
bool LightsEqual(const L& lhs, const L& rhs) {
    static_assert(std::is_trivially_copyable_v<L>, "add a LightsEqual overload for this light type");
    return std::memcmp(&lhs, &rhs, sizeof(L)) == 0;
}

    #ifdef CHRONO_HAS_VULKAN_RT
bool LightsEqual(const chrono::sensor::ChVulkanRTLight& a, const chrono::sensor::ChVulkanRTLight& b) {
    return a.type == b.type && a.pos == b.pos && a.dir == b.dir && a.color == b.color && a.range == b.range && a.angle == b.angle && a.const_color == b.const_color &&
           a.atten_scale == b.atten_scale && a.angle_falloff_start == b.angle_falloff_start && a.angle_atten_rate == b.angle_atten_rate && a.length_vec == b.length_vec &&
           a.width_vec == b.width_vec && a.radius == b.radius && a.area == b.area && a.texture == b.texture;
}
    #endif

    #ifdef CHRONO_HAS_METAL_RT
bool LightsEqual(const chrono::sensor::MetalSceneLight& a, const chrono::sensor::MetalSceneLight& b) {
    return a.pos == b.pos && a.range == b.range && a.color.R == b.color.R && a.color.G == b.color.G && a.color.B == b.color.B && a.type == b.type && a.dir == b.dir &&
           a.cosOuter == b.cosOuter && a.cosInner == b.cosInner && a.p0 == b.p0 && a.const_color == b.const_color;
}
    #endif

template <typename L>
::testing::AssertionResult SameLight(const L& lhs, const L& rhs, const char* which) {
    if (LightsEqual(lhs, rhs))
        return ::testing::AssertionSuccess();
    return ::testing::AssertionFailure() << which << ": Modify*Light did not reproduce what Add*Light builds from the same "
                                         << "parameters, so at least one field is not written by the modify path";
}

}  // namespace

template <typename SceneType>
void CheckModifyReproducesAdd() {
    const std::vector<LightPair<SceneType>> cases = {
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
        ASSERT_FALSE(LightsEqual(before[a], before[b])) << c.name << ": the two parameter sets produce identical lights, so this case cannot detect a "
                                                        << "modify path that writes nothing";

        c.modify_a_to_b(scene, a);

        const auto after = scene.GetLights();
        ASSERT_EQ(after.size(), 2u) << c.name << ": modifying a light must not add or remove one";
        EXPECT_TRUE(SameLight(after[a], after[b], c.name));
    }
}

template <typename SceneType>
void CheckModifyOutOfRangeIsIgnored() {
    SceneType scene;
    const unsigned int id = scene.AddPointLight(ChVector3f(1, 2, 3), ChColor(0.5f, 0.5f, 0.5f), 10.f, true);
    const auto before = scene.GetLights();

    // Out of range must not write past the end, and must not resize the list.
    scene.ModifyPointLight(id + 7, ChVector3f(9, 9, 9), ChColor(1, 1, 1), 99.f, false);

    const auto after = scene.GetLights();
    ASSERT_EQ(after.size(), before.size());
    EXPECT_TRUE(LightsEqual(after[id], before[id])) << "an out-of-range Modify*Light changed an existing light";
}

// Each check runs for every backend the build enabled, not just the first. SCOPED_TRACE names the
// backend so a failure in a multi-backend build says which one.
TEST(ChSensorSceneLights, modify_reproduces_add_for_every_light_type) {
    #ifdef CHRONO_HAS_OPTIX
    {
        SCOPED_TRACE("OptiX");
        CheckModifyReproducesAdd<chrono::sensor::ChOptixScene>();
    }
    #endif
    #ifdef CHRONO_HAS_METAL_RT
    {
        SCOPED_TRACE("Metal RT");
        CheckModifyReproducesAdd<chrono::sensor::ChMetalRTScene>();
    }
    #endif
    #ifdef CHRONO_HAS_VULKAN_RT
    {
        SCOPED_TRACE("Vulkan RT");
        CheckModifyReproducesAdd<chrono::sensor::ChVulkanRTScene>();
    }
    #endif
}

TEST(ChSensorSceneLights, modify_out_of_range_is_ignored) {
    #ifdef CHRONO_HAS_OPTIX
    {
        SCOPED_TRACE("OptiX");
        CheckModifyOutOfRangeIsIgnored<chrono::sensor::ChOptixScene>();
    }
    #endif
    #ifdef CHRONO_HAS_METAL_RT
    {
        SCOPED_TRACE("Metal RT");
        CheckModifyOutOfRangeIsIgnored<chrono::sensor::ChMetalRTScene>();
    }
    #endif
    #ifdef CHRONO_HAS_VULKAN_RT
    {
        SCOPED_TRACE("Vulkan RT");
        CheckModifyOutOfRangeIsIgnored<chrono::sensor::ChVulkanRTScene>();
    }
    #endif
}

#endif  // any render backend
