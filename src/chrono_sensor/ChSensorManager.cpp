// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist
// =============================================================================
//
// Class for managing rendered sensors and backend render engines
//
// =============================================================================

#include <chrono>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>

#include "chrono_sensor/ChSensorManager.h"
#ifdef CHRONO_HAS_OPTIX
    #include "chrono_sensor/sensors/ChOptixSensor.h"
#endif
#ifdef CHRONO_HAS_VULKAN_RT
    #include "chrono_sensor/sensors/ChVulkanSensor.h"
#endif
#ifdef CHRONO_HAS_METAL_RT
    #include "chrono_sensor/sensors/ChMetalSensor.h"
#endif
#ifdef CHRONO_FSI_SPH
    #include "chrono_fsi/sph/ChFsiFluidSystemSPH.h"
#endif

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace sensor {

// Defined with the rest of the deterministic-seeding code at the bottom of this file.
static unsigned int AcquireRngManagerSlot();
static void ReleaseRngManagerSlot(unsigned int slot);

CH_SENSOR_API ChSensorManager::ChSensorManager(ChSystem* chrono_system) : m_verbose(false), m_debug(false), m_optix_reflections(9) {
    // Assign the Chrono system handle
    m_system = chrono_system;
    m_device_list = {0};
    m_rng_manager_id = AcquireRngManagerSlot();
#ifdef CHRONO_HAS_OPTIX
    scene = chrono_types::make_shared<ChOptixScene>();
#endif
#ifdef CHRONO_HAS_VULKAN_RT
    vulkan_scene = chrono_types::make_shared<ChVulkanRTScene>();
    #ifndef CHRONO_HAS_OPTIX
    scene = vulkan_scene;
    #endif
#endif
#ifdef CHRONO_HAS_METAL_RT
    metal_scene = chrono_types::make_shared<ChMetalRTScene>();
    #if !defined(CHRONO_HAS_OPTIX) && !defined(CHRONO_HAS_VULKAN_RT)
    scene = metal_scene;
    #endif
#endif
}

CH_SENSOR_API ChSensorManager::~ChSensorManager() {
    ReleaseRngManagerSlot(m_rng_manager_id);
}

#ifdef CHRONO_HAS_OPTIX
CH_SENSOR_API std::shared_ptr<ChOptixEngine> ChSensorManager::GetEngine(int context_id) {
    if (context_id < m_engines.size())
        return m_engines[context_id];
    cerr << "ERROR: index out of render group vector bounds\n";
    return nullptr;
}
#endif

#ifdef CHRONO_HAS_VULKAN_RT
CH_SENSOR_API std::shared_ptr<ChVulkanRTEngine> ChSensorManager::GetVulkanEngine(int context_id) {
    if (context_id < m_vulkan_engines.size())
        return m_vulkan_engines[context_id];
    cerr << "ERROR: index out of Vulkan render group vector bounds\n";
    return nullptr;
}
#endif

#ifdef CHRONO_HAS_METAL_RT
CH_SENSOR_API std::shared_ptr<ChMetalRTEngine> ChSensorManager::GetMetalEngine(int context_id) {
    if (context_id < m_metal_engines.size())
        return m_metal_engines[context_id];
    cerr << "ERROR: index out of Metal render group vector bounds\n";
    return nullptr;
}
#endif

CH_SENSOR_API void ChSensorManager::Update() {
    // update the scene
    // scene->PackFrame(m_system);
    //
    // have all the optix engines update their sensor
#ifdef CHRONO_HAS_OPTIX
    for (auto pEngine : m_engines) {
        pEngine->UpdateSensors(scene);
    }
#endif
#ifdef CHRONO_HAS_VULKAN_RT
    auto active_vulkan_scene = vulkan_scene;

    #ifdef CHRONO_HAS_OPTIX
    // Most existing Chrono Sensor demos configure lights, ambient color, and
    // background through manager->scene, which is an OptiX ChOptixScene whenever
    // OptiX is compiled in.  The Vulkan renderer uses ChVulkanRTScene, so mirror
    // those scene-level settings before rendering.  Without this bridge, Vulkan
    // receives the geometry from SyncFromSystem but silently loses the user
    // configured Point/Spot/Directional/Environment lights, which makes the
    // image dark and removes the expected cast shadows.
    if (active_vulkan_scene && scene) {
        const auto optix_lights = scene->GetLights();
        const auto optix_background = scene->GetBackground();
        const bool optix_has_render_settings = !optix_lights.empty() || scene->GetBackgroundChanged();

        if (optix_has_render_settings) {
            active_vulkan_scene->SetAmbientLight(scene->GetAmbientLight());

            auto to_chvec = [](const float3& v) { return ChVector3f(v.x, v.y, v.z); };
            auto normalize_or_default = [](const ChVector3f& v, const ChVector3f& fallback) {
                const float len = v.Length();
                return len > 1e-12f ? v / len : fallback;
            };
            auto cross = [](const ChVector3f& a, const ChVector3f& b) {
                return ChVector3f(a.y() * b.z() - a.z() * b.y(),
                                  a.z() * b.x() - a.x() * b.z(),
                                  a.x() * b.y() - a.y() * b.x());
            };
            auto atten_scale = [](float range) { return range > 0.f ? 0.01f * range * range : 1.f; };

            Background mirrored_background = optix_background;
            std::vector<ChVulkanRTLight> mirrored_lights;
            mirrored_lights.reserve(optix_lights.size());

            for (const auto& light : optix_lights) {
                ChVulkanRTLight out;
                out.type = light.light_type;
                out.pos = to_chvec(light.pos);

                switch (light.light_type) {
                case LightType::POINT_LIGHT:
                    out.color = to_chvec(light.specific.point.color);
                    out.range = light.specific.point.max_range;
                    out.const_color = light.specific.point.const_color;
                    out.atten_scale = atten_scale(out.range);
                    mirrored_lights.push_back(out);
                    break;
                case LightType::SPOT_LIGHT:
                    out.dir = normalize_or_default(to_chvec(light.specific.spot.light_dir), ChVector3f(0.f, 0.f, -1.f));
                    out.color = to_chvec(light.specific.spot.color);
                    out.range = light.specific.spot.max_range;
                    out.angle = light.specific.spot.angle_range;
                    out.const_color = light.specific.spot.const_color;
                    out.atten_scale = atten_scale(out.range);
                    if (light.specific.spot.angle_falloff_start < light.specific.spot.angle_range - 1e-6f) {
                        out.angle_falloff_start = light.specific.spot.angle_falloff_start;
                        out.angle_atten_rate = 1.f / (light.specific.spot.angle_range - light.specific.spot.angle_falloff_start);
                    } else {
                        out.angle_falloff_start = light.specific.spot.angle_range;
                        out.angle_atten_rate = -1.f;
                    }
                    mirrored_lights.push_back(out);
                    break;
                case LightType::DIRECTIONAL_LIGHT:
                    out.dir = to_chvec(light.specific.directional.light_dir);
                    out.color = to_chvec(light.specific.directional.color);
                    mirrored_lights.push_back(out);
                    break;
                case LightType::RECTANGLE_LIGHT: {
                    out.color = to_chvec(light.specific.rectangle.color);
                    out.range = light.specific.rectangle.max_range;
                    out.const_color = light.specific.rectangle.const_color;
                    out.atten_scale = atten_scale(out.range);
                    out.length_vec = to_chvec(light.specific.rectangle.length_vec);
                    out.width_vec = to_chvec(light.specific.rectangle.width_vec);
                    const ChVector3f normal = cross(out.length_vec, out.width_vec);
                    out.area = normal.Length();
                    out.dir = normalize_or_default(normal, ChVector3f(0.f, 0.f, -1.f));
                    mirrored_lights.push_back(out);
                    break;
                }
                case LightType::DISK_LIGHT:
                    out.dir = normalize_or_default(to_chvec(light.specific.disk.light_dir), ChVector3f(0.f, 0.f, -1.f));
                    out.color = to_chvec(light.specific.disk.color);
                    out.range = light.specific.disk.max_range;
                    out.const_color = light.specific.disk.const_color;
                    out.atten_scale = atten_scale(out.range);
                    out.radius = light.specific.disk.radius;
                    out.area = 3.14159265358979323846f * out.radius * out.radius;
                    mirrored_lights.push_back(out);
                    break;
                case LightType::ENVIRONMENT_LIGHT:
                    out.texture = optix_background.env_tex;
                    out.color = ChVector3f(light.specific.environment.intensity_scale,
                                           light.specific.environment.intensity_scale,
                                           light.specific.environment.intensity_scale);
                    mirrored_background.mode = BackgroundMode::ENVIRONMENT_MAP;
                    mirrored_background.env_tex = optix_background.env_tex;
                    mirrored_lights.push_back(out);
                    break;
                case LightType::AREA_LIGHT:
                default:
                    break;
                }
            }

            active_vulkan_scene->SetBackground(mirrored_background);
            if (!optix_lights.empty())
                active_vulkan_scene->SetLights(mirrored_lights);
        }
    }
    #else
    if (!active_vulkan_scene)
        active_vulkan_scene = scene;
    #endif
    for (auto pEngine : m_vulkan_engines) {
        pEngine->UpdateSensors(active_vulkan_scene);
    }
#endif
#ifdef CHRONO_HAS_METAL_RT
    for (auto pEngine : m_metal_engines) {
        pEngine->UpdateSensors(metal_scene);
    }
#endif
    // have the sensor manager update all of the non-optix sensor (IMU and GPS).
    // TODO: perhaps create a thread that takes care of this? Trade-off since IMU should require some data from EVERY
    // step
    if (m_dynamics_manager)
        m_dynamics_manager->UpdateSensors();
}

CH_SENSOR_API void ChSensorManager::SetDeviceList(std::vector<unsigned int> device_ids) {
    // set the list of devices to use
    m_device_list = device_ids;
}
CH_SENSOR_API std::vector<unsigned int> ChSensorManager::GetDeviceList() {
    // return the list of devices being used
    return m_device_list;
}

CH_SENSOR_API void ChSensorManager::ReconstructScenes() {
#ifdef CHRONO_HAS_OPTIX
    for (auto eng : m_engines) {
        eng->ConstructScene();
    }
#endif
#ifdef CHRONO_HAS_VULKAN_RT
    for (auto eng : m_vulkan_engines) {
        eng->ConstructScene();
    }
#endif
#ifdef CHRONO_HAS_METAL_RT
    for (auto eng : m_metal_engines) {
        eng->ConstructScene();
    }
#endif
}

#ifdef CHRONO_FSI_SPH
CH_SENSOR_API int ChSensorManager::AttachFsiSphSystem(std::shared_ptr<chrono::fsi::sph::ChFsiFluidSystemSPH> sys, const ChFsiSphRenderOptions& options) {
    int handle = -1;
    #ifdef CHRONO_HAS_OPTIX
    handle = scene->AddFsiSphSystem(sys, options);
    ReconstructScenes();
    #endif
    return handle;
}

CH_SENSOR_API void ChSensorManager::DetachFsiSphSystem(int handle) {
    #ifdef CHRONO_HAS_OPTIX
    scene->RemoveFsiSphSystem(handle);
    #endif
    ReconstructScenes();
}

CH_SENSOR_API void ChSensorManager::ClearFsiSphSystems() {
    #ifdef CHRONO_HAS_OPTIX
    scene->ClearFsiSphSystems();
    #endif
    ReconstructScenes();
}
#endif

CH_SENSOR_API void ChSensorManager::SetMaxEngines(int num_groups) {
    if (num_groups > 0 && num_groups < 1000) {
        m_allowable_groups = num_groups;
    }
}

CH_SENSOR_API void ChSensorManager::SetRayRecursions(int rec) {
    if (rec >= 0)
        m_optix_reflections = rec;
}

CH_SENSOR_API void ChSensorManager::AddSensor(std::shared_ptr<ChSensor> sensor) {
    // check if sensor is already in sensor list
    if (std::find(m_sensor_list.begin(), m_sensor_list.end(), sensor) != m_sensor_list.end()) {
        cerr << "WARNING: Sensor already exists in manager. Ignoring this addition\n";
        return;
    }

    if (m_verbose)
        cout << "Add sensor '" << sensor->GetName() << "'" << endl;

    // Mint this sensor's RNG identity here, and here only. This must happen before push_back and
    // before any AssignSensor call below, because both AssignSensor implementations initialize the
    // sensor's filters, and a stochastic filter reads this identity while initializing. The
    // duplicate-pointer check above has already returned, so no sensor can be given two ordinals.
    sensor->m_rng_manager_id = m_rng_manager_id;
    sensor->m_rng_sensor_ordinal = (unsigned int)m_sensor_list.size();

    // Catch filters that never went through PushFilter. Filling the protected m_filters directly is
    // not an exceptional case: seven sensor classes do it in their constructors (ChCameraSensor,
    // ChDepthCamera, ChGPSSensor, ChIMUSensor, ChNormalCamera, ChPhysCameraSensor,
    // ChTachometerSensor), and ChPhysCameraSensor puts a cuRAND-owning noise filter there. So
    // PushFilter is not a complete hook for handing out RNG identity. Every sensor passes through
    // AddSensor, so this is.
    sensor->AssignPendingRngStreamIndices();

    m_sensor_list.push_back(sensor);

#ifdef CHRONO_HAS_OPTIX
    if (auto pOptixSensor = std::dynamic_pointer_cast<ChOptixSensor>(sensor)) {
        m_render_sensor.push_back(sensor);
        /******** give each render group all sensor with same update rate *************/
        bool found_group = false;

        // add the sensor to an engine with sensor of similar update frequencies
        for (auto engine : m_engines) {
            if (!found_group && engine->GetSensor().size() > 0 && abs(engine->GetSensor()[0]->GetUpdateRate() - sensor->GetUpdateRate()) < 0.001) {
                found_group = true;

                engine->AssignSensor(pOptixSensor);
                if (m_verbose)
                    cout << "Sensor added to existing engine\n";
            }
        }

        try {
            // create new engines only when we need them
            if (!found_group) {
                if (m_engines.size() < m_allowable_groups) {
                    // limits to 2 GPUs, TODO: check if device supports CUDA
                    if (m_verbose)
                        cout << "Create new OptiX engine\n";

                    auto engine = chrono_types::make_shared<ChOptixEngine>(m_system, m_device_list[(int)m_engines.size()], m_optix_reflections, m_verbose, m_debug);

    #ifdef CHRONO_FSI_SPH
                    engine->SetFsiSphSources(&scene->GetFsiSphSources());
    #endif

                    // engine->ConstructScene();

                    engine->AssignSensor(pOptixSensor);
                    m_engines.push_back(engine);

                    if (m_verbose)
                        cout << "Number of OptiX engines: " << m_engines.size() << endl;

                } else {
                    // if we are not allowed to create additional groups, warn the user and pollute the first group
                    m_engines[0]->AssignSensor(pOptixSensor);

                    if (m_verbose)
                        cout << "Couldn't find suitable existing OptiX engine, so adding to first engine\n";
                }
            }
        } catch (std::exception& e) {
            // The message used to assert a cause ("Failed to create a ChOptixEngine"), but this block
            // also covers AssignSensor, which initializes the sensor's whole filter chain. A filter
            // that threw during initialization was therefore reported as an engine-construction
            // failure, sending the reader to the wrong place. Name what was attempted, not what is
            // guessed to have failed, and name the sensor so the report points at one object.
            cerr << "Failed while adding sensor '" << sensor->GetName() << "' to an OptiX engine "
                    "(engine construction or filter initialization), with error:\n"
                 << e.what() << endl;
            exit(1);
        }

        return;
    }
#endif

#ifdef CHRONO_HAS_VULKAN_RT
    if (auto pVulkanSensor = std::dynamic_pointer_cast<ChVulkanSensor>(sensor)) {
        m_render_sensor.push_back(sensor);
        bool found_group = false;

        for (auto engine : m_vulkan_engines) {
            if (!found_group && engine->GetSensor().size() > 0 &&
                abs(engine->GetSensor()[0]->GetUpdateRate() - sensor->GetUpdateRate()) < 0.001) {
                found_group = true;
                engine->AssignSensor(pVulkanSensor);
                if (m_verbose)
                    cout << "Sensor added to existing Vulkan RT engine\n";
            }
        }

        try {
            if (!found_group) {
                if (m_vulkan_engines.size() < m_allowable_groups) {
                    if (m_verbose)
                        cout << "Create new Vulkan RT engine\n";

                    auto engine = chrono_types::make_shared<ChVulkanRTEngine>(
                        m_system, m_device_list[(int)m_vulkan_engines.size()], m_optix_reflections, m_verbose, m_debug);
                    engine->AssignSensor(pVulkanSensor);
                    m_vulkan_engines.push_back(engine);

                    if (m_verbose)
                        cout << "Number of Vulkan RT engines: " << m_vulkan_engines.size() << endl;
                } else {
                    m_vulkan_engines[0]->AssignSensor(pVulkanSensor);
                    if (m_verbose)
                        cout << "Couldn't find suitable existing Vulkan RT engine, so adding to first engine\n";
                }
            }
        } catch (std::exception& e) {
            cerr << "Failed to create a ChVulkanRTEngine, with error:\n" << e.what() << endl;
            exit(1);
        }

        return;
    }
#endif

#ifdef CHRONO_HAS_METAL_RT
    if (auto pMetalSensor = std::dynamic_pointer_cast<ChMetalSensor>(sensor)) {
        m_render_sensor.push_back(sensor);
        bool found_group = false;

        for (auto engine : m_metal_engines) {
            if (!found_group && engine->GetSensor().size() > 0 && abs(engine->GetSensor()[0]->GetUpdateRate() - sensor->GetUpdateRate()) < 0.001) {
                found_group = true;
                engine->AssignSensor(pMetalSensor);
                if (m_verbose)
                    cout << "Sensor added to existing Metal RT engine\n";
            }
        }

        try {
            if (!found_group) {
                if (m_metal_engines.size() < m_allowable_groups) {
                    if (m_verbose)
                        cout << "Create new Metal RT engine\n";

                    auto engine = chrono_types::make_shared<ChMetalRTEngine>(m_system, m_device_list[(int)m_metal_engines.size()], m_optix_reflections, m_verbose, m_debug);
                    engine->AssignSensor(pMetalSensor);
                    m_metal_engines.push_back(engine);

                    if (m_verbose)
                        cout << "Number of Metal RT engines: " << m_metal_engines.size() << endl;
                } else {
                    m_metal_engines[0]->AssignSensor(pMetalSensor);
                    if (m_verbose)
                        cout << "Couldn't find suitable existing Metal RT engine, so adding to first engine\n";
                }
            }
        } catch (std::exception& e) {
            cerr << "Failed to create a ChMetalRTEngine, with error:\n" << e.what() << endl;
            exit(1);
        }

        return;
    }
#endif

    if (!m_dynamics_manager) {
        m_dynamics_manager = chrono_types::make_shared<ChDynamicsManager>(m_system);
    }

    // add pure dynamic sensor to dynamic manager
    m_dynamics_manager->AssignSensor(sensor);
}

// -----------------------------------------------------------------------------
// Deterministic seeding.
//
// The base seed is file-scope state rather than a member, because the sensor render filters that
// need it hold no reference back to a ChSensorManager. Threading a back-pointer through the filter
// chain to carry one global switch would be a far larger change for no additional capability.
//
// What the base seed is NOT is the seed cuRAND sees. Handing one value to several curand_init calls
// makes them identical streams, because cuRAND separates generators by subsequence and the per-pixel
// index already occupies that parameter. So the base seed is only one field of a stream key, and
// GetDeterministicSeed below derives a distinct seed per RNG buffer from the whole key.
// -----------------------------------------------------------------------------
// Guarded by a mutex rather than left as bare statics. Two reasons, and the second is the one that
// matters: the pair must be read together. A caller on one thread calling ClearRandomSeed while
// another is deriving a seed could otherwise observe s_has_fixed_seed true with s_fixed_seed already
// zeroed, producing a "reproducible" stream from a seed nobody set. The lock cost is irrelevant
// because derivation happens once per RNG buffer at initialization, not per frame or per pixel.
//
// LIFETIME AND SETUP SEMANTICS. This is process-global state, deliberately: the filters that need
// the base seed hold no reference back to a manager. It is a global determinism switch, not
// per-manager configuration, which is why the manager identity is a separate field of the stream key
// rather than a second copy of the seed. It persists for the life of the process and across manager
// construction and destruction, so in a test binary a fixed seed set by one test remains in force
// for every later test in the same process unless ClearRandomSeed is called. Set it BEFORE
// registering sensors: the seed is read when a sensor's filters initialize, inside AddSensor.
static std::mutex s_seed_mutex;
static bool s_has_fixed_seed = false;
static unsigned int s_fixed_seed = 0;

// Manager ids separate the sensor ordinals of managers that exist AT THE SAME TIME, which is the
// only case that can collide: ordinals restart at zero in every manager while the base seed is
// process-global.
//
// So the id is the lowest slot not currently in use, NOT a monotonic counter. A counter would be
// simpler and would be wrong: build a manager, render, destroy it, build another, and the second
// would get a different id and therefore different noise, so a program that renders the same scene
// twice would stop being reproducible. That is precisely what SetRandomSeed exists to provide, and
// the existing byte-exact render tests do exactly that. Reusing a freed slot keeps sequential
// construction reproducible while still giving coexisting managers distinct ids.
//
// Pointer or address-derived ids were rejected for the same reproducibility reason.
static std::mutex s_rng_manager_slot_mutex;
static std::set<unsigned int> s_rng_manager_slots_in_use;

static unsigned int AcquireRngManagerSlot() {
    std::lock_guard<std::mutex> lock(s_rng_manager_slot_mutex);
    unsigned int slot = 0;
    while (s_rng_manager_slots_in_use.count(slot))
        ++slot;
    s_rng_manager_slots_in_use.insert(slot);
    return slot;
}

static void ReleaseRngManagerSlot(unsigned int slot) {
    std::lock_guard<std::mutex> lock(s_rng_manager_slot_mutex);
    s_rng_manager_slots_in_use.erase(slot);
}

void ChSensorManager::SetRandomSeed(unsigned int seed) {
    std::lock_guard<std::mutex> lock(s_seed_mutex);
    s_fixed_seed = seed;
    s_has_fixed_seed = true;
}

void ChSensorManager::ClearRandomSeed() {
    std::lock_guard<std::mutex> lock(s_seed_mutex);
    s_has_fixed_seed = false;
    s_fixed_seed = 0;
}

bool ChSensorManager::HasRandomSeed() {
    std::lock_guard<std::mutex> lock(s_seed_mutex);
    return s_has_fixed_seed;
}

unsigned long long ChSensorManager::MakeRngStreamId(unsigned int manager_id,
                                                    unsigned int sensor_ordinal,
                                                    unsigned int filter_stream_index,
                                                    RngUsage usage) {
    // The four fields occupy disjoint bit ranges of a 64-bit word, so distinct keys give distinct
    // ids. That is the whole injectivity argument, and it is why the widths are checked rather than
    // clamped: a clamp would map two different sensors onto one stream, quietly reintroducing the
    // defect. Bounds are generous (1M sensors, 256 stochastic filters per sensor, 2G managers), so
    // hitting one means something is wrong, not that the limit was too low.
    static_assert(CH_RNG_USAGE_BITS + CH_RNG_FILTER_BITS + CH_RNG_SENSOR_BITS + CH_RNG_MANAGER_BITS == 64,
                  "RNG stream id fields must pack into exactly 64 bits");
    static_assert((unsigned int)RngUsage::Count <= (1u << CH_RNG_USAGE_BITS),
                  "RngUsage has outgrown its field; widen CH_RNG_USAGE_BITS and narrow another field");

    const unsigned int usage_id = (unsigned int)usage;
    if (usage_id >= (1u << CH_RNG_USAGE_BITS))
        throw std::runtime_error("ChSensorManager: RngUsage value out of range for the RNG stream key");
    if (filter_stream_index >= (1u << CH_RNG_FILTER_BITS))
        throw std::runtime_error("ChSensorManager: more stochastic filters on one sensor than the RNG "
                                 "stream key can distinguish (limit " +
                                 std::to_string(1u << CH_RNG_FILTER_BITS) + ")");
    if (sensor_ordinal >= (1u << CH_RNG_SENSOR_BITS))
        throw std::runtime_error("ChSensorManager: more sensors in one manager than the RNG stream key "
                                 "can distinguish (limit " +
                                 std::to_string(1u << CH_RNG_SENSOR_BITS) + ")");
    if (manager_id >= (1u << CH_RNG_MANAGER_BITS))
        throw std::runtime_error("ChSensorManager: more managers constructed than the RNG stream key can "
                                 "distinguish");

    unsigned long long id = (unsigned long long)manager_id;
    id = (id << CH_RNG_SENSOR_BITS) | (unsigned long long)sensor_ordinal;
    id = (id << CH_RNG_FILTER_BITS) | (unsigned long long)filter_stream_index;
    id = (id << CH_RNG_USAGE_BITS) | (unsigned long long)usage_id;
    return id;
}

unsigned long long ChSensorManager::GetDeterministicSeed(const std::shared_ptr<ChSensor>& sensor,
                                                         RngUsage usage,
                                                         unsigned int filter_stream_index) {
    // Validate BEFORE branching on whether a fixed seed is set, not after.
    //
    // The obvious structure is to return the clock value immediately when no fixed seed is in force,
    // since none of the identity is used on that path. That is wrong, and subtly so: it means a
    // filter with no identity, or a null sensor, is diagnosed ONLY when someone turns reproducibility
    // on. A mistake made during ordinary development would sit silent until a user set a seed, and
    // would then surface far from its cause. The identity is a precondition of the call, so it is
    // checked on every call.
    if (!sensor)
        throw std::runtime_error("ChSensorManager::GetDeterministicSeed called with a null sensor");

    const unsigned int ordinal = sensor->GetRngSensorOrdinal();
    const unsigned int manager_id = sensor->GetRngManagerId();
    if (ordinal == CH_SENSOR_UNASSIGNED_RNG_ID || manager_id == CH_SENSOR_UNASSIGNED_RNG_ID)
        throw std::runtime_error(
            "ChSensorManager::GetDeterministicSeed: sensor '" + sensor->GetName() +
            "' has no RNG identity, which means its filters were initialized before "
            "ChSensorManager::AddSensor registered it. Seeding from the unassigned sentinel would "
            "give it the same random numbers as every other unregistered sensor.");
    if (filter_stream_index == CH_SENSOR_UNASSIGNED_RNG_ID)
        throw std::runtime_error(
            "ChSensorManager::GetDeterministicSeed: a filter on sensor '" + sensor->GetName() +
            "' has no RNG stream index. Indices are assigned by ChSensor::PushFilter and, for filters "
            "a derived sensor placed in m_filters directly, by AssignPendingRngStreamIndices during "
            "AddSensor. A filter reaching here unstamped was added after registration.");

    // Reject the placeholder and anything past the declared set. MakeRngStreamId only range-checks
    // against the FIELD WIDTH, which is deliberately wider than the enum so tests can prove
    // injectivity over more purposes than exist yet. That tolerance must not reach production: a
    // buffer seeded from Unknown, or from an integer cast into an undeclared slot, has no purpose
    // identity, which is the property the enum exists to guarantee.
    if (usage == RngUsage::Unknown || (unsigned int)usage >= (unsigned int)RngUsage::Count)
        throw std::runtime_error(
            "ChSensorManager::GetDeterministicSeed: RngUsage must name a declared purpose. "
            "RngUsage::Unknown is a placeholder, not a stream identity: a new stochastic filter must "
            "add its own constant to the enum rather than borrow or cast one.");

    unsigned int base_seed;
    {
        // Read the pair under the lock so a concurrent ClearRandomSeed cannot be observed halfway.
        std::lock_guard<std::mutex> lock(s_seed_mutex);
        if (!s_has_fixed_seed) {
            // Historical default, preserved deliberately: a fresh clock reading per call, so nothing
            // is reproducible unless the user asks for it. Identity is not mixed in here because
            // there is no base seed to make it meaningful, and mixing a constant into a clock value
            // would only make the default path look more deterministic than it is.
            return (unsigned long long)std::chrono::high_resolution_clock::now().time_since_epoch().count();
        }
        base_seed = s_fixed_seed;
    }

    // Adding the base seed cannot merge two distinct stream ids: for a fixed base, x -> base + x is
    // a bijection on the 64-bit ring, so wraparound permutes the ids rather than colliding them.
    // This is why no overflow check is needed on the addition itself, only on the field widths above.
    return (unsigned long long)base_seed + MakeRngStreamId(manager_id, ordinal, filter_stream_index, usage);
}

}  // namespace sensor
}  // namespace chrono
