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
// Class for managing the all sensor updates
//
// =============================================================================

#ifndef CHSENSORMANAGER_H
#define CHSENSORMANAGER_H

#include "chrono/physics/ChSystem.h"

#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/ChDynamicsManager.h"
#include "chrono_sensor/sensors/ChSensor.h"

#ifdef CHRONO_HAS_OPTIX
    #include "chrono_sensor/optix/ChOptixEngine.h"
    #include "chrono_sensor/optix/ChOptixScene.h"
#endif
#ifdef CHRONO_HAS_VULKAN_RT
    #include "chrono_sensor/vulkan/ChVulkanRTEngine.h"
    #include "chrono_sensor/vulkan/ChVulkanRTScene.h"
#endif
#ifdef CHRONO_HAS_METAL_RT
    #include "chrono_sensor/metal/ChMetalRTEngine.h"
    #include "chrono_sensor/metal/ChMetalRTScene.h"
#endif

#ifdef CHRONO_FSI_SPH
    #include "chrono_sensor/ChFsiSphRender.h"
#endif

#include <fstream>
#include <sstream>

namespace chrono {
namespace sensor {

/// @addtogroup sensor
/// @{

/// What an independent random-number stream is used for. One constant per stream in the sensor
/// module, and part of the key that keeps those streams from coinciding.
///
/// Closed on purpose: a new stochastic filter must add its own constant here rather than borrow or
/// cast one. Borrowing does not corrupt the stream (two filters sharing a constant on one sensor are
/// still separated by their per-filter stream index, see ChFilter::GetRngStreamIndex), but it makes
/// the purpose field meaningless, which is what keeps a camera's noise independent of a lidar's.
/// GetDeterministicSeed rejects RngUsage::Unknown and any value at or past Count.
enum class RngUsage : unsigned int {
    Unknown = 0,                  ///< never seed from this; present so the enum has a defined zero
    CameraNoiseConstNormal = 1,   ///< ChFilterCameraNoiseConstNormal
    CameraNoisePixDep = 2,        ///< ChFilterCameraNoisePixDep
    LidarNoiseXYZI = 3,           ///< ChFilterLidarNoiseXYZI
    PhysCameraShotNoise = 4,      ///< ChFilterPhysCameraNoise, shot noise only; FPN keeps its own seed
    OptixCameraRaygen = 5,        ///< ChFilterOptixRender, camera
    OptixPhysCameraRaygen = 6,    ///< ChFilterOptixRender, physical camera
    OptixSegmentationRaygen = 7,  ///< ChFilterOptixRender, segmentation camera
    OptixDepthRaygen = 8,         ///< ChFilterOptixRender, depth camera
    OptixNormalRaygen = 9,        ///< ChFilterOptixRender, normal camera
    VulkanCameraRaygen = 10,      ///< ChFilterVulkanRTRender, camera
    VulkanPhysCameraRaygen = 11,  ///< ChFilterVulkanRTRender, physical camera
    MetalCameraRaygen = 12,       ///< ChFilterMetalRTRender, camera
    MetalPhysCameraRaygen = 13,   ///< ChFilterMetalRTRender, physical camera
    Count                         ///< sentinel; keep last
};

/// Bit widths of the four fields packed into an RNG stream id. They sum to exactly 64, and each
/// field is range-checked against its width, which is what makes the packing injective: distinct
/// (manager, sensor, filter, usage) tuples occupy disjoint bit ranges and so cannot collide.
///
///     [63:33] manager id      [32:13] sensor ordinal      [12:5] filter stream      [4:0] usage
static const unsigned int CH_RNG_USAGE_BITS = 5;
static const unsigned int CH_RNG_FILTER_BITS = 8;
static const unsigned int CH_RNG_SENSOR_BITS = 20;
static const unsigned int CH_RNG_MANAGER_BITS = 31;

/// class for managing sensors. This is the Sensor system class.

class CH_SENSOR_API ChSensorManager {
  public:
    /// Class constructor.
    /// The chrono system with which the sensor manager is associated is used for time management.
    ChSensorManager(ChSystem* chrono_system);

    ~ChSensorManager();

    /// Not copyable or movable. The manager owns an RNG identity slot that its destructor releases,
    /// so a copy would release the same slot twice and hand a later manager an id that is still in
    /// use, silently correlating two sensors' random streams. Copying was already meaningless before
    /// that (two managers would drive the same OptiX engines and the same sensor list), and nothing
    /// in Chrono copies one: every use site holds a std::shared_ptr<ChSensorManager>.
    ChSensorManager(const ChSensorManager&) = delete;
    ChSensorManager& operator=(const ChSensorManager&) = delete;

    /// Update the sensors as needed according to the current time of the chrono simulation.
    void Update();

    /// Add a sensor to the manager.
    /// @param sensor The sensor that should be added to the system
    void AddSensor(std::shared_ptr<ChSensor> sensor);

    /// Get the list of sensors for which this manager is responsible.
    /// @return The list of sensors for which the manager is responsible and updates
    std::vector<std::shared_ptr<ChSensor>> GetSensorList() { return m_sensor_list; }

    /// Set the list of devices (GPUs) that should be used for rendering.
    /// @param device_ids List of IDs corresponding to the devices (GPUs) that should be used.
    void SetDeviceList(std::vector<unsigned int> device_ids);

    /// Get the list of devices that are intended for use
    /// @return List of device IDs that the manager will try to use when rendering.
    std::vector<unsigned int> GetDeviceList();

#ifdef CHRONO_HAS_OPTIX
    /// Get the number of engines the manager is currently using.
    /// @return An integer number of OptiX engines
    int GetNumEngines() { return (int)m_engines.size(); }

    /// Get a pointer to the engine based on the id of the engine.
    /// @param context_id The ID of the engine to be returned
    /// @return A shared pointer to an OptiX engine the manager is using
    std::shared_ptr<ChOptixEngine> GetEngine(int context_id);
#elif defined(CHRONO_HAS_VULKAN_RT)
    /// Get the number of render engines the manager is currently using.
    /// Preserves the OptiX-era count API for Vulkan-only builds.
    int GetNumEngines() { return (int)m_vulkan_engines.size(); }
#elif defined(CHRONO_HAS_METAL_RT)
    /// Get the number of render engines the manager is currently using.
    /// Preserves the OptiX-era count API for Metal-only builds.
    int GetNumEngines() { return (int)m_metal_engines.size(); }
#endif
#ifdef CHRONO_HAS_VULKAN_RT
    /// Get the number of Vulkan RT engines the manager is currently using.
    int GetNumVulkanEngines() { return (int)m_vulkan_engines.size(); }

    /// Get a pointer to a Vulkan RT engine based on its id.
    std::shared_ptr<ChVulkanRTEngine> GetVulkanEngine(int context_id);
#endif
#ifdef CHRONO_HAS_METAL_RT
    /// Get the number of Metal RT engines the manager is currently using.
    int GetNumMetalEngines() { return (int)m_metal_engines.size(); }

    /// Get a pointer to a Metal RT engine based on its id.
    std::shared_ptr<ChMetalRTEngine> GetMetalEngine(int context_id);
#endif

    /// Calls on the sensor manager to rebuild the scene.
    /// This translates all objects from the Chrono system into their active render-backend objects.
    void ReconstructScenes();

#ifdef CHRONO_FSI_SPH
    /// Attach a Chrono::FSI::SPH system for native Sensor rendering.
    /// Returns a handle that can be used to detach the source later.
    int AttachFsiSphSystem(std::shared_ptr<chrono::fsi::sph::ChFsiFluidSystemSPH> sys, const ChFsiSphRenderOptions& options = ChFsiSphRenderOptions());

    /// Detach a previously attached Chrono::FSI::SPH render source.
    void DetachFsiSphSystem(int handle);

    /// Remove all Chrono::FSI::SPH render sources from this manager.
    void ClearFsiSphSystems();
#endif

    /// Get the maximum number of allowed render engines for the manager.
    /// @return An integer specifying the maximum number of engines the manager is allowed to create.
    int GetMaxEngines() { return m_allowable_groups; }

    /// Set the maximum number of allowable render engines.
    /// The manager will spawn up to this number of render engines based on the update
    /// rate of the sensors. Sensors with similar update rates will be grouped on the same engine to reduce the number
    /// of scene updates that are required as this is a major bottleneck in the multi-threading paradigm of the render
    /// engine.
    /// @param num_groups The maximum number of render engines the manager is allowed to create.
    void SetMaxEngines(int num_groups);

    /// Set the number of recursions for ray tracing.
    /// @param rec The max number of recursions allowed in ray tracing
    void SetRayRecursions(int rec);

    /// Get the number of recursions used in ray tracing.
    /// @return The max number of recursions used in ray tracing
    int GetRayRecursions() { return m_optix_reflections; }

    /// Enable/disable verbose output mode (default: false).
    /// @param verbose whether the framework should print info
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// Get the verbose setting.
    /// @return the verbose setting
    bool GetVerbose() { return m_verbose; }

    /// Enable/disable sensor debug mode (default: false).
    void SetDebug(bool debug) { m_debug = debug; }

    /// Fix the BASE seed from which every sensor's per-pixel random state is derived, making
    /// stochastic renders reproducible run to run.
    ///
    /// By default each sensor seeds its RNG from the wall clock, so a render that consumes random
    /// numbers (environment lighting, global illumination, the PATH integrator, motion-blur shutter
    /// sampling, sensor noise) produces different output on every run. That is the right default for
    /// a simulation but it makes byte-exact regression tests impossible, so this hook exists to pin
    /// it. Call before creating sensors: the seed is read when a sensor's render filter initializes.
    ///
    /// This is a BASE seed, not the seed handed to cuRAND. Each RNG buffer gets its own derived seed
    /// from GetDeterministicSeed. Handing this value straight to curand_init at several call sites is
    /// exactly the defect that motivated the derivation: cuRAND separates generators by subsequence,
    /// which the per-pixel index already occupies, so two buffers given the same seed produce
    /// bit-identical numbers.
    ///
    /// Deliberately static, because the seed has to reach filters that hold no reference back to the
    /// manager. It is a global determinism switch, not per-manager state, which is why the manager id
    /// is a separate field of the stream key rather than a second copy of the seed.
    /// @param seed the base value from which per-stream seeds are derived
    static void SetRandomSeed(unsigned int seed);

    /// Revert to clock-based seeding, the default.
    static void ClearRandomSeed();

    /// Whether a fixed base seed is in force.
    static bool HasRandomSeed();

    /// The seed for one specific RNG buffer, and the only supported way to seed one.
    ///
    /// With a fixed base seed, the result is a deterministic function of
    /// (base seed, manager id, sensor ordinal, filter stream index, usage), injective over that
    /// domain, so no two buffers in a simulation share a stream. With no fixed seed it is a fresh
    /// clock reading per call, preserving the historical default.
    ///
    /// @param sensor the sensor owning the buffer; must already be registered via AddSensor
    /// @param usage what the buffer is for
    /// @param filter_stream_index the calling filter's ChFilter::GetRngStreamIndex()
    /// @return the 64-bit seed used to initialize the backend RNG stream
    /// @throws std::runtime_error if the sensor is null or was never registered, if the calling filter
    /// has no stream index, if `usage` is RngUsage::Unknown or a value at or past RngUsage::Count, or
    /// if any field of the stream key exceeds its bit width. Every one of these would otherwise
    /// silently correlate streams or seed a buffer with no purpose identity, so all are checked on
    /// every call, including when no fixed seed is in force.
    static unsigned long long GetDeterministicSeed(const std::shared_ptr<ChSensor>& sensor,
                                                   RngUsage usage,
                                                   unsigned int filter_stream_index);

    /// Pack a stream key into its 64-bit id, without applying a base seed. Exposed for tests that
    /// need to prove injectivity directly rather than through a rendered image.
    ///
    /// This is the RAW packer and is deliberately more permissive than the production path: it
    /// range-checks each field against its BIT WIDTH only, so a test can prove injectivity over more
    /// purposes than the enum currently declares. It does NOT enforce that the usage names a declared
    /// purpose. Production code must call GetDeterministicSeed, which does.
    /// @throws std::runtime_error if any field exceeds its bit width.
    static unsigned long long MakeRngStreamId(unsigned int manager_id,
                                              unsigned int sensor_ordinal,
                                              unsigned int filter_stream_index,
                                              RngUsage usage);

#ifdef CHRONO_HAS_OPTIX
    /// Public pointer to the OptiX scene.
    /// This is used to specify additional components including lights, background colors, etc.
    std::shared_ptr<ChOptixScene> scene;
#elif defined(CHRONO_HAS_VULKAN_RT)
    /// Public scene pointer preserved for OptiX-compatible demos when Vulkan RT is the render backend.
    std::shared_ptr<ChVulkanRTScene> scene;
#elif defined(CHRONO_HAS_METAL_RT)
    /// Public scene pointer preserved for OptiX-compatible demos when Metal RT is the render backend.
    std::shared_ptr<ChMetalRTScene> scene;
#endif
#ifdef CHRONO_HAS_VULKAN_RT
    /// Public pointer to the Vulkan RT scene staging object.
    std::shared_ptr<ChVulkanRTScene> vulkan_scene;
#endif
#ifdef CHRONO_HAS_METAL_RT
    /// Public pointer to the Metal RT scene staging object.
    std::shared_ptr<ChMetalRTScene> metal_scene;
#endif

  private:
    bool m_verbose;           ///< enable printing of messages and warnings
    bool m_debug;             ///< enable debug options in sensors (if supported)
    int m_optix_reflections;  ///< maximum number of ray tracing recursions
    int m_num_keyframes;      ///< number of keyframes to use

    // class variables
    ChSystem* m_system;                                     ///< Chrono system the manager is attached to
    std::shared_ptr<ChDynamicsManager> m_dynamics_manager;  ///< container for updating dynamic sensors
#ifdef CHRONO_HAS_OPTIX
    std::vector<std::shared_ptr<ChOptixEngine>> m_engines;  ///< The optix engine(s) used for rendered sensors
#endif
#ifdef CHRONO_HAS_VULKAN_RT
    std::vector<std::shared_ptr<ChVulkanRTEngine>> m_vulkan_engines;  ///< Vulkan RT engine(s) used for rendered sensors
#endif
#ifdef CHRONO_HAS_METAL_RT
    std::vector<std::shared_ptr<ChMetalRTEngine>> m_metal_engines;  ///< Metal RT engine(s) used for rendered sensors
#endif

    int m_allowable_groups = 1;  ///< default maximum number of allowable engines

    std::vector<unsigned int> m_device_list;                  ///< list of device IDs to use in rendering
    std::vector<std::shared_ptr<ChSensor>> m_sensor_list;     ///< list of all sensors
    std::vector<std::shared_ptr<ChSensor>> m_dynamic_sensor;  ///< list of dynamic sensors
    std::vector<std::shared_ptr<ChSensor>> m_render_sensor;   ///< list of rendered sensors

    /// Distinguishes this manager's sensor ordinals from another manager's. Sensor ordinals restart
    /// at zero in every manager while the fixed base seed is process-global, so without this two
    /// coexisting managers would hand their first sensors the same stream.
    unsigned int m_rng_manager_id;
};

/// @} sensor

}  // namespace sensor
}  // namespace chrono

#endif
