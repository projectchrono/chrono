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

// API include
#include "chrono_sensor/ChApiSensor.h"

#include "chrono/physics/ChSystem.h"

#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/optixcpp/ChOptixEngine.h"
#include "chrono_sensor/ChDynamicsManager.h"
#include "chrono_sensor/scene/ChScene.h"

#include <fstream>
#include <sstream>

namespace chrono {
namespace sensor {

/// @addtogroup sensor
/// @{

/// class for managing sensors. This is the Sensor system class.

class CH_SENSOR_API ChSensorManager {
  public:
    /// Class constructor
    /// @param chrono_system The chrono system with which the sensor manager is associated. Used for time management.
    /// created.
    ChSensorManager(ChSystem* chrono_system);

    /// Class destructor
    ~ChSensorManager();

    /// Update function that will prompt the manager to update its sensors if they need to be updated according to the
    /// current time of the chrono simulation
    void Update();  // update the sensor manager -> renders and updates as it needs

    /// Add a sensor to the manager
    /// @param sensor The sensor that should be added to the system
    void AddSensor(std::shared_ptr<ChSensor> sensor);

    /// Get the list of sensors for which this manager is responsible
    /// @return The list of sensors for which the manager is responsible and updates
    std::vector<std::shared_ptr<ChSensor>> GetSensorList() { return m_sensor_list; }

    /// Set the list of devices (GPUs) that should be used for rendering
    /// @param device_ids List of IDs corresponding to the devices (GPUs) that should be used.
    void SetDeviceList(std::vector<unsigned int> device_ids);

    /// Get the list of devices that are intended for use
    /// @return List of device IDs that the manager will try to use when rendering.
    std::vector<unsigned int> GetDeviceList();

    /// Get the number of engines the manager is currently using
    /// @return An integer number of OptiX engines
    int GetNumEngines() { return (int)m_engines.size(); }

    /// Get a pointer to the engine based on the id of the engine.
    /// @param context_id The ID of the engine to be returned
    /// @return A shared pointer to an OptiX engine the manager is using
    std::shared_ptr<ChOptixEngine> GetEngine(int context_id);

    /// Add many environment meshes that bypass the requirement to have them in the Chrono system.
    /// This adds meshes that only exist in OptiX. Meshes will be removed upon call to ReconstructScenes().
    void AddInstancedStaticSceneMeshes(std::vector<ChFrame<>>& frames, std::shared_ptr<ChTriangleMeshShape> mesh);

    /// Calls on the sensor manager to rebuild the scene, translating all objects from the Chrono system into their
    /// appropriate optix objects.
    void ReconstructScenes();

    /// Get the maximum number of allowed OptiX Engines for the manager.
    /// @return An integer specifying the maximum number of engines the manager is allowed to create.
    int GetMaxEngines() { return m_allowable_groups; }

    /// Set the maximum number of allowable optix engines. The manager will spawn up to this number of optix engines
    /// (separate threads for rendering) based on the update rate of the sensors. Sensors with similar update rates will
    /// be grouped on the same engine to reduce the number of scene updates that are required as this is a major
    /// bottleneck in the multithreading paradigm of the render engine.
    /// @param num_groups The maximum number of optix engines the manager is allowed to create.
    void SetMaxEngines(int num_groups);

    /// Set the max number of keyframes to use when rendering motion blur
    /// @param size The number of keyframes
    void SetKeyframeSize(int size);

    /// Get the max number of keyframes to use when rendering motion blur
    /// @return the number of keyframes being used
    int GetKeyframeSize() { return m_num_keyframes; }

    /// Set the max key frame size from a simulation time step and the largest collection window
    /// @param timestep The estimated timestep to be used in the simulation
    /// @param max_collection_window The largest collection window of any rendering sensor (camera or lidar)
    void SetKeyframeSizeFromTimeStep(float min_timestep, float max_collection_window);

    /// Set the number of recursions for ray tracing
    /// @param rec The max number of recursions allowed in ray tracing
    void SetRayRecursions(int rec);

    /// Get the number of recursions used in ray tracing
    /// @return The max number of recursions used in ray tracing
    int GetRayRecursions() { return m_optix_reflections; }

    /// Set if the sensor framework should print all info
    /// @param verbose Whether the framework should print info
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// Get the verbose setting
    /// @return The verbose setting
    bool GetVerbose() { return m_verbose; }

    /// Public pointer to the scene. This is used to specify additional componenets include lights, background colors,
    /// etc
    std::shared_ptr<ChScene> scene;

  private:
    bool m_verbose;           ///< Whether we should print messages and warnings
    int m_optix_reflections;  ///< Maximum number of ray tracing recursions
    int m_num_keyframes;      ///< number of keyframes to use

    // class variables
    ChSystem* m_system;                                     ///< Chrono system the manager is attached to
    std::vector<std::shared_ptr<ChOptixEngine>> m_engines;  ///< The optix engine(s) used for rendered sensors
    std::shared_ptr<ChDynamicsManager> m_dynamics_manager;  ///< Container for updating dynamic sensors

    int m_allowable_groups = 1;  ///< Default maximum number of allowable engines

    std::vector<unsigned int> m_device_list;  ///< List of device IDs to use in rendering.

    std::vector<std::shared_ptr<ChSensor>> m_sensor_list;  ///< List of all sensors for which the manager is responsible
    std::vector<std::shared_ptr<ChSensor>>
        m_dynamic_sensor;  ///< List of dynamic sensor for which the manager is responsible
    std::vector<std::shared_ptr<ChSensor>>
        m_render_sensor;  ///< List of optix based sensor for which the manager is responsible.
};

/// @} sensor

}  // namespace sensor
}  // namespace chrono

#endif
