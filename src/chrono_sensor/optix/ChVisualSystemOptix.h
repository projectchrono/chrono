// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Asher Elmquist
// =============================================================================

#ifndef CH_VISUAL_SYSTEM_OPTIX_H
#define CH_VISUAL_SYSTEM_OPTIX_H

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChPhysicsItem.h"
#include "chrono/assets/ChVisualSystem.h"

#include "chrono_sensor/sensors/ChSensor.h"
#include "chrono_sensor/optix/ChOptixEngine.h"
#include "chrono_sensor/optix/scene/ChScene.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor
/// @{

class CH_SENSOR_API ChVisualSystemOptix : public ChVisualSystem {
  public:
    ChVisualSystemOptix(ChSystem& sys);
    ~ChVisualSystemOptix();

    /// Process all visual assets in the associated ChSystem.
    virtual void BindAll() override;

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

    /// Get the maximum number of allowed OptiX Engines for the manager.
    /// @return An integer specifying the maximum number of engines the manager is allowed to create.
    int GetMaxEngines() { return m_allowable_engines; }

    /// Set the maximum number of allowable optix engines. The manager will spawn up to this number of optix engines
    /// (separate threads for rendering) based on the update rate of the sensors. Sensors with similar update rates will
    /// be grouped on the same engine to reduce the number of scene updates that are required as this is a major
    /// bottleneck in the multithreading paradigm of the render engine.
    /// @param num_groups The maximum number of optix engines the manager is allowed to create.
    void SetMaxEngines(int num_groups);

    /// Set the number of recursions for ray tracing
    /// @param rec The max number of recursions allowed in ray tracing
    void SetRayRecursions(int rec);

    /// Get the number of recursions used in ray tracing
    /// @return The max number of recursions used in ray tracing
    int GetRayRecursions() { return m_optix_reflections; }

    /// Public pointer to the scene. This is used to specify additional componenets include lights, background colors,
    /// etc
    std::shared_ptr<ChScene> scene;

  protected:
    /// Update the visualization system at the current time step.
    /// Called by the associated ChSystem.
    virtual void Update() override;

    int m_optix_reflections;  ///< Maximum number of ray tracing recursions
    int m_num_keyframes;      ///< number of keyframes to use

    std::vector<std::shared_ptr<ChOptixEngine>> m_engines;  ///< The optix engine(s) used for rendered sensors
    std::vector<std::shared_ptr<ChSensor>> m_sensor_list;  ///< List of all sensors for which the manager is responsible

    int m_allowable_engines = 1;  ///< Default maximum number of allowable engines
};

/// @} sensor

}  // namespace sensor
}  // namespace chrono

#endif
