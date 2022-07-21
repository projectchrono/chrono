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
// Authors: Asher Elmquist, Han Wang
// =============================================================================
//
// OptiX rendering engine for processing jobs for sensing. Jobs are defined on
// each sensor as a graph. Recommended to use one engine per GPU to mitigate
// OptiX blocking launch calls
//
// =============================================================================

#ifndef CHOPTIXENGINE_H
#define CHOPTIXENGINE_H

#include "chrono_sensor/ChApiSensor.h"

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>

#include <optix.h>

#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/optix/scene/ChScene.h"
#include "chrono_sensor/optix/ChOptixGeometry.h"
#include "chrono_sensor/optix/ChOptixPipeline.h"
#include "chrono_sensor/optix/ChFilterOptixRender.h"

#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_optix
/// @{

struct RenderThread {
    std::thread thread;          ///< thread for performing the rendering operation
    std::mutex mutex;            ///< mutex for protecting the render operations
    std::condition_variable cv;  ///< condition variable for notifying the render threads
    bool terminate;
    bool start;  ///< for signalling to the worker to start
    bool done;   ///< for signalling to any parents we are complete
};

/// Optix Engine that is responsible for managing all render-based sensors.
/// The is the interface between the dynamic simulation and the rendering/ray tracing.
/// All ray tracing code is setup and managed via this class.
class CH_SENSOR_API ChOptixEngine {
  public:
    /// Class constructor
    /// @param sys Pointer to the ChSystem that defines the simulation
    /// @param device_id The id of the GPU which this engine is running on
    /// @param max_scene_reflections The maximum number of ray recursions that should be allowed by this engine
    /// @param verbose Sets verbose level for the engine
    ChOptixEngine(ChSystem* sys, int device_id, int max_scene_reflections = 9, bool verbose = false);

    /// Class destructor
    ~ChOptixEngine();

    /// Add a sensor for this engine to manage and update
    /// @param sensor A shared pointer to an Optix-based sensor
    void AssignSensor(std::shared_ptr<ChOptixSensor> sensor);

    /// Updates the sensors if they need to be updated based on simulation time and last update time.
    /// @param scene The scene that should be rendered with.
    void UpdateSensors(std::shared_ptr<ChScene> scene);

    /// Tells the optix manager to construct the scene from scratch, translating all objects
    /// from Chrono to Optix
    void ConstructScene();

    /// Way to query the device ID on which the engine is running. CANNOT BE MODIFIED since the engine will have been
    /// already constructed
    /// @return the GPU ID
    int GetDevice() { return m_deviceId; }

    /// Query the number of sensors for which this engine is responsible.
    /// @return The number of sensors managed by this engine
    int GetNumSensor() { return (int)m_assignedSensor.size(); }

    /// Gives the user the optix context. This should be used with caution and should not be required for typical
    /// simulations.
    /// @return The Optix context
    // optix::Context GetContext() { return m_context; }

    /// Gives the user access to the list of sensors being managed by this engine.
    /// @return the vector of Chrono sensors
    std::vector<std::shared_ptr<ChOptixSensor>> GetSensor() { return m_assignedSensor; }

  private:
    void Start();           ///< start the render thread
    void StopAllThreads();  ///< stop the scene and render threads, remove all asigned sensors

    bool m_verbose;     ///< whether the context should print errors and warnings
    void Initialize();  ///< intialize function for the optix engine. This is what gets everything up and running
    // void Process();     ///< function that processes sensor added to its queue
    // void Process2();
    void RenderProcess(
        RenderThread& tself,
        std::shared_ptr<ChOptixSensor> sensor);  ///< render processing function for rendering in separate threads
    void SceneProcess(RenderThread& tself);  ///< scene processing function for building the scene in separate thread

    void UpdateCameraTransforms(
        std::vector<int>& to_be_updated,
        std::shared_ptr<ChScene> scene);  ///< updates all of the camera position and orientations
    void UpdateDeformableMeshes();        ///< updates the dynamic meshes in the scene
    void UpdateSceneDescription(
        std::shared_ptr<ChScene> scene);  ///< updates the scene characteristics such as lights, background, etc

    /// Creates an optix box visualization object from a Chrono box shape
    void boxVisualization(std::shared_ptr<ChBody> body,
                          std::shared_ptr<ChBoxShape> box_shape,
                          ChFrame<> asset_frame);
    /// Creates an optix sphere visualization object from a Chrono sphere shape
    void sphereVisualization(std::shared_ptr<ChBody> body,
                             std::shared_ptr<ChSphereShape> sphere_shape,
                             ChFrame<> asset_frame);
    /// Creates an optix cylinder visualization object from a Chrono cylinder shape
    void cylinderVisualization(std::shared_ptr<ChBody> body,
                               std::shared_ptr<ChCylinderShape> sphere_shape,
                               ChFrame<> asset_frame);
    /// Creates an optix rigid mesh visualization object from a Chrono mesh shape
    void rigidMeshVisualization(std::shared_ptr<ChBody> body,
                                std::shared_ptr<ChTriangleMeshShape> sphere_shape,
                                ChFrame<> asset_frame);

    /// Creates an optix deformable mesh visualization object from a Chrono mesh shape
    void deformableMeshVisualization(std::shared_ptr<ChBody> body,
                                     std::shared_ptr<ChTriangleMeshShape> sphere_shape,
                                     ChFrame<> asset_frame);

    std::vector<unsigned int> m_renderQueue;  ///< list of sensor indices that need to be updated

    // mutex and condition variables
    // std::mutex m_sceneBuildMutex;               ///< mutex for protecting the scene building operation
    // std::condition_variable m_sceneBuildCV;     ///< condition variable for notifying the scene building thread
    std::deque<RenderThread> m_renderThreads;  ///< threads for rendering
    RenderThread m_sceneThread;                ///< thread for performing scene builds

    bool m_terminate = false;  ///< worker thread stop variable
    bool m_started = false;    ///< worker thread start variable

    CUdeviceptr md_lights;  ///< lights on the gpu

    // information that belongs to the rendering concept of this engine
    OptixDeviceContext m_context;  ///< the optix context we use for everything
    ContextParameters m_params;
    ContextParameters* md_params = nullptr;
    OptixTraversableHandle m_root;
    std::shared_ptr<ChOptixGeometry> m_geometry;  // manager of all geometry in the scene
    std::shared_ptr<ChOptixPipeline> m_pipeline;  // manager of all geometry in the scene

    std::vector<std::shared_ptr<ChOptixSensor>> m_assignedSensor;  ///< list of sensor this engine is responsible for
    std::vector<std::shared_ptr<ChFilterOptixRender>>
        m_assignedRenderers;  ///< list of sensor this engine is responsible for

    std::vector<ChFrame<double>> m_cameraStartFrames;
    std::vector<bool> m_cameraStartFrames_set;

    ChSystem* m_system;       ///< the chrono system that defines the scene
    unsigned int m_deviceId;  ///< ID of the GPU the context should be attached to
    int m_recursions;         ///< number of allowable ray tracing recursions in optix
};

/// @} sensor_optix

}  // namespace sensor
}  // namespace chrono

#endif
