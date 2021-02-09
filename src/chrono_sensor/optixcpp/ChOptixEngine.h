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

#include "chrono_sensor/ChOptixSensor.h"
#include "chrono_sensor/scene/ChScene.h"

#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_optix
/// @{

/// Optix Engine that is responsible for managing all render-based sensors.
/// The is the interface between the dynamic simulation and the rendering/ray tracing.
/// All ray tracing code is setup and managed via this class.
class CH_SENSOR_API ChOptixEngine {
    /// Class for managing dynamic meshes such as SCM terrain
    class DynamicMesh {
      public:
        /// Class constructor
        /// @param mesh The Chrono core mesh object that reflects this dynamic mesh geometry
        /// @param tri_group The triangle group that refects the optix version of the geometry
        /// @param vertex_buffer Direct handle to the vertex buffer for modifying values in the optix scene
        /// @param normal_buffer Direct handle to the normal buffer for modifying values in the optix scene
        DynamicMesh(std::shared_ptr<geometry::ChTriangleMeshConnected> mesh,
                    optix::GeometryGroup tri_group,
                    optix::Buffer vertex_buffer,
                    optix::Buffer normal_buffer) {
            m_mesh = mesh;
            m_vertex_buffer = vertex_buffer;
            m_normal_buffer = normal_buffer;
            m_tri_group = tri_group;

            int device_id = m_normal_buffer->getContext()->getEnabledDevices()[0];
            m_size = (int)m_mesh->getCoordsVertices().size();
            m_normal_buffer_device_ptr = m_normal_buffer->getDevicePointer(device_id);
            m_vertex_buffer_device_ptr = m_vertex_buffer->getDevicePointer(device_id);
            m_host_normal_buffer = std::vector<float>(m_size * 3);
            m_host_vertex_buffer = std::vector<float>(m_size * 3);
            for (int i = 0; i < m_size; i++) {
                m_host_vertex_buffer[3 * i] = (float)mesh->getCoordsVertices()[i].x();
                m_host_vertex_buffer[3 * i + 1] = (float)mesh->getCoordsVertices()[i].y();
                m_host_vertex_buffer[3 * i + 2] = (float)mesh->getCoordsVertices()[i].z();

                m_host_normal_buffer[3 * i] = (float)mesh->getCoordsNormals()[i].x();
                m_host_normal_buffer[3 * i + 1] = (float)mesh->getCoordsNormals()[i].y();
                m_host_normal_buffer[3 * i + 2] = (float)mesh->getCoordsNormals()[i].z();
            }
        }

      public:
        std::shared_ptr<geometry::ChTriangleMeshConnected> getMesh() { return m_mesh; }
        optix::Buffer getNormalBuffer() { return m_normal_buffer; }
        optix::Buffer getVertexBuffer() { return m_vertex_buffer; }
        float* host_normalBuffer_ptr() { return m_host_normal_buffer.data(); }
        float* host_vertexBuffer_ptr() { return m_host_vertex_buffer.data(); }
        void* device_normalBuffer_ptr() { return m_normal_buffer_device_ptr; }
        void* device_vertexBuffer_ptr() { return m_vertex_buffer_device_ptr; }
        int getSize() { return m_size; }
        optix::GeometryGroup getTriGroup() { return m_tri_group; }

      private:
        optix::Buffer m_vertex_buffer;
        optix::Buffer m_normal_buffer;
        void* m_normal_buffer_device_ptr;
        void* m_vertex_buffer_device_ptr;
        int m_size;
        std::vector<float> m_host_normal_buffer;
        std::vector<float> m_host_vertex_buffer;
        optix::GeometryGroup m_tri_group;
        std::shared_ptr<geometry::ChTriangleMeshConnected> m_mesh;
    };

  public:
    /// Class constructor
    /// @param sys Pointer to the ChSystem that defines the simulation
    /// @param device_id The id of the GPU which this engine is running on
    /// @param max_scene_reflections The maximum number of ray recursions that should be allowed by this engine
    /// @param verbose Sets verbose level for the engine
    /// @param max_keyframes The number of keyframes to used for motion blur. Defaults to minimum of 2 which is smallest
    /// that enables interpolation.
    ChOptixEngine(ChSystem* sys,
                  int device_id,
                  int max_scene_reflections = 9,
                  bool verbose = false,
                  int max_keyframes = 2);

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

    /// adds a static triangle mesh to the scene that is external to Chrono. This is a way to add
    /// complex environment that includes trees, etc
    /// @param frames The reference frames that encode location, orientation, and scale. One for each object that should
    /// be added to the environment
    /// @param mesh The mesh that should be added at each reference frame.
    void AddInstancedStaticSceneMeshes(std::vector<ChFrame<>>& frames, std::shared_ptr<ChTriangleMeshShape> mesh);

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
    optix::Context GetContext() { return m_context; }

    /// Gives the user access to the list of sensors being managed by this engine.
    /// @return the vector of Chrono sensors
    std::vector<std::shared_ptr<ChOptixSensor>> GetSensor() { return m_assignedSensor; }

  private:
    void Start();  ///< start the render thread
    void Stop();   ///< stop the render thread

    bool m_verbose;     ///< whether the context should print errors and warnings
    void Initialize();  ///< intialize function for the optix engine. This is what gets everything up and running
    void Process();     ///< function that processes sensor added to its queue

    void UpdateCameraTransforms();  ///< updates all of the camera position and orientations
    void UpdateBodyTransforms();    ///< updates all the transforms associated with the bodies
    void PackKeyFrames();           ///< updates all of the bodies and places their transforms into a list of keyframes.
    void UpdateDynamicMeshes();     ///< updates the dynamic meshes in the scene
    void UpdateSceneDescription(
        std::shared_ptr<ChScene> scene);  ///< updates the scene characteristics such as lights, background, etc

    optix::Material GetDefaultMaterial();  ///< returns a default material, creating it if it does not yet exist
    optix::Material CreateMaterial(
        std::shared_ptr<ChVisualMaterial> chmat);               ///< creates a new material based on a ChVisualMaterial
    optix::TextureSampler CreateTexture(std::string filename);  ///< creates a texture from file
    optix::TextureSampler CreateTexture();                      ///< creates a default texture

    optix::Geometry
    GetOptixBoxGeometry();  ///< gets the box geometry and creates one if the single version doesn't yet exist
    optix::Geometry GetOptixSphereGeometry();    ///< get the sphere geometry and creates it if it does not yet exist
    optix::Geometry GetOptixCylinderGeometry();  ///< get the cylinder geometry and creates it if it does not yet exist

    /// Creates an optix box visualization object from a Chrono box shape
    void boxVisualization(std::shared_ptr<ChBoxShape> box_shape,
                          std::shared_ptr<ChVisualization> visual_asset,
                          optix::Group asset_group);
    /// Creates an optix sphere visualization object from a Chrono sphere shape
    void sphereVisualization(std::shared_ptr<ChSphereShape> sphere_shape,
                             std::shared_ptr<ChVisualization> visual_asset,
                             optix::Group asset_group);

    /// Creates an optix cylinder visualization from a Chrono cylinder shape
    void cylinderVisualization(std::shared_ptr<ChCylinderShape> cylinder_shape,
                               std::shared_ptr<ChVisualization> visual_asset,
                               optix::Group asset_group);

    /// Creates a static optix triangle mesh visualization from a Chrono triangle mesh shape
    void staticTrimeshVisualization(std::shared_ptr<ChTriangleMeshShape> trimesh_shape,
                                    std::shared_ptr<ChVisualization> visual_asset,
                                    optix::Group asset_group);

    /// Creates a dynamic optix triangle mesh visualization from a Chrono triangle mesh shape
    void dynamicTrimeshVisualization(std::shared_ptr<ChTriangleMeshShape> trimesh_shape,
                                     std::shared_ptr<ChVisualization> visual_asset,
                                     optix::Group asset_group);

    std::thread m_thread;                                  ///< worker thread for performing render operations
    std::vector<std::shared_ptr<ChSensor>> m_renderQueue;  ///< list of sensors for the engine to manage to process

    std::deque<std::tuple<float, std::vector<std::vector<float>>>>
        m_keyframes;  ///< queue of keyframes (queue of keyframes, each keyframe has a time and N bodies (vector), each
    ///< body has 12 floats that define transform)
    std::deque<std::tuple<float, std::vector<std::vector<float>>>>
        m_camera_keyframes;      ///< queue of keyframes (queue of keyframes, each keyframe has a time and N bodies
                                 ///< (vector), each body has 16 floats that define transform)
    int m_max_keyframes_needed;  ///< the maximum number of keyframes that should be stored

    std::vector<float> m_internal_keyframes;  ///< holder of data for internal keyframe data for optix.

    // noise buffer that is available to all programs using this context
    bool m_noise_initialized = false;        ///< toggling whether to initialize the noise buffer
    long unsigned int m_num_noise_vals = 0;  ///< number of noise values needed by largest sensor

    // mutex and condition variables
    std::mutex m_renderQueueMutex;            ///< mutex for protecting the render queue
    std::condition_variable m_renderQueueCV;  ///< condition variable for notifying the worker thread it should process
                                              ///< the filters from the queue
    bool m_terminate = false;                 ///< worker thread stop variable
    bool m_started = false;                   ///< worker thread start variable

    // objects that should be instanced or reused
    optix::Geometry m_box_geometry;      ///< box geometry that all boxes in the scenes will share
    optix::Acceleration m_box_accel;     ///< acceleration object that all box geometries in the scene will share
    optix::Geometry m_sphere_geometry;   ///< sphere geometry that all sphere in the scenes will share
    optix::Acceleration m_sphere_accel;  ///< acceleration object that all sphere geometries in the scene will share
    optix::Geometry m_cyl_geometry;      ///< sphere geometry that all sphere in the scenes will share
    optix::Acceleration m_cyl_accel;     ///< acceleration object that all sphere geometries in the scene will share

    // All RT Program should only be made once and then reused
    optix::Program m_camera_shader;    ///< camera material shader
    optix::Program m_lidar_shader;     ///< lidar material shader
    optix::Program m_shadow_shader;    ///< shadow shader
    optix::Program m_camera_miss;      ///< camera miss shader
    optix::Program m_lidar_miss;       ///< lidar miss shader
    optix::Program m_box_int;          ///< box intersection function
    optix::Program m_box_bounds;       ///< box bounding function
    optix::Program m_sphere_int;       ///< sphere intersection function
    optix::Program m_sphere_bounds;    ///< sphere bounds function
    optix::Program m_cylinder_int;     ///< cylinder intersection function
    optix::Program m_cylinder_bounds;  ///< cylinder intersection function
    optix::Program m_mesh_att;         ///< mesh attributes function
    optix::Program m_exception_prog;   ///< exception program

    // default material objects
    optix::TextureSampler m_empty_tex_sampler;  ///< default texture sampler
    optix::Material m_default_material;         ///< default material

    optix::Buffer m_light_buffer;  ///< scene light buffer

    // information that belongs to the rendering concept of this engine
    optix::Context m_context;                                      ///< the optix context we use for everything
    optix::Group m_root;                                           ///< root node of the optix scene
    std::vector<std::shared_ptr<ChOptixSensor>> m_assignedSensor;  ///< list of sensor this engine is responsible for
    ChSystem* m_system;                                            ///< the chrono system that defines the scene
    std::vector<std::pair<std::shared_ptr<ChBody>, optix::Transform>>
        m_bodies;  ///< matching bodies to transforms for quicker upgates of the scene
    std::vector<std::shared_ptr<DynamicMesh>> m_dynamicMeshes;  ///< list of dynamic meshes for quick updating
    unsigned int m_deviceId;                                             ///< ID of the GPU the context should be attached to
    int m_recursions;                                           ///< number of allowable ray tracing recursions in optix
};

/// @} sensor_optix

}  // namespace sensor
}  // namespace chrono

#endif
