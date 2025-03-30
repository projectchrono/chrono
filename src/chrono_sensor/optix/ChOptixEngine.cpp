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
// each sensor as a graph.
//
// =============================================================================

#define PROFILE false

#include "chrono_sensor/optix/ChOptixEngine.h"

#include <cuda.h>
#include <cuda_runtime.h>
#include <optix_stubs.h>
#include <optix_function_table_definition.h>

#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChRadarSensor.h"
#include "chrono_sensor/optix/ChOptixUtils.h"

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCapsule.h"
#include "chrono/assets/ChVisualShapeCone.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeEllipsoid.h"
#include "chrono/assets/ChVisualShapeLine.h"
#include "chrono/assets/ChVisualShapePath.h"
#include "chrono/assets/ChVisualShapeRoundedBox.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_sensor/optix/ChNVDBVolume.h"
#include <random>

#include "chrono_sensor/cuda/cuda_utils.cuh"

namespace chrono {
namespace sensor {


// using namespace optix;
ChOptixEngine::ChOptixEngine(ChSystem* sys, int device_id, int max_scene_reflections, bool verbose)
    : m_verbose(verbose), m_deviceId(device_id), m_recursions(max_scene_reflections), m_sceneThread() {
    m_sceneThread.start = false;
    m_sceneThread.terminate = false;
    m_sceneThread.done = true;  // thread is done to begin with (no work to complete)
    m_system = sys;
    Initialize();
}
ChOptixEngine::~ChOptixEngine() {
    StopAllThreads();  // if it hasn't been stopped yet, stop it ourselves
    // cleanup ChOptixGeometry and ChOptixPipeline before destroying the context
    cudaDeviceSynchronize();
    m_geometry->Cleanup();
    m_pipeline->Cleanup();
    // cleanup lights
    cudaFree(reinterpret_cast<void*>(md_params));
    // cleanup device context parameters
    cudaFree(reinterpret_cast<void*>(m_params.lights));
    optixDeviceContextDestroy(m_context);
}

void ChOptixEngine::Initialize() {
    cudaFree(0);
    OptixDeviceContext context;
    CUcontext cuCtx = 0;  // zero means take the current context, TODO: enable multigpu
    OPTIX_ERROR_CHECK(optixInit());
    OptixDeviceContextOptions options = {};
    options.logCallbackFunction = &optix_log_callback;

    if (m_verbose) {
        options.validationMode = OPTIX_DEVICE_CONTEXT_VALIDATION_MODE_ALL;
        options.logCallbackLevel = 4;
    } else {
        options.validationMode = OPTIX_DEVICE_CONTEXT_VALIDATION_MODE_OFF;
        options.logCallbackLevel = 2;
    }

    OPTIX_ERROR_CHECK(optixDeviceContextCreate(cuCtx, &options, &context));
    m_context = context;

    // defaults to no lights
    m_params.lights = {};
    m_params.arealights = {};
    m_params.arealights = 0;
    m_params.num_lights = 0;
    m_params.ambient_light_color = make_float3(0.0f, 0.0f, 0.0f);  // make_float3(0.1f, 0.1f, 0.1f);  // default value
    m_params.max_depth = m_recursions;
    m_params.scene_epsilon = 1.e-3f;    // TODO: determine a good value for this
    m_params.importance_cutoff = .01f;  /// TODO: determine a good value for this

    #ifdef USE_SENSOR_NVDB
        m_params.handle_ptr = nullptr;
    #else
    m_params.handle_ptr = 0;
    #endif  // USE_SENSOR_NVDB
    
    CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&md_params), sizeof(ContextParameters)));
    m_params.root = {};

    m_geometry = chrono_types::make_shared<ChOptixGeometry>(m_context);
    m_pipeline = chrono_types::make_shared<ChOptixPipeline>(m_context, m_recursions, m_verbose);

    // TODO: enable multigpu
}

void ChOptixEngine::AssignSensor(std::shared_ptr<ChOptixSensor> sensor) {
    // all optix sensors should be treated the same. Up to the sensor to specify a difference by using a unique launch
    // kernel and set of filters
    {
        // std::cout << "Assigning a sensor\n";
        std::unique_lock<std::mutex> lck(m_sceneThread.mutex);
        // std::cout << "going to wait for lock. done=" << m_sceneThread.done << "\n";
        while (!m_sceneThread.done) {
            m_sceneThread.cv.wait(lck);
        }
        // std::cout << "Done waiting for lock\n";

        if (std::find(m_assignedSensor.begin(), m_assignedSensor.end(), sensor) != m_assignedSensor.end()) {
            std::cerr << "WARNING: This sensor already exists in manager. Ignoring this addition\n";
            return;
        }

        m_assignedSensor.push_back(sensor);
        m_cameraStartFrames.push_back(sensor->GetParent()->GetVisualModelFrame());
        m_cameraStartFrames_set.push_back(false);
        m_pipeline->SpawnPipeline(sensor->GetPipelineType());
        // create a ChFilterOptixRender and push to front of filter list
        auto opx_filter = chrono_types::make_shared<ChFilterOptixRender>();
        unsigned int id = static_cast<unsigned int>(m_assignedSensor.size() - 1);
        opx_filter->m_optix_pipeline = m_pipeline->GetPipeline(id);
        opx_filter->m_optix_params = md_params;
        opx_filter->m_optix_sbt = m_pipeline->GetSBT(id);
        opx_filter->m_raygen_record = m_pipeline->GetRayGenRecord(id);

        // add a denoiser to the optix render filter if its a camera and global illumination is enabled
        if (auto cam = std::dynamic_pointer_cast<ChCameraSensor>(sensor)) {
            if (cam->GetUseGI()) {
                std::cout << "Sensor: " << cam->GetName() << " requested global illumination\n";
                opx_filter->m_denoiser = chrono_types::make_shared<ChOptixDenoiser>(m_context);
                //opx_filter->m_denoiser = nullptr;
            }
        }

        m_assignedRenderers.push_back(opx_filter);
        sensor->PushFilterFront(opx_filter);
        sensor->LockFilterList();

        std::shared_ptr<SensorBuffer> buffer;
        for (auto f : sensor->GetFilterList()) {
            f->Initialize(sensor, buffer);  // master thread should always be the one to initialize
        }
        // create the thread that will be in charge of this sensor (must be consistent thread for visualization reasons)
        m_renderThreads.emplace_back();
        id = static_cast<unsigned int>(m_renderThreads.size() - 1);
        m_renderThreads[id].done = true;
        m_renderThreads[id].start = false;
        m_renderThreads[id].terminate = false;
        m_renderThreads[id].thread =
            std::move(std::thread(&ChOptixEngine::RenderProcess, this, std::ref(m_renderThreads[id]), sensor));
    }
    if (!m_started) {
        Start();
    }
}

void ChOptixEngine::UpdateSensors(std::shared_ptr<ChScene> scene) {
    if (!m_params.root) {
        ConstructScene();
    }
    std::vector<int> to_be_updated;
    std::vector<int> to_be_waited_on;

    // check if any of the sensors would be collecting data right now, if so, pack a tmp start keyframe
    for (int i = 0; i < m_assignedSensor.size(); i++) {
        auto sensor = m_assignedSensor[i];
        if (m_system->GetChTime() > sensor->GetNumLaunches() / sensor->GetUpdateRate() - 1e-7 &&
            !m_cameraStartFrames_set[i]) {
            // do this once per sensor because we don't know if they will be updated at the same time
            m_geometry->UpdateBodyTransformsStart((float)m_system->GetChTime(),
                                                  (float)m_system->GetChTime() + sensor->GetCollectionWindow());
            m_cameraStartFrames[i] = sensor->GetParent()->GetVisualModelFrame();
            m_cameraStartFrames_set[i] = true;
        }
    }

    // check which sensors need to be updated this step
    for (int i = 0; i < m_assignedSensor.size(); i++) {
        auto sensor = m_assignedSensor[i];
        if (m_system->GetChTime() >
            sensor->GetNumLaunches() / sensor->GetUpdateRate() + sensor->GetCollectionWindow() - 1e-7) {
            to_be_updated.push_back(i);
        }
    }

    if (to_be_updated.size() > 0) {
        {
            // lock the render queue to make sure previous rendering has completed
            std::unique_lock<std::mutex> lck(m_sceneThread.mutex);
            while (!m_sceneThread.done) {
                m_sceneThread.cv.wait(lck);
            }
            // m_sceneThread.cv.wait(lck);  // wait for the scene thread to tell us it is done
            cudaDeviceSynchronize();  // TODO: do we need to synchronize here?

            // update the scene for the optix context
            UpdateCameraTransforms(to_be_updated, scene);

            m_geometry->UpdateBodyTransformsEnd((float)m_system->GetChTime());

            // m_renderThreads
            UpdateSceneDescription(scene);
            UpdateDeformableMeshes();

            float t = (float)m_system->GetChTime();
            // push the sensors that need updating to the render queue
            for (auto i : to_be_updated) {
                m_renderQueue.push_back(i);
                m_assignedSensor[i]->IncrementNumLaunches();
                m_assignedRenderers[i]->m_time_stamp = t;
                m_renderThreads[i].done =
                    false;  // this render thread must not be done now given we have prepped some data for it
            }
        }
        // we only notify the worker thread when there is a sensor to launch and filters to process
        m_sceneThread.done = false;     // tell the thread it is not done
        m_sceneThread.start = true;     // tell the thread it should start
        m_sceneThread.cv.notify_all();  // tell the scene thread it should proceed
    }

    // wait for any sensors whose lag times would mean the data should be available before the next ones start rendering
    // bool data_complete = false;

    for (int i = 0; i < m_assignedSensor.size(); i++) {
        auto sensor = m_assignedSensor[i];
        if (m_system->GetChTime() > (sensor->GetNumLaunches() - 1) / sensor->GetUpdateRate() +
                                        sensor->GetCollectionWindow() + sensor->GetLag() - 1e-7) {
            // wait for the sensor thread i which will notify everyone when done

            // if (!m_mainLock.owns_lock())
            //     m_mainLock.lock();  // will wait for lock to come back from scene thread - TODO: make this check the
            // render threads instead
            // m_renderThreads.cv.wait()
            // if any sensors need to have their data returned, we must wait until optix is done rendering their data
            // TODO: allow waiting for the specific sensor rather than all of them
            // bool data_complete = false;
            // while (!data_complete) {
            //     std::lock_guard<std::mutex> lck(m_sceneThread.mutex);
            //     if (m_renderQueue.empty())
            //         data_complete = true;
            // }

            // see if this specific thread is done
            std::unique_lock<std::mutex> lck(m_renderThreads[i].mutex);
            while (!m_renderThreads[i].done) {
                m_renderThreads[i].cv.wait(lck);
            }
        }
    }
}  // namespace sensor

void ChOptixEngine::StopAllThreads() {
    // stop the scene building thread
    {
        // wait for last processing to be done
        std::unique_lock<std::mutex> lck(m_sceneThread.mutex);
        while (!m_sceneThread.done) {
            m_sceneThread.cv.wait(lck);
        }
        m_sceneThread.terminate = true;
        m_sceneThread.start = true;
        m_sceneThread.done = false;
    }
    m_sceneThread.cv.notify_all();

    // wait for it to finish the terminate proces
    {
        std::unique_lock<std::mutex> lck(m_sceneThread.mutex);
        while (!m_sceneThread.done) {
            m_sceneThread.cv.wait(lck);
        }
    }

    if (m_sceneThread.thread.joinable()) {
        m_sceneThread.thread.join();
    }

    m_started = false;

    // stop all the render threads
    for (int i = 0; i < m_renderThreads.size(); i++) {
        {
            // wait for previous processing to be done
            std::unique_lock<std::mutex> lck(m_renderThreads[i].mutex);
            while (!m_renderThreads[i].done) {
                m_renderThreads[i].cv.wait(lck);
            }
            m_renderThreads[i].terminate = true;
            m_renderThreads[i].start = true;
            m_renderThreads[i].done = false;
        }
        m_renderThreads[i].cv.notify_all();

        // wait for it to finish the terminate proces
        std::unique_lock<std::mutex> lck(m_renderThreads[i].mutex);
        while (!m_renderThreads[i].done) {
            m_renderThreads[i].cv.wait(lck);
        }
        if (m_renderThreads[i].thread.joinable()) {
            m_renderThreads[i].thread.join();
        }
    }

    // clear the assigned sensors as their corresponding threads have been stopped
    m_assignedSensor.clear();
    m_assignedRenderers.clear();
    m_renderThreads.clear();
}

void ChOptixEngine::Start() {
    if (!m_started) {
        m_sceneThread.start = false;
        m_sceneThread.terminate = false;
        m_sceneThread.done = true;  // thread is done to begin with (no work to complete)
        m_sceneThread.thread = std::move(std::thread(&ChOptixEngine::SceneProcess, this, std::ref(m_sceneThread)));
        m_started = true;
    }
}

void ChOptixEngine::RenderProcess(RenderThread& tself, std::shared_ptr<ChOptixSensor> sensor) {
    bool terminate = false;

    // keep the thread running until we submit a terminate job or equivalent
    while (!terminate) {
        std::unique_lock<std::mutex> tmp_lock(tself.mutex);
        while (!tself.start) {
            tself.cv.wait(tmp_lock);
        }
        tself.start = false;
        terminate = tself.terminate;

#if PROFILE
        auto start = std::chrono::high_resolution_clock::now();
#endif
        if (!terminate) {
            // run through the filter graph of our sensor
            for (auto f : sensor->GetFilterList()) {
                f->Apply();
            }
        }

        // wait for stream to synchronize
        cudaStreamSynchronize(sensor->GetCudaStream());
#if PROFILE
        auto elapsed = std::chrono::high_resolution_clock::now() - start;
        auto milli = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
        std::cout << "Sensor = " << sensor->GetName() << ", Process time = " << milli << "ms" << std::endl;
#endif
        tself.done = true;
        tmp_lock.unlock();  // explicitely release the lock on this render thread
        tself.cv.notify_all();
    }
}

void ChOptixEngine::SceneProcess(RenderThread& tself) {
    // continually loop and perform two functions: add filters from sensor to job queue, empty the job queue

    bool terminate = false;

    // keep the thread running until we submit a terminate job or equivalent
    while (!terminate) {
        std::unique_lock<std::mutex> tmp_lock(tself.mutex);
        // wait for a notification from the main thread
        while (!tself.start) {
            tself.cv.wait(tmp_lock);
        }
        tself.start = false;  // reset start variable
        terminate = tself.terminate;

        if (!terminate) {
            // wait for the previous render threads before starting this rebuild

            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
            m_geometry->RebuildRootStructure();
            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> wall_time =
                std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
            // launch the render threads
            for (auto i : m_renderQueue) {
                m_renderThreads[i].done = false;
                m_renderThreads[i].start = true;
                m_renderThreads[i].cv.notify_all();  // notify render thread it should proceed

                // for (auto f : m_assignedSensor[i]->GetFilterList()) {
                //     f->Apply();
                // }
                // cudaStreamSynchronize(m_assignedSensor[i]->GetCudaStream());
            }

            // wait for each of the thread to be done before we say we are done
            for (auto i : m_renderQueue) {
                std::unique_lock<std::mutex> lck(m_renderThreads[i].mutex);
                while (!m_renderThreads[i].done) {
                    m_renderThreads[i].cv.wait(lck);
                }
            }
        }
        m_renderQueue.clear();  // empty list of sensor when everything is processed
        tself.done = true;
        tmp_lock.unlock();      // explicitely release the lock on the job queue
        tself.cv.notify_all();  // wake up anyone waiting for us
    }
}

void ChOptixEngine::boxVisualization(std::shared_ptr<ChBody> body,
                                     std::shared_ptr<ChVisualShapeBox> box_shape,
                                     ChFrame<> asset_frame) {
    ChVector3d size = box_shape->GetLengths();

    unsigned int mat_id;
    if (box_shape->GetNumMaterials() == 0) {
        mat_id = m_pipeline->GetBoxMaterial();
    } else {
        mat_id = m_pipeline->GetBoxMaterial(box_shape->GetMaterials());
    }
    m_geometry->AddBox(body, asset_frame, size, mat_id);
    m_pipeline->AddBody(body);
}

void ChOptixEngine::sphereVisualization(std::shared_ptr<ChBody> body,
                                        std::shared_ptr<ChVisualShapeSphere> sphere_shape,
                                        ChFrame<> asset_frame) {
    ChVector3d size(sphere_shape->GetRadius());

    unsigned int mat_id;
    if (sphere_shape->GetNumMaterials() == 0) {
        mat_id = m_pipeline->GetSphereMaterial();
    } else {
        mat_id = m_pipeline->GetSphereMaterial(sphere_shape->GetMaterials());
    }
    m_geometry->AddSphere(body, asset_frame, size, mat_id);
    m_pipeline->AddBody(body);
}

void ChOptixEngine::cylinderVisualization(std::shared_ptr<ChBody> body,
                                          std::shared_ptr<ChVisualShapeCylinder> cyl_shape,
                                          ChFrame<> asset_frame) {
    double radius = cyl_shape->GetRadius();
    double height = cyl_shape->GetHeight();

    ChVector3d size = {radius, radius, height};

    unsigned int mat_id;
    if (cyl_shape->GetNumMaterials() == 0) {
        mat_id = m_pipeline->GetCylinderMaterial();
    } else {
        mat_id = m_pipeline->GetCylinderMaterial(cyl_shape->GetMaterials());
    }
    m_geometry->AddCylinder(body, asset_frame, size, mat_id);
    m_pipeline->AddBody(body);
}

void ChOptixEngine::rigidMeshVisualization(std::shared_ptr<ChBody> body,
                                           std::shared_ptr<ChVisualShapeTriangleMesh> mesh_shape,
                                           ChFrame<> asset_frame) {
    if (mesh_shape->IsWireframe()) {
        std::cerr << "WARNING: Chrono::Sensor does not support wireframe meshes. Defaulting back to solid mesh, please "
                     "check for visual issues.\n";
    }
    ChVector3d size = mesh_shape->GetScale();

    unsigned int mat_id;
    CUdeviceptr d_vertex_buffer;  // handle will go to m_geometry
    CUdeviceptr d_index_buffer;   // handle will go to m_geometry

    mat_id = m_pipeline->GetRigidMeshMaterial(d_vertex_buffer, d_index_buffer, mesh_shape, mesh_shape->GetMaterials());
    m_geometry->AddRigidMesh(d_vertex_buffer, d_index_buffer, mesh_shape, body, asset_frame, size, mat_id);
    m_pipeline->AddBody(body);
}

void ChOptixEngine::deformableMeshVisualization(std::shared_ptr<ChBody> body,
                                                std::shared_ptr<ChVisualShapeTriangleMesh> mesh_shape,
                                                ChFrame<> asset_frame) {
    if (mesh_shape->IsWireframe()) {
        std::cerr << "WARNING: Chrono::Sensor does not support wireframe meshes. Defaulting back to solid mesh, please "
                     "check for visual issues.\n";
    }
    ChVector3d size = mesh_shape->GetScale();

    unsigned int mat_id;
    CUdeviceptr d_vertex_buffer;  // handle will go to m_geometry
    CUdeviceptr d_index_buffer;   // handle will go to m_geometry
    mat_id =
        m_pipeline->GetDeformableMeshMaterial(d_vertex_buffer, d_index_buffer, mesh_shape, mesh_shape->GetMaterials());
    m_geometry->AddDeformableMesh(d_vertex_buffer, d_index_buffer, mesh_shape, body, asset_frame, size, mat_id);
    m_pipeline->AddBody(body);
}

#ifdef USE_SENSOR_NVDB
void ChOptixEngine::nvdbVisualization(std::shared_ptr<ChBody> body,
                                      std::shared_ptr<ChNVDBShape> box_shape,
                                      ChFrame<> asset_frame) {
    ChVector3d size = box_shape->GetBoxGeometry().GetLengths();

    unsigned int mat_id;
    if (box_shape->GetNumMaterials() == 0) {
        mat_id = m_pipeline->GetNVDBMaterial();
    } else {
        mat_id = m_pipeline->GetNVDBMaterial(box_shape->GetMaterials());
    }
    m_geometry->AddNVDBVolume(body, asset_frame, size, mat_id);
    m_pipeline->AddBody(body);
}
#endif  // USE_SENSOR_NVDB

void ChOptixEngine::ConstructScene() {
    // need to lock before touching any optix stuff
    // std::lock_guard<std::mutex> lck(
    //     m_sceneThread.mutex);  /// here we should not wait for a notify, it is good enough to get a lock
    std::unique_lock<std::mutex> lck(m_sceneThread.mutex);
    while (!m_sceneThread.done) {
        m_sceneThread.cv.wait(lck);
    }
    cudaDeviceSynchronize();
    // wipeout all of old scene
    m_geometry->Cleanup();         // remove all geometry
    m_pipeline->CleanMaterials();  // remove all programs and materials

    // iterate through all bodies in Chrono and add a subnode for each body in Chrono
    for (auto body : m_system->GetBodies()) {
        if (body->GetVisualModel()) {
            for (auto& shape_instance : body->GetVisualModel()->GetShapeInstances()) {
                const auto& shape = shape_instance.shape;
                const auto& shape_frame = shape_instance.frame;
                // check if the asset is a ChVisualShape

                // if (std::shared_ptr<ChVisualShape> visual_asset = std::dynamic_pointer_cast<ChVisualShape>(asset)) {

                // collect relative position and orientation of the asset
                // ChVector3d asset_pos = visual_asset->Pos;
                // ChMatrix33<double> asset_rot_mat = visual_asset->Rot;

                // const ChFrame<float> asset_frame = ChFrame<float>(asset_pos,asset_rot_mat);

                if (!shape->IsVisible()) {
                    // std::cout << "Ignoring an asset that is set to invisible\n";
                } else if (auto box_shape = std::dynamic_pointer_cast<ChVisualShapeBox>(shape)) {
                    boxVisualization(body, box_shape, shape_frame);
                } 
                #ifdef USE_SENSOR_NVDB
                else if (std::shared_ptr<ChNVDBShape> nvdb_shape = std::dynamic_pointer_cast<ChNVDBShape>(shape)) {
                    nvdbVisualization(body, nvdb_shape, shape_frame);
                    printf("Added NVDB Shape!");
                }
                #endif
                else if (auto sphere_shape = std::dynamic_pointer_cast<ChVisualShapeSphere>(shape)) {
                    sphereVisualization(body, sphere_shape, shape_frame);

                } else if (auto cylinder_shape = std::dynamic_pointer_cast<ChVisualShapeCylinder>(shape)) {
                    cylinderVisualization(body, cylinder_shape, shape_frame);

                } else if (auto trimesh_shape = std::dynamic_pointer_cast<ChVisualShapeTriangleMesh>(shape)) {
                    if (!trimesh_shape->IsMutable()) {
                        rigidMeshVisualization(body, trimesh_shape, shape_frame);

                        // added_asset_for_body = true;
                    } else {
                        deformableMeshVisualization(body, trimesh_shape, shape_frame);
                    }

                } else if (auto ellipsoid_shape = std::dynamic_pointer_cast<ChVisualShapeEllipsoid>(shape)) {
                } else if (auto cone_shape = std::dynamic_pointer_cast<ChVisualShapeCone>(shape)) {
                } else if (auto rbox_shape = std::dynamic_pointer_cast<ChVisualShapeRoundedBox>(shape)) {
                } else if (auto capsule_shape = std::dynamic_pointer_cast<ChVisualShapeCapsule>(shape)) {
                } else if (auto path_shape = std::dynamic_pointer_cast<ChVisualShapePath>(shape)) {
                } else if (auto line_shape = std::dynamic_pointer_cast<ChVisualShapeLine>(shape)) {
                }

                // TODO: Add NVDB Vis condition
                // }
                // }
            }
        }
    }

    // Assumption made here that other physics items don't have a transform -> not always true!!!
    for (auto item : m_system->GetOtherPhysicsItems()) {
        if (item->GetVisualModel()) {
            for (auto& shape_instance : item->GetVisualModel()->GetShapeInstances()) {
                const auto& shape = shape_instance.shape;
                const auto& shape_frame = shape_instance.frame;

                auto dummy_body = chrono_types::make_shared<ChBody>();

                if (!shape->IsVisible()) {
                    // std::cout << "Ignoring an asset that is set to invisible\n";
                } else if (auto box_shape = std::dynamic_pointer_cast<ChVisualShapeBox>(shape)) {
                    boxVisualization(dummy_body, box_shape, shape_frame);

                } else if (auto sphere_shape = std::dynamic_pointer_cast<ChVisualShapeSphere>(shape)) {
                    sphereVisualization(dummy_body, sphere_shape, shape_frame);

                } else if (auto cylinder_shape = std::dynamic_pointer_cast<ChVisualShapeCylinder>(shape)) {
                    cylinderVisualization(dummy_body, cylinder_shape, shape_frame);

                } else if (auto trimesh_shape = std::dynamic_pointer_cast<ChVisualShapeTriangleMesh>(shape)) {
                    if (!trimesh_shape->IsMutable()) {
                        rigidMeshVisualization(dummy_body, trimesh_shape, shape_frame);
                    } else {
                        deformableMeshVisualization(dummy_body, trimesh_shape, shape_frame);
                    }

                } else if (auto ellipsoid_shape = std::dynamic_pointer_cast<ChVisualShapeEllipsoid>(shape)) {
                } else if (auto cone_shape = std::dynamic_pointer_cast<ChVisualShapeCone>(shape)) {
                } else if (auto rbox_shape = std::dynamic_pointer_cast<ChVisualShapeRoundedBox>(shape)) {
                } else if (auto capsule_shape = std::dynamic_pointer_cast<ChVisualShapeCapsule>(shape)) {
                } else if (auto path_shape = std::dynamic_pointer_cast<ChVisualShapePath>(shape)) {
                } else if (auto line_shape = std::dynamic_pointer_cast<ChVisualShapeLine>(shape)) {
                }
            }
        }
    }

    m_params.root = m_geometry->CreateRootStructure();
    m_pipeline->UpdateAllSBTs();
    m_pipeline->UpdateAllPipelines();
    m_params.mesh_pool = reinterpret_cast<MeshParameters*>(m_pipeline->GetMeshPool());
    m_params.material_pool = reinterpret_cast<MaterialParameters*>(m_pipeline->GetMaterialPool());
    cudaMemcpy(reinterpret_cast<void*>(md_params), &m_params, sizeof(ContextParameters), cudaMemcpyHostToDevice);
}

void ChOptixEngine::UpdateCameraTransforms(std::vector<int>& to_be_updated, std::shared_ptr<ChScene> scene) {
    // go through the sensors to be updated and see if we need to move the scene origin
    for (unsigned int i = 0; i < to_be_updated.size(); i++) {
        int id = to_be_updated[i];
        ChFrame<double> f_offset = m_assignedSensor[id]->GetOffsetPose();
        ChFrame<double> f_body_0 = m_cameraStartFrames[id];
        ChFrame<double> global_loc_0 = f_body_0 * f_offset;
        scene->UpdateOriginOffset(global_loc_0.GetPos());
    }

    // go through all the sensors to be updated
    for (unsigned int i = 0; i < to_be_updated.size(); i++) {
        int id = to_be_updated[i];
        auto sensor = m_assignedSensor[id];

        // update radar velocity
        if (auto radar = std::dynamic_pointer_cast<ChRadarSensor>(sensor)) {
            ChVector3f origin(0, 0, 0);
            auto r = radar->GetOffsetPose().GetPos() - origin;
            auto ang_vel = radar->GetAngularVelocity() % r;
            auto vel_abs =
                radar->GetOffsetPose().TransformDirectionLocalToParent(ang_vel) + radar->GetTranslationalVelocity();
            m_assignedRenderers[id]->m_raygen_record->data.specific.radar.velocity.x = vel_abs.x();
            m_assignedRenderers[id]->m_raygen_record->data.specific.radar.velocity.y = vel_abs.y();
            m_assignedRenderers[id]->m_raygen_record->data.specific.radar.velocity.z = vel_abs.z();
            m_pipeline->UpdateObjectVelocity();
        }

        ChFrame<double> f_offset = sensor->GetOffsetPose();
        ChFrame<double> f_body_0 = m_cameraStartFrames[i];
        m_cameraStartFrames_set[i] = false;  // reset this camera frame so that we know it should be packed again
        ChFrame<double> f_body_1 = sensor->GetParent()->GetVisualModelFrame();
        ChFrame<double> global_loc_0 = f_body_0 * f_offset;
        ChFrame<double> global_loc_1 = f_body_1 * f_offset;

        ChVector3f pos_0 = global_loc_0.GetPos() - scene->GetOriginOffset();
        ChVector3f pos_1 = global_loc_1.GetPos() - scene->GetOriginOffset();

        m_assignedRenderers[id]->m_raygen_record->data.t0 =
            (float)(m_system->GetChTime() - sensor->GetCollectionWindow());
        m_assignedRenderers[id]->m_raygen_record->data.t1 = (float)(m_system->GetChTime());
        m_assignedRenderers[id]->m_raygen_record->data.pos0 = make_float3(pos_0.x(), pos_0.y(), pos_0.z());
        m_assignedRenderers[id]->m_raygen_record->data.rot0 =
            make_float4((float)global_loc_0.GetRot().e0(), (float)global_loc_0.GetRot().e1(),
                        (float)global_loc_0.GetRot().e2(), (float)global_loc_0.GetRot().e3());
        m_assignedRenderers[id]->m_raygen_record->data.pos1 = make_float3(pos_1.x(), pos_1.y(), pos_1.z());
        m_assignedRenderers[id]->m_raygen_record->data.rot1 =
            make_float4((float)global_loc_1.GetRot().e0(), (float)global_loc_1.GetRot().e1(),
                        (float)global_loc_1.GetRot().e2(), (float)global_loc_1.GetRot().e3());
        m_assignedRenderers[id]->m_time_stamp = (float)m_system->GetChTime();
    }
}

void ChOptixEngine::UpdateDeformableMeshes() {
    // update the mesh in the pipeline
    m_pipeline->UpdateDeformableMeshes();
    // update the meshes in the geometric scene
    m_geometry->UpdateDeformableMeshes();
}

void ChOptixEngine::UpdateSceneDescription(std::shared_ptr<ChScene> scene) {
    if (scene->GetBackgroundChanged()) {
        m_pipeline->UpdateBackground(scene->GetBackground());

        m_params.scene_epsilon = scene->GetSceneEpsilon();
        m_params.fog_color = {scene->GetFogColor().x(), scene->GetFogColor().y(), scene->GetFogColor().z()};
        m_params.fog_scattering = scene->GetFogScattering();

        cudaMemcpy(reinterpret_cast<void*>(md_params), &m_params, sizeof(ContextParameters), cudaMemcpyHostToDevice);
        scene->ResetBackgroundChanged();
    }

    if (scene->GetLightsChanged() || scene->GetOriginChanged() || scene->GetAreaLightsChanged()) {

        // Handling changes to area lights

        std::vector<AreaLight> a = scene->GetAreaLights();
        
       
        if (a.size() != m_params.num_arealights) {  // need new memory in this case
            if (m_params.arealights)
                CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(m_params.arealights)));

            cudaMalloc(reinterpret_cast<void**>(&m_params.arealights), a.size() * sizeof(AreaLight));
        }

        
        for (unsigned int i = 0; i < a.size(); i++) {
            a[i].pos = make_float3(a[i].pos.x - scene->GetOriginOffset().x(), a[i].pos.y - scene->GetOriginOffset().y(),
                                   a[i].pos.z - scene->GetOriginOffset().z());
        }

        cudaMemcpy(reinterpret_cast<void*>(m_params.arealights), a.data(), a.size() * sizeof(AreaLight),
                   cudaMemcpyHostToDevice);

        m_params.num_arealights = static_cast<int>(a.size());
        
        // Handling changes for point lights

        std::vector<PointLight> l = scene->GetPointLights();
        if (l.size() != m_params.num_lights) {  // need new memory in this case
            if (m_params.lights)
                CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(m_params.lights)));

            cudaMalloc(reinterpret_cast<void**>(&m_params.lights), l.size() * sizeof(PointLight));
        }

        for (unsigned int i = 0; i < l.size(); i++) {
            l[i].pos = make_float3(l[i].pos.x - scene->GetOriginOffset().x(), l[i].pos.y - scene->GetOriginOffset().y(),
                                   l[i].pos.z - scene->GetOriginOffset().z());
        }

        cudaMemcpy(reinterpret_cast<void*>(m_params.lights), l.data(), l.size() * sizeof(PointLight),
                   cudaMemcpyHostToDevice);
        m_params.num_lights = static_cast<int>(l.size());
        
        // Handling changes in origin

        m_params.ambient_light_color = {scene->GetAmbientLight().x(), scene->GetAmbientLight().y(),
                                        scene->GetAmbientLight().z()};

        cudaMemcpy(reinterpret_cast<void*>(md_params), &m_params, sizeof(ContextParameters), cudaMemcpyHostToDevice);

        m_geometry->SetOriginOffset(scene->GetOriginOffset());
        scene->ResetLightsChanged();
        scene->ResetAreaLightsChanged();
        scene->ResetOriginChanged();
    }

    #ifdef USE_SENSOR_NVDB
    if (float* d_pts = scene->GetFSIParticles()) {
        int n = scene->GetNumFSIParticles();
        
        printf("Creatinng NanoVDB Handle...\n");
        using buildType = nanovdb::Point;
        nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> handle = createNanoVDBGridHandle(d_pts, n);
        nanovdb::NanoGrid<buildType>* grid = handle.deviceGrid<buildType>();
        handle.deviceDownload();
        auto* grid_h = handle.grid<buildType>();
        auto* tree = grid_h->treePtr();

        // printf("Grid Size: %d\n", grid_h->gridSize());
        ////printf("Point Count: %d", (int)grid_h->pointCount());
        // printf("Upper Internal Nodes: %d\n", grid_h->tree().nodeCount(2));
        // printf("Lower Internal Nodes: %d\n", grid_h->tree().nodeCount(1));
        // printf("Leaf Nodes: %d\n", grid_h->tree().nodeCount(0));

        // float wBBoxDimZ = (float)grid_h->worldBBox().dim()[2] * 2;
        // nanovdb::Vec3<float> wBBoxCenter = nanovdb::Vec3<float>(grid_h->worldBBox().min() + grid_h->worldBBox().dim()
        // * 0.5f); nanovdb::CoordBBox treeIndexBbox = grid_h->tree().bbox(); std::cout << "Bounds: "
        //          << "[" << treeIndexBbox.min()[0] << "," << treeIndexBbox.min()[1] << "," << treeIndexBbox.min()[2]
        //          << "] -> [" << treeIndexBbox.max()[0] << "," << treeIndexBbox.max()[1] << "," <<
        //          treeIndexBbox.max()[2]
        //          << "]" << std::endl;

        /* printf("size of handle_ptr: %d | size of grid*: %d\n", sizeof(m_params.handle_ptr), sizeof(grid));
         printf("Grid ptr: %p | Grid Size: %d | Grid Type: %d | Grid Empty: %d\n ", grid, handle.gridSize(),
         handle.gridType(), handle.empty()); printf("size of ContextParameters: %d\n", sizeof(ContextParameters));*/

        cudaMalloc((void**)&m_params.handle_ptr, handle.gridSize());
        cudaMemcpy((void*)m_params.handle_ptr, grid, handle.gridSize(), cudaMemcpyDeviceToDevice);
        /* cudaError_t status = cudaMalloc((void**)&md_params->handle_ptr, handle.gridSize());
         if (status != cudaSuccess) {
            printf("cudaMalloc failed: %s\n",cudaGetErrorString(status));
          }
         printf("md grid ptr: %p\n", md_params->handle_ptr);

         cudaMemcpy(md_params->handle_ptr, grid, handle.gridSize(), cudaMemcpyDeviceToDevice);*/
        /*printf("Done!\n");
        size_t sz = handle.gridSize();
        cudaMalloc(reinterpret_cast<void**>(&m_params.handle_ptr), sz);he
        printf("handle: %p\n", &handle);
        cudaMemcpy(reinterpret_cast<void*>(m_params.handle_ptr), &handle, sz, cudaMemcpyHostToDevice);*/

        cudaMemcpy(reinterpret_cast<void*>(md_params), &m_params, sizeof(ContextParameters), cudaMemcpyHostToDevice);
    }
    #endif
    }

}  // namespace sensor
}  // namespace chrono
