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

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCapsuleShape.h"
#include "chrono/assets/ChConeShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChEllipsoidShape.h"
#include "chrono/assets/ChLineShape.h"
#include "chrono/assets/ChPathShape.h"
#include "chrono/assets/ChRoundedBoxShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/physics/ChSystem.h"

#include <random>

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
    m_params.num_lights = 0;
    m_params.ambient_light_color = make_float3(0.0f, 0.0f, 0.0f);  // make_float3(0.1f, 0.1f, 0.1f);  // default value
    m_params.max_depth = m_recursions;
    m_params.scene_epsilon = 1.e-3f;    // TODO: determine a good value for this
    m_params.importance_cutoff = .01f;  /// TODO: determine a good value for this

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
                                     std::shared_ptr<ChBoxShape> box_shape,
                                     ChFrame<> asset_frame) {
    ChVector<double> size = box_shape->GetBoxGeometry().GetLengths();

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
                                        std::shared_ptr<ChSphereShape> sphere_shape,
                                        ChFrame<> asset_frame) {
    ChVector<double> size = {sphere_shape->GetSphereGeometry().rad, sphere_shape->GetSphereGeometry().rad,
                             sphere_shape->GetSphereGeometry().rad};

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
                                          std::shared_ptr<ChCylinderShape> cyl_shape,
                                          ChFrame<> asset_frame) {
    double radius = cyl_shape->GetCylinderGeometry().rad;
    double height = (cyl_shape->GetCylinderGeometry().p1 - cyl_shape->GetCylinderGeometry().p2).Length();
    ChVector<double> center = (cyl_shape->GetCylinderGeometry().p1 + cyl_shape->GetCylinderGeometry().p2) / 2;

    ChVector<double> size = {radius, height, radius};

    ChFrame<double> cyl_frame = asset_frame * ChFrame<double>(center);

    unsigned int mat_id;
    if (cyl_shape->GetNumMaterials() == 0) {
        mat_id = m_pipeline->GetCylinderMaterial();
    } else {
        mat_id = m_pipeline->GetCylinderMaterial(cyl_shape->GetMaterials());
    }
    m_geometry->AddCylinder(body, cyl_frame, size, mat_id);
    m_pipeline->AddBody(body);
}

void ChOptixEngine::rigidMeshVisualization(std::shared_ptr<ChBody> body,
                                           std::shared_ptr<ChTriangleMeshShape> mesh_shape,
                                           ChFrame<> asset_frame) {
    if (mesh_shape->IsWireframe()) {
        std::cerr << "WARNING: Chrono::Sensor does not support wireframe meshes. Defaulting back to solid mesh, please "
                     "check for visual issues.\n";
    }
    ChVector<double> size = mesh_shape->GetScale();

    unsigned int mat_id;
    CUdeviceptr d_vertex_buffer;  // handle will go to m_geometry
    CUdeviceptr d_index_buffer;   // handle will go to m_geometry

    mat_id = m_pipeline->GetRigidMeshMaterial(d_vertex_buffer, d_index_buffer, mesh_shape, mesh_shape->GetMaterials());
    m_geometry->AddRigidMesh(d_vertex_buffer, d_index_buffer, mesh_shape, body, asset_frame, size, mat_id);
    m_pipeline->AddBody(body);
}

void ChOptixEngine::deformableMeshVisualization(std::shared_ptr<ChBody> body,
                                                std::shared_ptr<ChTriangleMeshShape> mesh_shape,
                                                ChFrame<> asset_frame) {
    if (mesh_shape->IsWireframe()) {
        std::cerr << "WARNING: Chrono::Sensor does not support wireframe meshes. Defaulting back to solid mesh, please "
                     "check for visual issues.\n";
    }
    ChVector<double> size = mesh_shape->GetScale();

    unsigned int mat_id;
    CUdeviceptr d_vertex_buffer;  // handle will go to m_geometry
    CUdeviceptr d_index_buffer;   // handle will go to m_geometry
    mat_id =
        m_pipeline->GetDeformableMeshMaterial(d_vertex_buffer, d_index_buffer, mesh_shape, mesh_shape->GetMaterials());
    m_geometry->AddDeformableMesh(d_vertex_buffer, d_index_buffer, mesh_shape, body, asset_frame, size, mat_id);
    m_pipeline->AddBody(body);
}

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
    for (auto body : m_system->Get_bodylist()) {
        if (body->GetVisualModel()) {
            for (auto& shape_instance : body->GetVisualModel()->GetShapes()) {
                const auto& shape = shape_instance.first;
                const auto& shape_frame = shape_instance.second;
                // check if the asset is a ChVisualShape

                // if (std::shared_ptr<ChVisualShape> visual_asset = std::dynamic_pointer_cast<ChVisualShape>(asset)) {

                // collect relative position and orientation of the asset
                // ChVector<double> asset_pos = visual_asset->Pos;
                // ChMatrix33<double> asset_rot_mat = visual_asset->Rot;

                // const ChFrame<float> asset_frame = ChFrame<float>(asset_pos,asset_rot_mat);

                if (!shape->IsVisible()) {
                    // std::cout << "Ignoring an asset that is set to invisible\n";
                } else if (std::shared_ptr<ChBoxShape> box_shape = std::dynamic_pointer_cast<ChBoxShape>(shape)) {
                    boxVisualization(body, box_shape, shape_frame);

                } else if (std::shared_ptr<ChSphereShape> sphere_shape =
                               std::dynamic_pointer_cast<ChSphereShape>(shape)) {
                    sphereVisualization(body, sphere_shape, shape_frame);

                } else if (std::shared_ptr<ChCylinderShape> cylinder_shape =
                               std::dynamic_pointer_cast<ChCylinderShape>(shape)) {
                    cylinderVisualization(body, cylinder_shape, shape_frame);

                } else if (std::shared_ptr<ChTriangleMeshShape> trimesh_shape =
                               std::dynamic_pointer_cast<ChTriangleMeshShape>(shape)) {
                    if (!trimesh_shape->IsMutable()) {
                        rigidMeshVisualization(body, trimesh_shape, shape_frame);
                        
                        // added_asset_for_body = true;
                    } else {
                        deformableMeshVisualization(body, trimesh_shape, shape_frame);
                    }

                } else if (std::shared_ptr<ChEllipsoidShape> ellipsoid_shape =
                               std::dynamic_pointer_cast<ChEllipsoidShape>(shape)) {
                } else if (std::shared_ptr<ChConeShape> cone_shape = std::dynamic_pointer_cast<ChConeShape>(shape)) {
                } else if (std::shared_ptr<ChRoundedBoxShape> rbox_shape =
                               std::dynamic_pointer_cast<ChRoundedBoxShape>(shape)) {
                } else if (std::shared_ptr<ChCapsuleShape> capsule_shape =
                               std::dynamic_pointer_cast<ChCapsuleShape>(shape)) {
                } else if (std::shared_ptr<ChPathShape> path_shape = std::dynamic_pointer_cast<ChPathShape>(shape)) {
                } else if (std::shared_ptr<ChLineShape> line_shape = std::dynamic_pointer_cast<ChLineShape>(shape)) {
                }

                // }
                // }
            }
        }
    }

    // // Assumption made here that other physics items don't have a transform -> not always true!!!
    for (auto item : m_system->Get_otherphysicslist()) {
        if (item->GetVisualModel()) {
            for (auto& shape_instance : item->GetVisualModel()->GetShapes()) {
                const auto& shape = shape_instance.first;
                const auto& shape_frame = shape_instance.second;

                auto dummy_body = chrono_types::make_shared<ChBody>();

                if (!shape->IsVisible()) {
                    // std::cout << "Ignoring an asset that is set to invisible\n";
                } else if (std::shared_ptr<ChBoxShape> box_shape = std::dynamic_pointer_cast<ChBoxShape>(shape)) {
                    boxVisualization(dummy_body, box_shape, shape_frame);

                } else if (std::shared_ptr<ChSphereShape> sphere_shape =
                               std::dynamic_pointer_cast<ChSphereShape>(shape)) {
                    sphereVisualization(dummy_body, sphere_shape, shape_frame);

                } else if (std::shared_ptr<ChCylinderShape> cylinder_shape =
                               std::dynamic_pointer_cast<ChCylinderShape>(shape)) {
                    cylinderVisualization(dummy_body, cylinder_shape, shape_frame);

                } else if (std::shared_ptr<ChTriangleMeshShape> trimesh_shape =
                               std::dynamic_pointer_cast<ChTriangleMeshShape>(shape)) {
                    if (!trimesh_shape->IsMutable()) {
                        rigidMeshVisualization(dummy_body, trimesh_shape, shape_frame);
                    } else {
                        deformableMeshVisualization(dummy_body, trimesh_shape, shape_frame);
                    }

                } else if (std::shared_ptr<ChEllipsoidShape> ellipsoid_shape =
                               std::dynamic_pointer_cast<ChEllipsoidShape>(shape)) {
                } else if (std::shared_ptr<ChConeShape> cone_shape = std::dynamic_pointer_cast<ChConeShape>(shape)) {
                } else if (std::shared_ptr<ChRoundedBoxShape> rbox_shape =
                               std::dynamic_pointer_cast<ChRoundedBoxShape>(shape)) {
                } else if (std::shared_ptr<ChCapsuleShape> capsule_shape =
                               std::dynamic_pointer_cast<ChCapsuleShape>(shape)) {
                } else if (std::shared_ptr<ChPathShape> path_shape = std::dynamic_pointer_cast<ChPathShape>(shape)) {
                } else if (std::shared_ptr<ChLineShape> line_shape = std::dynamic_pointer_cast<ChLineShape>(shape)) {
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
            ChVector<float> origin(0, 0, 0);
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

        ChVector<float> pos_0 = global_loc_0.GetPos() - scene->GetOriginOffset();
        ChVector<float> pos_1 = global_loc_1.GetPos() - scene->GetOriginOffset();

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

    if (scene->GetLightsChanged() || scene->GetOriginChanged()) {
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
        m_params.ambient_light_color = {scene->GetAmbientLight().x(), scene->GetAmbientLight().y(),
                                        scene->GetAmbientLight().z()};
        cudaMemcpy(reinterpret_cast<void*>(md_params), &m_params, sizeof(ContextParameters), cudaMemcpyHostToDevice);

        m_geometry->SetOriginOffset(scene->GetOriginOffset());
        scene->ResetLightsChanged();
        scene->ResetOriginChanged();
    }
}
}  // namespace sensor
}  // namespace chrono
