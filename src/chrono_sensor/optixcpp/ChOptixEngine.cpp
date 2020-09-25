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

#include "chrono_sensor/optixcpp/ChOptixEngine.h"

#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/optixcpp/ChOptixUtils.h"
#include "chrono_sensor/scene/lights.h"
#include "chrono_sensor/utils/ChVisualMaterialUtils.h"

#include "chrono/assets/ChAsset.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCapsuleShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChConeShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChEllipsoidShape.h"
#include "chrono/assets/ChLineShape.h"
#include "chrono/assets/ChPathShape.h"
#include "chrono/assets/ChRoundedBoxShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChVisualization.h"
#include "chrono/physics/ChSystem.h"

#include <random>

namespace chrono {
namespace sensor {

using namespace optix;

ChOptixEngine::ChOptixEngine(ChSystem* sys, int device_id, int max_scene_reflections, bool verbose, int max_keyframes)
    : m_verbose(verbose),
      m_deviceId(device_id),
      m_recursions(max_scene_reflections),
      m_max_keyframes_needed(max_keyframes) {
    m_system = sys;
    Initialize();
}
ChOptixEngine::~ChOptixEngine() {
    if (!m_terminate) {
        Stop();  // if it hasn't been stopped yet, stop it ourselves
    }
    // if (m_thread.joinable()) {
    //     m_thread.join();
    // }
    m_context->destroy();
}

void ChOptixEngine::AssignSensor(std::shared_ptr<ChOptixSensor> sensor) {
    // all optix sensors should be treated the same. Up to the sensor to specify a difference by using a unique launch
    // kernel and set of filters
    {
        std::lock_guard<std::mutex> lck(m_renderQueueMutex);

        sensor->m_context = m_context;
        m_assignedSensor.push_back(sensor);

        // initialize filters just in case they want to create any chunks of memory from the start
        for (auto f : sensor->GetFilterList()) {
            f->Initialize(sensor);  // master thread should always be the one to initialize
        }
        sensor->LockFilterList();

        if (m_verbose) {
            if (!m_exception_prog)
                m_exception_prog = GetRTProgram(m_context, "exception", "base");
            m_context->setExceptionProgram(m_context->getEntryPointCount() - 1, m_exception_prog);
        }

        m_noise_initialized = false;
        m_num_noise_vals = max(m_num_noise_vals, sensor->m_width * sensor->m_height);

        // m_max_keyframes_needed = max(m_max_keyframes_needed,sensor);
    }
    if (!m_started) {
        Start();
    }
}

void ChOptixEngine::UpdateSensors(std::shared_ptr<ChScene> scene) {
    std::vector<int> to_be_updated;
    std::vector<int> to_be_waited_on;

    PackKeyFrames();

    // check which sensors need to be updated
    for (int i = 0; i < m_assignedSensor.size(); i++) {
        auto sensor = m_assignedSensor[i];
        if (m_system->GetChTime() >
            sensor->GetNumLaunches() / sensor->GetUpdateRate() + sensor->GetCollectionWindow() - 1e-7) {
            // time to start the launch
            to_be_updated.push_back(i);
        }
    }

    if (to_be_updated.size() > 0) {
        // m_keyframes = keyframes;

        // lock the render queue
        {
            std::lock_guard<std::mutex> lck(m_renderQueueMutex);

            // initialize noise if not done yet
            if (!m_noise_initialized && m_num_noise_vals > 0) {
                m_noise_initialized = true;
                /// create and set the noise buffer here
                optix::Buffer ray_gen_noise_buffer =
                    m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT, m_num_noise_vals);
                m_context["noise_buffer"]->setBuffer(ray_gen_noise_buffer);

                auto generator = std::minstd_rand(
                    (unsigned int)std::chrono::high_resolution_clock::now().time_since_epoch().count());
                float* ptr = (float*)ray_gen_noise_buffer->map();
                for (int i = 0; i < (int)m_num_noise_vals; i++) {
                    ptr[i] = generator() / (float)generator.max();
                }

                ray_gen_noise_buffer->unmap();
            }

            // std::cout << "Starting optix scene update\n";
            // update the scene for the optix context
            UpdateCameraTransforms();
            UpdateBodyTransforms();
            UpdateSceneDescription(scene);
            UpdateDynamicMeshes();
            // std::cout << "Updated optix scene\n";

            float t = (float)m_system->GetChTime();
            // push the sensors that need updating to the render queue
            for (int i = 0; i < to_be_updated.size(); i++) {
                m_renderQueue.push_back(m_assignedSensor[to_be_updated[i]]);
                m_assignedSensor[to_be_updated[i]]->IncrementNumLaunches();
                m_assignedSensor[to_be_updated[i]]->m_time_stamp = t;
            }
        }

        // we only notify the worker thread when there is a sensor to launch and filters to process
        m_renderQueueCV.notify_all();
    }

    // wait for any sensors whose lag times would mean the data should be available before the next ones start rendering
    for (int i = 0; i < m_assignedSensor.size(); i++) {
        auto sensor = m_assignedSensor[i];
        if (m_system->GetChTime() > (sensor->GetNumLaunches() - 1) / sensor->GetUpdateRate() +
                                        sensor->GetCollectionWindow() + sensor->GetLag() - 1e-7) {
            // if any sensors need to have their data returned, we must wait until optix is done rendering their data
            // TODO: allow waiting for the specific sensor rather than all of them
            bool data_complete = false;
            while (!data_complete) {
                std::lock_guard<std::mutex> lck(m_renderQueueMutex);
                if (m_renderQueue.empty())
                    data_complete = true;
            }
        }
    }
}

void ChOptixEngine::Stop() {
    {
        std::lock_guard<std::mutex> lck(m_renderQueueMutex);
        m_terminate = true;
    }
    m_renderQueueCV.notify_all();

    if (m_thread.joinable()) {
        m_thread.join();
    }

    m_started = false;
}

void ChOptixEngine::Start() {
    if (!m_started) {
        m_thread = std::move(std::thread(&ChOptixEngine::Process, this));
        m_started = true;
    }
}

void ChOptixEngine::Process() {
    // continually loop and perform two functions: add filters from sensor to job queue, empty the job queue

    bool terminate = false;

    // keep the thread running until we submit a terminate job or equivalent
    while (!terminate) {
        std::unique_lock<std::mutex> tmp_lock(m_renderQueueMutex);

        // wait for a notification from the master thread
        while (m_renderQueue.empty() && !m_terminate) {
            m_renderQueueCV.wait(tmp_lock);
        }

        terminate = m_terminate;

        if (!terminate) {
            for (auto pSensor : m_renderQueue) {
                std::shared_ptr<SensorBuffer> buffer;
                // step through the filter list, applying each filter
                for (auto filter : pSensor->GetFilterList()) {
                    filter->Apply(pSensor, buffer);
                }
            }
        }

        m_renderQueue.clear();  // empty list of sensor when everything is processed
        tmp_lock.unlock();      // explicitely release the lock on the job queue
    }
}

void ChOptixEngine::boxVisualization(std::shared_ptr<ChBoxShape> box_shape,
                                     // Program box_bounds,
                                     // Program box_intersect,
                                     std::shared_ptr<ChVisualization> visual_asset,
                                     optix::Group asset_group) {
    // optix::Transform trans) {
    ChVector<double> asset_pos = visual_asset->Pos;
    ChMatrix33<double> asset_rot_mat = visual_asset->Rot;
    ChVector<float> size = box_shape->GetBoxGeometry().GetLengths();

    // create the box geometry
    Geometry box = GetOptixBoxGeometry();

    // create a material for the box
    Material matl;
    if (box_shape->material_list.size() == 0) {
        matl = GetDefaultMaterial();
    } else {
        matl = CreateMaterial(box_shape->material_list[0]);
    }

    // create geometry instance for the box
    GeometryInstance gis = m_context->createGeometryInstance();
    gis->setGeometry(box);
    gis->setMaterialCount(1);
    gis->setMaterial(0, matl);

    GeometryGroup geometrygroup = m_context->createGeometryGroup();
    geometrygroup->setChildCount(1);
    geometrygroup->setChild(0, gis);  // add the box geometry to the geometry group

    if (!m_box_accel)
        m_box_accel = m_context->createAcceleration("trbvh");
    geometrygroup->setAcceleration(m_box_accel);
    // geometrygroup->getAcceleration()->setProperty("refit", "1");

    Transform asset_transform = CreateTransform(m_context, asset_rot_mat, asset_pos, size);

    // add the asset transform as a child of the body transform
    asset_group->addChild(asset_transform);
    // add the geometry group as child of the asset transform
    asset_transform->setChild(geometrygroup);
}

CH_SENSOR_API optix::Geometry ChOptixEngine::GetOptixBoxGeometry() {
    if (m_box_geometry)
        return m_box_geometry;

    m_box_geometry = m_context->createGeometry();

    if (!m_box_bounds)
        m_box_bounds = GetRTProgram(m_context, "box", "box_bounds");
    if (!m_box_int)
        m_box_int = GetRTProgram(m_context, "box", "box_intersect");

    m_box_geometry->setPrimitiveCount(1u);
    m_box_geometry->setBoundingBoxProgram(m_box_bounds);
    m_box_geometry->setIntersectionProgram(m_box_int);

    return m_box_geometry;
}

void ChOptixEngine::sphereVisualization(std::shared_ptr<ChSphereShape> sphere_shape,
                                        std::shared_ptr<ChVisualization> visual_asset,
                                        optix::Group asset_group) {
    ChVector<double> asset_pos = visual_asset->Pos;
    ChMatrix33<double> asset_rot_mat = visual_asset->Rot;
    float radius = (float)sphere_shape->GetSphereGeometry().rad;
    ChVector<double> center = sphere_shape->GetSphereGeometry().center;

    // create the sphere geometry
    Geometry sphere = GetOptixSphereGeometry();

    // create a material to do diffuse shading
    Material matl;
    if (sphere_shape->material_list.size() == 0) {
        matl = GetDefaultMaterial();
    } else {
        matl = CreateMaterial(sphere_shape->material_list[0]);
    }

    // create geometry instance for the sphere
    GeometryInstance gis = m_context->createGeometryInstance(sphere, &matl, &matl + 1);
    GeometryGroup geometrygroup = m_context->createGeometryGroup();
    geometrygroup->setChildCount(1);
    geometrygroup->setChild(0, gis);  // add the sphere geometry to the geometry group
    if (!m_sphere_accel)
        m_sphere_accel = m_context->createAcceleration("trbvh");
    geometrygroup->setAcceleration(m_sphere_accel);
    // geometrygroup->getAcceleration()->setProperty("refit", "1");

    // create a transform that will manipulate the sphere
    // Transform asset_transform = m_context->createTransform();
    Transform asset_transform = CreateTransform(m_context, asset_rot_mat, asset_pos + center, ChVector<>(radius));

    // add the asset transform as child of the body transform
    asset_group->addChild(asset_transform);
    asset_transform->setChild(geometrygroup);
}

CH_SENSOR_API optix::Geometry ChOptixEngine::GetOptixSphereGeometry() {
    if (m_sphere_geometry)
        return m_sphere_geometry;

    m_sphere_geometry = m_context->createGeometry();

    if (!m_sphere_int)
        m_sphere_int = GetRTProgram(m_context, "sphere", "sphere_intersect");
    if (!m_sphere_bounds)
        m_sphere_bounds = GetRTProgram(m_context, "sphere", "sphere_bounds");

    m_sphere_geometry->setPrimitiveCount(1u);
    m_sphere_geometry->setBoundingBoxProgram(m_sphere_bounds);
    m_sphere_geometry->setIntersectionProgram(m_sphere_int);

    return m_sphere_geometry;
}

void ChOptixEngine::cylinderVisualization(std::shared_ptr<ChCylinderShape> cylinder_shape,
                                          std::shared_ptr<ChVisualization> visual_asset,
                                          optix::Group asset_group) {
    ChVector<double> asset_pos = visual_asset->Pos;
    ChMatrix33<double> asset_rot_mat = visual_asset->Rot;
    float radius = (float)cylinder_shape->GetCylinderGeometry().rad;
    float height =
        (float)(cylinder_shape->GetCylinderGeometry().p1 - cylinder_shape->GetCylinderGeometry().p2).Length();

    // create the sphere geometry
    Geometry cylinder = GetOptixCylinderGeometry();  // m_context->createGeometry();

    // create a material to do diffuse shading
    Material matl;
    if (cylinder_shape->material_list.size() == 0) {
        matl = GetDefaultMaterial();
    } else {
        matl = CreateMaterial(cylinder_shape->material_list[0]);
    }

    // create geometry instance for the cylinder
    GeometryInstance gis = m_context->createGeometryInstance(cylinder, &matl, &matl + 1);
    GeometryGroup geometrygroup = m_context->createGeometryGroup();
    geometrygroup->setChildCount(1);
    geometrygroup->setChild(0, gis);  // add the cylinder geometry to the geometry group

    if (!m_cyl_accel)
        m_cyl_accel = m_context->createAcceleration("trbvh");
    geometrygroup->setAcceleration(m_cyl_accel);
    // geometrygroup->getAcceleration()->setProperty("refit", "1");

    // create a transform that will manipulate the cylinder
    Transform end_point_transform = CreateTransformFromEndPoints(m_context, cylinder_shape->GetCylinderGeometry().p1,
                                                                 cylinder_shape->GetCylinderGeometry().p2, {0, 1, 0},
                                                                 ChVector<>(radius, height, radius));

    Transform asset_transform = CreateTransform(m_context, asset_rot_mat, asset_pos);

    // add the asset transform as child of the body transform
    asset_group->addChild(end_point_transform);
    end_point_transform->setChild(asset_transform);
    asset_transform->setChild(geometrygroup);
}

CH_SENSOR_API optix::Geometry ChOptixEngine::GetOptixCylinderGeometry() {
    if (m_cyl_geometry)
        return m_cyl_geometry;

    m_cyl_geometry = m_context->createGeometry();

    if (!m_cylinder_int)
        m_cylinder_int = GetRTProgram(m_context, "cylinder", "cylinder_intersect");
    if (!m_cylinder_bounds)
        m_cylinder_bounds = GetRTProgram(m_context, "cylinder", "cylinder_bounds");

    m_cyl_geometry->setPrimitiveCount(1u);
    m_cyl_geometry->setBoundingBoxProgram(m_cylinder_bounds);
    m_cyl_geometry->setIntersectionProgram(m_cylinder_int);

    return m_cyl_geometry;
}

void ChOptixEngine::staticTrimeshVisualization(std::shared_ptr<ChTriangleMeshShape> trimesh_shape,
                                               std::shared_ptr<ChVisualization> visual_asset,
                                               optix::Group asset_group) {
    std::shared_ptr<geometry::ChTriangleMeshConnected> mesh = trimesh_shape->GetMesh();
    ChVector<double> asset_pos = visual_asset->Pos;
    ChMatrix33<double> asset_rot_mat = visual_asset->Rot;

    if (trimesh_shape->material_list.size() == 0) {
        // Create a "proper" mesh if one doesn't already exist for it
        CreateModernMeshAssets(trimesh_shape);
    }
    // create all the materials
    // create a material to do diffuse shading
    std::vector<Material> mat_list;
    for (int m = 0; m < trimesh_shape->material_list.size(); m++) {
        Material matl;
        auto vis_mat = trimesh_shape->material_list[m];
        matl = CreateMaterial(vis_mat);

        mat_list.push_back(matl);
    }
    if (mat_list.size() == 0) {
        Material matl;
        matl = GetDefaultMaterial();

        mat_list.push_back(matl);
    }

    if (mesh->m_face_col_indices.size() == 0) {
        mesh->m_face_col_indices = std::vector<ChVector<int>>(mesh->getIndicesVertexes().size());
    }
    // setup mesh buffers
    optix::Buffer vertex_index_buffer =
        m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_INT3, mesh->getIndicesVertexes().size());
    optix::Buffer normal_index_buffer =
        m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_INT3, mesh->getIndicesNormals().size());
    optix::Buffer texcoord_index_buffer =
        m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_INT3, mesh->getIndicesUV().size());
    optix::Buffer mat_index_buffer =
        m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_INT, mesh->getIndicesColors().size());

    optix::Buffer vertex_buffer =
        m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT3, mesh->getCoordsVertices().size());
    optix::Buffer normal_buffer =
        m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT3, mesh->getCoordsNormals().size());
    optix::Buffer texcoord_buffer =
        m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT2, mesh->getCoordsUV().size());

    // copy over the vertex index buffer
    unsigned int* tmp_vertex_index_buffer = (unsigned int*)(vertex_index_buffer->map());
    for (int i = 0; i < mesh->getIndicesVertexes().size(); i++) {
        tmp_vertex_index_buffer[3 * i] = (unsigned int)mesh->getIndicesVertexes().data()[i].x();
        tmp_vertex_index_buffer[3 * i + 1] = (unsigned int)mesh->getIndicesVertexes().data()[i].y();
        tmp_vertex_index_buffer[3 * i + 2] = (unsigned int)mesh->getIndicesVertexes().data()[i].z();
    }
    vertex_index_buffer->unmap();

    // copy over the vertex index buffer
    unsigned int* tmp_normal_index_buffer = (unsigned int*)(normal_index_buffer->map());
    for (int i = 0; i < mesh->getIndicesNormals().size(); i++) {
        tmp_normal_index_buffer[3 * i] = (unsigned int)mesh->getIndicesNormals().data()[i].x();
        tmp_normal_index_buffer[3 * i + 1] = (unsigned int)mesh->getIndicesNormals().data()[i].y();
        tmp_normal_index_buffer[3 * i + 2] = (unsigned int)mesh->getIndicesNormals().data()[i].z();
    }
    normal_index_buffer->unmap();

    // copy over the vertex index buffer
    unsigned int* tmp_texcoord_index_buffer = (unsigned int*)(texcoord_index_buffer->map());
    for (int i = 0; i < mesh->getIndicesUV().size(); i++) {
        tmp_texcoord_index_buffer[3 * i] = (unsigned int)mesh->getIndicesUV().data()[i].x();
        tmp_texcoord_index_buffer[3 * i + 1] = (unsigned int)mesh->getIndicesUV().data()[i].y();
        tmp_texcoord_index_buffer[3 * i + 2] = (unsigned int)mesh->getIndicesUV().data()[i].z();
    }
    texcoord_index_buffer->unmap();

    // copy over the material index buffer
    unsigned int* tmp_mat_index_buffer = (unsigned int*)(mat_index_buffer->map());
    for (int i = 0; i < mesh->getIndicesColors().size(); i++) {
        tmp_mat_index_buffer[i] = (unsigned int)mesh->getIndicesColors().data()[i].x();
    }
    mat_index_buffer->unmap();

    // copy over the vertex buffer
    float* tmp_vertex_buffer = (float*)(vertex_buffer->map());
    for (int i = 0; i < mesh->getCoordsVertices().size(); i++) {
        tmp_vertex_buffer[3 * i] = (float)mesh->getCoordsVertices().data()[i].x();
        tmp_vertex_buffer[3 * i + 1] = (float)mesh->getCoordsVertices().data()[i].y();
        tmp_vertex_buffer[3 * i + 2] = (float)mesh->getCoordsVertices().data()[i].z();
    }
    vertex_buffer->unmap();

    // copy over the normal buffer
    float* tmp_normal_buffer = (float*)(normal_buffer->map());
    for (int i = 0; i < mesh->getCoordsNormals().size(); i++) {
        tmp_normal_buffer[3 * i] = (float)mesh->getCoordsNormals().data()[i].x();
        tmp_normal_buffer[3 * i + 1] = (float)mesh->getCoordsNormals().data()[i].y();
        tmp_normal_buffer[3 * i + 2] = (float)mesh->getCoordsNormals().data()[i].z();
    }
    normal_buffer->unmap();

    // copy over the texcoord buffer
    float* tmp_texcoord_buffer = (float*)(texcoord_buffer->map());
    for (int i = 0; i < mesh->getCoordsUV().size(); i++) {
        tmp_texcoord_buffer[2 * i] = (float)mesh->getCoordsUV().data()[i].x();
        tmp_texcoord_buffer[2 * i + 1] = (float)mesh->getCoordsUV().data()[i].y();
    }
    texcoord_buffer->unmap();

    // create the optix nodes for the triangle mesh
    optix::GeometryTriangles tris = m_context->createGeometryTriangles();
    tris->setPrimitiveCount((unsigned int)mesh->getIndicesVertexes().size());
    tris->setTriangleIndices(vertex_index_buffer, RT_FORMAT_UNSIGNED_INT3);
    tris->setVertices((unsigned int)mesh->getCoordsVertices().size(), vertex_buffer, RT_FORMAT_FLOAT3);
    // tris->setMaterialCount(1);  // TODO: allow multiple materials for a single mesh
    tris->setMaterialCount((unsigned int)mat_list.size());
    tris->setMaterialIndices(mat_index_buffer, 0, sizeof(unsigned), RT_FORMAT_UNSIGNED_INT);

    if (!m_mesh_att)
        m_mesh_att = GetRTProgram(m_context, "triangle_mesh", "mesh_attributes");
    tris->setAttributeProgram(m_mesh_att);

    optix::GeometryInstance triangle_instance = m_context->createGeometryInstance();
    triangle_instance->setGeometryTriangles(tris);
    triangle_instance->setMaterialCount((unsigned int)mat_list.size());  // TODO: allow for multiple materials
    for (int m = 0; m < mat_list.size(); m++) {
        triangle_instance->setMaterial(m, mat_list[m]);
    }

    triangle_instance["vertex_buffer"]->setBuffer(vertex_buffer);
    triangle_instance["normal_buffer"]->setBuffer(normal_buffer);
    triangle_instance["texcoord_buffer"]->setBuffer(texcoord_buffer);
    triangle_instance["vertex_index_buffer"]->setBuffer(vertex_index_buffer);
    triangle_instance["normal_index_buffer"]->setBuffer(normal_index_buffer);
    triangle_instance["texcoord_index_buffer"]->setBuffer(texcoord_index_buffer);
    // triangle_instance["material_buffer"]->setBuffer(mat_index_buffer);

    if (mesh->getIndicesUV().size() == 0 && mesh->getCoordsUV().size() > 0) {
        triangle_instance["texcoord_index_buffer"]->setBuffer(vertex_index_buffer);
    }

    // std::cout << "Static Triangle Mesh Buffers\n";
    // size_t n;
    // vertex_buffer->getSize(n);
    // std::cout << "vertex_buffer: " << n << std::endl;
    // normal_buffer->getSize(n);
    // std::cout << "normal_buffer: " << n << std::endl;
    // texcoord_buffer->getSize(n);
    // std::cout << "texcoord_buffer: " << n << std::endl;
    // vertex_index_buffer->getSize(n);
    // std::cout << "vertex_index_buffer: " << n << std::endl;
    // normal_index_buffer->getSize(n);
    // std::cout << "normal_index_buffer: " << n << std::endl;
    // texcoord_index_buffer->getSize(n);
    // std::cout << "texcoord_index_buffer: " << n << std::endl;

    optix::GeometryGroup triangle_group = m_context->createGeometryGroup();
    triangle_group->setAcceleration(m_context->createAcceleration("trbvh"));
    // triangle_group->getAcceleration()->setProperty("refit", "1");

    // create a transform that will manipulate the sphere
    // Transform asset_transform = m_context->createTransform();
    Transform asset_transform = CreateTransform(m_context, asset_rot_mat, asset_pos);

    // add all the nodes to the graph
    asset_group->addChild(asset_transform);
    asset_transform->setChild(triangle_group);
    triangle_group->addChild(triangle_instance);
}

void ChOptixEngine::dynamicTrimeshVisualization(std::shared_ptr<ChTriangleMeshShape> trimesh_shape,
                                                std::shared_ptr<ChVisualization> visual_asset,
                                                optix::Group asset_group) {
    std::shared_ptr<geometry::ChTriangleMeshConnected> mesh = trimesh_shape->GetMesh();
    ChVector<double> asset_pos = visual_asset->Pos;
    ChMatrix33<double> asset_rot_mat = visual_asset->Rot;

    if (trimesh_shape->material_list.size() == 0) {
        // Create a "proper" mesh if one doesn't already exist for it
        CreateModernMeshAssets(trimesh_shape);
    }

    // create all the materials
    // create a material to do diffuse shading
    std::vector<Material> mat_list;
    for (int m = 0; m < trimesh_shape->material_list.size(); m++) {
        Material matl;
        auto vis_mat = trimesh_shape->material_list[m];
        matl = CreateMaterial(vis_mat);

        mat_list.push_back(matl);
    }
    if (mat_list.size() == 0) {
        Material matl;
        matl = GetDefaultMaterial();

        mat_list.push_back(matl);
    }

    if (mesh->m_face_col_indices.size() == 0) {
        mesh->m_face_col_indices = std::vector<ChVector<int>>(mesh->getIndicesVertexes().size());
    }
    optix::Buffer vertex_index_buffer =
        m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_INT3, mesh->getIndicesVertexes().size());
    optix::Buffer normal_index_buffer =
        m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_INT3, mesh->getIndicesNormals().size());
    optix::Buffer texcoord_index_buffer =
        m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_INT3, mesh->getIndicesUV().size());
    optix::Buffer mat_index_buffer =
        m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_INT, mesh->getIndicesColors().size());

    optix::Buffer vertex_buffer =
        m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT3, mesh->getCoordsVertices().size());
    optix::Buffer normal_buffer =
        m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT3, mesh->getCoordsNormals().size());
    optix::Buffer texcoord_buffer =
        m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT2, mesh->getCoordsUV().size());

    // copy over the vertex index buffer
    unsigned int* tmp_vertex_index_buffer = (unsigned int*)(vertex_index_buffer->map());
    for (int i = 0; i < mesh->getIndicesVertexes().size(); i++) {
        tmp_vertex_index_buffer[3 * i] = mesh->getIndicesVertexes().data()[i].x();
        tmp_vertex_index_buffer[3 * i + 1] = mesh->getIndicesVertexes().data()[i].y();
        tmp_vertex_index_buffer[3 * i + 2] = mesh->getIndicesVertexes().data()[i].z();
    }
    vertex_index_buffer->unmap();

    // copy over the vertex index buffer
    unsigned int* tmp_normal_index_buffer = (unsigned int*)(normal_index_buffer->map());
    for (int i = 0; i < mesh->getIndicesNormals().size(); i++) {
        tmp_normal_index_buffer[3 * i] = mesh->getIndicesNormals().data()[i].x();
        tmp_normal_index_buffer[3 * i + 1] = mesh->getIndicesNormals().data()[i].y();
        tmp_normal_index_buffer[3 * i + 2] = mesh->getIndicesNormals().data()[i].z();
    }
    normal_index_buffer->unmap();

    // copy over the vertex index buffer
    unsigned int* tmp_texcoord_index_buffer = (unsigned int*)(texcoord_index_buffer->map());
    for (int i = 0; i < mesh->getIndicesUV().size(); i++) {
        tmp_texcoord_index_buffer[3 * i] = mesh->getIndicesUV().data()[i].x();
        tmp_texcoord_index_buffer[3 * i + 1] = mesh->getIndicesUV().data()[i].y();
        tmp_texcoord_index_buffer[3 * i + 2] = mesh->getIndicesUV().data()[i].z();
    }
    texcoord_index_buffer->unmap();

    // copy over the material index buffer
    unsigned int* tmp_mat_index_buffer = (unsigned int*)(mat_index_buffer->map());
    for (int i = 0; i < mesh->getIndicesColors().size(); i++) {
        tmp_mat_index_buffer[i] = mesh->getIndicesColors().data()[i].x();
    }
    mat_index_buffer->unmap();

    // copy over the vertex buffer
    float* tmp_vertex_buffer = (float*)(vertex_buffer->map());
    for (int i = 0; i < mesh->getCoordsVertices().size(); i++) {
        tmp_vertex_buffer[3 * i] = (float)mesh->getCoordsVertices().data()[i].x();
        tmp_vertex_buffer[3 * i + 1] = (float)mesh->getCoordsVertices().data()[i].y();
        tmp_vertex_buffer[3 * i + 2] = (float)mesh->getCoordsVertices().data()[i].z();
    }
    vertex_buffer->unmap();

    // copy over the normal buffer
    float* tmp_normal_buffer = (float*)(normal_buffer->map());
    for (int i = 0; i < mesh->getCoordsNormals().size(); i++) {
        tmp_normal_buffer[3 * i] = (float)mesh->getCoordsNormals().data()[i].x();
        tmp_normal_buffer[3 * i + 1] = (float)mesh->getCoordsNormals().data()[i].y();
        tmp_normal_buffer[3 * i + 2] = (float)mesh->getCoordsNormals().data()[i].z();
    }
    normal_buffer->unmap();

    // copy over the texcoord buffer
    float* tmp_texcoord_buffer = (float*)(texcoord_buffer->map());
    for (int i = 0; i < mesh->getCoordsUV().size(); i++) {
        tmp_texcoord_buffer[2 * i] = (float)mesh->getCoordsUV().data()[i].x();
        tmp_texcoord_buffer[2 * i + 1] = (float)mesh->getCoordsUV().data()[i].y();
    }
    texcoord_buffer->unmap();

    // create the optix nodes for the triangle mesh
    optix::GeometryTriangles tris = m_context->createGeometryTriangles();
    tris->setPrimitiveCount((unsigned int)mesh->getIndicesVertexes().size());
    tris->setTriangleIndices(vertex_index_buffer, RT_FORMAT_UNSIGNED_INT3);
    tris->setVertices((unsigned int)mesh->getCoordsVertices().size(), vertex_buffer, RT_FORMAT_FLOAT3);
    // tris->setMaterialCount(1);  // TODO: allow multiple materials for a single mesh
    tris->setMaterialCount((unsigned int)mat_list.size());
    tris->setMaterialIndices(mat_index_buffer, 0, sizeof(unsigned), RT_FORMAT_UNSIGNED_INT);

    if (!m_mesh_att)
        m_mesh_att = GetRTProgram(m_context, "triangle_mesh", "mesh_attributes");
    tris->setAttributeProgram(m_mesh_att);

    optix::GeometryInstance triangle_instance = m_context->createGeometryInstance();
    triangle_instance->setGeometryTriangles(tris);
    triangle_instance->setMaterialCount((unsigned int)mat_list.size());  // TODO: allow for multiple materials
    for (int m = 0; m < mat_list.size(); m++) {
        triangle_instance->setMaterial(m, mat_list[m]);
    }

    triangle_instance["vertex_buffer"]->setBuffer(vertex_buffer);
    triangle_instance["normal_buffer"]->setBuffer(normal_buffer);
    triangle_instance["texcoord_buffer"]->setBuffer(texcoord_buffer);
    triangle_instance["vertex_index_buffer"]->setBuffer(vertex_index_buffer);
    triangle_instance["normal_index_buffer"]->setBuffer(normal_index_buffer);
    triangle_instance["texcoord_index_buffer"]->setBuffer(texcoord_index_buffer);

    if (mesh->getIndicesUV().size() == 0 && mesh->getCoordsUV().size() > 0) {
        triangle_instance["texcoord_index_buffer"]->setBuffer(vertex_index_buffer);
    }

    // std::cout << "=== Deformable Triangle Mesh Buffers ===\n";
    // size_t n;
    // vertex_buffer->getSize(n);
    // std::cout << "vertex_buffer: " << n << std::endl;
    // normal_buffer->getSize(n);
    // std::cout << "normal_buffer: " << n << std::endl;
    // texcoord_buffer->getSize(n);
    // std::cout << "texcoord_buffer: " << n << std::endl;
    // vertex_index_buffer->getSize(n);
    // std::cout << "vertex_index_buffer: " << n << std::endl;
    // normal_index_buffer->getSize(n);
    // std::cout << "normal_index_buffer: " << n << std::endl;
    // texcoord_index_buffer->getSize(n);
    // std::cout << "texcoord_index_buffer: " << n << std::endl;

    optix::GeometryGroup triangle_group = m_context->createGeometryGroup();
    triangle_group->setAcceleration(m_context->createAcceleration("trbvh"));
    // triangle_group->getAcceleration()->setProperty("refit", "1");

    // create a transform that will manipulate the sphere
    // Transform asset_transform = m_context->createTransform();
    Transform asset_transform = CreateTransform(m_context, asset_rot_mat, asset_pos);

    // add all the nodes to the graph
    asset_group->addChild(asset_transform);
    asset_transform->setChild(triangle_group);
    triangle_group->addChild(triangle_instance);

    std::shared_ptr<DynamicMesh> dmesh(new DynamicMesh(mesh, triangle_group, vertex_buffer, normal_buffer));
    m_dynamicMeshes.push_back(dmesh);
}

void ChOptixEngine::Initialize() {
    // initialize an optix context -> one for each render group
    m_context = Context::create();

    rtContextSetExceptionEnabled(m_context->get(), RT_EXCEPTION_ALL, m_verbose);

    unsigned int n_devices;
    rtContextGetDeviceCount(m_context->get(), &n_devices);
    std::vector<int> dev_ids = std::vector<int>(n_devices);
    rtContextGetDevices(m_context->get(), dev_ids.data());
    if (m_verbose) {
        std::cout << "Devices available: ";
        for (unsigned int i = 0; i < n_devices; i++) {
            std::cout << i << ", ";
        }
        std::cout << std::endl;
        rtContextSetPrintEnabled(m_context->get(), 1);
    }
    if (m_deviceId > n_devices - 1) {
        std::cerr << "Requested GPU not available, falling back on RT Device 0\n";
        m_deviceId = 0;
    }

    rtContextSetDevices(m_context->get(), 1, &dev_ids[m_deviceId]);

    int n;
    if (rtContextGetDevices(m_context->get(), &n) == RT_SUCCESS && m_verbose)
        std::cout << "Creating an optix context. GPU requested: " << m_deviceId << ", GPU used: " << n << std::endl;

    // try to enable RTX acceleration if it is available
    int rtx = 1;
    if (rtGlobalSetAttribute(RT_GLOBAL_ATTRIBUTE_ENABLE_RTX, sizeof(int), &rtx) != RT_SUCCESS)
        printf("RTX Mode Disabled on this device. \n");

    // set context-wide parameters -> make changeable by functions before initialization
    m_context->setRayTypeCount(3);  // 0=camera, 1=shadow, 2=lidar
    m_context->setMaxTraceDepth(
        m_recursions);  // max allowable by OptiX, doesn't mean we should try to go this far though

    m_context["max_scene_distance"]->setFloat(1.e4f);
    m_context["max_depth"]->setInt(m_recursions);
    m_context["scene_epsilon"]->setFloat(1.0e-3f);
    m_context["importance_cutoff"]->setFloat(1 / 255.0f);
    m_context["ambient_light_color"]->setFloat(0.2f, 0.2f, 0.2f);

    // full noise buffer will be make later as needed
    optix::Buffer dummy_buffer = m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT, 1);
    m_context["noise_buffer"]->setBuffer(dummy_buffer);

    m_root = m_context->createGroup();
    m_root->setAcceleration(m_context->createAcceleration("trbvh"));
    // m_root->getAcceleration()->setProperty("refit", "1");
    m_context["root_node"]->set(m_root);

    if (!m_camera_miss)
        m_camera_miss = GetRTProgram(m_context, "miss", "camera_miss");
    if (!m_lidar_miss)
        m_lidar_miss = GetRTProgram(m_context, "miss", "lidar_miss");

    m_context->setMissProgram(0, m_camera_miss);
    m_context->setMissProgram(2, m_lidar_miss);

    m_camera_miss["default_color"]->setFloat(0.5f, 0.6f, 0.7f);
    m_lidar_miss["default_depth"]->setFloat(-1.f);

    TextureSampler tex_sampler = CreateTexture();
    m_camera_miss["environment_map"]->setTextureSampler(tex_sampler);
    m_camera_miss["has_environment_map"]->setInt(0);

    m_light_buffer = m_context->createBuffer(RT_BUFFER_INPUT);
    m_light_buffer->setFormat(RT_FORMAT_USER);

    m_light_buffer->setElementSize(sizeof(PointLight));
    m_light_buffer->setSize(0);

    m_context["lights"]->set(m_light_buffer);

    // m_context->setStackSize(0);

    // std::cout << "Context stack size: " << m_context->getStackSize() << std::endl;
    // std::cout << "Context call depth size: " << m_context->getMaxCallableProgramDepth() << std::endl;
}

void ChOptixEngine::AddInstancedStaticSceneMeshes(std::vector<ChFrame<>>& frames,
                                                  std::shared_ptr<ChTriangleMeshShape> mesh) {
    // std::unique_lock<std::mutex> tmp_lock(m_renderQueueMutex);

    std::lock_guard<std::mutex> lck(m_renderQueueMutex);

    optix::Group asset_group = m_context->createGroup();
    staticTrimeshVisualization(mesh, std::dynamic_pointer_cast<ChVisualization>(mesh), asset_group);

    unsigned int i = 0;
    optix::GeometryGroup g = asset_group->getChild<Transform>(i)->getChild<GeometryGroup>();

    for (auto f : frames) {
        const ChVector<double> pos = f.GetPos();
        const ChMatrix33<double> rot_mat = f.Amatrix;

        Transform mesh_transform = CreateTransform(m_context, rot_mat, pos);

        m_root->addChild(mesh_transform);
        mesh_transform->setChild(g);
    }

    m_root->getAcceleration()->markDirty();
}

void ChOptixEngine::ConstructScene() {
    // need to lock before touching any optix stuff
    std::lock_guard<std::mutex> lck(m_renderQueueMutex);

    // update what objects are in scene
    // remove all children from root node
    while (m_root->getChildCount() > 0) {
        m_root->removeChild(0);
    }

    // clear all the old bodies
    m_bodies.clear();

    // clear all the old keyframes for bodies and cameras (sensors)
    m_keyframes.clear();
    m_camera_keyframes.clear();

    // create the programs for each geometry -> TODO: refactor so only only one instance of a geometry or material
    // is made

    // iterate through all bodies in Chrono and add a subnode for each body in Chrono
    for (auto body : m_system->Get_bodylist()) {
        // check that the body list is not empty
        if (body->GetAssets().size() > 0) {
            // collect position and orientation of the body from Chrono
            const ChVector<double> body_pos = body->GetFrame_REF_to_abs().GetPos();
            const ChMatrix33<double> body_rot_mat = body->GetFrame_REF_to_abs().Amatrix;

            // set the transform that is applicable to the entire body
            // Transform body_transform = CreateTransform(m_context, body_rot_mat, body_pos);
            Transform body_transform = CreateEmptyTransform(m_context);

            bool added_asset_for_body = false;

            optix::Group asset_group = m_context->createGroup();
            asset_group->setAcceleration(m_context->createAcceleration("trbvh"));
            // asset_group->getAcceleration()->setProperty("refit", "1");

            body_transform->setChild(asset_group);

            // iterate through all assets in the body
            for (auto asset : body->GetAssets()) {
                // check if the asset is a ChVisualization

                if (std::shared_ptr<ChVisualization> visual_asset = std::dynamic_pointer_cast<ChVisualization>(asset)) {
                    // collect relative position and orientation of the asset
                    ChVector<double> asset_pos = visual_asset->Pos;
                    ChMatrix33<double> asset_rot_mat = visual_asset->Rot;

                    if (!visual_asset->IsVisible()) {
                        std::cout << "Ignoring an asset that is set to invisible\n";
                    } else if (std::shared_ptr<ChBoxShape> box_shape = std::dynamic_pointer_cast<ChBoxShape>(asset)) {
                        boxVisualization(box_shape, visual_asset, asset_group);
                        added_asset_for_body = true;

                    } else if (std::shared_ptr<ChSphereShape> sphere_shape =
                                   std::dynamic_pointer_cast<ChSphereShape>(asset)) {
                        sphereVisualization(sphere_shape, visual_asset, asset_group);
                        added_asset_for_body = true;

                    } else if (std::shared_ptr<ChCylinderShape> cylinder_shape =
                                   std::dynamic_pointer_cast<ChCylinderShape>(asset)) {
                        cylinderVisualization(cylinder_shape, visual_asset, asset_group);
                        added_asset_for_body = true;

                    } else if (std::shared_ptr<ChTriangleMeshShape> trimesh_shape =
                                   std::dynamic_pointer_cast<ChTriangleMeshShape>(asset)) {
                        if (trimesh_shape->IsStatic()) {
                            staticTrimeshVisualization(trimesh_shape, visual_asset, asset_group);
                            //                            std::cout<<"mesh is stored"<<std::endl;
                            added_asset_for_body = true;
                        } else {
                            dynamicTrimeshVisualization(trimesh_shape, visual_asset, asset_group);
                        }

                    } else if (std::shared_ptr<ChEllipsoidShape> ellipsoid_shape =
                                   std::dynamic_pointer_cast<ChEllipsoidShape>(asset)) {
                    } else if (std::shared_ptr<ChCylinderShape> cylinder_shape =
                                   std::dynamic_pointer_cast<ChCylinderShape>(asset)) {
                    } else if (std::shared_ptr<ChConeShape> cone_shape =
                                   std::dynamic_pointer_cast<ChConeShape>(asset)) {
                    } else if (std::shared_ptr<ChRoundedBoxShape> shape =
                                   std::dynamic_pointer_cast<ChRoundedBoxShape>(asset)) {
                    } else if (std::shared_ptr<ChCapsuleShape> capsule_shape =
                                   std::dynamic_pointer_cast<ChCapsuleShape>(asset)) {
                    } else if (std::shared_ptr<ChTriangleMeshShape> trimesh_shape =
                                   std::dynamic_pointer_cast<ChTriangleMeshShape>(asset)) {
                    } else if (std::shared_ptr<ChPathShape> path_shape =
                                   std::dynamic_pointer_cast<ChPathShape>(asset)) {
                    } else if (std::shared_ptr<ChLineShape> line_shape =
                                   std::dynamic_pointer_cast<ChLineShape>(asset)) {
                    }

                    // check if the asset is a ChColorAsset
                    else if (std::shared_ptr<ChColorAsset> visual_asset =
                                 std::dynamic_pointer_cast<ChColorAsset>(asset)) {
                        // std::cout << "Asset was color\n";
                    }

                    // check if the asset is a ChTexture
                    else if (std::shared_ptr<ChTexture> visual_asset = std::dynamic_pointer_cast<ChTexture>(asset)) {
                        // std::cout << "Asset was texture\n";
                    }
                }

                if (added_asset_for_body) {
                    // add node to root and signify need for a rebuild
                    m_root->addChild(body_transform);
                    m_root->getAcceleration()->markDirty();

                    // keep a handle to the body and transform pair for updating
                    m_bodies.push_back(std::pair<std::shared_ptr<ChBody>, optix::Transform>(body, body_transform));
                }
            }
        }
    }

    // Assumption made here that other physics items don't have a transform -> not always true!!!
    for (auto item : m_system->Get_otherphysicslist()) {
        // add items one by one

        if (item->GetAssets().size() > 0) {
            optix::Group asset_group = m_context->createGroup();
            asset_group->setAcceleration(m_context->createAcceleration("trbvh"));
            // asset_group->getAcceleration()->setProperty("refit", "1");
            bool added_asset_to_root = false;
            // iterate through all visualization assets in the item
            for (auto asset : item->GetAssets()) {
                if (std::shared_ptr<ChVisualization> visual_asset = std::dynamic_pointer_cast<ChVisualization>(asset)) {
                    ChVector<double> asset_pos = visual_asset->Pos;
                    ChMatrix33<double> asset_rot_mat = visual_asset->Rot;

                    if (!visual_asset->IsVisible()) {
                        std::cout << "Ignoring an asset that is set to invisible in otherphysicslist\n";
                    } else if (std::shared_ptr<ChBoxShape> box_shape = std::dynamic_pointer_cast<ChBoxShape>(asset)) {
                        // add box to somesort of data structure
                        boxVisualization(box_shape, visual_asset, asset_group);
                        added_asset_to_root = true;
                    } else if (std::shared_ptr<ChCylinderShape> cylinder_shape =
                                   std::dynamic_pointer_cast<ChCylinderShape>(asset)) {
                        // add cylinder to data structre
                        cylinderVisualization(cylinder_shape, visual_asset, asset_group);
                        added_asset_to_root = true;
                    } else if (std::shared_ptr<ChSphereShape> sphere_shape =
                                   std::dynamic_pointer_cast<ChSphereShape>(asset)) {
                        // add sphere to data structure
                        sphereVisualization(sphere_shape, visual_asset, asset_group);
                        added_asset_to_root = true;
                    } else if (std::shared_ptr<ChTriangleMeshShape> trimesh_shape =
                                   std::dynamic_pointer_cast<ChTriangleMeshShape>(asset)) {
                        if (trimesh_shape->IsStatic()) {
                            staticTrimeshVisualization(trimesh_shape, visual_asset, asset_group);
                            added_asset_to_root = true;
                        } else {
                            dynamicTrimeshVisualization(trimesh_shape, visual_asset, asset_group);
                            added_asset_to_root = true;
                        }
                    }

                    if (added_asset_to_root) {
                        m_root->addChild(asset_group);
                        m_root->getAcceleration()->markDirty();
                    }
                }
            }
        }
    }
}

void ChOptixEngine::UpdateCameraTransforms() {
    // go through all of the bodies
    for (int i = 0; i < m_assignedSensor.size(); i++) {
        auto sensor = m_assignedSensor[i];
        Program ray_gen = sensor->m_ray_gen;

        float end_time = std::get<0>(m_camera_keyframes[m_camera_keyframes.size() - 1]);
        float start_time = end_time - m_assignedSensor[i]->GetCollectionWindow();
        ray_gen["start_time"]->setFloat(start_time);
        ray_gen["end_time"]->setFloat(end_time);

        // find index of camera transform that corresponds to sensor start time
        int start_index = 0;
        for (int j = (int)m_camera_keyframes.size() - 1; j >= 0; j--) {
            if (std::get<0>(m_camera_keyframes[j]) < start_time + 1e-6) {
                start_index = j;
                break;
            }
        }

        std::vector<float> pose_0 = std::get<1>(m_camera_keyframes[start_index])[i];
        std::vector<float> pose_1 = std::get<1>(m_camera_keyframes[m_camera_keyframes.size() - 1])[i];

        ray_gen["origin_0"]->setFloat(pose_0[0], pose_0[1], pose_0[2]);  // origin_0
        ray_gen["origin_1"]->setFloat(pose_1[0], pose_1[1], pose_1[2]);  // origin_0

        ray_gen["rot_0"]->setFloat(pose_0[3], pose_0[4], pose_0[5], pose_0[6]);  // origin_0
        ray_gen["rot_1"]->setFloat(pose_1[3], pose_1[4], pose_1[5], pose_1[6]);  // origin_0
    }
}

void ChOptixEngine::UpdateBodyTransforms() {
    m_internal_keyframes.clear();

    // create a flat vector for all the transforms
    m_internal_keyframes = std::vector<float>(m_bodies.size() * m_keyframes.size() * 12);

    // go through all of the bodies
    for (int i = 0; i < m_bodies.size(); i++) {
        // get the transform associated with the body
        optix::Transform t = m_bodies[i].second;

        t->setMotionRange(std::get<0>(m_keyframes[0]), std::get<0>(m_keyframes[m_keyframes.size() - 1]));
        // update all the keyframes for that transform
        for (int j = 0; j < m_keyframes.size(); j++) {
            for (int k = 0; k < 12; k++) {
                m_internal_keyframes[i * m_keyframes.size() * 12 + j * 12 + k] = std::get<1>(m_keyframes[j])[i][k];
            }
        }

        auto res = rtTransformSetMotionKeys(t->get(), (unsigned int)m_keyframes.size(), RT_MOTIONKEYTYPE_MATRIX_FLOAT12,
                                            &m_internal_keyframes[i * m_keyframes.size() * 12]);
        t->setMotionBorderMode(RT_MOTIONBORDERMODE_CLAMP, RT_MOTIONBORDERMODE_CLAMP);
    }
    // mark the transform for rebuild
    m_root->getAcceleration()->markDirty();
}

void ChOptixEngine::PackKeyFrames() {
    // update the transforms for objects already in optix

    // pack the bodies keyframe
    std::vector<std::vector<float>> keyframe;
    for (auto bodyPair : m_bodies) {
        const ChVector<double> b = bodyPair.first->GetFrame_REF_to_abs().GetPos();
        const ChMatrix33<double> a = bodyPair.first->GetFrame_REF_to_abs().Amatrix;
        std::vector<float> transform = {(float)a(0), (float)a(1), (float)a(2), (float)b.x(),
                                        (float)a(3), (float)a(4), (float)a(5), (float)b.y(),
                                        (float)a(6), (float)a(7), (float)a(8), (float)b.z()};

        keyframe.push_back(transform);
    }
    m_keyframes.push_back(std::make_tuple((float)m_system->GetChTime(), keyframe));

    // make sure we have at least two keyframes, otherwise the transform is illdefined
    while (m_keyframes.size() < 2) {
        m_keyframes.push_back(std::make_tuple((float)m_system->GetChTime(), keyframe));
    }

    // pack the cameras keyframe
    std::vector<std::vector<float>> cam_keyframe;
    for (auto sensor : m_assignedSensor) {
        ChFrame<double> f_offset = sensor->GetOffsetPose();
        ChFrame<double> f_body = sensor->GetParent()->GetAssetsFrame();
        ChFrame<double> global_loc = f_body * f_offset;

        const ChVector<double> pos = global_loc.GetPos();
        const ChQuaternion<double> rot = global_loc.GetRot();

        std::vector<float> transform = {(float)pos.x(),  (float)pos.y(),  (float)pos.z(), (float)rot.e0(),
                                        (float)rot.e1(), (float)rot.e2(), (float)rot.e3()};

        cam_keyframe.push_back(transform);
    }
    m_camera_keyframes.push_back(std::make_tuple((float)m_system->GetChTime(), cam_keyframe));

    // make sure we have at least two keyframes, otherwise the transform is illdefined
    while (m_camera_keyframes.size() < 2) {
        m_camera_keyframes.push_back(std::make_tuple((float)m_system->GetChTime(), cam_keyframe));
    }

    // maintain keyframe queue
    while (m_keyframes.size() > m_max_keyframes_needed) {
        m_keyframes.pop_front();
    }

    // maintain camera keyframe queue
    while (m_camera_keyframes.size() > m_max_keyframes_needed) {
        m_camera_keyframes.pop_front();
    }
}

void ChOptixEngine::UpdateDynamicMeshes() {
    for (auto dynamicMeshes : m_dynamicMeshes) {
        // access to chrono mesh and optix buffers
        std::shared_ptr<geometry::ChTriangleMeshConnected> mesh = dynamicMeshes->getMesh();
        optix::Buffer normal_buffer = dynamicMeshes->getNormalBuffer();
        optix::Buffer vertex_buffer = dynamicMeshes->getVertexBuffer();

        // get device pointers
        float* host_normal_buffer = dynamicMeshes->host_normalBuffer_ptr();
        float* host_vertex_buffer = dynamicMeshes->host_vertexBuffer_ptr();
        float* vertex_buffer_device_ptr = (float*)dynamicMeshes->device_vertexBuffer_ptr();
        float* normal_buffer_device_ptr = (float*)dynamicMeshes->device_normalBuffer_ptr();

        int size = dynamicMeshes->getSize();

        int start_index = 0;
        int end_index = 0;
        bool assign = true;

        for (int i = 0; i < size; i++) {
            if (abs(host_vertex_buffer[3 * i + 2] - (float)mesh->getCoordsVertices()[i].z()) > 1e-6) {
                if (assign) {
                    assign = false;
                    start_index = i;
                }
                host_normal_buffer[3 * i] = (float)mesh->getCoordsNormals()[i].x();
                host_normal_buffer[3 * i + 1] = (float)mesh->getCoordsNormals()[i].y();
                host_normal_buffer[3 * i + 2] = (float)mesh->getCoordsNormals()[i].z();
                host_vertex_buffer[3 * i] = (float)mesh->getCoordsVertices()[i].x();
                host_vertex_buffer[3 * i + 1] = (float)mesh->getCoordsVertices()[i].y();
                host_vertex_buffer[3 * i + 2] = (float)mesh->getCoordsVertices()[i].z();
                end_index = i;
            }
        }

        cudaMemcpy(&vertex_buffer_device_ptr[3 * start_index], &host_vertex_buffer[3 * start_index],
                   3 * sizeof(float) * (end_index - start_index), cudaMemcpyHostToDevice);

        cudaMemcpy(&normal_buffer_device_ptr[3 * start_index], &host_normal_buffer[3 * start_index],
                   3 * sizeof(float) * (end_index - start_index), cudaMemcpyHostToDevice);

        dynamicMeshes->getTriGroup()->getAcceleration()->markDirty();
    }
}

void ChOptixEngine::UpdateSceneDescription(std::shared_ptr<ChScene> scene) {
    // get the information that we can handle from the scene and set those properties in OptiX

    // create lights -> TODO: move to own function to add more and remove them on the fly

    m_light_buffer->setElementSize(sizeof(PointLight));
    m_light_buffer->setSize(scene->GetPointLights().size());
    memcpy(m_light_buffer->map(), scene->GetPointLights().data(), scene->GetPointLights().size() * sizeof(PointLight));
    m_light_buffer->unmap();

    m_context["lights"]->set(m_light_buffer);

    // we need to check if things have changed before we just go ahead and overwrite
    if (scene->GetBackground().has_changed) {
        if (!m_camera_miss)
            m_camera_miss = GetRTProgram(m_context, "miss", "camera_miss");
        m_context->setMissProgram(0, m_camera_miss);
        m_camera_miss["default_color"]->setFloat(scene->GetBackground().color.x(), scene->GetBackground().color.y(),
                                                 scene->GetBackground().color.z());
        if (scene->GetBackground().has_texture) {
            TextureSampler tex_sampler = CreateTexture(GetChronoDataFile(scene->GetBackground().env_tex));
            m_camera_miss["environment_map"]->setTextureSampler(tex_sampler);
            m_camera_miss["has_environment_map"]->setInt(1);
        } else {
            TextureSampler tex_sampler = CreateTexture();
            m_camera_miss["environment_map"]->setTextureSampler(tex_sampler);
            m_camera_miss["has_environment_map"]->setInt(0);
        }

        scene->GetBackground().has_changed = false;
    }
}

Material ChOptixEngine::GetDefaultMaterial() {
    if (m_default_material)
        return m_default_material;

    m_default_material = m_context->createMaterial();

    // use the single pbr shader that can handle reflectivity and transparency if it exists
    if (!m_camera_shader)
        m_camera_shader = GetRTProgram(m_context, "camera_shaders", "pbr_shader");

    if (!m_shadow_shader)
        m_shadow_shader = GetRTProgram(m_context, "shadow_shaders", "hit_shadow");

    if (!m_lidar_shader)
        m_lidar_shader = GetRTProgram(m_context, "lidar_shaders", "diffuse_shader");

    m_default_material["Ka"]->setFloat(.2f, .2f, .2f);
    m_default_material["Kd"]->setFloat(.5f, .5f, .5f);
    m_default_material["Ks"]->setFloat(.5f, .5f, .5f);
    m_default_material["phong_exp"]->setFloat(88.f);
    m_default_material["fresnel_exp"]->setFloat(5.f);
    m_default_material["fresnel_min"]->setFloat(0.f);
    m_default_material["fresnel_max"]->setFloat(1.f);
    m_default_material["transparency"]->setFloat(1.0);
    m_default_material["roughness"]->setFloat(.5);

    if (!m_empty_tex_sampler)
        m_empty_tex_sampler = CreateTexture();

    m_default_material["Kd_map"]->setTextureSampler(m_empty_tex_sampler);
    m_default_material["has_texture"]->setInt(0);

    m_default_material["normal_map"]->setTextureSampler(m_empty_tex_sampler);
    m_default_material["has_normal_map"]->setInt(0);

    m_default_material->setClosestHitProgram(0, m_camera_shader);
    m_default_material->setAnyHitProgram(1, m_shadow_shader);
    m_default_material->setClosestHitProgram(2, m_lidar_shader);

    m_default_material->validate();

    return m_default_material;
}

Material ChOptixEngine::CreateMaterial(std::shared_ptr<ChVisualMaterial> chmat) {
    optix::Material mat = m_context->createMaterial();

    // use the single pbr shader that can handle reflectivity and transparency if it exists
    if (!m_camera_shader)
        m_camera_shader = GetRTProgram(m_context, "camera_shaders", "pbr_shader");
    if (!m_shadow_shader)
        m_shadow_shader = GetRTProgram(m_context, "shadow_shaders", "hit_shadow");
    if (!m_lidar_shader)
        m_lidar_shader = GetRTProgram(m_context, "lidar_shaders", "diffuse_shader");

    mat["Ka"]->setFloat(chmat->GetAmbientColor().x(), chmat->GetAmbientColor().y(), chmat->GetAmbientColor().z());
    mat["Kd"]->setFloat(chmat->GetDiffuseColor().x(), chmat->GetDiffuseColor().y(), chmat->GetDiffuseColor().z());
    mat["Ks"]->setFloat(chmat->GetSpecularColor().x(), chmat->GetSpecularColor().y(), chmat->GetSpecularColor().z());
    mat["phong_exp"]->setFloat(chmat->GetSpecularExponent());
    mat["fresnel_exp"]->setFloat(chmat->GetFresnelExp());
    mat["fresnel_min"]->setFloat(chmat->GetFresnelMin());
    mat["fresnel_max"]->setFloat(chmat->GetFresnelMax());
    mat["transparency"]->setFloat(chmat->GetTransparency());
    mat["roughness"]->setFloat(chmat->GetRoughness());

    if (chmat->GetKdTexture() != "") {
        TextureSampler tex_sampler = CreateTexture(chmat->GetKdTexture());
        mat["Kd_map"]->setTextureSampler(tex_sampler);
        mat["has_texture"]->setInt(1);
    } else {
        if (!m_empty_tex_sampler) {
            m_empty_tex_sampler = CreateTexture();
        }
        mat["Kd_map"]->setTextureSampler(m_empty_tex_sampler);
        mat["has_texture"]->setInt(0);
    }

    if (chmat->GetNormalMapTexture() != "") {
        TextureSampler tex_sampler = CreateTexture(chmat->GetNormalMapTexture());
        mat["normal_map"]->setTextureSampler(tex_sampler);
        mat["has_normal_map"]->setInt(1);
        // std::cout<< "has normal map"<<std::endl;
    } else {
        if (!m_empty_tex_sampler) {
            m_empty_tex_sampler = CreateTexture();
        }
        mat["normal_map"]->setTextureSampler(m_empty_tex_sampler);
        mat["has_normal_map"]->setInt(0);
    }

    mat->setClosestHitProgram(0, m_camera_shader);
    mat->setAnyHitProgram(1, m_shadow_shader);
    mat->setClosestHitProgram(2, m_lidar_shader);
    return mat;
}

TextureSampler ChOptixEngine::CreateTexture(std::string filename) {
    TextureSampler tex_sampler = m_context->createTextureSampler();
    tex_sampler->setWrapMode(0, RT_WRAP_REPEAT);
    tex_sampler->setWrapMode(1, RT_WRAP_REPEAT);
    tex_sampler->setWrapMode(2, RT_WRAP_REPEAT);
    tex_sampler->setIndexingMode(RT_TEXTURE_INDEX_NORMALIZED_COORDINATES);
    tex_sampler->setReadMode(RT_TEXTURE_READ_NORMALIZED_FLOAT);
    tex_sampler->setMaxAnisotropy(1.0f);
    tex_sampler->setMipLevelCount(1u);
    tex_sampler->setArraySize(1u);
    tex_sampler->setFilteringModes(RT_FILTER_LINEAR, RT_FILTER_LINEAR, RT_FILTER_NONE);

    // load texture file
    ByteImageData img = LoadImage(filename);

    // if image loading fails, create a default failure texture
    if (img.h == 0 || img.w == 0 || img.c == 0) {
        Buffer buffer = m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_BYTE4, 1u, 1u);
        unsigned char* tmp_buffer = static_cast<unsigned char*>(buffer->map());
        tmp_buffer[0] = 255;
        tmp_buffer[1] = 0;
        tmp_buffer[2] = 255;
        tmp_buffer[3] = 255;
        buffer->unmap();

        tex_sampler->setBuffer(0u, 0u, buffer);

        return tex_sampler;
    }

    Buffer buffer = m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_BYTE4, img.w, img.h);
    if (img.c == 4) {
        unsigned char* tmp_buffer = static_cast<unsigned char*>(buffer->map());
        for (int i = 0; i < img.h; i++) {
            for (int j = 0; j < img.w; j++) {
                tmp_buffer[i * img.w * 4 + j * 4 + 0] = img.data[(img.h - i - 1) * img.w * 4 + j * 4 + 0];
                tmp_buffer[i * img.w * 4 + j * 4 + 1] = img.data[(img.h - i - 1) * img.w * 4 + j * 4 + 1];
                tmp_buffer[i * img.w * 4 + j * 4 + 2] = img.data[(img.h - i - 1) * img.w * 4 + j * 4 + 2];
                tmp_buffer[i * img.w * 4 + j * 4 + 3] = img.data[(img.h - i - 1) * img.w * 4 + j * 4 + 3];
            }
        }
    } else if (img.c == 3) {
        unsigned char* tmp_buffer = static_cast<unsigned char*>(buffer->map());
        for (int i = 0; i < img.h; i++) {
            for (int j = 0; j < img.w; j++) {
                tmp_buffer[i * img.w * 4 + j * 4 + 0] = img.data[(img.h - i - 1) * img.w * 3 + j * 3 + 0];
                tmp_buffer[i * img.w * 4 + j * 4 + 1] = img.data[(img.h - i - 1) * img.w * 3 + j * 3 + 1];
                tmp_buffer[i * img.w * 4 + j * 4 + 2] = img.data[(img.h - i - 1) * img.w * 3 + j * 3 + 2];
                tmp_buffer[i * img.w * 4 + j * 4 + 3] = 255;
            }
        }
    } else if (img.c == 2) {
        unsigned char* tmp_buffer = static_cast<unsigned char*>(buffer->map());
        for (int i = 0; i < img.h; i++) {
            for (int j = 0; j < img.w; j++) {
                tmp_buffer[i * img.w * 4 + j * 4 + 0] = img.data[(img.h - i - 1) * img.w * 2 + j * 2 + 0];
                tmp_buffer[i * img.w * 4 + j * 4 + 1] = img.data[(img.h - i - 1) * img.w * 2 + j * 2 + 0];
                tmp_buffer[i * img.w * 4 + j * 4 + 2] = img.data[(img.h - i - 1) * img.w * 2 + j * 2 + 0];
                tmp_buffer[i * img.w * 4 + j * 4 + 3] = img.data[(img.h - i - 1) * img.w * 2 + j * 2 + 1];
            }
        }
    } else if (img.c == 1) {
        unsigned char* tmp_buffer = static_cast<unsigned char*>(buffer->map());
        for (int i = 0; i < img.h; i++) {
            for (int j = 0; j < img.w; j++) {
                tmp_buffer[i * img.w * 4 + j * 4 + 0] = img.data[(img.h - i - 1) * img.w + j];
                tmp_buffer[i * img.w * 4 + j * 4 + 1] = img.data[(img.h - i - 1) * img.w + j];
                tmp_buffer[i * img.w * 4 + j * 4 + 2] = img.data[(img.h - i - 1) * img.w + j];
                tmp_buffer[i * img.w * 4 + j * 4 + 3] = 255;
            }
        }
    } else {
        std::cerr << "Error: unsupported number of channels in texture image. Channels=" << img.c << std::endl;
    }

    buffer->unmap();
    tex_sampler->setBuffer(0u, 0u, buffer);
    return tex_sampler;
}

TextureSampler ChOptixEngine::CreateTexture() {
    TextureSampler tex_sampler = m_context->createTextureSampler();
    tex_sampler->setWrapMode(0, RT_WRAP_REPEAT);
    tex_sampler->setWrapMode(1, RT_WRAP_REPEAT);
    tex_sampler->setWrapMode(2, RT_WRAP_REPEAT);
    tex_sampler->setIndexingMode(RT_TEXTURE_INDEX_NORMALIZED_COORDINATES);
    tex_sampler->setReadMode(RT_TEXTURE_READ_NORMALIZED_FLOAT);
    tex_sampler->setMaxAnisotropy(1.0f);
    tex_sampler->setMipLevelCount(1u);
    tex_sampler->setArraySize(1u);
    tex_sampler->setFilteringModes(RT_FILTER_LINEAR, RT_FILTER_LINEAR, RT_FILTER_NONE);

    // create a dummy texture
    Buffer buffer = m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_BYTE4, 1u, 1u);
    unsigned char* tmp_buffer = static_cast<unsigned char*>(buffer->map());
    tmp_buffer[0] = 255;
    tmp_buffer[1] = 0;
    tmp_buffer[2] = 255;
    tmp_buffer[3] = 255;
    buffer->unmap();

    tex_sampler->setBuffer(0u, 0u, buffer);

    return tex_sampler;
}

}  // namespace sensor
}  // namespace chrono
