// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
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
// Unit test for MatrMultiplyAVX and MatrMultiplyTAVX.
//
// =============================================================================

#include "gtest/gtest.h"

#include "chrono/core/ChLog.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/optixcpp/ChOptixUtils.h"

using namespace chrono;
using namespace sensor;

const float SIM_RUN_TIME = 10.0;

TEST(ChSensorManager, box_intersect) {
    ChSystemNSC mphysicalSystem;
    auto boxA = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 100, true, false);

    boxA->SetPos({2, 0, 0});
    boxA->SetBodyFixed(true);

    mphysicalSystem.Add(boxA);

    auto c = chrono_types::make_shared<ChSensorManager>(&mphysicalSystem);
    // c->SetRenderDimensions(5, 5);

    auto cam = chrono_types::make_shared<ChCameraSensor>(
        boxA, 50, chrono::ChFrame<double>({-8, 0, 3}, Q_from_AngAxis(0, {1, 0, 0})), 1, 1, 1);
    cam->SetName("Camera Sensor");
    c->AddSensor(cam);

    // c->AddCamera(5, 5, 1);

    // optix::Context context = c->GetEngine(0)->GetOptixContext();
    //
    // context->setEntryPointCount(1);  // set the entry point count
    //
    // optix::Buffer buffer = context->createBuffer(RT_BUFFER_OUTPUT, RT_FORMAT_FLOAT, 5, 5);
    //
    // // create the camera for measuring depth
    // const std::string ptx_pre = "ChronoEngine_sensor_generated_";
    // const std::string ptx_suff = ".cu.ptx";
    // const std::string camera_name = "depth_trace_camera";
    // std::string ptx_file = std::string(GetPTXGeneratedLocation()) + ptx_pre + std::string("depth_trace") + ptx_suff;
    // std::string ptx = ptxFromFile(ptx_file);
    // optix::Program ray_gen_program = context->createProgramFromPTXString(ptx, camera_name);
    // ray_gen_program["output_buffer"]->set(buffer);
    // context->setRayGenerationProgram(0, ray_gen_program);
    //
    // // set the miss program and background color
    // const std::string miss_name = "miss_function";
    // context->setMissProgram(0, context->createProgramFromPTXString(ptx, miss_name));
    // context["default_depth"]->setFloat(0.0f);
    //
    // context->validate();
    // context->launch(0, 5, 5);
    // context->get();
    //
    // float* data = (float*)buffer->map();
    // buffer->unmap();
    //
    // float eps = 1e-7f;
    // ASSERT_LT(abs(data[0]), eps);
    // ASSERT_LT(abs(data[12] - 1.5), eps);
    //
    // std::cout << std::setprecision(4) << "Depth data for single BOX test: \n";
    // for (int i = 4; i >= 0; i--) {
    //     for (int j = 0; j < 5; j++) {
    //         std::cout << data[i * 5 + j] << ", \t";
    //     }
    //     std::cout << "\n";
    // }
}

TEST(ChSensorManager, sphere_intersect) {
    ChSystemNSC mphysicalSystem;
    auto sphere = chrono_types::make_shared<ChBodyEasySphere>(1, 100, true, false);

    sphere->SetPos({2, 0, 0});

    sphere->SetBodyFixed(true);

    mphysicalSystem.Add(sphere);

    auto c = chrono_types::make_shared<ChSensorManager>(&mphysicalSystem);
    auto cam = chrono_types::make_shared<ChCameraSensor>(
        sphere, 50, chrono::ChFrame<double>({-8, 0, 3}, Q_from_AngAxis(0, {1, 0, 0})), 1, 1, 1);
    cam->SetName("Camera Sensor");
    c->AddSensor(cam);

    // optix::Context context = c->GetEngine(0)->GetOptixContext();
    //
    // context->setEntryPointCount(1);  // set the entry point count
    //
    // optix::Buffer buffer = context->createBuffer(RT_BUFFER_OUTPUT, RT_FORMAT_FLOAT, 5, 5);
    //
    // // create the camera for measuring depth
    // const std::string ptx_pre = "ChronoEngine_sensor_generated_";
    // const std::string ptx_suff = ".cu.ptx";
    // const std::string camera_name = "depth_trace_camera";
    // std::string ptx_file = std::string(GetPTXGeneratedLocation()) + ptx_pre + std::string("depth_trace") + ptx_suff;
    // std::string ptx = ptxFromFile(ptx_file);
    // optix::Program ray_gen_program = context->createProgramFromPTXString(ptx, camera_name);
    // ray_gen_program["output_buffer"]->set(buffer);
    // context->setRayGenerationProgram(0, ray_gen_program);
    //
    // // set the miss program and background color
    // const std::string miss_name = "miss_function";
    // context->setMissProgram(0, context->createProgramFromPTXString(ptx, miss_name));
    // context["default_depth"]->setFloat(0.0f);
    //
    // context->validate();
    // context->launch(0, 5, 5);
    // context->get();
    //
    // float* data = (float*)buffer->map();
    // buffer->unmap();
    //
    // std::cout << std::setprecision(4) << "Depth data for single SPHERE test: \n";
    // for (int i = 4; i >= 0; i--) {
    //     for (int j = 0; j < 5; j++) {
    //         std::cout << data[i * 5 + j] << ", \t";
    //     }
    //     std::cout << "\n";
    // }
    //
    // float eps = 1e-7f;
    // ASSERT_LT(abs(data[0]), eps);              // we should not see anything at this pixel
    // ASSERT_LT(abs(data[12] - 1.0), eps);       // we should see the exact center of the sphere 1 meter away
    // ASSERT_GT(data[11], data[12]);             // the value to left of center better be larger
    // ASSERT_LT(abs(data[11] - data[13]), eps);  // values had better be symmetric horizontally
    // ASSERT_LT(abs(data[11] - data[17]), eps);  // values to left should be same as values up
    // ASSERT_GT(data[16], data[11]);             // values further from center should be larger
}

TEST(ChSensorManager, mesh_intersect) {
    ChSystemNSC mphysicalSystem;
    auto dummy = chrono_types::make_shared<ChBodyEasySphere>(1, 100, true, false);
    mphysicalSystem.Add(dummy);

    auto c = chrono_types::make_shared<ChSensorManager>(&mphysicalSystem);
    auto cam = chrono_types::make_shared<ChCameraSensor>(
        dummy, 50, chrono::ChFrame<double>({-8, 0, 3}, Q_from_AngAxis(0, {1, 0, 0})), 1, 1, 1);
    cam->SetName("Camera Sensor");
    c->AddSensor(cam);

    // optix::Context context = c->GetEngine(0)->GetOptixContext();
    //
    // context->setEntryPointCount(1);  // set the entry point count
    //
    // optix::Buffer buffer = context->createBuffer(RT_BUFFER_OUTPUT, RT_FORMAT_FLOAT, 5, 5);
    //
    // // create the camera for measuring depth
    // const std::string ptx_pre = "ChronoEngine_sensor_generated_";
    // const std::string ptx_suff = ".cu.ptx";
    // const std::string camera_name = "depth_trace_camera";
    // std::string ptx_file = std::string(GetPTXGeneratedLocation()) + ptx_pre + std::string("depth_trace") + ptx_suff;
    // std::string ptx = ptxFromFile(ptx_file);
    // optix::Program ray_gen_program = context->createProgramFromPTXString(ptx, camera_name);
    // ray_gen_program["output_buffer"]->set(buffer);
    // context->setRayGenerationProgram(0, ray_gen_program);
    //
    // // set the miss program and background color
    // const std::string miss_name = "miss_function";
    // context->setMissProgram(0, context->createProgramFromPTXString(ptx, miss_name));
    // context["default_depth"]->setFloat(0.0f);
    //
    // optix::Group root = c->GetEngine(0)->GetRootNode();
    //
    // int num_tris = 1;
    // int num_vertices = 3;
    //
    // unsigned int triangle_indices[] = {0, 1, 2};
    // float triangle_vertices[] = {1.0, -1.0, -1.0, 1.0, 0.0, 1.0, 1.0, 1.0, -1.0};
    // unsigned int material_indices[] = {0};
    //
    // // add a triangle mesh to the scene
    // optix::GeometryTriangles tris = context->createGeometryTriangles();
    // tris->setPrimitiveCount(num_tris);
    // optix::Buffer index_buffer = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_INT3, num_tris);
    // optix::Buffer normal_buffer = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT3, num_vertices);
    // optix::Buffer vertex_buffer = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT3, num_vertices);
    // optix::Buffer texcoord_buffer = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT2, num_vertices);
    // optix::Buffer mat_index_buffer = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_INT, 1);
    //
    // memcpy(index_buffer->map(), triangle_indices, sizeof(triangle_indices));
    // index_buffer->unmap();
    // memcpy(vertex_buffer->map(), triangle_vertices, sizeof(triangle_vertices));
    // vertex_buffer->unmap();
    // memcpy(mat_index_buffer->map(), material_indices, sizeof(material_indices));
    // mat_index_buffer->unmap();
    //
    // tris->setTriangleIndices(index_buffer, RT_FORMAT_UNSIGNED_INT3);
    // tris->setVertices(num_vertices, vertex_buffer, RT_FORMAT_FLOAT3);
    // tris->setMaterialCount(1);
    // tris->setMaterialIndices(mat_index_buffer, 0, sizeof(unsigned), RT_FORMAT_UNSIGNED_INT);
    // tris->setAttributeProgram(GetRTProgram(context, "triangle_mesh", "mesh_attributes"));
    //
    // optix::Material matl = context->createMaterial();
    // optix::Program closest_hit = GetRTProgram(context, "basic_shader", "normal_shader");
    // optix::Program shadow_prog = GetRTProgram(context, "basic_shader", "hit_shadow");
    //
    // matl->setClosestHitProgram(0, closest_hit);
    // matl->setAnyHitProgram(1, shadow_prog);
    //
    // optix::GeometryInstance triangle_instance = context->createGeometryInstance();
    // triangle_instance->setGeometryTriangles(tris);
    // triangle_instance->setMaterialCount(1);
    // triangle_instance->setMaterial(0, matl);
    // triangle_instance["vertex_buffer"]->setBuffer(vertex_buffer);
    // triangle_instance["normal_buffer"]->setBuffer(normal_buffer);
    // triangle_instance["texcoord_buffer"]->setBuffer(texcoord_buffer);
    // triangle_instance["index_buffer"]->setBuffer(index_buffer);
    // triangle_instance["material_buffer"]->setBuffer(mat_index_buffer);
    //
    // optix::GeometryGroup triangle_group = context->createGeometryGroup();
    // triangle_group->addChild(triangle_instance);
    // triangle_group->setAcceleration(context->createAcceleration("Trbvh"));
    // root->addChild(triangle_group);
    // //
    //
    // context->validate();
    // context->launch(0, 5, 5);
    // context->get();
    //
    // float* data = (float*)buffer->map();
    // buffer->unmap();
    //
    // std::cout << std::setprecision(4) << "Depth data for single TRIANGLE test: \n";
    // for (int i = 4; i >= 0; i--) {
    //     for (int j = 0; j < 5; j++) {
    //         std::cout << data[i * 5 + j] << ", \t";
    //     }
    //     std::cout << "\n";
    // }
    //
    // float eps = 1e-7f;
    // // ASSERT_LT(abs(data[0]), eps);
    // ASSERT_LT(abs(data[12] - 1.0), eps);
}

TEST(OptixGeometry, cylinder) {
    ChSystemNSC mphysicalSystem;
    auto cylinder = chrono_types::make_shared<ChBodyEasyCylinder>(0.3, 1, 100, true, false);

    cylinder->SetPos({0, 0, 0});
    cylinder->SetBodyFixed(true);
    mphysicalSystem.Add(cylinder);

    auto c = chrono_types::make_shared<ChSensorManager>(&mphysicalSystem);
    c->scene->AddPointLight({-5, 5, 10}, {1, 1, 1}, 100);
    c->scene->AddPointLight({5, -5, 10}, {1, 1, 1}, 100);
    auto cam = chrono_types::make_shared<ChCameraSensor>(
        cylinder, 50, chrono::ChFrame<double>({-8, 0, 2.5}, Q_from_AngAxis(.4, {0, 1, 0})), 1280, 720, CH_C_PI / 3);
    cam->SetName("Camera Sensor");
    // cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1280, 720, "Cylinder"));
    c->AddSensor(cam);

    float orbit_radius = 8.f;
    float orbit_rate = 0.5;
    float ch_time = 0.0;

    while (ch_time < SIM_RUN_TIME) {
        ch_time = mphysicalSystem.GetChTime();
        cam->SetOffsetPose(chrono::ChFrame<double>(
            {-orbit_radius * cos(ch_time * orbit_rate), -orbit_radius * sin(ch_time * orbit_rate), 0},
            Q_from_AngAxis(ch_time * orbit_rate, {0, 0, 1})));
        mphysicalSystem.DoStepDynamics(0.001);
        c->Update();
    }
}
