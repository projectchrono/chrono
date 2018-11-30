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
// Class to manage adding, changing, and removing godot scene nodes
//
// =============================================================================

//#include <scene/3d/camera.h>
//#include <scene/3d/immediate_geometry.h>
#include <scene/3d/light.h>
//#include <scene/3d/mesh_instance.h>
//#include <scene/3d/ray_cast.h>
#include <scene/3d/camera.h>
#include <scene/3d/scenario_fx.h>
#include <scene/3d/spatial.h>
#include <scene/3d/sprite_3d.h>
#include <scene/gui/texture_rect.h>
#include <scene/gui/viewport_container.h>

//#include <scene/2d/canvas_item.h>

#include "chrono_godot/nodes/ChGdCameraSensor.h"
#include "chrono_godot/nodes/ChGdInteractiveCam.h"

//#include <scene/main/viewport.h>
#include <scene/resources/environment.h>
//#include <scene/resources/material.h>
//#include <scene/resources/mesh.h>
//#include <scene/resources/primitive_meshes.h>

#include "chrono_godot/ChGdScene.h"

namespace chrono {
namespace gd {

ChGdScene::ChGdScene() {
    m_assetManager = std::make_shared<ChGdAssetManager>();
}

ChGdScene::~ChGdScene() {}

void ChGdScene::Initialize(MainLoop* main_loop, ChSystem* chsystem) {
    // m_mainLoop = std::make_shared<MainLoop>();
    m_sceneTree = Object::cast_to<SceneTree>(main_loop);
    m_spatialRoot = Object::cast_to<Spatial>(m_sceneTree->get_root()->get_child(0));
    // m_sceneTree->get_root()->print_tree_pretty();
    // m_rootViewport = m_sceneTree->get_root();

    // std::cout << "Getting scene root\n";

    m_hud = std::make_shared<ChGdHUD>();
    m_hud->Initialize(chsystem, m_sceneTree->get_root());

    // std::cout<<m_sceneTree->get_root()->get_name()<<std::endl;
    // m_sceneTree->get_root()->print_tree_pretty();
}

void ChGdScene::PrintSceneTree() {
    m_sceneTree->get_root()->print_tree_pretty();
}

SceneTree* ChGdScene::GetSceneRoot() {
    return m_sceneTree;
}

void ChGdScene::AddEnvironment() {
    // add the world node to the scenetree
    WorldEnvironment* world = memnew(WorldEnvironment);
    m_resourceList.push_back(world);

    //    Spatial* worldRoot = memnew(Spatial);
    //    m_resourceList.push_back(worldRoot);

    //    worldRoot->set_name("WorldRoot");
    //    worldRoot->set_rotation(Vector3(0,0,0));
    //    worldRoot->add_child(world);

    world->set_name("WorldEnvironment");
    m_spatialRoot->add_child(world);
    //    m_spatialRoot->set_rotation_degrees(Vector3(-90,0,0));
    // Transform t;
    // t.rotate(Vector3(1, 0, 0), -1.57);
    // m_spatialRoot->set_transform(t);

    Environment* env = memnew(Environment);
    m_resourceList.push_back(env);
    Ref<Environment> environment{env};
    world->set_environment(environment);

    ProceduralSky* sky_resource = memnew(ProceduralSky);
    m_resourceList.push_back(env);
    Ref<ProceduralSky> sky{sky_resource};

    //    sky->set_sky_top_color(Color(1.0,1.0,1.0));
    //	sky->set_ground_bottom_color(Color(0.2,0.2,0.2));
    sky->set_name("ProceduralSky");
    environment->set_sky(sky);
    environment->set_background(Environment::BG_SKY);

    //    std::cout<<"SkyName: "<<world->get_environment()->get_sky()->get_name()<<std::endl;
}

void ChGdScene::AddInteractiveCamera(ChVector<> location, ChVector<> target_location, ChVector<> up, float FOV) {
    // auto camera = std::make_shared<ChGdMainCamera>(m_sceneTree->get_root(), location, target_location, up, FOV);
    // m_cameraList.push_back(camera);

    ClassDB::register_class<ChGdInteractiveCam>();
    // ClassDB::_add_class<ChGdInteractiveCam>();
    // ClassDB::register_class(ChGdInteractiveCam);
    // ClassDB::init();
    // ClassDB::init<ChGdInteractiveCam>();

    std::cout << "IntCam Exists?: " << ClassDB::class_exists("ChGdInteractiveCam") << std::endl;
    std::cout << "IntCam Enabled?: " << ClassDB::is_class_enabled("ChGdInteractiveCam") << std::endl;
    // std::cout << "IntCam Exists?: " << ClassDB::class_exists("ChGdInteractiveCam") << std::endl;
    // Camera* cam = memnew(Camera);
    ChGdInteractiveCam* con = memnew(ChGdInteractiveCam);
    con->set_name("InteractiveCam");

    // con->set_begin(Point2(0, 0));
    // con->set_end(Point2(1280, 720));
    // con->set_size()
    // std::cout << "Control area: " << con->get_rect().get_area() << std::endl;
    // std::cout << "Viewport area: " << m_sceneTree->get_root()->get_viewport()->get_visible_rect().get_area()
    // << std::endl;

    // cam->set_process(true);
    // cam->set_process_input(true);
    // cam->set_process_unhandled_input(true);

    // cam->set_name("MainCamera");
    // cam->look_at_from_position(GDVector(location), GDVector(target_location), GDVector(up));
    // cam->set_fov(FOV);

    m_sceneTree->get_root()->add_child(con);
    // cam->add_child(con);
    con->set_process(true);
    //    con->is_processing();
    std::cout << "IntCam is processing: " << con->is_processing() << std::endl;
    //    con->notification(0,false);
    //    con->notification(1,false);
    //    con->notification(2,false);
    //    con->notification(3,false);
    // con->request_ready();
    //    con->notification(4,false);
    //    con->notification(5,false);
    //    con->notification(6,false);
    //    con->notification(7,false);

    // con->set_focus_mode(Control::FOCUS_ALL);
    // con->grab_focus();
    // con->grab_click_focus();
    //     con->set_process(true);
    // con->set_process_input(true);
    // con->set_process_unhandled_input(true);
    // con->set_process_unhandled_key_input(true);

    // add camera sensor. TODO: move this somewhere cleaner. It is only hear to test/develop functionality

    // add a viewport container

    //     ChGdCameraSensor* cam_sensor = memnew(ChGdCameraSensor);
    //     cam_sensor->set_name("CameraSensor");
    //     cam_sensor->set_size(Size2(1280, 720));
    //     cam_sensor->set_use_own_world(false);
    //     cam_sensor->set_attach_to_screen_rect(Rect2(0, 0, 1280, 720));  // for some reason this is needed
    //     m_sceneTree->get_root()->get_child(0)->add_child(cam_sensor);
    //     cam_sensor->set_process(true);
    //
    //     ViewportContainer* v = memnew(ViewportContainer);
    //     v->set_name("ViewportContainer");
    //     cam_sensor->add_child(v);
    //
    //     // add a viewport with a camera
    //     //    ChGdCameraSensor* cam_sensor = memnew(ChGdCameraSensor);
    //     //    cam_sensor->set_name("CameraSensor");
    //     //    cam_sensor->set_size(Size2(1280, 720));
    //     //    cam_sensor->set_use_own_world(false);
    //     //    cam_sensor->set_attach_to_screen_rect(Rect2(0, 0, 1280, 720));  // for some reason this is needed
    //     //    v->add_child(cam_sensor);
    //     //    cam_sensor->set_process(true);
    //
    //     Viewport* view = memnew(Viewport);
    //     view->set_name("ViewPort");
    //     view->set_size(Size2(1280, 720));
    //     view->set_use_own_world(false);
    //     view->set_attach_to_screen_rect(Rect2(0, 0, 1280, 720));  // for some reason this is needed
    //     v->add_child(view);
    //     // cam_sensor->set_process(true);
    //
    //     Camera* cam_sensor_cam = memnew(Camera);
    //     cam_sensor_cam->set_name("Camera");
    //     cam_sensor_cam->look_at_from_position({0, 0, -20}, {0, 0, 0}, {0, 1, 0});
    //     view->add_child(cam_sensor_cam);
    //
    //     Ref<Shader> screen_shader{memnew(Shader)};
    //     screen_shader->set_name("ScreenShader");
    //     //	screen_shader->set_code(" \
// //		shader_type canvas_item;\
// //        \
// //		varying float A;\
// //		varying float B;\
// //		\
// //		void vertex(){\
// //			A = PROJECTION_MATRIX[2][2];\
// //			B = PROJECTION_MATRIX[3][2];\
// //		}\
// //		\
// //		void fragment(){\
// //			float C = (A - 1.0)/2.0;\
// //			float D = B / 2.0;\
// //			\
// //			float depth = FRAGCOORD.z / FRAGCOORD.w;\
// //			if(C>0.0001){\
// //				depth = depth - D / C;\
// //			}\
// //		\
// //			COLOR.rgb = vec3(depth,depth,depth);\
// //		}");
    //
    //     screen_shader->set_code(
    //         " \
//     		shader_type canvas_item;\
//             \
//     		varying vec4 p0;\
// 			varying vec4 p1;\
// 			varying vec4 p2;\
// 			varying vec4 p3;\
// 		\
// 			void vertex(){\
// 				mat4 proj = inverse(PROJECTION_MATRIX);\
// 				p0 = proj[0];\
// 				p1 = proj[1];\
// 				p2 = proj[2];\
// 				p3 = proj[3];\
// 				\
// 			}\
// 		\
// 			void fragment(){\
// 				mat4 inv_proj = mat4(p0,p1,p2,p3);\
// 				float depth=FRAGCOORD.z/FRAGCOORD.w;\
// 				\
// 				vec4 upos = inv_proj * vec4(SCREEN_UV*2.0-1.0,depth*2.0-1.0,1.0);\
// 				vec3 pixel_position = upos.xyz/upos.w;\
// 				COLOR = vec4(pixel_position.z,pixel_position.z,pixel_position.z,1.0);\
//     		    COLOR = vec4(FRAGCOORD.z, FRAGCOORD.y,FRAGCOORD.x,0.5);\
//     		}");
    //
    //     //	screen_shader->set_code(" \
// //		shader_type canvas_item;\
// //		render_mode unshaded;\
// //		\
// //		uniform vec2 ball_position = vec2(0, 0);\
// //		uniform int active_distance = 50;\
// //		\
// //		varying vec2 world_pos;\
// //		\
// //		void vertex() {\
// //			world_pos = (WORLD_MATRIX * vec4(VERTEX, 1.0, 1.0)).xy;\
// //		}\
// //		\
// //		void fragment() {\
// //			float distance_to_ball = distance(world_pos, ball_position);\
// //			if (distance_to_ball > float(active_distance)) {\
// //				COLOR = vec4(1, 0, 0, 1);\
// //			} else {\
// //				float strength = distance_to_ball / float(active_distance);\
// //				COLOR = vec4(0, strength, 0, 1);\
// //			}\
// //		}");
    //
    //     Ref<ShaderMaterial> mat{memnew(ShaderMaterial)};
    //     mat->set_shader(screen_shader);
    //     v->set_material(mat);
    //
    //     //    Ref<Shader> depth_shader{memnew(Shader)};
    //     //    depth_shader->set_name("depth_shader");
    //     //    depth_shader->set_code(" \
// //		shader_type spatial;\
// //		\
// //		uniform float amount : hint_range(0,50);\
// //		\
// //		void fragment() {\
// //        float depth = textureLod(DEPTH_TEXTURE,SCREEN_UV,0.0).r;\
// //        vec4 upos = INV_PROJECTION_MATRIX * vec4(SCREEN_UV*2.0-1.0,depth*2.0-1.0,1.0);\
// //        vec3 pixel_position = upos.xyz/upos.w;\
// //        }");
    //     //
    //     //	Ref<ShaderMaterial> mat2{memnew(ShaderMaterial)};
    //     //	mat2->set_shader(depth_shader);
    //     //
    //     //	QuadMesh* quad = memnew(QuadMesh);
    //     //	quad->set_size(Vector2(1280,720));
    //     //
    //     //	MeshInstance* quad_node = memnew(MeshInstance);
    //     //	quad_node->set_name("QuadMesh");
    //     //	quad_node->set_mesh(quad);
    //     //	quad_node->set_surface_material(0,mat2);
    //     //	quad_node->set_process(true);
    //     //
    //     //	cam_sensor_cam->add_child(quad_node);
    //
    //     // SphereMesh* mesh = memnew(SphereMesh);
    //     //     //	m_resourceList.push_back(mesh);
    //     //     mesh->set_radius(0.5);
    //     //     mesh->set_height(1.0);  // the height needs to be twice the radius for a sphere
    //     //
    //     //     Ref<SpatialMaterial> material{memnew(SpatialMaterial)};
    //     //     //	m_resourceList.push_back(material);
    //     //     material->set_flag(SpatialMaterial::FLAG_SRGB_VERTEX_COLOR, true);
    //     //     material->set_flag(SpatialMaterial::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
    //     //     material->set_albedo(Color(0.0, 1.0, 0.0));
    //     //     material->set_script_instance(nullptr);
    //     //     SpatialMaterial::flush_changes();
    //     //
    //     //     MeshInstance* mesh_node = memnew(MeshInstance);
    //     //     //	m_resourceList.push_back(mesh_node);
    //     //     mesh_node->set_name("SphereMesh");
    //     //     m_sceneTree->get_root()->get_child(0)->add_child(mesh_node);
    //     //     mesh_node->set_mesh(mesh);
    //     //     mesh_node->set_surface_material(0, material);
    //     //     mesh_node->set_translation({0.0, 0.0, 0});
    //
    //     //	ChGdCamTexture* sprite = memnew(ChGdCamTexture);
    //     //	sprite->set_name("Sprite3D");
    //     //	sprite->set_process(true);
    //     //	sprite->set_material_override(mat);
    //     //	sprite->set_region_rect(Rect2(0,0,1280,720));
    //     //	std::cout<<"Sprite dims: "<<sprite->get_region_rect().size.height<<",
    //     //"<<sprite->get_region_rect().size.width<<std::endl;
    //     //
    //     //	v->add_child(sprite);

    // std::cout << "Camera sensor should be added to scene now\n";
}

void ChGdScene::SetDisplayHUD(bool display) {
    m_hud->SetDisplay(display);
}

void ChGdScene::AddDirectionalLight(ChQuaternion<> q) {
    DirectionalLight* light = memnew(DirectionalLight);
    m_resourceList.push_back(light);
    light->set_name("DirectionalLight");
    light->set_param(DirectionalLight::PARAM_SHADOW_MAX_DISTANCE, 2000);
    light->set_shadow(true);
    light->set_shadow_mode(DirectionalLight::SHADOW_PARALLEL_4_SPLITS);
    light->set_shadow_depth_range(DirectionalLight::SHADOW_DEPTH_RANGE_OPTIMIZED);

    Transform t = GDTransform(q);
    light->set_transform(t);

    m_spatialRoot->add_child(light);
}

void ChGdScene::AddPointLight(ChVector<> location) {
    OmniLight* light = memnew(OmniLight);
    m_resourceList.push_back(light);
    light->set_name("PointLight");
    light->set_shadow(true);
    light->set_shadow_mode(OmniLight::SHADOW_DUAL_PARABOLOID);
    light->set_param(OmniLight::PARAM_RANGE, 20);
    light->set_param(OmniLight::PARAM_SHADOW_MAX_DISTANCE, 20);
    light->set_translation(GDVector(location));
    m_spatialRoot->add_child(light);
}

void ChGdScene::UpdateBodies(ChSystem* chsystem) {
    for (auto body : chsystem->Get_bodylist()) {
        auto godot_body = std::make_shared<ChGdBody>(m_assetManager);
        godot_body->CreateFromChrono(m_spatialRoot, body);
        m_bodyList.push_back(godot_body);
    }
}

void ChGdScene::UpdatePoses() {
    for (auto body : m_bodyList) {
        body->UpdatePose();
    }
}

void ChGdScene::UpdateHUD() {
    // update HUD data
    m_hud->Update();
}

// void ChGdScene::AddCamera(ChVector<> pos, ChVector<> look_at, float FOV, ChVector<> up, std::string name) {
//     Ref<Environment> environment{memnew(Environment)};
//     //	m_resourceList.push_back(environment);
//
//     WorldEnvironment* world = memnew(WorldEnvironment);
//     // WorldEnvironment> world = std::make_shared<WorldEnvironment>();
//     //	m_resourceList.push_back(world);
//     world->set_name("WorldEnvironment");
//     world->set_environment(environment);
//     if (world->get_environment() == NULL) {
//         std::cout << "Environment NULL\n";
//     } else {
//         std::cout << "Light Energy Val: " << world->get_environment()->get_ambient_light_energy() << std::endl;
//     }
//     m_sceneTree->get_root()->get_child(0)->add_child(world);
//
//     Camera* camera = memnew(Camera);
//     // std::shared_ptr<Camera> camera = std::make_shared<Camera>();
//
//     //	Camera* camera = new Camera();
//     //	m_resourceList.push_back(camera);
//     camera->set_name("Camera");
//     camera->look_at_from_position({5, 0, 0}, {0, 0, 0}, {0, 0, 1});
//     camera->set_fov(30);
//     m_sceneTree->get_root()->get_child(0)->add_child(camera);
//
//     viewport1 = memnew(Viewport);
//     //	m_resourceList.push_back(viewport1);
//     viewport1->set_name("Viewport1");
//     viewport1->set_size(Size2(1280, 720));
//     viewport1->set_use_own_world(false);
//     viewport1->set_attach_to_screen_rect(Rect2(0, 0, 1280, 720));  // for some reason this is needed
//     m_sceneTree->get_root()->get_child(0)->add_child(viewport1);
//
//     Camera* camera1 = memnew(Camera);
//     //	m_resourceList.push_back(camera1);
//     camera1->set_name("Camera1");
//     camera1->look_at_from_position({0, 5, 0}, {0, 0, 0}, {0, 0, 1});
//     camera1->set_fov(30);
//     //	camera1->set_perspective(30,.1,100);
//     viewport1->add_child(camera1);
//
//     viewport2 = memnew(Viewport);
//     //	m_resourceList.push_back(viewport2);
//     //	viewport2->get_texture()->get_data()->save_png("viewport2.png");
//     viewport2->set_name("Viewport2");
//     viewport2->set_size(Size2(1280, 720));
//     viewport2->set_use_own_world(false);
//     viewport2->set_attach_to_screen_rect(Rect2(0, 0, 1280, 720));  // for some reason this is needed
//     m_sceneTree->get_root()->get_child(0)->add_child(viewport2);
//
//     Camera* camera2 = memnew(Camera);
//     //	m_resourceList.push_back(camera2);
//     camera2->set_name("Camera2");
//     camera2->look_at_from_position({5, 0, 0}, {0, 0, 0}, {0, 0, 1});
//     camera2->set_fov(30);
//     //	camera2->set_perspective(30,.05,100);
//     viewport2->add_child(camera2);
//
//     ray = memnew(RayCast);
//     ray->set_visible(true);
//     ray->set_name("RayCast");
//     ray->set_cast_to({0, 5, 0});
//     //	ray->set_translation({2,0,0});
//     ray->translate({0, -5, 0});
//     ray->set_enabled(true);
//     m_sceneTree->get_root()->get_child(0)->add_child(ray);
//
//     // add a basic HUD
//     Control* hud = memnew(Control);
//     hud->set_name("HUD");
//     m_sceneTree->get_root()->add_child(hud);
//
//     // add text box
//     Label* text_box = memnew(Label);
//     text_box->set_name("ExampleTextBox");
//     text_box->set_text("Example Text Box\nSecond Line");
//     text_box->set_position({500, 100});
//     hud->add_child(text_box);
//
//     // add button -> can easily add, but need to solve clicking
//     Button* button = memnew(Button);
//     button->set_name("Button");
//     button->set_text("ExampleButton");
//     button->set_position({50, 500});
//     button->set_disabled(false);
//     hud->add_child(button);
//
//     // add basic lines
//     ImmediateGeometry* geo = memnew(ImmediateGeometry);
//     geo->set_name("Geometry(lines)");
//     geo->set_color(Color(0, 0, 255));
//     geo->clear();
//     geo->begin(Mesh::PRIMITIVE_LINE_STRIP);
//     geo->add_vertex({0, 0, 0});
//     geo->add_vertex({0, 0, 1});
//     geo->add_vertex({0, 1, 1});
//     geo->add_vertex({1, 1, 1});
//     geo->add_vertex({1, 1, 0});
//     geo->end();
//     m_sceneTree->get_root()->get_child(0)->add_child(geo);
//
//     //	m_sceneTree->get_root()->set_attach_to_screen_rect(Rect2(0,0,1280,720));
//
//     //	Camera* camera2 = memnew(Camera);
//     //	camera2->set_name("Camera2");
//     //	camera2->look_at_from_position({0,5,0},{0,0,0},{0,0,1});
//     //	camera2->set_fov(30);
//     //	m_sceneTree->get_root()->get_child(0)->add_child(camera2);
//     //	camera2->make_current();
//     // m_cameraList.push_back(camera);
// }
//
// void ChGdScene::DrawDebug() {
//     // test writing viewport images to file
//     //	m_sceneTree->get_root()->get_texture()->get_data()->
//     //			save_png("/home/asher/code/chrono_godot/chrono-dev/build/bin/viewport_main.png");
//     ////
//     //
//     viewport1->get_texture()->get_data()->save_png("/home/asher/code/chrono_godot/chrono-dev/build/bin/viewport1.png");
//     //
//     viewport2->get_texture()->get_data()->save_png("/home/asher/code/chrono_godot/chrono-dev/build/bin/viewport2.png");
//
//     //	ray->force_raycast_update();
//     //	std::cout<<"\nRay colliding: "<<ray->is_colliding()<<std::endl;
//
//     mesh_move->translate({0, 0, .001});
// }
//
// void ChGdScene::AddLight() {
//     OmniLight* light = memnew(OmniLight);
//     m_resourceList.push_back(light);
//     light->set_name("OmniLight");
//     m_sceneTree->get_root()->get_child(0)->add_child(light);
//     light->translate({2.0, 0, 0});
// }
//
// void ChGdScene::AddSphere() {
//     SphereMesh* mesh = memnew(SphereMesh);
//     //	m_resourceList.push_back(mesh);
//     mesh->set_radius(0.5);
//     mesh->set_height(1.0);  // the height needs to be twice the radius for a sphere
//
//     Ref<SpatialMaterial> material{memnew(SpatialMaterial)};
//     //	m_resourceList.push_back(material);
//     material->set_flag(SpatialMaterial::FLAG_SRGB_VERTEX_COLOR, true);
//     material->set_flag(SpatialMaterial::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
//     material->set_albedo(Color(0.0, 1.0, 0.0));
//     material->set_script_instance(nullptr);
//     SpatialMaterial::flush_changes();
//
//     MeshInstance* mesh_node = memnew(MeshInstance);
//     //	m_resourceList.push_back(mesh_node);
//     mesh_node->set_name("SphereMesh");
//     m_sceneTree->get_root()->get_child(0)->add_child(mesh_node);
//     mesh_node->set_mesh(mesh);
//     mesh_node->set_surface_material(0, material);
//     mesh_node->set_translation({0.0, 0.0, 0});
//
//     // second sphere to help figure out where cameras are
//     Ref<SpatialMaterial> material2{memnew(SpatialMaterial)};
//     //	m_resourceList.push_back(material2);
//     material2->set_flag(SpatialMaterial::FLAG_SRGB_VERTEX_COLOR, true);
//     material2->set_flag(SpatialMaterial::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
//     material2->set_albedo(Color(1.0, 0.0, 0.0));
//     material2->set_script_instance(nullptr);
//     SpatialMaterial::flush_changes();
//
//     MeshInstance* mesh_node2 = memnew(MeshInstance);
//     //	m_resourceList.push_back(mesh_node2);
//     mesh_node2->set_name("SphereMesh2");
//     m_sceneTree->get_root()->get_child(0)->add_child(mesh_node2);
//     mesh_node2->set_mesh(mesh);
//     mesh_node2->set_surface_material(0, material2);
//     mesh_node2->set_translation({1.0, 1.0, 0});
//     mesh_node2->set_scale({0.5, 0.5, 0.5});
//
//     mesh_move = mesh_node;
// }

}  // namespace gd
}  // end namespace chrono
