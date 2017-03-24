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
// Authors: Hammad Mazhar
// =============================================================================
// OpenGL viewer, this class draws the system to the screen and handles input
// =============================================================================

#include "chrono_opengl/ChOpenGLViewer.h"
#include "chrono_opengl/ChOpenGLMaterials.h"
#include "chrono/ChConfig.h"

#ifdef CHRONO_PARALLEL
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/physics/Ch3DOFContainer.h"
#endif

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChEllipsoidShape.h"
#include "chrono/assets/ChConeShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChRoundedBoxShape.h"
#include "chrono/assets/ChRoundedConeShape.h"
#include "chrono/assets/ChRoundedCylinderShape.h"
#include "chrono/assets/ChCapsuleShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/solver/ChIterativeSolver.h"
#include "chrono/assets/ChLineShape.h"
#include "chrono/assets/ChPathShape.h"

// Includes are generated at compile time!
#include "resources/phong_frag.h"
#include "resources/phong_vert.h"
#include "resources/cloud_frag.h"
#include "resources/cloud_vert.h"
#include "resources/dot_frag.h"
#include "resources/dot_vert.h"
#include "resources/sphere_frag.h"
#include "resources/sphere_vert.h"
// Standard mesh includes
#include "resources/box.h"
#include "resources/sphere.h"
#include "resources/cylinder.h"
#include "resources/cone.h"

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace chrono {
namespace opengl {

ChOpenGLViewer::ChOpenGLViewer(ChSystem* system) {
    physics_system = system;

    render_camera.SetMode(FREE);
    render_camera.SetPosition(glm::vec3(0, 0, -10));
    render_camera.SetLookAt(glm::vec3(0, 0, 0));
    render_camera.SetClipping(.1, 1000);
    render_camera.SetFOV(45);

    simulation_frame = 0;
    simulation_time = 0;
    simulation_h = 0;
    pause_sim = 0;
    pause_vis = 0;
    single_step = 0;
    view_contacts = 0;
    view_aabb = 0;
    view_help = 0;
    view_grid = 0;
    view_info = 0;
    use_vsync = 0;
    render_mode = POINTS;
    time_total = old_time = current_time = 0;
    time_text = time_geometry = 0;
    fps = 0;
}

ChOpenGLViewer::~ChOpenGLViewer() {}

void ChOpenGLViewer::TakeDown() {
    render_camera.TakeDown();
    main_shader.TakeDown();

    cloud_shader.TakeDown();
    dot_shader.TakeDown();
    sphere_shader.TakeDown();

    sphere.TakeDown();
    box.TakeDown();
    cylinder.TakeDown();
    cone.TakeDown();
    cloud.TakeDown();
    grid.TakeDown();
    fea_nodes.TakeDown();
    fea_elements.TakeDown();
    contact_renderer.TakeDown();
    HUD_renderer.TakeDown();
    graph_renderer.TakeDown();
    for (std::map<std::string, ChOpenGLMesh>::iterator iter = obj_files.begin(); iter != obj_files.end(); iter++) {
        (*iter).second.TakeDown();
    }
}

bool ChOpenGLViewer::Initialize() {
    // Initialize all of the shaders and compile them
    if (!main_shader.InitializeStrings("phong", phong_vert, phong_frag)) {
        return false;
    }
    if (!cloud_shader.InitializeStrings("cloud", cloud_vert, cloud_frag)) {
        return false;
    }
    if (!dot_shader.InitializeStrings("dot", dot_vert, dot_frag)) {
        return false;
    }
    if (!sphere_shader.InitializeStrings("sphere", sphere_vert, sphere_frag)) {
        return false;
    }

    // Setup the generic shapes and load them from file
    sphere.InitializeString(sphere_mesh_data, sphere_color, &main_shader);
    box.InitializeString(box_mesh_data, box_color, &main_shader);
    cylinder.InitializeString(cylinder_mesh_data, cylinder_color, &main_shader);
    cone.InitializeString(cone_mesh_data, cone_color, &main_shader);

    HUD_renderer.Initialize(&render_camera, &timer_render, &timer_text, &timer_render);

    cloud_data.push_back(glm::vec3(0, 0, 0));
    grid_data.push_back(glm::vec3(0, 0, 0));
    fea_node_data.push_back(glm::vec3(0, 0, 0));
    fea_element_data.push_back(glm::vec3(0, 0, 0));
    mpm_grid_data.push_back(glm::vec3(0, 0, 0));
    mpm_node_data.push_back(glm::vec3(0, 0, 0));
    line_path_data.push_back(glm::vec3(0, 0, 0));

    cloud.Initialize(cloud_data, white, &cloud_shader);
    fluid.Initialize(cloud_data, white, &sphere_shader);
    grid.Initialize(grid_data, white, &cloud_shader);
    mpm_grid.Initialize(grid_data, white, &cloud_shader);
    mpm_node.Initialize(cloud_data, white, &cloud_shader);
    line_path.Initialize(line_path_data, red, &cloud_shader);

    fea_nodes.Initialize(fea_node_data, fea_color, &dot_shader);
    fea_elements.Initialize(fea_element_data, fea_color, &cloud_shader);

    contact_renderer.Initialize(contact_color, &dot_shader);
    graph_renderer.Initialize(white, &cloud_shader);

    // glEnable(GL_MULTISAMPLE);
    glEnable(GL_POINT_SPRITE);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    // glLineWidth(10);
    // glEnable(GL_LINE_SMOOTH);
    return 1;
}
bool ChOpenGLViewer::Update(double time_step) {
    if (pause_sim == true && single_step == false) {
        return false;
    }
    simulation_h = time_step;
    physics_system->DoStepDynamics(time_step);
    single_step = false;
    return true;
}
void ChOpenGLViewer::Render() {
    timer_render.reset();
    timer_text.reset();
    timer_geometry.reset();

    timer_render.start();
    if (pause_vis == false) {
        timer_geometry.start();
        render_camera.aspect = window_aspect;
        render_camera.window_width = window_size.x;
        render_camera.window_height = window_size.y;
        render_camera.Update();
        render_camera.GetMatricies(projection, view, model);

        main_shader.SetViewport(window_size);
        cloud_shader.SetViewport(window_size);
        dot_shader.SetViewport(window_size);
        sphere_shader.SetViewport(window_size);

        if (render_mode == WIREFRAME) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        } else {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }

        if (render_mode != POINTS) {
            model_box.clear();
            model_sphere.clear();
            model_cone.clear();
            model_cylinder.clear();
            model_obj.clear();
            line_path_data.clear();
            for (int i = 0; i < physics_system->Get_bodylist()->size(); i++) {
                auto abody = physics_system->Get_bodylist()->at(i);
                DrawObject(abody);
            }
            if (model_box.size() > 0) {
                box.Update(model_box);
                box.Draw(projection, view);
            }
            if (model_sphere.size() > 0) {
                sphere.Update(model_sphere);
                sphere.Draw(projection, view);
            }
            if (model_cone.size() > 0) {
                cone.Update(model_cone);
                cone.Draw(projection, view);
            }
            if (model_cylinder.size() > 0) {
                cylinder.Update(model_cylinder);
                cylinder.Draw(projection, view);
            }
            if (model_obj.size() > 0) {
                for (std::map<std::string, ChOpenGLMesh>::iterator iter = obj_files.begin(); iter != obj_files.end();
                     iter++) {
                    (*iter).second.Update(model_obj[(*iter).first]);
                    (*iter).second.Draw(projection, view);
                }
            }
            if (line_path_data.size() > 0) {
                line_path.Update(line_path_data);
                line_path.Draw(projection, view);
            }

        } else {
            cloud_data.resize(physics_system->Get_bodylist()->size());
#pragma omp parallel for
            for (int i = 0; i < physics_system->Get_bodylist()->size(); i++) {
                auto abody = physics_system->Get_bodylist()->at(i);
                ChVector<> pos = abody->GetPos();
                cloud_data[i] = glm::vec3(pos.x(), pos.y(), pos.z());
            }
        }

        if (render_mode == POINTS) {
            cloud.Update(cloud_data);
            glm::mat4 model(10);
            cloud.Draw(projection, view * model);
        }

        RenderFluid();
        RenderFEA();

        RenderGrid();
        RenderAABB();
        RenderPlots();
        RenderContacts();

        timer_geometry.stop();
        time_geometry = .5 * timer_geometry() + .5 * time_geometry;

        timer_text.start();
        DisplayHUD();
        timer_text.stop();
        time_text = .5 * timer_text() + .5 * time_text;
    }
    timer_render.stop();
    time_total = .5 * timer_render() + .5 * time_total;
    current_time = time_total;
    current_time = current_time * 0.5 + old_time * 0.5;
    old_time = current_time;
    fps = 1.0 / current_time;
}

void ChOpenGLViewer::DrawObject(std::shared_ptr<ChBody> abody) {
    if (abody->GetAssets().size() == 0) {
        return;
    }
	//position of the body
    const Vector pos = abody->GetFrame_REF_to_abs().GetPos();
	//rotation of the body
    Quaternion rot = abody->GetFrame_REF_to_abs().GetRot();
    double angle;
    Vector axis;
    rot.Q_to_AngAxis(angle, axis);

    for (int i = 0; i < abody->GetAssets().size(); i++) {
        auto asset = abody->GetAssets().at(i);

        if (!std::dynamic_pointer_cast<ChVisualization>(asset)) {
            continue;
        }

        ChVisualization* visual_asset = ((ChVisualization*)(asset.get()));
		//position of the asset
        Vector center = visual_asset->Pos;
		//rotate asset pos into global frame
        center = rot.Rotate(center);
		//Get the local rotation of the asset
        Quaternion lrot = visual_asset->Rot.Get_A_quaternion();
		//add the local rotation to the rotation of the body
        lrot = rot % lrot;
        lrot.Normalize();
        lrot.Q_to_AngAxis(angle, axis);

        if (ChSphereShape* sphere_shape = dynamic_cast<ChSphereShape*>(asset.get())) {
            double radius = sphere_shape->GetSphereGeometry().rad;
            ChVector<> pos_final = pos + center;

            model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x(), pos_final.y(), pos_final.z()));
            model = glm::rotate(model, float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::scale(model, glm::vec3(radius, radius, radius));
            model_sphere.push_back(model);

        } else if (ChEllipsoidShape* ellipsoid_shape = dynamic_cast<ChEllipsoidShape*>(asset.get())) {
            Vector radius = ellipsoid_shape->GetEllipsoidGeometry().rad;
            ChVector<> pos_final = pos + center;
            model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x(), pos_final.y(), pos_final.z()));
            model = glm::rotate(model, float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::scale(model, glm::vec3(radius.x(), radius.y(), radius.z()));
            model_sphere.push_back(model);

        } else if (ChBoxShape* box_shape = dynamic_cast<ChBoxShape*>(asset.get())) {
            ChVector<> pos_final = pos + center;
            Vector radius = box_shape->GetBoxGeometry().Size;

            model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x(), pos_final.y(), pos_final.z()));
            model = glm::rotate(model, float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::scale(model, glm::vec3(radius.x(), radius.y(), radius.z()));
            model_box.push_back(model);

        } else if (ChCylinderShape* cylinder_shape = dynamic_cast<ChCylinderShape*>(asset.get())) {
            double rad = cylinder_shape->GetCylinderGeometry().rad;
			ChVector<> dir = cylinder_shape->GetCylinderGeometry().p2 - cylinder_shape->GetCylinderGeometry().p1;
			double height = dir.Length();
			dir.Normalize();
			ChVector<> mx, my, mz;
			dir.DirToDxDyDz(my, mz, mx);  // y is axis, in cylinder.obj frame
			ChMatrix33<> mrot;
			mrot.Set_A_axis(mx, my, mz);
            lrot =rot % (visual_asset->Rot.Get_A_quaternion() % mrot.Get_A_quaternion());
			//position of cylinder based on two points
			ChVector<> mpos = 0.5 * (cylinder_shape->GetCylinderGeometry().p2 + cylinder_shape->GetCylinderGeometry().p1);

            lrot.Q_to_AngAxis(angle, axis);
			ChVector<> pos_final = pos  + rot.Rotate(mpos);

            model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x(), pos_final.y(), pos_final.z()));
            model = glm::rotate(model, float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::scale(model, glm::vec3(rad, height * .5, rad));
            model_cylinder.push_back(model);

        } else if (ChConeShape* cone_shape = dynamic_cast<ChConeShape*>(asset.get())) {
            Vector rad = cone_shape->GetConeGeometry().rad;
            ChVector<> pos_final = pos + center;
            model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x(), pos_final.y(), pos_final.z()));
            model = glm::rotate(model, float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::scale(model, glm::vec3(rad.x(), rad.y(), rad.z()));
            model_cone.push_back(model);

        } else if (ChRoundedBoxShape* shape = dynamic_cast<ChRoundedBoxShape*>(asset.get())) {
            Vector rad = shape->GetRoundedBoxGeometry().Size;
            double radsphere = shape->GetRoundedBoxGeometry().radsphere;
            ChVector<> pos_final = pos + center;
            model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x(), pos_final.y(), pos_final.z()));
            model = glm::rotate(model, float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::scale(model, glm::vec3(rad.x(), rad.y(), rad.z()));
            model_box.push_back(model);

            glm::vec3 local =
                glm::rotate(glm::vec3(rad.x(), rad.y(), rad.z()), float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::translate(glm::mat4(1),
                                   glm::vec3(pos_final.x() + local.x, pos_final.y() + local.y, pos_final.z() + local.z));
            model = glm::scale(model, glm::vec3((float)radsphere));
            model_sphere.push_back(model);

            local = glm::rotate(glm::vec3(rad.x(), rad.y(), -rad.z()), float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::translate(glm::mat4(1),
                                   glm::vec3(pos_final.x() + local.x, pos_final.y() + local.y, pos_final.z() + local.z));
            model = glm::scale(model, glm::vec3((float)radsphere));
            model_sphere.push_back(model);

            local = glm::rotate(glm::vec3(-rad.x(), rad.y(), rad.z()), float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::translate(glm::mat4(1),
                                   glm::vec3(pos_final.x() + local.x, pos_final.y() + local.y, pos_final.z() + local.z));
            model = glm::scale(model, glm::vec3((float)radsphere));
            model_sphere.push_back(model);

            local = glm::rotate(glm::vec3(-rad.x(), rad.y(), -rad.z()), float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::translate(glm::mat4(1),
                                   glm::vec3(pos_final.x() + local.x, pos_final.y() + local.y, pos_final.z() + local.z));
            model = glm::scale(model, glm::vec3((float)radsphere));
            model_sphere.push_back(model);

            local = glm::rotate(glm::vec3(rad.x(), -rad.y(), rad.z()), float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::translate(glm::mat4(1),
                                   glm::vec3(pos_final.x() + local.x, pos_final.y() + local.y, pos_final.z() + local.z));
            model = glm::scale(model, glm::vec3((float)radsphere));
            model_sphere.push_back(model);

            local = glm::rotate(glm::vec3(rad.x(), -rad.y(), -rad.z()), float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::translate(glm::mat4(1),
                                   glm::vec3(pos_final.x() + local.x, pos_final.y() + local.y, pos_final.z() + local.z));
            model = glm::scale(model, glm::vec3((float)radsphere));
            model_sphere.push_back(model);

            local = glm::rotate(glm::vec3(-rad.x(), -rad.y(), rad.z()), float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::translate(glm::mat4(1),
                                   glm::vec3(pos_final.x() + local.x, pos_final.y() + local.y, pos_final.z() + local.z));
            model = glm::scale(model, glm::vec3((float)radsphere));
            model_sphere.push_back(model);

            local = glm::rotate(glm::vec3(-rad.x(), -rad.y(), -rad.z()), float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::translate(glm::mat4(1),
                                   glm::vec3(pos_final.x() + local.x, pos_final.y() + local.y, pos_final.z() + local.z));
            model = glm::scale(model, glm::vec3((float)radsphere));
            model_sphere.push_back(model);
        } else if (ChCapsuleShape* capsule_shape = dynamic_cast<ChCapsuleShape*>(asset.get())) {
            double rad = capsule_shape->GetCapsuleGeometry().rad;
            double height = capsule_shape->GetCapsuleGeometry().hlen;
            // Quaternion rott(1,0,0,0);
            Quaternion lrot = visual_asset->Rot.Get_A_quaternion();
            // lrot = lrot % rott;
            lrot = rot % lrot;

            lrot.Q_to_AngAxis(angle, axis);
            ChVector<> pos_final = pos + center;
            model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x(), pos_final.y(), pos_final.z()));
            model = glm::rotate(model, float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::scale(model, glm::vec3(rad, height, rad));
            model_cylinder.push_back(model);
            glm::vec3 local = glm::rotate(glm::vec3(0, height, 0), float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::translate(glm::mat4(1),
                                   glm::vec3(pos_final.x() + local.x, pos_final.y() + local.y, pos_final.z() + local.z));
            model = glm::scale(model, glm::vec3((float)rad));
            model_sphere.push_back(model);

            local = glm::rotate(glm::vec3(0, -height, 0), float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));

            model = glm::translate(glm::mat4(1),
                                   glm::vec3(pos_final.x() + local.x, pos_final.y() + local.y, pos_final.z() + local.z));
            model = glm::scale(model, glm::vec3((float)rad));
            model_sphere.push_back(model);

        } else if (ChTriangleMeshShape* trimesh_shape = dynamic_cast<ChTriangleMeshShape*>(asset.get())) {
            ChVector<> pos_final = pos + center;
            model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x(), pos_final.y(), pos_final.z()));
            model = glm::rotate(model, float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));

            if (obj_files.find(trimesh_shape->GetName()) == obj_files.end()) {
                ////std::cout << trimesh_shape->GetName() << std::endl;
                obj_files[trimesh_shape->GetName()].Initialize(trimesh_shape, mesh_color);
                obj_files[trimesh_shape->GetName()].AttachShader(&main_shader);
                model_obj[trimesh_shape->GetName()].push_back(model);
            } else {
                model_obj[trimesh_shape->GetName()].push_back(model);
            }
        } else if (ChPathShape* path_shape = dynamic_cast<ChPathShape*>(asset.get())) {
            // std::shared_ptr<geometry::ChLine> mline;
            // mline = path_shape->GetPathGeometry();
        } else if (ChLineShape* line_shape = dynamic_cast<ChLineShape*>(asset.get())) {
            std::shared_ptr<geometry::ChLine> mline;
            mline = line_shape->GetLineGeometry();

            Quaternion lrot = visual_asset->Rot.Get_A_quaternion();
            lrot = rot % lrot;
            lrot.Q_to_AngAxis(angle, axis);

            double maxU = 1;
            if (auto mline_path = std::dynamic_pointer_cast<geometry::ChLinePath>(mline))
                maxU = mline_path->GetPathDuration();
            ChVector<> pos_final = pos + center;

            for (unsigned int ig = 0; ig < 200; ig++) {
                double mU = maxU * ((double)ig / (double)(200 - 1));  // abscyssa
                ChVector<> t2;
                mline->Evaluate(t2, mU, 0, 0);
                t2 = pos_final + lrot.Rotate(t2);
                line_path_data.push_back(glm::vec3(t2.x(), t2.y(), t2.z()));

                mU = maxU * ((double)(ig + 1) / (double)(200 - 1));  // abscyssa
                mline->Evaluate(t2, mU, 0, 0);
                t2 = pos_final + lrot.Rotate(t2);
                line_path_data.push_back(glm::vec3(t2.x(), t2.y(), t2.z()));
            }
        }
    }
}

void ChOpenGLViewer::DisplayHUD() {
    GLReturnedError("Start text");
    HUD_renderer.Update(window_size, dpi, fps, time_geometry, time_text, time_total);
    if (view_help) {
        HUD_renderer.GenerateHelp();
    } else {
        HUD_renderer.GenerateStats(physics_system);
    }

    if (view_info) {
        HUD_renderer.GenerateExtraStats(physics_system);
    }

    HUD_renderer.Draw();
}

void ChOpenGLViewer::RenderContacts() {
    if (view_contacts == false) {
        return;
    }

    contact_renderer.Update(physics_system);
    contact_renderer.Draw(projection, view);
}
void ChOpenGLViewer::RenderAABB() {
    if (view_aabb == false) {
        return;
    }
#ifdef CHRONO_PARALLEL
    if (ChSystemParallel* system = dynamic_cast<ChSystemParallel*>(physics_system)) {
        ChParallelDataManager* data_manager = system->data_manager;
        model_box.clear();

        custom_vector<real3>& aabb_min = data_manager->host_data.aabb_min;
        custom_vector<real3>& aabb_max = data_manager->host_data.aabb_max;

        model_box.resize(data_manager->num_rigid_shapes);
#pragma omp parallel for
        for (int i = 0; i < (signed)data_manager->num_rigid_shapes; i++) {
            real3 min_p = aabb_min[i] + data_manager->measures.collision.global_origin;
            real3 max_p = aabb_max[i] + data_manager->measures.collision.global_origin;

            real3 radius = (max_p - min_p) * .5;
            real3 center = (min_p + max_p) * .5;

            glm::mat4 model = glm::translate(glm::mat4(1), glm::vec3(center.x, center.y, center.z));
            model = glm::scale(model, glm::vec3(radius.x, radius.y, radius.z));
            model_box[i] = (model);
        }
        if (model_box.size() > 0) {
            box.Update(model_box);
            box.Draw(projection, view);
        }
    }

#endif
}
void ChOpenGLViewer::RenderFluid() {
#ifdef CHRONO_PARALLEL
    ChSystemParallel* parallel_system = dynamic_cast<ChSystemParallel*>(physics_system);
    if (!parallel_system || parallel_system->data_manager->num_fluid_bodies <= 0) {
        return;
    }

    if (render_mode != POINTS) {
        fluid.AttachShader(&sphere_shader);
    } else {
        fluid.AttachShader(&dot_shader);
    }

    fluid_data.resize(parallel_system->data_manager->num_fluid_bodies);
#pragma omp parallel for
    for (int i = 0; i < (signed)parallel_system->data_manager->num_fluid_bodies; i++) {
        real3 pos = parallel_system->data_manager->host_data.pos_3dof[i];
        fluid_data[i] = glm::vec3(pos.x, pos.y, pos.z);
    }

    if (ChFluidContainer* fluid_container =
            dynamic_cast<ChFluidContainer*>(parallel_system->data_manager->node_container)) {
        fluid.SetPointSize(float(fluid_container->kernel_radius * .75));
    }
    if (Ch3DOFRigidContainer* rigid_container =
            dynamic_cast<Ch3DOFRigidContainer*>(parallel_system->data_manager->node_container)) {
        fluid.SetPointSize(float(rigid_container->kernel_radius * .75));
    }
    fluid.Update(fluid_data);
    glm::mat4 model(1);
    fluid.Draw(projection, view * model);
#endif
}

void ChOpenGLViewer::RenderFEA() {
#ifdef CHRONO_PARALLEL
    ChSystemParallel* parallel_system = dynamic_cast<ChSystemParallel*>(physics_system);
    if (!parallel_system || parallel_system->data_manager->num_fea_nodes <= 0) {
        return;
    }

    fea_element_data.clear();
    for (int i = 0; i < parallel_system->data_manager->host_data.boundary_triangles_fea.size(); i++) {
        uvec4 ind = parallel_system->data_manager->host_data.boundary_triangles_fea[i];
        real3 pos1 = parallel_system->data_manager->host_data.pos_node_fea[ind.x];
        real3 pos2 = parallel_system->data_manager->host_data.pos_node_fea[ind.y];
        real3 pos3 = parallel_system->data_manager->host_data.pos_node_fea[ind.z];

        fea_element_data.push_back(glm::vec3(pos1.x, pos1.y, pos1.z));
        fea_element_data.push_back(glm::vec3(pos2.x, pos2.y, pos2.z));

        fea_element_data.push_back(glm::vec3(pos1.x, pos1.y, pos1.z));
        fea_element_data.push_back(glm::vec3(pos3.x, pos3.y, pos3.z));

        fea_element_data.push_back(glm::vec3(pos2.x, pos2.y, pos2.z));
        fea_element_data.push_back(glm::vec3(pos3.x, pos3.y, pos3.z));
    }
    fea_elements.Update(fea_element_data);
    glm::mat4 model(1);
    fea_elements.Draw(projection, view * model);
#endif
}

void ChOpenGLViewer::RenderGrid() {
    if (view_grid == false) {
        return;
    }
    grid_data.clear();
#ifdef CHRONO_PARALLEL
    if (ChSystemParallel* parallel_sys = dynamic_cast<ChSystemParallel*>(physics_system)) {
        vec3 bins_per_axis = parallel_sys->data_manager->settings.collision.bins_per_axis;
        real3 bin_size_vec = parallel_sys->data_manager->measures.collision.bin_size;
        real3 min_pt = parallel_sys->data_manager->measures.collision.min_bounding_point;
        real3 max_pt = parallel_sys->data_manager->measures.collision.max_bounding_point;
        real3 center = (min_pt + max_pt) * .5;

        for (int i = 0; i <= bins_per_axis.x; i++) {
            grid_data.push_back(glm::vec3(i * bin_size_vec.x + min_pt.x, center.y, min_pt.z));
            grid_data.push_back(glm::vec3(i * bin_size_vec.x + min_pt.x, center.y, max_pt.z));
        }
        for (int i = 0; i <= bins_per_axis.z; i++) {
            grid_data.push_back(glm::vec3(min_pt.x, center.y, i * bin_size_vec.z + min_pt.z));
            grid_data.push_back(glm::vec3(max_pt.x, center.y, i * bin_size_vec.z + min_pt.z));
        }

        for (int i = 0; i <= bins_per_axis.y; i++) {
            grid_data.push_back(glm::vec3(min_pt.x, i * bin_size_vec.y + min_pt.y, center.z));
            grid_data.push_back(glm::vec3(max_pt.x, i * bin_size_vec.y + min_pt.y, center.z));
        }
        for (int i = 0; i <= bins_per_axis.y; i++) {
            grid_data.push_back(glm::vec3(center.x, i * bin_size_vec.y + min_pt.y, min_pt.z));
            grid_data.push_back(glm::vec3(center.x, i * bin_size_vec.y + min_pt.y, max_pt.z));
        }

        for (int i = 0; i <= bins_per_axis.x; i++) {
            grid_data.push_back(glm::vec3(i * bin_size_vec.x + min_pt.x, min_pt.y, center.z));
            grid_data.push_back(glm::vec3(i * bin_size_vec.x + min_pt.x, max_pt.y, center.z));
        }
        for (int i = 0; i <= bins_per_axis.z; i++) {
            grid_data.push_back(glm::vec3(center.x, min_pt.y, i * bin_size_vec.z + min_pt.z));
            grid_data.push_back(glm::vec3(center.x, max_pt.y, i * bin_size_vec.z + min_pt.z));
        }

        grid.Update(grid_data);
    }
#endif
    glm::mat4 model(1);
    grid.Draw(projection, view * model);

///======
#ifdef CHRONO_PARALLEL
/*mpm_grid_data.clear();
mpm_node_data.clear();
if (ChSystemParallelDVI* parallel_sys = dynamic_cast<ChSystemParallelDVI*>(physics_system)) {
    vec3 bins_per_axis;
    real3 bin_size_vec;
    real3 min_pt;
    real3 max_pt;
    real3 center;
    real bin_edge;
    uint num_mpm_nodes;


    mpm_node_data.resize(num_mpm_nodes);
    for (int i = 0; i < num_mpm_nodes; i++) {
        vec3 g = GridDecode(i, bins_per_axis);
        real3 current_node_location = NodeLocation(g.x, g.y, g.z, bin_edge, min_pt);
        mpm_node_data[i] = glm::vec3(current_node_location.x, current_node_location.y,
        current_node_location.z);
    }
    mpm_node.SetPointSize(.002);
    mpm_node.Update(mpm_node_data);
    mpm_node.Draw(projection, view * model);
    glm::vec3 offset = glm::vec3(.5 * bin_size_vec.x, .5 * bin_size_vec.x, .5 * bin_size_vec.x);
    for (int i = 0; i <= bins_per_axis.x; i++) {
        mpm_grid_data.push_back(glm::vec3(i * bin_size_vec.x + min_pt.x, center.y, min_pt.z) - offset);
        mpm_grid_data.push_back(glm::vec3(i * bin_size_vec.x + min_pt.x, center.y, max_pt.z) - offset);
    }
    for (int i = 0; i <= bins_per_axis.z; i++) {
        mpm_grid_data.push_back(glm::vec3(min_pt.x, center.y, i * bin_size_vec.z + min_pt.z) - offset);
        mpm_grid_data.push_back(glm::vec3(max_pt.x, center.y, i * bin_size_vec.z + min_pt.z) - offset);
    }

    for (int i = 0; i <= bins_per_axis.y; i++) {
        mpm_grid_data.push_back(glm::vec3(min_pt.x, i * bin_size_vec.y + min_pt.y, center.z) - offset);
        mpm_grid_data.push_back(glm::vec3(max_pt.x, i * bin_size_vec.y + min_pt.y, center.z) - offset);
    }
    for (int i = 0; i <= bins_per_axis.y; i++) {
        mpm_grid_data.push_back(glm::vec3(center.x, i * bin_size_vec.y + min_pt.y, min_pt.z) - offset);
        mpm_grid_data.push_back(glm::vec3(center.x, i * bin_size_vec.y + min_pt.y, max_pt.z) - offset);
    }

    for (int i = 0; i <= bins_per_axis.x; i++) {
        mpm_grid_data.push_back(glm::vec3(i * bin_size_vec.x + min_pt.x, min_pt.y, center.z) - offset);
        mpm_grid_data.push_back(glm::vec3(i * bin_size_vec.x + min_pt.x, max_pt.y, center.z) - offset);
    }
    for (int i = 0; i <= bins_per_axis.z; i++) {
        mpm_grid_data.push_back(glm::vec3(center.x, min_pt.y, i * bin_size_vec.z + min_pt.z) - offset);
        mpm_grid_data.push_back(glm::vec3(center.x, max_pt.y, i * bin_size_vec.z + min_pt.z) - offset);
    }

    mpm_grid.Update(mpm_grid_data);
    glm::mat4 model(1);

    mpm_grid.Draw(projection, view * model);
}*/
#endif
}

void ChOpenGLViewer::RenderPlots() {
    //  if (view_info == false || view_help) {
    //    return;
    //  }
    //  graph_renderer.Update(physics_system, window_size);
    //
    //  projection = glm::ortho(0.0f, float(window_size.x), 0.0f, float(window_size.y), -2.0f, 2.0f);
    //  modelview = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -1));
    //  graph_renderer.Draw(projection, modelview);
}

void ChOpenGLViewer::HandleInput(unsigned char key, int x, int y) {
    // printf("%f,%f,%f\n", render_camera.camera_position.x,
    // render_camera.camera_position.y, render_camera.camera_position.z);
    switch (key) {
        case 'W':
            render_camera.Move(FORWARD);
            break;
        case 'S':
            render_camera.Move(BACK);
            break;
        case 'D':
            render_camera.Move(RIGHT);
            break;
        case 'A':
            render_camera.Move(LEFT);
            break;
        case 'Q':
            render_camera.Move(DOWN);
            break;
        case 'E':
            render_camera.Move(UP);
            break;
        case GLFW_KEY_SPACE:
            pause_sim = !pause_sim;
            break;
        case 'P':
            pause_vis = !pause_vis;
            break;
        case '1':
            render_mode = POINTS;
            break;
        case '2':
            render_mode = WIREFRAME;
            break;
        case '3':
            render_mode = SOLID;
            break;
        case 'C':
            view_contacts = !view_contacts;
            break;
        case 'B':
            view_aabb = !view_aabb;
            break;
        case 'G':
            view_grid = !view_grid;
            break;
        case 'H':
            view_help = !view_help;
            break;
        case 'I':
            view_info = !view_info;
            break;
        case 'V':
            //         use_vsync = !use_vsync;
            //         if (use_vsync) {
            //            glfwSwapInterval(1);
            //         } else {
            //            glfwSwapInterval(0);
            //         }
            break;
        case GLFW_KEY_PERIOD:
            single_step = true;
            break;
        default:
            break;
    }
}

}  // end namespace opengl
}  // end namespace chrono
