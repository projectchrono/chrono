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

#include "chrono/ChConfig.h"

#include "chrono/physics/ChSystem.h"

#ifdef CHRONO_MULTICORE
    #include "chrono_multicore/physics/ChSystemMulticore.h"
    #include "chrono_multicore/ChDataManager.h"
    #include "chrono_multicore/physics/Ch3DOFContainer.h"
#endif

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeEllipsoid.h"
#include "chrono/assets/ChVisualShapeCone.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeRoundedBox.h"
#include "chrono/assets/ChVisualShapeCapsule.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono/assets/ChVisualShapeLine.h"
#include "chrono/assets/ChVisualShapePath.h"

#include "chrono/physics/ChParticleCloud.h"

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

#include "chrono_opengl/ChOpenGLViewer.h"
#include "chrono_opengl/ChVisualSystemOpenGL.h"
#include "chrono_opengl/ChOpenGLMaterials.h"

namespace chrono {
namespace opengl {

ChOpenGLViewer::ChOpenGLViewer(ChVisualSystemOpenGL* vis) : m_vis(vis) {
    render_camera.SetMode(FREE);
    render_camera.SetPosition(glm::vec3(0, 0, -10));
    render_camera.SetLookAt(glm::vec3(0, 0, 0));
    render_camera.SetClipping(.1, 1000);
    render_camera.SetFOV(45);

    view_contacts = false;
    view_aabb = false;
    view_help = false;
    view_grid = false;
    use_vsync = false;
    render_mode = SOLID;
    particle_render_mode = POINTS;
    particle_radius = 0.1f;
    time_total = old_time = current_time = 0;
    time_text = time_geometry = 0;
    fps = 0;

}

ChOpenGLViewer::~ChOpenGLViewer() {}

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

    m_vis->stats_renderer->Initialize(&render_camera);

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

    particles.Initialize(cloud_data, cadet_blue, &sphere_shader);

    fea_nodes.Initialize(fea_node_data, fea_color, &dot_shader);
    fea_elements.Initialize(fea_element_data, fea_color, &cloud_shader);

    contact_renderer.Initialize(contact_color, &dot_shader);
    graph_renderer.Initialize(white, &cloud_shader);

#ifndef __EMSCRIPTEN__
    glEnable(GL_POINT_SPRITE);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
#endif
    // glEnable(GL_MULTISAMPLE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    // glLineWidth(10);
    // glEnable(GL_LINE_SMOOTH);
    return 1;
}

void ChOpenGLViewer::TakeDown() {
    render_camera.TakeDown();
    ortho_camera.TakeDown();

    main_shader.TakeDown();
    cloud_shader.TakeDown();
    dot_shader.TakeDown();
    sphere_shader.TakeDown();

    sphere.TakeDown();
    box.TakeDown();
    cylinder.TakeDown();
    cone.TakeDown();

    cloud.TakeDown();
    fluid.TakeDown();
    grid.TakeDown();

    particles.TakeDown();

    mpm_grid.TakeDown();
    mpm_node.TakeDown();

    fea_nodes.TakeDown();
    fea_elements.TakeDown();

    line_path.TakeDown();

    contact_renderer.TakeDown();
    graph_renderer.TakeDown();
    m_vis->stats_renderer->TakeDown();

    for (std::map<std::string, ChOpenGLMesh>::iterator iter = obj_files.begin(); iter != obj_files.end(); iter++) {
        (*iter).second.TakeDown();
    }
}

void ChOpenGLViewer::Render(bool render_stats) {
    timer_render.reset();
    timer_text.reset();
    timer_geometry.reset();

    timer_render.start();
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

#ifndef __EMSCRIPTEN__
    if (render_mode == WIREFRAME) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    } else {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
#endif

    if (render_mode != POINTS) {
        model_box.clear();
        model_sphere.clear();
        model_cone.clear();
        model_cylinder.clear();
        model_obj.clear();
        line_path_data.clear();
        for (auto s : m_vis->GetSystems()) {
            for (const auto& b : s->Get_bodylist())
                DrawVisualModel(b);
            for (const auto& l : s->Get_linklist())
                DrawVisualModel(l);
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
        size_t cloud_size = 0;
        for (auto s : m_vis->GetSystems())
            cloud_size += s->Get_bodylist().size();
        cloud_data.resize(cloud_size);
        for (auto s : m_vis->GetSystems()) {
#pragma omp parallel for
            for (int i = 0; i < s->Get_bodylist().size(); i++) {
                auto abody = s->Get_bodylist().at(i);
                ChVector<> pos = abody->GetPos();
                cloud_data[i] = glm::vec3(pos.x(), pos.y(), pos.z());
            }
        }
    }

    if (render_mode == POINTS) {
        cloud.Update(cloud_data);
        glm::mat4 model(10);
        cloud.Draw(projection, view * model);
    }

    RenderFluid();
    RenderFEA();

    RenderParticles();

    RenderGrid();
    RenderAABB();
    RenderPlots();
    RenderContacts();

    timer_geometry.stop();
    time_geometry = .5 * timer_geometry() + .5 * time_geometry;

    timer_text.start();
    if (render_stats || view_help)
        RenderStats();
    timer_text.stop();
    time_text = .5 * timer_text() + .5 * time_text;

    timer_render.stop();
    time_total = .5 * timer_render() + .5 * time_total;
    current_time = time_total;
    current_time = current_time * 0.5 + old_time * 0.5;
    old_time = current_time;
    fps = 1.0 / current_time;
}

void ChOpenGLViewer::DrawVisualModel(std::shared_ptr<ChPhysicsItem> item) {
    if (!item->GetVisualModel())
        return;

    // Get the visual model reference frame
    const ChFrame<>& X_AM = item->GetVisualModelFrame();

    for (auto& shape_instance : item->GetVisualModel()->GetShapes()) {
        const auto& shape = shape_instance.first;
        const auto& X_SM = shape_instance.second;

        ChFrame<> X_SA = X_AM * X_SM;
        auto pos = X_SA.GetPos();
        auto rot = X_SA.GetRot();
        double angle;
        Vector axis;
        rot.Q_to_AngAxis(angle, axis);

        if (ChVisualShapeSphere* sphere_shape = dynamic_cast<ChVisualShapeSphere*>(shape.get())) {
            double radius = sphere_shape->GetRadius();

            model = glm::translate(glm::mat4(1), glm::vec3(pos.x(), pos.y(), pos.z()));
            model = glm::rotate(model, float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::scale(model, glm::vec3(radius, radius, radius));
            model_sphere.push_back(model);
        } else if (ChVisualShapeEllipsoid* ellipsoid_shape = dynamic_cast<ChVisualShapeEllipsoid*>(shape.get())) {
            Vector radius = ellipsoid_shape->GetSemiaxes();

            model = glm::translate(glm::mat4(1), glm::vec3(pos.x(), pos.y(), pos.z()));
            model = glm::rotate(model, float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::scale(model, glm::vec3(radius.x(), radius.y(), radius.z()));
            model_sphere.push_back(model);
        } else if (ChVisualShapeBox* box_shape = dynamic_cast<ChVisualShapeBox*>(shape.get())) {
            const Vector& size = box_shape->GetHalflengths();

            model = glm::translate(glm::mat4(1), glm::vec3(pos.x(), pos.y(), pos.z()));
            model = glm::rotate(model, float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::scale(model, glm::vec3(size.x(), size.y(), size.z()));
            model_box.push_back(model);
        } else if (ChVisualShapeCylinder* cylinder_shape = dynamic_cast<ChVisualShapeCylinder*>(shape.get())) {
            double rad = cylinder_shape->GetRadius();
            double height = cylinder_shape->GetHeight();

            model = glm::translate(glm::mat4(1), glm::vec3(pos.x(), pos.y(), pos.z()));
            model = glm::rotate(model, float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::scale(model, glm::vec3(rad, rad, height));
            model_cylinder.push_back(model);
        } else if (ChVisualShapeCone* cone_shape = dynamic_cast<ChVisualShapeCone*>(shape.get())) {
            double radius = cone_shape->GetRadius();
            double height = cone_shape->GetHeight();

            model = glm::translate(glm::mat4(1), glm::vec3(pos.x(), pos.y(), pos.z()));
            model = glm::rotate(model, float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::scale(model, glm::vec3(radius, radius, height));
            model_cone.push_back(model);
        } else if (ChVisualShapeCapsule* capsule_shape = dynamic_cast<ChVisualShapeCapsule*>(shape.get())) {
            double rad = capsule_shape->GetRadius();
            double height = capsule_shape->GetHeight();

            model = glm::translate(glm::mat4(1), glm::vec3(pos.x(), pos.y(), pos.z()));
            model = glm::rotate(model, float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::scale(model, glm::vec3(rad, rad, height));
            model_cylinder.push_back(model);

            glm::vec3 s1 = glm::rotate(glm::vec3(0, 0, +height / 2), float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::translate(glm::mat4(1), glm::vec3(pos.x() + s1.x, pos.y() + s1.y, pos.z() + s1.z));
            model = glm::scale(model, glm::vec3((float)rad));
            model_sphere.push_back(model);

            glm::vec3 s2 = glm::rotate(glm::vec3(0, 0, -height / 2), float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::translate(glm::mat4(1), glm::vec3(pos.x() + s2.x, pos.y() + s2.y, pos.z() + s2.z));
            model = glm::scale(model, glm::vec3((float)rad));
            model_sphere.push_back(model);
        } else if (ChVisualShapeTriangleMesh* trimesh_shape = dynamic_cast<ChVisualShapeTriangleMesh*>(shape.get())) {
            model = glm::translate(glm::mat4(1), glm::vec3(pos.x(), pos.y(), pos.z()));
            model = glm::rotate(model, float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));

            if (obj_files.find(trimesh_shape->GetName()) == obj_files.end()) {
                ////std::cout << trimesh_shape->GetName() << std::endl;
                obj_files[trimesh_shape->GetName()].Initialize(trimesh_shape, mesh_color);
                obj_files[trimesh_shape->GetName()].AttachShader(&main_shader);
                model_obj[trimesh_shape->GetName()].push_back(model);
            } else {
                model_obj[trimesh_shape->GetName()].push_back(model);
            }
        } else if (ChVisualShapeLine* line_shape = dynamic_cast<ChVisualShapeLine*>(shape.get())) {
            auto mline = line_shape->GetLineGeometry();
            auto npoints = line_shape->GetNumRenderPoints();

            double maxU = 1;
            if (auto mline_path = std::dynamic_pointer_cast<geometry::ChLinePath>(mline))
                maxU = mline_path->GetPathDuration();

            ChVector<> t2 = mline->Evaluate(0.0);
            t2 = X_SA.TransformPointLocalToParent(t2);
            line_path_data.push_back(glm::vec3(t2.x(), t2.y(), t2.z()));

            for (unsigned int ig = 1; ig < npoints; ig++) {
                double mU = maxU * ((double)ig / (double)(npoints - 1));  // abscissa

                t2 = mline->Evaluate(mU);
                t2 = X_SA.TransformPointLocalToParent(t2);
                line_path_data.push_back(glm::vec3(t2.x(), t2.y(), t2.z()));
                line_path_data.push_back(glm::vec3(t2.x(), t2.y(), t2.z()));
            }

            t2 = mline->Evaluate(maxU);
            t2 = X_SA.TransformPointLocalToParent(t2);
            line_path_data.push_back(glm::vec3(t2.x(), t2.y(), t2.z()));
        }
        /*
        else if (ChVisualShapeRoundedBox* shape = dynamic_cast<ChVisualShapeRoundedBox*>(asset.get())) {
            Vector rad = shape->GetHalflengths();
            double radsphere = shape->GetRadius();
            ChVector<> pos_final = pos + center;
            model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x(), pos_final.y(), pos_final.z()));
            model = glm::rotate(model, float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::scale(model, glm::vec3(rad.x(), rad.y(), rad.z()));
            model_box.push_back(model);

            glm::vec3 local = glm::rotate(glm::vec3(rad.x(), rad.y(), rad.z()), float(angle),
                                          glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::translate(
                glm::mat4(1), glm::vec3(pos_final.x() + local.x, pos_final.y() + local.y, pos_final.z() + local.z));
            model = glm::scale(model, glm::vec3((float)radsphere));
            model_sphere.push_back(model);

            local = glm::rotate(glm::vec3(rad.x(), rad.y(), -rad.z()), float(angle),
                                glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::translate(
                glm::mat4(1), glm::vec3(pos_final.x() + local.x, pos_final.y() + local.y, pos_final.z() + local.z));
            model = glm::scale(model, glm::vec3((float)radsphere));
            model_sphere.push_back(model);

            local = glm::rotate(glm::vec3(-rad.x(), rad.y(), rad.z()), float(angle),
                                glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::translate(
                glm::mat4(1), glm::vec3(pos_final.x() + local.x, pos_final.y() + local.y, pos_final.z() + local.z));
            model = glm::scale(model, glm::vec3((float)radsphere));
            model_sphere.push_back(model);

            local = glm::rotate(glm::vec3(-rad.x(), rad.y(), -rad.z()), float(angle),
                                glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::translate(
                glm::mat4(1), glm::vec3(pos_final.x() + local.x, pos_final.y() + local.y, pos_final.z() + local.z));
            model = glm::scale(model, glm::vec3((float)radsphere));
            model_sphere.push_back(model);

            local = glm::rotate(glm::vec3(rad.x(), -rad.y(), rad.z()), float(angle),
                                glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::translate(
                glm::mat4(1), glm::vec3(pos_final.x() + local.x, pos_final.y() + local.y, pos_final.z() + local.z));
            model = glm::scale(model, glm::vec3((float)radsphere));
            model_sphere.push_back(model);

            local = glm::rotate(glm::vec3(rad.x(), -rad.y(), -rad.z()), float(angle),
                                glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::translate(
                glm::mat4(1), glm::vec3(pos_final.x() + local.x, pos_final.y() + local.y, pos_final.z() + local.z));
            model = glm::scale(model, glm::vec3((float)radsphere));
            model_sphere.push_back(model);

            local = glm::rotate(glm::vec3(-rad.x(), -rad.y(), rad.z()), float(angle),
                                glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::translate(
                glm::mat4(1), glm::vec3(pos_final.x() + local.x, pos_final.y() + local.y, pos_final.z() + local.z));
            model = glm::scale(model, glm::vec3((float)radsphere));
            model_sphere.push_back(model);

            local = glm::rotate(glm::vec3(-rad.x(), -rad.y(), -rad.z()), float(angle),
                                glm::vec3(axis.x(), axis.y(), axis.z()));
            model = glm::translate(
                glm::mat4(1), glm::vec3(pos_final.x() + local.x, pos_final.y() + local.y, pos_final.z() + local.z));
            model = glm::scale(model, glm::vec3((float)radsphere));
            model_sphere.push_back(model);
        }
        */
    }
}

void ChOpenGLViewer::RenderStats() {
    if (m_vis->GetSystems().empty())
        return;

    GLReturnedError("Start text");
    m_vis->stats_renderer->Update(window_size, dpi, fps, time_geometry, time_text, time_total);
    if (view_help) {
        m_vis->stats_renderer->GenerateHelp();
    } else {
        m_vis->stats_renderer->GenerateStats(m_vis->GetSystem(0));
    }

    m_vis->stats_renderer->Render();
}

void ChOpenGLViewer::RenderContacts() {
    if (view_contacts == false) {
        return;
    }

    for (auto s : m_vis->GetSystems())
        contact_renderer.Update(s);

    contact_renderer.Draw(projection, view);
}

void ChOpenGLViewer::RenderAABB() {
    if (view_aabb == false) {
        return;
    }

#ifdef CHRONO_MULTICORE
    uint num_rigid_shapes = 0;
    for (auto s : m_vis->m_systems_mcore) {
        num_rigid_shapes += s->data_manager->cd_data->num_rigid_shapes;
    }

    if (num_rigid_shapes <= 0)
        return;

    model_box.clear();
    model_box.resize(num_rigid_shapes);

    int start = 0;
    for (auto s : m_vis->m_systems_mcore) {
        ChMulticoreDataManager* data_manager = s->data_manager;
        custom_vector<real3>& aabb_min = data_manager->cd_data->aabb_min;
        custom_vector<real3>& aabb_max = data_manager->cd_data->aabb_max;

    #pragma omp parallel for
        for (int i = 0; i < (signed)data_manager->cd_data->num_rigid_shapes; i++) {
            real3 min_p = aabb_min[i] + data_manager->measures.collision.global_origin;
            real3 max_p = aabb_max[i] + data_manager->measures.collision.global_origin;

            real3 radius = (max_p - min_p) * .5;
            real3 center = (min_p + max_p) * .5;

            glm::mat4 model = glm::translate(glm::mat4(1), glm::vec3(center.x, center.y, center.z));
            model = glm::scale(model, glm::vec3(radius.x, radius.y, radius.z));
            model_box[start + i] = (model);
        }

        start += data_manager->cd_data->num_rigid_shapes;
    }

    if (model_box.size() > 0) {
        box.Update(model_box);
        box.Draw(projection, view);
    }
#endif
}

void ChOpenGLViewer::RenderFluid() {
#ifdef CHRONO_MULTICORE
    uint num_fluid_bodies = 0;
    for (auto s : m_vis->m_systems_mcore) {
        num_fluid_bodies += s->data_manager->num_fluid_bodies;
    }

    if (num_fluid_bodies <= 0)
        return;

    fluid_data.resize(num_fluid_bodies);

    if (render_mode != POINTS) {
        fluid.AttachShader(&sphere_shader);
    } else {
        fluid.AttachShader(&dot_shader);
    }

    int start = 0;
    for (auto s : m_vis->m_systems_mcore) {
        ChMulticoreDataManager* data_manager = s->data_manager;
        if (data_manager->num_fluid_bodies <= 0)
            continue;

    #pragma omp parallel for
        for (int i = 0; i < (signed)data_manager->num_fluid_bodies; i++) {
            real3 pos = data_manager->host_data.pos_3dof[i];
            fluid_data[start + i] = glm::vec3(pos.x, pos.y, pos.z);
        }

        if (auto f_container = std::dynamic_pointer_cast<ChFluidContainer>(data_manager->node_container)) {
            fluid.SetPointSize(float(f_container->kernel_radius * .75));
        } else if (auto p_container = std::dynamic_pointer_cast<ChParticleContainer>(data_manager->node_container)) {
            fluid.SetPointSize(float(p_container->kernel_radius * .75));
        }

        start += data_manager->num_fluid_bodies;
    }

    fluid.Update(fluid_data);
    glm::mat4 model(1);
    fluid.Draw(projection, view * model);
#endif
}

void ChOpenGLViewer::RenderParticles() {
    size_t num_particles = 0;
    for (auto s : m_vis->GetSystems()) {
        for (auto& item : s->Get_otherphysicslist()) {
            if (auto pcloud = std::dynamic_pointer_cast<ChParticleCloud>(item)) {
                if (!pcloud->GetVisualModel())
                    continue;
                num_particles += pcloud->GetNparticles();
            }
        }
    }

    if (num_particles <= 0)
        return;

    particle_data.resize(num_particles);

    if (render_mode != SOLID || particle_render_mode == POINTS)
        particles.AttachShader(&dot_shader);
    else
        particles.AttachShader(&sphere_shader);

    num_particles = 0;
    for (auto s : m_vis->GetSystems()) {
        for (auto& item : s->Get_otherphysicslist()) {
            if (auto pcloud = std::dynamic_pointer_cast<ChParticleCloud>(item)) {
                if (!pcloud->GetVisualModel())
                    continue;

                size_t n = 0;
                for (int i = 0; i < pcloud->GetNparticles(); i++) {
                    const auto& pos = pcloud->GetParticle(i).GetPos();
                    if (!m_vis->particle_selector || m_vis->particle_selector->Render(pos)) {
                        particle_data[num_particles + n++] = glm::vec3(pos.x(), pos.y(), pos.z());
                    }
                }

                num_particles += n;
            }
        }
    }

    particle_data.resize(num_particles);
    particles.SetPointSize(particle_radius);
    particles.Update(particle_data);
    glm::mat4 model(1);
    particles.Draw(projection, view * model);
}

void ChOpenGLViewer::RenderFEA() {
    /*
#ifdef CHRONO_MULTICORE
    ChSystemMulticore* parallel_system = dynamic_cast<ChSystemMulticore*>(physics_system);
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
*/
}

void ChOpenGLViewer::RenderGrid() {
    if (view_grid == false)
        return;

    grid_data.clear();
#ifdef CHRONO_MULTICORE
    if (m_vis->m_systems_mcore.empty())
        return;

    //// RADU TODO
    //// Consider adapting to render grids for more than one Multicore system
    ChMulticoreDataManager* data_manager = m_vis->m_systems_mcore[0]->data_manager;

    vec3 bins_per_axis = data_manager->settings.collision.bins_per_axis;
    real3 bin_size_vec = data_manager->measures.collision.bin_size;
    real3 min_pt = data_manager->measures.collision.min_bounding_point;
    real3 max_pt = data_manager->measures.collision.max_bounding_point;
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
#endif
    glm::mat4 model(1);
    grid.Draw(projection, view * model);
}

void ChOpenGLViewer::RenderPlots() {
    //  if (view_info == false || view_help) {
    //    return;
    //  }
    //  graph_renderer.Update(m_systems[0], window_size);
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
        case 'V':
            //         use_vsync = !use_vsync;
            //         if (use_vsync) {
            //            glfwSwapInterval(1);
            //         } else {
            //            glfwSwapInterval(0);
            //         }
            break;
        default:
            break;
    }
}

}  // end namespace opengl
}  // end namespace chrono
