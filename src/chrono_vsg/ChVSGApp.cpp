// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
// Vulkan Scene Graph viewer, this class will hopefully draw the system to the
// screen and handle input some day
// =============================================================================

#include "chrono_vsg/core/ChApiVSG.h"

#include "ChVSGApp.h"
#include "chrono/geometry/ChBox.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono_vsg/shapes/ChVSGBox.h"

using namespace chrono::vsg3d;

ChVSGApp::ChVSGApp() : m_horizonMountainHeight(0.0) {}

bool ChVSGApp::Initialize(int windowWidth, int windowHeight, const char* windowTitle, ChSystem* system) {
    if (!system) {
        return false;
    }
    m_system = system;
    m_windowTraits = ::vsg::WindowTraits::create();
    m_windowTraits->windowTitle = windowTitle;
    m_windowTraits->width = windowWidth;
    m_windowTraits->height = windowHeight;
    m_windowTraits->x = 100;
    m_windowTraits->y = 100;

    m_searchPaths = ::vsg::getEnvPaths("VSG_FILE_PATH");

    m_scenegraph = vsg::Group::create();

    // fill the scenegraph with asset definitions from the physical system
    AnalyseSystem();

    // create viewer
    m_viewer = ::vsg::Viewer::create();

    // create window
    m_window = ::vsg::Window::create(m_windowTraits);
    // if (!window) {
    if (!m_window) {
        GetLog() << "Could not create windows.\n";
        return false;
    }

    m_viewer->addWindow(m_window);

    // compute the bounds of the scene graph to help position camera
    vsg::ComputeBounds computeBounds;
    m_scenegraph->accept(computeBounds);
    vsg::dvec3 centre = (computeBounds.bounds.min + computeBounds.bounds.max) * 0.5;
    double radius = vsg::length(computeBounds.bounds.max - computeBounds.bounds.min) * 0.6;
    double nearFarRatio = 0.001;
    GetLog() << "BoundMin = {" << computeBounds.bounds.min.x << ";" << computeBounds.bounds.min.y << ";"
             << computeBounds.bounds.min.z << "}\n";
    GetLog() << "BoundMax = {" << computeBounds.bounds.max.x << ";" << computeBounds.bounds.max.y << ";"
             << computeBounds.bounds.max.z << "}\n";
    // set up the camera
    auto lookAt = vsg::LookAt::create(centre + vsg::dvec3(0.0, -radius * 3.5, 0.0), centre, vsg::dvec3(0.0, 0.0, 1.0));

    auto perspective = vsg::Perspective::create(
        30.0, static_cast<double>(m_window->extent2D().width) / static_cast<double>(m_window->extent2D().height),
        nearFarRatio * radius, radius * 4.5);

    auto camera = vsg::Camera::create(perspective, lookAt, vsg::ViewportState::create(m_window->extent2D()));

    // add close handler to respond to pressing the window close window button and pressing escape
    m_viewer->addEventHandler(::vsg::CloseHandler::create(m_viewer));

    // add a trackball event handler to control the camera view use the mouse
    m_viewer->addEventHandler(::vsg::Trackball::create(camera));

    auto commandGraph = vsg::createCommandGraphForView(m_window, camera, m_scenegraph);
    m_viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});

    m_viewer->compile();
    return true;
}

ChVSGApp::~ChVSGApp() {
    ;
}

void ChVSGApp::Render() {
    m_viewer->handleEvents();
    m_viewer->update();
    m_viewer->recordAndSubmit();
    m_viewer->present();
}

void ChVSGApp::AnalyseSystem() {
    // analyse system, look for bodies and assets
    for (auto body : m_system->Get_bodylist()) {
        GetLog() << "ChVSGApp::ChVSGApp(): Body " << body << "\n";
        // position of the body
        const Vector pos = body->GetFrame_REF_to_abs().GetPos();
        // rotation of the body
        Quaternion rot = body->GetFrame_REF_to_abs().GetRot();
        double angle;
        Vector axis;
        rot.Q_to_AngAxis(angle, axis);
        for (int i = 0; i < body->GetAssets().size(); i++) {
            auto asset = body->GetAssets().at(i);

            if (!std::dynamic_pointer_cast<ChVisualization>(asset)) {
                continue;
            }
            ChVisualization* visual_asset = ((ChVisualization*)(asset.get()));
            // position of the asset
            Vector center = visual_asset->Pos;
            // rotate asset pos into global frame
            center = rot.Rotate(center);
            // Get the local rotation of the asset
            Quaternion lrot = visual_asset->Rot.Get_A_quaternion();
            // add the local rotation to the rotation of the body
            lrot = rot % lrot;
            lrot.Normalize();
            lrot.Q_to_AngAxis(angle, axis);
            vsg::vec4 color(1, 1, 1, 1);
            if (ChBoxShape* box_shape = dynamic_cast<ChBoxShape*>(asset.get())) {
                GetLog() << "Found BoxShape!\n";
                vsg::vec3 size(box_shape->GetBoxGeometry().GetSize().x(), box_shape->GetBoxGeometry().GetSize().y(),
                               box_shape->GetBoxGeometry().GetSize().z());
                ChVector<> pos_final = pos + center;
                // GetLog() << "pos final = " << pos_final << "\n";

                // model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x(), pos_final.y(), pos_final.z()));
                // model = glm::rotate(model, float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
                // model = glm::scale(model, glm::vec3(radius, radius, radius));
                // model_sphere.push_back(model);
                auto transform = vsg::MatrixTransform::create();
                transform->setMatrix(vsg::translate(pos_final.x(), pos_final.y(), pos_final.z()) *
                                     vsg::rotate(angle, axis.x(), axis.y(), axis.z()));
                ChVSGBox theBox;
                vsg::ref_ptr<vsg::Node> node = theBox.createTexturedNode(size, color, transform);
                m_scenegraph->addChild(node);
                m_transformList.push_back(transform);  // we will need it later
            }
        }
    }
}