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

using namespace chrono::vsg3d;

ChVSGApp::ChVSGApp(ChSystem* system) : m_system(system) {
    if (!system) {
        GetLog() << "ChVSGApp::ChVSGApp(): Chrono System undefined!\n";
        return;
    }

    m_windowTraits = ::vsg::WindowTraits::create();
    m_windowTraits->windowTitle = "Chrono VSG Viewer";

    m_searchPaths = ::vsg::getEnvPaths("VSG_FILE_PATH");

    // analyze system, look for bodies and assets
    for (auto body : m_system->Get_bodylist()) {
        GetLog() << "ChVSGApp::ChVSGApp(): Body " << body << "\n";
        for (auto asset : body->GetAssets()) {
            GetLog() << "   Asset " << asset << "\n";
        }
        // DrawObject(body);
    }

    // create viewer
    m_viewer = ::vsg::Viewer::create();

    // create window
    //::vsg::ref_ptr<::vsg::Window> window(::vsg::Window::create(m_windowTraits));
    m_window = ::vsg::Window::create(m_windowTraits);
    // if (!window) {
    if (!m_window) {
        GetLog() << "Could not create windows.\n";
        return;
    }

    m_viewer->addWindow(m_window);

    // add close handler to respond to pressing the window close window button and pressing escape
    m_viewer->addEventHandler(::vsg::CloseHandler::create(m_viewer));

    // add a trackball event handler to control the camera view use the mouse
    // viewer->addEventHandler(::vsg::Trackball::create(camera));
}

ChVSGApp::~ChVSGApp() {
    ;
}
