// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// OpenGL-based visualization wrapper for vehicles.
//
// =============================================================================

#include "chrono_vehicle/ChVehicleVisualSystemOpenGL.h"
#include "chrono_vsg/ChGuiComponentVSG.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

class ChVehicleKeyboardHandlerOpenGL : public opengl::ChOpenGLEventCB {
  public:
    ChVehicleKeyboardHandlerOpenGL(ChVehicleVisualSystemOpenGL* app) : m_app(app) {}

    // Keyboard events for chase-cam and interactive driver control
    virtual bool CallbackKeyboard(GLFWwindow* window, int key, int scancode, int action, int mode) override {
        if (!m_app->m_vehicle)
            return false;

        switch (key) {
            case GLFW_KEY_V:
                m_app->m_vehicle->LogConstraintViolations();
                return true;
            case GLFW_KEY_LEFT:
                m_app->m_camera->Turn(-1);
                return true;
            case GLFW_KEY_RIGHT:
                m_app->m_camera->Turn(1);
                return true;
            case GLFW_KEY_DOWN:
                m_app->m_camera->Zoom(+1);
                return true;
            case GLFW_KEY_UP:
                m_app->m_camera->Zoom(-1);
                return true;
            case GLFW_KEY_PAGE_UP:
                m_app->m_camera->Raise(-1);
                return true;
            case GLFW_KEY_PAGE_DOWN:
                m_app->m_camera->Raise(+1);
                return true;
            case GLFW_KEY_1:
                m_app->SetChaseCameraState(utils::ChChaseCamera::Chase);
                return true;
            case GLFW_KEY_2:
                m_app->SetChaseCameraState(utils::ChChaseCamera::Follow);
                return true;
            case GLFW_KEY_3:
                m_app->SetChaseCameraState(utils::ChChaseCamera::Track);
                return true;
            case GLFW_KEY_4:
                m_app->SetChaseCameraState(utils::ChChaseCamera::Inside);
                return true;
            case GLFW_KEY_5:
                m_app->SetChaseCameraState(utils::ChChaseCamera::Free);
                return true;
        }

        return false;
    }

    virtual bool CallbackMouseButton(GLFWwindow* window, int button, int action, int mods) override { return false; }
    virtual bool CallbackMousePos(GLFWwindow* window, double x, double y) override { return false; }

  private:
    ChVehicleVisualSystemOpenGL* m_app;
};

// -----------------------------------------------------------------------------

ChVehicleVisualSystemOpenGL::ChVehicleVisualSystemOpenGL() : ChVisualSystemOpenGL() {
    // Default camera uses Z up
    SetCameraVertical(CameraVerticalDir::Z);
}

ChVehicleVisualSystemOpenGL::~ChVehicleVisualSystemOpenGL() {}

void ChVehicleVisualSystemOpenGL::Initialize() {
    // Add keyboard handler
    AddUserEventReceiver(chrono_types::make_shared<ChVehicleKeyboardHandlerOpenGL>(this));

    // Invoke the base Initialize method
    ChVisualSystemOpenGL::Initialize();

    // Initialize chase-cam mode
    SetChaseCameraState(utils::ChChaseCamera::State::Chase);
}

void ChVehicleVisualSystemOpenGL::Advance(double step) {
    // Update the ChChaseCamera: take as many integration steps as needed to exactly reach the value 'step'
    double t = 0;
    while (t < step) {
        double h = std::min<>(m_stepsize, step - t);
        m_camera->Update(h);
        t += h;
    }

    // Update the OpenGL camera
    SetCameraPosition(m_camera->GetCameraPos());
    SetCameraTarget(m_camera->GetTargetPos());
}

}  // namespace vehicle
}  // namespace chrono
