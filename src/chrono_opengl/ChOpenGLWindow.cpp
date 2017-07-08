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
// OpenGL window singleton, this class manages the opengl context and window
// =============================================================================

#include "chrono_opengl/ChOpenGLWindow.h"

namespace chrono {
namespace opengl {

void ChOpenGLWindow::Initialize(int size_x, int size_y, const char* title, ChSystem* msystem) {
    if (!glfwInit()) {
        std::cout << "could not initialize glfw- exiting" << std::endl;
        exit(EXIT_FAILURE);
    }

    glfwSetErrorCallback(CallbackError);

    // glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    // glfwWindowHint(GLFW_SAMPLES, 16);

    window = glfwCreateWindow(size_x, size_y, title, NULL, NULL);
    if (!window) {
        std::cout << "could not create window - exiting" << std::endl;
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwMakeContextCurrent(window);

    // Disable vsync!!
    glfwSwapInterval(0);

    GLReturnedError("Initialize GLFW");

    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (err != GLEW_OK) {
        fprintf(stdout, "Failed to initialize GLEW\n");
        fprintf(stdout, "%s\n", glewGetErrorString(err));
        glfwTerminate();
        return;
    }

    glfwSetWindowCloseCallback(window, CallbackClose);
    glfwSetFramebufferSizeCallback(window, CallbackReshape);
    glfwSetKeyCallback(window, CallbackKeyboard);
    glfwSetMouseButtonCallback(window, CallbackMouseButton);
    glfwSetCursorPosCallback(window, CallbackMousePos);

    GLReturnedError("Initialize GLEW");
    GLFWGetVersion(window);

    viewer = new ChOpenGLViewer(msystem);

    glfwGetFramebufferSize(window, &size_x, &size_y);
    if (size_y > 0) {
        viewer->window_size = glm::ivec2(size_x, size_y);
        viewer->window_aspect = float(size_x) / float(size_y);

        GLFWmonitor* primary = glfwGetPrimaryMonitor();
        glfwGetMonitorPhysicalSize(primary, &viewer->window_physical_size.x, &viewer->window_physical_size.y);
        glfwGetMonitorPos(primary, &viewer->window_position.x, &viewer->window_position.y);
        const GLFWvidmode* mode = glfwGetVideoMode(primary);
        viewer->dpi = mode->width / (viewer->window_physical_size.x / 25.4);
    }
    if (!viewer->Initialize()) {
        printf("Viewer Initialization Failed\n");
    }
    glfwSetWindowUserPointer(window, viewer);
    GLReturnedError("Initialize Viewer ");
    poll_frame = 0;
}

void ChOpenGLWindow::StartDrawLoop(double time_step) {
    GLReturnedError("Start Draw Loop");
    ChOpenGLViewer* pointer = ((ChOpenGLViewer*)(glfwGetWindowUserPointer(window)));
    while (Active()) {
        pointer->Update(time_step);
        Render();
    }
}
bool ChOpenGLWindow::DoStepDynamics(double time_step) {
    ChOpenGLViewer* pointer = ((ChOpenGLViewer*)(glfwGetWindowUserPointer(window)));
    return pointer->Update(time_step);
}
void ChOpenGLWindow::Render() {
    ChOpenGLViewer* pointer = ((ChOpenGLViewer*)(glfwGetWindowUserPointer(window)));
    if (pointer->pause_vis == false) {
        glEnable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_CULL_FACE);
        glClearColor(18.0f / 255.0f, 26.0f / 255.0f, 32.0f / 255.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        GLReturnedError("Before Render");
        pointer->Render();
        GLReturnedError("After Render");
        glfwSwapBuffers(window);
    }

    glfwPollEvents();
}

bool ChOpenGLWindow::Active() {
    return !glfwWindowShouldClose(window);
}

bool ChOpenGLWindow::Running() {
    ChOpenGLViewer* pointer = ((ChOpenGLViewer*)(glfwGetWindowUserPointer(window)));
    return !pointer->pause_sim;
}

// Pause simulation
void ChOpenGLWindow::Pause() {
    ChOpenGLViewer* pointer = ((ChOpenGLViewer*)(glfwGetWindowUserPointer(window)));
    pointer->pause_sim = true;
}

void ChOpenGLWindow::GLFWGetVersion(GLFWwindow* main_window) {
    int major, minor, rev;
    major = glfwGetWindowAttrib(main_window, GLFW_CONTEXT_VERSION_MAJOR);
    minor = glfwGetWindowAttrib(main_window, GLFW_CONTEXT_VERSION_MINOR);
    rev = glfwGetWindowAttrib(main_window, GLFW_CONTEXT_REVISION);
    fprintf(stdout, "Version: %d.%d.%d\n", major, minor, rev);

    const GLubyte* vendor = glGetString(GL_VENDOR);
    const GLubyte* renderer = glGetString(GL_RENDERER);
    const GLubyte* version = glGetString(GL_VERSION);
    const GLubyte* glsl_ver = glGetString(GL_SHADING_LANGUAGE_VERSION);

    printf("%s : %s (%s)\n >> GLSL: %s\n", vendor, renderer, version, glsl_ver);
}

void ChOpenGLWindow::CallbackError(int error, const char* description) {
    fputs(description, stderr);
    fputs("\n", stderr);
}

void ChOpenGLWindow::CallbackClose(GLFWwindow* window) {
    ChOpenGLViewer* pointer = ((ChOpenGLViewer*)(glfwGetWindowUserPointer(window)));
    pointer->TakeDown();
    glfwSetWindowShouldClose(window, GL_TRUE);
}

void ChOpenGLWindow::CallbackReshape(GLFWwindow* window, int w, int h) {
    ChOpenGLViewer* pointer = ((ChOpenGLViewer*)(glfwGetWindowUserPointer(window)));
    if (h > 0) {
        pointer->window_size = glm::ivec2(w, h);
        pointer->window_aspect = float(w) / float(h);
    }

    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    glfwGetMonitorPhysicalSize(primary, &pointer->window_physical_size.x, &pointer->window_physical_size.y);
    glfwGetMonitorPos(primary, &pointer->window_position.x, &pointer->window_position.y);
    const GLFWvidmode* mode = glfwGetVideoMode(primary);
    pointer->dpi = mode->width / (pointer->window_physical_size.x / 25.4);
}

void ChOpenGLWindow::CallbackKeyboard(GLFWwindow* window, int key, int scancode, int action, int mode) {
    ChOpenGLViewer* pointer = ((ChOpenGLViewer*)(glfwGetWindowUserPointer(window)));

    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GL_TRUE);
    }

    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        pointer->HandleInput(key, 0, 0);
    }
}

void ChOpenGLWindow::CallbackMouseButton(GLFWwindow* window, int button, int state, int mods) {
    ChOpenGLViewer* pointer = ((ChOpenGLViewer*)(glfwGetWindowUserPointer(window)));
    double x, y;
    glfwGetCursorPos(window, &x, &y);

    pointer->render_camera.SetPos(button, state, (int)x, (int)y);
}
void ChOpenGLWindow::CallbackMousePos(GLFWwindow* window, double x, double y) {
    ChOpenGLViewer* pointer = ((ChOpenGLViewer*)(glfwGetWindowUserPointer(window)));

    pointer->render_camera.Move2D((int)x, (int)y);
}
}
}
