// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Radu Serban,  Hammad Mazhar
// =============================================================================

#include "chrono_opengl/ChVisualSystemOpenGL.h"

#ifdef CHRONO_MULTICORE
    #include "chrono_multicore/physics/ChSystemMulticore.h"
#endif

#include "chrono_thirdparty/stb/stb_image_write.h"

namespace chrono {
namespace opengl {

ChVisualSystemOpenGL::ChVisualSystemOpenGL()
    : viewer(nullptr),
      window(nullptr),
      m_width(1200),
      m_height(800),
      m_camera_pos(1, 0, 0),
      m_camera_targ(0, 0, 0),
      m_camera_up(0, 0, 1),
      m_camera_scale(0.5f),
      m_camera_near(0.1f),
      m_camera_far(1000.0f),
      particle_selector(nullptr),
      render_stats(true) {
    stats_renderer = chrono_types::make_shared<ChOpenGLStatsDefault>();
}

ChVisualSystemOpenGL::~ChVisualSystemOpenGL() {
    viewer->TakeDown();
    delete viewer;
    if (window)
        glfwDestroyWindow(window);
}

void ChVisualSystemOpenGL::SetWindowSize(unsigned int width, unsigned int height) {
    m_width = (int)width;
    m_height = (int)height;
}

void ChVisualSystemOpenGL::SetWindowTitle(const std::string& win_title) {
    m_win_title = win_title;
}

int ChVisualSystemOpenGL::AddCamera(const ChVector<>& pos, ChVector<> target) {
    m_camera_pos = pos;
    m_camera_targ = target;
    if (viewer) {
        viewer->render_camera.camera_position = glm::vec3(m_camera_pos.x(), m_camera_pos.y(), m_camera_pos.z());
        viewer->render_camera.camera_look_at = glm::vec3(m_camera_targ.x(), m_camera_targ.y(), m_camera_targ.z());
    }
    return 0;
}

void ChVisualSystemOpenGL::SetCameraPosition(int id, const ChVector<>& pos) {
    m_camera_pos = pos;
    if (viewer)
        viewer->render_camera.camera_position = glm::vec3(m_camera_pos.x(), m_camera_pos.y(), m_camera_pos.z());
}

void ChVisualSystemOpenGL::SetCameraTarget(int id, const ChVector<>& target) {
    m_camera_targ = target;
    if (viewer)
        viewer->render_camera.camera_look_at = glm::vec3(m_camera_targ.x(), m_camera_targ.y(), m_camera_targ.z());
}

void ChVisualSystemOpenGL::SetCameraPosition(const ChVector<>& pos) {
    m_camera_pos = pos;
    if (viewer)
        viewer->render_camera.camera_position = glm::vec3(m_camera_pos.x(), m_camera_pos.y(), m_camera_pos.z());
}

void ChVisualSystemOpenGL::SetCameraTarget(const ChVector<>& target) {
    m_camera_targ = target;
    if (viewer)
        viewer->render_camera.camera_look_at = glm::vec3(m_camera_targ.x(), m_camera_targ.y(), m_camera_targ.z());
}

void ChVisualSystemOpenGL::SetCameraVertical(CameraVerticalDir vert) {
    m_camera_up = (vert == CameraVerticalDir::Y) ? ChVector<>(0, 1, 0) : ChVector<>(0, 0, 1);
    if (viewer) {
        viewer->render_camera.camera_up = glm::vec3(m_camera_up.x(), m_camera_up.y(), m_camera_up.z());
    }
}

void ChVisualSystemOpenGL::SetCameraVertical(const ChVector<>& up) {
    m_camera_up = up;
    if (viewer) {
        viewer->render_camera.camera_up = glm::vec3(m_camera_up.x(), m_camera_up.y(), m_camera_up.z());
    }
}

void ChVisualSystemOpenGL::SetCameraProperties(float scale, float near_clip_dist, float far_clip_dist) {
    m_camera_scale = scale;
    m_camera_near = near_clip_dist;
    m_camera_far = far_clip_dist;
    if (viewer) {
        viewer->render_camera.camera_scale = m_camera_scale;
        viewer->render_camera.SetClipping(m_camera_near, m_camera_far);
    }
}

void ChVisualSystemOpenGL::SetRenderMode(RenderMode mode) {
    m_solid_mode = mode;
    if (viewer)
        viewer->render_mode = m_solid_mode;
}

void ChVisualSystemOpenGL::SetParticleRenderMode(float radius, RenderMode mode) {
    m_particle_render_mode = mode;
    m_particle_radius = radius;

    if (viewer) {
        viewer->particle_radius = m_particle_radius;
        viewer->particle_render_mode = m_particle_render_mode;
    }
}

void ChVisualSystemOpenGL::AttachStatsRenderer(std::shared_ptr<ChOpenGLStats> renderer) {
    stats_renderer = renderer;
    if (viewer)
        stats_renderer->Initialize(&viewer->render_camera);
}

void ChVisualSystemOpenGL::AttachSystem(ChSystem* sys) {
    ChVisualSystem::AttachSystem(sys);
#ifdef CHRONO_MULTICORE
    ChSystemMulticore* msys = dynamic_cast<ChSystemMulticore*>(sys);
    if (msys)
        m_systems_mcore.push_back(msys);
#endif
}

// -----------------------------------------------------------------------------

void ChVisualSystemOpenGL::Initialize() {
    viewer = new ChOpenGLViewer(this);

    viewer->render_camera.camera_position = glm::vec3(m_camera_pos.x(), m_camera_pos.y(), m_camera_pos.z());
    viewer->render_camera.camera_look_at = glm::vec3(m_camera_targ.x(), m_camera_targ.y(), m_camera_targ.z());
    viewer->render_camera.camera_up = glm::vec3(m_camera_up.x(), m_camera_up.y(), m_camera_up.z());
    viewer->render_camera.camera_scale = m_camera_scale;
    viewer->render_camera.SetClipping(m_camera_near, m_camera_far);

    viewer->render_mode = m_solid_mode;
    viewer->particle_radius = m_particle_radius;
    viewer->particle_render_mode = m_particle_render_mode;

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

    window = glfwCreateWindow(m_width, m_height, m_win_title.c_str(), NULL, NULL);
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

    glfwGetFramebufferSize(window, &m_width, &m_height);
    if (m_height > 0) {
        viewer->window_size = glm::ivec2(m_width, m_height);
        viewer->window_aspect = float(m_width) / float(m_height);

        GLFWmonitor* primary = glfwGetPrimaryMonitor();
        glfwGetMonitorPhysicalSize(primary, &viewer->window_physical_size.x, &viewer->window_physical_size.y);
        glfwGetMonitorPos(primary, &viewer->window_position.x, &viewer->window_position.y);
        const GLFWvidmode* mode = glfwGetVideoMode(primary);
        viewer->dpi = mode->width / (viewer->window_physical_size.x / 25.4);
    }

    if (!viewer->Initialize()) {
        printf("Viewer Initialization Failed\n");
    }

    glfwSetWindowUserPointer(window, this);

    GLReturnedError("Initialize Viewer ");
}

// -----------------------------------------------------------------------------

void ChVisualSystemOpenGL::GLFWGetVersion(GLFWwindow* main_window) {
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

void ChVisualSystemOpenGL::CallbackError(int error, const char* description) {
    fputs(description, stderr);
    fputs("\n", stderr);
}

void ChVisualSystemOpenGL::CallbackClose(GLFWwindow* window) {
    ChVisualSystemOpenGL* vsGL = (ChVisualSystemOpenGL*)glfwGetWindowUserPointer(window);
    vsGL->viewer->TakeDown();
    glfwSetWindowShouldClose(window, GL_TRUE);
}

void ChVisualSystemOpenGL::CallbackReshape(GLFWwindow* window, int w, int h) {
    ChVisualSystemOpenGL* vsGL = (ChVisualSystemOpenGL*)glfwGetWindowUserPointer(window);
    if (h > 0) {
        vsGL->viewer->window_size = glm::ivec2(w, h);
        vsGL->viewer->window_aspect = float(w) / float(h);
    }

    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    glfwGetMonitorPhysicalSize(primary, &vsGL->viewer->window_physical_size.x, &vsGL->viewer->window_physical_size.y);
    glfwGetMonitorPos(primary, &vsGL->viewer->window_position.x, &vsGL->viewer->window_position.y);
    const GLFWvidmode* mode = glfwGetVideoMode(primary);
    vsGL->viewer->dpi = mode->width / (vsGL->viewer->window_physical_size.x / 25.4);
}

void ChVisualSystemOpenGL::CallbackKeyboard(GLFWwindow* window, int key, int scancode, int action, int mode) {
    ChVisualSystemOpenGL* vsGL = (ChVisualSystemOpenGL*)glfwGetWindowUserPointer(window);

    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GL_TRUE);
    }

    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        for (auto ue : vsGL->user_receivers) {
            if (ue->CallbackKeyboard(window, key, scancode, action, mode))
                return;
        }

        vsGL->viewer->HandleInput(key, 0, 0);
    }
}

void ChVisualSystemOpenGL::CallbackMouseButton(GLFWwindow* window, int button, int state, int mods) {
    ChVisualSystemOpenGL* vsGL = (ChVisualSystemOpenGL*)glfwGetWindowUserPointer(window);

    for (auto ue : vsGL->user_receivers) {
        if (ue->CallbackMouseButton(window, button, state, mods))
            return;
    }

    double x, y;
    glfwGetCursorPos(window, &x, &y);
    vsGL->viewer->render_camera.SetPos(button, state, (int)x, (int)y);
}

void ChVisualSystemOpenGL::CallbackMousePos(GLFWwindow* window, double x, double y) {
    ChVisualSystemOpenGL* vsGL = (ChVisualSystemOpenGL*)glfwGetWindowUserPointer(window);

    for (auto ue : vsGL->user_receivers) {
        if (ue->CallbackMousePos(window, x, y))
            return;
    }

    vsGL->viewer->render_camera.Move2D((int)x, (int)y);
}

// -----------------------------------------------------------------------------

void ChVisualSystemOpenGL::OnSetup(ChSystem* sys) {}

void ChVisualSystemOpenGL::OnUpdate(ChSystem* sys) {}

void ChVisualSystemOpenGL::OnClear(ChSystem* sys) {}

// -----------------------------------------------------------------------------

void ChVisualSystemOpenGL::BindAll() {}

void ChVisualSystemOpenGL::BindItem(std::shared_ptr<ChPhysicsItem> item) {}

bool ChVisualSystemOpenGL::Run() {
    return !glfwWindowShouldClose(window);
}

// -----------------------------------------------------------------------------

void ChVisualSystemOpenGL::Render() {
    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_CULL_FACE);
    glClearColor(18.0f / 255.0f, 26.0f / 255.0f, 32.0f / 255.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    GLReturnedError("Before Render");
    viewer->Render(render_stats);
    GLReturnedError("After Render");
    glfwSwapBuffers(window);
    glfwPollEvents();
}

// -----------------------------------------------------------------------------

void ChVisualSystemOpenGL::WrapRenderStep(void* stepFunction) {
    (*((std::function<void()>*)stepFunction))();
}

// -----------------------------------------------------------------------------

void ChVisualSystemOpenGL::WriteImageToFile(const std::string& filename) {
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    GLsizei nrChannels = 3;
    GLsizei stride = nrChannels * width;
    stride += (stride % 4) ? (4 - stride % 4) : 0;
    GLsizei bufferSize = stride * height;
    std::vector<char> buffer(bufferSize);
    glPixelStorei(GL_PACK_ALIGNMENT, 4);
    glReadBuffer(GL_FRONT);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, buffer.data());
    stbi_flip_vertically_on_write(true);
    stbi_write_png(filename.c_str(), width, height, nrChannels, buffer.data(), stride);
}

}  // namespace opengl
}  // namespace chrono
