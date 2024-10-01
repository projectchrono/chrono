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
// Screen capture code from https://github.com/vsg-dev/vsgExamples.git
//
// =============================================================================
// Radu Serban, Rainer Gericke
// =============================================================================

#include <algorithm>
#include <cstddef>
#include <cctype>
#include <sstream>
#include <iomanip>

#include "chrono/utils/ChUtils.h"

#include "chrono_vsg/ChVisualSystemVSG.h"
#include "chrono_vsg/utils/ChConversionsVSG.h"
#include "chrono_vsg/utils/ChUtilsVSG.h"

namespace chrono {
namespace vsg3d {

using namespace std;

// -----------------------------------------------------------------------------

class ChMainGuiVSG : public vsg::Inherit<vsg::Command, ChMainGuiVSG> {
  public:
    vsg::ref_ptr<vsgImGui::Texture> texture;

    ChMainGuiVSG(ChVisualSystemVSG* app, vsg::ref_ptr<vsg::Options> options = {}, float tex_height = 64)
        : m_app(app), m_tex_height(tex_height) {
        auto texData = vsg::read_cast<vsg::Data>(m_app->m_logo_filename, options);
        texture = vsgImGui::Texture::create_if(texData, texData);
    }

    // we need to compile textures before we can use them for rendering
    void compile(vsg::Context& context) override {
        if (texture)
            texture->compile(context);
    }

    // Example here taken from the Dear imgui comments (mostly)
    void record(vsg::CommandBuffer& cb) const override {
        // Display logo first, so gui elements can cover it.
        // When the logo covers gui elements, sometimes gui malfunctions occur.
        if (texture) {
            // UV in the logo texture - usually rectangular
            ImVec2 squareUV(1.0f, 1.0f);

            if (m_app->m_show_logo) {
                const float sizey = m_tex_height;
                const float sizex = sizey * static_cast<float>(texture->width) / texture->height;
                const float pad = 10;

                // Copied from imgui_demo.cpp simple overlay
                ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                                                ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
                                                ImGuiWindowFlags_NoNav;
                const ImGuiViewport* viewport = ImGui::GetMainViewport();
                ImVec2 work_pos = viewport->WorkPos;  // Use work area to avoid menu-bar/task-bar, if any!
                ImVec2 work_size = viewport->WorkSize;
                ImVec2 window_pos, window_pos_pivot;
                window_pos.x = work_pos.x + work_size.x - sizex - m_app->m_logo_pos.x() - pad;
                window_pos.y = work_pos.y + sizey + m_app->m_logo_pos.y() + pad;
                window_pos_pivot.x = 0.0f;
                window_pos_pivot.y = 1.0f;
                ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
                window_flags |= ImGuiWindowFlags_NoMove;
                ImGui::SetNextWindowBgAlpha(0.0f);  // Transparent background
                ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
                ImGui::Begin("vsgCS UI", nullptr, window_flags);

                // Display a rectangle from the VSG logo
                ImGui::Image(texture->id(cb.deviceID), ImVec2(sizex, sizey), ImVec2(0.0f, 0.0f), squareUV);

                ImGui::End();
                ImGui::PopStyleVar();
            }
        }

        // Render GUI
        if (m_app->m_show_gui) {
            for (auto& gui : m_app->m_gui) {
                if (gui->IsVisible())
                    gui->render();
            }
        }
    }
    ChVisualSystemVSG* m_app;
    float m_tex_height;
};

// -----------------------------------------------------------------------------

class ChBaseGuiComponentVSG : public ChGuiComponentVSG {
  public:
    ChBaseGuiComponentVSG(ChVisualSystemVSG* app) : m_app(app) {}

    // Example here taken from the Dear imgui comments (mostly)
    virtual void render() override {
        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
        ////ImGui::SetNextWindowPos(ImVec2(5.0f, 5.0f));
        ImGui::Begin("Simulation");

        if (ImGui::BeginTable("SimTable", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                              ImVec2(0.0f, 0.0f))) {
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Model Time:");
            ImGui::TableNextColumn();
            ImGui::Text("%8.3f s", m_app->GetSimulationTime());

            ImGui::TableNextRow();
            double current_time = double(clock()) / double(CLOCKS_PER_SEC);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Wall Clock Time:");
            ImGui::TableNextColumn();
            ImGui::Text("%8.3f s", current_time - m_app->m_start_time);

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Real Time Factor:");
            ImGui::TableNextColumn();
            ImGui::Text("%8.3f", m_app->GetSimulationRTF());

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Rendering FPS:");
            ImGui::TableNextColumn();
            ImGui::Text("%8.3f", m_app->GetRenderingFPS());

            ImGui::EndTable();
        }

        ImGui::Spacing();

        if (ImGui::BeginTable("Frames", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                              ImVec2(0.0f, 0.0f))) {
            ImGui::TextUnformatted("COG:");
            ImGui::TableNextColumn();
            static bool bCOG_frame_active = false;
            if (ImGui::Checkbox("COG", &bCOG_frame_active))
                m_app->ToggleCOGFrameVisibility();
            ImGui::TableNextColumn();
            float cog_frame_scale = m_app->m_cog_frame_scale;
            ImGui::PushItemWidth(120.0f);
            ImGui::SliderFloat("scale##cog", &cog_frame_scale, 0.1f, 10.0f);
            ImGui::PopItemWidth();
            m_app->m_cog_frame_scale = cog_frame_scale;

            ImGui::TableNextRow();
            ImGui::TextUnformatted("Joint:");
            ImGui::TableNextColumn();
            static bool bJoint_frame_active = false;
            if (ImGui::Checkbox("Joint", &bJoint_frame_active))
                m_app->ToggleJointFrameVisibility();
            ImGui::TableNextColumn();
            float joint_frame_scale = m_app->m_joint_frame_scale;
            ImGui::PushItemWidth(120.0f);
            ImGui::SliderFloat("scale##joint", &joint_frame_scale, 0.1f, 5.0f);
            ImGui::PopItemWidth();
            m_app->m_joint_frame_scale = joint_frame_scale;

            ImGui::EndTable();
        }

        ImGui::Spacing();

        if (ImGui::Button("Quit"))
            m_app->Quit();

        ImGui::End();
    }

    ChVisualSystemVSG* m_app;
};

class ChCameraGuiComponentVSG : public ChGuiComponentVSG {
  public:
    ChCameraGuiComponentVSG(ChVisualSystemVSG* app) : m_app(app) { m_visible = false; }

    virtual void render() override {
        auto p = m_app->GetCameraPosition();
        auto t = m_app->GetCameraTarget();

        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
        ////ImGui::SetNextWindowPos(ImVec2(5.0f, 5.0f));
        ImGui::Begin("Camera");

        if (ImGui::BeginTable("Location", 4, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                              ImVec2(0.0f, 0.0f))) {
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Location");
            for (int i = 0; i < 3; i++) {
                ImGui::TableNextColumn();
                ImGui::Text(" %5.1f", p[i]);
            }

            ImGui::TableNextRow();

            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Look-at");
            for (int i = 0; i < 3; i++) {
                ImGui::TableNextColumn();
                ImGui::Text(" %5.1f", t[i]);
            }

            ImGui::EndTable();

            ImGui::End();
        }
    }

    ChVisualSystemVSG* m_app;
};

class ChColorbarGuiComponentVSG : public ChGuiComponentVSG {
  public:
    ChColorbarGuiComponentVSG(const std::string& title, double min_val, double max_val)
        : m_title(title), m_min_val(min_val), m_max_val(max_val) {}

    //// RADU TODO
    ////   replace with a proper texture.
    ////   see https://github.com/libigl/libigl/issues/1388

    virtual void render() override {
        char label[64];
        int nstr = sizeof(label) - 1;

        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
        ImGui::Begin(m_title.c_str());

        float alpha = 1.0f;
        float cv = 0.9f;
        float cv13 = cv / 3;
        float cv23 = 2 * cv13;
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0, 0.0, cv, alpha));
        snprintf(label, nstr, "%.3f", m_min_val);
        ImGui::Button(label);
        ImGui::PopStyleColor(1);
        ImGui::SameLine();
        double stride = m_max_val - m_min_val;
        double val = m_min_val + stride * 1.0 / 6.0;
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0, cv13, cv, alpha));
        snprintf(label, nstr, "%.3f", val);
        ImGui::Button(label);
        ImGui::PopStyleColor(1);
        ImGui::SameLine();
        val = m_min_val + stride * 2.0 / 6.0;
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0, cv23, cv, alpha));
        snprintf(label, nstr, "%.3f", val);
        ImGui::Button(label);
        ImGui::PopStyleColor(1);
        ImGui::SameLine();
        val = m_min_val + 0.5 * stride;
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0, cv, 0.0, alpha));
        snprintf(label, nstr, "%.3f", val);
        ImGui::Button(label);
        ImGui::PopStyleColor(1);
        ImGui::SameLine();
        val = m_min_val + stride * 4.0 / 6.0;
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(cv, cv23, 0.0, alpha));
        snprintf(label, nstr, "%.3f", val);
        ImGui::Button(label);
        ImGui::PopStyleColor(1);
        ImGui::SameLine();
        val = m_min_val + stride * 5.0 / 6.0;
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(cv, cv13, 0.0, alpha));
        snprintf(label, nstr, "%.3f", val);
        ImGui::Button(label);
        ImGui::PopStyleColor(1);
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(cv, 0.0, 0.0, alpha));
        snprintf(label, nstr, "%.3f", m_max_val);
        ImGui::Button(label);
        ImGui::PopStyleColor(1);

        ImGui::End();
    }

  private:
    std::string m_title;
    double m_min_val;
    double m_max_val;
};

// -----------------------------------------------------------------------------

class EventHandlerWrapper : public vsg::Inherit<vsg::Visitor, EventHandlerWrapper> {
  public:
    EventHandlerWrapper(std::shared_ptr<ChEventHandlerVSG> component, ChVisualSystemVSG* app)
        : m_component(component), m_app(app) {}

    void apply(vsg::KeyPressEvent& keyPress) override { m_component->process(keyPress); }

  private:
    std::shared_ptr<ChEventHandlerVSG> m_component;
    ChVisualSystemVSG* m_app;
};

class ChBaseEventHandlerVSG : public ChEventHandlerVSG {
  public:
    ChBaseEventHandlerVSG(ChVisualSystemVSG* app) : m_app(app) {}

    virtual void process(vsg::KeyPressEvent& keyPress) override {
        if (keyPress.keyBase == 'm' || keyPress.keyModified == 'm') {
            m_app->ToggleGuiVisibility();
        }
        if (keyPress.keyBase == 'n' || keyPress.keyModified == 'n') {
            m_app->GetGuiComponent(m_app->m_camera_gui)->ToggleVisibility();
        }
        if (keyPress.keyBase == vsg::KEY_Escape || keyPress.keyModified == 65307) {
            m_app->Quit();
        }
    }

    ChVisualSystemVSG* m_app;
};

// -----------------------------------------------------------------------------

// Utility visitor class for accessing the vec3 data in the N-th vertex buffer of an object.
// Note: since VSG v.1.0.8 VertexIndexDraw is used instead of BindVertexBuffers!
template <int N>
class FindVec3BufferData : public vsg::Visitor {
  public:
    FindVec3BufferData() : m_buffer(nullptr) {}
    void apply(vsg::Object& object) override { object.traverse(*this); }
    void apply(vsg::BindVertexBuffers& bvd) override {
        if (bvd.arrays.empty())
            return;
        bvd.arrays[N]->data->accept(*this);
    }
    void apply(vsg::VertexIndexDraw& vid) override {
        if (vid.arrays.empty())
            return;
        vid.arrays[N]->data->accept(*this);
    }
    void apply(vsg::vec3Array& vertices) override {
        if (!m_buffer)
            m_buffer = &vertices;
    }
    vsg::ref_ptr<vsg::vec3Array> getBufferData() {
        vsg::ref_ptr<vsg::vec3Array> data;
        data = const_cast<vsg::vec3Array*>(m_buffer);
        return data;
    }
    vsg::vec3Array* m_buffer;
};

// Utility visitor class for accessing the vec4 data in the N-th vertex buffer of an object.
// Note: since VSG v.1.0.8 VertexIndexDraw is used instead of BindVertexBuffers!
template <int N>
class FindVec4BufferData : public vsg::Visitor {
  public:
    FindVec4BufferData() : m_buffer(nullptr) {}
    void apply(vsg::Object& object) override { object.traverse(*this); }
    void apply(vsg::BindVertexBuffers& bvd) override {
        if (bvd.arrays.empty())
            return;
        bvd.arrays[N]->data->accept(*this);
    }
    void apply(vsg::VertexIndexDraw& vid) override {
        if (vid.arrays.empty())
            return;
        vid.arrays[N]->data->accept(*this);
    }
    void apply(vsg::vec4Array& vertices) override {
        if (!m_buffer)
            m_buffer = &vertices;
    }
    vsg::ref_ptr<vsg::vec4Array> getBufferData() {
        vsg::ref_ptr<vsg::vec4Array> data;
        data = const_cast<vsg::vec4Array*>(m_buffer);
        return data;
    }
    vsg::vec4Array* m_buffer;
};

// -----------------------------------------------------------------------------

struct Merge : public vsg::Inherit<vsg::Operation, Merge> {
    Merge(const vsg::Path& in_path,
          vsg::observer_ptr<vsg::Viewer> in_viewer,
          vsg::ref_ptr<vsg::Group> in_attachmentPoint,
          vsg::ref_ptr<vsg::Node> in_node,
          const vsg::CompileResult& in_compileResult)
        : path(in_path),
          viewer(in_viewer),
          attachmentPoint(in_attachmentPoint),
          node(in_node),
          compileResult(in_compileResult) {}

    vsg::Path path;
    vsg::observer_ptr<vsg::Viewer> viewer;
    vsg::ref_ptr<vsg::Group> attachmentPoint;
    vsg::ref_ptr<vsg::Node> node;
    vsg::CompileResult compileResult;

    void run() override {
        // std::cout << "Merge::run() path = " << path << ", " << attachmentPoint << ", " << node << std::endl;

        vsg::ref_ptr<vsg::Viewer> ref_viewer = viewer;
        if (ref_viewer) {
            updateViewer(*ref_viewer, compileResult);
        }

        attachmentPoint->addChild(node);
    }
};

struct LoadOperation : public vsg::Inherit<vsg::Operation, LoadOperation> {
    LoadOperation(vsg::ref_ptr<vsg::Viewer> in_viewer,
                  vsg::ref_ptr<vsg::Group> in_attachmentPoint,
                  const vsg::Path& in_filename,
                  vsg::ref_ptr<vsg::Options> in_options)
        : viewer(in_viewer), attachmentPoint(in_attachmentPoint), filename(in_filename), options(in_options) {}

    vsg::observer_ptr<vsg::Viewer> viewer;
    vsg::ref_ptr<vsg::Group> attachmentPoint;
    vsg::Path filename;
    vsg::ref_ptr<vsg::Options> options;

    void run() override {
        vsg::ref_ptr<vsg::Viewer> ref_viewer = viewer;
        if (auto node = vsg::read_cast<vsg::Node>(filename, options)) {
            auto result = ref_viewer->compileManager->compile(node);
            if (result)
                ref_viewer->addUpdateOperation(Merge::create(filename, viewer, attachmentPoint, node, result));
        }
    }
};

// -----------------------------------------------------------------------------

ChVisualSystemVSG::ChVisualSystemVSG(int num_divs)
    : m_show_logo(true),
      m_logo_pos({10, 10}),
      m_logo_height(64),
      m_yup(false),
      m_useSkybox(false),
      m_capture_image(false),
      m_wireframe(false),
      m_show_gui(true),
      m_show_base_gui(true),
      m_camera_trackball(true),
      m_cog_frame_scale(1),
      m_show_cog_frames(false),
      m_joint_frame_scale(1),
      m_show_joint_frames(false),
      m_frame_number(0),
      m_start_time(0),
      m_time_total(0),
      m_old_time(0),
      m_current_time(0),
      m_fps(0) {
    m_windowTitle = string("Window Title");
    m_clearColor = ChColor(0, 0, 0);
    m_skyboxPath = string("vsg/textures/chrono_skybox.ktx2");
    m_cameraUpVector = vsg::dvec3(0, 0, 1);

    m_logo_filename = GetChronoDataFile("logo_chronoengine_alpha.png");

    // creation here allows to set entries before initialize
    m_bodyScene = vsg::Group::create();
    m_cogFrameScene = vsg::Switch::create();
    m_jointFrameScene = vsg::Switch::create();
    m_pointpointScene = vsg::Group::create();
    m_particleScene = vsg::Group::create();
    m_decoScene = vsg::Group::create();
    m_deformableScene = vsg::Group::create();

    // set up defaults and read command line arguments to override them
    m_options = vsg::Options::create();
    m_options->paths = vsg::getEnvPaths("VSG_FILE_PATH");
    m_options->paths.push_back(GetChronoDataPath());

    // add vsgXchange's support for reading and writing 3rd party file formats, mandatory for chrono_vsg!
    m_options->add(vsgXchange::all::create());
    m_options->sharedObjects = vsg::SharedObjects::create();
    m_shapeBuilder = ShapeBuilder::create(m_options, num_divs);
    m_vsgBuilder = vsg::Builder::create();
    m_vsgBuilder->options = m_options;

    // make some default settings
    SetWindowTitle("");
    SetWindowSize(ChVector2i(800, 600));
    SetWindowPosition(ChVector2i(50, 50));
    SetUseSkyBox(true);
    SetCameraAngleDeg(40);
    SetLightIntensity(1.0);
    SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
#ifdef __APPLE__
    SetGuiFontSize(20.0);
#else
    SetGuiFontSize(13.0);
#endif
}

ChVisualSystemVSG::~ChVisualSystemVSG() {}

void ChVisualSystemVSG::SetOutputScreen(int screenNum) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::SetOutputScreen must be used before initialization!" << std::endl;
        return;
    }
    int maxNum = vsg::Device::maxNumDevices();
    std::cout << "Screens found: " << maxNum << std::endl;
    if (screenNum >= 0 && screenNum < maxNum) {
        m_screen_num = screenNum;
    } else {
        std::cerr << "Screen #" << screenNum << " cannot be used on this computer!" << std::endl;
        throw std::runtime_error("Screen #" + std::to_string(screenNum) + " cannot be used on this computer!");
    }
}

void ChVisualSystemVSG::SetFullscreen(bool yesno) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::SetFullscreen must be used before initialization!" << std::endl;
        return;
    }
    m_use_fullscreen = yesno;
}

size_t ChVisualSystemVSG::AddGuiComponent(std::shared_ptr<ChGuiComponentVSG> gc) {
    m_gui.push_back(gc);
    return m_gui.size() - 1;
}

size_t ChVisualSystemVSG::AddGuiColorbar(const std::string& title, double min_val, double max_val) {
    m_gui.push_back(chrono_types::make_shared<ChColorbarGuiComponentVSG>(title, min_val, max_val));
    return m_gui.size() - 1;
}

std::shared_ptr<ChGuiComponentVSG> ChVisualSystemVSG::GetGuiComponent(size_t id) {
    return m_gui.at(id);
}

void ChVisualSystemVSG::SetBaseGuiVisibility(bool show_gui) {
    m_show_base_gui = show_gui;
    if (m_initialized)
        m_base_gui->SetVisibility(m_show_base_gui);
}

void ChVisualSystemVSG::ToggleBaseGuiVisibility() {
    m_show_base_gui = !m_show_base_gui;
    if (m_initialized)
        m_base_gui->SetVisibility(m_show_base_gui);
}

void ChVisualSystemVSG::AddEventHandler(std::shared_ptr<ChEventHandlerVSG> eh) {
    m_evhandler.push_back(eh);
}

void ChVisualSystemVSG::Quit() {
    m_viewer->close();
}

void ChVisualSystemVSG::SetGuiFontSize(float theSize) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::SetGuiFontSize must be used before initialization!" << std::endl;
        return;
    }
    m_guiFontSize = theSize;
}

void ChVisualSystemVSG::SetWindowSize(const ChVector2i& size) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::SetGuiFontSize must be used before initialization!" << std::endl;
        return;
    }
    m_windowWidth = size[0];
    m_windowHeight = size[1];
}

void ChVisualSystemVSG::SetWindowSize(int width, int height) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::SetWindowSize must be used before initialization!" << std::endl;
        return;
    }
    m_windowWidth = width;
    m_windowHeight = height;
}

void ChVisualSystemVSG::SetWindowPosition(const ChVector2i& pos) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::SetWindowPosition must be used before initialization!" << std::endl;
        return;
    }
    m_windowX = pos[0];
    m_windowY = pos[1];
}

void ChVisualSystemVSG::SetWindowPosition(int from_left, int from_top) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::SetWindowPosition must be used before initialization!" << std::endl;
        return;
    }
    m_windowX = from_left;
    m_windowY = from_top;
}

void ChVisualSystemVSG::SetWindowTitle(const std::string& title) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::SetWindowTitle must be used before initialization!" << std::endl;
        return;
    }
    m_windowTitle = title;
}

void ChVisualSystemVSG::SetClearColor(const ChColor& color) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::SetClearColor must be used before initialization!" << std::endl;
        return;
    }
    m_clearColor = color;
}

void ChVisualSystemVSG::SetUseSkyBox(bool yesno) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::SetUseSkyBox must be used before initialization!" << std::endl;
        return;
    }
    m_useSkybox = yesno;
}

int ChVisualSystemVSG::AddCamera(const ChVector3d& pos, ChVector3d targ) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::AddCamera must be used before initialization!" << std::endl;
        return 1;
    }

    ChVector3d test = pos - targ;
    if (test.Length() == 0.0) {
        std::cerr << "Function ChVisualSystemVSG::AddCamera Camera Pos and Target cannot be identical!" << std::endl;
        std::cerr << "  pos    = { " << pos.x() << " ; " << pos.y() << " ; " << pos.z() << " }" << std::endl;
        std::cerr << "  target = { " << targ.x() << " ; " << targ.y() << " ; " << targ.z() << " }" << std::endl;
        throw std::runtime_error("Function ChVisualSystemVSG::AddCamera Camera Pos and Target cannot be identical!");
    }
    if (m_yup) {
        if (pos.x() == 0.0 && pos.z() == 0.0) {
            std::cout << "Function ChVisualSystemVSG::AddCamera Line of sight is parallel to upvector! -> Corrected!!"
                      << std::endl;
            m_vsg_cameraEye = vsg::dvec3(pos.x() + 1.0, pos.y(), pos.z() + 1.0);
        } else {
            m_vsg_cameraEye = vsg::dvec3(pos.x(), pos.y(), pos.z());
        }
    } else {
        if (pos.x() == 0.0 && pos.y() == 0.0) {
            std::cout << "Function ChVisualSystemVSG::AddCamera Line of sight is parallel to upvector! -> Corrected!!"
                      << std::endl;
            m_vsg_cameraEye = vsg::dvec3(pos.x() + 1.0, pos.y() + 1.0, pos.z());
        } else {
            m_vsg_cameraEye = vsg::dvec3(pos.x(), pos.y(), pos.z());
        }
    }
    m_vsg_cameraTarget = vsg::dvec3(targ.x(), targ.y(), targ.z());

    return 0;
}

void ChVisualSystemVSG::SetCameraPosition(int id, const ChVector3d& pos) {
    m_lookAt->eye = vsg::dvec3(pos.x(), pos.y(), pos.z());
}

void ChVisualSystemVSG::SetCameraTarget(int id, const ChVector3d& target) {
    m_lookAt->center = vsg::dvec3(target.x(), target.y(), target.z());
}

void ChVisualSystemVSG::SetCameraPosition(const ChVector3d& pos) {
    m_lookAt->eye = vsg::dvec3(pos.x(), pos.y(), pos.z());
}

void ChVisualSystemVSG::SetCameraTarget(const ChVector3d& target) {
    m_lookAt->center = vsg::dvec3(target.x(), target.y(), target.z());
}

ChVector3d ChVisualSystemVSG::GetCameraPosition() const {
    const auto& p = m_lookAt->eye;
    return ChVector3d(p.x, p.y, p.z);
}

ChVector3d ChVisualSystemVSG::GetCameraTarget() const {
    const auto& p = m_lookAt->center;
    return ChVector3d(p.x, p.y, p.z);
}

void ChVisualSystemVSG::SetCameraVertical(CameraVerticalDir upDir) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::SetCameraVertical must be used before initialization!" << std::endl;
        return;
    }
    switch (upDir) {
        case CameraVerticalDir::Y:
            m_cameraUpVector = vsg::dvec3(0, 1, 0);
            m_yup = true;
            break;
        case CameraVerticalDir::Z:
            m_cameraUpVector = vsg::dvec3(0, 0, 1);
            m_yup = false;
            break;
    }
}

void ChVisualSystemVSG::SetLightIntensity(float intensity) {
    m_lightIntensity = ChClamp(intensity, 0.0f, 1.0f);
}

void ChVisualSystemVSG::SetLightDirection(double azimuth, double elevation) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::SetLightDirection must be used before initialization!" << std::endl;
        return;
    }
    m_azimuth = ChClamp(azimuth, -CH_PI, CH_PI);
    m_elevation = ChClamp(elevation, 0.0, CH_PI_2);
}

void ChVisualSystemVSG::Initialize() {
    if (m_initialized)
        return;

    auto builder = vsg::Builder::create();
    builder->options = m_options;

    auto windowTraits = vsg::WindowTraits::create();
    windowTraits->windowTitle = m_windowTitle;
    windowTraits->width = m_windowWidth;
    windowTraits->height = m_windowHeight;
    windowTraits->x = m_windowX;
    windowTraits->y = m_windowY;
    windowTraits->debugLayer = false;
    windowTraits->deviceExtensionNames = {VK_KHR_MULTIVIEW_EXTENSION_NAME, VK_KHR_MAINTENANCE2_EXTENSION_NAME,
                                          VK_KHR_CREATE_RENDERPASS_2_EXTENSION_NAME,
                                          VK_KHR_DEPTH_STENCIL_RESOLVE_EXTENSION_NAME};
    windowTraits->swapchainPreferences.imageUsage =
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_IMMEDIATE_KHR;
    windowTraits->depthImageUsage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    windowTraits->fullscreen = m_use_fullscreen;
    windowTraits->screenNum = m_screen_num;

    m_scene = vsg::Group::create();

    double radius = 50.0;
    vsg::dbox bound;

    if (m_useSkybox) {
        vsg::Path fileName(m_skyboxPath);
        auto skyPtr = createSkybox(fileName, m_options, m_yup);
        if (skyPtr)
            m_scene->addChild(skyPtr);
        else
            m_useSkybox = false;
    }

    auto ambientLight = vsg::AmbientLight::create();
    ambientLight->name = "ambient";
    ambientLight->color.set(1.0f, 1.0f, 1.0f);
    ambientLight->intensity = 0.2f;

    auto directionalLight = vsg::DirectionalLight::create();
    directionalLight->name = "sun light";
    directionalLight->color.set(1.0f, 1.0f, 1.0f);
    directionalLight->intensity = m_lightIntensity;
    if (m_use_shadows) {
        uint32_t numShadowsPerLight = 10;
        auto shadowSettings = vsg::HardShadows::create(numShadowsPerLight);
        directionalLight->shadowSettings = shadowSettings;
        directionalLight->intensity *= 0.8f;  // try to avoid saturation due to additional lights
    }

    double se = std::sin(m_elevation);
    double ce = std::cos(m_elevation);
    double sa = std::sin(m_azimuth);
    double ca = std::cos(m_azimuth);
    if (m_yup)
        directionalLight->direction.set(-ce * ca, -se, -ce * sa);
    else
        directionalLight->direction.set(-ce * ca, -ce * sa, -se);

    /* Head Light, moves with camera orientation
    auto absoluteTransform = vsg::AbsoluteTransform::create();
    absoluteTransform->addChild(ambientLight);
    absoluteTransform->addChild(directionalLight);
    m_scene->addChild(absoluteTransform);
     */
    // Directional (Sun) Light, moves with object orientation
    m_scene->addChild(ambientLight);
    m_scene->addChild(directionalLight);
    if (m_use_shadows) {
        // helper light to improve quality of the dark side
        auto overheadLight = vsg::DirectionalLight::create();
        overheadLight->name = "head light";
        overheadLight->color.set(1.0f, 1.0f, 1.0f);
        overheadLight->intensity = 0.2f;
        if (m_yup)
            overheadLight->direction.set(-ce * ca, -se, -ce * sa);
        else
            overheadLight->direction.set(-ce * ca, -ce * sa, -se);
        auto absoluteTransform = vsg::AbsoluteTransform::create();
        absoluteTransform->addChild(overheadLight);
        m_scene->addChild(absoluteTransform);
    }
    m_scene->addChild(m_bodyScene);
    m_scene->addChild(m_cogFrameScene);
    m_scene->addChild(m_jointFrameScene);
    m_scene->addChild(m_pointpointScene);
    m_scene->addChild(m_particleScene);
    m_scene->addChild(m_decoScene);
    m_scene->addChild(m_deformableScene);

    BindAll();

    // create the viewer and assign window(s) to it
    m_viewer = vsg::Viewer::create();

    m_window = vsg::Window::create(windowTraits);
    if (!m_window) {
        std::cout << "Could not create window." << std::endl;
        return;
    }

    ////auto& limits = m_window->getOrCreatePhysicalDevice()->getProperties().limits;  // VkPhysicalDeviceLimits
    const auto& prop = m_window->getOrCreatePhysicalDevice()->getProperties();

    if (m_verbose) {
        std::cout << "----------------------------------------------------" << std::endl;
        std::cout << "* Chrono::VSG Vulkan Scene Graph 3D-Visualization" << std::endl;
        std::cout << "* GPU Name: " << prop.deviceName << std::endl;
        switch (prop.deviceType) {
            default:
            case VK_PHYSICAL_DEVICE_TYPE_OTHER:
                std::cout << "* GPU Type: VK_PHYSICAL_DEVICE_TYPE_OTHER" << std::endl;
                break;
            case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU:
                std::cout << "* GPU Type: VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU" << std::endl;
                break;
            case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU:
                std::cout << "* GPU Type: VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU" << std::endl;
                break;
            case VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU:
                std::cout << "* GPU Type: VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU" << std::endl;
                break;
            case VK_PHYSICAL_DEVICE_TYPE_CPU:
                std::cout << "* GPU Type: VK_PHYSICAL_DEVICE_TYPE_CPU" << std::endl;
                break;
        }
        std::cout << "* Vulkan Version: " << VK_VERSION_MAJOR(VK_HEADER_VERSION_COMPLETE) << "."
                  << VK_VERSION_MINOR(VK_HEADER_VERSION_COMPLETE) << "."
                  << VK_API_VERSION_PATCH(VK_HEADER_VERSION_COMPLETE) << std::endl;
        std::cout << "* Vulkan Scene Graph Version: " << VSG_VERSION_STRING << std::endl;
        std::cout << "* Graphic Output Possible on: " << vsg::Device::maxNumDevices() << " Screens." << std::endl;
        std::cout << "----------------------------------------------------" << std::endl;
    }

    m_window->clearColor() = VkClearColorValue{{m_clearColor.R, m_clearColor.G, m_clearColor.B, 1}};
    m_viewer->addWindow(m_window);

    // set up the camera
    m_lookAt = vsg::LookAt::create(m_vsg_cameraEye, m_vsg_cameraTarget, m_cameraUpVector);

    double nearFarRatio = 0.001;
    auto perspective = vsg::Perspective::create(
        m_cameraAngleDeg,
        static_cast<double>(m_window->extent2D().width) / static_cast<double>(m_window->extent2D().height),
        nearFarRatio * radius, radius * 10.0);

    m_vsg_camera = vsg::Camera::create(perspective, m_lookAt, vsg::ViewportState::create(m_window->extent2D()));

    // default sets automatic directional light
    // auto renderGraph = vsg::RenderGraph::create(m_window, m_view);
    // switches off automatic directional light setting

    auto renderGraph =
        vsg::createRenderGraphForView(m_window, m_vsg_camera, m_scene, VK_SUBPASS_CONTENTS_INLINE, false);
    auto commandGraph = vsg::CommandGraph::create(m_window, renderGraph);

    // initialize ImGui
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = "../data/vsg/imgui.ini";

#ifdef __APPLE__
    // application runs on retina display by default (window 800*600 generates a viewport 1600*1200)
    // there can be reasons to switch to standard resolution
    // 1) there is no retina display
    // 2) standard resolution is demanded by MacOS (in our Irrlicht/VSG examples), set in the app bundle
    // in this case the desired font size is too big. We take the standard font instead.
    if (m_window->traits()->width != m_window->extent2D().width) {
#endif

        auto foundFontFile = vsg::findFile("vsg/fonts/Ubuntu_Mono/UbuntuMono-Regular.ttf", m_options);
        if (foundFontFile) {
            // convert native filename to UTF8 string that is compatible with ImuGUi.
            std::string c_fontFile = foundFontFile.string();

            // read the font via ImGui, which will then be current when vsgImGui::RenderImGui initializes the rest of
            // ImGui/Vulkan below
            ImGuiIO& io = ImGui::GetIO();
            auto imguiFont = io.Fonts->AddFontFromFileTTF(c_fontFile.c_str(), m_guiFontSize);
            if (!imguiFont) {
                std::cout << "Failed to load font: " << c_fontFile << std::endl;
                return;
            }
        }
#ifdef __APPLE__
    } else {
        m_logo_height /= 2.0f;
        // ignore loadable ttf font
        std::cout << "App runs with standard resolution on the Mac. Font size setting ignored." << std::endl;
    }
#endif

    auto renderImGui = vsgImGui::RenderImGui::create(m_window, ChMainGuiVSG::create(this, m_options, m_logo_height));
    renderGraph->addChild(renderImGui);

    m_base_gui = chrono_types::make_shared<ChBaseGuiComponentVSG>(this);
    m_base_gui->SetVisibility(m_show_base_gui);
    AddGuiComponent(m_base_gui);

    // Add the camera info GUI component (initially invisible)
    m_camera_gui = AddGuiComponent(chrono_types::make_shared<ChCameraGuiComponentVSG>(this));

    // ImGui events shall have priority to other events
    m_viewer->addEventHandler(vsgImGui::SendEventsToImGui::create());

    // Add the base keyboard event handler
    auto base_kbhandler = chrono_types::make_shared<ChBaseEventHandlerVSG>(this);
    auto base_kbhandler_wrapper = EventHandlerWrapper::create(base_kbhandler, this);

    m_viewer->addEventHandler(base_kbhandler_wrapper);

    // Add all user-specified event handlers
    for (const auto& eh : m_evhandler) {
        auto evhandler_wrapper = EventHandlerWrapper::create(eh, this);
        m_viewer->addEventHandler(evhandler_wrapper);
    }

    // Add event handler for window close events
    m_viewer->addEventHandler(vsg::CloseHandler::create(m_viewer));

    // Add event handler for mouse camera view manipulation
    if (m_camera_trackball)
        m_viewer->addEventHandler(vsg::Trackball::create(m_vsg_camera));

    m_viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});

    // Assign a CompileTraversal to the Builders that will compile for all the views assigned to the viewer.
    // Must be done after Viewer.assignRecordAndSubmitTasksAndPresentations()
    auto compileTraversal = vsg::CompileTraversal::create(*m_viewer);
    m_shapeBuilder->assignCompileTraversal(compileTraversal);
    m_vsgBuilder->assignCompileTraversal(compileTraversal);
    vsg::ref_ptr<vsg::ResourceHints> resourceHints;
    if (!resourceHints) {
        // To help reduce the number of vsg::DescriptorPool that need to be allocated we'll provide a minimum
        // requirement via ResourceHints.
        resourceHints = vsg::ResourceHints::create();
        resourceHints->numDescriptorSets = 256;
        resourceHints->descriptorPoolSizes.push_back(
            VkDescriptorPoolSize{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 256});
    }

    m_viewer->compile(resourceHints);

    // Prepare reading 3d files
    m_loadThreads = vsg::OperationThreads::create(m_numThreads, m_viewer->status);

    m_initialized = true;
}

bool ChVisualSystemVSG::Run() {
    return m_viewer->active();
}

void ChVisualSystemVSG::Render() {
    if (m_write_images && m_frame_number > 0) {
        // Zero-pad frame numbers in file names for postprocessing
        std::ostringstream filename;
        filename << m_image_dir << "/img_" << std::setw(4) << std::setfill('0') << m_frame_number << ".png";
        WriteImageToFile(filename.str());
    }

    if (m_frame_number == 0)
        m_start_time = double(clock()) / double(CLOCKS_PER_SEC);

    m_timer_render.reset();
    m_timer_render.start();

    UpdateFromMBS();

    if (!m_viewer->advanceToNextFrame()) {
        return;
    }

    // Let the viewer handle any events
    m_viewer->handleEvents();

    m_viewer->update();

    // Dynamic data transfer CPU->GPU for point clouds
    auto hide_pos = m_lookAt->eye - (m_lookAt->center - m_lookAt->eye) * 0.1;

    for (const auto& cloud : m_clouds) {
        if (cloud.dynamic_positions) {
            unsigned int k = 0;
            for (auto& p : *cloud.positions) {
                if (cloud.pcloud->IsVisible(k))
                    p = vsg::vec3CH(cloud.pcloud->Particle(k).GetPos());
                else
                    p = hide_pos;//vsg::vec3(0, 0, 0);
                k++;
            }
            cloud.positions->dirty();
        }
        if (cloud.dynamic_colors) {
            unsigned int k = 0;
            for (auto& c : *cloud.colors)
                c = vsg::vec4CH(cloud.pcloud->GetVisualColor(k++));
            cloud.colors->dirty();
        }
    }

    // Dynamic data transfer CPU->GPU for deformable meshes
    for (auto& def_mesh : m_def_meshes) {
        if (def_mesh.dynamic_vertices) {
            const auto& new_vertices =
                def_mesh.mesh_soup ? def_mesh.trimesh->getFaceVertices() : def_mesh.trimesh->GetCoordsVertices();
            assert(def_mesh.vertices->size() == new_vertices.size());
            size_t k = 0;
            for (auto& v : *def_mesh.vertices)
                v = vsg::vec3CH(new_vertices[k++]);
            def_mesh.vertices->dirty();
        }

        if (def_mesh.dynamic_normals) {
            const auto& new_normals =
                def_mesh.mesh_soup ? def_mesh.trimesh->getFaceNormals() : def_mesh.trimesh->getAverageNormals();
            assert(def_mesh.normals->size() == new_normals.size());
            size_t k = 0;
            for (auto& n : *def_mesh.normals)
                n = vsg::vec3CH(new_normals[k++]);
            def_mesh.normals->dirty();
        }

        if (def_mesh.dynamic_colors) {
            const auto& new_colors =
                def_mesh.mesh_soup ? def_mesh.trimesh->getFaceColors() : def_mesh.trimesh->GetCoordsColors();
            assert(def_mesh.colors->size() == new_colors.size());
            size_t k = 0;
            for (auto& c : *def_mesh.colors)
                c = vsg::vec4CH(new_colors[k++]);
            def_mesh.colors->dirty();
        }
    }

    m_viewer->recordAndSubmit();

    if (m_capture_image) {
        // exportScreenshot(m_window, m_options, m_imageFilename);
        exportScreenImage();
        m_capture_image = false;
    }

    m_viewer->present();
    m_frame_number++;

    m_timer_render.stop();
    m_time_total = .5 * m_timer_render() + .5 * m_time_total;
    m_current_time = m_time_total;
    m_current_time = m_current_time * 0.5 + m_old_time * 0.5;
    m_old_time = m_current_time;
    m_fps = 1.0 / m_current_time;
}

void ChVisualSystemVSG::RenderCOGFrames(double axis_length) {
    m_cog_frame_scale = axis_length;
    m_show_cog_frames = true;

    if (m_initialized) {
        for (auto& child : m_cogFrameScene->children)
            child.mask = m_show_cog_frames;
    }
}

void ChVisualSystemVSG::SetCOGFrameScale(double axis_length) {
    m_cog_frame_scale = axis_length;
}

void ChVisualSystemVSG::ToggleCOGFrameVisibility() {
    m_show_cog_frames = !m_show_cog_frames;

    if (m_initialized) {
        for (auto& child : m_cogFrameScene->children)
            child.mask = m_show_cog_frames;
    }
}

void ChVisualSystemVSG::RenderJointFrames(double axis_length) {
    m_joint_frame_scale = axis_length;
    m_show_joint_frames = true;

    if (m_initialized) {
        for (auto& child : m_jointFrameScene->children)
            child.mask = m_show_joint_frames;
    }
}

void ChVisualSystemVSG::SetJointFrameScale(double axis_length) {
    m_joint_frame_scale = axis_length;
}

void ChVisualSystemVSG::ToggleJointFrameVisibility() {
    m_show_joint_frames = !m_show_joint_frames;

    if (m_initialized) {
        for (auto& child : m_jointFrameScene->children)
            child.mask = m_show_joint_frames;
    }
}

void ChVisualSystemVSG::WriteImageToFile(const string& filename) {
    m_imageFilename = filename;
    m_capture_image = true;
}

// -----------------------------------------------------------------------------

// Utility function for creating a frame with its X axis defined by 2 points.
ChFrame<> PointPointFrame(const ChVector3d& P1, const ChVector3d& P2, double& dist) {
    ChVector3d dir = P2 - P1;
    dist = dir.Length();
    dir.Normalize();
    ChVector3d mx, my, mz;
    dir.GetDirectionAxesAsX(my, mz, mx);
    ChMatrix33<> R_CS;
    R_CS.SetFromDirectionAxes(mx, my, mz);

    return ChFrame<>(0.5 * (P2 + P1), R_CS);
}

// Utility function to populate a VSG group with shape groups (from the given visual model).
// The visual model may or may not be associated with a Chrono physics item.
void ChVisualSystemVSG::PopulateGroup(vsg::ref_ptr<vsg::Group> group,
                                      std::shared_ptr<ChVisualModel> model,
                                      std::shared_ptr<ChPhysicsItem> phitem) {
    for (const auto& shape_instance : model->GetShapeInstances()) {
        const auto& shape = shape_instance.first;
        const auto& X_SM = shape_instance.second;

        if (!shape->IsVisible())
            continue;

        // Material for primitive shapes (assumed at most one defined)
        std::shared_ptr<ChVisualMaterial> material =
            shape->GetMaterials().empty() ? ChVisualMaterial::Default() : shape->GetMaterial(0);

        if (auto box = std::dynamic_pointer_cast<ChVisualShapeBox>(shape)) {
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, box->GetHalflengths());

            // We have boxes and dice. Dice take cubetextures, boxes take 6 identical textures.
            // Use a die if a kd map exists and its name contains "cubetexture". Otherwise, use a box.
            auto grp = !material->GetKdTexture().empty() && material->GetKdTexture().find("cubetexture") != string::npos
                           ? m_shapeBuilder->CreatePbrShape(ShapeBuilder::ShapeType::DIE_SHAPE, material, transform,
                                                            m_wireframe)
                           : m_shapeBuilder->CreatePbrShape(ShapeBuilder::ShapeType::BOX_SHAPE, material, transform,
                                                            m_wireframe);
            group->addChild(grp);
        } else if (auto sphere = std::dynamic_pointer_cast<ChVisualShapeSphere>(shape)) {
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, sphere->GetRadius());
            auto grp =
                m_shapeBuilder->CreatePbrShape(ShapeBuilder::ShapeType::SPHERE_SHAPE, material, transform, m_wireframe);
            group->addChild(grp);
        } else if (auto ellipsoid = std::dynamic_pointer_cast<ChVisualShapeEllipsoid>(shape)) {
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, ellipsoid->GetSemiaxes());
            auto grp =
                m_shapeBuilder->CreatePbrShape(ShapeBuilder::ShapeType::SPHERE_SHAPE, material, transform, m_wireframe);
            group->addChild(grp);
        } else if (auto cylinder = std::dynamic_pointer_cast<ChVisualShapeCylinder>(shape)) {
            double rad = cylinder->GetRadius();
            double height = cylinder->GetHeight();
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, ChVector3d(rad, rad, height));
            auto grp = m_shapeBuilder->CreatePbrShape(ShapeBuilder::ShapeType::CYLINDER_SHAPE, material, transform,
                                                      m_wireframe);
            group->addChild(grp);
        } else if (auto capsule = std::dynamic_pointer_cast<ChVisualShapeCapsule>(shape)) {
            double rad = capsule->GetRadius();
            double height = capsule->GetHeight();
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, ChVector3d(rad, rad, rad / 2 + height / 4));
            auto grp = m_shapeBuilder->CreatePbrShape(ShapeBuilder::ShapeType::CAPSULE_SHAPE, material, transform,
                                                      m_wireframe);
            group->addChild(grp);
        } else if (auto barrel = std::dynamic_pointer_cast<ChVisualShapeBarrel>(shape)) {
            //// TODO
        } else if (auto cone = std::dynamic_pointer_cast<ChVisualShapeCone>(shape)) {
            double rad = cone->GetRadius();
            double height = cone->GetHeight();
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, ChVector3d(rad, rad, height));
            auto grp =
                m_shapeBuilder->CreatePbrShape(ShapeBuilder::ShapeType::CONE_SHAPE, material, transform, m_wireframe);
            group->addChild(grp);
        } else if (auto trimesh = std::dynamic_pointer_cast<ChVisualShapeTriangleMesh>(shape)) {
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, trimesh->GetScale());
            /*
            auto grp = trimesh->GetNumMaterials() > 0
                           ? m_shapeBuilder->createTrimeshPhongMatShape(trimesh, transform, m_wireframe)
                           : m_shapeBuilder->createTrimeshColShape(trimesh, transform, m_wireframe);
            */
            auto grp = trimesh->GetNumMaterials() > 0
                           ? m_shapeBuilder->CreateTrimeshPbrMatShape(trimesh, transform, m_wireframe)
                           : m_shapeBuilder->CreateTrimeshColShape(trimesh, transform, m_wireframe);
            group->addChild(grp);
        } else if (auto surface = std::dynamic_pointer_cast<ChVisualShapeSurface>(shape)) {
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, 1.0);
            auto grp = m_shapeBuilder->CreatePbrSurfaceShape(surface, material, transform, m_wireframe);
            group->addChild(grp);
        } else if (auto obj = std::dynamic_pointer_cast<ChVisualShapeModelFile>(shape)) {
            const auto& objFilename = obj->GetFilename();
            const auto& scale = obj->GetScale();
            size_t objHashValue = m_stringHash(objFilename);
            auto grp = vsg::Group::create();
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(ChFrame<>(X_SM.GetPos(), X_SM.GetRot() * QuatFromAngleX(-CH_PI_2)), scale);
            grp->addChild(transform);
            // needed, when BindAll() is called after Initialization
            // vsg::observer_ptr<vsg::Viewer> observer_viewer(m_viewer);
            // m_loadThreads->add(LoadOperation::create(observer_viewer, transform, objFilename, m_options));
            map<size_t, vsg::ref_ptr<vsg::Node>>::iterator objIt;
            objIt = m_objCache.find(objHashValue);
            if (objIt == m_objCache.end()) {
                auto node = vsg::read_cast<vsg::Node>(objFilename, m_options);
                if (node) {
                    transform->addChild(node);
                    group->addChild(grp);
                    m_objCache[objHashValue] = node;
                }
            } else {
                transform->addChild(m_objCache[objHashValue]);
                group->addChild(grp);
            }
        } else if (auto line = std::dynamic_pointer_cast<ChVisualShapeLine>(shape)) {
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, 1.0);
            group->addChild(m_shapeBuilder->CreateLineShape(shape_instance, material, transform, line));
        } else if (auto path = std::dynamic_pointer_cast<ChVisualShapePath>(shape)) {
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, 1.0);
            group->addChild(m_shapeBuilder->CreatePathShape(shape_instance, material, transform, path));
        }

    }  // end loop over visual shapes
}

void ChVisualSystemVSG::BindBody(const std::shared_ptr<ChBody>& body) {
    const auto& vis_model = body->GetVisualModel();
    const auto& vis_frame = body->GetVisualModelFrame();

    if (!vis_model)
        return;

    // Important for update: keep the correct scenegraph hierarchy
    //     modelGroup->model_transform->shapes_group

    // Create a group to hold this visual model
    auto modelGroup = vsg::Group::create();

    // Create a group to hold the shapes with their subtransforms
    auto shapes_group = vsg::Group::create();

    // Populate the group with shapes in the visual model
    PopulateGroup(shapes_group, vis_model, body);

    // Attach a transform to the group and initialize it with the body current position
    auto model_transform = vsg::MatrixTransform::create();
    model_transform->matrix = vsg::dmat4CH(vis_frame, 1.0);
    model_transform->subgraphRequiresLocalFrustum = false;
    if (m_options->sharedObjects) {
        m_options->sharedObjects->share(modelGroup);
        m_options->sharedObjects->share(model_transform);
    }
    model_transform->addChild(shapes_group);
    modelGroup->addChild(model_transform);

    // Set group properties
    modelGroup->setValue("Body", body);
    modelGroup->setValue("Transform", model_transform);

    // Add the group to the global holder
    m_bodyScene->addChild(modelGroup);
}

void ChVisualSystemVSG::BindDeformableMesh(const std::shared_ptr<ChPhysicsItem>& item) {
    const auto& vis_model = item->GetVisualModel();

    if (!vis_model)
        return;

    for (auto& shape_instance : vis_model->GetShapeInstances()) {
        auto& shape = shape_instance.first;

        //// RADU TODO: process glyphs
        ////            for now, only treat the trimeshes in the visual model
        auto trimesh = std::dynamic_pointer_cast<ChVisualShapeTriangleMesh>(shape);
        if (!trimesh)
            continue;

        if (trimesh->GetMesh()->GetNumVertices() == 0)
            continue;

        DeformableMesh def_mesh;
        def_mesh.trimesh = trimesh->GetMesh();

        auto transform = vsg::MatrixTransform::create();
        auto child = (trimesh->GetNumMaterials() > 0)
                         ? m_shapeBuilder->CreateTrimeshPbrMatShape(trimesh, transform, trimesh->IsWireframe())
                         : m_shapeBuilder->CreateTrimeshColShape(trimesh, transform, trimesh->IsWireframe());
        m_deformableScene->addChild(child);

        def_mesh.mesh_soup = true;

        def_mesh.vertices = vsg::visit<FindVec3BufferData<0>>(child).getBufferData();
        assert(def_mesh.vertices->size() == 3 * trimesh->GetMesh()->GetNumTriangles());
        def_mesh.vertices->properties.dataVariance = vsg::DYNAMIC_DATA;
        def_mesh.dynamic_vertices = true;

        if (!trimesh->IsWireframe()) {
            def_mesh.normals = vsg::visit<FindVec3BufferData<1>>(child).getBufferData();
            assert(def_mesh.normals->size() == def_mesh.vertices->size());
            def_mesh.normals->properties.dataVariance = vsg::DYNAMIC_DATA;
            def_mesh.dynamic_normals = true;
        } else {
            def_mesh.dynamic_normals = false;
        }

        if (trimesh->GetNumMaterials() == 0) {
            def_mesh.colors = vsg::visit<FindVec4BufferData<3>>(child).getBufferData();
            assert(def_mesh.colors->size() == def_mesh.vertices->size());
            def_mesh.colors->properties.dataVariance = vsg::DYNAMIC_DATA;
            def_mesh.dynamic_colors = true;
        } else {
            def_mesh.dynamic_colors = false;
        }

        m_def_meshes.push_back(def_mesh);
    }
}

void ChVisualSystemVSG::BindPointPoint(const std::shared_ptr<ChPhysicsItem>& item) {
    const auto& vis_model = item->GetVisualModel();

    if (!vis_model)
        return;

    for (auto& shape_instance : vis_model->GetShapeInstances()) {
        auto& shape = shape_instance.first;
        if (auto segshape = std::dynamic_pointer_cast<ChVisualShapeSegment>(shape)) {
            double length;
            auto X = PointPointFrame(segshape->GetPoint1Abs(), segshape->GetPoint2Abs(), length);
            std::shared_ptr<ChVisualMaterial> material =
                shape->GetMaterials().empty() ? ChVisualMaterial::Default() : shape->GetMaterial(0);

            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X, ChVector3d(0, length, 0));
            m_pointpointScene->addChild(m_shapeBuilder->CreateUnitSegment(shape_instance, material, transform));
        } else if (auto sprshape = std::dynamic_pointer_cast<ChVisualShapeSpring>(shape)) {
            double rad = sprshape->GetRadius();
            double length;
            auto X = PointPointFrame(sprshape->GetPoint1Abs(), sprshape->GetPoint2Abs(), length);
            std::shared_ptr<ChVisualMaterial> material =
                shape->GetMaterials().empty() ? ChVisualMaterial::Default() : shape->GetMaterial(0);

            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X, ChVector3d(rad, length, rad));
            m_pointpointScene->addChild(
                m_shapeBuilder->CreateSpringShape(shape_instance, material, transform, sprshape));
        }
    }
}

void ChVisualSystemVSG::BindParticleCloud(const std::shared_ptr<ChParticleCloud>& pcloud) {
    const auto& vis_model = pcloud->GetVisualModel();
    auto num_particles = pcloud->GetNumParticles();

    if (!vis_model)
        return;

    // Search for an appropriate rendering shape
    typedef ChGeometry::Type ShapeType;
    auto shape = vis_model->GetShape(0);
    ShapeType shape_type = ShapeType::NONE;
    ChVector3d shape_size(0);
    if (auto sph = std::dynamic_pointer_cast<ChVisualShapeSphere>(shape)) {
        shape_type = ShapeType::SPHERE;
        shape_size = ChVector3d(2 * sph->GetRadius());
    } else if (auto ell = std::dynamic_pointer_cast<ChVisualShapeEllipsoid>(shape)) {
        shape_type = ShapeType::ELLIPSOID;
        shape_size = ell->GetAxes();
    } else if (auto box = std::dynamic_pointer_cast<ChVisualShapeBox>(shape)) {
        shape_type = ShapeType::BOX;
        shape_size = box->GetLengths();
    } else if (auto cap = std::dynamic_pointer_cast<ChVisualShapeCapsule>(shape)) {
        double rad = cap->GetRadius();
        double height = cap->GetHeight();
        shape_type = ShapeType::CAPSULE;
        shape_size = ChVector3d(2 * rad, 2 * rad, height);
    } else if (auto cyl = std::dynamic_pointer_cast<ChVisualShapeCylinder>(shape)) {
        double rad = cyl->GetRadius();
        double height = cyl->GetHeight();
        shape_type = ShapeType::CYLINDER;
        shape_size = ChVector3d(2 * rad, 2 * rad, height);
    } else if (auto cone = std::dynamic_pointer_cast<ChVisualShapeCone>(shape)) {
        double rad = cone->GetRadius();
        double height = cone->GetHeight();
        shape_type = ShapeType::CONE;
        shape_size = ChVector3d(2 * rad, 2 * rad, height);
    }

    if (shape_type == ShapeType::NONE)
        return;

    // Search for the base color
    ChColor shape_color;
    if (shape->GetNumMaterials() > 0)
        shape_color = shape->GetMaterial(0)->GetDiffuseColor();
    else
        shape_color = ChVisualMaterial::Default()->GetDiffuseColor();

    // Create an new entry in the set of Chrono::VSG particle clouds
    ParticleCloud cloud;
    cloud.pcloud = pcloud;
    cloud.dynamic_positions = pcloud->IsActive();
    cloud.dynamic_colors = pcloud->UseDynamicColors();

    // Set up geometry and state info for vsgBuilder
    vsg::GeometryInfo geomInfo;
    geomInfo.dx.set((float)shape_size.x(), 0, 0);
    geomInfo.dy.set(0, (float)shape_size.y(), 0);
    geomInfo.dz.set(0, 0, (float)shape_size.z());

    if (cloud.dynamic_colors) {
        cloud.colors = vsg::vec4Array::create(num_particles);
        geomInfo.colors = cloud.colors;
        for (size_t k = 0; k < num_particles; k++)
            cloud.colors->set(k, vsg::vec4(0, 0, 0, 1));
        cloud.colors->properties.dataVariance = vsg::DYNAMIC_DATA;
    } else {
        geomInfo.color.set(shape_color.R, shape_color.G, shape_color.B, 1.0);
    }

    cloud.positions = vsg::vec3Array::create(num_particles);
    geomInfo.positions = cloud.positions;
    for (unsigned int k = 0; k < num_particles; k++)
        cloud.positions->set(k, vsg::vec3CH(pcloud->Particle(k).GetPos()));
    if (cloud.dynamic_positions) {
        cloud.positions->properties.dataVariance = vsg::DYNAMIC_DATA;
    }

    vsg::StateInfo stateInfo;
    stateInfo.wireframe = m_wireframe;
    stateInfo.instance_positions_vec3 = true;

    // Add child node for this cloud
    switch (shape_type) {
        case ShapeType::SPHERE:
        case ShapeType::ELLIPSOID:
            m_particleScene->addChild(m_vsgBuilder->createSphere(geomInfo, stateInfo));
            break;
        case ShapeType::BOX:
            m_particleScene->addChild(m_vsgBuilder->createBox(geomInfo, stateInfo));
            break;
        case ShapeType::CAPSULE:
            m_particleScene->addChild(m_vsgBuilder->createCapsule(geomInfo, stateInfo));
            break;
        case ShapeType::CYLINDER:
            m_particleScene->addChild(m_vsgBuilder->createCylinder(geomInfo, stateInfo));
            break;
        case ShapeType::CONE:
            m_particleScene->addChild(m_vsgBuilder->createCone(geomInfo, stateInfo));
            break;
        default:
            break;
    }

    m_clouds.push_back(cloud);
}

void ChVisualSystemVSG::BindBodyFrame(const std::shared_ptr<ChBody>& body) {
    auto cog_transform = vsg::MatrixTransform::create();
    cog_transform->matrix = vsg::dmat4CH(body->GetFrameCOMToAbs(), m_cog_frame_scale);
    vsg::Mask mask = m_show_cog_frames;
    auto cog_node = m_shapeBuilder->createFrameSymbol(cog_transform, 1.0f);
    cog_node->setValue("Body", body);
    cog_node->setValue("Transform", cog_transform);
    m_cogFrameScene->addChild(mask, cog_node);
}

void ChVisualSystemVSG::BindLinkFrame(const std::shared_ptr<ChLink>& link) {
    vsg::Mask mask = m_show_cog_frames;

    {
        auto joint_transform = vsg::MatrixTransform::create();
        joint_transform->matrix = vsg::dmat4CH(link->GetFrame1Abs(), m_joint_frame_scale);
        auto joint_node = m_shapeBuilder->createFrameSymbol(joint_transform, 0.75f);
        joint_node->setValue("Joint", link);
        joint_node->setValue("Body", 1);
        joint_node->setValue("Transform", joint_transform);
        m_jointFrameScene->addChild(mask, joint_node);
    }

    {
        auto joint_transform = vsg::MatrixTransform::create();
        joint_transform->matrix = vsg::dmat4CH(link->GetFrame2Abs(), m_joint_frame_scale);
        auto joint_node = m_shapeBuilder->createFrameSymbol(joint_transform, 0.5f);
        joint_node->setValue("Joint", link);
        joint_node->setValue("Body", 2);
        joint_node->setValue("Transform", joint_transform);
        m_jointFrameScene->addChild(mask, joint_node);
    }
}

void ChVisualSystemVSG::BindItem(std::shared_ptr<ChPhysicsItem> item) {
    if (auto body = std::dynamic_pointer_cast<ChBody>(item)) {
        BindBodyFrame(body);
        BindBody(body);
        return;
    }

    if (auto link = std::dynamic_pointer_cast<ChLink>(item)) {
        BindLinkFrame(link);
        BindPointPoint(link);
        return;
    }

    if (auto mesh = std::dynamic_pointer_cast<fea::ChMesh>(item)) {
        mesh->UpdateVisualModel();
        BindDeformableMesh(mesh);
        return;
    }

    if (item->GetVisualModel()) {
        BindDeformableMesh(item);
        BindPointPoint(item);
        if (const auto& pcloud = std::dynamic_pointer_cast<ChParticleCloud>(item))
            BindParticleCloud(pcloud);
    }
}

void ChVisualSystemVSG::BindAll() {
    for (auto sys : m_systems) {
        // Bind visual models associated with bodies in the system
        for (const auto& body : sys->GetAssembly().GetBodies()) {
            BindBodyFrame(body);
            BindBody(body);
        }

        // Bind visual models associated with links in the system
        for (const auto& link : sys->GetLinks()) {
            if (auto link1 = std::dynamic_pointer_cast<ChLink>(link)) {
                BindLinkFrame(link1);
                BindPointPoint(link1);
            }
        }

        // Bind visual models associated with FEA meshes
        for (const auto& mesh : sys->GetAssembly().GetMeshes()) {
            mesh->UpdateVisualModel();
            BindDeformableMesh(mesh);
        }

        // Bind visual models associated with other physics items in the system
        for (const auto& item : sys->GetOtherPhysicsItems()) {
            BindDeformableMesh(item);
            BindPointPoint(item);
            if (const auto& pcloud = std::dynamic_pointer_cast<ChParticleCloud>(item))
                BindParticleCloud(pcloud);
        }
    }  // end loop over systems
}

// -----------------------------------------------------------------------------

void ChVisualSystemVSG::UpdateFromMBS() {
    // Update VSG nodes for body COG frame visualization
    if (m_show_cog_frames) {
        for (auto& child : m_cogFrameScene->children) {
            std::shared_ptr<ChBody> body;
            vsg::ref_ptr<vsg::MatrixTransform> transform;
            if (!child.node->getValue("Body", body))
                continue;
            if (!child.node->getValue("Transform", transform))
                continue;

            transform->matrix = vsg::dmat4CH(body->GetFrameCOMToAbs(), m_cog_frame_scale);
        }
    }

    // Update VSG nodes for joint frame visualization
    if (m_show_joint_frames) {
        for (auto& child : m_jointFrameScene->children) {
            std::shared_ptr<ChLink> link;
            vsg::ref_ptr<vsg::MatrixTransform> transform;
            int body;
            if (!child.node->getValue("Joint", link))
                continue;
            if (!child.node->getValue("Transform", transform))
                continue;
            if (!child.node->getValue("Body", body))
                continue;

            if (body == 1)
                transform->matrix = vsg::dmat4CH(link->GetFrame1Abs(), m_joint_frame_scale);
            else
                transform->matrix = vsg::dmat4CH(link->GetFrame2Abs(), m_joint_frame_scale);
        }
    }

    // Update VSG nodes for body visualization
    for (const auto& child : m_bodyScene->children) {
        std::shared_ptr<ChBody> body;
        vsg::ref_ptr<vsg::MatrixTransform> transform;
        if (!child->getValue("Body", body))
            continue;
        if (!child->getValue("Transform", transform))
            continue;
        transform->matrix = vsg::dmat4CH(body->GetVisualModelFrame(), 1.0);
    }

    // Update all VSG nodes with point-point visualization assets
    for (const auto& child : m_pointpointScene->children) {
        ChVisualModel::ShapeInstance shapeInstance;
        vsg::ref_ptr<vsg::MatrixTransform> transform;
        if (!child->getValue("ShapeInstance", shapeInstance))
            continue;
        if (!child->getValue("Transform", transform))
            continue;

        auto& shape = shapeInstance.first;

        if (auto segshape = std::dynamic_pointer_cast<ChVisualShapeSegment>(shape)) {
            double length;
            auto X = PointPointFrame(segshape->GetPoint1Abs(), segshape->GetPoint2Abs(), length);
            transform->matrix = vsg::dmat4CH(X, ChVector3d(0, length, 0));
        } else if (auto sprshape = std::dynamic_pointer_cast<ChVisualShapeSpring>(shape)) {
            double rad = sprshape->GetRadius();
            double length;
            auto X = PointPointFrame(sprshape->GetPoint1Abs(), sprshape->GetPoint2Abs(), length);
            transform->matrix = vsg::dmat4CH(X, ChVector3d(rad, length, rad));
        }
    }
}

void ChVisualSystemVSG::OnSetup(ChSystem* sys) {
    //// RADU TODO
    ////    delete VSG elements associated with physics items no longer present in the system
}

int ChVisualSystemVSG::AddVisualModel(std::shared_ptr<ChVisualModel> model, const ChFrame<>& frame) {
    // Important for update: keep the correct scenegraph hierarchy
    //     model_group->model_transform->shapes_group

    // Create a group to hold this visual model
    auto model_group = vsg::Group::create();

    // Create a group to hold the shapes with their subtransforms
    auto shapes_group = vsg::Group::create();

    // Populate the group with shapes in the visual model
    PopulateGroup(shapes_group, model, nullptr);

    // Attach a transform to the group and initialize it with the provided frame
    auto model_transform = vsg::MatrixTransform::create();
    model_transform->matrix = vsg::dmat4CH(frame, 1.0);
    model_transform->subgraphRequiresLocalFrustum = false;
    if (m_options->sharedObjects) {
        m_options->sharedObjects->share(model_group);
        m_options->sharedObjects->share(model_transform);
    }
    model_transform->addChild(shapes_group);
    model_group->addChild(model_transform);

    // Set group properties
    model_group->setValue("Transform", model_transform);

    // Add the group to the global holder
    m_decoScene->addChild(model_group);

    return m_decoScene->children.size() - 1;
}

int ChVisualSystemVSG::AddVisualModel(std::shared_ptr<ChVisualShape> shape, const ChFrame<>& frame) {
    auto model = chrono_types::make_shared<ChVisualModel>();
    model->AddShape(shape);
    return AddVisualModel(model, frame);
}

void ChVisualSystemVSG::UpdateVisualModel(int id, const ChFrame<>& frame) {
    if (id == -1 || id >= m_decoScene->children.size()) {
        return;
    }

    auto model_group = m_decoScene->children[id];
    vsg::ref_ptr<vsg::MatrixTransform> transform;
    if (!model_group->getValue("Transform", transform))
        return;

    transform->matrix = vsg::dmat4CH(frame, 1.0);
}

// -----------------------------------------------------------------------------

void ChVisualSystemVSG::AddGrid(double x_step, double y_step, int nx, int ny, ChCoordsys<> pos, ChColor col) {
    m_decoScene->addChild(m_shapeBuilder->CreateGrid(x_step, y_step, nx, ny, pos, col));
}

void ChVisualSystemVSG::exportScreenImage() {
    m_write_images = false;

    auto width = m_window->extent2D().width;
    auto height = m_window->extent2D().height;

    auto device = m_window->getDevice();
    auto physicalDevice = m_window->getPhysicalDevice();
    auto swapchain = m_window->getSwapchain();

    // get the colour buffer image of the previous rendered frame as the current frame hasn't been rendered yet.  The 1
    // in window->imageIndex(1) means image from 1 frame ago.
    auto sourceImage = m_window->imageView(m_window->imageIndex(1))->image;

    VkFormat sourceImageFormat = swapchain->getImageFormat();
    VkFormat targetImageFormat = sourceImageFormat;

    //
    // 1) Check to see if Blit is supported.
    //
    VkFormatProperties srcFormatProperties;
    vkGetPhysicalDeviceFormatProperties(*(physicalDevice), sourceImageFormat, &srcFormatProperties);

    VkFormatProperties destFormatProperties;
    vkGetPhysicalDeviceFormatProperties(*(physicalDevice), VK_FORMAT_R8G8B8A8_UNORM, &destFormatProperties);

    bool supportsBlit = ((srcFormatProperties.optimalTilingFeatures & VK_FORMAT_FEATURE_BLIT_SRC_BIT) != 0) &&
                        ((destFormatProperties.linearTilingFeatures & VK_FORMAT_FEATURE_BLIT_DST_BIT) != 0);

    if (supportsBlit) {
        // we can automatically convert the image format when blit, so take advantage of it to ensure RGBA
        targetImageFormat = VK_FORMAT_R8G8B8A8_UNORM;
    }

    // vsg::info("supportsBlit = ", supportsBlit);

    //
    // 2) create image to write to
    //
    auto destinationImage = vsg::Image::create();
    destinationImage->imageType = VK_IMAGE_TYPE_2D;
    destinationImage->format = targetImageFormat;
    destinationImage->extent.width = width;
    destinationImage->extent.height = height;
    destinationImage->extent.depth = 1;
    destinationImage->arrayLayers = 1;
    destinationImage->mipLevels = 1;
    destinationImage->initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    destinationImage->samples = VK_SAMPLE_COUNT_1_BIT;
    destinationImage->tiling = VK_IMAGE_TILING_LINEAR;
    destinationImage->usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT;

    destinationImage->compile(device);

    auto deviceMemory =
        vsg::DeviceMemory::create(device, destinationImage->getMemoryRequirements(device->deviceID),
                                  VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

    destinationImage->bind(deviceMemory, 0);

    //
    // 3) create command buffer and submit to graphics queue
    //
    auto commands = vsg::Commands::create();

    // 3.a) transition destinationImage to transfer destination initialLayout
    auto transitionDestinationImageToDestinationLayoutBarrier = vsg::ImageMemoryBarrier::create(
        0,                                                              // srcAccessMask
        VK_ACCESS_TRANSFER_WRITE_BIT,                                   // dstAccessMask
        VK_IMAGE_LAYOUT_UNDEFINED,                                      // oldLayout
        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,                           // newLayout
        VK_QUEUE_FAMILY_IGNORED,                                        // srcQueueFamilyIndex
        VK_QUEUE_FAMILY_IGNORED,                                        // dstQueueFamilyIndex
        destinationImage,                                               // image
        VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}  // subresourceRange
    );

    // 3.b) transition swapChainImage from present to transfer source initialLayout
    auto transitionSourceImageToTransferSourceLayoutBarrier = vsg::ImageMemoryBarrier::create(
        VK_ACCESS_MEMORY_READ_BIT,                                      // srcAccessMask
        VK_ACCESS_TRANSFER_READ_BIT,                                    // dstAccessMask
        VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,                                // oldLayout
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,                           // newLayout
        VK_QUEUE_FAMILY_IGNORED,                                        // srcQueueFamilyIndex
        VK_QUEUE_FAMILY_IGNORED,                                        // dstQueueFamilyIndex
        sourceImage,                                                    // image
        VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}  // subresourceRange
    );

    auto cmd_transitionForTransferBarrier =
        vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_TRANSFER_BIT,                        // srcStageMask
                                     VK_PIPELINE_STAGE_TRANSFER_BIT,                        // dstStageMask
                                     0,                                                     // dependencyFlags
                                     transitionDestinationImageToDestinationLayoutBarrier,  // barrier
                                     transitionSourceImageToTransferSourceLayoutBarrier     // barrier
        );

    commands->addChild(cmd_transitionForTransferBarrier);

    if (supportsBlit) {
        // 3.c.1) if blit using vkCmdBlitImage
        VkImageBlit region{};
        region.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.srcSubresource.layerCount = 1;
        region.srcOffsets[0] = VkOffset3D{0, 0, 0};
        region.srcOffsets[1] = VkOffset3D{static_cast<int32_t>(width), static_cast<int32_t>(height), 1};
        region.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.dstSubresource.layerCount = 1;
        region.dstOffsets[0] = VkOffset3D{0, 0, 0};
        region.dstOffsets[1] = VkOffset3D{static_cast<int32_t>(width), static_cast<int32_t>(height), 1};

        auto blitImage = vsg::BlitImage::create();
        blitImage->srcImage = sourceImage;
        blitImage->srcImageLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        blitImage->dstImage = destinationImage;
        blitImage->dstImageLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        blitImage->regions.push_back(region);
        blitImage->filter = VK_FILTER_NEAREST;

        commands->addChild(blitImage);
    } else {
        // 3.c.2) else use vkCmdCopyImage

        VkImageCopy region{};
        region.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.srcSubresource.layerCount = 1;
        region.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.dstSubresource.layerCount = 1;
        region.extent.width = width;
        region.extent.height = height;
        region.extent.depth = 1;

        auto copyImage = vsg::CopyImage::create();
        copyImage->srcImage = sourceImage;
        copyImage->srcImageLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        copyImage->dstImage = destinationImage;
        copyImage->dstImageLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        copyImage->regions.push_back(region);

        commands->addChild(copyImage);
    }

    // 3.d) transition destination image from transfer destination layout to general layout to enable mapping to image
    // DeviceMemory
    auto transitionDestinationImageToMemoryReadBarrier = vsg::ImageMemoryBarrier::create(
        VK_ACCESS_TRANSFER_WRITE_BIT,                                   // srcAccessMask
        VK_ACCESS_MEMORY_READ_BIT,                                      // dstAccessMask
        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,                           // oldLayout
        VK_IMAGE_LAYOUT_GENERAL,                                        // newLayout
        VK_QUEUE_FAMILY_IGNORED,                                        // srcQueueFamilyIndex
        VK_QUEUE_FAMILY_IGNORED,                                        // dstQueueFamilyIndex
        destinationImage,                                               // image
        VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}  // subresourceRange
    );

    // 3.e) transition swap chain image back to present
    auto transitionSourceImageBackToPresentBarrier = vsg::ImageMemoryBarrier::create(
        VK_ACCESS_TRANSFER_READ_BIT,                                    // srcAccessMask
        VK_ACCESS_MEMORY_READ_BIT,                                      // dstAccessMask
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,                           // oldLayout
        VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,                                // newLayout
        VK_QUEUE_FAMILY_IGNORED,                                        // srcQueueFamilyIndex
        VK_QUEUE_FAMILY_IGNORED,                                        // dstQueueFamilyIndex
        sourceImage,                                                    // image
        VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}  // subresourceRange
    );

    auto cmd_transitionFromTransferBarrier =
        vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_TRANSFER_BIT,                 // srcStageMask
                                     VK_PIPELINE_STAGE_TRANSFER_BIT,                 // dstStageMask
                                     0,                                              // dependencyFlags
                                     transitionDestinationImageToMemoryReadBarrier,  // barrier
                                     transitionSourceImageBackToPresentBarrier       // barrier
        );

    commands->addChild(cmd_transitionFromTransferBarrier);

    auto fence = vsg::Fence::create(device);
    auto queueFamilyIndex = physicalDevice->getQueueFamily(VK_QUEUE_GRAPHICS_BIT);
    auto commandPool = vsg::CommandPool::create(device, queueFamilyIndex);
    auto queue = device->getQueue(queueFamilyIndex);

    vsg::submitCommandsToQueue(commandPool, fence, 100000000000, queue,
                               [&](vsg::CommandBuffer& commandBuffer) { commands->record(commandBuffer); });

    //
    // 4) map image and copy
    //
    VkImageSubresource subResource{VK_IMAGE_ASPECT_COLOR_BIT, 0, 0};
    VkSubresourceLayout subResourceLayout;
    vkGetImageSubresourceLayout(*device, destinationImage->vk(device->deviceID), &subResource, &subResourceLayout);

    size_t destRowWidth = width * sizeof(vsg::ubvec4);
    vsg::ref_ptr<vsg::Data> imageData;
    if (destRowWidth == subResourceLayout.rowPitch) {
        imageData = vsg::MappedData<vsg::ubvec4Array2D>::create(deviceMemory, subResourceLayout.offset, 0,
                                                                vsg::Data::Properties{targetImageFormat}, width,
                                                                height);  // deviceMemory, offset, flags and dimensions
    } else {
        // Map the buffer memory and assign as a ubyteArray that will automatically unmap itself on destruction.
        // A ubyteArray is used as the graphics buffer memory is not contiguous like vsg::Array2D, so map to a flat
        // buffer first then copy to Array2D.
        auto mappedData = vsg::MappedData<vsg::ubyteArray>::create(deviceMemory, subResourceLayout.offset, 0,
                                                                   vsg::Data::Properties{targetImageFormat},
                                                                   subResourceLayout.rowPitch * height);
        imageData = vsg::ubvec4Array2D::create(width, height, vsg::Data::Properties{targetImageFormat});
        for (uint32_t row = 0; row < height; ++row) {
            std::memcpy(imageData->dataPointer(row * width), mappedData->dataPointer(row * subResourceLayout.rowPitch),
                        destRowWidth);
        }
    }

    if (!vsg::write(imageData, m_imageFilename, m_options)) {
        std::cout << "Failed to write color buffer to " << m_imageFilename << std::endl;
    }
}

}  // namespace vsg3d
}  // namespace chrono
