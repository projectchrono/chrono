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

#include "chrono_vsg/tools/createSkybox.h"
#include "chrono_vsg/tools/createQuad.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChEllipsoidShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChCapsuleShape.h"
#include "chrono/assets/ChBarrelShape.h"
#include "chrono/assets/ChConeShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChSurfaceShape.h"
#include "chrono/assets/ChObjFileShape.h"
#include "chrono/assets/ChLineShape.h"
#include "chrono/assets/ChPathShape.h"
#include "chrono/physics/ChParticleCloud.h"
#include "ChVisualSystemVSG.h"
#include <algorithm>
#include <string>
#include <cstddef>
#include <cctype>

namespace chrono {
namespace vsg3d {

using namespace std;

class GuiComponent {
  public:
    GuiComponent(vsg::ref_ptr<ChVisualSystemVSG::StateParams> params, ChVisualSystemVSG* appPtr)
        : _params(params), m_appPtr(appPtr) {}

    // Example here taken from the Dear imgui comments (mostly)
    bool operator()() {
        bool visibleComponents = false;
        /*
        ImGuiIO& io = ImGui::GetIO();
        io.FontGlobalScale = _params->guiFontScale;
        */
        // 1. Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.
        if (_params->showGui) {
            char label[64];
            int nstr = sizeof(label) - 1;
            ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
            ImGui::Begin("App:");  // Create a window called "Hello, world!" and append into it.

            if (_params->showVehicleState) {
                ImGui::BeginTable("CamTable", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                                  ImVec2(0.0f, 0.0f));
                ImGui::TableNextColumn();
                ImGui::Text("Camera State:");
                ImGui::TableNextColumn();
                ImGui::Text(_params->camera_mode.c_str());
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Input Node:");
                ImGui::TableNextColumn();
                ImGui::Text(_params->input_mode.c_str());
                ImGui::EndTable();
                ImGui::Spacing();
                ImGui::BeginTable("VehTable", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                                  ImVec2(0.0f, 0.0f));
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Vehicle Speed:");
                ImGui::TableNextColumn();
                snprintf(label, nstr, "%.3f m/s", _params->vehicleSpeed);
                ImGui::Text(label);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Steering:");
                ImGui::TableNextColumn();
                snprintf(label, nstr, "%.3f", _params->steering);
                ImGui::Text(label);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Throttle:");
                ImGui::TableNextColumn();
                snprintf(label, nstr, "%.3f", _params->throttle);
                ImGui::Text(label);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Braking:");
                ImGui::TableNextColumn();
                snprintf(label, nstr, "%.3f", _params->braking);
                ImGui::Text(label);
                ImGui::EndTable();
                ImGui::Spacing();
                ImGui::BeginTable("PowerTable", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                                  ImVec2(0.0f, 0.0f));
                ImGui::TableNextColumn();
                ImGui::Text("Engine Speed:");
                ImGui::TableNextColumn();
                snprintf(label, nstr, "%.1lf RPM", m_appPtr->GetEngineSpeedRPM());
                ImGui::Text(label);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Engine Torque:");
                ImGui::TableNextColumn();
                snprintf(label, nstr, "%.1lf Nm", m_appPtr->GetEngineTorque());
                ImGui::Text(label);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                switch (m_appPtr->GetDriveMode()) {
                    case 'F':
                        snprintf(label, nstr, "[%c] Gear forward:", m_appPtr->GetTransmissionMode());
                        break;
                    case 'N':
                        snprintf(label, nstr, "[%c] Gear neutral:", m_appPtr->GetTransmissionMode());
                        break;
                    case 'R':
                        snprintf(label, nstr, "[%c] Gear reverse:", m_appPtr->GetTransmissionMode());
                        break;
                    default:
                        snprintf(label, nstr, "[%c] Gear ?:", m_appPtr->GetTransmissionMode());
                        break;
                }
                ImGui::Text(label);
                ImGui::TableNextColumn();
                snprintf(label, nstr, "%d", m_appPtr->GetGearPosition());
                ImGui::Text(label);
                ImGui::TableNextRow();
                ImGui::EndTable();
                ImGui::Spacing();
                if (_params->show_converter_data) {
                    ImGui::BeginTable("ConvTable", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                                      ImVec2(0.0f, 0.0f));
                    ImGui::TableNextColumn();
                    ImGui::Text("T.conv.slip:");
                    ImGui::TableNextColumn();
                    snprintf(label, nstr, "%.2f", m_appPtr->GetTconvSlip());
                    ImGui::Text(label);
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::Text("T.conv.torque.in:");
                    ImGui::TableNextColumn();
                    snprintf(label, nstr, "%.1f Nm", m_appPtr->GetTconvTorqueInput());
                    ImGui::Text(label);
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::Text("T.conv.torque.out:");
                    ImGui::TableNextColumn();
                    snprintf(label, nstr, "%.1f Nm", m_appPtr->GetTconvTorqueOutput());
                    ImGui::Text(label);
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::Text("T.conv.speed.out:");
                    ImGui::TableNextColumn();
                    snprintf(label, nstr, "%.1f RPM", m_appPtr->GetTconvSpeedOutput());
                    ImGui::Text(label);
                    ImGui::TableNextRow();
                    ImGui::EndTable();
                    ImGui::Spacing();
                }
                if (m_appPtr->GetNumDrivenAxles() > 0) {
                    ImGui::BeginTable("TireTable", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                                      ImVec2(0.0f, 0.0f));
                    ImGui::TableNextColumn();
                    snprintf(label, nstr, "Torques wheel L: %+5.1f Nm", m_appPtr->GetTireTorque(0, 0));
                    ImGui::Text(label);
                    ImGui::TableNextColumn();
                    snprintf(label, nstr, " R: %+5.1f Nm", m_appPtr->GetTireTorque(0, 1));
                    ImGui::Text(label);
                    ImGui::TableNextRow();
                    if (m_appPtr->GetNumDrivenAxles() >= 2) {
                        ImGui::TableNextColumn();
                        snprintf(label, nstr, "Torques wheel L: %+5.1f Nm", m_appPtr->GetTireTorque(1, 0));
                        ImGui::Text(label);
                        ImGui::TableNextColumn();
                        snprintf(label, nstr, " R: %+5.1f Nm", m_appPtr->GetTireTorque(1, 1));
                        ImGui::Text(label);
                        ImGui::TableNextRow();
                    }
                    if (m_appPtr->GetNumDrivenAxles() >= 4) {
                        ImGui::TableNextColumn();
                        snprintf(label, nstr, "Torques wheel L: %+5.1f Nm", m_appPtr->GetTireTorque(2, 0));
                        ImGui::Text(label);
                        ImGui::TableNextColumn();
                        snprintf(label, nstr, " R: %+5.1f Nm", m_appPtr->GetTireTorque(2, 1));
                        ImGui::Text(label);
                        ImGui::TableNextRow();
                        ImGui::TableNextColumn();
                        snprintf(label, nstr, "Torques wheel L: %+5.1f Nm", m_appPtr->GetTireTorque(3, 0));
                        ImGui::Text(label);
                        ImGui::TableNextColumn();
                        snprintf(label, nstr, " R: %+5.1f Nm", m_appPtr->GetTireTorque(3, 1));
                        ImGui::Text(label);
                        ImGui::TableNextRow();
                    }
                    ImGui::EndTable();
                    ImGui::Spacing();
                }
            }
            if (_params->showTrackedVehicleState) {
                ImGui::BeginTable("TrackDriveTable", 3, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                                  ImVec2(0.0f, 0.0f));
                ImGui::TableNextColumn();
                snprintf(label, nstr, "Sprocket Torque");
                ImGui::Text(label);
                ImGui::TableNextColumn();
                snprintf(label, nstr, "L: %+5.1f Nm", m_appPtr->GetSprocketTorque(0));
                ImGui::Text(label);
                ImGui::TableNextColumn();
                snprintf(label, nstr, " R: %+5.1f Nm", m_appPtr->GetSprocketTorque(1));
                ImGui::Text(label);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                snprintf(label, nstr, "Sprocket Speed");
                ImGui::Text(label);
                ImGui::TableNextColumn();
                snprintf(label, nstr, "L: %+5.1f RPM", m_appPtr->GetSprocketSpeed(0));
                ImGui::Text(label);
                ImGui::TableNextColumn();
                snprintf(label, nstr, " R: %+5.1f RPM", m_appPtr->GetSprocketSpeed(1));
                ImGui::Text(label);
                ImGui::TableNextRow();
                ImGui::EndTable();
                ImGui::Spacing();
            }
            ImGui::BeginTable("SimTable", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                              ImVec2(0.0f, 0.0f));
            snprintf(label, nstr, "%.4f s", m_appPtr->GetModelTime());
            ImGui::TableNextColumn();
            ImGui::Text("Model Time:");
            ImGui::TableNextColumn();
            ImGui::Text(label);
            ImGui::TableNextRow();
            snprintf(label, nstr, "%.4f s", m_appPtr->GetWallclockTime());
            ImGui::TableNextColumn();
            ImGui::Text("Wall Clock Time:");
            ImGui::TableNextColumn();
            ImGui::Text(label);
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("Real Time Factor:");
            ImGui::TableNextColumn();
            snprintf(label, nstr, "%.2f", m_appPtr->GetRealtimeFactor());
            ImGui::Text(label);
            ImGui::EndTable();
            ImGui::Spacing();

            if (_params->show_color_bar) {
                float alpha = 1.0f;
                float cv = 0.9f;
                float cv13 = cv / 3;
                float cv23 = 2 * cv13;
                ImGui::Text("Color Code: %s", _params->cb_title.c_str());
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0, 0.0, cv, alpha));
                snprintf(label, nstr, "%.3f", _params->cb_min);
                ImGui::Button(label);
                ImGui::PopStyleColor(1);
                ImGui::SameLine();
                double cb_stride = _params->cb_max - _params->cb_min;
                double cb_val = _params->cb_min + cb_stride * 1.0 / 6.0;
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0, cv13, cv, alpha));
                snprintf(label, nstr, "%.3f", cb_val);
                ImGui::Button(label);
                ImGui::PopStyleColor(1);
                ImGui::SameLine();
                cb_val = _params->cb_min + cb_stride * 2.0 / 6.0;
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0, cv23, cv, alpha));
                snprintf(label, nstr, "%.3f", cb_val);
                ImGui::Button(label);
                ImGui::PopStyleColor(1);
                ImGui::SameLine();
                cb_val = _params->cb_min + 0.5 * cb_stride;
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0, cv, 0.0, alpha));
                snprintf(label, nstr, "%.3f", cb_val);
                ImGui::Button(label);
                ImGui::PopStyleColor(1);
                ImGui::SameLine();
                cb_val = _params->cb_min + cb_stride * 4.0 / 6.0;
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(cv, cv23, 0.0, alpha));
                snprintf(label, nstr - 1, "%.3f", cb_val);
                ImGui::Button(label);
                ImGui::PopStyleColor(1);
                ImGui::SameLine();
                cb_val = _params->cb_min + cb_stride * 5.0 / 6.0;
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(cv, cv13, 0.0, alpha));
                snprintf(label, nstr, "%.3f", cb_val);
                ImGui::Button(label);
                ImGui::PopStyleColor(1);
                ImGui::SameLine();
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(cv, 0.0, 0.0, alpha));
                snprintf(label, nstr, "%.3f", _params->cb_max);
                ImGui::Button(label);
                ImGui::PopStyleColor(1);
            }
            if (ImGui::Button(
                    "Quit"))  // Buttons return true when clicked (most widgets return true whenedited/activated)
                m_appPtr->Quit();

            ImGui::End();
            visibleComponents = true;
        }

        return visibleComponents;
    }

  private:
    vsg::ref_ptr<ChVisualSystemVSG::StateParams> _params;
    ChVisualSystemVSG* m_appPtr;
};

class FindVertexData : public vsg::Visitor {
  public:
    void apply(vsg::Object& object) { object.traverse(*this); }

    void apply(vsg::BindVertexBuffers& bvd) {
        if (bvd.arrays.empty())
            return;
        bvd.arrays[0]->data->accept(*this);
    }

    void apply(vsg::vec3Array& vertices) {
        if (verticesSet.count(&vertices) == 0) {
            verticesSet.insert(&vertices);
        }
    }

    std::vector<vsg::ref_ptr<vsg::vec3Array>> getVerticesList() {
        std::vector<vsg::ref_ptr<vsg::vec3Array>> verticesList(verticesSet.size());
        auto vertices_itr = verticesList.begin();
        for (auto& vertices : verticesSet) {
            (*vertices_itr++) = const_cast<vsg::vec3Array*>(vertices);
        }

        return verticesList;
    }

    std::set<vsg::vec3Array*> verticesSet;
};

class FindNormalData : public vsg::Visitor {
  public:
    void apply(vsg::Object& object) { object.traverse(*this); }

    void apply(vsg::BindVertexBuffers& bvd) {
        if (bvd.arrays.empty())
            return;
        bvd.arrays[1]->data->accept(*this);
    }

    void apply(vsg::vec3Array& normals) {
        if (normalsSet.count(&normals) == 0) {
            normalsSet.insert(&normals);
        }
    }

    std::vector<vsg::ref_ptr<vsg::vec3Array>> getNormalsList() {
        std::vector<vsg::ref_ptr<vsg::vec3Array>> normalsList(normalsSet.size());
        auto normals_itr = normalsList.begin();
        for (auto& normals : normalsSet) {
            (*normals_itr++) = const_cast<vsg::vec3Array*>(normals);
        }

        return normalsList;
    }

    std::set<vsg::vec3Array*> normalsSet;
};

class FindColorData : public vsg::Visitor {
  public:
    void apply(vsg::Object& object) { object.traverse(*this); }

    void apply(vsg::BindVertexBuffers& bvd) {
        if (bvd.arrays.empty())
            return;
        bvd.arrays[3]->data->accept(*this);
    }

    void apply(vsg::vec4Array& colors) {
        if (colorsSet.count(&colors) == 0) {
            colorsSet.insert(&colors);
        }
    }

    std::vector<vsg::ref_ptr<vsg::vec4Array>> getColorsList() {
        std::vector<vsg::ref_ptr<vsg::vec4Array>> colorsList(colorsSet.size());
        auto colors_itr = colorsList.begin();
        for (auto& colors : colorsSet) {
            (*colors_itr++) = const_cast<vsg::vec4Array*>(colors);
        }

        return colorsList;
    }

    std::set<vsg::vec4Array*> colorsSet;
};

class AppKeyboardHandler : public vsg::Inherit<vsg::Visitor, AppKeyboardHandler> {
  public:
    AppKeyboardHandler(vsg::Viewer* viewer) : m_viewer(viewer) {}

    void SetParams(vsg::ref_ptr<ChVisualSystemVSG::StateParams> params, ChVisualSystemVSG* appPtr) {
        _params = params;
        m_appPtr = appPtr;
    }

    void apply(vsg::KeyPressEvent& keyPress) override {
        if (keyPress.keyBase == 'm' || keyPress.keyModified == 'm') {
            // toggle graphical menu
            _params->showGui = !_params->showGui;
        }
        if (keyPress.keyBase == 't' || keyPress.keyModified == 't') {
            // terminate process
            m_appPtr->Quit();
        }
        if (keyPress.keyBase == vsg::KEY_Escape || keyPress.keyModified == 65307) {
            // terminate process
            m_appPtr->Quit();
        }
    }

  private:
    vsg::observer_ptr<vsg::Viewer> m_viewer;
    vsg::ref_ptr<ChVisualSystemVSG::StateParams> _params;
    ChVisualSystemVSG* m_appPtr;
};

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

double ChVisualSystemVSG::GetModelTime() {
    if (m_systems[0]) {
        return m_systems[0]->GetChTime();
    } else {
        return 0.0;
    }
}

double ChVisualSystemVSG::GetWallclockTime() {
    if (GetModelTime() > 0.0)
        return double(clock()) / double(CLOCKS_PER_SEC) - m_params->time_begin;
    else
        return 0.0;
}

double ChVisualSystemVSG::GetRealtimeFactor() {
    if (GetModelTime() > 0.0) {
        return GetWallclockTime() / GetModelTime();
    } else {
        return 0.0;
    }
}

ChVisualSystemVSG::ChVisualSystemVSG() {
    m_windowTitle = string("Window Title");
    m_clearColor = ChColor(0.0, 0.0, 0.0);
    m_skyboxPath = string("vsg/textures/chrono_skybox.ktx2");
    m_useSkybox = false;
    m_cameraUpVector = vsg::dvec3(0, 0, 1);
    m_yup = false;
    // creation here allows to set entries before initialize
    m_bodyScene = vsg::Group::create();
    m_cogScene = vsg::Group::create();
    m_linkScene = vsg::Group::create();
    m_particleScene = vsg::Group::create();
    m_decoScene = vsg::Group::create();
    m_symbolScene = vsg::Group::create();
    m_deformableScene = vsg::Group::create();
    // set up defaults and read command line arguments to override them
    m_options = vsg::Options::create();
    m_options->paths = vsg::getEnvPaths("VSG_FILE_PATH");
    m_options->paths.push_back(GetChronoDataPath());
    // add vsgXchange's support for reading and writing 3rd party file formats, mandatory for chrono_vsg!
    m_options->add(vsgXchange::all::create());
    m_options->sharedObjects = vsg::SharedObjects::create();
    m_shapeBuilder = ShapeBuilder::create();
    m_shapeBuilder->m_options = m_options;
    m_shapeBuilder->m_sharedObjects = m_options->sharedObjects;
    m_vsgBuilder = vsg::Builder::create();
    m_vsgBuilder->options = m_options;
    m_renderGui = nullptr;
    // make some default settings
    SetWindowTitle("VSG: Vehicle Demo");
    SetWindowSize(ChVector2<int>(800, 600));
    SetWindowPosition(ChVector2<int>(50, 50));
    SetUseSkyBox(true);
    SetCameraAngleDeg(40);
    SetLightIntensity(1.0);
    SetLightDirection(1.5 * CH_C_PI_2, CH_C_PI_4);
#ifdef __APPLE__
    SetGuiFontSize(20.0);
#else
    SetGuiFontSize(10.0);
#endif
}

void ChVisualSystemVSG::SetOutputScreen(int screenNum) {
    if (m_initialized) {
        GetLog() << "Function '" << __func__ << "' must be used before initialization!\n";
        return;
    }
    int maxNum = vsg::Device::maxNumDevices();
    GetLog() << "Screens found: " << maxNum << "\n";
    if(screenNum >= 0 && screenNum < maxNum) {
        m_screen_num = screenNum;
    } else {
        GetLog() << "Screen #" << screenNum << " cannot be used on this computer!\n";
        exit(1);
    }
}

void ChVisualSystemVSG::SetFullscreen(bool yesno) {
    if (m_initialized) {
        GetLog() << "Function '" << __func__ << "' must be used before initialization!\n";
        return;
    }
    m_use_fullscreen = yesno;
}

void ChVisualSystemVSG::AttachGui() {
    m_renderGui = vsgImGui::RenderImGui::create(m_window, GuiComponent(m_params, this));
    if (!m_renderGui) {
        GetLog() << "Could not create GUI!\n";
    }
}

ChVisualSystemVSG::~ChVisualSystemVSG() {}

void ChVisualSystemVSG::Quit() {
    m_viewer->close();
}

void ChVisualSystemVSG::SetGuiFontSize(float theSize) {
    if (m_initialized) {
        GetLog() << "Function '" << __func__ << "' must be used before initialization!\n";
        return;
    }
    m_guiFontSize = theSize;
}

void ChVisualSystemVSG::SetWindowSize(ChVector2<int> size) {
    if (m_initialized) {
        GetLog() << "Function '" << __func__ << "' must be used before initialization!\n";
        return;
    }
    m_windowWidth = size[0];
    m_windowHeight = size[1];
}

void ChVisualSystemVSG::SetWindowSize(int width, int height) {
    if (m_initialized) {
        GetLog() << "Function '" << __func__ << "' must be used before initialization!\n";
        return;
    }
    m_windowWidth = width;
    m_windowHeight = height;
}

void ChVisualSystemVSG::SetWindowPosition(ChVector2<int> pos) {
    if (m_initialized) {
        GetLog() << "Function '" << __func__ << "' must be used before initialization!\n";
        return;
    }
    m_windowX = pos[0];
    m_windowY = pos[1];
}

void ChVisualSystemVSG::SetWindowPosition(int from_left, int from_top) {
    if (m_initialized) {
        GetLog() << "Function '" << __func__ << "' must be used before initialization!\n";
        return;
    }
    m_windowX = from_left;
    m_windowY = from_top;
}

void ChVisualSystemVSG::SetWindowTitle(std::string title) {
    if (m_initialized) {
        GetLog() << "Function '" << __func__ << "' must be used before initialization!\n";
        return;
    }
    m_windowTitle = title;
}

void ChVisualSystemVSG::SetClearColor(ChColor color) {
    if (m_initialized) {
        GetLog() << "Function '" << __func__ << "' must be used before initialization!\n";
        return;
    }
    m_clearColor = color;
}

void ChVisualSystemVSG::SetUseSkyBox(bool yesno) {
    if (m_initialized) {
        GetLog() << "Function '" << __func__ << "' must be used before initialization!\n";
        return;
    }
    m_useSkybox = yesno;
}

void ChVisualSystemVSG::AddCamera(const ChVector<>& pos, ChVector<> targ) {
    if (m_initialized) {
        GetLog() << "Function '" << __func__ << "' must be used before initialization!\n";
        exit(42);
    }
    ChVector<> test = pos - targ;
    if(test.Length() == 0.0) {
        GetLog() << "Function '" << __func__ << "' Camera Pos and Target cannot be identical!\n";
        GetLog() << "  pos    = { " << pos.x() << " ; " << pos.y() << " ; " << pos.z() << " }\n";
        GetLog() << "  target = { " << targ.x() << " ; " << targ.y() << " ; " << targ.z() << " }\n";
        exit(42);
    }
    if(m_yup) {
        if(pos.x() == 0.0 && pos.z() == 0.0) {
            GetLog() << "Function '" << __func__ << "' Line of sight is parallel to upvector! -> Corrected!!\n";
            m_vsg_cameraEye = vsg::dvec3(pos.x()+1.0, pos.y(), pos.z()+1.0);
        } else {
            m_vsg_cameraEye = vsg::dvec3(pos.x(), pos.y(), pos.z());
        }
    } else {
        if(pos.x() == 0.0 && pos.y() == 0.0) {
            GetLog() << "Function '" << __func__ << "' Line of sight is parallel to upvector! -> Corrected!!\n";
            m_vsg_cameraEye = vsg::dvec3(pos.x()+1.0, pos.y()+1.0, pos.z());
        } else {
            m_vsg_cameraEye = vsg::dvec3(pos.x(), pos.y(), pos.z());
        }
    }
    m_vsg_cameraTarget = vsg::dvec3(targ.x(), targ.y(), targ.z());
}

void ChVisualSystemVSG::UpdateCamera(const ChVector<>& pos, ChVector<> targ) {
    if (!m_initialized) {
        GetLog() << "Function '" << __func__ << "' must be used after initialization!\n";
        exit(42);
    }
    ChVector<> test = pos - targ;
    if(test.Length() == 0.0) {
        GetLog() << "Function '" << __func__ << "' Camera Pos and Target cannot be identical!\n";
        GetLog() << "  pos    = { " << pos.x() << " ; " << pos.y() << " ; " << pos.z() << " }\n";
        GetLog() << "  target = { " << targ.x() << " ; " << targ.y() << " ; " << targ.z() << " }\n";
        exit(42);
    }
    m_lookAt->eye = vsg::dvec3(pos.x(), pos.y(), pos.z());
    m_lookAt->center = vsg::dvec3(targ.x(), targ.y(), targ.z());
}

void ChVisualSystemVSG::ShowAllCoGs(double size) {
    if (m_initialized) {
        GetLog() << "Function '" << __func__ << "' must be used before initialization!\n";
        return;
    }
    m_params->cogSymbolSize = size;
}

void ChVisualSystemVSG::SetCameraVertical(CameraVerticalDir upDir) {
    if (m_initialized) {
        GetLog() << "Function '" << __func__ << "' must be used before initialization!\n";
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

void ChVisualSystemVSG::SetLightDirection(double acimut, double elevation) {
    if (m_initialized) {
        GetLog() << "Function '" << __func__ << "' must be used before initialization!\n";
        return;
    }
    m_acimut = ChClamp(acimut, -CH_C_PI, CH_C_PI);
    m_elevation = ChClamp(elevation, 0.0, CH_C_PI_2);
}

size_t ChVisualSystemVSG::GetFrameNumber() {
    return m_params->frame_number;
}

void ChVisualSystemVSG::Initialize() {
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
    ambientLight->color.set(1.0, 1.0, 1.0);
    ambientLight->intensity = 0.1;

    auto directionalLight = vsg::DirectionalLight::create();
    directionalLight->name = "head light";
    directionalLight->color.set(1.0, 1.0, 1.0);
    directionalLight->intensity = m_lightIntensity;
    if (m_yup) {
        directionalLight->direction.set(-cos(m_elevation) * cos(m_acimut), -sin(m_elevation),
                                        -cos(m_elevation) * sin(m_acimut));
    } else {
        directionalLight->direction.set(-cos(m_elevation) * cos(m_acimut), -cos(m_elevation) * sin(m_acimut),
                                        -sin(m_elevation));
    }

    auto absoluteTransform = vsg::AbsoluteTransform::create();
    absoluteTransform->addChild(ambientLight);
    absoluteTransform->addChild(directionalLight);

    m_scene->addChild(absoluteTransform);
    m_scene->addChild(m_bodyScene);
    m_scene->addChild(m_cogScene);
    m_scene->addChild(m_linkScene);
    m_scene->addChild(m_particleScene);
    m_scene->addChild(m_decoScene);
    m_scene->addChild(m_symbolScene);
    m_scene->addChild(m_deformableScene);

    BindAll();

    // create the viewer and assign window(s) to it
    m_viewer = vsg::Viewer::create();

    m_window = vsg::Window::create(windowTraits);
    if (!m_window) {
        std::cout << "Could not create window." << std::endl;
        return;
    }
    auto& limits = m_window->getOrCreatePhysicalDevice()->getProperties().limits;  // VkPhysicalDeviceLimits
    auto prop = m_window->getPhysicalDevice()->getProperties();
    GetLog() << "****************************************************\n";
    GetLog() << "* Chrono::VSG Vulkan Scene Graph 3D-Visualization\n";
    GetLog() << "* GPU Name: " << prop.deviceName << "\n";
    switch (prop.deviceType) {
        default:
        case VK_PHYSICAL_DEVICE_TYPE_OTHER:
            GetLog() << "* GPU Type: VK_PHYSICAL_DEVICE_TYPE_OTHER"
                     << "\n";
            break;
        case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU:
            GetLog() << "* GPU Type: VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU"
                     << "\n";
            break;
        case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU:
            GetLog() << "* GPU Type: VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU"
                     << "\n";
            break;
        case VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU:
            GetLog() << "* GPU Type: VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU"
                     << "\n";
            break;
        case VK_PHYSICAL_DEVICE_TYPE_CPU:
            GetLog() << "* GPU Type: VK_PHYSICAL_DEVICE_TYPE_CPU"
                     << "\n";
            break;
    }
    GetLog() << "* Vulkan Version: " << VK_VERSION_MAJOR(VK_HEADER_VERSION_COMPLETE) << "."
             << VK_VERSION_MINOR(VK_HEADER_VERSION_COMPLETE) << "." << VK_API_VERSION_PATCH(VK_HEADER_VERSION_COMPLETE)
             << "\n";
    GetLog() << "* Vulkan Scene Graph Version: " << VSG_VERSION_STRING << "\n";
    GetLog() << "* Graphic Output Possible on: " << vsg::Device::maxNumDevices() << " Screens.\n";
    GetLog() << "****************************************************\n";
    m_shapeBuilder->m_maxAnisotropy = limits.maxSamplerAnisotropy;
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

    // add keyboard handler
    auto kbHandler = AppKeyboardHandler::create(m_viewer);
    kbHandler->SetParams(m_params, this);
    m_viewer->addEventHandler(kbHandler);

    m_viewer->addEventHandler(vsg::CloseHandler::create(m_viewer));

    if (!m_params->showVehicleState)
        m_viewer->addEventHandler(vsg::Trackball::create(m_vsg_camera));

    // default sets automatic directional light
    // auto renderGraph = vsg::RenderGraph::create(m_window, m_view);
    // switches off automatic directional light setting

    auto renderGraph =
        vsg::createRenderGraphForView(m_window, m_vsg_camera, m_scene, VK_SUBPASS_CONTENTS_INLINE, false);
    auto commandGraph = vsg::CommandGraph::create(m_window, renderGraph);

    // initialize ImGui
    ImGui::CreateContext();
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
    // Create the ImGui node and add it to the renderGraph
    AttachGui();
    if (m_renderGui) {
        renderGraph->addChild(m_renderGui);
    }

    // Add the ImGui event handler first to handle events early
    m_viewer->addEventHandler(vsgImGui::SendEventsToImGui::create());

    m_viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});

    // assign a CompileTraversal to the Builder that will compile for all the views assigned to the viewer,
    // must be done after Viewer.assignRecordAndSubmitTasksAndPresentations();
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

    // prepare reading 3d files
    m_loadThreads = vsg::OperationThreads::create(m_numThreads, m_viewer->status);

    m_initialized = true;
}

bool ChVisualSystemVSG::Run() {
    return m_viewer->active();
}

void ChVisualSystemVSG::Render() {
    if (m_params->frame_number == 0)
        m_params->time_begin = double(clock()) / double(CLOCKS_PER_SEC);

    UpdateFromMBS();

    if (!m_viewer->advanceToNextFrame()) {
        return;
    }

    // pass any events into EventHandlers assigned to the Viewer
    m_viewer->handleEvents();

    m_viewer->update();

    //=================================================================================
    // Dynamic data transfer CPU -> GPU
    if (m_allowVertexTransfer) {
        for (auto& vertices : m_vsgVerticesList) {
            size_t k = 0;
            for (auto& v : *vertices) {
                float x = m_mbsMesh->GetMesh()->getCoordsVertices().at(k).x();
                float y = m_mbsMesh->GetMesh()->getCoordsVertices().at(k).y();
                float z = m_mbsMesh->GetMesh()->getCoordsVertices().at(k).z();
                v.set(x, y, z);
                k++;
            }
            vertices->dirty();
        }
    }

    if (m_allowNormalsTransfer) {
        for (auto& normals : m_vsgNormalsList) {
            size_t k = 0;
            for (auto& n : *normals) {
                float x = m_mbsMesh->GetMesh()->getCoordsNormals().at(k).x();
                float y = m_mbsMesh->GetMesh()->getCoordsNormals().at(k).y();
                float z = m_mbsMesh->GetMesh()->getCoordsNormals().at(k).z();
                n.set(x, y, z);
                k++;
            }
            normals->dirty();
        }
    }

    if (m_allowColorsTransfer) {
        for (auto& colors : m_vsgColorsList) {
            size_t k = 0;
            for (auto& c : *colors) {
                float r = m_mbsMesh->GetMesh()->getCoordsColors().at(k).R;
                float g = m_mbsMesh->GetMesh()->getCoordsColors().at(k).G;
                float b = m_mbsMesh->GetMesh()->getCoordsColors().at(k).B;
                float a = 1.0f;
                c.set(r, g, b, a);
                k++;
            }
            colors->dirty();
        }
    }
    //=================================================================================

    m_viewer->recordAndSubmit();

    if (m_params->do_image_capture) {
        exportScreenshot(m_window, m_options, m_imageFilename);
        m_params->do_image_capture = false;
    }

    m_viewer->present();
    m_params->frame_number++;
}

void ChVisualSystemVSG::WriteImageToFile(const string& filename) {
    m_imageFilename = filename;
    m_params->do_image_capture = true;
}

void ChVisualSystemVSG::BindAll() {
    if (m_systems.empty()) {
        return;
    }
    if (m_systems[0]->Get_bodylist().size() < 1) {
        return;
    }
    // generate CoG symbols if needed
    if (m_params->cogSymbolSize > 0.0f) {
        for (auto& body : m_systems[0]->GetAssembly().Get_bodylist()) {
            auto pos = body->GetPos();
            auto rotAngle = body->GetRotAngle();
            auto rotAxis = body->GetRotAxis();
            vsg::dvec3 scale(m_params->cogSymbolSize, m_params->cogSymbolSize, m_params->cogSymbolSize);
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                                vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) * vsg::scale(scale);
            m_cogScene->addChild(m_shapeBuilder->createCoGSymbol(body, transform));
        }
    }
    for (auto& body : m_systems[0]->GetAssembly().Get_bodylist()) {
        // CreateIrrNode(body);
        if (!body->GetVisualModel()) {
            continue;
        }
        // Get the visual model reference frame
        const ChFrame<>& X_AM = body->GetVisualModelFrame();
        for (const auto& shape_instance : body->GetVisualModel()->GetShapes()) {
            auto& shape = shape_instance.first;
            auto& X_SM = shape_instance.second;
            ChFrame<> X_SA = X_AM * X_SM;
            auto pos = X_SA.GetPos();
            auto rot = X_SA.GetRot();
            double rotAngle;
            ChVector<> rotAxis;
            rot.Q_to_AngAxis(rotAngle, rotAxis);
            std::shared_ptr<ChVisualMaterial> material;
            if (shape->GetMaterials().empty()) {
                material = chrono_types::make_shared<ChVisualMaterial>();
                material->SetDiffuseColor(ChColor(1.0, 1.0, 1.0));
                material->SetAmbientColor(ChColor(1.0, 1.0, 1.0));
            } else {
                material = shape->GetMaterial(0);
            }
            if (!shape->IsVisible()) {
                continue;
            }
            if (auto box = std::dynamic_pointer_cast<ChBoxShape>(shape)) {
                // we have boxes and dices. Dices take cubetextures, boxes take 6 identical textures
                // dirty trick: if a kd map is set and its name contains "cubetexture" we use dice,
                // if not, we use box
                bool isDice = false;
                if (!material->GetKdTexture().empty()) {
                    size_t found = material->GetKdTexture().find("cubetexture");
                    if (found != string::npos) {
                        isDice = true;
                    }
                }

                ChVector<> scale = box->GetBoxGeometry().Size;
                auto transform = vsg::MatrixTransform::create();
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                                    vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                                    vsg::scale(scale.x(), scale.y(), scale.z());
                if (isDice) {
                    auto tmpGroup =
                        m_shapeBuilder->createShape(ShapeBuilder::DICE_SHAPE, material, transform, m_draw_as_wireframe);
                    ShapeBuilder::SetMBSInfo(tmpGroup, body, shape_instance);
                    m_bodyScene->addChild(tmpGroup);
                } else {
                    auto tmpGroup =
                        m_shapeBuilder->createShape(ShapeBuilder::BOX_SHAPE, material, transform, m_draw_as_wireframe);
                    ShapeBuilder::SetMBSInfo(tmpGroup, body, shape_instance);
                    m_bodyScene->addChild(tmpGroup);
                }
            } else if (auto sphere = std::dynamic_pointer_cast<ChSphereShape>(shape)) {
                ChVector<> scale = sphere->GetSphereGeometry().rad;
                auto transform = vsg::MatrixTransform::create();
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                                    vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                                    vsg::scale(scale.x(), scale.y(), scale.z());
                auto tmpGroup =
                    m_shapeBuilder->createShape(ShapeBuilder::SPHERE_SHAPE, material, transform, m_draw_as_wireframe);
                ShapeBuilder::SetMBSInfo(tmpGroup, body, shape_instance);
                m_bodyScene->addChild(tmpGroup);
            } else if (auto ellipsoid = std::dynamic_pointer_cast<ChEllipsoidShape>(shape)) {
                ChVector<> scale = ellipsoid->GetEllipsoidGeometry().rad;
                auto transform = vsg::MatrixTransform::create();
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                                    vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                                    vsg::scale(scale.x(), scale.y(), scale.z());
                auto tmpGroup =
                    m_shapeBuilder->createShape(ShapeBuilder::SPHERE_SHAPE, material, transform, m_draw_as_wireframe);
                ShapeBuilder::SetMBSInfo(tmpGroup, body, shape_instance);
                m_bodyScene->addChild(tmpGroup);
            } else if (auto capsule = std::dynamic_pointer_cast<ChCapsuleShape>(shape)) {
                double rad = capsule->GetCapsuleGeometry().rad;
                double height = capsule->GetCapsuleGeometry().hlen;
                auto transform = vsg::MatrixTransform::create();
                ChVector<> scale(rad, height, rad);
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                                    vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                                    vsg::scale(scale.x(), scale.y(), scale.z());
                auto tmpGroup =
                    m_shapeBuilder->createShape(ShapeBuilder::CAPSULE_SHAPE, material, transform, m_draw_as_wireframe);
                ShapeBuilder::SetMBSInfo(tmpGroup, body, shape_instance);
                m_bodyScene->addChild(tmpGroup);
            } else if (auto barrel = std::dynamic_pointer_cast<ChBarrelShape>(shape)) {
                //// TODO
            } else if (auto cone = std::dynamic_pointer_cast<ChConeShape>(shape)) {
                Vector rad = cone->GetConeGeometry().rad;
                auto transform = vsg::MatrixTransform::create();
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                                    vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                                    vsg::scale(rad.x(), rad.y(), rad.z());
                auto tmpGroup =
                    m_shapeBuilder->createShape(ShapeBuilder::CONE_SHAPE, material, transform, m_draw_as_wireframe);
                ShapeBuilder::SetMBSInfo(tmpGroup, body, shape_instance);
                m_bodyScene->addChild(tmpGroup);
            } else if (auto trimesh = std::dynamic_pointer_cast<ChTriangleMeshShape>(shape)) {
                ChVector<> scale = trimesh->GetScale();
                auto transform = vsg::MatrixTransform::create();
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                                    vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                                    vsg::scale(scale.x(), scale.y(), scale.z());
                if (trimesh->GetNumMaterials() > 0) {
                    auto tmpGroup = m_shapeBuilder->createTrimeshMatShape(transform, m_draw_as_wireframe, trimesh);
                    ShapeBuilder::SetMBSInfo(tmpGroup, body, shape_instance);
                    m_bodyScene->addChild(tmpGroup);
                } else {
                    auto tmpGroup = m_shapeBuilder->createTrimeshColShape(transform, m_draw_as_wireframe, trimesh);
                    ShapeBuilder::SetMBSInfo(tmpGroup, body, shape_instance);
                    m_bodyScene->addChild(tmpGroup);
                }
            } else if (auto surface = std::dynamic_pointer_cast<ChSurfaceShape>(shape)) {
                auto transform = vsg::MatrixTransform::create();
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                                    vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                                    vsg::scale(1.0, 1.0, 1.0);
                auto tmpGroup = m_shapeBuilder->createShape(ShapeBuilder::SURFACE_SHAPE, material, transform,
                                                            m_draw_as_wireframe, surface);
                ShapeBuilder::SetMBSInfo(tmpGroup, body, shape_instance);
                m_bodyScene->addChild(tmpGroup);
            } else if (auto obj = std::dynamic_pointer_cast<ChObjFileShape>(shape)) {
                string objFilename = obj->GetFilename();
                size_t objHashValue = m_stringHash(objFilename);
                auto grp = vsg::Group::create();
                auto transform = vsg::MatrixTransform::create();
                grp->setValue("ItemPtr", body);
                grp->setValue("ShapeInstancePtr", shape_instance);
                grp->setValue("TransformPtr", transform);
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                                    vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z());
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
                        m_bodyScene->addChild(grp);
                        m_objCache[objHashValue] = node;
                    }
                } else {
                    transform->addChild(m_objCache[objHashValue]);
                    m_bodyScene->addChild(grp);
                }
            } else if (auto line = std::dynamic_pointer_cast<ChLineShape>(shape)) {
                auto transform = vsg::MatrixTransform::create();
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                                    vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                                    vsg::scale(1.0, 1.0, 1.0);
                m_bodyScene->addChild(m_shapeBuilder->createLineShape(body, shape_instance, material, transform, line));
            } else if (auto path = std::dynamic_pointer_cast<ChPathShape>(shape)) {
                auto transform = vsg::MatrixTransform::create();
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                                    vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                                    vsg::scale(1.0, 1.0, 1.0);
                m_bodyScene->addChild(m_shapeBuilder->createPathShape(body, shape_instance, material, transform, path));
            } else if (auto cylinder = std::dynamic_pointer_cast<ChCylinderShape>(shape)) {
                double rad = cylinder->GetCylinderGeometry().rad;
                const auto& P1 = cylinder->GetCylinderGeometry().p1;
                const auto& P2 = cylinder->GetCylinderGeometry().p2;

                ChVector<> dir = P2 - P1;
                double height = dir.Length();
                dir.Normalize();
                ChVector<> mx, my, mz;
                dir.DirToDxDyDz(my, mz, mx);  // y is axis, in cylinder.obj frame
                ChMatrix33<> R_CS;
                R_CS.Set_A_axis(mx, my, mz);

                auto t_CS = 0.5 * (P2 + P1);
                ChFrame<> X_CS(t_CS, R_CS);
                ChFrame<> X_CA = X_SA * X_CS;

                pos = X_CA.GetPos();
                rot = X_CA.GetRot();
                rot.Q_to_AngAxis(rotAngle, rotAxis);

                auto transform = vsg::MatrixTransform::create();
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                                    vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                                    vsg::scale(rad, height, rad);
                auto tmpGroup =
                    m_shapeBuilder->createShape(ShapeBuilder::CYLINDER_SHAPE, material, transform, m_draw_as_wireframe);
                ShapeBuilder::SetMBSInfo(tmpGroup, body, shape_instance);
                m_bodyScene->addChild(tmpGroup);
            }
        }
    }
    for (auto& item : m_systems[0]->Get_otherphysicslist()) {
        if (auto pcloud = std::dynamic_pointer_cast<ChParticleCloud>(item)) {
            if (!pcloud->GetVisualModel())
                continue;
            if (!m_particlePattern) {
                std::shared_ptr<ChVisualMaterial> material;
                material = chrono_types::make_shared<ChVisualMaterial>();
                material->SetDiffuseColor(ChColor(1.0, 1.0, 1.0));
                material->SetAmbientColor(ChColor(0.1, 0.1, 0.1));
                m_particlePattern = m_shapeBuilder->createParticlePattern(material, m_draw_as_wireframe);
            }
            auto numParticles = pcloud->GetNparticles();
            std::vector<double> size = pcloud->GetCollisionModel()->GetShapeDimensions(0);
            for (int i = 0; i < pcloud->GetNparticles(); i++) {
                auto group = vsg::Group::create();
                const auto& pos = pcloud->GetVisualModelFrame(i).GetPos();
                auto transform = vsg::MatrixTransform::create();
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) * vsg::scale(size[0], size[0], size[0]);
                transform->addChild(m_particlePattern);
                group->setValue("TransformPtr", transform);
                group->addChild(transform);
                m_particleScene->addChild(group);
                // m_particleScene->addChild(
                //     m_shapeBuilder->createParticleShape(material, transform, m_draw_as_wireframe));
            }
        } else if (auto loadcont = std::dynamic_pointer_cast<ChLoadContainer>(item)) {
            auto visModel = loadcont->GetVisualModel();
            if (!visModel)
                continue;
            const auto& shape_instance = visModel->GetShapes().at(0);
            auto& shape = shape_instance.first;
            auto trimesh = std::dynamic_pointer_cast<ChTriangleMeshShape>(shape);
            if (!trimesh)
                continue;
            auto transform = vsg::MatrixTransform::create();
            if (trimesh->GetNumMaterials() > 0) {
                m_deformableScene->addChild(
                    m_shapeBuilder->createTrimeshMatShape(transform, trimesh->IsWireframe(), trimesh));
            } else {
                m_deformableScene->addChild(
                    m_shapeBuilder->createTrimeshColShapeSCM(transform, trimesh->IsWireframe(), trimesh));
            }
            m_vsgVerticesList = vsg::visit<FindVertexData>(m_deformableScene->children.at(0)).getVerticesList();
            for (auto& vertices : m_vsgVerticesList) {
                vertices->properties.dataVariance = vsg::DYNAMIC_DATA;
                m_num_vsgVertexList += vertices->size();
            }
            m_mbsMesh = trimesh;
            if (m_num_vsgVertexList == trimesh->GetMesh()->getCoordsVertices().size()) {
                m_allowVertexTransfer = true;
            }
            if (m_allowVertexTransfer && !trimesh->IsWireframe()) {
                m_vsgNormalsList = vsg::visit<FindNormalData>(m_deformableScene->children.at(0)).getNormalsList();
                size_t num_vsgNormalsList = 0;
                for (auto& normals : m_vsgNormalsList) {
                    normals->properties.dataVariance = vsg::DYNAMIC_DATA;
                    num_vsgNormalsList += normals->size();
                }
                if (num_vsgNormalsList == m_num_vsgVertexList) {
                    m_allowNormalsTransfer = true;
                }
            }
            if (m_allowVertexTransfer) {
                m_vsgColorsList = vsg::visit<FindColorData>(m_deformableScene->children.at(0)).getColorsList();
                size_t num_vsgColorsList = 0;
                for (auto& colors : m_vsgColorsList) {
                    colors->properties.dataVariance = vsg::DYNAMIC_DATA;
                    num_vsgColorsList += colors->size();
                }
                if (num_vsgColorsList == m_num_vsgVertexList) {
                    m_allowColorsTransfer = true;
                }
            }
        }
    }
    // loop through links in the system
    for (auto ilink : m_systems[0]->Get_linklist()) {
        if (auto link = std::dynamic_pointer_cast<ChLinkTSDA>(ilink)) {
            auto lnkVisModel = link->GetVisualModel();
            if (!lnkVisModel)
                break;
            auto lnkNumShapes = lnkVisModel->GetNumShapes();
            if (lnkNumShapes == 0)
                break;
            for (auto& shape_instance : link->GetVisualModel()->GetShapes()) {
                auto& shape = shape_instance.first;
                if (auto segshape = std::dynamic_pointer_cast<ChSegmentShape>(shape)) {
                    ChVector<> P1 = link->GetPoint1Abs();
                    ChVector<> P2 = link->GetPoint2Abs();
                    double rotAngle, height;
                    ChVector<> rotAxis, pos;
                    Point2PointHelperAbs(P1, P2, height, pos, rotAngle, rotAxis);
                    std::shared_ptr<ChVisualMaterial> material;
                    if (segshape->GetMaterials().empty()) {
                        material = chrono_types::make_shared<ChVisualMaterial>();
                        material->SetDiffuseColor(ChColor(1.0f, 1.0f, 1.0f));
                        material->SetAmbientColor(ChColor(0.1f, 0.1f, 0.1f));
                    } else {
                        material = segshape->GetMaterial(0);
                    }

                    auto transform = vsg::MatrixTransform::create();
                    transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                                        vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                                        vsg::scale(0.0, height, 0.0);
                    m_linkScene->addChild(
                        m_shapeBuilder->createUnitSegment(ilink, shape_instance, material, transform));
                } else if (auto sprshape = std::dynamic_pointer_cast<ChSpringShape>(shape)) {
                    double rad = sprshape->GetRadius();
                    ChVector<> P1 = link->GetPoint1Abs();
                    ChVector<> P2 = link->GetPoint2Abs();
                    double rotAngle, height;
                    ChVector<> rotAxis, pos;
                    Point2PointHelperAbs(P1, P2, height, pos, rotAngle, rotAxis);
                    std::shared_ptr<ChVisualMaterial> material;
                    if (sprshape->GetMaterials().empty()) {
                        material = chrono_types::make_shared<ChVisualMaterial>();
                        material->SetDiffuseColor(ChColor(1.0, 1.0, 1.0));
                        material->SetAmbientColor(ChColor(0.1, 0.1, 0.1));
                    } else {
                        material = sprshape->GetMaterial(0);
                    }

                    auto transform = vsg::MatrixTransform::create();
                    transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                                        vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                                        vsg::scale(rad, height, rad);
                    m_linkScene->addChild(
                        m_shapeBuilder->createSpringShape(ilink, shape_instance, material, transform, sprshape));
                }
            }
        } else if (auto link = std::dynamic_pointer_cast<ChLinkDistance>(ilink)) {
            auto lnkVisModel = link->GetVisualModel();
            if (!lnkVisModel)
                continue;
            auto lnkNumShapes = lnkVisModel->GetNumShapes();
            if (lnkNumShapes == 0)
                continue;
            for (auto& shape_instance : link->GetVisualModel()->GetShapes()) {
                auto& shape = shape_instance.first;
                if (auto segshape = std::dynamic_pointer_cast<ChSegmentShape>(shape)) {
                    ChVector<> P1 = link->GetEndPoint1Abs();
                    ChVector<> P2 = link->GetEndPoint2Abs();
                    double rotAngle, height;
                    ChVector<> rotAxis, pos;
                    Point2PointHelperAbs(P1, P2, height, pos, rotAngle, rotAxis);
                    std::shared_ptr<ChVisualMaterial> material;
                    if (segshape->GetMaterials().empty()) {
                        material = chrono_types::make_shared<ChVisualMaterial>();
                        material->SetDiffuseColor(ChColor(1.0, 1.0, 1.0));
                        material->SetAmbientColor(ChColor(0.1, 0.1, 0.1));
                    } else {
                        material = segshape->GetMaterial(0);
                    }

                    auto transform = vsg::MatrixTransform::create();
                    transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                                        vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                                        vsg::scale(0.0, height, 0.0);
                    m_linkScene->addChild(
                        m_shapeBuilder->createUnitSegment(ilink, shape_instance, material, transform));
                }
            }
        }
    }
}

void ChVisualSystemVSG::UpdateFromMBS() {
    // generate CoG symbols if needed
    if (m_params->cogSymbolSize > 0.0f) {
        for (auto child : m_cogScene->children) {
            std::shared_ptr<ChBody> body;
            vsg::ref_ptr<vsg::MatrixTransform> transform;
            if (!child->getValue("BodyPtr", body))
                continue;
            if (!child->getValue("TransformPtr", transform))
                continue;
            auto pos = body->GetPos();
            auto rotAngle = body->GetRotAngle();
            auto rotAxis = body->GetRotAxis();
            vsg::dvec3 scale(m_params->cogSymbolSize, m_params->cogSymbolSize, m_params->cogSymbolSize);
            transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                                vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) * vsg::scale(scale);
        }
    }
    // update body visualization related graphic nodes
    for (auto child : m_bodyScene->children) {
        std::shared_ptr<ChPhysicsItem> item;
        ChVisualModel::ShapeInstance shapeInstance;
        vsg::ref_ptr<vsg::MatrixTransform> transform;
        if (!child->getValue("ItemPtr", item))
            continue;
        if (!child->getValue("ShapeInstancePtr", shapeInstance))
            continue;
        if (!child->getValue("TransformPtr", transform))
            continue;
        // begin matrix update
        // Get the visual model reference frame
        const ChFrame<>& X_AM = item->GetVisualModelFrame();
        auto shape = shapeInstance.first;
        const auto& X_SM = shapeInstance.second;

        ChFrame<> X_SA = X_AM * X_SM;
        vsg::dvec3 pos(X_SA.GetPos().x(), X_SA.GetPos().y(), X_SA.GetPos().z());
        auto rot = X_SA.GetRot();
        double angle;
        Vector axis;
        rot.Q_to_AngAxis(angle, axis);
        vsg::dvec3 rotax(axis.x(), axis.y(), axis.z());
        if (auto box = std::dynamic_pointer_cast<ChBoxShape>(shape)) {
            vsg::dvec3 size(box->GetBoxGeometry().GetSize().x(), box->GetBoxGeometry().GetSize().y(),
                            box->GetBoxGeometry().GetSize().z());
            transform->matrix = vsg::translate(pos) * vsg::rotate(angle, rotax) * vsg::scale(size);
        } else if (auto sphere = std::dynamic_pointer_cast<ChSphereShape>(shape)) {
            double radius = sphere->GetSphereGeometry().rad;
            // ChVector<> size(radius, radius, radius);
            vsg::dvec3 size(radius, radius, radius);
            transform->matrix = vsg::translate(pos) * vsg::rotate(angle, rotax) * vsg::scale(size);
        } else if (auto line = std::dynamic_pointer_cast<ChLineShape>(shape)) {
            // ChVector<> size(radius, radius, radius);
            vsg::dvec3 size(1.0, 1.0, 1.0);
            transform->matrix = vsg::translate(pos) * vsg::rotate(angle, rotax) * vsg::scale(size);
        } else if (auto path = std::dynamic_pointer_cast<ChPathShape>(shape)) {
            // ChVector<> size(radius, radius, radius);
            vsg::dvec3 size(1.0, 1.0, 1.0);
            transform->matrix = vsg::translate(pos) * vsg::rotate(angle, rotax) * vsg::scale(size);
        } else if (auto surface = std::dynamic_pointer_cast<ChSurfaceShape>(shape)) {
            // ChVector<> size(radius, radius, radius);
            vsg::dvec3 size(1.0, 1.0, 1.0);
            transform->matrix = vsg::translate(pos) * vsg::rotate(angle, rotax) * vsg::scale(size);
        } else if (auto trimesh = std::dynamic_pointer_cast<ChTriangleMeshShape>(shape)) {
            vsg::dvec3 size(trimesh->GetScale().x(), trimesh->GetScale().y(), trimesh->GetScale().z());
            transform->matrix = vsg::translate(pos) * vsg::rotate(angle, rotax) * vsg::scale(size);
        } else if (auto ellipsoid = std::dynamic_pointer_cast<ChEllipsoidShape>(shape)) {
            ChVector<> radius = ellipsoid->GetEllipsoidGeometry().rad;
            // ChVector<> size(radius, radius, radius);
            vsg::dvec3 size(radius.x(), radius.y(), radius.z());
            transform->matrix = vsg::translate(pos) * vsg::rotate(angle, rotax) * vsg::scale(size);
        } else if (auto cone = std::dynamic_pointer_cast<ChConeShape>(shape)) {
            ChVector<> radius = cone->GetConeGeometry().rad;
            // ChVector<> size(radius, radius, radius);
            vsg::dvec3 size(radius.x(), radius.y(), radius.z());
            transform->matrix = vsg::translate(pos) * vsg::rotate(angle, rotax) * vsg::scale(size);
        } else if (auto capsule = std::dynamic_pointer_cast<ChCapsuleShape>(shape)) {
            double rad = capsule->GetCapsuleGeometry().rad;
            double height = capsule->GetCapsuleGeometry().hlen;
            ChVector<> scale(rad, height, rad);
            transform->matrix =
                vsg::translate(pos) * vsg::rotate(angle, rotax) * vsg::scale(scale.x(), scale.y(), scale.z());
        } else if (auto obj = std::dynamic_pointer_cast<ChObjFileShape>(shape)) {
            string objFilename = obj->GetFilename();
            transform->matrix = vsg::translate(pos) * vsg::rotate(angle, rotax);
        } else if (auto cylinder = std::dynamic_pointer_cast<ChCylinderShape>(shape)) {
            double rad = cylinder->GetCylinderGeometry().rad;
            const auto& P1 = cylinder->GetCylinderGeometry().p1;
            const auto& P2 = cylinder->GetCylinderGeometry().p2;

            ChVector<> dir = P2 - P1;
            double height = dir.Length();
            dir.Normalize();
            ChVector<> mx, my, mz;
            dir.DirToDxDyDz(my, mz, mx);  // y is axis, in cylinder.obj frame
            ChMatrix33<> R_CS;
            R_CS.Set_A_axis(mx, my, mz);

            auto t_CS = 0.5 * (P2 + P1);
            ChFrame<> X_CS(t_CS, R_CS);
            ChFrame<> X_CA = X_SA * X_CS;

            auto pos = X_CA.GetPos();
            rot = X_CA.GetRot();
            double rotAngle;
            ChVector<> rotAxis;
            rot.Q_to_AngAxis(rotAngle, rotAxis);

            transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                                vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                                vsg::scale(rad, height, rad);
        }
    }
    // Update particles
    for (auto& item : m_systems[0]->Get_otherphysicslist()) {
        if (auto pcloud = std::dynamic_pointer_cast<ChParticleCloud>(item)) {
            auto numParticles = pcloud->GetNparticles();
            std::vector<double> size = pcloud->GetCollisionModel()->GetShapeDimensions(0);
            bool childTest = numParticles == m_particleScene->children.size();
            if (!childTest) {
                GetLog() << "Caution: Ill Shaped Particle Scenegraph! Not Updated.\n";
                GetLog() << "Found Particles = " << numParticles << "\n";
                GetLog() << "Found Children  = " << m_particleScene->children.size() << "\n";
                continue;
            }
            size_t idx = 0;
            for (auto child : m_particleScene->children) {
                vsg::ref_ptr<vsg::MatrixTransform> transform;
                if (!child->getValue("TransformPtr", transform))
                    continue;
                const auto& pos = pcloud->GetVisualModelFrame(idx).GetPos();
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) * vsg::scale(size[0], size[0], size[0]);
                idx++;
            }
        }
    }
    // Update link shapes
    for (auto child : m_linkScene->children) {
        std::shared_ptr<ChLinkBase> item;
        ChVisualModel::ShapeInstance shapeInstance;
        vsg::ref_ptr<vsg::MatrixTransform> transform;
        if (!child->getValue("LinkPtr", item))
            continue;
        if (!child->getValue("ShapeInstancePtr", shapeInstance))
            continue;
        if (!child->getValue("TransformPtr", transform))
            continue;
        if (auto link = std::dynamic_pointer_cast<ChLinkTSDA>(item)) {
            if (!link->GetVisualModel())
                continue;
            auto& shape = shapeInstance.first;
            if (auto segshape = std::dynamic_pointer_cast<ChSegmentShape>(shape)) {
                ChVector<> P1 = link->GetPoint1Abs();
                ChVector<> P2 = link->GetPoint2Abs();
                double rotAngle, height;
                ChVector<> rotAxis, pos;
                Point2PointHelperAbs(P1, P2, height, pos, rotAngle, rotAxis);
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                                    vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                                    vsg::scale(0.0, height, 0.0);
            } else if (auto sprshape = std::dynamic_pointer_cast<ChSpringShape>(shape)) {
                double rad = sprshape->GetRadius();
                ChVector<> P1 = link->GetPoint1Abs();
                ChVector<> P2 = link->GetPoint2Abs();
                double rotAngle, height;
                ChVector<> rotAxis, pos;
                Point2PointHelperAbs(P1, P2, height, pos, rotAngle, rotAxis);
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                                    vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                                    vsg::scale(rad, height, rad);
            }
        } else if (auto link = std::dynamic_pointer_cast<ChLinkDistance>(item)) {
            if (!link->GetVisualModel())
                continue;
            auto& shape = shapeInstance.first;
            if (auto segshape = std::dynamic_pointer_cast<ChSegmentShape>(shape)) {
                ChVector<> P1 = link->GetEndPoint1Abs();
                ChVector<> P2 = link->GetEndPoint2Abs();
                double rotAngle, height;
                ChVector<> rotAxis, pos;
                Point2PointHelperAbs(P1, P2, height, pos, rotAngle, rotAxis);
                transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                                    vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                                    vsg::scale(0.0, height, 0.0);
            }
        }
    }
}

void ChVisualSystemVSG::Point2PointHelperAbs(ChVector<>& P1,
                                             ChVector<>& P2,
                                             double& height,
                                             ChVector<>& pos,
                                             double& rotAngle,
                                             ChVector<>& rotAxis) {
    ChVector<> dir = P2 - P1;
    height = dir.Length();
    dir.Normalize();
    ChVector<> mx, my, mz;
    dir.DirToDxDyDz(my, mz, mx);  // y is axis, in cylinder.obj frame
    ChMatrix33<> R_CS;
    R_CS.Set_A_axis(mx, my, mz);

    auto t_CS = 0.5 * (P2 + P1);
    ChFrame<> X_CS(t_CS, R_CS);

    pos = X_CS.GetPos();
    auto rot = X_CS.GetRot();
    rot.Q_to_AngAxis(rotAngle, rotAxis);
}

void ChVisualSystemVSG::SetDecoGrid(double ustep, double vstep, int nu, int nv, ChCoordsys<> pos, ChColor col) {
    m_decoScene->addChild(m_shapeBuilder->createDecoGrid(ustep, vstep, nu, nv, pos, col));
}

void ChVisualSystemVSG::SetDecoObject(std::string objFileName, ChCoordsys<> csys, ChVector<> scale, ChColor col) {
    auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(objFileName, true, true);
    if (trimesh == nullptr) {
        GetLog() << "Couldn't read file " << objFileName << "\n";
        return;
    }
    ChVector<> pos = csys.pos;
    ChQuaternion<> quat = csys.rot;
    ChVector<> rotAxis;
    double rotAngle;
    quat.Q_to_AngAxis(rotAngle, rotAxis);
    auto transform = vsg::MatrixTransform::create();
    transform->matrix = vsg::translate(pos.x(), pos.y(), pos.z()) *
                        vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z()) *
                        vsg::scale(scale.x(), scale.y(), scale.z());
    auto tms = chrono_types::make_shared<ChTriangleMeshShape>();
    tms->SetMesh(trimesh, true);
    if (tms->GetNumMaterials() > 0) {
        for (int i = 0; i < tms->GetNumMaterials(); i++) {
            tms->GetMaterial(i)->SetDiffuseColor(col);
            tms->GetMaterial(i)->SetAmbientColor(ChColor(0.2, 0.2, 0.2));
        }
        auto tmpGroup = m_shapeBuilder->createTrimeshMatShape(transform, m_draw_as_wireframe, tms);
        m_decoScene->addChild(tmpGroup);
    } else {
        auto tmpGroup = m_shapeBuilder->createTrimeshColShape(transform, m_draw_as_wireframe, tms);
        m_decoScene->addChild(tmpGroup);
    }
}

void ChVisualSystemVSG::SetSystemSymbol(double size) {
    m_system_symbol_size = vsg::dvec3(size, size, size);

    // Is the symbol already present?
    vsg::ref_ptr<vsg::MatrixTransform> transform;
    bool found = false;
    for (auto child : m_symbolScene->children) {
        char sType = ' ';
        if (!child->getValue("SymbolType", sType))
            continue;
        if (!child->getValue("TransformPtr", transform))
            continue;
        if (sType != 'G')
            continue;
        // we only set the size
        transform->matrix = vsg::translate(m_system_symbol_position) * vsg::scale(m_system_symbol_size);
        found = true;
    }
    if (found)
        return;

    // Symbol is not found, build it
    transform = vsg::MatrixTransform::create();
    transform->matrix = vsg::translate(m_system_symbol_position) * vsg::scale(m_system_symbol_size);

    m_symbolScene->addChild(m_shapeBuilder->createMovingSystemSymbol(transform));
}

void ChVisualSystemVSG::SetSystemSymbolPosition(ChVector<> pos) {
    m_system_symbol_position = vsg::dvec3(pos.x(), pos.y(), pos.z());
    for (auto child : m_symbolScene->children) {
        vsg::ref_ptr<vsg::MatrixTransform> transform;
        char sType = ' ';
        if (!child->getValue("SymbolType", sType))
            continue;
        if (!child->getValue("TransformPtr", transform))
            continue;
        if (sType != 'G')
            continue;
        transform->matrix = vsg::translate(m_system_symbol_position) * vsg::scale(m_system_symbol_size);
    }
}

void ChVisualSystemVSG::SetColorBar(std::string title, double min_val, double max_val) {
    m_params->cb_title = title;
    m_params->cb_min = min_val;
    m_params->cb_max = max_val;
    m_params->show_color_bar = true;
}

}  // namespace vsg3d
}  // namespace chrono
