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
// Radu Serban
// =============================================================================

#include <algorithm>

#include "chrono/assets/ChVisualSystem.h"
#include "chrono/utils/ChUtils.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {

ChVisualSystem ::ChVisualSystem() : m_initialized(false), m_background_color(ChColor(0.10f, 0.20f, 0.30f)), m_verbose(false), m_rtf(0), m_write_images(false), m_image_dir(".") {}

ChVisualSystem ::~ChVisualSystem() {
    for (auto s : m_systems)
        if (s->visual_system)
            s->visual_system = nullptr;
}

void ChVisualSystem::AttachSystem(ChSystem* sys) {
    // Attach provided system only if not already done
    if (std::find(m_systems.begin(), m_systems.end(), sys) == m_systems.end()) {
        m_systems.push_back(sys);
        sys->visual_system = this;
    }
}

void ChVisualSystem::SetBackgroundColor(const ChColor& color) {
    m_background_color = color;
}

void ChVisualSystem::UpdateCamera(int id, const ChVector3d& pos, ChVector3d target) {
    SetCameraPosition(id, pos);
    SetCameraTarget(id, target);
}

void ChVisualSystem::UpdateCamera(const ChVector3d& pos, ChVector3d target) {
    SetCameraPosition(pos);
    SetCameraTarget(target);
}

// -----------------------------------------------------------------------------

void ChVisualSystem::Render() {
    static double t_last = 0;
    double t_curr = GetSimulationTime();
    m_timer.stop();
    if (t_curr > t_last)
        m_rtf = m_timer() / (t_curr - t_last);
    t_last = t_curr;
    m_timer.reset();
    m_timer.start();
}

// -----------------------------------------------------------------------------

double ChVisualSystem::GetSimulationRTF(unsigned int i) const {
    if (i >= m_systems.size())
        return 0;

    return m_systems[i]->GetRTF();
}

std::vector<double> ChVisualSystem::GetSimulationRTFs() const {
    std::vector<double> rtf(m_systems.size());
    for (size_t i = 0; i < m_systems.size(); i++)
        rtf[i] = m_systems[i]->GetRTF();
    return rtf;
}

double ChVisualSystem::GetSimulationTime() const {
    if (m_systems.empty())
        return 0;
    return m_systems[0]->GetChTime();
}

unsigned int ChVisualSystem::GetNumBodies() const {
    unsigned int count = 0;
    for (const auto& sys : m_systems)
        count += sys->GetNumBodiesActive();

    return count;
}

unsigned int ChVisualSystem::GetNumLinks() const {
    unsigned int count = 0;
    for (const auto& sys : m_systems)
        count += sys->GetNumLinksActive();

    return count;
}

unsigned int ChVisualSystem::GetNumMeshes() const {
    unsigned int count = 0;
    for (const auto& sys : m_systems)
        count += sys->GetNumMeshes();

    return count;
}

unsigned int ChVisualSystem::GetNumShafts() const {
    unsigned int count = 0;
    for (const auto& sys : m_systems)
        count += sys->GetNumShafts();

    return count;
}

unsigned int ChVisualSystem::GetNumStates() const {
    unsigned int count = 0;
    for (const auto& sys : m_systems)
        count += sys->GetNumCoordsVelLevel();

    return count;
}

unsigned int ChVisualSystem::GetNumConstraints() const {
    unsigned int count = 0;
    for (const auto& sys : m_systems)
        count += sys->GetNumConstraints();

    return count;
}

unsigned int ChVisualSystem::GetNumContacts() const {
    unsigned int count = 0;
    for (const auto& sys : m_systems)
        count += sys->GetNumContacts();

    return count;
}

// -----------------------------------------------------------------------------

ChVisualSystem::Settings::Settings()
    : render(false),
      render_fps(120),
      camera_vertical(CameraVerticalDir::Z),
      camera_location({0, -1, 0}),
      camera_target({0, 0, 0}),
      enable_shadows(true),
      write_images(false),
      image_dir(".") {}

ChVisualSystem::Settings::Settings(const Settings& other) {
    render = other.render;
    render_fps = other.render_fps;
    camera_vertical = other.camera_vertical;
    camera_location = other.camera_location;
    camera_target = other.camera_target;
    enable_shadows = other.enable_shadows;
    write_images = other.write_images;
    image_dir = other.image_dir;
}

ChVisualSystem::Settings& ChVisualSystem::Settings::operator=(const Settings& other) {
    render = other.render;
    render_fps = other.render_fps;
    camera_vertical = other.camera_vertical;
    camera_location = other.camera_location;
    camera_target = other.camera_target;
    enable_shadows = other.enable_shadows;
    write_images = other.write_images;
    image_dir = other.image_dir;
    return *this;
}

#ifdef CHRONO_HAS_YAML

ChVisualSystem::Settings::Settings(const YAML::Node& a) : Settings() {
    render = true;
    if (a["render_fps"])
        render_fps = a["render_fps"].as<double>();
    if (a["enable_shadows"])
        enable_shadows = a["enable_shadows"].as<bool>();
    if (a["camera"]) {
        if (a["camera"]["vertical"]) {
            auto vertical = ChToUpper(a["camera"]["vertical"].as<std::string>());
            if (vertical == "Y")
                camera_vertical = CameraVerticalDir::Y;
            else if (vertical == "Z")
                camera_vertical = CameraVerticalDir::Z;
            else {
                cerr << "Incorrect camera vertical " << a["camera"]["vertical"].as<std::string>() << endl;
                throw std::runtime_error("Incorrect camera vertical");
            }
        }
        if (a["camera"]["location"])
            camera_location = ReadVector(a["camera"]["location"]);
        if (a["camera"]["target"])
            camera_target = ReadVector(a["camera"]["target"]);
    }
    if (a["output"]) {
        auto b = a["output"];
        if (b["save_images"])
            write_images = b["save_images"].as<bool>();
        if (b["output_directory"])
            image_dir = b["output_directory"].as<std::string>();
    }
}

ChVisualSystem::Settings ChVisualSystem::Settings::Read(const YAML::Node& a) {
    Settings params(a);
    return params;
}

#endif

void ChVisualSystem::Settings::PrintInfo() const {
    if (!render) {
        cout << "no run-time visualization" << endl;
        return;
    }

    cout << "run-time visualization" << endl;
    cout << "  render FPS:           " << render_fps << endl;
    cout << "  enable shadows?       " << std::boolalpha << enable_shadows << endl;
    cout << "  camera vertical dir:  " << (camera_vertical == CameraVerticalDir::Y ? "Y" : "Z") << endl;
    cout << "  camera location:      " << camera_location << endl;
    cout << "  camera target:        " << camera_target << endl;
}

}  // namespace chrono
