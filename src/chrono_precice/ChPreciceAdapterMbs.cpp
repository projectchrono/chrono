// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include <algorithm>

#include "chrono/utils/ChUtils.h"

#include "chrono_precice/ChPreciceAdapterMbs.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace ch_precice {

ChPreciceAdapterMbs::ChPreciceAdapterMbs(std::shared_ptr<ChSystem> sys, double time_step, bool verbose) : m_sys(sys), m_time_step(time_step) {
    SetVerbose(verbose);
}

#if defined(CHRONO_PARSERS) && defined(CHRONO_HAS_YAML)
ChPreciceAdapterMbs::ChPreciceAdapterMbs(const std::string& input_filename, bool verbose) {
    SetVerbose(verbose);

    // Create the MBS from the YAML specification file
    parsers::ChParserMbsYAML parser(input_filename, verbose);
    m_sys = parser.CreateSystem();
    m_time_step = parser.GetTimestep();
    parser.Populate(*m_sys);

    // Extract information from parsed YAML files
    m_vis.render = parser.Render();
    m_vis.render_fps = parser.GetRenderFPS();
    m_vis.camera_vertical = parser.GetCameraVerticalDir();
    m_vis.camera_location = parser.GetCameraLocation();
    m_vis.camera_target = parser.GetCameraTarget();
    m_vis.enable_shadows = parser.EnableShadows();

    // Read in preCICE participant configuration
    ConstructSolver(input_filename);

    // Check consistency between preCICE participant specification and the MBS
    YAML::Node yaml = YAML::LoadFile(input_filename);
    ChAssertAlways(yaml["precice_adapter_config"]);
    auto config = yaml["precice_adapter_config"];

    // Read names of interface physics items and check that they are defined in the MBS
    if (config["bodies"]) {
        auto bodies = config["bodies"];
        ChAssertAlways(bodies.IsSequence());
        for (int i = 0; i < bodies.size(); i++) {
            ChAssertAlways(bodies[i]["name"]);
            auto body_name = bodies[i]["name"].as<std::string>();
            auto body = parser.FindBodyByName(body_name);
            if (body) {
                AddCouplingBody(body);
            } else {
                cerr << "No body named '" << body_name << "' was found in the MBS" << endl;
                throw std::runtime_error("Interface body not present in MBS");
            }
        }
    }

    if (config["meshes1d"]) {
        //// TODO
    }

    if (config["meshes2d"]) {
        //// TODO
    }

    if (m_verbose) {
    }
}
#endif

ChPreciceAdapterMbs::~ChPreciceAdapterMbs() {}

void ChPreciceAdapterMbs::AddCouplingBody(std::shared_ptr<ChBodyAuxRef> body) {
    auto c_body = chrono_types::make_shared<CouplingBody>();
    c_body->index = (int)m_coupling_bodies.size();
    c_body->body = body;
    c_body->accumulator_index = body->AddAccumulator();
    m_coupling_bodies.push_back(c_body);
}

// -----------------------------------------------------------------------------

ChPreciceAdapterMbs::VisParams::VisParams()
    : render(false), render_fps(120), camera_vertical(CameraVerticalDir::Z), camera_location({0, -1, 0}), camera_target({0, 0, 0}), enable_shadows(true) {}

bool ChPreciceAdapterMbs::EnableVisualization(double render_fps,
                                              CameraVerticalDir camera_vertical,
                                              const ChVector3d& camera_location,
                                              const ChVector3d& camera_target,
                                              bool enable_shadows) {
#ifdef CHRONO_VSG
    m_vis.render_fps = render_fps;
    m_vis.camera_vertical = camera_vertical;
    m_vis.camera_location = camera_location;
    m_vis.camera_target = camera_target;
    m_vis.enable_shadows = enable_shadows;
    m_vis.render = true;
    return true;
#else
    cerr << "Chrono::VSG not enabled. Disabling run-time visualization" << endl;
    m_vis.render = false;
    return false;
#endif
}

void ChPreciceAdapterMbs::InitializeParticipant() {
    ChPreciceAdapter::InitializeParticipant();

    // Go through all interface meshes and
    // - check that coupling meshes have dimension 3 (as reported by preCICE)
    // - check that coupling data have dimension 3 (as reported by preCICE)
    // - set mesh vertices (depending on data type)
    // - register mesh with preCICE
    for (const auto& mesh_name : GetMeshNames()) {
        ChAssertAlways(GetMeshDimensions(mesh_name) == 3);

        for (const auto& data_name : GetReadDataNamesOnMesh(mesh_name)) {
            ChAssertAlways(GetDataDimensions(mesh_name, data_name) == 3);
        }
        for (const auto& data_name : GetWriteDataNamesOnMesh(mesh_name)) {
            ChAssertAlways(GetDataDimensions(mesh_name, data_name) == 3);
        }

        std::vector<ChVector3d> vertices;
        auto& mesh_info = m_coupling_meshes[mesh_name];
        switch (mesh_info.type) {
            case MeshType::RIGID_BODY_REF_POINTS: {
                for (const auto& c_body : m_coupling_bodies)
                    vertices.push_back(c_body->body->GetFrameRefToAbs().GetPos());
                break;
            }
            case MeshType::RIGID_BODY_MESH_POINTS: {
                ////break;
                throw std::runtime_error("MeshType::RIGID_BODY_MESH_POINTS not yet implemented");
            }
            case MeshType::FEA_MESH1D_NODES: {
                ////break;
                throw std::runtime_error("MeshType::FEA_MESH1D_NODES not yet implemented");
            }
            case MeshType::FEA_MESH2D_NODES: {
                ////break;
                throw std::runtime_error("MeshType::FEA_MESH2D_NODES not yet implemented");
            }
        }

        // Register coupling mesh with preCICE
        RegisterMesh(mesh_name, vertices);
    }

    // Allocate space for checkpoint
    m_sys->Setup();
    auto np = m_sys->GetNumCoordsPosLevel();
    auto nv = m_sys->GetNumCoordsVelLevel();
    m_checkpoint.time = m_sys->GetChTime();
    m_checkpoint.x.setZero(np, m_sys.get());
    m_checkpoint.v.setZero(nv, m_sys.get());

#ifdef CHRONO_VSG
    // Enable runtime visualization
    if (m_vis.render) {
        m_vsg = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        m_vsg->AttachSystem(m_sys.get());
        m_vsg->SetWindowTitle("Chrono preCICE MBS participant - " + m_participant_name);
        m_vsg->AddCamera(m_vis.camera_location, m_vis.camera_target);
        m_vsg->SetWindowSize(1280, 800);
        m_vsg->SetWindowPosition(100, 100);
        m_vsg->SetCameraVertical(m_vis.camera_vertical);
        m_vsg->SetCameraAngleDeg(40.0);
        m_vsg->SetLightIntensity(1.0f);
        m_vsg->SetLightDirection(-CH_PI_4, CH_PI_4);
        m_vsg->EnableShadows(m_vis.enable_shadows);
        m_vsg->ToggleAbsFrameVisibility();
        m_vsg->Initialize();
    }
#endif
}

void ChPreciceAdapterMbs::WriteCheckpoint(double time) {
    ChPreciceAdapter::WriteCheckpoint(time);

    double sys_time;
    m_sys->StateGather(m_checkpoint.x, m_checkpoint.v, sys_time);
    assert(time == sys_time);
    m_checkpoint.time = time;
}

void ChPreciceAdapterMbs::ReadCheckpoint(double time) {
    ChPreciceAdapter::ReadCheckpoint(time);

    ChAssertAlways(m_checkpoint.time == time);
    m_sys->StateScatter(m_checkpoint.x, m_checkpoint.v, m_checkpoint.time, UpdateFlags::UPDATE_ALL);
}

void ChPreciceAdapterMbs::ReadData() {
    ChPreciceAdapter::ReadData();

    for (auto& c_body : m_coupling_bodies) {
        c_body->body->EmptyAccumulator(c_body->accumulator_index);
    }

    for (const auto& [mesh_name, mesh_info] : m_coupling_meshes) {
        switch (mesh_info.type) {
            case MeshType::RIGID_BODY_REF_POINTS:
                ReadBodyRefData(mesh_name, mesh_info);
                break;
            case MeshType::RIGID_BODY_MESH_POINTS:
                //// TODO
                break;
            case MeshType::FEA_MESH1D_NODES:
                //// TODO
                break;
            case MeshType::FEA_MESH2D_NODES:
                //// TODO
                break;
        }
    }
}

double ChPreciceAdapterMbs::GetSolverTimeStep(double max_time_step) const {
    return std::min(m_time_step, max_time_step);
}

void ChPreciceAdapterMbs::AdvanceParticipant(double time, double time_step) {
    ChPreciceAdapter::AdvanceParticipant(time, time_step);

    ChAssertAlways(time == m_sys->GetChTime());

    static int render_frame = 0;
    if (m_vis.render && m_vsg->Run()) {
        if (time >= render_frame / m_vis.render_fps) {
            m_vsg->Render();
            render_frame++;
        }
    }

    m_sys->DoStepDynamics(time_step);
}

void ChPreciceAdapterMbs::WriteData() {
    for (auto& [mesh_name, mesh_info] : m_coupling_meshes) {
        switch (mesh_info.type) {
            case MeshType::RIGID_BODY_REF_POINTS:
                WriteBodyRefData(mesh_name, mesh_info);
                break;
            case MeshType::RIGID_BODY_MESH_POINTS:
                //// TODO
                break;
            case MeshType::FEA_MESH1D_NODES:
                //// TODO
                break;
            case MeshType::FEA_MESH2D_NODES:
                //// TODO
                break;
        }
    }

    ChPreciceAdapter::WriteData();
}

// -----------------------------------------------------------------------------

void ChPreciceAdapterMbs::ReadBodyRefData(const std::string& mesh_name, const MeshInfo& mesh_info) {
    for (const auto& data_name : m_data_read[mesh_name]) {
        const auto& data_info = mesh_info.data.at(data_name);
        auto data_type = data_info.type;
        const auto& data_values = data_info.values;
        switch (data_type) {
            case DataType::FORCES:
                assert(data_values.size() == 3 * m_coupling_bodies.size());
                for (size_t i = 0; i < m_coupling_bodies.size(); i++) {
                    auto& c_body = m_coupling_bodies[i];
                    ChVector3d force;
                    force.x() = data_values[3 * i + 0];
                    force.y() = data_values[3 * i + 1];
                    force.z() = data_values[3 * i + 2];
                    c_body->body->AccumulateForce(c_body->accumulator_index, force, c_body->body->GetFrameRefToAbs().GetPos(), false);
                    if (m_verbose)
                        cout << m_prefix2 << "body: " << c_body->body->GetName() << " | force:  " << force << endl;
                }
                break;
            case DataType::TORQUES:
                assert(data_values.size() == 3 * m_coupling_bodies.size());
                for (size_t i = 0; i < m_coupling_bodies.size(); i++) {
                    auto& c_body = m_coupling_bodies[i];
                    ChVector3d torque;
                    torque.x() = data_values[3 * i + 0];
                    torque.y() = data_values[3 * i + 1];
                    torque.z() = data_values[3 * i + 2];
                    c_body->body->AccumulateTorque(c_body->accumulator_index, torque, false);
                    if (m_verbose)
                        cout << m_prefix2 << "body: " << c_body->body->GetName() << " | torque: " << torque << endl;
                }
                break;
            default:
                throw std::runtime_error("Invalid read data type for MBS");
        }
    }
}

void ChPreciceAdapterMbs::WriteBodyRefData(const std::string& mesh_name, MeshInfo& mesh_info) {
    for (const auto& data_name : m_data_write[mesh_name]) {
        auto& data_info = mesh_info.data[data_name];
        auto data_type = data_info.type;
        auto& data_values = data_info.values;
        switch (data_type) {
            case DataType::POSITIONS:
                assert(data_values.size() == 3 * m_coupling_bodies.size());
                for (size_t i = 0; i < m_coupling_bodies.size(); i++) {
                    auto& c_body = m_coupling_bodies[i];
                    const auto& pos = c_body->body->GetFrameRefToAbs().GetPos();
                    data_values[3 * i + 0] = pos.x();
                    data_values[3 * i + 1] = pos.y();
                    data_values[3 * i + 2] = pos.z();
                    if (m_verbose)
                        cout << m_prefix2 << "body: " << c_body->body->GetName() << " | pos:  " << pos << endl;
                }
                break;
            case DataType::VELOCITIES:
                assert(data_values.size() == 3 * m_coupling_bodies.size());
                for (size_t i = 0; i < m_coupling_bodies.size(); i++) {
                    auto& c_body = m_coupling_bodies[i];
                    const auto& vel = c_body->body->GetFrameRefToAbs().GetPosDt();
                    data_values[3 * i + 0] = vel.x();
                    data_values[3 * i + 1] = vel.y();
                    data_values[3 * i + 2] = vel.z();
                    if (m_verbose)
                        cout << m_prefix2 << "body: " << c_body->body->GetName() << " | vel:  " << vel << endl;
                }
                break;
            default:
                throw std::runtime_error("Invalid write data type for MBS");
        }
    }
}

}  // end namespace ch_precice
}  // namespace chrono
