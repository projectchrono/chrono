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

#include "chrono_precice/ChPreciceAdapterTdpf.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace ch_precice {

ChPreciceAdapterTdpf::ChPreciceAdapterTdpf(const std::string& precice_config_filename, std::shared_ptr<fsi::tdpf::ChFsiFluidSystemTDPF> sysTDPF, double time_step, bool verbose)
    : ChPreciceAdapter(precice_config_filename, "model_TDPF", verbose), m_sysTDPF(sysTDPF) {}

#if defined(CHRONO_PARSERS) && defined(CHRONO_HAS_YAML)
ChPreciceAdapterTdpf::ChPreciceAdapterTdpf(const std::string& precice_config_filename, const std::string& input_filename, bool verbose)
    : ChPreciceAdapter(precice_config_filename, "", verbose) {
    // Create a TDPF YAML parser and the underlying fluid solver, but do not initialize the FSI problem
    parsers::ChParserTdpfYAML parser(input_filename, verbose);
    m_model_name = parser.GetName();
    m_sysFSI = parser.CreateFsiSystemTDPF(false);
    m_sysTDPF = parser.GetFluidSystemTDPF();

    // Extract information from parsed YAML files
    m_output_settings = parser.GetOutputSettings();
    #ifdef CHRONO_VSG
    m_vis_settings = parser.GetVisualizationSettings();
    m_visTDPF_settings = parser.GetTdpfVisualizationSettings();
    #endif

    // Read common preCICE participant configuration (participant name, file handler, and coupling interfaces)
    ReadParticipantConfigurationYAML(input_filename);

    // Read TDPF-specific preCICE configuration (coupling objects)
    YAML::Node yaml = YAML::LoadFile(input_filename);
    ChAssertAlways(yaml["precice_adapter_config"]);
    auto config = yaml["precice_adapter_config"];

    // - read information on coupling bodies and check that they are defined in the MBS
    if (config["bodies"]) {
        auto bodies = config["bodies"];
        ChAssertAlways(bodies.IsSequence());
        for (int i = 0; i < bodies.size(); i++) {
            ChAssertAlways(bodies[i]["name"]);
            auto body_name = bodies[i]["name"].as<std::string>();

            ChAssertAlways(bodies[i]["location"]);
            ChVector3d body_pos = ReadVector(bodies[i]["location"]);
            ChQuaterniond body_rot = QUNIT;
            if (bodies[i]["orientation"])
                body_rot = ReadRotation(bodies[i]["orientation"], m_use_degrees);

            AddCouplingBody(body_name, ChFramed(body_pos, body_rot));
        }
    }

    #ifdef CHRONO_FEA
    // - read information on FEA meshes and check that they are defined in the MBS
    if (config["meshes"]) {
        //// TODO
    }
    #endif

    // Initialize the fluid solver (now that FSI solids are specified)
    // Attention: do *not* call ChFsiSystemTDPF::Initialize() as that attempts to set up added mass for an MBS system!
    m_sysFSI->ChFsiSystem::Initialize();

    if (m_verbose) {
        cout << "\n-------------------------------------------------\n" << endl;
    }
}
#endif

// -----------------------------------------------------------------------------

void ChPreciceAdapterTdpf::AddCouplingBody(const std::string& name, const ChFramed& frame) {
    auto c_body = chrono_types::make_shared<CouplingBody>();
    c_body->index = (int)m_coupling_bodies.size();
    c_body->init_body_frame = frame;
    c_body->points = std::vector<ChVector3d>();
    m_coupling_bodies.push_back(c_body);

    // Create a dummy body
    auto body = chrono_types::make_shared<ChBodyAuxRef>();
    body->SetFrameRefToAbs(frame);

    m_sysFSI->AddFsiBody(body, nullptr, false);
}

// -----------------------------------------------------------------------------

void ChPreciceAdapterTdpf::InitializeParticipant() {
    // For each interface mesh:
    // - check that coupling meshes have dimension 3 (as reported by preCICE)
    // - check correct coupling data type (read and write)
    // - check that coupling data have dimension consistent to the mesh dimension (as reported by preCICE)
    // - set mesh vertices
    // - register mesh with preCICE
    if (m_verbose)
        cout << m_prefix2 << "Check and register coupling meshes" << endl;

    for (const auto& mesh_name : GetCouplingMeshNames()) {
        auto mesh_dim = GetCouplingMeshDimensions(mesh_name);
        ChAssertAlways(mesh_dim == 3);

        if (m_verbose)
            cout << m_prefix2 << "mesh: '" << mesh_name << "'" << endl;

        // Check consistency of mesh dimension and read data dimension
        for (const auto& data_name : GetReadDataNamesOnMesh(mesh_name)) {
            if (!GetCouplingDataUsed(mesh_name, data_name)) {
                if (m_verbose)
                    cout << m_prefix2 << "  skip unreferenced data block `" << data_name << "`" << endl;
                continue;
            }
            if (m_verbose)
                cout << m_prefix2 << "  read data: '" << data_name << "' ... ";
            auto data_type = GetCouplingDataType(mesh_name, data_name);
            auto data_dim = GetCouplingDataDimensions(mesh_name, data_name);
            switch (data_type) {
                case CouplingDataType::POSITIONS:
                case CouplingDataType::ROTATIONS:
                case CouplingDataType::LINEAR_VELOCITIES:
                case CouplingDataType::ANGULAR_VELOCITIES:
                    ChAssertAlways(data_dim == mesh_dim);
                    break;
                case CouplingDataType::DISPLACEMENTS:
                case CouplingDataType::FORCES:
                case CouplingDataType::TORQUES:
                    if (m_verbose)
                        cout << "FAIL" << endl;
                    cerr << "[InitializeParticipant] Invalid Chrono TDPF read data type (" << GetCouplingDataTypeAsString(data_type) << ")" << endl;
                    throw std::runtime_error("Invalid Chrono TDPF read data type");
            }
            if (m_verbose)
                cout << "OK" << endl;
        }

        // Check consistency of mesh dimension and write data dimension
        for (const auto& data_name : GetWriteDataNamesOnMesh(mesh_name)) {
            if (!GetCouplingDataUsed(mesh_name, data_name)) {
                if (m_verbose)
                    cout << m_prefix2 << "  skip unreferenced data block `" << data_name << "`" << endl;
                continue;
            }
            if (m_verbose)
                cout << m_prefix2 << "  write data: '" << data_name << "' ... ";
            auto data_type = GetCouplingDataType(mesh_name, data_name);
            auto data_dim = GetCouplingDataDimensions(mesh_name, data_name);
            switch (data_type) {
                case CouplingDataType::FORCES:
                    // Forces must have the same dimension as the coupling mesh
                    ChAssertAlways(data_dim == mesh_dim);
                    break;
                case CouplingDataType::TORQUES:
                    // Torques must be 3D for a 3D mesh and 1D for a 2D mesh
                    ChAssertAlways((mesh_dim == 3 && data_dim == 3) || (mesh_dim == 2 && data_dim == 1));
                    break;
                case CouplingDataType::POSITIONS:
                case CouplingDataType::ROTATIONS:
                case CouplingDataType::DISPLACEMENTS:
                case CouplingDataType::LINEAR_VELOCITIES:
                case CouplingDataType::ANGULAR_VELOCITIES:
                    if (m_verbose)
                        cout << "FAIL" << endl;
                    cerr << "[InitializeParticipant] Invalid Chrono TDPF write data type (" << GetCouplingDataTypeAsString(data_type) << ")" << endl;
                    throw std::runtime_error("Invalid Chrono TDPF write data type");
            }
            if (m_verbose)
                cout << "OK" << endl;
        }

        // Set mesh vertices, based on mesh type
        std::vector<ChVector3d> vertices;
        auto& mesh_info = m_coupling_meshes[mesh_name];
        switch (mesh_info.type) {
            case CouplingMeshType::RIGID_BODY_REFS: {
                for (const auto& c_body : m_coupling_bodies)
                    vertices.push_back(c_body->init_body_frame.GetPos());
                break;
            }
            case CouplingMeshType::RIGID_BODY_POINTS: {
                for (const auto& c_body : m_coupling_bodies) {
                    ChAssertAlways(!c_body->points.empty());
                    for (const auto& pos_loc : c_body->points) {
                        ChVector3d pos_abs = c_body->init_body_frame.TransformPointLocalToParent(pos_loc);
                        vertices.push_back(pos_abs);
                    }
                }
                break;
            }
            case CouplingMeshType::FEA_MESH_NODES: {
                //// TODO
                ////break;
                throw std::runtime_error("CouplingMeshType::FEA_MESH_NODES not yet implemented");
            }
            case CouplingMeshType::FEA_MESH_POINTS: {
                //// TODO
                ////break;
                throw std::runtime_error("CouplingMeshType::FEA_MESH_POINTS not yet implemented");
            }
        }

        // Register coupling mesh with preCICE, taking into account mesh dimension
        RegisterMesh(mesh_name, vertices);
    }

    //// TODO -- checkpointing...
    ////if (m_verbose)
    ////    cout << m_prefix1 << "Set up checkpointing" << endl;

#ifdef CHRONO_VSG
    // Enable runtime visualization
    if (m_visualize && m_vis_settings.render) {
        if (m_verbose)
            cout << m_prefix1 << "Set up run-time visualization" << endl;

        // TDPF visualization plugin
        auto visFSI = chrono_types::make_shared<fsi::tdpf::ChTdpfVisualizationVSG>(m_sysTDPF.get());
        visFSI->SetWaveMeshVisibility(true);
        visFSI->SetWaveMeshColormap(m_visTDPF_settings.colormap, 0.95f);
        visFSI->SetWaveMeshColorMode(m_visTDPF_settings.mode, m_visTDPF_settings.range);
        visFSI->SetWaveMeshUpdateFrequency(m_visTDPF_settings.update_fps);

        // VSG visual system (attach visFSI as plugin)
        m_vsg = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        m_vsg->AttachPlugin(visFSI);
        m_vsg->SetWindowTitle("Chrono preCICE TDPF participant - " + m_participant_name);
        m_vsg->SetWindowSize(1280, 800);
        m_vsg->SetWindowPosition(100, 100);
        m_vsg->AddCamera(m_vis_settings.camera_location, m_vis_settings.camera_target);
        m_vsg->SetCameraVertical(m_vis_settings.camera_vertical);
        m_vsg->SetCameraAngleDeg(40.0);
        m_vsg->SetLightIntensity(0.9f);
        m_vsg->SetLightDirection(CH_PI_2, CH_PI / 6);
        m_vsg->EnableShadows(false);

        m_vsg->Initialize();
    }
#endif
}

// -----------------------------------------------------------------------------

void ChPreciceAdapterTdpf::WriteCheckpoint(double time) {
    throw std::runtime_error("Checkpointing not available for Chrono::FSI-TDPF");
}

void ChPreciceAdapterTdpf::ReadCheckpoint(double time) {
    throw std::runtime_error("Checkpointing not available for Chrono::FSI-TDPF");
}

// -----------------------------------------------------------------------------

void ChPreciceAdapterTdpf::ReadData() {
    ChPreciceAdapter::ReadData();

    if (m_verbose)
        cout << m_prefix1 << "Process read data" << endl;

    for (const auto& [mesh_name, mesh_info] : m_coupling_meshes) {
        switch (mesh_info.type) {
            case CouplingMeshType::RIGID_BODY_REFS:
                ReadBodyRefData(mesh_name, mesh_info);
                break;
            case CouplingMeshType::RIGID_BODY_POINTS:
                ReadBodyMeshData(mesh_name, mesh_info);
                break;
            case CouplingMeshType::FEA_MESH_NODES:
                //// TODO
                break;
            case CouplingMeshType::FEA_MESH_POINTS:
                //// TODO
                break;
        }
    }
}

void ChPreciceAdapterTdpf::WriteData() {
    if (m_verbose)
        cout << m_prefix1 << "Prepare write data" << endl;

    for (auto& [mesh_name, mesh_info] : m_coupling_meshes) {
        switch (mesh_info.type) {
            case CouplingMeshType::RIGID_BODY_REFS:
                WriteBodyRefData(mesh_name, mesh_info);
                break;
            case CouplingMeshType::RIGID_BODY_POINTS:
                WriteBodyMeshData(mesh_name, mesh_info);
                break;
            case CouplingMeshType::FEA_MESH_NODES:
                //// TODO
                break;
            case CouplingMeshType::FEA_MESH_POINTS:
                //// TODO
                break;
        }
    }

    ChPreciceAdapter::WriteData();
}

void ChPreciceAdapterTdpf::ReadBodyRefData(const std::string& mesh_name, const CouplingMeshInfo& mesh_info) {
    auto mesh_dim = GetCouplingMeshDimensions(mesh_name);

    size_t num_bodies = m_coupling_bodies.size();
    std::vector<fsi::FsiBodyState> body_states(num_bodies);

    // Read data
    for (const auto& data_name : m_data_read[mesh_name]) {
        const auto& data_info = mesh_info.data.at(data_name);
        if (!data_info.used)
            continue;
        auto data_type = data_info.type;
        const auto& data_values = data_info.values;
        auto data_dim = GetCouplingDataDimensions(mesh_name, data_name);
        assert(data_values.size() == data_dim * m_coupling_bodies.size());
        switch (data_type) {
            case CouplingDataType::POSITIONS: {
                assert(data_dim == mesh_dim);
                size_t i_data = 0;
                for (size_t i_body = 0; i_body < num_bodies; i_body++) {
                    auto& bstates = body_states[i_body];
                    bstates.pos.x() = data_values[i_data + 0];
                    bstates.pos.y() = data_values[i_data + 1];
                    bstates.pos.z() = data_values[i_data + 2];
                    i_data += 3;
                }
                break;
            }
            case CouplingDataType::ROTATIONS: {
                assert(data_dim == mesh_dim);
                size_t i_data = 0;
                for (size_t i_body = 0; i_body < num_bodies; i_body++) {
                    auto& bstates = body_states[i_body];
                    ChQuaterniond q1 = QuatFromAngleZ(data_values[i_data + 0]);  // roll
                    ChQuaterniond q2 = QuatFromAngleY(data_values[i_data + 1]);  // pitch
                    ChQuaterniond q3 = QuatFromAngleX(data_values[i_data + 2]);  // yaw
                    bstates.rot = q1 * q2 * q3;
                    i_data += 3;
                }
                break;
            }
            case CouplingDataType::LINEAR_VELOCITIES: {
                assert(data_dim == mesh_dim);
                size_t i_data = 0;
                for (size_t i_body = 0; i_body < num_bodies; i_body++) {
                    auto& bstates = body_states[i_body];
                    bstates.lin_vel.x() = data_values[i_data + 0];
                    bstates.lin_vel.y() = data_values[i_data + 1];
                    bstates.lin_vel.z() = data_values[i_data + 2];
                    i_data += 3;
                }
                break;
            }
            case CouplingDataType::ANGULAR_VELOCITIES: {
                assert(data_dim == mesh_dim);
                size_t i_data = 0;
                for (size_t i_body = 0; i_body < num_bodies; i_body++) {
                    auto& bstates = body_states[i_body];
                    bstates.ang_vel.x() = data_values[i_data + 0];
                    bstates.ang_vel.y() = data_values[i_data + 1];
                    bstates.ang_vel.z() = data_values[i_data + 2];
                    i_data += 3;
                }
                break;
            }
            default:
                cerr << "[ReadBodyRefData] Invalid Chrono TDPF read data type (" << GetCouplingDataTypeAsString(data_type) << ")" << endl;
                throw std::runtime_error("Invalid Chrono TDPF read data type");
        }
    }

    // Pass the body states to the TDPF solver
    m_sysTDPF->LoadSolidStates(body_states);
    m_sysTDPF->OnExchangeSolidStates();
}

void ChPreciceAdapterTdpf::WriteBodyRefData(const std::string& mesh_name, CouplingMeshInfo& mesh_info) {
    auto mesh_dim = GetCouplingMeshDimensions(mesh_name);

    size_t num_bodies = m_coupling_bodies.size();
    std::vector<fsi::FsiBodyForce> body_forces(num_bodies);

    // Get the body forces from the TDPF solver
    m_sysTDPF->OnExchangeSolidForces();
    m_sysTDPF->StoreSolidForces(body_forces);

    // Write data
    for (const auto& data_name : m_data_write[mesh_name]) {
        auto& data_info = mesh_info.data[data_name];
        if (!data_info.used)
            continue;
        auto data_type = data_info.type;
        auto& data_values = data_info.values;
        auto data_dim = GetCouplingDataDimensions(mesh_name, data_name);
        assert(data_values.size() == data_dim * m_coupling_bodies.size());
        switch (data_type) {
            case CouplingDataType::FORCES: {
                assert(data_dim == mesh_dim);
                size_t i_data = 0;
                for (size_t i_body = 0; i_body < num_bodies; i_body++) {
                    const auto& bforces = body_forces[i_body];
                    data_values[i_data + 0] = bforces.force.x();
                    data_values[i_data + 1] = bforces.force.y();
                    data_values[i_data + 2] = bforces.force.z();
                    i_data += 3;
                }
                break;
            }
            case CouplingDataType::TORQUES: {
                assert(data_dim == mesh_dim);
                size_t i_data = 0;
                for (size_t i_body = 0; i_body < num_bodies; i_body++) {
                    const auto& bforces = body_forces[i_body];
                    data_values[i_data + 0] = bforces.torque.x();
                    data_values[i_data + 1] = bforces.torque.y();
                    data_values[i_data + 2] = bforces.torque.z();
                    i_data += 3;
                }
                break;
            }
            default:
                cerr << "[ReadBodyRefData] Invalid Chrono TDPF write data type (" << GetCouplingDataTypeAsString(data_type) << ")" << endl;
                throw std::runtime_error("Invalid Chrono TDPF write data type");
        }
    }
}

void ChPreciceAdapterTdpf::ReadBodyMeshData(const std::string& mesh_name, const CouplingMeshInfo& mesh_info) {
    //// TODO
    throw std::runtime_error("CouplingMeshType::RIGID_BODY_MESH_POINTS not yet implemented");
}

void ChPreciceAdapterTdpf::WriteBodyMeshData(const std::string& mesh_name, CouplingMeshInfo& mesh_info) {
    //// TODO
    throw std::runtime_error("CouplingMeshType::RIGID_BODY_MESH_POINTS not yet implemented");
}

// -----------------------------------------------------------------------------

double ChPreciceAdapterTdpf::GetSolverTimeStep(double max_time_step) const {
    return max_time_step;
}

void ChPreciceAdapterTdpf::AdvanceParticipant(double time, double time_step) {
    ChAssertAlways(time == m_sysTDPF->GetSimTime());

    // Generate output (if enabled)
    Output(time);

    // Render (if enabled)
    Render(time);

    // Advance system dynamics
    m_sysTDPF->DoStepDynamics(time_step);
}

// -----------------------------------------------------------------------------

void ChPreciceAdapterTdpf::WriteOutput(int frame, double time) {
    // Invoke first the base class function, to create the output DB if needed
    ChPreciceAdapter::WriteOutput(frame, time);

    //// TODO
}

}  // namespace ch_precice
}  // namespace chrono
