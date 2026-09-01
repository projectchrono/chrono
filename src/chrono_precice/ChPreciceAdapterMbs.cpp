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

#include "chrono_precice/ChPreciceAdapterMbs.h"

#ifdef CHRONO_HAS_HDF5
    #include "chrono/input_output/ChUtilsHDF5.h"
#endif

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace ch_precice {

ChPreciceAdapterMbs::ChPreciceAdapterMbs(const std::string& precice_config_filename, std::shared_ptr<ChSystem> sys, double time_step, bool verbose)
    : ChPreciceAdapter(precice_config_filename, "model_MBS", verbose), m_sys(sys), m_time_step(time_step), m_enforce_realtime(false), m_has_added_mass(false) {}

#if defined(CHRONO_PARSERS) && defined(CHRONO_HAS_YAML)

ChPreciceAdapterMbs::ChPreciceAdapterMbs(const std::string& precice_config_filename, const std::string& input_filename, bool verbose)
    : ChPreciceAdapter(precice_config_filename, "", verbose), m_has_added_mass(false) {
    // Create the MBS from the YAML specification file
    parsers::ChParserMbsYAML parser(input_filename, m_verbose);
    m_model_name = parser.GetName();
    m_sys = parser.CreateSystem();
    m_time_step = parser.GetTimestep();
    parser.Populate(*m_sys);

    // Extract information from parsed YAML files
    m_enforce_realtime = parser.EnforceRealtime();
    m_output_settings = parser.GetOutputSettings();
    #ifdef CHRONO_VSG
    m_vis_settings = parser.GetVisualizationSettings();
    #endif

    // Read common preCICE participant configuration (participant name, file handler, and coupling interfaces)
    ReadParticipantConfigurationYAML(input_filename);

    // Read MBS-specific preCICE configuration (coupling physics items)
    YAML::Node yaml = YAML::LoadFile(input_filename);
    ChAssertAlways(yaml["precice_adapter_config"]);
    auto config = yaml["precice_adapter_config"];

    // - read information on coupling bodies and check that they are defined in the MBS
    if (config["bodies"])
        LoadBodiesYAML(config["bodies"], parser);

    #ifdef CHRONO_FEA
    // - read information on FEA meshes and check that they are defined in the MBS
    if (config["meshes"])
        LoadMeshesYAML(config["meshes"], parser);
    #endif

    // - if expected, look for specification of added mass blocks
    if (m_use_added_mass) {
        if (config["added_mass"]) {
            LoadAddedMassYAML(config["added_mass"], parser);
            m_has_added_mass = true;
        } else {
            cerr << "Added mass enabled, but no YAML specification provided." << endl;
            throw std::runtime_error("Added mass enabled, but no YAML specification provided");
        }
    }

    if (m_verbose) {
        cout << "\n-------------------------------------------------\n" << endl;
    }
}

void ChPreciceAdapterMbs::LoadBodiesYAML(const YAML::Node& bodies, const parsers::ChParserMbsYAML& parser) {
    ChAssertAlways(bodies.IsSequence());
    for (int i = 0; i < bodies.size(); i++) {
        ChAssertAlways(bodies[i]["name"]);
        auto body_name = bodies[i]["name"].as<std::string>();
        auto body = parser.FindBodyByName(body_name);
        if (!body) {
            cerr << "No body named '" << body_name << "' was found in the MBS" << endl;
            throw std::runtime_error("Interface body not present in MBS");
        }
        if (bodies[i]["points"]) {
            auto points_file = bodies[i]["points"].as<std::string>();
            auto points_ext = std::filesystem::path(points_file).extension().string();
            if (points_ext == ".obj" || points_ext == ".OBJ") {
                auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(m_file_handler.GetFilename(points_file), false, false);
                AddCouplingBody(body, mesh->GetCoordsVertices());
            } else if (points_ext == ".stl" || points_ext == ".STL") {
                auto mesh = ChTriangleMeshConnected::CreateFromSTLFile(m_file_handler.GetFilename(points_file), false);
                AddCouplingBody(body, mesh->GetCoordsVertices());
            } else {
                auto points = ReadPoints(m_file_handler.GetFilename(points_file));
                AddCouplingBody(body, points);
            }
        } else {
            AddCouplingBody(body, std::vector<ChVector3d>());
        }
    }
}

void ChPreciceAdapterMbs::LoadMeshesYAML(const YAML::Node& meshes, const parsers::ChParserMbsYAML& parser) {
    #ifdef CHRONO_FEA
    ChAssertAlways(meshes.IsSequence());
    //// TODO
    #endif
}

void ChPreciceAdapterMbs::LoadAddedMassYAML(const YAML::Node& added_mass, const parsers::ChParserMbsYAML& parser) {
    if (added_mass["h5_filename"]) {
        auto h5_filename = added_mass["h5_filename"].as<std::string>();
        SetAddedMassBlocks(h5_filename);
        return;
    }

    if (added_mass["blocks"]) {
        ChAssertAlways(added_mass["density"]);
        auto rho = added_mass["density"].as<double>();
        auto blocks = added_mass["blocks"];
        ChAssertAlways(blocks.IsSequence());
        auto num_bodies = m_coupling_bodies.size();
        ChAssertAlways(blocks.size() == num_bodies);
        if (m_verbose)
            cout << "Read added mass blocks" << endl;
        for (size_t ib = 0; ib < num_bodies; ib++) {
            ChAssertAlways(blocks[ib]["name"]);
            auto body_name = blocks[ib]["name"].as<std::string>();
            ChAssertAlways(body_name == m_coupling_bodies[ib]->body->GetName());
            ChAssertAlways(blocks[ib]["data"]);
            auto data = blocks[ib]["data"];
            ChAssertAlways(data.IsSequence());
            ChAssertAlways(data.size() == 6);
            ChMatrixDynamic<> M(6, num_bodies * 6);
            for (int i = 0; i < data.size(); i++) {
                ChAssertAlways(data[i].IsSequence());
                ChAssertAlways(data[i].size() == 6 * num_bodies);
                for (int j = 0; j < data[i].size(); j++)
                    M(i, j) = data[i][j].as<double>();
            }
            M *= rho;
            m_added_mass_blocks.push_back(M);
            if (m_verbose) {
                cout << "- body '" << body_name << "' - read " << M.rows() << "x" << M.cols() << " block" << endl;
                cout << M << endl;
            }
        }
        return;
    }

    cerr << "Added mass enabled, but no valid YAML specification provided." << endl;
    throw std::runtime_error("Added mass enabled, but no valid YAML specification provided");
}

#endif

// -----------------------------------------------------------------------------

void ChPreciceAdapterMbs::AddCouplingBody(std::shared_ptr<ChBodyAuxRef> body, const std::vector<ChVector3d>& points) {
    auto c_body = chrono_types::make_shared<CouplingBody>();
    c_body->index = (int)m_coupling_bodies.size();
    c_body->body = body;
    c_body->points = points;
    c_body->init_body_frame = body->GetFrameRefToAbs();
    c_body->accumulator_index = body->AddAccumulator();
    m_coupling_bodies.push_back(c_body);

    m_output_data.bodies.push_back(body);
}

#ifdef CHRONO_FEA
void ChPreciceAdapterMbs::AddCouplingFEAMesh(std::shared_ptr<fea::ChMesh> fea_mesh) {
    //// TODO
    ////m_output_data.meshes.push_back(fea_mesh);
}
#endif

void ChPreciceAdapterMbs::SetAddedMassBlocks(const std::string& h5_filename) {
    if (!m_use_added_mass) {
        if (m_verbose)
            cout << m_prefix1 << "No added mass requested via the preCICE configuration file. Ignoring." << endl;
        return;
    }

#ifdef CHRONO_HAS_HDF5
    try {
        H5::H5File h5_file(m_file_handler.GetFilename(h5_filename), H5F_ACC_RDONLY);
        auto rho = ReadDouble(h5_file, "simulation_parameters/rho");
        if (m_verbose)
            cout << "Read added mass blocks" << endl;
        auto num_bodies = m_coupling_bodies.size();
        for (size_t i = 0; i < num_bodies; i++) {
            auto data_name = "body" + std::to_string(i + 1) + "/hydro_coeffs/added_mass/inf_freq";
            auto M = ReadMatrix(h5_file, data_name);
            M *= rho;
            m_added_mass_blocks.push_back(M);
            if (m_verbose) {
                cout << "- body " << i + 1 << " - read " << M.rows() << "x" << M.cols() << " block" << endl;
                cout << M << endl;
            }
        }
    } catch (const H5::Exception& e) {
        cerr << "Unable to open/read HDF5 file." << endl;
        cerr << "  HDF5 error: " << e.getDetailMsg() << endl;
        throw std::runtime_error("Unable to open/read HDF5 file");
    }
    m_has_added_mass = true;
#else
    cerr << "No HDF5 support enabled. Cannot read added mass information from HDF5 file." << endl;
    throw std::runtime_error("No HDF5 support enabled. Cannot read added mass information from HDF5 file");
#endif
}

void ChPreciceAdapterMbs::SetAddedMassBlocks(const std::vector<ChMatrixDynamic<>> blocks) {
    if (!m_use_added_mass) {
        if (m_verbose)
            cout << m_prefix1 << "No added mass requested via the preCICE configuration file. Ignoring." << endl;
        return;
    }
    m_added_mass_blocks = blocks;
    m_has_added_mass = true;
}

// -----------------------------------------------------------------------------

size_t ChPreciceAdapterMbs::GetNumFsiBodies() const {
    return m_coupling_bodies.size();
}

void ChPreciceAdapterMbs::InitializeParticipant() {
    // For each interface mesh:
    // - check that coupling meshes have dimension 2 or 3 (as reported by preCICE)
    // - check that coupling data have dimension consistent with the mesh dimension (as reported by preCICE)
    // - set mesh vertices (depending on mesh type and dimension)
    // - register mesh with preCICE
    if (m_verbose)
        cout << m_prefix1 << "Check and register coupling meshes" << endl;

    for (const auto& mesh_name : GetCouplingMeshNames()) {
        auto mesh_dim = GetCouplingMeshDimensions(mesh_name);
        ChAssertAlways(mesh_dim == 3 || mesh_dim == 2);
        auto& mesh_info = m_coupling_meshes[mesh_name];

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
                    cerr << "\nERROR: Invalid Chrono MBS read data type (" << GetCouplingDataTypeAsString(data_type) << ")" << endl;
                    throw std::runtime_error("Invalid Chrono MBS read data type");
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
                case CouplingDataType::POSITIONS:
                case CouplingDataType::DISPLACEMENTS:
                case CouplingDataType::LINEAR_VELOCITIES:
                    // Positions, displacement, and velocities must have the same dimension as the coupling mesh
                    ChAssertAlways(data_dim == mesh_dim);
                    break;
                case CouplingDataType::ROTATIONS:
                case CouplingDataType::ANGULAR_VELOCITIES:
                    if (mesh_info.type == CouplingMeshType::RIGID_BODY_REFS) {
                        ChAssertAlways((mesh_dim == 3 && data_dim == 3) || (mesh_dim == 2 && data_dim == 1));
                    } else {
                        if (m_verbose)
                            cout << "FAIL" << endl;
                        cerr << "\nERROR: Invalid Chrono MBS write data type (" << GetCouplingDataTypeAsString(data_type) << ")" << endl;
                        throw std::runtime_error("Invalid Chrono MBS write data type");
                    }
                    break;
                case CouplingDataType::FORCES:
                case CouplingDataType::TORQUES:
                    if (m_verbose)
                        cout << "FAIL" << endl;
                    cerr << "\nERROR: Invalid Chrono MBS write data type (" << GetCouplingDataTypeAsString(data_type) << ")" << endl;
                    throw std::runtime_error("Invalid Chrono MBS write data type");
            }
            if (m_verbose)
                cout << "OK" << endl;
        }

        // Set mesh vertices, based on mesh type
        std::vector<ChVector3d> vertices;
        switch (mesh_info.type) {
            case CouplingMeshType::RIGID_BODY_REFS: {
                for (const auto& c_body : m_coupling_bodies)
                    vertices.push_back(c_body->body->GetFrameRefToAbs().GetPos());
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

    // Handle added mass info (applied via a Chrono ChLoadHydrodynamics)
    auto num_bodies = m_coupling_bodies.size();

    // 1. if using dynamic added mass but the participant does not provide the necessary information,
    //    assume zero "static" added mass blocks and allocate space for the updates communicated via preCICE.
    if (m_use_dynamic_added_mass && !m_has_added_mass) {
        m_added_mass_blocks.resize(num_bodies);
        for (auto& block : m_added_mass_blocks)
            block.setZero(6, 6 * num_bodies);
        m_has_added_mass = true;
    }

    // 2. if we have added mass blocks, create the ChHydrodynamicsLoad object
    if (m_has_added_mass) {
        ChAssertAlways(m_added_mass_blocks.size() == num_bodies);
        ChBodyAddedMassBlocks body_blocks;
        for (size_t i = 0; i < num_bodies; i++) {
            body_blocks.push_back({m_coupling_bodies[i]->body, m_added_mass_blocks[i]});
        }
        m_hydro_load = chrono_types::make_shared<ChLoadHydrodynamics>(body_blocks);
        m_hydro_load->SetVerbose(m_verbose);
        m_sys->Add(m_hydro_load);
    }

    // Allocate space for checkpoint
    if (m_verbose)
        cout << m_prefix1 << "Set up checkpointing" << endl;

    m_sys->Setup();
    auto np = m_sys->GetNumCoordsPosLevel();
    auto nv = m_sys->GetNumCoordsVelLevel();
    m_checkpoint.time = m_sys->GetChTime();
    m_checkpoint.x.setZero(np, m_sys.get());
    m_checkpoint.v.setZero(nv, m_sys.get());

#ifdef CHRONO_VSG
    // Enable runtime visualization
    if (m_visualize && m_vis_settings.render) {
        if (m_verbose)
            cout << m_prefix1 << "Set up run-time visualization" << endl;

        m_vsg = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        m_vsg->AttachSystem(m_sys.get());
        m_vsg->SetWindowTitle("Chrono preCICE MBS participant - " + m_participant_name);
        m_vsg->AddCamera(m_vis_settings.camera_location, m_vis_settings.camera_target);
        m_vsg->SetWindowSize(1280, 800);
        m_vsg->SetWindowPosition(100, 100);
        m_vsg->SetCameraVertical(m_vis_settings.camera_vertical);
        m_vsg->SetCameraAngleDeg(40.0);
        m_vsg->SetLightIntensity(1.0f);
        m_vsg->SetLightDirection(-CH_PI_4, CH_PI_4);
        m_vsg->EnableShadows(m_vis_settings.enable_shadows);
        m_vsg->ToggleAbsFrameVisibility();
        m_vsg->Initialize();
    }
#endif
}

// -----------------------------------------------------------------------------

void ChPreciceAdapterMbs::OnWriteCheckpoint(double time) {
    double sys_time;
    m_sys->StateGather(m_checkpoint.x, m_checkpoint.v, sys_time);
    assert(time == sys_time);
    m_checkpoint.time = time;
}

void ChPreciceAdapterMbs::OnReadCheckpoint(double time) {
    ChAssertAlways(m_checkpoint.time == time);
    m_sys->StateScatter(m_checkpoint.x, m_checkpoint.v, m_checkpoint.time, UpdateFlags::UPDATE_ALL);
}

// -----------------------------------------------------------------------------

void ChPreciceAdapterMbs::OnReadData() {
    for (auto& c_body : m_coupling_bodies) {
        c_body->body->EmptyAccumulator(c_body->accumulator_index);
    }

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

void ChPreciceAdapterMbs::OnWriteData() {
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
}

void ChPreciceAdapterMbs::ReadBodyRefData(const std::string& mesh_name, const CouplingMeshInfo& mesh_info) {
    auto mesh_dim = GetCouplingMeshDimensions(mesh_name);

    for (const auto& data_name : m_data_read[mesh_name]) {
        const auto& data_info = mesh_info.data.at(data_name);
        if (!data_info.used)
            continue;
        auto data_type = data_info.type;
        const auto& data_values = data_info.values;
        auto data_dim = GetCouplingDataDimensions(mesh_name, data_name);
        assert(data_values.size() == data_dim * m_coupling_bodies.size());
        switch (data_type) {
            case CouplingDataType::FORCES: {
                assert(data_dim == mesh_dim);
                size_t i_data = 0;
                for (auto& c_body : m_coupling_bodies) {
                    ChVector3d force_abs;
                    if (data_dim == 2) {
                        force_abs.x() = data_values[i_data + 0];
                        force_abs.y() = data_values[i_data + 1];
                        force_abs.z() = 0;
                        i_data += 2;
                    } else {
                        force_abs.x() = data_values[i_data + 0];
                        force_abs.y() = data_values[i_data + 1];
                        force_abs.z() = data_values[i_data + 2];
                        i_data += 3;
                    }
                    c_body->body->AccumulateForce(c_body->accumulator_index, force_abs, c_body->body->GetFrameRefToAbs().GetPos(), false);
                    if (m_verbose)
                        cout << m_prefix2 << "body: " << c_body->body->GetName() << " | force:  " << force_abs << endl;
                }
                break;
            }
            case CouplingDataType::TORQUES: {
                assert((mesh_dim == 3 && data_dim == 3) || (mesh_dim == 2 && data_dim == 1));
                size_t i_data = 0;
                for (auto& c_body : m_coupling_bodies) {
                    ChVector3d torque_abs;
                    if (data_dim == 1) {
                        torque_abs.x() = 0;
                        torque_abs.y() = 0;
                        torque_abs.z() = data_values[i_data + 0];
                        i_data += 1;
                    } else {
                        torque_abs.x() = data_values[i_data + 0];
                        torque_abs.y() = data_values[i_data + 1];
                        torque_abs.z() = data_values[i_data + 2];
                        i_data += 3;
                    }
                    c_body->body->AccumulateTorque(c_body->accumulator_index, torque_abs, false);
                    if (m_verbose)
                        cout << m_prefix2 << "body: " << c_body->body->GetName() << " | torque: " << torque_abs << endl;
                }
                break;
            }
            default:
                cerr << "\nERROR: Invalid Chrono MBS read data type (" << GetCouplingDataTypeAsString(data_type) << ")" << endl;
                throw std::runtime_error("Invalid Chrono MBS read data type");
        }
    }
}

void ChPreciceAdapterMbs::WriteBodyRefData(const std::string& mesh_name, CouplingMeshInfo& mesh_info) {
    auto mesh_dim = GetCouplingMeshDimensions(mesh_name);

    for (const auto& data_name : m_data_write[mesh_name]) {
        auto& data_info = mesh_info.data[data_name];
        if (!data_info.used)
            continue;
        auto data_type = data_info.type;
        auto& data_values = data_info.values;
        auto data_dim = GetCouplingDataDimensions(mesh_name, data_name);
        assert(data_values.size() == data_dim * m_coupling_bodies.size());
        switch (data_type) {
            case CouplingDataType::POSITIONS: {
                assert(data_dim == mesh_dim);
                size_t i_data = 0;
                for (auto& c_body : m_coupling_bodies) {
                    const auto& pos_abs = c_body->body->GetFrameRefToAbs().GetPos();
                    if (data_dim == 2) {
                        data_values[i_data + 0] = pos_abs.x();
                        data_values[i_data + 1] = pos_abs.y();
                        i_data += 2;
                    } else {
                        data_values[i_data + 0] = pos_abs.x();
                        data_values[i_data + 1] = pos_abs.y();
                        data_values[i_data + 2] = pos_abs.z();
                        i_data += 3;
                    }
                    if (m_verbose)
                        cout << m_prefix2 << "body: " << c_body->body->GetName() << " | pos:  " << pos_abs << endl;
                }
                break;
            }
            case CouplingDataType::ROTATIONS: {
                // Note: RotVecFromQuat gives a rotation angle in [-2*pi, 2*pi].
                //       A complete implementation would require proper 2*pi wrap handling which would take care of cases such as a continuously rotating body.
                //// TODO
                assert((mesh_dim == 3 && data_dim == 3) || (mesh_dim == 2 && data_dim == 1));
                size_t i_data = 0;
                for (auto& c_body : m_coupling_bodies) {
                    const auto& rot_abs = c_body->body->GetFrameRefToAbs().GetRot();
                    auto rotvec_abs = RotVecFromQuat(rot_abs);
                    if (data_dim == 1) {
                        data_values[i_data + 0] = rotvec_abs.z();
                        i_data += 1;
                    } else {
                        data_values[i_data + 0] = rotvec_abs.x();
                        data_values[i_data + 1] = rotvec_abs.y();
                        data_values[i_data + 2] = rotvec_abs.z();
                        i_data += 3;
                    }
                    if (m_verbose)
                        cout << m_prefix2 << "body: " << c_body->body->GetName() << " | rot dir:  " << rotvec_abs.GetNormalized() << " rot angle: " << rotvec_abs.Length() << endl;
                }
                break;
            }
            case CouplingDataType::DISPLACEMENTS: {
                assert(data_dim == mesh_dim);
                size_t i_data = 0;
                for (auto& c_body : m_coupling_bodies) {
                    auto displ_abs = c_body->body->GetFrameRefToAbs().GetPos() - c_body->init_body_frame.GetPos();
                    if (data_dim == 2) {
                        data_values[i_data + 0] = displ_abs.x();
                        data_values[i_data + 1] = displ_abs.y();
                        i_data += 2;
                    } else {
                        data_values[i_data + 0] = displ_abs.x();
                        data_values[i_data + 1] = displ_abs.y();
                        data_values[i_data + 2] = displ_abs.z();
                        i_data += 3;
                    }
                    if (m_verbose)
                        cout << m_prefix2 << "body: " << c_body->body->GetName() << " | displacement:  " << displ_abs << endl;
                }
                break;
            }
            case CouplingDataType::LINEAR_VELOCITIES: {
                assert(data_dim == mesh_dim);
                size_t i_data = 0;
                for (auto& c_body : m_coupling_bodies) {
                    const auto& lin_vel_abs = c_body->body->GetFrameRefToAbs().GetPosDt();
                    if (data_dim == 2) {
                        data_values[i_data + 0] = lin_vel_abs.x();
                        data_values[i_data + 1] = lin_vel_abs.y();
                        i_data += 2;
                    } else {
                        data_values[i_data + 0] = lin_vel_abs.x();
                        data_values[i_data + 1] = lin_vel_abs.y();
                        data_values[i_data + 2] = lin_vel_abs.z();
                        i_data += 3;
                    }
                    if (m_verbose)
                        cout << m_prefix2 << "body: " << c_body->body->GetName() << " | lin_vel:  " << lin_vel_abs << endl;
                }
                break;
            }
            case CouplingDataType::ANGULAR_VELOCITIES: {
                assert((mesh_dim == 3 && data_dim == 3) || (mesh_dim == 2 && data_dim == 1));
                size_t i_data = 0;
                for (auto& c_body : m_coupling_bodies) {
                    const auto& ang_vel_abs = c_body->body->GetAngVelParent();
                    if (data_dim == 1) {
                        data_values[i_data + 0] = ang_vel_abs.z();
                        i_data += 1;
                    } else {
                        data_values[i_data + 0] = ang_vel_abs.x();
                        data_values[i_data + 1] = ang_vel_abs.y();
                        data_values[i_data + 2] = ang_vel_abs.z();
                        i_data += 3;
                    }
                    if (m_verbose)
                        cout << m_prefix2 << "body: " << c_body->body->GetName() << " | ang_vel:  " << ang_vel_abs << endl;
                }
                break;
            }
            default:
                cerr << "\nERROR: Invalid Chrono MBS write data type (" << GetCouplingDataTypeAsString(data_type) << ")" << endl;
                throw std::runtime_error("Invalid Chrono MBS write data type");
        }
    }
}

void ChPreciceAdapterMbs::ReadBodyMeshData(const std::string& mesh_name, const CouplingMeshInfo& mesh_info) {
    auto mesh_dim = GetCouplingMeshDimensions(mesh_name);

    for (const auto& data_name : m_data_read[mesh_name]) {
        const auto& data_info = mesh_info.data.at(data_name);
        if (!data_info.used)
            continue;
        auto data_type = data_info.type;
        const auto& data_values = data_info.values;
        auto data_dim = GetCouplingDataDimensions(mesh_name, data_name);
        assert(data_values.size() == data_dim * GetNumVertices(mesh_name));
        switch (data_type) {
            case CouplingDataType::FORCES: {
                assert(data_dim == mesh_dim);
                size_t i_data = 0;
                for (auto& c_body : m_coupling_bodies) {
                    for (const auto& pos_loc : c_body->points) {
                        ChVector3d force_abs;
                        if (data_dim == 2) {
                            force_abs.x() = data_values[i_data + 0];
                            force_abs.y() = data_values[i_data + 1];
                            force_abs.z() = 0;
                            i_data += 2;
                        } else {
                            force_abs.x() = data_values[i_data + 0];
                            force_abs.y() = data_values[i_data + 1];
                            force_abs.z() = data_values[i_data + 2];
                            i_data += 3;
                        }
                        ChVector3d pos_abs = c_body->body->GetFrameRefToAbs().TransformPointLocalToParent(pos_loc);
                        c_body->body->AccumulateForce(c_body->accumulator_index, force_abs, pos_abs, false);
                    }
                }
                break;
            }
            case CouplingDataType::TORQUES: {
                assert((mesh_dim == 3 && data_dim == 3) || (mesh_dim == 2 && data_dim == 1));
                size_t i_data = 0;
                for (auto& c_body : m_coupling_bodies) {
                    for (const auto& pos_loc : c_body->points) {
                        ChVector3d torque_abs;
                        if (data_dim == 1) {
                            torque_abs.x() = 0;
                            torque_abs.y() = 0;
                            torque_abs.z() = data_values[i_data + 0];
                            i_data += 1;
                        } else {
                            torque_abs.x() = data_values[i_data + 0];
                            torque_abs.y() = data_values[i_data + 1];
                            torque_abs.z() = data_values[i_data + 2];
                            i_data += 3;
                        }
                        c_body->body->AccumulateTorque(c_body->accumulator_index, torque_abs, false);
                    }
                }
                break;
            }
            default:
                cerr << "\nERROR: Invalid Chrono MBS read data type (" << GetCouplingDataTypeAsString(data_type) << ")" << endl;
                throw std::runtime_error("Invalid Chrono MBS read data type");
        }
    }
}

void ChPreciceAdapterMbs::WriteBodyMeshData(const std::string& mesh_name, CouplingMeshInfo& mesh_info) {
    auto mesh_dim = GetCouplingMeshDimensions(mesh_name);

    for (const auto& data_name : m_data_write[mesh_name]) {
        auto& data_info = mesh_info.data[data_name];
        if (!data_info.used)
            continue;
        auto data_type = data_info.type;
        auto& data_values = data_info.values;
        auto data_dim = GetCouplingDataDimensions(mesh_name, data_name);
        assert(data_values.size() == data_dim * GetNumVertices(mesh_name));
        switch (data_type) {
            case CouplingDataType::POSITIONS: {
                assert(data_dim == mesh_dim);
                size_t i_data = 0;
                for (auto& c_body : m_coupling_bodies) {
                    for (const auto& pos_loc : c_body->points) {
                        ChVector3d pos_abs = c_body->body->GetFrameRefToAbs().TransformPointLocalToParent(pos_loc);
                        if (data_dim == 2) {
                            data_values[i_data + 0] = pos_abs.x();
                            data_values[i_data + 1] = pos_abs.y();
                            i_data += 2;
                        } else {
                            data_values[i_data + 0] = pos_abs.x();
                            data_values[i_data + 1] = pos_abs.y();
                            data_values[i_data + 2] = pos_abs.z();
                            i_data += 3;
                        }
                    }
                }
                break;
            }
            case CouplingDataType::DISPLACEMENTS: {
                assert(data_dim == mesh_dim);
                size_t i_data = 0;
                for (auto& c_body : m_coupling_bodies) {
                    for (const auto& pos_loc : c_body->points) {
                        ChVector3d displ_abs = c_body->body->GetFrameRefToAbs().TransformPointLocalToParent(pos_loc) - c_body->init_body_frame.TransformPointLocalToParent(pos_loc);
                        if (data_dim == 2) {
                            data_values[i_data + 0] = displ_abs.x();
                            data_values[i_data + 1] = displ_abs.y();
                            i_data += 2;
                        } else {
                            data_values[i_data + 0] = displ_abs.x();
                            data_values[i_data + 1] = displ_abs.y();
                            data_values[i_data + 2] = displ_abs.z();
                            i_data += 3;
                        }
                    }
                }
                break;
            }
            case CouplingDataType::LINEAR_VELOCITIES: {
                assert(data_dim == mesh_dim);
                size_t i_data = 0;
                for (auto& c_body : m_coupling_bodies) {
                    for (const auto& pos_loc : c_body->points) {
                        ChVector3d vel_abs = c_body->body->PointSpeedLocalToParent(pos_loc);
                        if (data_dim == 2) {
                            data_values[i_data + 0] = vel_abs.x();
                            data_values[i_data + 1] = vel_abs.y();
                            i_data += 2;
                        } else {
                            data_values[i_data + 0] = vel_abs.x();
                            data_values[i_data + 1] = vel_abs.y();
                            data_values[i_data + 2] = vel_abs.z();
                            i_data += 3;
                        }
                    }
                }
                break;
            }
            default:
                cerr << "\nERROR: Invalid Chrono MBS write data type (" << GetCouplingDataTypeAsString(data_type) << ")" << endl;
                throw std::runtime_error("Invalid Chrono MBS write data type");
        }
    }
}

// -----------------------------------------------------------------------------

void ChPreciceAdapterMbs::OnReadDataAM(const std::vector<ChMatrix66d>& blocks) {
    auto num_bodies = m_coupling_bodies.size();
    assert(blocks.size() > 0);
    assert(m_has_added_mass);

    m_hydro_load->UpdateBodyAddedMassBlocks(blocks);
}

// -----------------------------------------------------------------------------

double ChPreciceAdapterMbs::GetSolverTimeStep(double max_time_step) const {
    return std::min(m_time_step, max_time_step);
}

void ChPreciceAdapterMbs::AdvanceParticipant(double time, double time_step) {
    ChAssertAlways(time == m_sys->GetChTime());

    // Generate output (if enabled)
    Output(time);

    // Render (if enabled)
    Render(time);

    // Execute pre-step callbacks, advance system dynamics, execute post-step callbacks
    if (m_beforestep_callback)
        m_beforestep_callback->OnStepDynamics(time, time_step);

    m_sys->DoStepDynamics(time_step);

    if (m_afterstep_callback)
        m_afterstep_callback->OnStepDynamics(time, time_step);

    // Enforce soft real-time
    if (m_enforce_realtime)
        m_rt_timer.Spin(time_step);
}

// -----------------------------------------------------------------------------

void ChPreciceAdapterMbs::OnWriteOutput(int frame, double time) {
    m_output_db->Write(frame, time, m_output_data);
#ifdef CHRONO_FEA
    //// TODO
    ////m_output_db->WriteFeaMeshes(m_output_data.meshes);
#endif
}

}  // end namespace ch_precice
}  // namespace chrono
