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

#include "chrono/input_output/ChOutputASCII.h"
#ifdef CHRONO_HAS_HDF5
    #include "chrono/input_output/ChOutputHDF5.h"
#endif

#include "chrono_precice/ChPreciceAdapterMbs.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace ch_precice {

// -----------------------------------------------------------------------------

// Utility function to read a list of 3D vectors from a space-delimited file.
static std::vector<ChVector3d> ReadPoints(const std::string& filename) {
    // Open input file stream
    std::ifstream ifile;
    std::string line;
    try {
        ifile.exceptions(std::ios::failbit | std::ios::badbit | std::ios::eofbit);
        ifile.open(filename);
    } catch (const std::exception&) {
        cerr << "Cannot open input file '" << filename << "'" << endl;
        throw std::invalid_argument("Cannot open input file");
    }

    // Read number of points
    std::getline(ifile, line);
    std::istringstream iss(line);
    size_t num_points;
    iss >> num_points;

    // Read points
    std::vector<ChVector3d> points;
    for (size_t i = 0; i < num_points; i++) {
        std::getline(ifile, line);
        std::istringstream jss(line);
        double x, y, z;
        jss >> x >> y >> z;
        points.push_back(ChVector3d(x, y, z));
    }

    return points;
}

// -----------------------------------------------------------------------------

ChPreciceAdapterMbs::ChPreciceAdapterMbs(std::shared_ptr<ChSystem> sys, double time_step, bool verbose)
    : m_sys(sys), m_time_step(time_step), m_enforce_realtime(false), m_output_dir(".") {
    SetVerbose(verbose);
}

#if defined(CHRONO_PARSERS) && defined(CHRONO_HAS_YAML)
ChPreciceAdapterMbs::ChPreciceAdapterMbs(const std::string& input_filename, bool verbose) : m_output_dir(".") {
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

    m_enforce_realtime = parser.EnforceRealtime();

    // Read in preCICE participant configuration
    ConfigureParticipant(input_filename);

    // Check consistency between preCICE participant specification and the MBS
    YAML::Node yaml = YAML::LoadFile(input_filename);
    ChAssertAlways(yaml["precice_adapter_config"]);
    auto config = yaml["precice_adapter_config"];

    // Read configuration of data file specification
    m_file_handler.SetReferenceDirectory(input_filename);
    m_file_handler.Read(config);

    // Read information on interface physics items and check that they are defined in the MBS
    if (config["bodies"]) {
        auto bodies = config["bodies"];
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

    #ifdef CHRONO_FEA
    if (config["meshes1d"]) {
        //// TODO
    }

    if (config["meshes2d"]) {
        //// TODO
    }
    #endif

    if (m_verbose) {
        cout << "\n-------------------------------------------------\n" << endl;
    }
}
#endif

ChPreciceAdapterMbs::~ChPreciceAdapterMbs() {}

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

    m_output_data.meshes.push_back(fea_mesh);
}
#endif

// -----------------------------------------------------------------------------

ChPreciceAdapterMbs::OutputParameters::OutputParameters() : type(ChOutput::Type::NONE), mode(ChOutput::Mode::FRAMES), fps(100) {}

bool ChPreciceAdapterMbs::EnableOutput(ChOutput::Type db_type, ChOutput::Mode mode, double output_fps) {
    m_output.type = db_type;
    m_output.mode = mode;
    m_output.fps = output_fps;
    return (db_type != ChOutput::Type::NONE);
}

void ChPreciceAdapterMbs::SetOutputDir(const std::string& out_dir) {
    auto p = std::filesystem::path(out_dir);
    if (!exists(p) || !is_directory(p)) {
        std::cerr << "The specified path " << out_dir << " is not a valid directory." << std::endl;
        throw std::runtime_error("Invalid directory");
    }

    m_output_dir = out_dir;

    if (m_verbose) {
        auto filename = m_output_dir + "/mbs_results";
        switch (m_output.type) {
            case ChOutput::Type::ASCII:
                filename += ".txt";
                break;
            case ChOutput::Type::HDF5:
#ifdef CHRONO_HAS_HDF5
                filename += ".h5";
                break;
#else
                return;
#endif
        }
        cout << "Output file: " << filename << endl;
    }
}

ChPreciceAdapterMbs::VisParameters::VisParameters()
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

    // For each interface mesh:
    // - check that coupling meshes have dimension 2 or 3 (as reported by preCICE)
    // - check that coupling data have dimension equal to the mesh dimension (as reported by preCICE)
    // - set mesh vertices (depending on mesh type and dimension)
    // - register mesh with preCICE
    for (const auto& mesh_name : GetCouplingMeshNames()) {
        auto mesh_dim = GetCouplingMeshDimensions(mesh_name);

        // Check consistency of mesh dimension and read data dimension
        for (const auto& data_name : GetReadDataNamesOnMesh(mesh_name)) {
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
                case CouplingDataType::DISPLACEMENTS:
                case CouplingDataType::VELOCITIES:
                    cerr << "[InitializeParticipant] Invalid Chrono MBS read data type (" << GetCouplingDataTypeAsString(data_type) << ")" << endl;
                    throw std::runtime_error("Invalid Chrono MBS read data type");
            }
        }

        // Check consistency of mesh dimension and read data dimension
        for (const auto& data_name : GetWriteDataNamesOnMesh(mesh_name)) {
            auto data_type = GetCouplingDataType(mesh_name, data_name);
            auto data_dim = GetCouplingDataDimensions(mesh_name, data_name);
            switch (data_type) {
                case CouplingDataType::POSITIONS:
                case CouplingDataType::DISPLACEMENTS:
                case CouplingDataType::VELOCITIES:
                    // Positions and velocities must have the same simension as the coupling mesh
                    ChAssertAlways(data_dim == mesh_dim);
                    break;
                case CouplingDataType::FORCES:
                case CouplingDataType::TORQUES:
                    cerr << "[InitializeParticipant] Invalid Chrono MBS write data type (" << GetCouplingDataTypeAsString(data_type) << ")" << endl;
                    throw std::runtime_error("Invalid Chrono MBS write data type");
            }
        }

        // Set mesh vertices, based on mesh type
        std::vector<ChVector3d> vertices;
        auto& mesh_info = m_coupling_meshes[mesh_name];
        switch (mesh_info.type) {
            case CouplingMeshType::RIGID_BODY_REF_POINTS: {
                for (const auto& c_body : m_coupling_bodies)
                    vertices.push_back(c_body->body->GetFrameRefToAbs().GetPos());
                break;
            }
            case CouplingMeshType::RIGID_BODY_MESH_POINTS: {
                for (const auto& c_body : m_coupling_bodies) {
                    ChAssertAlways(!c_body->points.empty());
                    for (const auto& pos_loc : c_body->points) {
                        ChVector3d pos_abs = c_body->init_body_frame.TransformPointLocalToParent(pos_loc);
                        vertices.push_back(pos_abs);
                    }
                }
                break;
            }
            case CouplingMeshType::FEA_MESH1D_NODES: {
                //// TODO
                ////break;
                throw std::runtime_error("CouplingMeshType::FEA_MESH1D_NODES not yet implemented");
            }
            case CouplingMeshType::FEA_MESH2D_NODES: {
                //// TODO
                ////break;
                throw std::runtime_error("CouplingMeshType::FEA_MESH2D_NODES not yet implemented");
            }
        }

        // Register coupling mesh with preCICE, taking into account mesh dimension
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
            case CouplingMeshType::RIGID_BODY_REF_POINTS:
                ReadBodyRefData(mesh_name, mesh_info);
                break;
            case CouplingMeshType::RIGID_BODY_MESH_POINTS:
                ReadBodyMeshData(mesh_name, mesh_info);
                break;
            case CouplingMeshType::FEA_MESH1D_NODES:
                //// TODO
                break;
            case CouplingMeshType::FEA_MESH2D_NODES:
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

    static int output_frame = 0;
    if (m_output.type != ChOutput::Type::NONE) {
        if (time >= output_frame / m_output.fps) {
            SaveOutput(output_frame);
            output_frame++;
        }
    }

    if (m_beforestep_callback)
        m_beforestep_callback->OnStepDynamics(time, time_step);

    m_sys->DoStepDynamics(time_step);

    if (m_afterstep_callback)
        m_afterstep_callback->OnStepDynamics(time, time_step);

    if (m_enforce_realtime)
        m_rt_timer.Spin(time_step);
}

void ChPreciceAdapterMbs::WriteData() {
    for (auto& [mesh_name, mesh_info] : m_coupling_meshes) {
        switch (mesh_info.type) {
            case CouplingMeshType::RIGID_BODY_REF_POINTS:
                WriteBodyRefData(mesh_name, mesh_info);
                break;
            case CouplingMeshType::RIGID_BODY_MESH_POINTS:
                WriteBodyMeshData(mesh_name, mesh_info);
                break;
            case CouplingMeshType::FEA_MESH1D_NODES:
                //// TODO
                break;
            case CouplingMeshType::FEA_MESH2D_NODES:
                //// TODO
                break;
        }
    }

    ChPreciceAdapter::WriteData();
}

// -----------------------------------------------------------------------------

void ChPreciceAdapterMbs::SaveOutput(int frame) {
    // Create the output DB if needed
    if (!m_output_db) {
        auto filename = m_output_dir + "/mbs_results";
        switch (m_output.type) {
            case ChOutput::Type::ASCII:
                filename += ".txt";
                m_output_db = chrono_types::make_shared<ChOutputASCII>(filename);
                break;
            case ChOutput::Type::HDF5:
#ifdef CHRONO_HAS_HDF5
                filename += ".h5";
                m_output_db = chrono_types::make_shared<ChOutputHDF5>(filename, m_output.mode);
                break;
#else
                return;
#endif
        }

        m_output_db->Initialize();
    }

    m_output_db->WriteBodies(m_output_data.bodies);
#ifdef CHRONO_FEA
    //// TODO
    ////m_output_db->WriteFeaMeshes(m_output_data.meshes);
#endif
}

// -----------------------------------------------------------------------------

void ChPreciceAdapterMbs::ReadBodyRefData(const std::string& mesh_name, const CouplingMeshInfo& mesh_info) {
    auto mesh_dim = GetCouplingMeshDimensions(mesh_name);

    for (const auto& data_name : m_data_read[mesh_name]) {
        auto data_dim = GetCouplingDataDimensions(mesh_name, data_name);
        const auto& data_info = mesh_info.data.at(data_name);
        auto data_type = data_info.type;
        const auto& data_values = data_info.values;
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
                cerr << "[ReadBodyRefData] Invalid Chrono MBS read data type (" << GetCouplingDataTypeAsString(data_type) << ")" << endl;
                throw std::runtime_error("Invalid Chrono MBS read data type");
        }
    }
}

void ChPreciceAdapterMbs::WriteBodyRefData(const std::string& mesh_name, CouplingMeshInfo& mesh_info) {
    auto mesh_dim = GetCouplingMeshDimensions(mesh_name);

    for (const auto& data_name : m_data_write[mesh_name]) {
        auto data_dim = GetCouplingDataDimensions(mesh_name, data_name);
        auto& data_info = mesh_info.data[data_name];
        auto data_type = data_info.type;
        auto& data_values = data_info.values;
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
                        cout << m_prefix2 << "body: " << c_body->body->GetName() << " | displ:  " << displ_abs << endl;
                }
                break;
            }
            case CouplingDataType::VELOCITIES: {
                assert(data_dim == mesh_dim);
                size_t i_data = 0;
                for (auto& c_body : m_coupling_bodies) {
                    const auto& vel_abs = c_body->body->GetFrameRefToAbs().GetPosDt();
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
                    if (m_verbose)
                        cout << m_prefix2 << "body: " << c_body->body->GetName() << " | vel:  " << vel_abs << endl;
                }
                break;
            }
            default:
                cerr << "[WriteBodyRefData] Invalid Chrono MBS write data type (" << GetCouplingDataTypeAsString(data_type) << ")" << endl;
                throw std::runtime_error("Invalid Chrono MBS write data type");
        }
    }
}

void ChPreciceAdapterMbs::ReadBodyMeshData(const std::string& mesh_name, const CouplingMeshInfo& mesh_info) {
    auto mesh_dim = GetCouplingMeshDimensions(mesh_name);

    for (const auto& data_name : m_data_read[mesh_name]) {
        auto data_dim = GetCouplingDataDimensions(mesh_name, data_name);
        const auto& data_info = mesh_info.data.at(data_name);
        auto data_type = data_info.type;
        const auto& data_values = data_info.values;
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
                cerr << "[ReadBodyMeshData] Invalid Chrono MBS read data type (" << GetCouplingDataTypeAsString(data_type) << ")" << endl;
                throw std::runtime_error("Invalid Chrono MBS read data type");
        }
    }
}

void ChPreciceAdapterMbs::WriteBodyMeshData(const std::string& mesh_name, CouplingMeshInfo& mesh_info) {
    auto mesh_dim = GetCouplingMeshDimensions(mesh_name);

    for (const auto& data_name : m_data_write[mesh_name]) {
        auto data_dim = GetCouplingDataDimensions(mesh_name, data_name);
        auto& data_info = mesh_info.data[data_name];
        auto data_type = data_info.type;
        auto& data_values = data_info.values;
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
            case CouplingDataType::VELOCITIES: {
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
                cerr << "[WriteBodyMeshData] Invalid Chrono MBS write data type (" << GetCouplingDataTypeAsString(data_type) << ")" << endl;
                throw std::runtime_error("Invalid Chrono MBS write data type");
        }
    }
}

}  // end namespace ch_precice
}  // namespace chrono
