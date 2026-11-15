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

#include "chrono_precice/ChPreciceAdapterSph.h"

#ifdef CHRONO_FEA
    #include "chrono/fea/ChBeamSectionCableANCF.h"
    #include "chrono/fea/ChElementCableANCF.h"
#endif

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace ch_precice {

ChPreciceAdapterSph::ChPreciceAdapterSph(const std::string& precice_config_filename, std::shared_ptr<fsi::sph::ChFsiFluidSystemSPH> sysSPH, double time_step, bool verbose)
    : ChPreciceAdapter(precice_config_filename, "model_SPH", verbose), m_sysSPH(sysSPH), m_time_step(time_step) {}

#if defined(CHRONO_PARSERS) && defined(CHRONO_HAS_YAML)
ChPreciceAdapterSph::ChPreciceAdapterSph(const std::string& precice_config_filename, const std::string& input_filename, bool verbose)
    : ChPreciceAdapter(precice_config_filename, "", verbose) {
    // Create an SPH YAML parser and the underlying FSI problem, but do not initialize the FSI problem
    parsers::ChParserSphYAML parser(input_filename, verbose);
    m_model_name = parser.GetName();
    m_fsi_problem = parser.CreateFsiProblemSPH(false);
    m_sysSPH = m_fsi_problem->GetFluidSystemSPH();
    m_time_step = parser.GetTimestep();

    // Extract information from parsed YAML files
    m_output_settings = parser.GetOutputSettings();
    #ifdef CHRONO_VSG
    m_vis_settings = parser.GetVisualizationSettings();
    m_visSPH_settings = parser.GetSphVisualizationSettings();
    #endif

    // Read common preCICE participant configuration (participant name, file handler, and coupling interfaces)
    ReadParticipantConfigurationYAML(input_filename);

    // Read SPH-specific preCICE configuration (coupling objects)
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

            if (bodies[i]["shapes"]) {
                ChAssertAlways(bodies[i]["shapes"].IsSequence());
                auto body_geometry = ReadCollisionGeometry(bodies[i]["shapes"], m_file_handler, m_use_degrees);
                AddCouplingBody(body_name, ChFramed(body_pos, body_rot), body_geometry);
            } else if (bodies[i]["points"]) {
                auto points_file = bodies[i]["points"].as<std::string>();
                auto points = ReadPoints(m_file_handler.GetFilename(points_file));
                AddCouplingBody(body_name, ChFramed(body_pos, body_rot), points);
            } else {
                throw std::runtime_error("ERROR");
            }
        }
    }

    #ifdef CHRONO_FEA
    // - read the method for obtaining FEA node directions (applies to all FSI meshes)
    if (config["node_directions"]) {
        auto val = ChToUpper(config["node_directions"].as<std::string>());
        if (val == "NONE") {
            m_fsi_problem->UseNodeDirections(fsi::NodeDirectionsMode::NONE);
        } else if (val == "AVERAGE") {
            m_fsi_problem->UseNodeDirections(fsi::NodeDirectionsMode::AVERAGE);
        } else if (val == "EXACT") {
            cerr << "FEA node directions mode EXACT is not supported (node directions are not exchanged)" << endl;
            throw std::runtime_error("FEA node directions mode EXACT not supported");
        } else {
            cerr << "Unknown FEA node directions mode '" << val << "'" << endl;
            throw std::runtime_error("Unknown FEA node directions mode");
        }
    }

    // - read information on FEA meshes (1-D meshes specified through their nodes and segments)
    if (config["meshes"]) {
        auto meshes = config["meshes"];
        ChAssertAlways(meshes.IsSequence());
        for (int i = 0; i < meshes.size(); i++) {
            ChAssertAlways(meshes[i]["name"]);
            ChAssertAlways(meshes[i]["nodes"]);
            auto mesh_name = meshes[i]["name"].as<std::string>();

            // Mesh nodes, either listed explicitly or read from a file
            std::vector<ChVector3d> nodes;
            if (meshes[i]["nodes"].IsSequence()) {
                for (const auto& node : meshes[i]["nodes"])
                    nodes.push_back(ReadVector(node));
            } else {
                auto nodes_file = meshes[i]["nodes"].as<std::string>();
                nodes = ReadPoints(m_file_handler.GetFilename(nodes_file));
            }

            // Mesh segments (1-based node indices); if not specified, the nodes form a single chain
            std::vector<ChVector2i> segments;
            if (meshes[i]["segments"]) {
                auto segs = meshes[i]["segments"];
                ChAssertAlways(segs.IsSequence());
                for (const auto& seg : segs) {
                    ChAssertAlways(seg.IsSequence() && seg.size() == 2);
                    segments.push_back(ChVector2i(seg[0].as<int>() - 1, seg[1].as<int>() - 1));
                }
            } else {
                for (int j = 0; j + 1 < (int)nodes.size(); j++)
                    segments.push_back(ChVector2i(j, j + 1));
            }

            // BCE marker settings
            auto pattern = fsi::sph::BcePatternMesh1D::FULL;
            if (meshes[i]["bce_pattern_1D"]) {
                auto val = ChToUpper(meshes[i]["bce_pattern_1D"].as<std::string>());
                if (val == "STAR")
                    pattern = fsi::sph::BcePatternMesh1D::STAR;
                else if (val != "FULL") {
                    cerr << "Unknown 1-D BCE pattern '" << val << "' for FEA mesh '" << mesh_name << "'" << endl;
                    throw std::runtime_error("Unknown 1-D BCE pattern");
                }
            }
            bool remove_center = meshes[i]["remove_center"] ? meshes[i]["remove_center"].as<bool>() : false;
            bool check_embedded = meshes[i]["check_embedded"] ? meshes[i]["check_embedded"].as<bool>() : true;

            AddCouplingFEAMesh(mesh_name, nodes, segments, pattern, remove_center, check_embedded);
        }
    }
    #endif

    // Initialize the FSI problem (now that FSI solids are specified)
    m_fsi_problem->Initialize();

    if (m_verbose) {
        cout << "\n-------------------------------------------------\n" << endl;
    }
}
#endif

// -----------------------------------------------------------------------------

//// TODO - get rid of using a ChFsiProblemSPH when adding a body!!!
//// OK to use it when initializing the preCICE adapter from a YAML file (because the parser creates it anyway)

void ChPreciceAdapterSph::AddCouplingBody(const std::string& name, const ChFramed& frame, std::shared_ptr<ChBodyGeometry> geometry) {
    auto c_body = chrono_types::make_shared<CouplingBody>();
    c_body->index = (int)m_coupling_bodies.size();
    c_body->init_body_frame = frame;
    c_body->points = std::vector<ChVector3d>();
    m_coupling_bodies.push_back(c_body);

    // Create a dummy body
    auto body = chrono_types::make_shared<ChBodyAuxRef>();
    body->SetFrameRefToAbs(frame);

    m_fsi_problem->AddRigidBody(body, geometry, true);
}

void ChPreciceAdapterSph::AddCouplingBody(const std::string& name, const ChFramed& frame, const std::vector<ChVector3d> bce) {
    auto c_body = chrono_types::make_shared<CouplingBody>();
    c_body->index = (int)m_coupling_bodies.size();
    c_body->init_body_frame = frame;
    c_body->points = bce;
    m_coupling_bodies.push_back(c_body);

    // Create a dummy body
    auto body = chrono_types::make_shared<ChBodyAuxRef>();
    body->SetFrameRefToAbs(frame);

    m_fsi_problem->AddRigidBody(body, bce, ChFramed(), true);
}

#ifdef CHRONO_FEA
void ChPreciceAdapterSph::AddCouplingFEAMesh(const std::string& name,
                                             const std::vector<ChVector3d>& nodes,
                                             const std::vector<ChVector2i>& segments,
                                             fsi::sph::BcePatternMesh1D bce_pattern,
                                             bool remove_center,
                                             bool check_embedded) {
    ChAssertAlways(m_fsi_problem);
    ChAssertAlways(!segments.empty());

    auto c_mesh = chrono_types::make_shared<CouplingFEAMesh>();
    c_mesh->index = (int)m_coupling_fea.size();
    c_mesh->name = name;

    // Create a proxy FEA mesh with ANCF cable elements. The elements only define the segments used to generate the
    // BCE markers of the FSI mesh; the proxy mesh is never integrated, so the section properties are irrelevant.
    auto section = chrono_types::make_shared<fea::ChBeamSectionCableANCF>();
    c_mesh->mesh = chrono_types::make_shared<fea::ChMesh>();
    c_mesh->mesh->SetName(name);
    for (const auto& pos : nodes) {
        auto node = chrono_types::make_shared<fea::ChNodeFEAxyzD>(pos);
        c_mesh->mesh->AddNode(node);
        c_mesh->nodes.push_back(node);
    }
    for (const auto& seg : segments) {
        if (seg.x() < 0 || seg.x() >= (int)nodes.size() || seg.y() < 0 || seg.y() >= (int)nodes.size() || seg.x() == seg.y()) {
            cerr << "Invalid segment for FEA mesh '" << name << "'" << endl;
            throw std::runtime_error("Invalid FEA mesh segment");
        }
        const auto& node0 = c_mesh->nodes[seg.x()];
        const auto& node1 = c_mesh->nodes[seg.y()];
        auto dir = (node1->GetPos() - node0->GetPos()).GetNormalized();
        node0->SetSlope1(dir);
        node1->SetSlope1(dir);
        auto element = chrono_types::make_shared<fea::ChElementCableANCF>();
        element->SetNodes(node0, node1);
        element->SetSection(section);
        c_mesh->mesh->AddElement(element);
    }

    // Add the proxy mesh to the FSI problem (creates the FSI mesh and its BCE markers)
    m_fsi_problem->SetBcePattern1D(bce_pattern, remove_center);
    m_fsi_problem->AddFeaMesh(c_mesh->mesh, check_embedded);

    // Cache the associated FSI mesh and the FSI index of each mesh node
    const auto& fsi_meshes = m_fsi_problem->GetFsiSystemSPH()->GetFsiInterface().GetMeshes1D();
    ChAssertAlways(fsi_meshes.size() == m_coupling_fea.size() + 1);
    c_mesh->fsi_mesh = fsi_meshes.back();
    if (c_mesh->fsi_mesh->GetNumNodes() != nodes.size()) {
        cerr << "Some nodes of FEA mesh '" << name << "' do not belong to any segment" << endl;
        throw std::runtime_error("FEA mesh nodes not on segments");
    }
    for (const auto& node : c_mesh->nodes)
        c_mesh->fsi_node_index.push_back(c_mesh->fsi_mesh->ptr2ind_map.at(node));

    if (m_verbose)
        cout << m_prefix1 << "Add coupling FEA mesh '" << name << "' with " << nodes.size() << " nodes and " << segments.size() << " segments" << endl;

    m_coupling_fea.push_back(c_mesh);
}
#endif

// -----------------------------------------------------------------------------

size_t ChPreciceAdapterSph::GetNumFsiBodies() const {
    return m_coupling_bodies.size();
}

void ChPreciceAdapterSph::InitializeParticipant() {
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
                case CouplingDataType::LINEAR_VELOCITIES:
                case CouplingDataType::LINEAR_ACCELERATIONS:
                    ChAssertAlways(data_dim == mesh_dim);
                    break;
                case CouplingDataType::ROTATIONS:
                case CouplingDataType::ANGULAR_VELOCITIES:
                case CouplingDataType::ANGULAR_ACCELERATIONS:
                    // Rotational quantities are only defined for rigid bodies
                    if (m_coupling_meshes[mesh_name].type == CouplingMeshType::FEA_MESH_NODES || m_coupling_meshes[mesh_name].type == CouplingMeshType::FEA_MESH_POINTS) {
                        if (m_verbose)
                            cout << "FAIL" << endl;
                        cerr << "\nERROR: Invalid Chrono SPH read data type (" << GetCouplingDataTypeAsString(data_type) << ") on FEA mesh" << endl;
                        throw std::runtime_error("Invalid Chrono SPH read data type");
                    }
                    ChAssertAlways(data_dim == mesh_dim);
                    break;
                case CouplingDataType::DISPLACEMENTS:
                case CouplingDataType::FORCES:
                case CouplingDataType::TORQUES:
                    if (m_verbose)
                        cout << "FAIL" << endl;
                    cerr << "\nERROR: Invalid Chrono SPH read data type (" << GetCouplingDataTypeAsString(data_type) << ")" << endl;
                    throw std::runtime_error("Invalid Chrono SPH read data type");
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
                    // Torques are only defined for rigid bodies
                    if (m_coupling_meshes[mesh_name].type == CouplingMeshType::FEA_MESH_NODES || m_coupling_meshes[mesh_name].type == CouplingMeshType::FEA_MESH_POINTS) {
                        if (m_verbose)
                            cout << "FAIL" << endl;
                        cerr << "\nERROR: Invalid Chrono SPH write data type (" << GetCouplingDataTypeAsString(data_type) << ") on FEA mesh" << endl;
                        throw std::runtime_error("Invalid Chrono SPH write data type");
                    }
                    // Torques must be 3D for a 3D mesh and 1D for a 2D mesh
                    ChAssertAlways((mesh_dim == 3 && data_dim == 3) || (mesh_dim == 2 && data_dim == 1));
                    break;
                case CouplingDataType::POSITIONS:
                case CouplingDataType::ROTATIONS:
                case CouplingDataType::DISPLACEMENTS:
                case CouplingDataType::LINEAR_VELOCITIES:
                case CouplingDataType::ANGULAR_VELOCITIES:
                case CouplingDataType::LINEAR_ACCELERATIONS:
                case CouplingDataType::ANGULAR_ACCELERATIONS:
                    if (m_verbose)
                        cout << "FAIL" << endl;
                    cerr << "\nERROR: Invalid Chrono SPH write data type (" << GetCouplingDataTypeAsString(data_type) << ")" << endl;
                    throw std::runtime_error("Invalid Chrono SPH write data type");
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
#ifdef CHRONO_FEA
                ChAssertAlways(!m_coupling_fea.empty());
                for (const auto& c_mesh : m_coupling_fea) {
                    for (const auto& node : c_mesh->nodes)
                        vertices.push_back(node->GetPos());
                }
                break;
#else
                throw std::runtime_error("CouplingMeshType::FEA_MESH_NODES requires the Chrono::FEA module");
#endif
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

    // Allocate and initialize the solid states and forces exchanged with the SPH solver
    m_body_states.resize(m_coupling_bodies.size());
    m_body_forces.resize(m_coupling_bodies.size());
    for (size_t i = 0; i < m_coupling_bodies.size(); i++) {
        m_body_states[i].pos = m_coupling_bodies[i]->init_body_frame.GetPos();
        m_body_states[i].rot = m_coupling_bodies[i]->init_body_frame.GetRot();
    }
#ifdef CHRONO_FEA
    const auto& fsi_interface = m_fsi_problem->GetFsiSystemSPH()->GetFsiInterface();
    ChAssertAlways(fsi_interface.GetNumMeshes1D() == m_coupling_fea.size());
    m_mesh1D_states.resize(m_coupling_fea.size());
    m_mesh1D_forces.resize(m_coupling_fea.size());
    for (const auto& c_mesh : m_coupling_fea) {
        ChAssertAlways(c_mesh->fsi_mesh->index == c_mesh->index);
        auto num_nodes = c_mesh->fsi_mesh->GetNumNodes();
        auto& states = m_mesh1D_states[c_mesh->index];
        states.pos.resize(num_nodes);
        states.vel.assign(num_nodes, VNULL);
        states.acc.assign(num_nodes, VNULL);
        states.dir.assign(num_nodes, VNULL);
        states.has_node_directions = false;
        for (const auto& [index, node] : c_mesh->fsi_mesh->ind2ptr_map)
            states.pos[index] = node->GetPos();
        m_mesh1D_forces[c_mesh->index].force.assign(num_nodes, VNULL);
    }
    ChAssertAlways(fsi_interface.GetNumMeshes2D() == 0);
#endif

    //// TODO -- checkpointing...
    ////if (m_verbose)
    ////    cout << m_prefix1 << "Set up checkpointing" << endl;

#ifdef CHRONO_VSG
    // Enable runtime visualization
    if (m_visualize && m_vis_settings.render) {
        if (m_verbose)
            cout << m_prefix1 << "Set up run-time visualization" << endl;

        // SPH visualization plugin
        auto visFSI = chrono_types::make_shared<fsi::sph::ChSphVisualizationVSG>(m_sysSPH.get());
        visFSI->EnableFluidMarkers(m_visSPH_settings.sph_markers);
        visFSI->EnableBoundaryMarkers(m_visSPH_settings.bndry_bce_markers);
        visFSI->EnableRigidBodyMarkers(m_visSPH_settings.rigid_bce_markers);
        visFSI->EnableFeaMeshMarkers(m_visSPH_settings.flex_bce_markers);
        if (m_visSPH_settings.color_callback)
            visFSI->SetSPHColorCallback(m_visSPH_settings.color_callback, m_visSPH_settings.colormap);
        if (m_visSPH_settings.visibility_callback_sph)
            visFSI->SetSPHVisibilityCallback(m_visSPH_settings.visibility_callback_sph);
        if (m_visSPH_settings.visibility_callback_bce)
            visFSI->SetBCEVisibilityCallback(m_visSPH_settings.visibility_callback_bce);

        // VSG visual system (attach visFSI as plugin)
        m_vsg = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        m_vsg->AttachPlugin(visFSI);
        m_vsg->SetWindowTitle("Chrono preCICE SPH participant - " + m_participant_name);
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

void ChPreciceAdapterSph::OnWriteCheckpoint(double time) {
    throw std::runtime_error("Checkpointing not available for Chrono::FSI-SPH");
}

void ChPreciceAdapterSph::OnReadCheckpoint(double time) {
    throw std::runtime_error("Checkpointing not available for Chrono::FSI-SPH");
}

// -----------------------------------------------------------------------------

void ChPreciceAdapterSph::OnReadData() {
    // Load the received data into the cached solid states
    for (const auto& [mesh_name, mesh_info] : m_coupling_meshes) {
        switch (mesh_info.type) {
            case CouplingMeshType::RIGID_BODY_REFS:
                ReadBodyRefData(mesh_name, mesh_info);
                break;
            case CouplingMeshType::RIGID_BODY_POINTS:
                ReadBodyMeshData(mesh_name, mesh_info);
                break;
            case CouplingMeshType::FEA_MESH_NODES:
#ifdef CHRONO_FEA
                ReadFeaNodeData(mesh_name, mesh_info);
#endif
                break;
            case CouplingMeshType::FEA_MESH_POINTS:
                //// TODO
                break;
        }
    }

    // Pass the solid states to the SPH solver
#ifdef CHRONO_FEA
    m_sysSPH->LoadSolidStates(m_body_states, m_mesh1D_states, m_mesh2D_states);
#else
    m_sysSPH->LoadSolidStates(m_body_states);
#endif
    m_sysSPH->OnExchangeSolidStates();
}

void ChPreciceAdapterSph::OnWriteData() {
    // Get the solid forces from the SPH solver
    m_sysSPH->OnExchangeSolidForces();
#ifdef CHRONO_FEA
    m_sysSPH->StoreSolidForces(m_body_forces, m_mesh1D_forces, m_mesh2D_forces);
#else
    m_sysSPH->StoreSolidForces(m_body_forces);
#endif

    // Load the data to be sent from the cached solid forces
    for (auto& [mesh_name, mesh_info] : m_coupling_meshes) {
        switch (mesh_info.type) {
            case CouplingMeshType::RIGID_BODY_REFS:
                WriteBodyRefData(mesh_name, mesh_info);
                break;
            case CouplingMeshType::RIGID_BODY_POINTS:
                WriteBodyMeshData(mesh_name, mesh_info);
                break;
            case CouplingMeshType::FEA_MESH_NODES:
#ifdef CHRONO_FEA
                WriteFeaNodeData(mesh_name, mesh_info);
#endif
                break;
            case CouplingMeshType::FEA_MESH_POINTS:
                //// TODO
                break;
        }
    }
}

void ChPreciceAdapterSph::ReadBodyRefData(const std::string& mesh_name, const CouplingMeshInfo& mesh_info) {
    size_t num_bodies = m_coupling_bodies.size();
    auto& body_states = m_body_states;

    // Read data
    for (const auto& data_name : m_data_read[mesh_name]) {
        const auto& data_info = mesh_info.data.at(data_name);
        if (!data_info.used)
            continue;
        auto data_type = data_info.type;
        const auto& data_values = data_info.values;
        assert(data_values.size() == GetCouplingDataDimensions(mesh_name, data_name) * m_coupling_bodies.size());
        switch (data_type) {
            case CouplingDataType::POSITIONS: {
                assert(GetCouplingDataDimensions(mesh_name, data_name) == GetCouplingMeshDimensions(mesh_name));
                size_t i_data = 0;
                for (size_t i_body = 0; i_body < num_bodies; i_body++) {
                    auto& bstates = body_states[i_body];
                    bstates.pos.x() = data_values[i_data + 0];
                    bstates.pos.y() = data_values[i_data + 1];
                    bstates.pos.z() = data_values[i_data + 2];
                    i_data += 3;
                    if (m_verbose)
                        cout << m_prefix2 << "body: " << i_body << " | pos:  " << bstates.pos << endl;
                }
                break;
            }
            case CouplingDataType::ROTATIONS: {
                assert(GetCouplingDataDimensions(mesh_name, data_name) == GetCouplingMeshDimensions(mesh_name));
                size_t i_data = 0;
                for (size_t i_body = 0; i_body < num_bodies; i_body++) {
                    auto& bstates = body_states[i_body];
                    ChVector3d rotvec_abs;
                    rotvec_abs.x() = data_values[i_data + 0];
                    rotvec_abs.y() = data_values[i_data + 1];
                    rotvec_abs.z() = data_values[i_data + 2];
                    bstates.rot = QuatFromRotVec(rotvec_abs);
                    i_data += 3;
                    if (m_verbose)
                        cout << m_prefix2 << "body: " << i_body << " | rot dir:  " << rotvec_abs.GetNormalized() << " rot angle: " << rotvec_abs.Length() << endl;
                }
                break;
            }
            case CouplingDataType::LINEAR_VELOCITIES: {
                assert(GetCouplingDataDimensions(mesh_name, data_name) == GetCouplingMeshDimensions(mesh_name));
                size_t i_data = 0;
                for (size_t i_body = 0; i_body < num_bodies; i_body++) {
                    auto& bstates = body_states[i_body];
                    bstates.lin_vel.x() = data_values[i_data + 0];
                    bstates.lin_vel.y() = data_values[i_data + 1];
                    bstates.lin_vel.z() = data_values[i_data + 2];
                    i_data += 3;
                    if (m_verbose)
                        cout << m_prefix2 << "body: " << i_body << " | lin_vel:  " << bstates.lin_vel << endl;
                }
                break;
            }
            case CouplingDataType::ANGULAR_VELOCITIES: {
                assert(GetCouplingDataDimensions(mesh_name, data_name) == GetCouplingMeshDimensions(mesh_name));
                size_t i_data = 0;
                for (size_t i_body = 0; i_body < num_bodies; i_body++) {
                    auto& bstates = body_states[i_body];
                    bstates.ang_vel.x() = data_values[i_data + 0];
                    bstates.ang_vel.y() = data_values[i_data + 1];
                    bstates.ang_vel.z() = data_values[i_data + 2];
                    i_data += 3;
                    if (m_verbose)
                        cout << m_prefix2 << "body: " << i_body << " | ang_vel:  " << bstates.ang_vel << endl;
                }
                break;
            }
            case CouplingDataType::LINEAR_ACCELERATIONS: {
                assert(GetCouplingDataDimensions(mesh_name, data_name) == GetCouplingMeshDimensions(mesh_name));
                size_t i_data = 0;
                for (size_t i_body = 0; i_body < num_bodies; i_body++) {
                    auto& bstates = body_states[i_body];
                    bstates.lin_acc.x() = data_values[i_data + 0];
                    bstates.lin_acc.y() = data_values[i_data + 1];
                    bstates.lin_acc.z() = data_values[i_data + 2];
                    i_data += 3;
                    if (m_verbose)
                        cout << m_prefix2 << "body: " << i_body << " | lin_acc:  " << bstates.lin_acc << endl;
                }
                break;
            }
            case CouplingDataType::ANGULAR_ACCELERATIONS: {
                assert(GetCouplingDataDimensions(mesh_name, data_name) == GetCouplingMeshDimensions(mesh_name));
                size_t i_data = 0;
                for (size_t i_body = 0; i_body < num_bodies; i_body++) {
                    auto& bstates = body_states[i_body];
                    bstates.ang_acc.x() = data_values[i_data + 0];
                    bstates.ang_acc.y() = data_values[i_data + 1];
                    bstates.ang_acc.z() = data_values[i_data + 2];
                    i_data += 3;
                    if (m_verbose)
                        cout << m_prefix2 << "body: " << i_body << " | ang_acc:  " << bstates.ang_acc << endl;
                }
                break;
            }
            default:
                cerr << "\nERROR: Invalid Chrono SPH read data type (" << GetCouplingDataTypeAsString(data_type) << ")" << endl;
                throw std::runtime_error("Invalid Chrono SPH read data type");
        }
    }
}

void ChPreciceAdapterSph::WriteBodyRefData(const std::string& mesh_name, CouplingMeshInfo& mesh_info) {
    size_t num_bodies = m_coupling_bodies.size();
    const auto& body_forces = m_body_forces;

    // Write data
    for (const auto& data_name : m_data_write[mesh_name]) {
        auto& data_info = mesh_info.data[data_name];
        if (!data_info.used)
            continue;
        auto data_type = data_info.type;
        auto& data_values = data_info.values;
        assert(data_values.size() == GetCouplingDataDimensions(mesh_name, data_name) * m_coupling_bodies.size());
        switch (data_type) {
            case CouplingDataType::FORCES: {
                assert(GetCouplingDataDimensions(mesh_name, data_name) == GetCouplingMeshDimensions(mesh_name));
                size_t i_data = 0;
                for (size_t i_body = 0; i_body < num_bodies; i_body++) {
                    const auto& bforces = body_forces[i_body];
                    data_values[i_data + 0] = bforces.force.x();
                    data_values[i_data + 1] = bforces.force.y();
                    data_values[i_data + 2] = bforces.force.z();
                    i_data += 3;
                    if (m_verbose)
                        cout << m_prefix2 << "body: " << i_body << " | force: " << bforces.force << endl;
                }
                break;
            }
            case CouplingDataType::TORQUES: {
                assert(GetCouplingDataDimensions(mesh_name, data_name) == GetCouplingMeshDimensions(mesh_name));
                size_t i_data = 0;
                for (size_t i_body = 0; i_body < num_bodies; i_body++) {
                    const auto& bforces = body_forces[i_body];
                    data_values[i_data + 0] = bforces.torque.x();
                    data_values[i_data + 1] = bforces.torque.y();
                    data_values[i_data + 2] = bforces.torque.z();
                    i_data += 3;
                    if (m_verbose)
                        cout << m_prefix2 << "body: " << i_body << " | torque: " << bforces.torque << endl;
                }
                break;
            }
            default:
                cerr << "\nERROR: Invalid Chrono SPH write data type (" << GetCouplingDataTypeAsString(data_type) << ")" << endl;
                throw std::runtime_error("Invalid Chrono SPH write data type");
        }
    }
}

void ChPreciceAdapterSph::ReadBodyMeshData(const std::string& mesh_name, const CouplingMeshInfo& mesh_info) {
    //// TODO
    throw std::runtime_error("CouplingMeshType::RIGID_BODY_MESH_POINTS not yet implemented");
}

void ChPreciceAdapterSph::WriteBodyMeshData(const std::string& mesh_name, CouplingMeshInfo& mesh_info) {
    //// TODO
    throw std::runtime_error("CouplingMeshType::RIGID_BODY_MESH_POINTS not yet implemented");
}

// -----------------------------------------------------------------------------

#ifdef CHRONO_FEA

void ChPreciceAdapterSph::ReadFeaNodeData(const std::string& mesh_name, const CouplingMeshInfo& mesh_info) {
    for (const auto& data_name : m_data_read[mesh_name]) {
        const auto& data_info = mesh_info.data.at(data_name);
        if (!data_info.used)
            continue;
        auto data_type = data_info.type;
        const auto& data_values = data_info.values;
        assert(data_values.size() == 3 * GetNumVertices(mesh_name));
        size_t i_data = 0;
        for (const auto& c_mesh : m_coupling_fea) {
            auto& states = m_mesh1D_states[c_mesh->index];
            for (size_t i = 0; i < c_mesh->nodes.size(); i++) {
                ChVector3d value(data_values[i_data + 0], data_values[i_data + 1], data_values[i_data + 2]);
                i_data += 3;
                auto index = c_mesh->fsi_node_index[i];
                switch (data_type) {
                    case CouplingDataType::POSITIONS:
                        states.pos[index] = value;
                        break;
                    case CouplingDataType::LINEAR_VELOCITIES:
                        states.vel[index] = value;
                        break;
                    case CouplingDataType::LINEAR_ACCELERATIONS:
                        states.acc[index] = value;
                        break;
                    default:
                        cerr << "\nERROR: Invalid Chrono SPH read data type (" << GetCouplingDataTypeAsString(data_type) << ")" << endl;
                        throw std::runtime_error("Invalid Chrono SPH read data type");
                }
            }
        }
    }
}

void ChPreciceAdapterSph::WriteFeaNodeData(const std::string& mesh_name, CouplingMeshInfo& mesh_info) {
    for (const auto& data_name : m_data_write[mesh_name]) {
        auto& data_info = mesh_info.data[data_name];
        if (!data_info.used)
            continue;
        auto data_type = data_info.type;
        auto& data_values = data_info.values;
        assert(data_values.size() == 3 * GetNumVertices(mesh_name));
        if (data_type != CouplingDataType::FORCES) {
            cerr << "\nERROR: Invalid Chrono SPH write data type (" << GetCouplingDataTypeAsString(data_type) << ")" << endl;
            throw std::runtime_error("Invalid Chrono SPH write data type");
        }
        size_t i_data = 0;
        for (const auto& c_mesh : m_coupling_fea) {
            const auto& forces = m_mesh1D_forces[c_mesh->index].force;
            for (size_t i = 0; i < c_mesh->nodes.size(); i++) {
                const auto& force = forces[c_mesh->fsi_node_index[i]];
                data_values[i_data + 0] = force.x();
                data_values[i_data + 1] = force.y();
                data_values[i_data + 2] = force.z();
                i_data += 3;
            }
        }
    }
}

#endif

// -----------------------------------------------------------------------------

double ChPreciceAdapterSph::GetSolverTimeStep(double max_time_step) const {
    return std::min(m_time_step, max_time_step);
}

void ChPreciceAdapterSph::AdvanceParticipant(double time, double time_step) {
    ChAssertAlways(time == m_sysSPH->GetSimTime());

    // Generate output (if enabled)
    Output(time);

    // Render (if enabled)
    Render(time);

    // Advance system dynamics
    m_sysSPH->DoStepDynamics(time_step);
}

// -----------------------------------------------------------------------------

void ChPreciceAdapterSph::OnWriteOutput(int frame, double time) {
    //// TODO
}

}  // namespace ch_precice
}  // namespace chrono
