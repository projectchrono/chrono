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
#ifdef CHRONO_HAS_YAML
    #include "chrono/input_output/ChUtilsYAML.h"
#endif

#include "chrono_precice/ChPreciceAdapter.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace ch_precice {

ChPreciceAdapter::ChPreciceAdapter()
    : m_participant(nullptr),
      m_process_index(0),
      m_process_size(1),
      m_interfaces_created(false),
      m_participant_created(false),
      m_mesh_created(false),
      m_initialized(false),
      m_verbose(false),
      m_visualize(false),
      m_output(false),
      m_output_dir(".") {}

void ChPreciceAdapter::SetOutputDir(const std::string& out_dir) {
    auto p = std::filesystem::path(out_dir);
    if (!exists(p) || !is_directory(p)) {
        std::cerr << "The specified path " << out_dir << " is not a valid directory." << std::endl;
        throw std::runtime_error("Invalid directory");
    }

    m_output_dir = out_dir;
}

// -----------------------------------------------------------------------------

#ifdef CHRONO_HAS_YAML

static std::string ToUpper(std::string in) {
    std::transform(in.begin(), in.end(), in.begin(), ::toupper);
    return in;
}

static ChPreciceAdapter::CouplingMeshType ReadCouplingMeshType(const YAML::Node& a) {
    auto type = ChToUpper(a.as<std::string>());
    if (type == "GENERIC")
        return ChPreciceAdapter::CouplingMeshType::GENERIC;
    if (type == "RIGID_BODY_REF_POINTS")
        return ChPreciceAdapter::CouplingMeshType::RIGID_BODY_REF_POINTS;
    if (type == "RIGID_BODY_MESH_POINTS")
        return ChPreciceAdapter::CouplingMeshType::RIGID_BODY_MESH_POINTS;
    if (type == "FEA_MESH1D_NODES")
        return ChPreciceAdapter::CouplingMeshType::FEA_MESH1D_NODES;
    if (type == "FEA_MESH2D_NODES")
        return ChPreciceAdapter::CouplingMeshType::FEA_MESH2D_NODES;

    cerr << "Unknown mesh type: " << a.as<std::string>() << endl;
    throw std::runtime_error("Invalid mesh type");
}

static ChPreciceAdapter::CouplingDataType ReadCouplingDataType(const YAML::Node& a) {
    auto type = ChToUpper(a.as<std::string>());
    if (type == "GENERIC")
        return ChPreciceAdapter::CouplingDataType::GENERIC;
    if (type == "POSITIONS")
        return ChPreciceAdapter::CouplingDataType::POSITIONS;
    if (type == "DISPLACEMENTS")
        return ChPreciceAdapter::CouplingDataType::DISPLACEMENTS;
    if (type == "VELOCITIES")
        return ChPreciceAdapter::CouplingDataType::VELOCITIES;
    if (type == "FORCES")
        return ChPreciceAdapter::CouplingDataType::FORCES;
    if (type == "TORQUES")
        return ChPreciceAdapter::CouplingDataType::TORQUES;

    cerr << "Unknown data type: " << a.as<std::string>() << endl;
    throw std::runtime_error("Invalid data type");
}

std::string ChPreciceAdapter::GetParticipantName(const std::string& input_filename) {
    YAML::Node yaml = YAML::LoadFile(input_filename);
    ChAssertAlways(yaml["precice_adapter_config"]);
    auto config = yaml["precice_adapter_config"];
    ChAssertAlways(config["participant_name"]);
    return config["participant_name"].as<std::string>();
}

void ChPreciceAdapter::ConfigureParticipant(const std::string& input_filename) {
    // Read YAML input file and search for preCICE adapter configuration
    YAML::Node yaml = YAML::LoadFile(input_filename);
    ChAssertAlways(yaml["precice_adapter_config"]);
    auto config = yaml["precice_adapter_config"];

    // Read participant name from the YAML configuration
    ChAssertAlways(config["participant_name"]);
    m_participant_name = config["participant_name"].as<std::string>();

    // Read mesh interfaces and data names for writing and reading from the YAML configuration, and initialize the data maps for each mesh/data pair
    ChAssertAlways(config["interfaces"]);
    auto interfaces = config["interfaces"];
    ChAssertAlways(interfaces.IsSequence());

    for (size_t i = 0; i < interfaces.size(); i++) {
        ChAssertAlways(interfaces[i]["mesh_name"]);
        auto mesh_name = interfaces[i]["mesh_name"].as<std::string>();
        auto mesh_type = CouplingMeshType::GENERIC;
        if (interfaces[i]["mesh_type"])
            mesh_type = ReadCouplingMeshType(interfaces[i]["mesh_type"]);

        CouplingMeshInfo mesh_info;
        mesh_info.type = mesh_type;
        mesh_info.vertex_ids = std::vector<int>();  // initialize empty vector

        if (interfaces[i]["write_data"]) {
            auto write_data = interfaces[i]["write_data"];
            ChAssertAlways(write_data.IsSequence());
            for (size_t j = 0; j < write_data.size(); j++) {
                ChAssertAlways(write_data[j]["name"]);
                auto data_name = write_data[j]["name"].as<std::string>();
                auto data_type = CouplingDataType::GENERIC;
                if (write_data[j]["type"])
                    data_type = ReadCouplingDataType(write_data[j]["type"]);
                m_data_write[mesh_name].push_back(data_name);
                mesh_info.data[data_name].type = data_type;
                mesh_info.data[data_name].values = std::vector<double>();  // initialize empty vector
            }
        }

        if (interfaces[i]["read_data"]) {
            auto read_data = interfaces[i]["read_data"];
            ChAssertAlways(read_data.IsSequence());
            for (size_t j = 0; j < read_data.size(); j++) {
                ChAssertAlways(read_data[j]["name"]);
                auto data_name = read_data[j]["name"].as<std::string>();
                auto data_type = CouplingDataType::GENERIC;
                if (read_data[j]["type"])
                    data_type = ReadCouplingDataType(read_data[j]["type"]);
                m_data_read[mesh_name].push_back(data_name);
                mesh_info.data[data_name].type = data_type;
                mesh_info.data[data_name].values = std::vector<double>();  // initialize empty vector
            }
        }
        m_coupling_meshes.insert({mesh_name, mesh_info});
    }

    m_interfaces_created = (interfaces.size() > 0);

    //// TODO : add checks for validity of the configuration (e.g., check that mesh names and data names are consistent with those declared in the preCICE configuration file, etc.)
}

#endif

// -----------------------------------------------------------------------------

void ChPreciceAdapter::AddCouplingMeshInterface(const std::string& mesh_name,
                                                CouplingMeshType data_type,
                                                const std::vector<std::string>& data_write_names,
                                                const std::vector<std::string>& data_read_names) {
    CouplingMeshInfo mesh_info;
    mesh_info.type = data_type;
    mesh_info.vertex_ids = std::vector<int>();  // initialize empty vector

    for (const auto& data_name : data_write_names) {
        m_data_write[mesh_name].push_back(data_name);
        mesh_info.data[data_name].values = std::vector<double>();  // initialize empty vector
    }
    for (const auto& data_name : data_read_names) {
        m_data_read[mesh_name].push_back(data_name);
        mesh_info.data[data_name].values = std::vector<double>();  // initialize empty vector
    }
    m_coupling_meshes.insert({mesh_name, mesh_info});

    m_interfaces_created = true;
}

// -----------------------------------------------------------------------------

void ChPreciceAdapter::RegisterParticipant(const std::string& precice_config_filename, int process_index, int process_size) {
    m_prefix1 = "[" + m_participant_name + "] ";
    m_prefix2 = std::string(m_prefix1.size(), ' ');

    assert(m_participant == nullptr);
    m_participant = std::make_unique<precice::Participant>(m_participant_name, precice_config_filename, m_process_index, m_process_size);
    m_participant_created = true;
}

// -----------------------------------------------------------------------------

void ChPreciceAdapter::RegisterMesh(const std::string& mesh_name, const std::vector<ChVector3d>& positions) {
    assert(m_participant_created);
    assert(m_interfaces_created);

    auto mesh_dim = m_participant->getMeshDimensions(mesh_name);
    ChAssertAlways(mesh_dim == 2 || mesh_dim == 3);

    std::vector<double> pos_vec;
    for (const auto& pos : positions) {
        pos_vec.push_back(pos.x());
        pos_vec.push_back(pos.y());
        if (mesh_dim == 3)
            pos_vec.push_back(pos.z());
    }

    RegisterMesh(mesh_name, pos_vec);
}

void ChPreciceAdapter::RegisterMesh(const std::string& mesh_name, const std::vector<double>& positions) {
    assert(m_participant_created);
    assert(m_interfaces_created);

    int mesh_dim = m_participant->getMeshDimensions(mesh_name);
    ChAssertAlways(mesh_dim == 2 || mesh_dim == 3);
    ChAssertAlways(positions.size() % mesh_dim == 0);

    size_t num_vertices = positions.size() / mesh_dim;
    m_coupling_meshes[mesh_name].vertex_ids.resize(num_vertices);
    m_participant->setMeshVertices(mesh_name, positions, m_coupling_meshes[mesh_name].vertex_ids);
    m_mesh_created = true;

    // Resize the vectors of values for all data associated with this mesh.
    // Data dimension is the number of values per vertex for the data, which is determined by preCICE based on the configuration file
    // (e.g., scalar data has dimension 1, vector data has dimension equal to mesh dimension, etc.)
    for (auto& [data_name, data_info] : m_coupling_meshes[mesh_name].data) {
        int data_dim = m_participant->getDataDimensions(mesh_name, data_name);
        data_info.values.resize(num_vertices * data_dim);
    }

    if (m_verbose) {
        cout << m_prefix1 << "Register mesh '" << mesh_name << "'" << endl;
        cout << m_prefix2 << "dimension:        " << mesh_dim << endl;
        cout << m_prefix2 << "num. vertices:    " << GetNumVertices(mesh_name) << endl;
        cout << m_prefix2 << "mesh type:        " << GetCouplingMeshTypeAsString(mesh_name) << endl;
        cout << m_prefix2 << "read interfaces:  ";
        for (auto& data_name : GetReadDataNamesOnMesh(mesh_name))
            cout << "'" << data_name << "' (" << GetCouplingDataDimensions(mesh_name, data_name) << "," << GetCouplingDataTypeAsString(mesh_name, data_name) << ")  ";
        cout << endl;
        cout << m_prefix2 << "write interfaces: ";
        for (auto& data_name : GetWriteDataNamesOnMesh(mesh_name))
            cout << "'" << data_name << "' (" << GetCouplingDataDimensions(mesh_name, data_name) << "," << GetCouplingDataTypeAsString(mesh_name, data_name) << ")  ";
        cout << endl;
    }
}

ChPreciceAdapter::CouplingMeshType ChPreciceAdapter::GetCouplingMeshType(const std::string& mesh_name) const {
    return m_coupling_meshes.at(mesh_name).type;
}

std::string ChPreciceAdapter::GetCouplingMeshTypeAsString(const std::string& mesh_name) const {
    auto type = GetCouplingMeshType(mesh_name);
    switch (type) {
        case CouplingMeshType::GENERIC:
            return "GENERIC";
        case CouplingMeshType::RIGID_BODY_REF_POINTS:
            return "RIGID_BODY_REF_POINTS";
        case CouplingMeshType::RIGID_BODY_MESH_POINTS:
            return "RIGID_BODY_MESH_POINTS";
        case CouplingMeshType::FEA_MESH1D_NODES:
            return "FEA_MESH1D_NODES";
        case CouplingMeshType::FEA_MESH2D_NODES:
            return "FEA_MESH2D_NODES";
    }
    return "UNKNOWN";
}

ChPreciceAdapter::CouplingDataType ChPreciceAdapter::GetCouplingDataType(const std::string& mesh_name, const std::string& data_name) const {
    return m_coupling_meshes.at(mesh_name).data.at(data_name).type;
}

std::string ChPreciceAdapter::GetCouplingDataTypeAsString(CouplingDataType type) {
    switch (type) {
        case CouplingDataType::GENERIC:
            return "GENERIC";
        case CouplingDataType::POSITIONS:
            return "POSITIONS";
        case CouplingDataType::DISPLACEMENTS:
            return "DISPLACEMENTS";
        case CouplingDataType::VELOCITIES:
            return "VELOCITIES";
        case CouplingDataType::FORCES:
            return "FORCES";
        case CouplingDataType::TORQUES:
            return "TORQUES";
    }
    return "UNKNOWN";
}

std::string ChPreciceAdapter::GetCouplingDataTypeAsString(const std::string& mesh_name, const std::string& data_name) const {
    auto type = GetCouplingDataType(mesh_name, data_name);
    return GetCouplingDataTypeAsString(type);
}

int ChPreciceAdapter::GetCouplingMeshDimensions(const std::string& mesh_name) const {
    assert(m_participant_created);
    return m_participant->getMeshDimensions(mesh_name);
}

int ChPreciceAdapter::GetCouplingDataDimensions(const std::string& mesh_name, const std::string& data_name) const {
    assert(m_participant_created);
    return m_participant->getDataDimensions(mesh_name, data_name);
}

double ChPreciceAdapter::GetMaxTimeStepSize() const {
    return m_participant->getMaxTimeStepSize();
}

const std::string& ChPreciceAdapter::GetParticipantName() const {
    assert(m_participant_created);
    return m_participant_name;
}

std::vector<std::string> ChPreciceAdapter::GetCouplingMeshNames() const {
    assert(m_participant_created);

    std::vector<std::string> mesh_names;
    for (const auto& imap : m_coupling_meshes)
        mesh_names.push_back(imap.first);
    return mesh_names;
}

std::vector<std::string> ChPreciceAdapter::GetReadDataNamesOnMesh(const std::string& mesh_name) const {
    assert(m_participant_created);
    return m_data_read.at(mesh_name);
}

std::vector<std::string> ChPreciceAdapter::GetWriteDataNamesOnMesh(const std::string& mesh_name) const {
    assert(m_participant_created);
    return m_data_write.at(mesh_name);
}

size_t ChPreciceAdapter::GetNumVertices(const std::string& mesh_name) {
    assert(m_participant_created);
    return m_coupling_meshes[mesh_name].vertex_ids.size();
}

// -----------------------------------------------------------------------------

bool ChPreciceAdapter::MustWriteInitialData() {
    assert(m_participant_created);
    return m_participant->requiresInitialData();
}

bool ChPreciceAdapter::IsCouplingOngoing() {
    assert(m_participant_created);
    return m_participant->isCouplingOngoing();
}

bool ChPreciceAdapter::IsTimeWindowComplete() {
    assert(m_participant_created);
    return m_participant->isTimeWindowComplete();
}

// -----------------------------------------------------------------------------

void ChPreciceAdapter::InitializeSimulation() {
    assert(m_participant_created);
    assert(!m_initialized);

    InitializeParticipant();

    assert(m_mesh_created);

    if (m_participant->requiresInitialData()) {
        WriteData();
    }

    m_participant->initialize();

    m_initialized = true;
}

void ChPreciceAdapter::RunSimulation() {
    assert(m_participant_created);
    assert(m_interfaces_created);
    assert(m_mesh_created);
    assert(m_initialized);

    double time = 0;
    while (IsCouplingOngoing()) {
        if (m_participant->requiresWritingCheckpoint())
            WriteCheckpoint(time);

        // Agree on time step size
        double max_time_step = GetMaxTimeStepSize();
        double time_step = std::min(max_time_step, GetSolverTimeStep(max_time_step));

        // Compute time step for solver and advance solver and coupling
        ReadData();
        AdvanceParticipant(time, time_step);
        WriteData();
        m_participant->advance(time_step);

        if (m_participant->requiresReadingCheckpoint())
            ReadCheckpoint(time);
        else
            time += time_step;
    }
}

void ChPreciceAdapter::FinalizeSimulation() {
    assert(m_participant_created);
    assert(m_initialized);

    FinalizeParticipant();
    m_participant->finalize();
}

// -----------------------------------------------------------------------------

void ChPreciceAdapter::InitializeParticipant() {
    if (m_verbose)
        cout << m_prefix1 << "Initialization" << endl;
}

void ChPreciceAdapter::WriteData() {
    std::string msg = m_prefix1 + "Write data\n";
    for (auto& [mesh_name, mesh_info] : m_coupling_meshes) {
        for (const auto& data_name : m_data_write[mesh_name]) {
            auto data_dim = std::to_string(GetCouplingDataDimensions(mesh_name, data_name));
            msg += m_prefix2 + mesh_name + ":" + data_name + " (" + data_dim + "," + GetCouplingDataTypeAsString(mesh_name, data_name) + ")\n";
            WriteDataBlock(mesh_name, data_name, mesh_info.data[data_name].values);
        }
        if (m_verbose)
            cout << msg;
    }
}

void ChPreciceAdapter::ReadData() {
    std::string msg = m_prefix1 + "Read data\n";
    for (auto& [mesh_name, mesh_info] : m_coupling_meshes) {
        for (const auto& data_name : m_data_read[mesh_name]) {
            auto data_dim = std::to_string(GetCouplingDataDimensions(mesh_name, data_name));
            msg += m_prefix2 + mesh_name + ":" + data_name + " (" + data_dim + "," + GetCouplingDataTypeAsString(mesh_name, data_name) + ")\n";
            mesh_info.data[data_name].values = ReadDataBlock(mesh_name, data_name);
        }
        if (m_verbose)
            cout << msg;
    }
}

void ChPreciceAdapter::WriteCheckpoint(double time) {
    if (m_verbose)
        cout << m_prefix1 << "Write checkpoint at time = " << time << endl;
}

void ChPreciceAdapter::ReadCheckpoint(double time) {
    if (m_verbose)
        cout << m_prefix1 << "Read checkpoint for time = " << time << endl;
}

void ChPreciceAdapter::AdvanceParticipant(double time, double time_step) {
    if (m_verbose)
        cout << m_prefix1 << "Advance from " << time << " by " << time_step << endl;
}

void ChPreciceAdapter::FinalizeParticipant() {
    if (m_verbose)
        cout << m_prefix1 << "Shutdown" << endl;
}

// -----------------------------------------------------------------------------

void ChPreciceAdapter::SetDataBlock(const std::string& mesh_name, const std::string& data_name, const std::vector<double>& data) {
    auto it_mesh = m_coupling_meshes.find(mesh_name);
    assert(it_mesh != m_coupling_meshes.end());
    auto& mesh_info = it_mesh->second.data;

    auto it_data = mesh_info.find(data_name);
    assert(it_data != mesh_info.end());
    it_data->second.values = data;
}

void ChPreciceAdapter::WriteDataBlock(const std::string& mesh_name, const std::string& data_name) {
    const auto& vertex_ids = m_coupling_meshes[mesh_name].vertex_ids;
    const auto& data_info = m_coupling_meshes[mesh_name].data[data_name];
    m_participant->writeData(mesh_name, data_name, vertex_ids, data_info.values);
}

void ChPreciceAdapter::WriteDataBlock(const std::string& mesh_name, const std::string& data_name, const std::vector<double>& data) {
    SetDataBlock(mesh_name, data_name, data);
    WriteDataBlock(mesh_name, data_name);
}

void ChPreciceAdapter::ReadDataBlock(const std::string& mesh_name, const std::string& data_name, double relative_read_time) {
    const auto& vertex_ids = m_coupling_meshes[mesh_name].vertex_ids;
    auto& data_info = m_coupling_meshes[mesh_name].data[data_name];
    m_participant->readData(mesh_name, data_name, vertex_ids, relative_read_time, data_info.values);
}

const std::vector<double>& ChPreciceAdapter::GetDataBlock(const std::string& mesh_name, const std::string& data_name) const {
    auto it_mesh = m_coupling_meshes.find(mesh_name);
    assert(it_mesh != m_coupling_meshes.end());
    auto& mesh_info = it_mesh->second.data;

    auto it_data = mesh_info.find(data_name);
    assert(it_data != mesh_info.end());
    return it_data->second.values;
}

const std::vector<double>& ChPreciceAdapter::ReadDataBlock(const std::string& mesh_name, const std::string& data_name) {
    ReadDataBlock(mesh_name, data_name, 0);
    return GetDataBlock(mesh_name, data_name);
}

// -----------------------------------------------------------------------------

bool ChPreciceAdapter::WriteCheckpointIfRequired(double time) {
    assert(m_participant_created);
    if (!m_participant->requiresWritingCheckpoint())
        return false;
    WriteCheckpoint(time);
    return true;
}

bool ChPreciceAdapter::ReadCheckpointIfRequired(double time) {
    assert(m_participant_created);
    if (!m_participant->requiresReadingCheckpoint())
        return false;
    ReadCheckpoint(time);
    return true;
}

// -----------------------------------------------------------------------------

std::vector<double> ChPreciceAdapter::SetVerticesToData(const std::vector<ChVector2d>& vertices) {
    std::vector<double> data;
    data.reserve(vertices.size() * 2);
    for (const auto& v : vertices) {
        data.push_back(v.x());
        data.push_back(v.y());
    }
    return data;
}

std::vector<double> ChPreciceAdapter::SetVerticesToData(const std::vector<ChVector3d>& vertices) {
    std::vector<double> data;
    data.reserve(vertices.size() * 3);
    for (const auto& v : vertices) {
        data.push_back(v.x());
        data.push_back(v.y());
        data.push_back(v.z());
    }
    return data;
}

}  // end namespace ch_precice
}  // namespace chrono
