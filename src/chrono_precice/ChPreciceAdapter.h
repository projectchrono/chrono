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

#ifndef CH_PRECICE_ADAPTER_H
#define CH_PRECICE_ADAPTER_H

#include <string>
#include <vector>
#include <map>

#include "chrono_precice/ChApiPrecice.h"

#include "chrono/ChConfig.h"
#include "chrono/ChVersion.h"
#include "chrono/core/ChVector2.h"
#include "chrono/core/ChVector3.h"
#include "chrono/utils/ChUtils.h"

#include "precice/precice.hpp"

namespace chrono {
namespace ch_precice {

/// @addtogroup precice_module
/// @{

/// Base class for all preCICE adapters.
class ChApiPrecice ChPreciceAdapter {
  public:
    /// Chrono coupling mesh types (mesh vertex semantics).
    enum class CouplingMeshType {
        GENERIC,                 ///< generic mesh
        RIGID_BODY_REF_POINTS,   ///< set of points corresponding to rigid body reference frames
        RIGID_BODY_MESH_POINTS,  ///< set of points on rigid body meshes
        FEA_MESH1D_NODES,        ///< set of points corresponding to 1D FEA meshes (segment nodes)
        FEA_MESH2D_NODES         ///< set of points corresponding to 2D FEA meshes (surface mesh nodes)
    };

    /// Chrono coupling data type.
    enum class CouplingDataType {
        GENERIC,        ///< generic data
        POSITIONS,      ///< 3D positions (body reference, body points, of FEA nodes)
        DISPLACEMENTS,  ///< 3D displacements (relative to initial position)
        VELOCITIES,     ///< 3D velocities (of bodies, body points, or FEA nodes)
        FORCES,         ///< 3D forces (on bodies, body points, or FEA nodes)
        TORQUES         ///< 3D torques (on bodies or FEA nodes)
    };

    virtual ~ChPreciceAdapter() {}

    /// Enable verbose terminal output (default: false).
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    // ---- preCICE participant registration

    /// Register the participant with preCICE, using the specified preCICE configuration file, and the solver process size and index.
    void RegisterParticipant(const std::string& precice_config_filename, int process_index = 0, int process_size = 1);

    // ---- Accessor functions

    /// Get the number of spatial dimensions for the mesh with specified name.
    int GetCouplingMeshDimensions(const std::string& mesh_name) const;

    /// Get the number of vertices for the mesh with specified name.
    size_t GetNumVertices(const std::string& mesh_name);

    /// Get the type (vertex semantics) for the mesh with specified name.
    CouplingMeshType GetCouplingMeshType(const std::string& mesh_name) const;

    /// Get the type (vertex semantics) for the mesh with specified name.
    std::string GetCouplingMeshTypeAsString(const std::string& mesh_name) const;

    /// Get the data dimensions for the data with specified name on the mesh with specified name.
    int GetCouplingDataDimensions(const std::string& mesh_name, const std::string& data_name) const;

    /// Get the type for the coupling data with specified name on the mesh with specified name.
    CouplingDataType GetCouplingDataType(const std::string& mesh_name, const std::string& data_name) const;

    /// Get the type for the coupling data with specified name on the mesh with specified name.
    std::string GetCouplingDataTypeAsString(const std::string& mesh_name, const std::string& data_name) const;

    /// Return the data type as a string.
    static std::string GetCouplingDataTypeAsString(CouplingDataType type);

    /// Get the maximum time step size from preCICE.
    double GetMaxTimeStepSize() const;

    /// Get the participant name from config
    const std::string& GetParticipantName() const;

    /// Get the coupled mesh names on this participant.
    std::vector<std::string> GetCouplingMeshNames() const;

    /// Get the data names for reading on the mesh with specified name.
    std::vector<std::string> GetReadDataNamesOnMesh(const std::string& mesh_name) const;

    /// Get the data names for writing on the mesh with specified name.
    std::vector<std::string> GetWriteDataNamesOnMesh(const std::string& mesh_name) const;

    // ---- Simulation control

    /// Check if the participant is required to provide initial data.
    /// If true, the participant needs to write initial data to defined vertices prior to calling Initialize().
    bool MustWriteInitialData();

    /// Check if coupling is ongoing.
    bool IsCouplingOngoing();

    /// Check if the time window has completed.
    bool IsTimeWindowComplete();

    /// Wrapper function for initializing the coupled simulation for this participant.
    /// The participant initializes the output data (if needed), after which the coupling is initialized.
    /// The operations performed by this function are:
    /// - initialize the participant solver
    /// - let participant to write initial data (if requested)
    /// - initialize the preCICE coupling for this participant
    void InitializeSimulation();

    /// Wrapper function for performing the simulation loop.
    /// While coupling is ongoing, at each iteration, the participant:
    /// - writes a checkpoint if requested
    /// - agrees on the simulation time step size
    /// - reads data
    /// - advances the underlying solver dynamics
    /// - writes data
    /// - advances the preCICE coupling
    /// - reads a checkpoint if requested, otherwise advances time
    void RunSimulation();

    /// Wrapper function for finalizing the coupled simulation for this participant.
    /// The operations performed by this function are:
    /// - let participant finalize (shutdown)
    /// - finalize preCICE coupling
    void FinalizeSimulation();

  protected:
    ChPreciceAdapter();
    ChPreciceAdapter(const ChPreciceAdapter&) = delete;
    void operator=(const ChPreciceAdapter&) = delete;

    // ---- preCICE participant construction

#ifdef CHRONO_HAS_YAML
    /// Get the participant name from the specified YAML input file.
    static std::string GetParticipantName(const std::string& input_filename);

    /// Configure the Chrono participant/solver and its mesh interfaces for use with preCICE, using the specified input file.
    /// The YAML input file must have a group named "precice_adapter_config" with the following parameters:
    /// - participant_name: name of the participant or solver
    /// - interfaces:       list of interfaces, each with the following parameters:
    ///     - mesh_name:    name of the coupling mesh
    ///     - read_data:    data to read from preCICE on this mesh
    ///     - write_data:   data to write to preCICE on this mesh
    void ConfigureParticipant(const std::string& input_filename);
#endif

    /// Set the participant name.
    /// Used when the adapter is not constructed from a YAML specification file.
    void SetParticipantName(const std::string& participant_name) { m_participant_name = participant_name; }

    /// Add a coupling mesh with specified data type, using the given lists of data names for writing and reading.
    /// Used when the adapter is not constructed from a YAML specification file. This function can be called more than once.
    void AddCouplingMeshInterface(const std::string& mesh_name,
                                  CouplingMeshType data_type,
                                  const std::vector<std::string>& data_write_names,
                                  const std::vector<std::string>& data_read_names);

    // ---- Mesh specification

    /// Register a coupling mesh with preCICE, using the specified mesh name and its vertex positions (of ChVector3d type).
    /// If the mesh dimension is 2, the z component of the given position vectors is discarded.
    /// With the mesh size, the data maps inside the adapter initialize the relevant data vector to size of mesh_size*data_dimension.
    void RegisterMesh(const std::string& mesh_name, const std::vector<ChVector3d>& positions);

    /// Register a coupling mesh with preCICE, using the specified mesh name and its vertex positions (as a flattened vector of doubles).
    /// This method can be used for both 2D and 3D meshes by providing the appropriate positions vector.
    /// The positions vector is expected to be in the format:
    /// - (x0, y0, x1, y1, ...) for 2D meshes, and
    /// - (x0, y0, z0, x1, y1, z1, ...) for 3D meshes.
    /// With the mesh size, the data maps inside the adapter initialize the relevant data vector to size of mesh_size*data_dimension.
    /// - scalar data (declared with <data:scalar ...>) has data_dimension = 1; e.g., temperature, pressure, etc.
    /// - vector data (declared with <data:vector ...>) has data_dimension equal to the mesh dimension; e.g., velocity, displacement, etc.
    void RegisterMesh(const std::string& mesh_name, const std::vector<double>& positions);

    // ---- Data exchange

    // Set the (write) data vector for the specified mesh and data names.
    void SetDataBlock(const std::string& mesh_name, const std::string& data_name, const std::vector<double>& data);

    /// Write (send) a block of data to preCICE.
    void WriteDataBlock(const std::string& mesh_name, const std::string& data_name);

    /// Set and write (send) a block of data to preCICE.
    /// This is a convenience function that combines SetDataBlock and WriteDataBlock into a single call.
    void WriteDataBlock(const std::string& mesh_name, const std::string& data_name, const std::vector<double>& data);

    /// Read (receive) a block of data from preCICE.
    void ReadDataBlock(const std::string& mesh_name, const std::string& data_name, double relative_read_time);

    /// Get the (read) data vector for the specified mesh and data names.
    const std::vector<double>& GetDataBlock(const std::string& mesh_name, const std::string& data_name) const;

    /// Read (receive) a block of data from preCICE and return the data vector.
    /// This is a convenience function that combines ReadDataBlock and GetDataBlock into a single call.
    /// It is assumed that the data is always read at the beginning of the time step (relative_read_time = 0).
    const std::vector<double>& ReadDataBlock(const std::string& mesh_name, const std::string& data_name);

    // ---- Checkpointing

    /// Write the solver state to a checkpoint if required by preCICE.
    /// If requested by preCISE, this function invokes the solver-specific checkpoint writing function.
    /// The return value indicates whether a checkpoint was written.
    bool WriteCheckpointIfRequired(double time);

    /// Read the solver state from a checkpoint if required by preCICE.
    /// If requested by preCISE, this function invokes the solver-specific checkpoint reading function.
    /// The return value indicates whether a checkpoint was read.
    bool ReadCheckpointIfRequired(double time);

    // ---- Participant/solver-specific functions to be implemented by derived classes

    /// Let the derived class perform any necessary operations during the simulation initialization.
    /// This function is called before the solver writes initial data (if requested) and before the preCICE coupling is initialized.
    /// After the call to InitializeParticipant, it is assumed that the coupling meshes have been set.
    virtual void InitializeParticipant();

    /// Let the derived class implement the actual checkpoint writing if required by preCICE.
    virtual void WriteCheckpoint(double time);

    /// Let the derived class implement the actual checkpoint reading if required by preCICE.
    /// The solver from a derived class must restore its state at the values in the last saved checkpoint and, if needed, reset its internal time to the provided value.
    virtual void ReadCheckpoint(double time);

    /// Read data from other solvers.
    /// A derived class must:
    /// - call the base class implementation to receive data from preCICE
    /// - perform any necessary processing of the data now stored in m_coupling_meshes
    /// - only access data from entries with names in m_data_read
    virtual void ReadData() = 0;

    /// Let the derived class implement the actual computation of the solver time step based on the maximum time step provided by preCICE.
    /// The default implementation simply returns the maximum time step provided by preCICE, but derived classes can override this to implement custom time-stepping logic.
    virtual double GetSolverTimeStep(double max_time_step) const { return max_time_step; }

    /// Let the derived class implement the actual solver time-stepping by the given time step.
    virtual void AdvanceParticipant(double time, double time_step);

    /// Write data for other solvers.
    /// A derived class must:
    /// - prepare the data to be sent and load it in m_coupling_meshes
    /// - only access data from entries with names in m_data_write
    /// - call the base class implementation to send data to preCICE
    virtual void WriteData() = 0;

    /// Let the derived class perform any necessary operations during simulation shutdown.
    /// This function is called before the preCICE coupling is finalized.
    virtual void FinalizeParticipant();

    // ---- Utility functions

    /// Convert a vector of ChVector2d to a vector of doubles in the format (x0, y0, x1, y1, ...).
    static std::vector<double> SetVerticesToData(const std::vector<ChVector2d>& vertices);

    /// Convert a vector of ChVector3d to a vector of doubles in the format (x0, y0, z0, x1, y1, z1, ...).
    static std::vector<double> SetVerticesToData(const std::vector<ChVector3d>& vertices);

    // ---- Member variables

    /// Definition of a coupling data block.
    struct CouplingDataInfo {
        CouplingDataType type;       ///< data type
        std::vector<double> values;  ///< data values
    };

    /// Data type to hold information for all data blocks in a coupling mesh, indexed by the data name.
    using CouplingData = std::map<std::string, CouplingDataInfo>;

    /// Definition of a coupling mesh.
    struct CouplingMeshInfo {
        CouplingMeshType type;        ///< mesh type (vertex semantics)
        std::vector<int> vertex_ids;  ///< preCICE IDs of the coupling mesh vertices
        CouplingData data;            ///< list of data objects defined on the coupling mesh
    };

    /// Data type to hold data for all coupling meshes, indexed by the mesh name.
    using CouplingMeshes = std::map<std::string, CouplingMeshInfo>;

    /// Data type to hold data names associated with a given mesh name.
    using MeshDataNames = std::map<std::string, std::vector<std::string>>;

    std::unique_ptr<precice::Participant> m_participant;  ///< preCICE instance
    std::string m_participant_name;                       ///< name of the participant/solver
    int m_process_size;                                   ///< number of processes used by an instance of this solver
    int m_process_index;                                  ///< index for each process used by this solver

    CouplingMeshes m_coupling_meshes;  ///< data for all coupling meshes
    MeshDataNames m_data_read;         ///< input data names for all coupling meshes
    MeshDataNames m_data_write;        ///< output data names for all coupling meshes

    bool m_interfaces_created;   ///< true if the data interfaces were created
    bool m_participant_created;  ///< true if the preCICE participant was created
    bool m_mesh_created;         ///< true if preCICE coupling meshes were created
    bool m_initialized;          ///< true if preCICE participant was initialized

    bool m_verbose;         ///< verbose terminal output
    std::string m_prefix1;  ///< prefix for terminal messages (first line)
    std::string m_prefix2;  ///< prefix for terminal messages (subsequent lines)
};

/// @} precice_module

}  // end namespace ch_precice
}  // namespace chrono

#endif
