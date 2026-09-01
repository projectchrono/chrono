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

#ifdef CHRONO_HAS_YAML
    #include "chrono/input_output/ChUtilsYAML.h"
#endif

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
#endif

#include "chrono_thirdparty/rapidxml/rapidxml.hpp"

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
        GENERIC,            ///< generic mesh
        RIGID_BODY_REFS,    ///< set of points corresponding to rigid body reference frames
        RIGID_BODY_POINTS,  ///< set of points on rigid bodies
        FEA_MESH_NODES,     ///< set of points corresponding FEA mesh nodes
        FEA_MESH_POINTS     ///< set of points on FEA meshes
    };

    /// Chrono coupling data type.
    enum class CouplingDataType {
        GENERIC,             ///< generic data
        POSITIONS,           ///< 3D positions (of body ref frames, body points, or FEA nodes)
        ROTATIONS,           ///< 3D rotations (of body ref frames)
        DISPLACEMENTS,       ///< 3D displacements (relative to initial position)
        LINEAR_VELOCITIES,   ///< 3D velocities (of body ref frames, body points, or FEA nodes)
        ANGULAR_VELOCITIES,  ///< 3D angular velocities (of body ref frames)
        FORCES,              ///< 3D forces (on body ref frames, body points, or FEA nodes)
        TORQUES              ///< 3D torques (on body ref frames)
    };

    virtual ~ChPreciceAdapter() {}

    /// Enable/disable verbose terminal output (default: false).
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// Enable/disable run-time visualization (default: false).
    /// A concrete Chrono preCICE adapter may or may not support run-time visualization.
    void EnableVisualization(bool vis) { m_visualize = vis; }

    /// Enable/disable simulation output (default: false).
    /// A concrete Chrono preCICE adapter may or may not support simulation output.
    void EnableOutput(bool out) { m_output = out; }

    /// Set root output directory (default: ".").
    /// The specified directory must exist.
    void SetOutputDir(const std::string& out_dir);

    /// Set Chrono simulation output settings.
    /// Note: If the Chrono preCICE adapter is created from a YAML specification file, these settings are read from that file.
    void SetOutputSettings(const ChOutput::Settings& settings);

    /// Set Chrono simulation output settings.
    /// Note: If the Chrono preCICE adapter is created from a YAML specification file, these settings are read from that file.
    void SetOutputSettings(ChOutput::Format format,  ///< output DB format
                           ChOutput::Mode mode,      ///< output mode
                           double output_fps         ///< output frequency
    );

#ifdef CHRONO_VSG
    /// Set Chrono run-time visualization settings.
    /// Note: If the Chrono preCICE adapter is created from a YAML specification file, these settings are read from that file.
    void SetVisualizationSettings(const ChVisualSystem::Settings& settings);

    /// Set Chrono run-time visualization settings.
    /// Note: If the Chrono preCICE adapter is created from a YAML specification file, these settings are read from that file.
    void SetVisualizationSettings(double render_fps,                  ///< rendering frequency
                                  CameraVerticalDir camera_vertical,  ///< camera vertical direction (Y or Z)
                                  const ChVector3d& camera_location,  ///< initial camera location
                                  const ChVector3d& camera_target,    ///< initial camera look-at point
                                  bool enable_shadows,                ///< enable dynamic shadows
                                  bool write_images,                  ///< save image snapshots
                                  const std::string& image_dir,       ///< image output directory
                                  const std::string& image_type       ///< image type (file extension)
    );
#endif

    /// Set the Chrono model name.
    void SetModelName(const std::string& name) { m_model_name = name; }

    /// Get the name of the Chrono model.
    const std::string& GetModelName() const { return m_model_name; }

    // ---- Accessor functions

    /// Get the number of spatial dimensions for the mesh with specified name.
    /// This information is obtained from the preCICE configuration.
    int GetCouplingMeshDimensions(const std::string& mesh_name) const;

    /// Get the number of vertices for the mesh with specified name.
    size_t GetNumVertices(const std::string& mesh_name);

    /// Get the type (vertex semantics) for the mesh with specified name.
    CouplingMeshType GetCouplingMeshType(const std::string& mesh_name) const;

    /// Get the type (vertex semantics) for the mesh with specified name.
    std::string GetCouplingMeshTypeAsString(const std::string& mesh_name) const;

    /// Get the data dimensions for the data with specified name on the mesh with specified name.
    /// This information is obtained from the preCICE configuration.
    int GetCouplingDataDimensions(const std::string& mesh_name, const std::string& data_name) const;

    /// Get the type for the coupling data with specified name on the mesh with specified name.
    CouplingDataType GetCouplingDataType(const std::string& mesh_name, const std::string& data_name) const;

    /// Get the `used` flag for the coupling data with specified name on the mesh with specified name.
    /// Return true if the data block is referenced in the preCICE configuration file and false otherwise.
    bool GetCouplingDataUsed(const std::string& mesh_name, const std::string& data_name) const;

    /// Get the type for the coupling data with specified name on the mesh with specified name.
    std::string GetCouplingDataTypeAsString(const std::string& mesh_name, const std::string& data_name) const;

    /// Return the data type as a string.
    static std::string GetCouplingDataTypeAsString(CouplingDataType type);

    /// Get the maximum time step size from preCICE.
    double GetMaxTimeStepSize() const;

    /// Get the preCICE participant name.
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

    /// Create the participant (using the specified solver process size and index) and initialize the coupled simulation for this participant.
    /// The participant initializes the output data (if needed), after which the coupling is initialized.
    /// The operations performed by this function are:
    /// - initialize the participant solver
    /// - let participant to write initial data (if requested)
    /// - initialize the preCICE coupling for this participant
    void InitializeSimulation(int process_index = 0, int process_size = 1);

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
    ChPreciceAdapter(const std::string& precice_config_filename, const std::string& model_name = "", bool verbose = false);

    ChPreciceAdapter(const ChPreciceAdapter&) = delete;
    void operator=(const ChPreciceAdapter&) = delete;

    // ---- preCICE participant construction

#ifdef CHRONO_HAS_YAML
    /// Extract the participant name from the specified YAML input file.
    static std::string ReadParticipantNameYAML(const std::string& input_filename);

    /// Configure the Chrono participant/solver and its mesh interfaces for use with preCICE, using the specified YAML input file.
    /// The YAML input file must have a group named "precice_adapter_config" with the following members:
    /// - participant_name [required]: name of the participant or solver
    /// - `data_path`      [optional]: file handler for data files
    /// - `angle_degrees`  [optional]: indicate angle units (degrees or radians)
    /// - `interfaces`     [required]: list of interfaces, each with the following required members:
    ///     - `mesh_name`:    name of the coupling mesh
    ///     - `read_data`:    data to read from preCICE on this mesh
    ///     - `write_data`:   data to write to preCICE on this mesh
    /// This function creates all coupling meshes and mesh data from the YAML specification.
    void ReadParticipantConfigurationYAML(const std::string& input_filename);
#endif

    /// Set the participant name.
    /// Notes:
    /// - if the Chrono preCICE participant is created from a YAML specification file, its name is read from that file.
    void SetParticipantName(const std::string& participant_name) { m_participant_name = participant_name; }

    /// Add a coupling mesh with specified data type, using the given lists of data names for writing and reading.
    /// This function can be called more than once.
    /// Notes:
    /// - if the Chrono preCICE participant is created from a YAML specification file, interface information is read from that file.
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

    /// Receive data from other participant(s) via preCICE.
    /// The received data is then passed to the concrete participant (via `OnReadData`).
    void ReadData();

    /// Send data to other participant(s) via preCICE.
    /// The concrete participant first prepares the data to be sent (via `OnWriteData`).
    void WriteData();

    /// Set the (write) data vector for the specified mesh and data names.
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
    bool WriteCheckpoint(double time);

    /// Read the solver state from a checkpoint if required by preCICE.
    /// If requested by preCISE, this function invokes the solver-specific checkpoint reading function.
    /// The return value indicates whether a checkpoint was read.
    bool ReadCheckpoint(double time);

    // ---- Output

    /// Prepare output database (if needed) and let the concrete participant write output.
    void WriteOutput(int frame, double time);

    // ---- Participant/solver-specific functions to be implemented by derived classes

    /// Let the derived class perform any necessary operations during the simulation initialization.
    /// This function is called before the solver writes initial data (if requested) and before the preCICE coupling is initialized.
    /// After the call to InitializeParticipant, it is assumed that the coupling meshes have been set.
    virtual void InitializeParticipant() = 0;

    /// Process data received from other solvers.
    /// A derived class must:
    /// - perform any necessary processing of the data now stored in m_coupling_meshes
    /// - only access data from entries with names in m_data_read
    virtual void OnReadData() = 0;

    /// Prepare data to be sent to other solvers.
    /// A derived class must:
    /// - prepare the data to be sent and load it in m_coupling_meshes
    /// - only access data from entries with names in m_data_write
    virtual void OnWriteData() = 0;

    /// Read a previously saved checkpoint (if required by preCICE).
    /// The solver from a derived class must restore its state at the values in the last saved checkpoint and, if needed, reset its internal time to the provided value.
    virtual void OnReadCheckpoint(double time) = 0;

    /// Write a checkpoint (if required by preCICE).
    virtual void OnWriteCheckpoint(double time) = 0;

    /// Let the derived class implement the actual computation of the solver time step based on the maximum time step provided by preCICE.
    /// The default implementation simply returns the maximum time step provided by preCICE, but derived classes can override this to implement custom time-stepping logic.
    virtual double GetSolverTimeStep(double max_time_step) const { return max_time_step; }

    /// Let the derived class implement the actual solver time-stepping by the given time step.
    virtual void AdvanceParticipant(double time, double time_step) = 0;

    /// Let the derived class perform any necessary operations during simulation shutdown.
    /// This function is called before the preCICE coupling is finalized.
    virtual void FinalizeParticipant() {}

    /// Write output from the Chrono preCICE participant.
    /// A derived class must:
    /// - write output to the DB
    virtual void OnWriteOutput(int frame, double time) = 0;

    // ---- Common functions

    /// Save output at the current frame is output is enabled.
    /// This function ensures that output occurs at the specified frequency.
    void Output(double time);

    /// Render the current frame if run-time visualization is available and enabled.
    /// This function ensures that frames are rendered at the specified frequency.
    /// If requested, snapshots are saved to disk.
    void Render(double time);

    // ---- Utility functions

    /// Convert a vector of ChVector2d to a vector of doubles in the format (x0, y0, x1, y1, ...).
    static std::vector<double> SetVerticesToData(const std::vector<ChVector2d>& vertices);

    /// Convert a vector of ChVector3d to a vector of doubles in the format (x0, y0, z0, x1, y1, z1, ...).
    static std::vector<double> SetVerticesToData(const std::vector<ChVector3d>& vertices);

    /// Read a set of 3D points from an ASCII file (one point per line, space-delimited).
    static std::vector<ChVector3d> ReadPoints(const std::string& filename);

    // ---- Member variables

    /// Definition of a coupling data block.
    struct CouplingDataInfo {
        bool used;                   ///< data referenced in preCICE configuration
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

    std::string m_model_name;  ///< Chrono model name

    std::string m_precice_config_filename;                   ///< name of the preCICE configuration file
    bool m_use_added_mass;                                   ///<
    bool m_use_dynamic_added_mass;                           ///<

    std::unique_ptr<precice::Participant> m_participant;  ///< preCICE instance
    std::string m_participant_name;                       ///< name of the Chrono preCICE participant/solver

    CouplingMeshes m_coupling_meshes;  ///< data for all coupling meshes
    MeshDataNames m_data_read;         ///< input data names for all coupling meshes
    MeshDataNames m_data_write;        ///< output data names for all coupling meshes

    bool m_interfaces_created;   ///< true if the data interfaces were created
    bool m_participant_created;  ///< true if the preCICE participant was created
    bool m_initialized;          ///< true if preCICE participant was initialized

    bool m_verbose;         ///< verbose terminal output
    std::string m_prefix1;  ///< prefix for terminal messages (first line)
    std::string m_prefix2;  ///< prefix for terminal messages (subsequent lines)

    bool m_visualize;  ///< enable/disable run-time visualization

    bool m_output;                          ///< enable/disable run-time output
    std::string m_output_dir;               ///< output directory name
    ChOutput::Settings m_output_settings;   ///< output settings
    std::unique_ptr<ChOutput> m_output_db;  ///< output database

#ifdef CHRONO_VSG
    // Run-time visualization
    ChVisualSystem::Settings m_vis_settings;          ///< visualization parameters
    std::shared_ptr<vsg3d::ChVisualSystemVSG> m_vsg;  ///< run-time visualization system
#endif

#if defined(CHRONO_HAS_YAML)
    ChYamlFileHandler m_file_handler;  ///< handler for data file paths in YAML file
    bool m_use_degrees;                ///< angles provided in degrees in YAML file
#endif

  private:
    /// Parse preCICE configuration.
    /// - Cache mesh and data information,
    /// - Check if using added mass.
    void ParseXML();

    /// Process preCICE configuration information.
    /// - Check consistency with the Chrono adapter configuration,
    /// - Mark mesh data as used (referenced) or not.
    void ProcessXML();

    MeshDataNames m_defined_meshes;                          ///< meshes defined in preCICE configuration file
    std::map<std::string, MeshDataNames> m_provided_meshes;  ///< meshes provided by a participant

  protected:
    /// Return dimension of one added mass block.
    /// Called only is using dynamic added mass.
    virtual size_t GetNumFsiBodies() const;

    /// Process added mass data received via preCICE.
    /// Called only if using dynamic added mass on a receiving participant.
    /// Must be overridden if using dynamic added mass and this is a sending participant; the default implementation throws in that case.
    virtual void OnReadDataAM(const std::vector<ChMatrix66d>& blocks);

    /// Prepare added mass data to be sent via preCICE.
    /// Called only if using dynamic added mass on a sending participant.
    /// Must be overridden if using dynamic added mass and this is a sending participant; the default implementation throws in that case.
    /// On entry, `blocks` is sized to the number of FSI bodies (as reported by `GetNumFsiBodies`) and is zero-initialized.
    /// The override is expected to set the 6x6 added mass block for every FSI body; any block left untouched is sent as a zero block.
    virtual void OnWriteDataAM(std::vector<ChMatrix66d>& blocks);

  private:
    /// Create and register internal exchange mesh for added mass information.
    /// The approach used here is based on a "pseudo reference domain":
    /// - use a fictitious mesh with coordinates encoding the FSI body and matrix row and column in the corresponding added mass block.
    /// - define scalar data on this mesh (each value is an entry in the added mass diagonal block for a body).
    /// - apply a nearest-neighbor mapping to pass generic arrays representing updates to the added mass generalized mass matrix term.
    void RegisterMeshAM();

    /// Read added mass data (if using dynamic added mass and if on a receiving participant).
    /// The received data is then passed to the concrete participant (via `OnReadDataAM`).
    void ReadDataAM();

    /// Write added mass data (if using dynamic added mass and if on a receiving participant).
    /// The concrete participant first prepares the data to be sent (via `OnWriteDataAM`).
    void WriteDataAM();

    bool m_AMmesh_write = false;
    bool m_AMmesh_read = false;
    std::string m_AMmesh_name;
    std::vector<int> m_AMmesh_vertexIDs;
    std::vector<double> m_AMmesh_values;
};

/// @} precice_module

}  // end namespace ch_precice
}  // namespace chrono

#endif
