// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Base class for a vehicle co-simulation node.
//
// =============================================================================

#ifndef CH_VEHCOSIM_BASENODE_H
#define CH_VEHCOSIM_BASENODE_H

#include <fstream>
#include <string>
#include <iostream>
#include <vector>

#include "chrono/core/ChTimer.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono_vehicle/ChApiVehicle.h"

#include "chrono_thirdparty/filesystem/path.h"

#define RIG_NODE_RANK 0
#define TERRAIN_NODE_RANK 1

namespace chrono {
namespace vehicle {

/// Base class for a co-simulation node.
class CH_VEHICLE_API ChVehicleCosimBaseNode {
  public:
    virtual ~ChVehicleCosimBaseNode() {}

    /// Set the integration step size (default: 1e-4).
    void SetStepSize(double step) { m_step_size = step; }

    /// Get the integration step size.
    double GetStepSize() const { return m_step_size; }

    /// Set the name of the output directory and an identifying suffix.
    /// Output files will be created in subdirectories named
    ///    dir_name/[NodeName]suffix/
    /// where [NodeName] is either "RIG" or "TERRAIN".
    void SetOutDir(const std::string& dir_name, const std::string& suffix);

    /// Enable/disable verbose messages during simulation (default: true).
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// Get the output directory name for this node.
    const std::string& GetOutDirName() const { return m_node_out_dir; }

    /// Get the simulation time for the current step on this node.
    double GetSimTime() const { return m_timer.GetTimeSeconds(); }

    /// Get the cumulative simulation time on this node.
    double GetTotalSimTime() const { return m_cum_sim_time; }

    /// Initialize this node.
    /// This function allows the node to initialize itself and, optionally, perform an
    /// initial data exchange with any other node.
    virtual void Initialize() = 0;

    /// Synchronize this node.
    /// This function is called at every co-simulation synchronization time to
    /// allow the node to exchange information with any other node.
    virtual void Synchronize(int step_number, double time) = 0;

    /// Advance simulation.
    /// This function is called after a synchronization to allow the node to advance
    /// its state by the specified time step.  A node is allowed to take as many internal
    /// integration steps as required, but no inter-node communication should occur.
    virtual void Advance(double step_size) = 0;

    /// Output logging and debugging data.
    virtual void OutputData(int frame) = 0;

    /// Write checkpoint to the specified file (which will be created in the output directory).
    virtual void WriteCheckpoint(const std::string& filename) const {}

    /// Utility function for creating an output file name.
    /// It generates and returns a string of the form "{dir}/{root}_{frame}.{ext}", where {frame} is printed using the
    /// format "%0{frame_digits}d".
    static std::string OutputFilename(const std::string& dir,
                                      const std::string& root,
                                      const std::string& ext,
                                      int frame,
                                      int frame_digits);

  protected:
    /// Mesh data
    struct MeshData {
        unsigned int nv;                       ///< number of vertices
        unsigned int nn;                       ///< number of normals
        unsigned int nt;                       ///< number of triangles
        std::vector<ChVector<>> verts;         ///< vertex positions (in local frame)
        std::vector<ChVector<>> norms;         ///< vertex normals (in local frame)
        std::vector<ChVector<int>> idx_verts;  ///< mesh vertex indices (connectivity)
        std::vector<ChVector<int>> idx_norms;  ///< mesh normal indices
    };

    /// Mesh state
    struct MeshState {
        std::vector<ChVector<>> vpos;  ///< vertex positions (in absolute frame)
        std::vector<ChVector<>> vvel;  ///< vertex velocities (in absolute frame)
    };

    /// Mesh contact information
    struct MeshContact {
        int nv;                          ///< number of vertices in contact
        std::vector<int> vidx;           ///< indices of vertices experiencing contact forces
        std::vector<ChVector<>> vforce;  ///< contact forces on mesh vertices
    };

  protected:
    ChVehicleCosimBaseNode(const std::string& name);

    double m_step_size;  ///< integration step size

    std::string m_name;          ///< name of the node
    std::string m_out_dir;       ///< top-level output directory
    std::string m_node_out_dir;  ///< node-specific output directory
    std::ofstream m_outf;        ///< output file stream

    ChTimer<double> m_timer;  ///< timer for integration cost
    double m_cum_sim_time;    ///< cumulative integration cost

    bool m_verbose;  ///< verbose messages during simulation?

    static const double m_gacc;
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
