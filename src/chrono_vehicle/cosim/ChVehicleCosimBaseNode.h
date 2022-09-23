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

#include <mpi.h>

#include "chrono/core/ChTimer.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChVehicleGeometry.h"

#include "chrono_thirdparty/filesystem/path.h"

#define MBS_NODE_RANK 0
#define TERRAIN_NODE_RANK 1
#define TIRE_NODE_RANK(i) (i + 2)

namespace chrono {
namespace vehicle {

/** @addtogroup vehicle_cosim
 *
 * The vehicle co-simulation module provides an MPI_based framework for co-simulating a multibody system representing a
 * wheeled or tracked mechanism with various terrain models and optionally various tire models.
 * It implements a 3-way explicit force-displacement co-simulation approach.
 * The three different types of nodes present in a co-simulation are as follows:
 * - MBS node, a single MPI rank which simulates the multibody system.
 * - Tire nodes, a number of MPI ranks (equal to the number of wheels), each simulating one of the tires.
 * - Terrain node(s), one or more MPI ranks which simulate the deformable terrain.
 *
 * For a wheeled system, the inter-node communication at each synchronization time is as follows:
 * - MBS node sends spindle body state to corresponding Tire nodes
 * - Tire nodes send spindle forces to MBS node
 * - Tire nodes send tire state (rigid body state or deformable mesh state) to Terrain node
 * - Terrain node sends tire forces (single resultant force or distributed vertex forces) to corresponding Tire node
 *
 * For a tracked system, the inter-node communication at each synchronization time is at follows:
 * - MBS node sends track shoe states to the Terrain node
 * - Terrain node sends forces acting on track shoes to the MBS node
 *
 * The communication interface between Tire and Terrain nodes or between tracked MBS and Terrain nodes can be of one of
 * two types:
 * - ChVehicleCosimBaseNode::InterfaceType::BODY, in which force-displacement data for a single rigid body is exchanged
 * - ChVehicleCosimBaseNode::InterfaceType::MESH, in which force-displacement data for a deformable mesh is exchanged
 */

/// @addtogroup vehicle_cosim
/// @{

// =============================================================================

namespace cosim {

/// Initialize the co-simulation framework.
/// This function creates an MPI communicator that includes all nodes designated of type TERRAIN.
/// Calling this framework initialization function is optional. If invoked, it *must* be called on all ranks.
/// Returns MPI_SUCCESS if successful and MPI_ERR_OTHER if there are not enough ranks.
CH_VEHICLE_API int InitializeFramework(int num_tires);

/// Return true if the co-simulation framework was initialized and false otherwise.
CH_VEHICLE_API bool IsFrameworkInitialized();

/// Return the MPI communicator for distributed terrain simulation.
/// This intra-communicator is created if more than one node is designated of type TERRAIN.
/// On a TERRAIN node, the rank within the intra-communicator is accessible through MPI_Comm_rank.
CH_VEHICLE_API MPI_Comm GetTerrainIntracommunicator();

};  // namespace cosim

// =============================================================================

/// Base class for a co-simulation node.
class CH_VEHICLE_API ChVehicleCosimBaseNode {
  public:
    /// Type of node participating in co-simulation
    enum class NodeType {
        MBS_WHEELED,  ///< node performing multibody dynamics (wheeled vehicle)
        MBS_TRACKED,  ///< node performing multibody dynamics (tracked vehicle)
        TERRAIN,      ///< node performing terrain simulation
        TIRE          ///< node performing tire simulation
    };

    /// Type of the tire-terrain communication interface.
    /// - A BODY interface assumes communication is done at the wheel spindle or track shoe level.  At a synchronization
    /// time, the terrain node receives the full state of the spindle or track shoe body and must send forces acting on
    /// that body, for each tire or track shoe present in the simulation.  This type of interface should be used for
    /// track shoes, rigid tires, or when the terrain node also performs the dynamics of a flexible tire.
    /// - A MESH interface assumes communication is done at the tire mesh level. At a synchronization time, the terrain
    /// node receives the tire mesh vertex states (positions and velocities) are must send forces acting on vertices of
    /// the mesh, for each object. This interface is typically used when flexible tires are simulated outside the
    /// terrain node (on separate tire nodes).
    enum class InterfaceType {
        BODY,  ///< exchange state and force for a single body (wheel spindle or track shoe)
        MESH   ///< exchange state and force for a mesh (flexible tire mesh)
    };

    virtual ~ChVehicleCosimBaseNode() {}

    /// Return the node type.
    virtual NodeType GetNodeType() const = 0;

    /// Return the node type as a string.
    std::string GetNodeTypeString() const;

    /// Return true if this node is part of the co-simulation infrastructure.
    bool IsCosimNode() const;

    /// Set the integration step size (default: 1e-4).
    void SetStepSize(double step) { m_step_size = step; }

    /// Get the integration step size.
    double GetStepSize() const { return m_step_size; }

    /// Set the name of the output directory and an identifying suffix.
    /// Output files will be created in subdirectories named
    ///    dir_name/[NodeName]suffix/
    /// where [NodeName] is "MBS", "TIRE", or "TERRAIN".
    void SetOutDir(const std::string& dir_name, const std::string& suffix);

    /// Enable/disable verbose messages during simulation (default: true).
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// Get the output directory name for this node.
    const std::string& GetOutDirName() const { return m_node_out_dir; }

    /// Get the simulation execution time for the current step on this node.
    /// This represents the time elapsed since the last synchronization point.
    double GetStepExecutionTime() const { return m_timer.GetTimeSeconds(); }

    /// Get the cumulative simulation execution time on this node.
    double GetTotalExecutionTime() const { return m_cum_sim_time; }

    /// Initialize this node.
    /// This function allows the node to initialize itself and, optionally, perform an initial data exchange with any
    /// other node. A derived class implementation should first call this base class function.
    virtual void Initialize();

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

    /// Output post-processing visualization data.
    /// If implemented, this function should write a file in the "visualization" subdirectory of m_node_out_dir.
    virtual void OutputVisualizationData(int frame) = 0;

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
    /// Mesh state information (sent to terrain node)
    struct MeshState {
        std::vector<ChVector<>> vpos;  ///< vertex positions (in absolute frame)
        std::vector<ChVector<>> vvel;  ///< vertex velocities (in absolute frame)
    };

    /// Mesh contact information (received from terrain node)
    struct MeshContact {
        int nv;                          ///< number of vertices in contact
        std::vector<int> vidx;           ///< indices of vertices experiencing contact forces
        std::vector<ChVector<>> vforce;  ///< contact forces on mesh vertices
    };

  protected:
    ChVehicleCosimBaseNode(const std::string& name);

    void SendGeometry(const ChVehicleGeometry& geom, int dest) const;
    void RecvGeometry(ChVehicleGeometry& geom, int source) const;

    int m_rank;  ///< MPI rank of this node (in MPI_COMM_WORLD)

    double m_step_size;  ///< integration step size

    std::string m_name;          ///< name of the node
    std::string m_out_dir;       ///< top-level output directory
    std::string m_node_out_dir;  ///< node-specific output directory
    std::ofstream m_outf;        ///< output file stream

    unsigned int m_num_wheeled_mbs_nodes;
    unsigned int m_num_tracked_mbs_nodes;
    unsigned int m_num_terrain_nodes;
    unsigned int m_num_tire_nodes;

    ChTimer<double> m_timer;  ///< timer for integration cost
    double m_cum_sim_time;    ///< cumulative integration cost

    bool m_verbose;  ///< verbose messages during simulation?

    static const double m_gacc;
};

/// @} vehicle_cosim

}  // end namespace vehicle
}  // end namespace chrono

#endif
