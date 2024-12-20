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
// Definition of the base vehicle co-simulation TRACKED MBS NODE class.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef CH_VEHCOSIM_TRACKED_MBS_NODE_H
#define CH_VEHCOSIM_TRACKED_MBS_NODE_H

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_vehicle/cosim/ChVehicleCosimBaseNode.h"
#include "chrono_vehicle/cosim/ChVehicleCosimDBPRig.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_cosim
/// @{

/// Base class for all MBS nodes with tracks.
class CH_VEHICLE_API ChVehicleCosimTrackedMBSNode : public ChVehicleCosimBaseNode {
  public:
    virtual ~ChVehicleCosimTrackedMBSNode();

    /// Return the node type as NodeType::MBS_TRACKED.
    virtual NodeType GetNodeType() const override { return NodeType::MBS_TRACKED; }

    /// Fix the chassis to ground (default: false).
    void SetChassisFixed(bool val) { m_fix_chassis = val; }

    /// Attach a drawbar pull rig to the MBS system.
    void AttachDrawbarPullRig(std::shared_ptr<ChVehicleCosimDBPRig> rig);

    /// Return the drawbar-pull rig, if one is attached.
    std::shared_ptr<ChVehicleCosimDBPRig> GetDrawbarPullRig() const;

    /// Initialize this node.
    /// This function allows the node to initialize itself and, optionally, perform an initial data exchange with any
    /// other node.
    virtual void Initialize() override final;

    /// Synchronize this node.
    /// This function is called at every co-simulation synchronization time to
    /// allow the node to exchange information with any other node.
    virtual void Synchronize(int step_number, double time) override final;

    /// Advance simulation.
    /// This function is called after a synchronization to allow the node to advance
    /// its state by the specified time step.  A node is allowed to take as many internal
    /// integration steps as required, but no inter-node communication should occur.
    virtual void Advance(double step_size) override final;

    /// Output logging and debugging data.
    virtual void OutputData(int frame) override final;

    /// Output post-processing visualization data.
    virtual void OutputVisualizationData(int frame) override final;

  protected:
    /// Construct a base class tracked MBS co-simulation node.
    /// By default, the underlying Chrono system is set yo use the Barzilai-Borwein solver and the Euler implicit
    /// linearized integrator. All OpenMP thread numbers are set to 1.
    ChVehicleCosimTrackedMBSNode();

    /// Initialize the underlying MBS
    virtual void InitializeMBS(const ChVector2d& terrain_size,  ///< terrain length x width
                               double terrain_height            ///< initial terrain height
                               ) = 0;

    /// Return terrain contact geometry and material information for one track shoe.
    virtual utils::ChBodyGeometry GetTrackShoeContactGeometry() const = 0;

    /// Return mass of one track shoe.
    virtual double GetTrackShoeMass() const = 0;

    /// Perform any required operations before advancing the state of the MBS.
    /// This function is called before every integration step.
    virtual void PreAdvance(double step_size) {}

    /// Perform any required operations after advancing the state of the MBS.
    /// This function is called after every integration step.
    virtual void PostAdvance(double step_size) {}

    /// Perform additional output at the specified frame (called from within OutputData).
    /// For example, output mechanism-specific data for post-procesing.
    virtual void OnOutputData(int frame) {}

    /// Apply the provided force to the specified track shoe body
    /// This function is called during synchronization when the force is received from the corresponding track node.
    virtual void ApplyTrackShoeForce(int track_id, int shoe_id, const TerrainForce& force) = 0;

    /// Get the number of track subsystems defined by the underlying MBS.
    virtual unsigned int GetNumTracks() const = 0;

    /// Return the number of track shoes in the specified track subsystem.
    virtual size_t GetNumTrackShoes(int track_id) const = 0;

    /// Return the total number of track shoes (in all track subsystems).
    virtual size_t GetNumTrackShoes() const = 0;

    /// Return the specified track shoe.
    virtual std::shared_ptr<ChBody> GetTrackShoeBody(int track_id, int shoe_id) const = 0;

    /// Get the body state of the specified track shoe body.
    virtual BodyState GetTrackShoeState(int track_id, int shoe_id) const = 0;

    /// Get the "chassis" body.
    virtual std::shared_ptr<ChBody> GetChassisBody() const = 0;

    /// Get the sprocket addendum radius (for slip calculation).
    virtual double GetSprocketAddendumRadius() const = 0;

    /// Impose sprocket angular speed function.
    /// This function is called (during initialization, after the call to InitializeMBS) only if a drawbar-pull rig is
    /// attached. A derived class must enforce the specified angular speed on all sprockets and, if appropriate,
    /// disconnect any other power transmission mechanism (such as a powertrain/driveline).
    virtual void OnInitializeDBPRig(std::shared_ptr<ChFunction> func) = 0;

  protected:
    ChSystemSMC* m_system;                            ///< containing system
    std::shared_ptr<ChVehicleCosimDBPRig> m_DBP_rig;  ///< DBP rig
    std::ofstream m_DBP_outf;                         ///< DBP output file stream

  private:
    virtual ChSystem* GetSystemPostprocess() const override { return m_system; }

    bool m_fix_chassis;
};

/// @} vehicle_cosim

}  // end namespace vehicle
}  // end namespace chrono

#endif
