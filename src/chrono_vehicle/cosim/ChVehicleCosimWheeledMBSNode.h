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
// Definition of the base vehicle co-simulation WHEELED MBS NODE class.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef CH_VEHCOSIM_WHEELED_MBS_NODE_H
#define CH_VEHCOSIM_WHEELED_MBS_NODE_H

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_vehicle/cosim/ChVehicleCosimBaseNode.h"
#include "chrono_vehicle/cosim/ChVehicleCosimDBPRig.h"

namespace chrono {
namespace vehicle {

/** @addtogroup vehicle_cosim_mbs
 *
 * This module defines concrete multibody system nodes:
 * - ChVehicleCosimRigNode wraps a single-wheel test rig.
 * - ChVehicleCosimWheeledVehicleNode wraps a Chrono::Vehicle wheeled vehicle defined through JSON specification files.
 * - ChVehicleCosimCuriosityNode wraps the Curiosity Mars rover model.
 * - ChVehicleCosimViperNode wraps the Viper lunar rover model.
 *
 * Additionally, this module provides various drawbar pull rigs which can be attached to any of the MBS nodes:
 * - ChVehicleCosimDBPRigImposedSlip allows imposing known (fixed) vehicle forward linear velocity and wheel angular velocity to maintain
 * a prescribed value of the longitudinal slip. The actuation specifies if the linear velocity or angular velocity is
 * considered as "base velocity", with the other one derived from the slip value. The DBP force is extracted as the
 * reaction force required to enforce the vehicle forward linear velocity (at steady state).  Each run of this
 * experiment produces one point on the slip-DBP curve.
 * - ChVehicleCosimDBPRigImposedAngVel enforces a prescribed wheel angular velocity. A linearly increasing resistive force is applied
 * against the forward motion of the vehicle and the experiment is ended when the vehicle stops. At each time, the
 * vehicle forward speed and resulting slip are calculated and stored together with the current resistive force (DBP).
 * This experiment produces the entire slip-DBP curve at once.
 */

/// @addtogroup vehicle_cosim
/// @{

/// Base class for all MBS nodes with wheels.
class CH_VEHICLE_API ChVehicleCosimWheeledMBSNode : public ChVehicleCosimBaseNode {
  public:
    virtual ~ChVehicleCosimWheeledMBSNode();

    /// Return the node type as NodeType::MBS_WHEELED.
    virtual NodeType GetNodeType() const override { return NodeType::MBS_WHEELED; }

    /// Set the number of OpenMP threads used in Chrono simulation (default: 1).
    void SetNumThreads(int num_threads);

    /// Set integrator and solver types.
    /// For the MKL solver, use slv_type = ChSolver::Type::CUSTOM.
    void SetIntegratorType(ChTimestepper::Type int_type,  ///< integrator type (default: HHT)
                           ChSolver::Type slv_type        ///< solver type (default:: MKL)
    );

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
    /// Construct a base class wheeled MBS node.
    ChVehicleCosimWheeledMBSNode();

    /// Initialize the underlying MBS
    virtual void InitializeMBS(const ChVector2<>& terrain_size,  ///< terrain length x width
                               double terrain_height             ///< initial terrain height
                               ) = 0;

    /// Apply tire info (after InitializeMBS).
    /// This includes mass, radius, and width for each tire.
    virtual void ApplyTireInfo(const std::vector<ChVector<>>& tire_info) = 0;

    /// Perform any required operations before advancing the state of the MBS.
    /// This function is called before every integration step.
    virtual void PreAdvance(double step_size) {}

    /// Perform any required operations after advancing the state of the MBS.
    /// This function is called after every integration step.
    virtual void PostAdvance(double step_size) {}

    /// Perform additional output at the specified frame (called from within OutputData).
    /// For example, output mechanism-specific data for post-procesing.
    virtual void OnOutputData(int frame) {}

    /// Apply the provided force to the i-th spindle body.
    /// This function is called during synchronization when the force is received from the corresponding tire node.
    virtual void ApplySpindleForce(unsigned int i, const TerrainForce& spindle_force) = 0;

    /// Get the number of spindles/wheels defined by the underlying MBS.
    /// A co-simulation must have a matching number of TIRE nodes.
    virtual int GetNumSpindles() const = 0;

    /// Get the spindle body to which the i-th wheel/tire is attached.
    virtual std::shared_ptr<ChBody> GetSpindleBody(unsigned int i) const = 0;

    /// Get the load weight on the i-th wheel/tire.
    /// Note: this must also include the mass of the tire itself.
    virtual double GetSpindleLoad(unsigned int i) const = 0;

    /// Get the body state of the spindle body to which the i-th wheel/tire is attached.
    virtual BodyState GetSpindleState(unsigned int i) const = 0;

    /// Get the "chassis" body.
    /// Only used is a drawbar-pull rig is attached.
    virtual std::shared_ptr<ChBody> GetChassisBody() const = 0;

    /// Impose spindle angular speed function.
    /// This function is called (during initialization, after the call to InitializeMBS) only if a drawbar-pull rig is
    /// attached. A derived class must enforce the specified angular speed on all spindles and, if appropriate,
    /// disconnect any other power transmission mechanism (such as a powertrain/driveline).
    virtual void OnInitializeDBPRig(std::shared_ptr<ChFunction> func) = 0;

  protected:
    ChSystemSMC* m_system;                            ///< containing system
    ChTimestepper::Type m_int_type;                   ///< integrator type
    ChSolver::Type m_slv_type;                        ///< solver type
    std::shared_ptr<ChTimestepperHHT> m_integrator;   ///< HHT integrator object
    std::shared_ptr<ChVehicleCosimDBPRig> m_DBP_rig;  ///< DBP rig
    std::ofstream m_DBP_outf;                         ///< DBP output file stream

  private:
    virtual ChSystem* GetSystemPostprocess() const override { return m_system; }
    void InitializeSystem();

    bool m_fix_chassis;
};

/// @} vehicle_cosim

}  // end namespace vehicle
}  // end namespace chrono

#endif
