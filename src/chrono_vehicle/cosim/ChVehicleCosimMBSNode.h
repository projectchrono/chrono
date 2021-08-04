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
// Definition of the base vehicle co-simulation MBS NODE class.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef CH_VEHCOSIM_MBS_NODE_H
#define CH_VEHCOSIM_MBS_NODE_H

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_vehicle/cosim/ChVehicleCosimBaseNode.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_cosim
/// @{

/// Base class for all MBS nodes.
class CH_VEHICLE_API ChVehicleCosimMBSNode : public ChVehicleCosimBaseNode {
  public:
    virtual ~ChVehicleCosimMBSNode();

    /// Return the node type as NodeType::MBS.
    virtual NodeType GetNodeType() const override { return NodeType::MBS; }

    /// Set the number of OpenMP threads used in Chrono simulation (default: 1).
    void SetNumThreads(int num_threads);

    /// Set integrator and solver types.
    /// For the MKL solver, use slv_type = ChSolver::Type::CUSTOM.
    void SetIntegratorType(ChTimestepper::Type int_type,  ///< integrator type (default: HHT)
                           ChSolver::Type slv_type        ///< solver type (default:: MKL)
    );

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

  protected:
    /// Construct a base class MBS node.
    ChVehicleCosimMBSNode();

    /// Initialize the underlying MBS
    virtual void InitializeMBS(const std::vector<ChVector<>>& tire_info,  ///< mass, radius, width for each tire
                               const ChVector2<>& terrain_size,           ///< terrain length x width
                               double terrain_height                      ///< initial terrain height
                               ) = 0;

    /// Perform any required operations before advancing the state of the MBS.
    /// This function is called before every integration step.
    virtual void PreAdvance() {}

    /// Perform any required operations after advancing the state of the MBS.
    /// This function is called after every integration step.
    virtual void PostAdvance() {}

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

  protected:
    ChSystemSMC* m_system;                           ///< containing system
    ChTimestepper::Type m_int_type;                  ///< integrator type
    ChSolver::Type m_slv_type;                       ///< solver type
    std::shared_ptr<ChTimestepperHHT> m_integrator;  ///< HHT integrator object

    std::vector<std::shared_ptr<ChBody>> m_spindles;  ///< spindle bodies

  private:
    void InitializeSystem();
};

/// @} vehicle_cosim

}  // end namespace vehicle
}  // end namespace chrono

#endif
