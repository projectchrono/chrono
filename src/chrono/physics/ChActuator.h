// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Base class for linear actuators between two bodies.
//
// =============================================================================

#ifndef CH_ACTUATOR_H
#define CH_ACTUATOR_H

#include <array>

#include "chrono/physics/ChExternalDynamicsODE.h"
#include "chrono/physics/ChBody.h"
#include "chrono/functions/ChFunction.h"

namespace chrono {

/// Base class for a linear actuator acting between two bodies.
/// An actuator can be attached between two bodies, in which case the actuator length and length rate of change
/// are inferred from the states of those two bodies. Alternatively, an actuator can be instantiated stand-alone
/// (e.g., for use in a co-simulation setting), in which case the actuator length and rate must be provided from
/// outside.
class ChApi ChActuator : public ChExternalDynamicsODE {
  public:
    ~ChActuator() {}

    /// Set the actuation function.
    /// This function should return an actuator input, normalized to the interval [-1,1].
    void SetInputFunction(std::shared_ptr<ChFunction> fun) { ref_fun = fun; }

    /// Set actuator initial length [m].
    /// This value is used only for an actuator not attached to bodies. For a connected actuator, the initial length is
    /// inferred from the initial body positions.
    void SetActuatorInitialLength(double len);

    /// Set initial loading force.
    /// If provided, this value is used in calculating consistent initial conditions.
    void SetInitialLoad(double initial_load);

    /// Get current actuator input.
    /// Evaluate the provided actuator input function and clamp return value to [-1,1].
    double GetInput(double t) const;

    /// Initialize the actuator stand-alone.
    virtual void Initialize() override { ChExternalDynamicsODE::Initialize(); }

    /// Initialize this actuator by connecting it between the two specified bodies.
    void Connect(std::shared_ptr<ChBody> body1,  ///< first connected body
                 std::shared_ptr<ChBody> body2,  ///< second connected body
                 bool local,                     ///< true if locations given in body local frames
                 ChVector3d loc1,                ///< location of connection point on body 1
                 ChVector3d loc2                 ///< location of connection point on body 2
    );

    /// Get the endpoint location on 1st body (expressed in absolute coordinate system).
    /// Returns a zero location if the actuator is not attached to bodies.
    ChVector3d GetPoint1Abs() const { return m_aloc1; }

    /// Get the endpoint location on 2nd body (expressed in body coordinate system).
    /// Returns a zero location if the actuator is not attached to bodies.
    ChVector3d GetPoint2Abs() const { return m_aloc2; }

    /// Set the current actuator length and rate of change.
    /// Can be used in a co-simulation interface.
    void SetActuatorLength(double len, double vel);

    /// Get the current actuator force.
    /// Can be used in a co-simulation interface.
    virtual double GetActuatorForce() = 0;

  protected:
    ChActuator();

    /// Update the physics item at current state.
    virtual void Update(double time, bool update_assets) override final;

    /// Load generalized forces.
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override final;

    /// Insert the optional coupling KRM block if a derived class uses it.
    virtual void InjectKRMMatrices(ChSystemDescriptor& descriptor) override final;

    /// Load the ChExternalDynamicsODE KRM block and, if used, the coupling KRM block.
    virtual void LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) override final;

    /// Enable use of a coupling KRM block.
    /// If enabled, the coupling KRM block depends on the variables of the two connected bodies. In that case, the
    /// derived class must provide an implementation for the ComputeCouplingKRM function.
    /// Note that this feature can be used only if the actuator is attached (i.e., Connect was invoked).
    virtual bool EnableCouplingKRM() { return false; }

    /// Compute the coupling KRM block, if used.
    virtual void ComputeCouplingKRM(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
        if (m_KRM_coupling.GetNumVariables() > 0)
            throw std::runtime_error("ERROR: Derived actuator class does not provide ComputeCouplingKRM");
    }

    bool is_attached;            ///< true if actuator attached to bodies
    ChBody* m_body1;             ///< first conected body
    ChBody* m_body2;             ///< second connected body
    ChVector3d m_loc1;           ///< point on body 1 (local frame)
    ChVector3d m_loc2;           ///< point on body 2 (local frame)
    ChVector3d m_aloc1;          ///< point on body 1 (global frame)
    ChVector3d m_aloc2;          ///< point on body 2 (global frame)
    ChVectorDynamic<> m_Qforce;  ///< generalized forcing terms

    std::shared_ptr<ChFunction> ref_fun;  ///< actuation function

    double s_0;  ///< initial actuator length [m]
    double s;    ///< current actuator length [m]
    double sd;   ///< current actuator speed [m/s]

    bool calculate_consistent_IC;  ///< solve initialization nonlinear system
    double F0;                     ///< estimated initial load

    ChKRMBlock m_KRM_coupling;  ///< optional KRM block, coupling the two connected bodies
};

}  // end namespace chrono

#endif
