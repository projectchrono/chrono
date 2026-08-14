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
//
// Base classes for linear and rotational actuators between two bodies.
//
// =============================================================================

#ifndef CH_ACTUATOR_H
#define CH_ACTUATOR_H

#include "chrono/physics/ChExternalDynamicsODE.h"
#include "chrono/physics/ChBody.h"
#include "chrono/functions/ChFunction.h"

namespace chrono {

/// Base class for a linear actuator acting between two bodies.
/// An actuator can be attached between two bodies, in which case the actuator length and length rate of change
/// are inferred from the states of those two bodies. Alternatively, an actuator can be instantiated stand-alone
/// (e.g., for use in co-simulation), in which case the actuator length and rate must be provided from outside.
/// For generality, ChActuator allows internal actuator dynamics (hence, external to Chrono).
/// By default, this base class assumes no internal actuator states.
class ChApi ChActuator : public ChExternalDynamicsODE {
  public:
    virtual ~ChActuator() {}

    /// Set the actuation function.
    /// This function should return an actuator input, normalized to the interval [-1,1].
    void SetInputFunction(std::shared_ptr<ChFunction> fun) { ref_fun = fun; }

    /// Get current actuator input.
    /// Evaluate the provided actuator input function and clamp return value to [-1,1].
    double GetInput(double t) const;

    /// Initialize the actuator stand-alone.
    /// This function is called from Connect() if the actuator is attached to bodies.
    /// this function should be called explicitly only in a co-simulation setting.
    virtual void Initialize() override { ChExternalDynamicsODE::Initialize(); }

  protected:
    ChActuator();

    // Interface to ChExternalDynamicsODE -- default implementation assumes no internal dynamics

    /// Set number of internal states.
    /// The default implementation of this base class assumes no internal dynamics.
    virtual unsigned int GetNumStates() const override { return 0; }

    /// Set the initial conditions for internal states.
    /// The default implementation of this base class assumes no internal dynamics.
    virtual void SetInitialConditions(ChVectorDynamic<>& y0) override {}

    /// Calculate the right-hand side for internal dynamics.
    /// The default implementation of this base class assumes no internal dynamics.
    virtual void CalculateRHS(double time, const ChVectorDynamic<>& y, ChVectorDynamic<>& rhs) override {}

    /// Load generalized forces.
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override final;

    /// Insert the optional coupling KRM block if a derived class uses it.
    virtual void InjectKRMMatrices(ChSystemDescriptor& descriptor) override final;

    /// Load the ChExternalDynamicsODE KRM block and, if used, the coupling KRM block.
    virtual void LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) override final;

    // Coupling between connected bodies

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

    // Member variables

    bool is_attached;            ///< true if actuator attached to bodies
    ChBody* m_body1;             ///< first connected body
    ChBody* m_body2;             ///< second connected body
    ChVectorDynamic<> m_Qforce;  ///< generalized forcing terms

    std::shared_ptr<ChFunction> ref_fun;  ///< actuation function

    bool calculate_consistent_IC;  ///< solve initialization nonlinear system

    ChKRMBlock m_KRM_coupling;  ///< optional KRM block, coupling the two connected bodies
};

/// Base class for a linear actuator acting between two bodies.
/// An actuator can be attached between two bodies, in which case the actuator length and length rate of change
/// are inferred from the states of those two bodies. Alternatively, an actuator can be instantiated stand-alone
/// (e.g., for use in co-simulation), in which case the actuator length and rate must be provided from outside.
class ChApi ChLinearActuator : public ChActuator {
  public:
    virtual ~ChLinearActuator() {}

    /// Set actuator initial length.
    /// This value is used only for an actuator not attached to bodies. For a connected actuator, the initial length is
    /// inferred from the initial body positions.
    void SetActuatorInitialLength(double len);

    /// Set initial loading force.
    /// If provided, this value is used in calculating consistent initial conditions.
    void SetInitialLoad(double initial_load);

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

    /// Get the actuator force at the specified time.
    /// Can be used in a co-simulation interface.
    virtual double GetActuatorForce(double time) = 0;

  protected:
    ChLinearActuator();

    /// Update the physics item at current state.
    virtual void Update(double time, UpdateFlags update_flags) override final;

    ChVector3d m_loc1;   ///< point on body 1 (local frame)
    ChVector3d m_loc2;   ///< point on body 2 (local frame)
    ChVector3d m_aloc1;  ///< point on body 1 (global frame)
    ChVector3d m_aloc2;  ///< point on body 2 (global frame)

    double s_0;  ///< initial actuator length [m]
    double s;    ///< current actuator length [m]
    double sd;   ///< current actuator speed [m/s]

    double F0;  ///< estimated initial loading force
};

/// Base class for a rotational actuator acting between two bodies.
/// An actuator can be attached between two bodies, in which case the actuator angle and angle rate of change
/// are inferred from the states of those two bodies. Alternatively, an actuator can be instantiated stand-alone
/// (e.g., for use in co-simulation), in which case the actuator angle and rate must be provided from outside.
class ChApi ChRotationalActuator : public ChActuator {
  public:
    virtual ~ChRotationalActuator() {}

    /// Set actuator initial angle.
    /// This value is used only for an actuator not attached to bodies. For a connected actuator, the initial angle is
    /// inferred from the initial body positions.
    void SetActuatorInitialAngle(double angle);

    /// Set initial loading torque.
    /// If provided, this value is used in calculating consistent initial conditions.
    void SetInitialLoad(double initial_load);

    /// Initialize this actuator by connecting it between the two specified bodies.
    void Connect(std::shared_ptr<ChBody> body1,  ///< first connected body
                 std::shared_ptr<ChBody> body2,  ///< second connected body
                 const ChFrame<>& frame          ///< actuator frame orientation (in absolute reference frame)
    );

    /// Set the current actuator angle and rate of change.
    /// Can be used in a co-simulation interface.
    void SetActuatorAngle(double angle, double ang_vel);

    /// Get the current actuator torque.
    /// Can be used in a co-simulation interface.
    virtual double GetActuatorTorque(double time) = 0;

  protected:
    ChRotationalActuator();

    /// Update the physics item at current state.
    virtual void Update(double time, UpdateFlags update_flags) override final;

    double a_0;  ///< initial actuator angle [red]
    double a;    ///< current actuator angle [red]
    double ad;   ///< current actuator angular speed [red/s]

    double T0;  ///< estimated initial loading torque

  private:
    void CalcAngle();
    void AdjustAngle();

    // Joint frame orientations (in body local frames)
    ChCoordsys<> m_csys1;  ///< joint frame orientation on body 1
    ChCoordsys<> m_csys2;  ///< joint frame orientation on body 2
    ChVector3d m_axis;     ///< RSDA axis (expressed in absolute frame)
    int m_turns;           ///< number of revolutions
    double m_last_angle;   ///< angle at previous evaluation
};

}  // end namespace chrono

#endif
