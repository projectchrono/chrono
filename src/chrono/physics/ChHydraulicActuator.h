// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Hydraulic actuator model based on the paper:
//   Rahikainen, J., Gonzalez, F., Naya, M.A., Sopanen, J. and Mikkola, A.,
//   "On the cosimulation of multibody systems and hydraulic dynamics."
//   Multibody System Dynamics, 50(2), pp.143-167, 2020.
//
// and the lumped fluid approach from:
//   Watton, J., "Fluid Power Systems: Modeling, Simulation, Analog and
//   Microcomputer Control." Prentice-Hall, New York, 1989
// =============================================================================

#ifndef CH_HYDRAULIC_ACTUATOR_H
#define CH_HYDRAULIC_ACTUATOR_H

#include <array>

#include "chrono/physics/ChExternalDynamics.h"
#include "chrono/physics/ChHydraulicCircuit.h"
#include "chrono/physics/ChBody.h"
#include "chrono/motion_functions/ChFunction.h"

namespace chrono {

/// Base class for a hydraulic actuator.
///
/// All hydraulic actuators contain a cylinder and a directional valve. Derived classes may add additional components
/// and combine these in circuits with different numbers of fluid (oil) volumes.
///
/// A hydraulic actuator can be attached between two bodies, in which case the actuator length and length rate of change
/// is inferred from the states of those two bodies. Alternatively, a hydraulic actuator can be instantiated stand-alone
/// (e.g., for use in a co-simulation setting), in which case the actuator length and rate must be provided from
/// outside.
class ChApi ChHydraulicActuatorBase : public ChExternalDynamics {
  public:
    virtual ~ChHydraulicActuatorBase() {}

    /// Set the actuation function.
    /// This function should return the directional valve input, normalized to the interval [-1,1]. The extreme values
    /// correspond to the maximum input voltage (providing full opening of the valve) to the proportional magnet
    /// controlling the valve spool position. Note that the provided input is automatically clamped to [-1,1].
    void SetInputFunction(std::shared_ptr<ChFunction> fun) { ref_fun = fun; }

    /// Set the tank and pump pressures.
    void SetPressures(double pump_pressure, double tank_pressure);

    /// Set actuator initial length [m].
    /// This value is used only for an actuator not attached to bodies. For a connected actuator, the initial length is
    /// inferred from the initial body positions.
    void SetActuatorInitialLength(double len);

    /// Set initial loading force.
    /// If provided, this value is used in calculating consistent initial conditions, using the initial dircetional
    /// valve spool position and the initial cylinder pressures as initial guesses. Otherwise, the initial actuator
    /// state is set to the user specified values (which may be inconsistent with the configuration of the cylinder
    /// piston).
    void SetInitialLoad(double F0);

    /// Initialize the hydraulic actuator stand-alone.
    /// In this case, actuator position and rate are supposed to be provided from the outside.
    void Initialize();

    /// Initialize this hydraulic actuator by connecting it between the two specified bodies.
    void Initialize(std::shared_ptr<ChBody> body1,  ///< first connected body
                    std::shared_ptr<ChBody> body2,  ///< second connected body
                    bool local,                     ///< true if locations given in body local frames
                    ChVector<> loc1,                ///< location of connection point on body 1
                    ChVector<> loc2                 ///< location of connection point on body 2
    );

    /// Access the hydraulic cylinder in this circuit.
    ChHydraulicCylinder& Cylinder() { return cyl; }

    /// Access the directoinal valve in this circuit.
    ChHydraulicDirectionalValve4x3& DirectionalValve() { return dvalve; }

    /// Get the endpoint location on 1st body (expressed in absolute coordinate system).
    /// Returns a zero location if the actuator is not attached to bodies.
    ChVector<> GetPoint1Abs() const { return m_aloc1; }

    /// Get the endpoint location on 2nd body (expressed in body coordinate system).
    /// Returns a zero location if the actuator is not attached to bodies.
    ChVector<> GetPoint2Abs() const { return m_aloc2; }

    /// Set the current actuator length and rate of change.
    /// Can be used in a co-simulation interface.
    void SetActuatorLength(double len, double vel);

    /// Get the current actuator force.
    /// Can be used in a co-simulation interface.
    double GetActuatorForce();

    /// Get the current cylinder pressures.
    std::array<double, 2> GetCylinderPressures();

    /// Get the current directional valve position.
    double GetValvePosition();

  protected:
    ChHydraulicActuatorBase();

    /// Process initial cylinder pressures and initial valve displacement.
    /// Optionally, if the initial load F0 is provided, a derived class may calculate consistent initial conditions
    /// using the provided values as an initial guess.
    virtual void OnInitialize(const Vec2& cyl_p0, const Vec2& cyl_L0, double dvalve_U0) = 0;

    /// Extract directional valve spool position from current state.
    virtual double ExtractValveSpoolPosition() const = 0;

    /// Extract cylinder pressures from current state.
    virtual Vec2 ExtractCylinderPressures() const = 0;

    /// Get current actuator input.
    double GetInput(double t) const;

    /// Declare the EOM of this physics item as stiff or non-stiff.
    virtual bool IsStiff() const override { return true; }

    /// Update the physics item at current state.
    virtual void Update(double time, bool update_assets = true) override;

    /// Load generalized forces.
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;

    bool is_attached;            ///< true if actuator attached to bodies
    ChBody* m_body1;             ///< first conected body
    ChBody* m_body2;             ///< second connected body
    ChVector<> m_loc1;           ///< point on body 1 (local frame)
    ChVector<> m_loc2;           ///< point on body 2 (local frame)
    ChVector<> m_aloc1;          ///< point on body 1 (global frame)
    ChVector<> m_aloc2;          ///< point on body 2 (global frame)
    ChVectorDynamic<> m_Qforce;  ///< generalized forcing terms

    ChHydraulicCylinder cyl;                ///< hydraulic cylinder
    ChHydraulicDirectionalValve4x3 dvalve;  ///< directional valve

    std::shared_ptr<ChFunction> ref_fun;  ///< actuation function (spool displacement reference)

    double s_0;  ///< initial actuator length [m]
    double s;    ///< current actuator length [m]
    double sd;   ///< current actuator speed [m/s]

    double pP;  ///< pump pressure [Pa]
    double pT;  ///< tank pressure [Pa]

    bool calculate_consistent_IC;  ///< solve initialization nonlinear system
    double F0;                     ///< estimated initial load
};

// -----------------------------------------------------------------------------

/// Hydraulic actuator using a circuit with 2 volumes.
/// Schematic:
/// <pre>
///
///                  piston      rod
///                   side      side
///                 ___________________
///                |   1     |     2   |
///  cylinder      |         |----------------
///                |___^_____|_____^___|
///                    |           |
///                    |           |
///                    | hose1     | hose2
///                    |           |
///                   _|___________|_
///  directional     |       X       |
///     valve        |___|____ __|___|
///                      |       |
///                    pump     tank
///
/// </pre>
/// Components:
///   - 1 pump and 1 tank, modeled as constant pressures
///   - 2 hoses
///   - 1 cylinder
///   - 1 directional valve 4x3
/// States:
///   - y(0): U -- dvalve spool position
///   - y(1): p1 -- cylinder pressure 1 (piston-side)
///   - y(2): p2 -- cylinder pressure 2 (rod-side)
class ChApi ChHydraulicActuator2 : public ChHydraulicActuatorBase {
  public:
    ChHydraulicActuator2();

    /// Set the bulk modulus for oil, hoses, and cylinder.
    void SetBulkModuli(double oil_bulk_modulus, double hose_bulk_modulus, double cyl_bulk_modulus);

    /// Set the volumes of the two hoses in this circuit.
    void SetHoseVolumes(double hose_dvalve_piston, double hose_dvalve_rod);

  private:
    // Interface to ChExternalDynamics

    virtual int GetNumStates() const override { return 1 + 2; }

    virtual void SetInitialConditions(ChVectorDynamic<>& y0) override;

    virtual void CalculateRHS(double time,                 ///< current time
                              const ChVectorDynamic<>& y,  ///< current ODE states
                              ChVectorDynamic<>& rhs       ///< output ODE right-hand side vector
                              ) override;

    virtual bool CalculateJac(double time,                   ///< current time
                              const ChVectorDynamic<>& y,    ///< current ODE states
                              const ChVectorDynamic<>& rhs,  ///< current ODE right-hand side vector
                              ChMatrixDynamic<>& J           ///< output Jacobian matrix
                              ) override;

    /// Process initial cylinder pressures and initial valve displacement.
    virtual void OnInitialize(const Vec2& cyl_p0, const Vec2& cyl_L0, double dvalve_U0) override;

    /// Extract directional valve spool position from current state.
    virtual double ExtractValveSpoolPosition() const override;

    /// Extract cylinder pressures from current state.
    virtual Vec2 ExtractCylinderPressures() const override;

    /// Evaluate pressure rates at curent state.
    Vec2 EvaluatePressureRates(double t, const Vec2& p, double U);

    Vec2 pc0;   ///< initial cylinder pressures
    double U0;  ///< initial dvalve spool displacement

    double hose1V;  ///< hose 1 volume [m^3]
    double hose2V;  ///< hose 2 volume [m^3]

    double Bo;  ///< oil bulk modulus [Pa]
    double Bh;  ///< hose bulk modulus [Pa]
    double Bc;  ///< cylinder bulk modulus [Pa]
};

// -----------------------------------------------------------------------------

/// Hydraulic actuator using a circuit with 3 volumes.
/// Schematic:
/// <pre>
///
///                   piston      rod
///                    side      side
///                 ___________________
///                |   1     |     2   |
///  cylinder      |         |----------------
///                |___^_____|_____^___|
///                    |           |
///                    | hose1     |
///                    |           | hose2
///  throttle         )|(          |
///   valve            |           |
///                    | hose3     |
///                   _|___________|_
///  directional     |       X       |
///     valve        |___|____ __|___|
///                      |       |
///                    pump     tank
///
/// </pre>
/// Components:
///   - 1 pump and 1 tank, modeled as constant pressures
///   - 3 hoses
///   - 1 cylinder
///   - 1 directional valve 4x3
///   - 1 throttle valve
/// States:
///   - y(0): U -- spool position
///   - y(1): p1 -- cylinder pressure 1
///   - y(2): p2 -- cylinder pressure 2
///   - y(3): p3 -- hose pressure
class ChApi ChHydraulicActuator3 : public ChHydraulicActuatorBase {
  public:
    ChHydraulicActuator3();

    /// Set the bulk modulus for oil, hoses, and cylinder.
    void SetBulkModuli(double oil_bulk_modulus, double hose_bulk_modulus, double cyl_bulk_modulus);

    /// Set the volumes of the three hoses in this circuit.
    void SetHoseVolumes(double hose_tvalve_piston, double hose_dvalve_rod, double hose_dvalve_tvalve);

    /// Access the throttle valve in this circuit.
    ChHydraulicThrottleValve& ThrottleValve() { return tvalve; }

  private:
    // Interface to ChExternalDynamics

    virtual int GetNumStates() const override { return 1 + 3; }

    virtual void SetInitialConditions(ChVectorDynamic<>& y0) override;

    virtual void CalculateRHS(double time,                 ///< current time
                              const ChVectorDynamic<>& y,  ///< current ODE states
                              ChVectorDynamic<>& rhs       ///< output ODE right-hand side vector
                              ) override;

    virtual bool CalculateJac(double time,                   ///< current time
                              const ChVectorDynamic<>& y,    ///< current ODE states
                              const ChVectorDynamic<>& rhs,  ///< current ODE right-hand side vector
                              ChMatrixDynamic<>& J           ///< output Jacobian matrix
                              ) override;

    /// Process initial cylinder pressures and initial valve displacement.
    virtual void OnInitialize(const Vec2& cyl_p0, const Vec2& cyl_L0, double dvalve_U0) override;

    /// Extract directional valve spool position from current state.
    virtual double ExtractValveSpoolPosition() const override;

    /// Extract cylinder pressures from current state.
    virtual Vec2 ExtractCylinderPressures() const override;

    /// Evaluate pressure rates at curent state.
    Vec3 EvaluatePressureRates(double t, const Vec3& p, double U);

    ChHydraulicThrottleValve tvalve;  ///< throttle valve

    Vec2 pc0;   ///< initial cylinder pressures
    double U0;  ///< initial dvalve spool displacement

    double hose1V;  ///< hose 1 volume [m^3]
    double hose2V;  ///< hose 2 volume [m^3]
    double hose3V;  ///< hose 3 volume [m^3]

    double Bo;  ///< oil bulk modulus [Pa]
    double Bh;  ///< hose bulk modulus [Pa]
    double Bc;  ///< cylinder bulk modulus [Pa]
};

}  // end namespace chrono

#endif
