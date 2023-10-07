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
//   Rahikainen, J., González, F., Naya, M.Á., Sopanen, J. and Mikkola, A.,
//   "On the cosimulation of multibody systems and hydraulic dynamics."
//   Multibody System Dynamics, 50(2), pp.143-167, 2020.
//
// and the lumped fluid approach from:
//   Watton, J., "Fluid Power Systems: Modeling, Simulation, Analog and
//   Microcomputer Control." Prentice-Hall, New York, 1989
// =============================================================================

#ifndef CH_HYDRAULIC_ACTUATOR_H
#define CH_HYDRAULIC_ACTUATOR_H

#include "chrono/physics/ChExternalDynamics.h"
#include "chrono/physics/ChHydraulicCircuit.h"
#include "chrono/physics/ChBody.h"
#include "chrono/motion_functions/ChFunction.h"

namespace chrono {

/// Base class for a hydraulic actuator.
/// This base class implements the models of the various components. All hydraulic actuators contain a cylinder; derived
/// classes may add additional components and combine these in circuits with different numbers of fluid (oil) volumes.
/// A hydraulic actuator can be attached between two bodies, in which case the actuator length and length rate of change
/// is inferred from the states of those two bodies. Alternatively, a hydraulic actuator can be instantiated stand-alone
/// (e.g., for use in a co-simulation setting), in which case the actuator length and rate must be provided from
/// outside.
class ChApi ChHydraulicActuatorBase : public ChExternalDynamics {
  public:
    virtual ~ChHydraulicActuatorBase() {}

    /// Set the actuation function.
    void SetInputFunction(std::shared_ptr<ChFunction> fun) { ref_fun = fun; }

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

    /// Set actuator initial length.
    /// This value is used only for an actuator not attached to bodies. For a connected actuator, the initial length is
    /// inferred from the initial body positions.
    void SetActuatorInitialLength(double len_0);

    /// Set the current actuator length and rate of change.
    /// Can be used in a co-simulation interface.
    void SetActuatorLength(double t, double len, double vel);

    /// Get the actuator force.
    /// Can be used in a co-simulation interface.
    double GetActuatorForce(double t);

  protected:
    ChHydraulicActuatorBase();

    virtual bool IsStiff() const override { return false; }

    /// Get current actuator input.
    double GetInput(double t) const;

    /// Extract cylinder pressures from current state.
    virtual double2 GetCylinderPressure() const = 0;

    /// Update the physics item at current state.
    virtual void Update(double time, bool update_assets = true) override;

    /// Load generalized forces.
    void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c);

    bool m_is_attached;  ///< true if actuator attached to bodies

    ChBody* m_body1;             ///< first conected body
    ChBody* m_body2;             ///< second connected body
    ChVector<> m_body1_loc;      ///< location on body 1
    ChVector<> m_body2_loc;      ///< location on body 2
    ChVectorDynamic<> m_Qforce;  ///< generalized forcing terms

    ChHydraulicCylinder cyl;  ///< hydraulic cylinder

    std::shared_ptr<ChFunction> ref_fun;
    double s_0;  ///< initial actuator length
    double s;    ///< current actuator length
    double sd;   ///< current actuator speed
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
    ChHydraulicActuator2() {}

    void SetPressures(double pump_pressure, double tank_pressure);
    void SetBulkModuli(double oil_bulk_modulus, double hose_bulk_modulus, double cyl_bulk_modulus);
    void SetHoseVolumes(double hose_dvalve_piston, double hose_dvalve_rod);

    ChHydraulicCylinder& Cylinder() { return cyl; }
    ChHydraulicDirectionalValve4x3& DirectionalValve() { return dvalve; }

  private:
    typedef ChVectorN<double, 2> Vec2;

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

    /// Extract cylinder pressures from current state.
    virtual double2 GetCylinderPressure() const override;

    Vec2 EvaluatePressureRates(double t, const Vec2& p, double U);

    ChHydraulicDirectionalValve4x3 dvalve;  ///< directional valve

    double hose1V = 3.14e-5;  // hose 1 volume [m^3]
    double hose2V = 7.85e-5;  // hose 2 volume [m^3]

    double pP = 7.6e6;  // pump pressure [Pa]
    double pT = 0.1e6;  // tank pressure [Pa]

    double Bo = 1500e6;   // oil bulk modulus [Pa]
    double Bh = 150e6;    // hose bulk modulus [Pa]
    double Bc = 31500e6;  // cylinder bulk modulus [Pa]
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
    ChHydraulicActuator3() {}

    void SetPressures(double pump_pressure, double tank_pressure);
    void SetBulkModuli(double oil_bulk_modulus, double hose_bulk_modulus, double cyl_bulk_modulus);
    void SetHoseVolumes(double hose_tvalve_piston, double hose_dvalve_rod, double hose_dvalve_tvalve);

    ChHydraulicCylinder& Cylinder() { return cyl; }
    ChHydraulicDirectionalValve4x3& DirectionalValve() { return dvalve; }
    ChHydraulicThrottleValve& ThrottleValve() { return tvalve; }

  private:
    typedef ChVectorN<double, 3> Vec3;

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

    /// Extract cylinder pressures from current state.
    virtual double2 GetCylinderPressure() const override;

    Vec3 EvaluatePressureRates(double t, const Vec3& p, double U);

    ChHydraulicDirectionalValve4x3 dvalve;  ///< directional valve
    ChHydraulicThrottleValve tvalve;        ///< throttle valve

    double hose1V = 3.14e-5;  // hose 1 volume [m^3]
    double hose2V = 7.85e-5;  // hose 2 volume [m^3]
    double hose3V = 4.71e-5;  // hose 3 volume [m^3]

    double pP = 7.6e6;  // pump pressure [Pa]
    double pT = 0.1e6;  // tank pressure [Pa]

    double Bo = 1500e6;   // oil bulk modulus [Pa]
    double Bh = 150e6;    // hose bulk modulus [Pa]
    double Bc = 31500e6;  // cylinder bulk modulus [Pa]
};

}  // end namespace chrono

#endif
