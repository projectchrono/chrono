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
// Hydraulic circuit models based on the paper:
//   Rahikainen, J., Gonzalez, F., Naya, M.A., Sopanen, J. and Mikkola, A.,
//   "On the cosimulation of multibody systems and hydraulic dynamics."
//   Multibody System Dynamics, 50(2), pp.143-167, 2020.
//
// and the lumped fluid approach from:
//   Watton, J., "Fluid Power Systems: Modeling, Simulation, Analog and
//   Microcomputer Control." Prentice-Hall, New York, 1989
// =============================================================================

#ifndef CH_HYDRAULIC_CIRCUIT_H
#define CH_HYDRAULIC_CIRCUIT_H

#include "chrono/physics/ChExternalDynamics.h"
#include "chrono/physics/ChBody.h"
#include "chrono/motion_functions/ChFunction.h"

namespace chrono {

typedef std::pair<double, double> double2;

/// ChHydraulicCylinder - a simple hydraulic cylinder
/// Schematic:
/// <pre>
///
///    ____________________
///   |  1  |  2           |        F+
///   |     |-------------------    -->
///   |_____|______________|        s+
///
///     l1         l2
///   <----><------------->
///           lTotal
///   <------------------->
///               s
///   <------------------------>
///
/// </pre>
class ChApi ChHydraulicCylinder {
  public:
    ChHydraulicCylinder();

    /// Set the piston and rod diameters.
    void SetDimensions(double piston_diameter, double rod_diameter);

    /// Set initial pressures in the cylinder chambers [N/m^2].
    void SetInitialChamberPressures(double pison_side, double rod_side);

    /// Set the initial location of the piston in the cylinder chamber [m].
    void SetInitialChamberLengths(double piston_side, double rod_side);

    /// Get the cross-section areas of the two chambers.
    const double2& GetAreas() const { return A; }

    /// Calculate current chamber lengths.
    double2 ComputeChamberLengths(double Delta_s) const;

    /// Calculate current chamber volumes.
    double2 ComputeChamberVolumes(const double2& L) const;

    /// Evaluate the force at the rod.
    double EvalForce(const double2& p, double Delta_s, double sd);

  private:
    double pistonD;  ///< piston diameter [m]
    double rodD;     ///< piston rod diameter [m]
    double pistonL;  ///< piston length [m]

    double2 p0;  ///< initial pressures (piston-side and rod-side) [Pa]
    double2 L0;  ///< initial lengths (piston-side and rod-side) [m]
    double2 A;   ///< areas (piston-side and rod-side) [m^2]

    bool length_exceeded;  ///< flag indicating whether piston past limits

    friend class ChHydraulicActuatorBase;
};

/// ChHydraulicDirectionalValve4x3 - a computational model of 4/3 directional valve
/// Schematic:
/// <pre>
///
///                      p4   p3
///                ________|_|__ ______
///               | |  ^ | T T | ^  / |
///            _ _| |  | |         X  |_ _
///  <-U+>    |_/_|_v__|_|_T_T_|_v__\_|_/_|
///                        | |
///                      p1   p2
///                  (tank)   (pump)
/// </pre>
class ChApi ChHydraulicDirectionalValve4x3 {
  public:
    ChHydraulicDirectionalValve4x3();

    /// Set valve parameters.
    void SetParameters(double linear_limit,  ///< laminar flow rate limit of 2 bar [N/m^2]
                       double dead_zone,     ///< limit for shut valve [m]
                       double fm45           ///< -45 degree phase shift frequency [Hz]
    );

    /// Set initial spool position: U0 = U(0).
    void SetInitialSpoolPosition(double U);

    /// Evaluate righ-hand side of spool position ODE: Ud = Ud(t, U).
    double EvaluateSpoolPositionRate(double t, double U, double Uref);

    /// Compute volume flows through the valve.
    double2 ComputeVolumeFlows(double p1, double p2, double p3, double p4, double U);

  private:
    double linear_limit;  ///< laminar flow rate limit of 2 bar [N/m^2]
    double dead_zone;     ///< limit for shut valve [m]
    double fm45;          ///< -45 degree phase shift frequency [Hz]

    double Cv;             ///< semi-empirical flow rate coefficient
    double time_constant;  ///< time-constant based on fm45 [s]

    double U0;  ///< initial spool position [m]

    friend class ChHydraulicActuatorBase;
};

/// ChHydraulicThrottleValve - a semi-empirical model of a throttle valve
/// Schematic:
/// <pre>
///
///     p2
///     |
///    )|( | Q
///     |
///     p1
///
/// </pre>
class ChApi ChHydraulicThrottleValve {
  public:
    ChHydraulicThrottleValve();

    /// Set valve parameters.
    void SetParameters(double valve_diameter,  ///< valve orifice diameter [m]
                       double oil_density,     ///< oil density [kg/m^3]
                       double linear_limit,    ///< laminar flow rate limit of 2 bar [N/m^2]
                       double Cd               ///< flow discharge coefficient of the orifice
    );

    /// Compute volume flow through the valve.
    double ComputeVolumeFlow(double p1, double p2);

  private:
    double Do;            ///< oil density [kg/m^3]
    double valveD;        ///< valve orifice diameter [m]
    double linear_limit;  ///< laminar flow rate limit of 2 bar [N/m^2]

    double Cd;  ///< flow discharge coefficient of the orifice
    double Cv;  ///< semi-empirical flow rate coefficient
};

}  // end namespace chrono

#endif
