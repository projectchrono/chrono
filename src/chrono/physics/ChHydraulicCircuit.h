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

typedef ChVectorN<double, 2> Vec2;
typedef ChVectorN<double, 3> Vec3;

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
    const Vec2& GetAreas() const { return A; }

    /// Calculate current chamber lengths.
    Vec2 ComputeChamberLengths(double Delta_s) const;

    /// Calculate current chamber volumes.
    Vec2 ComputeChamberVolumes(const Vec2& L) const;

    /// Evaluate the force at the rod.
    double EvalForce(const Vec2& p, double Delta_s, double sd);

  private:
    double pistonD;  ///< piston diameter [m]
    double rodD;     ///< piston rod diameter [m]
    double pistonL;  ///< piston length [m]

    Vec2 p0;  ///< initial pressures (piston-side and rod-side) [Pa]
    Vec2 L0;  ///< initial lengths (piston-side and rod-side) [m]
    Vec2 A;   ///< areas (piston-side and rod-side) [m^2]

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
    /// Construct a directional valve with default parameters.
    ChHydraulicDirectionalValve4x3();

    /// Set the parameters for the valve characteristic curve.
    /// The valve is modeled with a liniar characteristiv below the specified pressure difference threshold
    /// (corresponding to laminar flow) and a quadratic characteristic above that value.
    /// <pre>
    ///    Q = Cv * U * sqrt(delta_p)
    /// </pre>
    /// The valve spool position is usually controlled with a proportional magnet. Here, we consider a normalized input
    /// U (assumed to be in the interval [-1,+1]) with the extremes corresponding to the maximum input voltage
    /// (providing full opening of the valve). The semi-empirical flow rate constant is obtained from one operation
    /// point on the valve characteristic curve (e.g. knowing the nominal flow, with full opening, and the corresponding
    /// pressure difference). The default Cv value corresponds to a nominal flow (U = 1) of 24 l/min with a 35 bar
    /// pressure difference, the flow rate constant is (in SI units):
    /// <pre>
    ///    Cv = (24e-3 / 60) / (1.0 * sqrt(35e5))
    /// </pre>
    void SetCharacteristicParameters(double linear_limit,  ///< laminar flow rate limit of 2 bar [N/m^2]
                                     double Q,             ///< nominal flow (full opening) [m^3/s]
                                     double dp             ///< nominal presure difference [N/m^2]
    );

    /// Set the Bode diagram frequency at -45 degrees phase shift (default: 35 Hz).
    /// This value is used in calculating the valve time constant: tau = 1/2*pi*fm45.
    void SetTimeConstantFrequency(double fm45);

    /// Set the input threshold for shut valve (default: 1e-5).
    /// This is a non-dimensional quantity so that volume flows through the valve are set to 0 whenever the normalized
    /// input U is below this limit.
    void SetValveDeadZone(double dead_zone);

    /// Set the normalized initial spool position, U0 = U(0) (default: 0).
    void SetInitialSpoolPosition(double U);

    /// Evaluate righ-hand side of spool position ODE: Ud = Ud(t, U).
    /// The reference input is expected to be normalized in the [-1,+1] interval.
    double EvaluateSpoolPositionRate(double t, double U, double Uref);

    /// Compute volume flows through the valve.
    Vec2 ComputeVolumeFlows(double U, const Vec2& p, double pP, double pT);

  private:
    double linear_limit;  ///< laminar flow rate limit of 2 bar [N/m^2]
    double dead_zone;     ///< limit for shut valve [m]

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
