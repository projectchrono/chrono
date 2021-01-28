// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Simone Benatti
// =============================================================================
//
// Base class for copter model +
// Little Hexy model
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChForce.h"
#include "chrono_models/ChApiModels.h"

#ifndef COPTERS_H
#define COPTERS_H

namespace chrono {
/// Namespace with classes for the Copters model.
namespace copters {

// Number of propellers
template <int nop>
class CH_MODELS_API ChCopter {
  public:
    ChCopter(ChSystem& sys,                  /// the Chrono physical system
             ChVector<>& cpos,               /// Chassis position
             std::vector<ChVector<>>& ppos,  /// Propeller relative position
             const bool clockwise[],         /// i-th propeller rotates clockwise -> true
             bool are_prop_pos_rel = true,   /// if false, propeller axes position has to be given in the abs frame
             bool z_up = false);

    ~ChCopter() {}

    virtual std::shared_ptr<ChBody> GetChassis() { return chassis; }

    virtual void SetPropellersData(int mass,
                           ChVector<>& inerXX,
                           double diam,
                           double thrust_coeff,
                           double power_coeff,
                           double max_rpm);

    virtual void SetLinearDragCoeff(double ldc);

    virtual double GetLinearDragCoeff() { return Cd; }

    virtual void AddVisualizationAssets(std::string& chassismesh,
                                std::string& propellermesh,
                                ChFrame<> cor_m1,
                                ChFrame<> cor_m2);

    // Increment Propellers omegas
    virtual void ControlIncremental(double inputs[nop]);

    // Set Propellers omegas
    virtual void ControlAbsolute(double inputs[nop]);

    // Update the copter
    virtual void Update(double timestep);

    virtual void SetInitAirData(double rho0, double p0) {
        rho = rho0;
        pressure = p0;
    }

    virtual double GetAirPressure() { return pressure; }
	 
    virtual double GetGroundAirPressure() { return pressure0; }
	 
    virtual void SetGroundPressure(double p0) { pressure0 = p0; }
	 
    virtual double GetAirDensity() { return rho; }
	 
    virtual double GetAltitude() { return Altitude; }
	 
    virtual double GetInitAltitude() { return Altitude0; }
	 
    virtual void SetInitAltitude(double alt) { Altitude0 = alt; }
	 
    virtual double GetTemperature() { return Temp; }
	 
    virtual void SetTemperature(double temp) { Temp = temp; }
	 
    virtual double GetGroundTemperature() { return Temp0; }
	 
    virtual void SetGroundTemperature(double temp) { Temp0 = temp; }

  private:
    // Virtual method. Might be overridden for special condition (e.g. Mars atmosphere)
    // This model holds below 11 km altitude.
    void UpdateAirData();

  protected:
    // Chassis body
    std::shared_ptr<ChBody> chassis;
    // Visualization meshes
    std::string chassis_mesh_file;
    std::string prop_mesh_file;
    // Propellers bodies
    std::vector<std::shared_ptr<ChBody>> props;
    // Propellers position
    // std::vector<std::shared_ptr<ChBody>> prop_pos;
    // Max propeller rotations per SECOND (rps)
    double rps_max;
    // Propeller diameter [m]
    double Dp;
    // Thrust coefficient
    double Ct;
    // Power coefficient
    double Cp;
    // Propeller rotation as fraction of max rpm, 0<=ui<=1
    double u_p[nop];
    // Air density [kg/m^3]
    double rho = 1.225;
    // Air pressure [Pa]
    double pressure = 101325;
    // Ground Air pressure [Pa]
    double pressure0 = 101325;
    // Altitude [m]
    double Altitude;
    // Ground  Air Temperature [K]
    double Temp0 = 298;
    // Air Temperature [K]
    double Temp = 298;
    // Initial Altitude [m]
    double Altitude0 = 0;
    // Initial Altitude in simulation [m]
    double h0 = 0;
    // Vertical axis
    ChVector<> up;
    // Thrust and torques to be applied to propellers
    // Thrust Forces
    std::vector<std::shared_ptr<ChForce>> thrusts;
    // Propeller Resistance torques
    std::vector<std::shared_ptr<ChForce>> backtorques;
    // Propeller Motors
    std::vector<std::shared_ptr<ChLinkMotorRotationSpeed>> motors;
    // Propeller Motors Speed Functions
    std::vector<std::shared_ptr<ChFunction_Const>> speeds;
    // Linear drag
    double Cd = 0;  /// Drag coefficient
    std::shared_ptr<ChForce> lin_drag;
    // Drag Surface
    double Surf = 0;
    // Drag torque, neglected
    // std::shared_ptr<ChForce> drag_tor;
};

class CH_MODELS_API Little_Hexy : public ChCopter<6> {
  public:
    Little_Hexy(ChSystem& sys, ChVector<> cpos);

    // Add visualization shapes
    void AddVisualizationAssets();

    // Add collision shapes
    // The collision shape is a boundary box, anything more sophisticated is probably an overkill
    void AddCollisionShapes(std::shared_ptr<ChMaterialSurface> material);

    void Pitch_Down(double delta);

    void Pitch_Up(double delta);

    void Roll_Right(double delta);

    void Roll_Left(double delta);

    void Yaw_Right(double delta);

    void Yaw_Left(double delta);

    void Throttle(double delta);

  protected:
    static std::vector<ChVector<>> getPosVect();

  private:
    static const constexpr bool spins[6] = {false, true, false, true, false, true};
    std::string chassis_mesh_path = "copters/hexi_body.obj";
    std::string propeller_mesh_path = "copters/prop.obj";
    
};
}
}  // namespace chrono

#endif
