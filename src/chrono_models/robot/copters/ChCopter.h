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
// Base class for copter model
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChForce.h"
#include "chrono_models/ChApiModels.h"
#include "chrono/assets/ChTriangleMeshShape.h"

#ifndef COPTERS_H
#define COPTERS_H

namespace chrono {
/// Namespace with classes for the Copters model.
namespace copters {

// ChCopter is a base class for any copter, template parameter nop is the number of propellers.
// Propellers position, data, rotation verse, vertical axis and all mass porperties are settable
// Number of propellers
template <int nop>
class ChCopter {
  public:
    ChCopter(ChSystem& sys,                  /// the Chrono physical system
             ChVector<>& cpos,               /// Chassis position
             std::vector<ChVector<>>& ppos,  /// Propeller relative position
             const bool clockwise[],         /// i-th propeller rotates clockwise -> true
             bool are_prop_pos_rel = true,   /// if false, propeller axes position has to be given in the abs frame
             bool z_up = false);

    ~ChCopter() {}

	/// Get the chassis body
    virtual std::shared_ptr<ChBody> GetChassis() { return chassis; }

	/// Set the propellers properties 
	///coefficient as eq 6.29 of "Aerodynamics, Aeronautics, and Flight Mechanics" by McCormick 
    virtual void SetPropellersData(int mass,		/// Propellers mass 
                           ChVector<>& inerXX,		/// Propellers diagonal inertia (axis assumed principal)
                           double diam,				/// Propellers diameter
                           double thrust_coeff,		/// Propellers thrust coefficient
                           double power_coeff,		///	Propellers power coefficient
                           double max_rpm);			///	Propellers maximum revolutions per minute

	/// Set and get the coeffidient and surface for the evaluation of drag coefficient
    virtual void SetLinearDragCoeff(double ldc) { Cd = ldc; }
    virtual double GetLinearDragCoeff() { return Cd; }
	virtual void SetLinearDragSurf(double dsurf) { Surf = dsurf; }
    virtual double GetLinearDragSurf() { return Surf; }

	// Add generics triangular meshes to the chassis and the propellers
    virtual void AddVisualizationAssets(std::string& chassismesh,
                                std::string& propellermesh,
                                ChFrame<> cor_m1,
                                ChFrame<> cor_m2);

    // Increment Propellers omegas
    virtual void ControlIncremental(double inputs[nop]);

    // Set Propellers omegas
    virtual void ControlAbsolute(double inputs[nop]);

    // Update the copter internal physics
    virtual void Update(double timestep);

	// Set initial (h = 0) air density
    virtual void SetGroundAirDensity(double rho) { rho0 = rho; }

	// Get the air pressure at copter height
    virtual double GetAirPressure() { return pressure; }
	 
	// Get the air pressure at 0 height
    virtual double GetGroundAirPressure() { return pressure0; }
	 
	// Set the air pressure at copter height
    virtual void SetGroundPressure(double p0) { pressure0 = p0; }
	 
	// Get the air density at copter height
    virtual double GetGroundAirDensity() { return rho0; }

	// Get the air density at copter height
    virtual double GetAirDensity() { return rho; }
	
	// Get the copter altitude 
    virtual double GetAltitude() { return Altitude; }
	
	// Get the initial copter altitude 
    virtual double GetInitAltitude() { return Altitude0; }
	
	// Set the initial copter altitude 
    virtual void SetInitAltitude(double alt) { Altitude0 = alt; }
	 
	// Get air temperature
    virtual double GetTemperature() { return Temp; }
	
	// Set air temperature
    virtual void SetTemperature(double temp) { Temp = temp; }
	
	// Get ground air temperature
    virtual double GetGroundTemperature() { return Temp0; }
	
	// Set ground air temperature
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
    // Ground Air density [kg/m^3]
    double rho0 = 1.225;
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

template <int nop>
ChCopter<nop>::ChCopter(
    ChSystem& sys,                  /// the Chrono physical system
    ChVector<>& cpos,               /// Chassis position
    std::vector<ChVector<>>& ppos,  /// Propeller relative position
    const bool clockwise[],         /// i-th propeller rotates clockwise -> true
    bool are_prop_pos_rel = true,   /// if false, propeller axes position has to be given in the abs frame
    bool z_up = false) {
    // TODO: ChBodyAuxRef here might be more convenient
    up = (z_up) ? VECT_Z : VECT_Y;
    chassis = chrono_types::make_shared<ChBody>();
    chassis->SetPos(cpos);
    // placeholder Data.
    chassis->SetMass(10);
    chassis->SetInertiaXX(ChVector<>(1, 1, 1));
    chassis->SetBodyFixed(false);
    sys.AddBody(chassis);
    h0 = chassis->GetPos() ^ up;
    // 26.4 inch propellers
    for (int p = 0; p < nop; p++) {
        auto prop = chrono_types::make_shared<ChBody>();
        props.push_back(prop);
        if (are_prop_pos_rel) {
            prop->SetPos(cpos + ppos[p]);
        } else {
            prop->SetPos(ppos[p]);
        }
        // Data from little hexy, page 132.
        prop->SetMass(1);
        prop->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
        prop->SetBodyFixed(false);
        sys.AddBody(prop);

        auto propmot = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        ChQuaternion<> motor_rot = Q_ROTATE_Y_TO_Z;
        if (z_up) {
            motor_rot = QUNIT;
        };
        if (clockwise[p]) {
            motor_rot = Q_FLIP_AROUND_X * motor_rot;
        };
        propmot->Initialize(prop, chassis, ChFrame<>(ppos[p], motor_rot));
        auto speed = chrono_types::make_shared<ChFunction_Const>(0);
        propmot->SetSpeedFunction(speed);
        sys.AddLink(propmot);

        motors.push_back(propmot);
        speeds.push_back(speed);

        u_p[p] = 0;
        auto thrust = chrono_types::make_shared<ChForce>();
        prop->AddForce(thrust);
        thrust->SetMode(ChForce::FORCE);
        thrust->SetMforce(0);
        thrust->SetRelDir(up);
        thrusts.push_back(thrust);

        auto backtorque = std::make_shared<ChForce>();
        backtorque->SetBody(prop.get());
        backtorque->SetMode(ChForce::TORQUE);
        backtorque->SetMforce(0);
        // Resistance Torque direction opposed to omega
        ChVector<> tdir = (clockwise) ? up : -up;
        backtorque->SetRelDir(tdir);
        backtorques.push_back(backtorque);
        //
    }

    // linear drag on copter body
    lin_drag = std::make_shared<ChForce>();
    lin_drag->SetBody(chassis.get());
    lin_drag->SetMode(ChForce::FORCE);
    lin_drag->SetMforce(0);
}

template <int nop>
void ChCopter<nop>::SetPropellersData(int mass,
                                      ChVector<>& inerXX,
                                      double diam,
                                      double thrust_coeff,
                                      double power_coeff,
                                      double max_rpm) {
    Dp = diam;
    Ct = thrust_coeff;
    Cp = power_coeff;
    rps_max = max_rpm / 60;
    for (auto& prop : props) {
        prop->SetMass(mass);
        prop->SetInertiaXX(inerXX);
    }
}

template <int nop>
void ChCopter<nop>::AddVisualizationAssets(std::string& chassismesh,
                                           std::string& propellermesh,
                                           ChFrame<> cor_m1,
                                           ChFrame<> cor_m2) {
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->LoadWavefrontMesh(chassismesh, true, false);
    trimesh->Transform(cor_m1.GetPos(), cor_m1.GetA());
    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    // trimesh_shape->SetName(m_mesh_name);
    trimesh_shape->SetStatic(true);
    chassis->AddAsset(trimesh_shape);

    for (auto propeller : props) {
        auto prop_trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        prop_trimesh->LoadWavefrontMesh(propellermesh, true, false);
        prop_trimesh->Transform(cor_m2.GetPos(), cor_m2.GetA());
        auto trimesh_prop_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_prop_shape->SetMesh(prop_trimesh);
        // trimesh_prop_shape->SetName(m_mesh_name);
        trimesh_prop_shape->SetStatic(true);
        propeller->AddAsset(trimesh_prop_shape);
    }
}

// Increment Propellers omegas
template <int nop>
void ChCopter<nop>::ControlIncremental(double inputs[nop]) {
    for (int i = 0; i < nop; i++) {
        u_p[i] = ChClamp(u_p[i] + inputs[i], -1.0, 1.0);
        speeds[i]->Set_yconst(u_p[i] * rps_max * CH_C_2PI);
    }
}

// Set Propellers omegas
template <int nop>
void ChCopter<nop>::ControlAbsolute(double inputs[nop]) {
    for (int i = 0; i < nop; i++) {
        u_p[i] = ChClamp<double>(inputs[i], -1, 1);
        speeds[i]->Set_yconst(u_p[i] * rps_max * CH_C_2PI);
    }
}

// Update the copter
template <int nop>
void ChCopter<nop>::Update(double timestep) {
    // update propeller forces/torques
    for (int i = 0; i < nop; i++) {
        double rps = motors[i]->GetMotorRot_dt() / CH_C_2PI;
        thrusts[i]->SetMforce(Ct * rho * pow(rps, 2) * pow(Dp, 4));
        backtorques[i]->SetMforce((1 / CH_C_2PI) * Cp * rho * pow(rps, 4) * pow(Dp, 5));
    }
    // update linear drag / drag torque
    lin_drag->SetMforce(0.5 * Cd * Surf * rho * chassis->GetPos_dt().Length2());
    lin_drag->SetDir(-chassis->GetPos_dt());
    // update pressure, temperature, altitude:
    UpdateAirData();
}

// Virtual method. Might be overridden for special condition (e.g. Mars atmosphere)
// This model holds below 11 km altitude.
template <int nop>
void ChCopter<nop>::UpdateAirData() {
    Altitude = Altitude0 + (chassis->GetPos() ^ up - h0);
    Temp = Temp0 - (6.5 * (Altitude / 1000));
    pressure = pressure0 * pow((Temp0 / Temp), -5.255877);
}

}
}  // namespace chrono

#endif
