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

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChForce.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono_models/ChApiModels.h"

#ifndef COPTER_H
#define COPTER_H

namespace chrono {
/// Namespace with classes for the Copter models.
namespace copter {

/// @addtogroup robot_models_copter
/// @{

/// Base class for any copter, template parameter nop is the number of propellers.
/// Propellers position, data, rotation verse, vertical axis and all mass porperties are settable.
template <int nop>
class Copter {
  public:
    Copter(ChSystem& sys,                 ///< containing physical system
           const ChVector<>& cpos,        ///< chassis position
           std::vector<ChVector<>> ppos,  ///< propeller relative position
           const bool clockwise[],        ///< rotation direction ofr each propeller
           bool are_prop_pos_rel = true,  ///< if false, propeller axes position has to be given in the abs frame
           bool z_up = false              ///< orientation of vertical axis
    );

    /*virtual*/ ~Copter() {}

    /// Get the chassis body
    std::shared_ptr<ChBody> GetChassis() const { return chassis; }

    /// Set the propeller properties.
    /// Coefficient as eq 6.29 of "Aerodynamics, Aeronautics, and Flight Mechanics" by McCormick.
    void SetPropellerData(double mass,               ///< propeller mass
                          const ChVector<>& inerXX,  ///< propeller diagonal inertia (axis assumed principal)
                          double diam,               ///< propeller diameter
                          double thrust_coeff,       ///< propeller thrust coefficient
                          double power_coeff,        ///< propeller power coefficient
                          double max_rpm             ///< propeller maximum revolutions per minute
    );

    /// Set the drag coefficient.
    void SetLinearDragCoeff(double ldc) { Cd = ldc; }

    /// Get the drag coefficient.
    double GetLinearDragCoeff() const { return Cd; }

    /// Set the area used in evaluation of drag force.
    void SetLinearDragSurf(double dsurf) { Surf = dsurf; }

    /// Get the area used in evaluation of drag force.
    double GetLinearDragSurf() const { return Surf; }

    /// Add generic triangular meshes to the chassis and the propellers.
    void AddVisualizationAssets(const std::string& chassismesh,
                                const std::string& propellermesh,
                                const ChFrame<>& cor_m1,
                                const ChFrame<>& cor_m2);

    /// Increment propeller angular velocity.
    void ControlIncremental(double inputs[nop]);

    /// Set Propellers angular velocity.
    void ControlAbsolute(double inputs[nop]);

    /// Update the copter internal physics.
    void Update(double timestep);

    /// Set initial (h = 0) air density.
    void SetGroundAirDensity(double rho) { rho0 = rho; }

    /// Get the air pressure at copter height.
    double GetAirPressure() const { return pressure; }

    /// Get the air pressure at 0 height.
    double GetGroundAirPressure() const { return pressure0; }

    /// Set the air pressure at copter height.
    void SetGroundPressure(double p0) { pressure0 = p0; }

    /// Get the air density at copter height.
    double GetGroundAirDensity() const { return rho0; }

    /// Get the air density at copter height.
    double GetAirDensity() const { return rho; }

    /// Get the copter altitude.
    double GetAltitude() const { return Altitude; }

    /// Get the initial copter altitude.
    double GetInitAltitude() const { return Altitude0; }

    /// Set the initial copter altitude.
    void SetInitAltitude(double alt) { Altitude0 = alt; }

    /// Get air temperature.
    double GetTemperature() const { return Temp; }

    /// Set air temperature.
    void SetTemperature(double temp) { Temp = temp; }

    /// Get ground air temperature.
    double GetGroundTemperature() const { return Temp0; }

    /// Set ground air temperature.
    void SetGroundTemperature(double temp) { Temp0 = temp; }

	/// Get the number of propellers.
    int GetNumProps() const { return nop; }

	/// Get the propellers bodies.
    std::vector<std::shared_ptr<ChBody>> GetProps() const { return props; }

	/// Get the name of the Wavefront file with chassis visualization mesh.
    /// An empty string is returned if no mesh was specified.
    virtual const std::string& GetChassisMeshFilename() const { return chassis_mesh_path; }

	/// Get the name of the Wavefront file with propeller visualization mesh.
    /// An empty string is returned if no mesh was specified.
    virtual const std::string& GetPropellerMeshFilename() const { return propeller_mesh_path; }

	/// Rotates the whole copter given a 3x3 rotation matrix.
    void RotateCopter(ChMatrix33<>& A);
  private:
    // Would need to be modified for special condition (e.g. Mars atmosphere)
    // This model holds below 11 km altitude.
    void UpdateAirData();

  protected:
    std::shared_ptr<ChBody> chassis;                    ///< Chassis body
    std::string chassis_mesh_path;                      ///< Visualization meshes
    std::string propeller_mesh_path;                    ///< Visualization meshes
    std::vector<std::shared_ptr<ChBody>> props;         ///< Propeller bodies
    double rps_max;                                     ///< Max propeller angular speed [rot/s]
    double Dp;                                          ///< Propeller diameter [m]
    double Ct;                                          ///< Thrust coefficient
    double Cp;                                          ///< Power coefficient
    double u_p[nop];                                    ///< Propeller rotation as fraction of max rpm (in [0,1])
    double rho = 1.225;                                 ///< Air density [kg/m^3]
    double rho0 = 1.225;                                ///< Ground Air density [kg/m^3]
    double pressure = 101325;                           ///< Air pressure [Pa]
    double pressure0 = 101325;                          ///< Ground Air pressure [Pa]
    double Altitude;                                    ///< Altitude [m]
    double Temp0 = 298;                                 ///< Ground  Air Temperature [K]
    double Temp = 298;                                  ///< Air Temperature [K]
    double Altitude0 = 0;                               ///< Initial Altitude [m]
    double h0 = 0;                                      ///< Initial Altitude in simulation [m]
    ChVector<> up;                                      ///< Vertical axis
    std::vector<std::shared_ptr<ChForce>> thrusts;      ///< Thrust Forces
    std::vector<std::shared_ptr<ChForce>> backtorques;  ///< Propeller Resistance torques
    std::vector<std::shared_ptr<ChLinkMotorRotationSpeed>> motors;  ///< Propeller Motors
    std::vector<std::shared_ptr<ChFunction_Const>> speeds;          ///< Propeller Motors Speed Functions
    double Cd = 0;                                                  ///< Drag coefficient
    std::shared_ptr<ChForce> lin_drag;                              ///< Linear drag
    double Surf = 0;                                                ///< Drag Surface
};

template <int nop>
Copter<nop>::Copter(ChSystem& sys,
                    const ChVector<>& cpos,
                    std::vector<ChVector<>> ppos,
                    const bool clockwise[],
                    bool are_prop_pos_rel,
                    bool z_up) {
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
        
        if (are_prop_pos_rel) {
            propmot->Initialize(prop, chassis, ChFrame<>(cpos + ppos[p], motor_rot)); 
        } else {
            propmot->Initialize(prop, chassis, ChFrame<>(ppos[p], motor_rot));
        }
        auto speed = chrono_types::make_shared<ChFunction_Const>(0);
        propmot->SetSpeedFunction(speed);
        sys.AddLink(propmot);

        motors.push_back(propmot);
        speeds.push_back(speed);

        u_p[p] = 0;
        auto thrust = chrono_types::make_shared<ChForce>();
        thrust->SetBody(prop.get());
        prop->AddForce(thrust);
        thrust->SetMode(ChForce::FORCE);
        thrust->SetMforce(0);
        thrust->SetRelDir(up);
        thrusts.push_back(thrust);

        auto backtorque = std::make_shared<ChForce>();
        backtorque->SetBody(prop.get());
        prop->AddForce(backtorque);
        backtorque->SetMode(ChForce::TORQUE);
        backtorque->SetMforce(0);
        // Resistance Torque direction opposed to omega
        ChVector<> tdir = (clockwise[p]) ? up : -up;
        backtorque->SetRelDir(tdir);
        backtorques.push_back(backtorque);
    }

    // linear drag on copter body
    lin_drag = std::make_shared<ChForce>();
    lin_drag->SetBody(chassis.get());
    lin_drag->SetMode(ChForce::FORCE);
    lin_drag->SetMforce(0);
}

template <int nop>
void Copter<nop>::SetPropellerData(double mass,
                                   const ChVector<>& inerXX,
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
void Copter<nop>::AddVisualizationAssets(const std::string& chassismesh,
                                         const std::string& propellermesh,
                                         const ChFrame<>& cor_m1,
                                         const ChFrame<>& cor_m2) {
    auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(chassismesh, true, true);
    auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetMutable(false);
    trimesh_shape->SetColor(ChColor(0.2f, 0.32f, 0.48f));
    chassis->AddVisualShape(trimesh_shape, cor_m1);

    for (auto propeller : props) {
        auto prop_trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(propellermesh, true, true);
        auto trimesh_prop_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_prop_shape->SetMesh(prop_trimesh);
        trimesh_prop_shape->SetMutable(false);
        trimesh_prop_shape->SetColor(ChColor(0.8f, 0.68f, 0.52f));
        propeller->AddVisualShape(trimesh_prop_shape, cor_m2);
    }
}

// Increment Propellers omegas
template <int nop>
void Copter<nop>::ControlIncremental(double inputs[nop]) {
    for (int i = 0; i < nop; i++) {
        u_p[i] = ChClamp(u_p[i] + inputs[i], -1.0, 1.0);
        speeds[i]->Set_yconst(u_p[i] * rps_max * CH_C_2PI);
    }
}

// Set Propellers omegas
template <int nop>
void Copter<nop>::ControlAbsolute(double inputs[nop]) {
    for (int i = 0; i < nop; i++) {
        u_p[i] = ChClamp<double>(inputs[i], -1, 1);
        speeds[i]->Set_yconst(u_p[i] * rps_max * CH_C_2PI);
    }
}

// Update the copter
template <int nop>
void Copter<nop>::Update(double timestep) {
    // update propeller forces/torques
    for (int i = 0; i < nop; i++) {
        double rps = motors[i]->GetMotorRot_dt() / CH_C_2PI;
        thrusts[i]->SetMforce(Ct * rho * pow(rps, 2) * pow(Dp, 4));
        backtorques[i]->SetMforce((1 / CH_C_2PI) * Cp * rho * pow(rps, 2) * pow(Dp, 5));
    }
    // update linear drag / drag torque
    lin_drag->SetMforce(0.5 * Cd * Surf * rho * chassis->GetPos_dt().Length2());
    lin_drag->SetDir(-chassis->GetPos_dt());
    // update pressure, temperature, altitude:
    UpdateAirData();
}

// Would need to be modified for special condition (e.g. Mars atmosphere)
// This model holds below 11 km altitude.
template <int nop>
void Copter<nop>::UpdateAirData() {
    Altitude = Altitude0 + (chassis->GetPos() ^ up - h0);
    Temp = Temp0 - (6.5 * (Altitude / 1000));
    pressure = pressure0 * pow((Temp0 / Temp), -5.255877);
}

template <int nop>
void Copter<nop>::RotateCopter(ChMatrix33<>& A) {
    ChMatrix33<> mrot = this->GetChassis()->GetA() * A.transpose();
    this->GetChassis()->SetRot(mrot);
    for (auto prop : this->GetProps()) {
        ChMatrix33<> proprot = prop->GetA() * A.transpose();
        prop->SetRot(proprot);
        ChVector<> deltap = A * (prop->GetPos() - this->GetChassis()->GetPos());
        prop->SetPos(prop->GetPos() + deltap);
    }
}

/// @} robot_models_copter

}  // namespace copter
}  // namespace chrono

#endif
