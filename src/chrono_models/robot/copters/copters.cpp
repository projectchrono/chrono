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

#include "chrono_models/robot/copters/copters.h"
#include "chrono/assets/ChTriangleMeshShape.h"

// Use the namespaces of Chrono
namespace chrono {
namespace copters {

// Number of propellers
template <int nop>
ChCopter<nop>::ChCopter(ChSystem& sys,                  /// the Chrono physical system
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
void ChCopter<nop>::SetLinearDragCoeff(double ldc) {
    Cd = ldc;
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
    lin_drag->SetDir(chassis->GetPos_dt());
    // update rotor internal physics: Magnetic field, air pressure (gravity managed by chrono)
    UpdateAirData();
    // update sensors: gps, camera, magnetometer, altitude
}

// Virtual method. Might be overridden for special condition (e.g. Mars atmosphere)
// This model holds below 11 km altitude.
template <int nop>
void ChCopter<nop>::UpdateAirData() {
    Altitude = Altitude0 + (chassis->GetPos() ^ up - h0);
    Temp = Temp0 - (6.5 * (Altitude / 1000));
    pressure = pressure0 * pow((Temp0 / Temp), -5.255877);
}




Little_Hexy::Little_Hexy(ChSystem& sys, ChVector<> cpos) : ChCopter<6>(sys, cpos, getPosVect(), spins, true, true) {
    chassis->SetMass(13.83457);
    chassis->SetInertiaXX(ChVector<>(1.264, 1.277, 1.541));
	this->SetPropellersData(0.126,                                     /// Propeller mass
                            ChVector<>(0.004739, 0.004739, 0.004739),  /// Propeller Inertia
                            0.6718,                                    /// Propeller Diameter [m]
                            0.0587,                                    /// Propeller Thrust Coefficient
                            0.018734,                                  /// Propeller Power Coefficient
                            4468);                                     /// Propeller max RPM
}

// Add visualization shapes
void Little_Hexy::AddVisualizationAssets() {
    ChFrame<> nulldisp(VNULL, QUNIT);
    ChCopter::AddVisualizationAssets(GetChronoDataFile(chassis_mesh_path), GetChronoDataFile(propeller_mesh_path),
                                     nulldisp, nulldisp);
}

// Add collision shapes
// The collision shape is a boundary box, anything more sophisticated is probably an overkill
void Little_Hexy::AddCollisionShapes(std::shared_ptr<ChMaterialSurface> material) {
    chassis->GetCollisionModel()->ClearModel();
    // Legs and body boundary box
    chassis->GetCollisionModel()->AddBox(material, 0.279, 0.279, 0.46);
    // Arms and propellers boundary cylinder
    // propeller arm + propeller radius
    double radius = 0.762 + 0.6718 / 2;
    ChMatrix33<> matr(Q_ROTATE_Y_TO_Z);
    chassis->GetCollisionModel()->AddCylinder(material, radius, radius, 0.1, ChVector<>(0, 0, 0.2783), matr);
    chassis->GetCollisionModel()->BuildModel();
    chassis->SetCollide(true);
}

void Little_Hexy::Pitch_Down(double delta) {
    // Back Motors UP
    double commands[6] = {0, 0, delta, delta, 0, 0};
    this->ControlIncremental(commands);
}

void Little_Hexy::Pitch_Up(double delta) {
    // Front Motors UP
    double commands[6] = {delta, 0, 0, 0, 0, delta};
    this->ControlIncremental(commands);
}

void Little_Hexy::Roll_Right(double delta) {
    // Left Motors UP
    double commands[6] = {0, 0, 0, delta, delta, delta};
    this->ControlIncremental(commands);
}

void Little_Hexy::Roll_Left(double delta) {
	// Right Motors UP
    double commands[6] = {delta, delta, delta, 0, 0, 0};
    this->ControlIncremental(commands);
}

void Little_Hexy::Yaw_Right(double delta) {
	// CCW motors UP
    double commands[6] = {delta, 0, delta, 0, delta, 0};  // {false, true, false, true, false, true}
    this->ControlIncremental(commands);
}

void Little_Hexy::Yaw_Left(double delta) {
    // CW motors UP
    double commands[6] = {0, delta, 0, delta, 0, delta};  // {false, true, false, true, false, true}
    this->ControlIncremental(commands);
}

void Little_Hexy::Throttle(double delta) {
    // CW motors UP
    double commands[6] = {delta, delta, delta, delta, delta, delta};  // {false, true, false, true, false, true}
    this->ControlIncremental(commands);
}

std::vector<ChVector<>> Little_Hexy::getPosVect() {
    std::vector<ChVector<>> ppos;
    for (int i = 0; i < 6; i++) {
        double ang = CH_C_PI * (double(i) / 3) + CH_C_PI / 6;
        double R = 0.762;
        ChVector<> pos(std::cos(ang) * R, std::sin(ang) * R, 0.279);
        ppos.push_back(pos);
    };
    return ppos;
}
} // namespace copters
} // namespace chrono
