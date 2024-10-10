// ==================================================================================================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 Federico Maria Reato, Matteo Santelia, Filippo Morlacchi, Claudio Ricci, Maurizio Zama, Iseo Serrature S.p.a, projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// ==================================================================================================================================================
// Authors: Federico Maria Reato, Matteo Santelia, Filippo Morlacchi, Claudio Ricci
// ==================================================================================================================================================

#include "ChElectronicMotor.h"

namespace chrono {
namespace powerelectronics {

ChElectronicMotor::ChElectronicMotor(std::shared_ptr<ChBody> spindle, double t_step)
    : ChElectronicCircuit("../data/Circuit/MotorControl/Circuit_Netlist.cir", t_step) {
    this->spindle = spindle;
}

ChElectronicMotor::ChElectronicMotor(double t_step)
    : ChElectronicCircuit("../data/Circuit/MotorControl/Circuit_Netlist.cir", t_step) {
}

void ChElectronicMotor::PreInitialize() {
    // this->SetInitialPWLIn({
    //     {"VgenVAR", 15.},
    //     {"VbackemfCVAR", 0.0},
    //     {"VSW1VAR", 1.},
    //     {"VgenPWMVAR", 5.200000e+00}
    // });
    this->SetInitialPWLIn({
        {"VgenVAR", 0.},
        {"VbackemfCVAR", 0.0},
        {"VSW1VAR", 0.},
        {"VgenPWMVAR", 0.}
    });
    this->SetInitialFlowInICs({
        {"LaC", 45e-6},
        {"RaC", 25.},
    });

    this->SetBranchTracking({
        "LaC"
    });
}

void ChElectronicMotor::SetShaftAngVel(double angvel) {
    this->shaft_angvel = angvel;
}

ChVector3d ChElectronicMotor::GetOutputTorque() {
    return this->spindle_torque;
}

void ChElectronicMotor::SetPWM(double PWM) {
    this->VgenPWMVAR = PWM;
}

void ChElectronicMotor::PostInitialize() {
    // Additional post-initialization logic can be added here if necessary.
}

void ChElectronicMotor::PreAdvance() {

    this->flow_in["LaC"] = 45e-6;
    this->flow_in["RaC"] = 25.;

    // this->pwl_in["VgenVAR"] = 15.;
    // this->pwl_in["VbackemfCVAR"] = this->VbackemfCVAR;
    // this->pwl_in["VSW1VAR"] = 1.;
    // this->pwl_in["VgenPWMVAR"] = this->VgenPWMVAR;
    this->pwl_in["VgenVAR"] = 5.;
    this->pwl_in["VbackemfCVAR"] = 0.;
    this->pwl_in["VSW1VAR"] = 1.;
    this->pwl_in["VgenPWMVAR"] = 5.;

}

void ChElectronicMotor::PostAdvance() {


    auto res = this->GetResult();
    double IVprobe1 = res["vprobe1"][res["vprobe1"].size() - 1];

    // std::cout << IVprobe1 << std::endl;

    ChVector3d torque_vec_norm(0, 0, 1); // Normalized direction vector
    double spindle_torque_mag = this->kt_motor * IVprobe1* 1e3 * 1e3; // Convert to [kg]-[mm]-[s]

    // std::cout << this->kt_motor << " " << IVprobe1 << " " << spindle_torque_mag << std::endl;

    ChVector3d spindle_torque = spindle_torque_mag * torque_vec_norm;
    this->spindle_torque = spindle_torque;

    double ang_vel = this->shaft_angvel;

    if (this->spindle != nullptr) {
        ang_vel = spindle->GetAngVelLocal()[2];
    }

    this->VbackemfCVAR = ke_motor * ang_vel;

    if (this->spindle != nullptr) {
        // spindle->EmptyAccumulators(); // Clear previous forces/torques
        spindle->AccumulateTorque(spindle_torque, false); // Apply torque to the spindle body
    }


}

}
}
