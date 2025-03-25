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
    : ChElectronicCircuit("../data/powerelectronics/Circuit/MotorControl/Circuit_Netlist.cir", t_step) {
    this->spindle = spindle;
}

ChElectronicMotor::ChElectronicMotor(double t_step)
    : ChElectronicCircuit("../data/powerelectronics/Circuit/MotorControl/Circuit_Netlist.cir", t_step) {
}

void ChElectronicMotor::PreInitialize() {

    std::cout << "Initializing Motor" << std::endl;

    this->SetInitialPWLIn({
        {"VgenVAR", 48.},
        {"VbackemfCVAR", 0.0},
        {"VSW1VAR", 1.},
        {"VgenPWMVAR", 48.}
    });

    this->SetInitialFlowInICs({
        {"LaC", L_coil},
        {"RaC", R_coil},
    });

    /*this->SetBranchTracking({
        "LaC"
    });*/
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

void ChElectronicMotor::PreAdvance(double dt_mbs) {

    this->flow_in["LaC"] = L_coil;
    this->flow_in["RaC"] = R_coil;

    this->pwl_in["VgenVAR"] = 48.;
    // this->pwl_in["VbackemfCVAR"] = 20.;
    this->pwl_in["VbackemfCVAR"] = this->VbackemfCVAR;
    this->pwl_in["VSW1VAR"] = 1.;
    this->pwl_in["VgenPWMVAR"] = this->VgenPWMVAR;

}

void ChElectronicMotor::PostAdvance(double dt_mbs) {


    auto res = this->GetResult();
    
    /*// Plot the results
    std::cout << "Results:\n" << std::endl;
    for (const auto& [key, values] : res) { 
        std::cout << key << ": ";
        for (double value : values) {
            std::cout << value << " ";
        }
        std::cout << std::endl;}*/
    double IVprobe1 = res["vprobe1"].back();

    // std::cout << "###########################" << std::endl;

    ChVector3d torque_vec_norm(0, 0, 1); // Normalized direction vector
    double spindle_torque_mag = this->kt_motor * IVprobe1; //* 1e3 * 1e3; // Convert to [kg]-[mm]-[s]
    // if(IVprobe1 < 0) {
    //     spindle_torque_mag = 0.0;
    // }
    // std::cout << this->kt_motor << " " << IVprobe1 << " " << spindle_torque_mag << std::endl;

    ChVector3d spindle_torque = spindle_torque_mag * torque_vec_norm;
    this->spindle_torque = spindle_torque;
    

    double ang_vel = this->shaft_angvel*0.10472; // RPM to Rad/S

    if (this->spindle != nullptr) {
        ang_vel = spindle->GetAngVelLocal()[2];
    }
    std::cout << "Current " << IVprobe1 << " Ang Vel " << ang_vel << std::endl;


    this->VbackemfCVAR = ke_motor * ang_vel ;

    if (this->spindle != nullptr) {
        spindle->EmptyAccumulators(); // Clear previous forces/torques
        spindle->AccumulateTorque(spindle_torque, false); // Apply torque to the spindle body
    }


}

}
}
