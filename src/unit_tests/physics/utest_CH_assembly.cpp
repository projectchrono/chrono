// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Test for the assembly analysis.
//
// =============================================================================

////#include <cfloat>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include <iostream>
#include <cmath>

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"

using namespace chrono;

int main(int argc, char* argv[]) {
    double mass = 1.0;                     // mass of pendulum
    double length = 4.0;                   // length of pendulum
    ChVector<> inertiaXX(0.04, 0.1, 0.1);  // mass moments of inertia of pendulum (centroidal frame)
    double g = 9.80665;                    // gravitational acceleration

    ChVector<> jointLoc(1, 2, 3);                       // absolute location of revolute joint
    double jointAngle = -CH_C_PI_4;                     // joint rotation angle (about global X axis)
    ChQuaternion<> jointRot = Q_from_AngX(jointAngle);  // orientation of revolute joint

    // Create the mechanical system
    ChSystem my_system;
    my_system.Set_G_acc(ChVector<>(0.0, 0.0, -g));

    // Integrator settings
    my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    my_system.SetMaxItersSolverSpeed(100);
    my_system.SetMaxItersSolverStab(100);
    my_system.SetSolverType(ChSolver::Type::SOR);
    my_system.SetTol(1e-6);
    my_system.SetTolForce(1e-4);

    // Create the ground body
    auto ground = std::make_shared<ChBody>();
    my_system.AddBody(ground);
    ground->SetIdentifier(0);
    ground->SetName("ground");
    ground->SetBodyFixed(true);

    // Create the pendulum body in an initial configuration at rest, with an
    // orientation that matches the specified joint orientation and a position
    // consistent with the specified joint location.
    // The pendulum CG is assumed to be at half its length.
    auto pendulum = std::make_shared<ChBody>();
    my_system.AddBody(pendulum);
    pendulum->SetIdentifier(1);
    pendulum->SetName("pendulum");
    pendulum->SetPos(jointLoc + jointRot.Rotate(ChVector<>(length / 2, 0, 0)));
    pendulum->SetRot(jointRot);
    pendulum->SetMass(mass);
    pendulum->SetInertiaXX(inertiaXX);

    // Create revolute joint between pendulum and ground at "loc" in the global
    // reference frame. The revolute joint's axis of rotation will be the Z axis
    // of the specified rotation matrix.
    auto revoluteJoint = std::make_shared<ChLinkLockRevolute>();
    revoluteJoint->Initialize(pendulum, ground, ChCoordsys<>(jointLoc, jointRot));
    ////auto revoluteJoint = std::make_shared<ChLinkRevolute>();
    ////revoluteJoint->Initialize(pendulum, ground, ChFrame<>(jointLoc, jointRot));
    my_system.AddLink(revoluteJoint);

    // Perform a system assembly.
    ////my_system.DoAssembly(AssemblyLevel::VELOCITY | AssemblyLevel::ACCELERATION);
    my_system.DoFullAssembly();

    // Extract position, velocity, and acceleration of pendulum body.
    ChVector<> pos = pendulum->GetPos();
    ChQuaternion<> rot = pendulum->GetRot();
    ChVector<> lin_vel = pendulum->GetPos_dt();
    ChVector<> ang_vel = pendulum->GetWvel_par();
    ChVector<> lin_acc = pendulum->GetPos_dtdt();
    ChVector<> ang_acc = pendulum->GetWacc_par();

    // Joint frame on 2nd body (ground), expressed in the body frame
    ChCoordsys<> linkCoordsys = revoluteJoint->GetLinkRelativeCoords();

    // Reaction force and torque on ground, expressed in joint frame
    ChVector<> rfrc = revoluteJoint->Get_react_force();
    ChVector<> rtrq = revoluteJoint->Get_react_torque();

    // Reaction force and torque on ground, expressed in ground frame
    rfrc = linkCoordsys.TransformDirectionLocalToParent(rfrc);
    rtrq = linkCoordsys.TransformDirectionLocalToParent(rtrq);

    // Output results
    std::cout.setf(std::ios::scientific | std::ios::showpos);
    std::cout.precision(6);

    std::cout << "Position:      " << pos.x() << "  " << pos.y() << "  " << pos.z() << std::endl;
    std::cout << "Orientation:   " << rot.e0() << "  " << rot.e1() << "  " << rot.e2() << "  " << rot.e3() << std::endl;
    std::cout << "Lin. vel.:     " << lin_vel.x() << "  " << lin_vel.y() << "  " << lin_vel.z() << std::endl;
    std::cout << "Ang. vel.:     " << ang_vel.x() << "  " << ang_vel.y() << "  " << ang_vel.z() << std::endl;
    std::cout << "Lin. acc.:     " << lin_acc.x() << "  " << lin_acc.y() << "  " << lin_acc.z() << std::endl;
    std::cout << "Ang. acc.:     " << ang_acc.x() << "  " << ang_acc.y() << "  " << ang_acc.z() << std::endl;
    std::cout << "React. force:  " << rfrc.x() << "  " << rfrc.y() << "  " << rfrc.z() << std::endl;
    std::cout << "React. torque: " << rtrq.x() << "  " << rtrq.y() << "  " << rtrq.z() << std::endl;

    // Analytical solution

    // Position and orientation
    ChVector<> pos_ref = jointLoc + ChVector<>(0.5 * length, 0, 0);
    ChQuaternion<> rot_ref(std::cos(jointAngle / 2), std::sin(jointAngle / 2), 0, 0);

    // Linear and angular velocities (expressed in absolute frame)
    ChVector<> lin_vel_ref(0, 0, 0);
    ChVector<> ang_vel_ref(0, 0, 0);

    // Angular acceleration (expressed in local frame)
    double omg_z = -0.5 * mass * g * length * std::sin(jointAngle) / (inertiaXX.z() + 0.25 * mass * length * length);

    // Angular acceleration (expressed in absolute frame)
    ChVector<> ang_acc_ref(0, -std::sin(jointAngle) * omg_z, std::cos(jointAngle) * omg_z);

    // Linear acceleration (expressed in absolute frame)
    ChVector<> lin_acc_ref(0, 0.5 * length * std::cos(jointAngle) * omg_z, 0.5 * length * std::sin(jointAngle) * omg_z);

    // Lagrange multipliers
    double lambda_1 = 0;
    double lambda_2 = -0.5 * mass * length * std::cos(jointAngle) * omg_z;
    double lambda_3 = -mass * g - 0.5 * mass * length * std::sin(jointAngle) * omg_z;
    double lambda_4 = -0.5 * length * std::sin(jointAngle) * lambda_2 + 0.5 * length * std::cos(jointAngle) * lambda_3;
    double lambda_5 = 0;

    // Reaction force and torque on first body (ground), at joint location, expressed in absolute frame
    ChVector<> rfrc_ref(lambda_1, lambda_2, lambda_3);
    ChVector<> rtrq_ref(lambda_5, -std::cos(jointAngle) * lambda_4, -std::sin(jointAngle) * lambda_4);

    std::cout << std::endl << "Analytical solution" << std::endl;
    std::cout << "Position:      " << pos_ref.x() << "  " << pos_ref.y() << "  " << pos_ref.z() << std::endl;
    std::cout << "Orientation:   " << rot_ref.e0() << "  " << rot_ref.e1() << "  " << rot_ref.e2() << "  "
              << rot_ref.e3() << std::endl;
    std::cout << "Lin. vel.:     " << lin_vel_ref.x() << "  " << lin_vel_ref.y() << "  " << lin_vel_ref.z()
              << std::endl;
    std::cout << "Ang. vel.:     " << ang_vel_ref.x() << "  " << ang_vel_ref.y() << "  " << ang_vel_ref.z()
              << std::endl;
    std::cout << "Lin. acc.:     " << lin_acc_ref.x() << "  " << lin_acc_ref.y() << "  " << lin_acc_ref.z()
              << std::endl;
    std::cout << "Ang. acc.:     " << ang_acc_ref.x() << "  " << ang_acc_ref.y() << "  " << ang_acc_ref.z()
              << std::endl;
    std::cout << "React. force:  " << rfrc_ref.x() << "  " << rfrc_ref.y() << "  " << rfrc_ref.z() << std::endl;
    std::cout << "React. torque: " << rtrq_ref.x() << "  " << rtrq_ref.y() << "  " << rtrq_ref.z() << std::endl;

    // Compare simulation and analytical solution
    bool passed = true;
    passed &= pos.Equals(pos_ref, 1e-3);
    passed &= rot.Equals(rot_ref, 1e-3);
    passed &= lin_vel.Equals(lin_vel_ref, 1e-4);
    passed &= ang_vel.Equals(ang_vel_ref, 1e-4);
    passed &= lin_acc.Equals(lin_acc_ref, 1e-2);
    passed &= ang_acc.Equals(ang_acc_ref, 1e-2);
    passed &= rfrc.Equals(rfrc_ref, 1e-2);
    passed &= rtrq.Equals(rtrq_ref, 1e-2);

    std::cout << std::endl << "Test " << (passed ? "PASSED" : "FAILED") << std::endl;

    // Return 0 if test passed
    return !passed;
}
