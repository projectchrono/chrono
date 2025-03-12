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
// Little Hexy model
//
// =============================================================================

#include <cmath>

#include "chrono_models/robot/copters/Little_Hexy.h"

namespace chrono {
namespace copter {

static const bool spins[6] = {false, true, false, true, false, true};

Little_Hexy::Little_Hexy(ChSystem& sys, const ChVector3d& cpos)
    : Copter<6>(sys, cpos, getPosVect(), spins, true, true) {
    chassis->SetMass(13.83457);
    chassis->SetInertiaXX(ChVector3d(1.264, 1.277, 1.541));
    this->SetPropellerData(0.126,                                     // Propeller mass
                           ChVector3d(0.004739, 0.004739, 0.004739),  // Propeller Inertia
                           0.6718,                                    // Propeller Diameter [m]
                           0.0587,                                    // Propeller Thrust Coefficient
                           0.018734,                                  // Propeller Power Coefficient
                           4468);                                     // Propeller max RPM
}

// Add visualization shapes
void Little_Hexy::AddVisualizationAssets() {
    ChFrame<> nulldisp(VNULL, QUNIT);
    Copter::AddVisualizationAssets(GetChronoDataFile(chassis_mesh_path), GetChronoDataFile(propeller_mesh_path),
                                   nulldisp, nulldisp);
}

// Add collision shapes
// The collision shape is a boundary box, anything more sophisticated is probably an overkill
void Little_Hexy::AddCollisionShapes(std::shared_ptr<ChContactMaterial> material) {
    // Legs and body boundary box
    auto box = chrono_types::make_shared<ChCollisionShapeBox>(material, 0.558, 0.558, 0.92);
    chassis->AddCollisionShape(box);

    // Arms and propellers boundary cylinder
    // propeller arm + propeller radius
    auto cyl = chrono_types::make_shared<ChCollisionShapeCylinder>(material, 0.762 + 0.6718 / 2, 0.2);
    chassis->AddCollisionShape(cyl, ChFrame<>(ChVector3d(0, 0, 0.2783), Q_ROTATE_Y_TO_Z));

    chassis->EnableCollision(true);
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

std::vector<ChVector3d> Little_Hexy::getPosVect() {
    std::vector<ChVector3d> ppos;
    for (int i = 0; i < 6; i++) {
        double ang = i * CH_PI_3 + CH_PI / 6;
        double R = 0.762;
        ChVector3d pos(std::cos(ang) * R, std::sin(ang) * R, 0.279);
        ppos.push_back(pos);
    }
    return ppos;
}

}  // namespace copter
}  // namespace chrono
