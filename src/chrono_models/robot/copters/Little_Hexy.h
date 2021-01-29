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

#include "chrono_models/robot/copters/ChCopter.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChForce.h"
#include "chrono_models/ChApiModels.h"

#ifndef LITTLE_HEXY_H
#define LITTLE_HEXY_H

namespace chrono {
/// Namespace with classes for the Copters model.
namespace copter {

/// @addtogroup robot_models_copter
/// @{

static const bool spins[6] = {false, true, false, true, false, true};
// Little hexy (hexacopter) model
class CH_MODELS_API Little_Hexy : public ChCopter<6> {
  public:
    Little_Hexy(ChSystem& sys, ChVector<> cpos);

    // Add specific visualization shapes to shapes and propellers
    void AddVisualizationAssets();

    // Add collision shapes
    // The collision shape is a box + cylinder
    void AddCollisionShapes(std::shared_ptr<ChMaterialSurface> material);

	// Commands according to little hexy (and ArduPilot)
    void Pitch_Down(double delta);

    void Pitch_Up(double delta);

    void Roll_Right(double delta);

    void Roll_Left(double delta);

    void Yaw_Right(double delta);

    void Yaw_Left(double delta);

	// Increases all propellers speeds
	// Use a negative delta to decrease all 
    void Throttle(double delta);

  protected:
    static std::vector<ChVector<>> getPosVect();

  private:
	// clockwise (true) or CCW rotations according to Little Hexy manual

    std::string chassis_mesh_path = "copters/hexi_body.obj";
    std::string propeller_mesh_path = "copters/prop.obj";
    
};

/// @} robot_models_copter
}
}  // namespace chrono

#endif
