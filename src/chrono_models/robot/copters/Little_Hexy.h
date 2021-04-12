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

#include "chrono_models/robot/copters/Copter.h"

#ifndef LITTLE_HEXY_H
#define LITTLE_HEXY_H

namespace chrono {
namespace copter {

/// @addtogroup robot_models_copter
/// @{

/// Little hexy (hexacopter) model.
class CH_MODELS_API Little_Hexy : public Copter<6> {
  public:
    Little_Hexy(ChSystem& sys, const ChVector<>& cpos);

    /// Add specific visualization shapes to shapes and propellers.
    void AddVisualizationAssets();

    /// Add collision shapes.
    /// The collision shape is a box + cylinder.
    void AddCollisionShapes(std::shared_ptr<ChMaterialSurface> material);

    /// Pitch down by the specified angle.
    void Pitch_Down(double delta);

    /// Pitch up by the specified angle.
    void Pitch_Up(double delta);

    /// Roll to the right by the specified angle.
    void Roll_Right(double delta);

    /// Roll to the right by the specified angle.
    void Roll_Left(double delta);

    /// Yaw to the right by the specified angle.
    void Yaw_Right(double delta);

    /// Yaw to the left by the specified angle.
    void Yaw_Left(double delta);

    /// Increase all propellers speeds.
    /// Use a negative delta to decrease all.
    void Throttle(double delta);

	/// Get the name of the Wavefront file with chassis visualization mesh.
    /// An empty string is returned if no mesh was specified.
    virtual const std::string& GetChassisMeshFilename() const override { return chassis_mesh_path; }

    /// Get the name of the Wavefront file with propeller visualization mesh.
    /// An empty string is returned if no mesh was specified.
    virtual const std::string& GetPropellerMeshFilename() const override { return propeller_mesh_path; }

  protected:
    static std::vector<ChVector<>> getPosVect();

  private:
    // clockwise (true) or CCW rotations according to Little Hexy manual
    std::string chassis_mesh_path = "robot/copters/hexi_body.obj";
    std::string propeller_mesh_path = "robot/copters/prop.obj";
};

/// @} robot_models_copter

}  // namespace copter
}  // namespace chrono

#endif
