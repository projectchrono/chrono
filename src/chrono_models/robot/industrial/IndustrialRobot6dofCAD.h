// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dario Fusai
// =============================================================================
//
// Model of industrial 6-DOF articulated robot imported from CAD.
//
// =============================================================================

#ifndef INDUSTRIAL_ROBOT_6DOF_CAD_H
#define INDUSTRIAL_ROBOT_6DOF_CAD_H

#include "IndustrialRobot6dof.h"

namespace chrono {
namespace industrial {

/// @addtogroup robot_models_industrial
/// @{

class CH_MODELS_API IndustrialRobot6dofCAD : public IndustrialRobot6dof {
  public:
    /// Default constructor.
    IndustrialRobot6dofCAD(){};

    /// Build 6dof articulated robot model from CAD bodies already imported in sys.
    IndustrialRobot6dofCAD(
        ChSystem* sys,                            ///< containing sys
        const ChFramed& base_frame = ChFramed(),  ///< place robot base in these coordinates
        unsigned int id = 0,  ///< give robot a unique identifier (useful to import multiple instances of same CAD robot
                              ///< without name clashes)
        std::vector<std::string> bodynames = {"base", "shoulder", "biceps", "elbow", "forearm", "wrist", "end_effector"}
        ///< name of bodies to search in sys for building robot model
        ///  NB: if 'base' body is not provided here, robot arm is linked to internal 'ground' instead (fallback)
    );

    /// Get robot 'ground' body.
    /// NB: this is a "virtual" body that is automatically imported from Chrono::Solidworks plugin
    /// and represents the overall assembly fixed parent body (i.e. not an actual robot body).
    std::shared_ptr<ChBody> GetGround() { return m_ground; }

    /// Suppress parent class functions to add schematic visual shapes, since actual
    /// 3D body meshes are already imported.
    virtual void Add1dShapes(const ChColor& col = ChColor(0.0f, 0.0f, 0.0f)) override{};
    virtual void Add3dShapes(double rad, const ChColor& col = ChColor(0.2f, 0.2f, 0.2f)) override{};

  private:
    /// Setup robot internal bodies, markers and motors.
    virtual void SetupBodies() override;
    virtual void SetupMarkers() override;
    virtual void SetupLinks() override;

    /// Helper function to preprocess CAD-imported marker.
    /// Search it by name in system, rename it, and attach it to given body.
    virtual std::shared_ptr<ChMarker> PreprocessMarker(const std::string& name, std::shared_ptr<ChBody> body);

    unsigned int m_id = 0;                 ///< robot model unique identifier
    std::vector<std::string> m_bodynames;  ///< name of bodies to search in sys for building robot model
    std::shared_ptr<ChBody> m_ground;      ///< robot 'ground' virtual body
};

/// @} robot_models_industrial

}  // end namespace industrial
}  // end namespace chrono

#endif  // end INDUSTRIAL_ROBOT_6DOF_CAD_H