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
// Model of industrial SCARA robot imported from CAD.
//
// =============================================================================

#ifndef CH_INDUSTRIAL_ROBOT_SCARA_CAD_H
#define CH_INDUSTRIAL_ROBOT_SCARA_CAD_H

#include "IndustrialRobotSCARA.h"

namespace chrono {
namespace industrial {

class CH_MODELS_API IndustrialRobotSCARA_CAD : public IndustrialRobotSCARA {
  public:
    /// Default constructor.
    IndustrialRobotSCARA_CAD(){};

    /// Build SCARA R-R-R-P robot model from CAD bodies already imported in sys.
    IndustrialRobotSCARA_CAD(ChSystem* sys,                            ///< containing sys
                     const ChFramed& base_frame = ChFramed(),  ///< place robot base in these coordinatesv
                     unsigned int id = 0,  ///< give robot a unique identifier (useful to import multiple instances of
                                           ///< same CAD robot without name clashes)
                     std::vector<std::string> bodynames = {"base", "biceps", "forearm", "screw", "end_effector"}
                     ///< name of bodies to search in sys for building robot model (fallback)
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

    unsigned int m_id = 0;                   ///< robot model unique identifier
    std::vector<std::string> m_bodynames;    ///< name of bodies to search in sys for building robot model
    std::shared_ptr<ChBodyAuxRef> m_ground;  ///< robot 'ground' virtual body
};

}  // end namespace industrial
}  // end namespace chrono

#endif  // end CH_INDUSTRIAL_ROBOT_SCARA_CAD_H