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
// Class for analytical solution of SCARA robot kinematics.
//
// =============================================================================

#ifndef CH_INDUSTRIAL_ROBOT_KINEMATICS_SCARA_H
#define CH_INDUSTRIAL_ROBOT_KINEMATICS_SCARA_H

#include "IndustrialKinematics.h"

namespace chrono {
namespace industrial {

class CH_MODELS_API IndustrialKinematicsSCARA : public IndustrialKinematics {
  public:
    /// Default constructor.
    IndustrialKinematicsSCARA(){};

    /// Build model from joints absolute coordinates and link lengths.
    IndustrialKinematicsSCARA(
        const std::array<ChCoordsysd, 5>&
            joints_abs_coord,  ///< joints starting absolute coordinates (assume rotation about local Z axes)
        const std::array<double, 5>& lengths,  ///< robot arm lengths (H, L1, L2, D, L3)
        bool right_elbow = true                ///< choose right/left elbow IK solution
    );

    /// Copy constructor.
    IndustrialKinematicsSCARA(const IndustrialKinematicsSCARA& other);

    /// Virtual destructor.
    virtual ~IndustrialKinematicsSCARA(){};

    /// Set absolute and relative robot joints coordinates.
    void SetupCoords(const std::array<ChCoordsysd, 5>& joints_abs_coord);

    /// Set robot geometrical data.
    void SetupGeomData(const std::array<double, 5>& lengths, bool right_elbow);

    /// Get Forward Kinematics at given input u, up to TCP
    ChCoordsysd GetFK(const ChVectorDynamic<>& u) const;

    /// Get Inverse Kinematics at given joint coordinates (RRRP)
    ChVectorDynamic<> GetIK(const ChCoordsysd& targetcoord) const;

  private:
    std::array<ChCoordsysd, 5> m_joints_abs_coord;      ///< joints starting absolute coordinates
    std::array<ChCoordsysd, 5> m_joints_rel_coord;      ///< joints starting relative coordinates
    std::array<double, 5> m_lengths = {0, 0, 0, 0, 0};  ///< robot arm lengths (H, L1, L2, D, L3)
    bool m_right_elbow = true;                          ///< choose right/left elbow IK solution
    ChQuaternion<> m_TCP_rot0;                          ///< initial TCP rotation
    ChVector3d m_TCP_offset0;                           ///< initial TCP vertical offset from base
};

}  // end namespace industrial
}  // end namespace chrono

#endif  // end CH_INDUSTRIAL_ROBOT_KINEMATICS_SCARA_H