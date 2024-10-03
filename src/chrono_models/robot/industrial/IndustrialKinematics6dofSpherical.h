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
// Class for analytical solution of 6-DOF articulated robot kinematics.
//
// =============================================================================

#ifndef CH_INDUSTRIAL_ROBOT_KINEMATICS_6DOF_SPHERICAL_H
#define CH_INDUSTRIAL_ROBOT_KINEMATICS_6DOF_SPHERICAL_H

#include "IndustrialKinematics.h"

namespace chrono {
namespace industrial {

class CH_MODELS_API IndustrialKinematics6dofSpherical : public IndustrialKinematics {
  public:
    /// Default constructor.
    IndustrialKinematics6dofSpherical(){};

    /// Build model from joints absolute coordinates, vertical angles and link lengths.
    IndustrialKinematics6dofSpherical(
        const std::array<ChCoordsysd, 7>&
            joints_abs_coord,  ///< joints starting absolute coordinates (assume rotation about local Z axes)
        const std::array<double, 2>& vert_angs,  ///< adjust th2/th3 values: angles needed to start sub-robot 3dof from
                                                 ///< theoretical vertical configuration
        const std::array<double, 4>& lengths     ///< robot arm lengths (H, L1, L2, L3)
    );

    /// Build model from joints absolute coordinates and vertical angles. Infer link lengths from given joint
    /// coordinates.
    IndustrialKinematics6dofSpherical(const std::array<ChCoordsysd, 7>& joints_abs_coord,
                                   const std::array<double, 2>& vert_angs);

    /// Copy constructor.
    IndustrialKinematics6dofSpherical(const IndustrialKinematics6dofSpherical& other);

    /// Virtual destructor.
    virtual ~IndustrialKinematics6dofSpherical(){};

    /// Set absolute and relative robot joints coordinates.
    void SetupCoords(const std::array<ChCoordsysd, 7>& m_joints_abs_coord);

    /// Set robot geometrical data.
    void SetupGeomData(const std::array<double, 2>& vert_angs, const std::array<double, 4>& lengths);

    /// Get joints starting coordinates, wrt absolute frame.
    std::array<ChCoordsysd, 7> GetAbsJointsCoord() const { return m_joints_abs_coord; };

    /// Get joints starting coordinates, wrt previous local frame.
    std::array<ChCoordsysd, 7> GetRelJointsCoord() const { return m_joints_rel_coord; };

    /// Get Forward Kinematics at given input u, up to Nth link.
    virtual ChCoordsysd GetFK(const ChVectorDynamic<>& u, int Nth) const;

    /// Get Forward Kinematics at given input u, up to TCP.
    ChCoordsysd GetFK(const ChVectorDynamic<>& u) const;

    /// Get Inverse Kinematics for given target coordinates.
    virtual ChVectorDynamic<> GetIK(const ChCoordsysd& targetcoord) const;

  private:
    std::array<ChCoordsysd, 7> m_joints_abs_coord;   ///< joints starting absolute coordinates
    std::array<ChCoordsysd, 7> m_joints_rel_coord;   ///< joints starting relative coordinates
    std::array<double, 4> m_lengths = {0, 0, 0, 0};  ///< robot arm lengths (H, L1, L2, L3)
    std::array<double, 2> m_vert_angs = {0, 0};      ///< adjust th2/th3 values
};

}  // end namespace industrial
}  // end namespace chrono

#endif  // end CH_INDUSTRIAL_ROBOT_KINEMATICS_6DOF_SPHERICAL_H