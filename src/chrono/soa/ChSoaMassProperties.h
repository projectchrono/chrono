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
// Authors: Radu Serban
// =============================================================================
//
// This file contains the definition of the ChSoaMassProperties class.
//
// =============================================================================

#ifndef CH_SOA_MASS_PROPERTIES_H
#define CH_SOA_MASS_PROPERTIES_H

#include "chrono/core/ChFrame.h"
#include "chrono/core/ChMatrixMBD.h"

#include "chrono/soa/ChSoaSpatial.h"

namespace chrono {
namespace soa {

/// @addtogroup chrono_soa
/// @{

/// Inertial properties of a mobilized body.
/// ChSoaMassProperties encapsulates the mass, centroidal frame, and inertia of a mobilized body B.
class ChApi ChSoaMassProperties {
  public:
    ChSoaMassProperties();

    /// Construct inertia properties for a rigid body.
    /// Inertia properties are set from the specified mass, centroidal frame (assumed to be specified relative to the
    /// body reference frame), and inertia matrix (assumed to be specified in the centroidal frame).
    ChSoaMassProperties(double mass, const ChFramed& X_BC, const ChMatrix33<>& inertia);

    /// Construct inertia properties for a rigid body.
    /// Inertia properties are set from the specified mass, COM location (the centroidal frame is assumed parallel to
    /// the body reference frame), and inertia matrix (assumed to be specified in the centroidal frame).
    ChSoaMassProperties(double mass, const ChVector3d& com, const ChMatrix33<>& inertia);

    const double& mass() const { return m_mass; }
    const ChVector3d& com() const { return m_X_BC.GetPos(); }
    const ChMatrix33<>& inertia() const { return m_inertia_B; }

    const double& getOoMass() const { return m_ooMass; }
    const ChSpatialMat& getInverse() const { return m_invM_B; }

    bool isCentral() const { return m_X_BC.GetPos() == ChVector3d(0.0f); }

    /// Return the centroidal frame (relative to and expressed in the body reference frame)
    const ChFramed& centroidal_frame() const { return m_X_BC; }

    /// Return the mass properties as a symmetric spatial matrix.
    ChSpatialMat asSpatialMat() const;

    /// Calculate the inverse of the spatial matrix about the origin of the body frame B.
    /// The inverse has the form:
    /// <pre>
    ///		[         Ic^(-1)       |       -Ic^(-1) * p_x             ]
    ///		[ ----------------------+--------------------------------- ]
    ///		[      p_x * Ic^(-1)    |  (1/m)*I_3 - p_x * Ic^(-1) * p_x ]
    /// </pre>
    /// where Ic is the inertia matrix about the center of mass, m is the body mass, p is the location of the COM, and
    /// {}_x is the matrix operator for cross products.
    void calcInverse();

  private:
    /// Construct an inertia matrix for a mass point at a given distance.
    static ChMatrix33<> calcPointInertia(const ChVector3d& pos, double mass);

    double m_mass;             ///< body mass
    ChFramed m_X_BC;           ///< transform from body reference frame to centroidal frame
    ChMatrix33<> m_inertia_B;  ///< body inertia about COM, expressed in body reference frame

    double m_ooMass;        ///< mass inverse (1 / m_mass)
    ChSpatialMat m_invM_B;  ///< inverse inertia tensor (m_inertiaOB_B^{-1})
};

/// @} chrono_soa

}  // namespace soa
}  // namespace chrono

#endif
