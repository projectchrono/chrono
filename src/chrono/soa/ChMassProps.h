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
// This file contains the definition of the ChMassProps class. It encapsulates
// the mass, center of mass, and inertia of a mobilized body B. The center of
// mass is a vector from B's origin, expressed in the B frame. The inertia is
// taken about the origin of B, and expressed in B.
//
// =============================================================================

#ifndef CH_MASS_PROPS_H
#define CH_MASS_PROPS_H

#include "chrono/core/ChFrame.h"
#include "chrono/core/ChMatrixMBD.h"

#include "chrono/soa/ChSpatial.h"

namespace chrono {
namespace soa {

/// @addtogroup chrono_soa
/// @{

/// Inertial properties of a mobilized body.
/// ChMassProps encapsulates the mass, center of mass, and inertia of a mobilized body B. The center of mass is a vector
/// from B's origin, expressed in the B frame. The inertia is taken about the origin of B, and expressed in B.
class ChApi ChMassProps {
  public:
    /// Default constructor.
    /// Does not calculate the inverse of the mass and inertia matrix.
    ChMassProps();

    /// Construct the mass properties of a point mass located at the origin of the body frame.
    ChMassProps(double mass);

    /// Construct a general-form ChMassProps.
    /// The mass properties are set from the specified mass, location of the center of mass, and inertia matrix (assumed
    /// to be specified with respect to the origin of the body frame.
    ChMassProps(double mass, const ChVector3d& com, const ChMatrix33<>& inertia);

    double& mass() { return m_mass; }
    ChVector3d& com() { return m_com_B; }
    ChMatrix33<>& inertia() { return m_inertiaOB_B; }

    const double& mass() const { return m_mass; }
    const ChVector3d& com() const { return m_com_B; }
    const ChMatrix33<>& inertia() const { return m_inertiaOB_B; }

    const double& getOoMass() const { return m_ooMass; }
    const ChSpatialMat& getInverse() const { return m_invM_B; }

    bool isCentral() const { return m_com_B == ChVector3d(0.0f); }

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

    /// Transforms the inertia matrix into a central inertia.
    /// In other words, the inertia matrix  i.e. the inertia (currently given about the body frame B) is transformed
    /// into an inertia matrix about a frame B' aligned with B but centered at the COM.
    ChMatrix33<> calcCentralInertia() const;

    /// General transform of the inertia matrix.
    /// This function transforms the inertia matrix (currently given about the body frame B) into an inertia matrix
    /// about some frame B' aligned with B but centered at some given location (the shift must be done through the
    /// central inertia).
    ChMatrix33<> calcShiftedInertia(const ChVector3d& newOriginB) const;

    /// Reexpress the inertia matrix in a different frame.
    /// This function transforms the inertia matrix (currently given about the body frame B and expressed in B) into an
    /// inertia matrix about a frame C and expressed in C.
    ChMatrix33<> calcTransformedInertia(const ChFramed& X_BC) const;

    // ChMassProps::calcShiftedMassProps()
    // ChMassProps::calcTransformedMassProps()
    // ChMassProps::reexpress()
    //
    // These functions perform various transformations on the mass properties:
    // - calcShiftedMassProps transforms a body's mass properties from the implicit
    //   B frame to a new frame aligned with B but centered at a given point.
    // - calcTransformedMassProps transforms a body's mass properties from the
    //   implicit B frame to a new frame C, given the transform from B to C.
    // - reexpress simply expresses a body's mass properties in a new frame C, given
    //   the orientation of the frame C in the B frame.
    ChMassProps calcShiftedMassProps(const ChVector3d& newOriginB) const;
    ChMassProps calcTransformedMassProps(const ChFramed& X_BC) const;
    ChMassProps reexpress(const ChMatrix33<>& R_BC) const;

  private:
    /// Construct an inertia matrix for a mass point at a given distance.
    static ChMatrix33<> calcPointInertia(const ChVector3d& pos, double mass);

    double m_mass;
    ChVector3d m_com_B;
    ChMatrix33<> m_inertiaOB_B;

    double m_ooMass;
    ChSpatialMat m_invM_B;
};

/// @} chrono_soa

}  // namespace soa
}  // namespace chrono

#endif
