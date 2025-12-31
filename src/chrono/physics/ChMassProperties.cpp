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
// Authors: Radu Serban
// =============================================================================

#include "chrono/physics/ChMassProperties.h"

namespace chrono {

void ChInertiaUtils::InertiaFromCluster(const std::vector<ChVector3d>& positions,
                                        const std::vector<ChMatrix33<> >& rotations,
                                        const std::vector<ChMatrix33<> >& Jlocal,
                                        const std::vector<double>& masses,
                                        ChMatrix33<>& totJ,
                                        double& totmass,
                                        ChVector3d& baricenter) {
    assert(positions.size() == Jlocal.size());
    assert(positions.size() == masses.size());

    totmass = 0;
    totJ.setZero();
    baricenter = VNULL;

    // compute tot mass and baricenter position
    for (unsigned int i = 0; i < positions.size(); ++i) {
        baricenter = (baricenter * totmass + positions[i] * masses[i]) / (totmass + masses[i]);
        totmass += masses[i];
    }

    // compute inertia
    for (unsigned int i = 0; i < positions.size(); ++i) {
        // rotate ith tensor in absolute space
        ChMatrix33<> R = rotations[i];
        ChMatrix33<> Rt = rotations[i];
        Rt.transposeInPlace();
        ChMatrix33<> absJ = R * Jlocal[i] * Rt;

        // Huygens-Steiner parallel axis theorem:

        ChVector3d dist = positions[i] - baricenter;

        ChMatrix33<> absJtranslated;
        absJtranslated = absJ;
        absJtranslated(0, 0) += masses[i] * (dist.Length2() - dist.x() * dist.x());
        absJtranslated(1, 1) += masses[i] * (dist.Length2() - dist.y() * dist.y());
        absJtranslated(2, 2) += masses[i] * (dist.Length2() - dist.z() * dist.z());
        absJtranslated(0, 1) += masses[i] * (-dist.x() * dist.y());
        absJtranslated(0, 2) += masses[i] * (-dist.x() * dist.z());
        absJtranslated(1, 2) += masses[i] * (-dist.y() * dist.z());
        // symmetric part
        absJtranslated(1, 0) = absJtranslated(0, 1);
        absJtranslated(2, 0) = absJtranslated(0, 2);
        absJtranslated(2, 1) = absJtranslated(1, 2);

        totJ += absJtranslated;
    }
}

void ChInertiaUtils::RotateInertia(const ChMatrix33<> inertiaIn, const ChMatrix33<> R, ChMatrix33<>& inertiaOut) {
    ChMatrix33<> Rt = R;
    Rt.transposeInPlace();
    inertiaOut = R * inertiaIn * Rt;
}

void ChInertiaUtils::TranslateInertia(const ChMatrix33<> inertiaIn,
                                      const ChVector3d dist,
                                      const double mass,
                                      ChMatrix33<>& inertiaOut) {
    // Huygens-Steiner parallel axis theorem:
    inertiaOut = inertiaIn;
    inertiaOut(0, 0) += mass * (dist.Length2() - dist.x() * dist.x());
    inertiaOut(1, 1) += mass * (dist.Length2() - dist.y() * dist.y());
    inertiaOut(2, 2) += mass * (dist.Length2() - dist.z() * dist.z());
    inertiaOut(0, 1) += mass * (-dist.x() * dist.y());
    inertiaOut(0, 2) += mass * (-dist.x() * dist.z());
    inertiaOut(1, 2) += mass * (-dist.y() * dist.z());
    // symmetric part
    inertiaOut(1, 0) = inertiaOut(0, 1);
    inertiaOut(2, 0) = inertiaOut(0, 2);
    inertiaOut(2, 1) = inertiaOut(1, 2);
}

void ChInertiaUtils::PrincipalInertia(const ChMatrix33<>& inertia,
                                      ChVector3d& principal_inertia,
                                      ChMatrix33<>& principal_axes) {
    ChVectorN<double, 3> principal_I;
    inertia.SelfAdjointEigenSolve(principal_axes, principal_I);

    // Ensure the principal_axes represent a proper rotation matrix.
    // If the determinant is -1, flip the sign of one of the eigenvectors to obtain a proper rotation
    // (i.e. a transformation representing a right-handed rotation)
    if (principal_axes.determinant() < 0) {
        principal_axes.col(0) *= -1;
    }

    // Return the principal moments of inertia in a ChVector3d
    principal_inertia = principal_I;

    // Tests
    ////std::cout << "Input inertia matrix: J\n" << inertia << std::endl;
    ////std::cout << "Principal moments of inertia: I\n" << principal_I << std::endl;
    ////std::cout << "Principal axes: R\n" << principal_axes << std::endl;
    ////std::cout << "det(R) = " << principal_axes.determinant() << std::endl;
    ////std::cout << "Transform back: R * diag(I) * R'\n"
    ////          << principal_axes * ChMatrix33<>(principal_inertia) * principal_axes.transpose() << std::endl;
}

// -----------------------------------------------------------------------------

CompositeInertia::CompositeInertia() : m_mass(0) {
    m_inertia.setZero();
}

// Utility function for calculating an inertia shift matrix from a given vector.
// This matrix is used when applying the parallel axis theorem.
ChMatrix33<> CompositeInertia::InertiaShiftMatrix(const ChVector3d& v) {
    ChMatrix33<> shift;
    shift(0, 0) = v.y() * v.y() + v.z() * v.z();
    shift(1, 1) = v.x() * v.x() + v.z() * v.z();
    shift(2, 2) = v.x() * v.x() + v.y() * v.y();
    shift(0, 1) = -v.x() * v.y();
    shift(1, 0) = -v.x() * v.y();
    shift(0, 2) = -v.x() * v.z();
    shift(2, 0) = -v.x() * v.z();
    shift(1, 2) = -v.y() * v.z();
    shift(2, 1) = -v.y() * v.z();

    return shift;
}

// Calculate and return the inertia tensor of the composite body, with respect to its centroidal frame.
ChMatrix33<> CompositeInertia::GetInertia() const {
    return m_inertia - InertiaShiftMatrix(m_com) * m_mass;
}

// Include sub-component inertia properties.
void CompositeInertia::AddComponent(
    const ChFrame<>& frame,       // centroidal frame of sub-component
    double mass,                  // mass of sub-component
    const ChMatrix33<>& inertia,  // sub-component inertia tensor w.r.t. its centroidal frame
    bool is_void                  // indicate if sub-component represents a material void
) {
    const ChVector3d& com = frame.GetPos();
    const ChMatrix33<>& A = frame.GetRotMat();

    double sign = is_void ? -1 : +1;

    // Update location of composite COM
    m_com = (m_mass * m_com + sign * mass * com) / (m_mass + sign * mass);

    // Update composite mass
    m_mass = m_mass + sign * mass;

    // Express sub-component inertia w.r.t. the reference frame and update composite inertia
    ChMatrix33<> increment = A * inertia * A.transpose() + InertiaShiftMatrix(com) * mass;
    if (is_void)
        m_inertia -= increment;
    else
        m_inertia += increment;
}

}  // end namespace chrono
