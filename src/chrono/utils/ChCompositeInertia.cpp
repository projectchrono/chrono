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

#include "chrono/utils/ChCompositeInertia.h"

namespace chrono {
namespace utils {

CompositeInertia::CompositeInertia() : m_mass(0) {
    m_inertia.setZero();
}

// Utility function for calculating an inertia shift matrix from a given vector.
// This matrix is used when applying the parallel axis theorem.
ChMatrix33<> CompositeInertia::InertiaShiftMatrix(const ChVector<>& v) {
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
    const ChVector<>& com = frame.GetPos();
    const ChMatrix33<>& A = frame.GetA();

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

}  // end namespace utils
}  // end namespace chrono
