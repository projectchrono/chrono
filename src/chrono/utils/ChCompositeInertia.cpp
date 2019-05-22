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
    m_inertia.Reset();
}

// Calculate and return the inertia tensor of the composite body, with respect
// to its centroidal frame.
ChMatrix33<> CompositeInertia::GetInertia() const {
    ChMatrix33<> offset;
    offset.Set33Element(0, 0, m_com.y() * m_com.y() + m_com.z() * m_com.z());
    offset.Set33Element(1, 1, m_com.x() * m_com.x() + m_com.z() * m_com.z());
    offset.Set33Element(2, 2, m_com.x() * m_com.x() + m_com.y() * m_com.y());
    offset.Set33Element(0, 1, -m_com.x() * m_com.y());
    offset.Set33Element(1, 0, -m_com.x() * m_com.y());
    offset.Set33Element(0, 2, -m_com.x() * m_com.z());
    offset.Set33Element(2, 0, -m_com.x() * m_com.z());
    offset.Set33Element(1, 2, -m_com.y() * m_com.z());
    offset.Set33Element(2, 1, -m_com.y() * m_com.z());

    return m_inertia - offset * m_mass;
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

    // Express sub-component inertia w.r.t. the reference frame
    // and update composite inertia
    ChMatrix33<> offset;
    offset.Set33Element(0, 0, com.y() * com.y() + com.z() * com.z());
    offset.Set33Element(1, 1, com.x() * com.x() + com.z() * com.z());
    offset.Set33Element(2, 2, com.x() * com.x() + com.y() * com.y());
    offset.Set33Element(0, 1, -com.x() * com.y());
    offset.Set33Element(1, 0, -com.x() * com.y());
    offset.Set33Element(0, 2, -com.x() * com.z());
    offset.Set33Element(2, 0, -com.x() * com.z());
    offset.Set33Element(1, 2, -com.y() * com.z());
    offset.Set33Element(2, 1, -com.y() * com.z());

    ChMatrix33<> tmp;
    tmp.MatrTMultiply(A, inertia);  // tmp = A^T * inertia
    ChMatrix33<> increment = tmp * A + offset * mass;
    m_inertia = (is_void) ? m_inertia - increment : m_inertia + increment;
}

}  // end namespace utils
}  // end namespace chrono
