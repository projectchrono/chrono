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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChNodeBase.h"

namespace chrono {

ChNodeBase::ChNodeBase() : offset_x(0), offset_w(0), m_tag(-1) {}

ChNodeBase::ChNodeBase(const ChNodeBase& other) {
    offset_x = other.offset_x;
    offset_w = other.offset_w;
    m_tag = other.m_tag;
}

ChNodeBase& ChNodeBase::operator=(const ChNodeBase& other) {
    if (&other == this)
        return *this;

    offset_x = other.offset_x;
    offset_w = other.offset_w;
    m_tag = other.m_tag;

    return *this;
}

void ChNodeBase::NodeIntStateIncrement(const unsigned int off_x,
                                       ChState& x_new,
                                       const ChState& x,
                                       const unsigned int off_v,
                                       const ChStateDelta& Dv) {
    for (unsigned int i = 0; i < GetNumCoordsPosLevel(); ++i) {
        x_new(off_x + i) = x(off_x + i) + Dv(off_v + i);
    }
}

void ChNodeBase::NodeIntStateGetIncrement(const unsigned int off_x,
                                          const ChState& x_new,
                                          const ChState& x,
                                          const unsigned int off_v,
                                          ChStateDelta& Dv) {
    for (unsigned int i = 0; i < GetNumCoordsPosLevel(); ++i) {
        Dv(off_v + i) = x_new(off_x + i) - x(off_x + i);
    }
}

void ChNodeBase::NodeIntLoadResidual_F_domain(const unsigned int off,
    ChVectorDynamic<>& R,
    const double c,
    const ChOverlapTest& filter
) {
    if (filter.IsInto(this->GetCenter()))
        this->NodeIntLoadResidual_F(off, R, c);
}

void ChNodeBase::NodeIntLoadResidual_Mv_domain(const unsigned int off,
    ChVectorDynamic<>& R,
    const ChVectorDynamic<>& w,
    const double c,
    const ChOverlapTest& filter
) {
    if (filter.IsInto(this->GetCenter()))
        this->NodeIntLoadResidual_Mv(off, R, w, c);
}




void ChNodeBase::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChNodeBase>();
    // serialize all member data:
    archive_out << CHNVP(m_tag);
}

void ChNodeBase::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChNodeBase>();
    // deserialize all member data:
    archive_in >> CHNVP(m_tag);
}

}  // end namespace chrono
