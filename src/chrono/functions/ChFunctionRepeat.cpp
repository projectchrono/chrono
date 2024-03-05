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

#include "chrono/functions/ChFunctionRepeat.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionRepeat)

ChFunctionRepeat::ChFunctionRepeat(std::shared_ptr<ChFunction> func, double start, double length, double phase)
    : fa(func), m_slice_start(start), m_slice_width(length), m_slice_shift(phase) {}

ChFunctionRepeat::ChFunctionRepeat(const ChFunctionRepeat& other) {
    m_slice_start = other.m_slice_start;
    m_slice_width = other.m_slice_width;
    m_slice_shift = other.m_slice_shift;
    fa = std::shared_ptr<ChFunction>(other.fa->Clone());
}

double ChFunctionRepeat::GetVal(double x) const {
    return fa->GetVal(this->m_slice_start + fmod(x + this->m_slice_shift, this->m_slice_width));
}

void ChFunctionRepeat::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChFunctionRepeat>();
    // serialize parent class
    ChFunction::ArchiveOut(archive);
    // serialize all member data:
    archive << CHNVP(fa);
    archive << CHNVP(m_slice_start);
    archive << CHNVP(m_slice_width);
    archive << CHNVP(m_slice_shift);
}

void ChFunctionRepeat::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead<ChFunctionRepeat>();
    // deserialize parent class
    ChFunction::ArchiveIn(archive);
    // stream in all member data:
    archive >> CHNVP(fa);
    archive >> CHNVP(m_slice_start);
    archive >> CHNVP(m_slice_width);
    archive >> CHNVP(m_slice_shift);
}

}  // end namespace chrono
