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

#include "chrono/functions/ChFunctionConst.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionConst)

ChFunctionConst::ChFunctionConst() : m_constant(0) {}

ChFunctionConst::ChFunctionConst(double y_constant) : m_constant(y_constant) {}

ChFunctionConst::ChFunctionConst(const ChFunctionConst& other) : ChFunction(other) {
    m_constant = other.m_constant;
}

void ChFunctionConst::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionConst>();
    // serialize parent class
    ChFunction::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_constant);
}

void ChFunctionConst::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFunctionConst>();
    // deserialize parent class
    ChFunction::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(m_constant);
}

}  // end namespace chrono
