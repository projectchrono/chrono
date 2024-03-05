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

#include "chrono/functions/ChFunctionMirror.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionMirror)

ChFunctionMirror::ChFunctionMirror() : m_mirror_axis(0) {
    m_operand_fun = chrono_types::make_shared<ChFunctionConst>();
}

ChFunctionMirror::ChFunctionMirror(const ChFunctionMirror& other) {
    m_mirror_axis = other.m_mirror_axis;
    m_operand_fun = std::shared_ptr<ChFunction>(other.m_operand_fun->Clone());
}

double ChFunctionMirror::GetVal(double x) const {
    if (x <= this->m_mirror_axis)
        return m_operand_fun->GetVal(x);
    return m_operand_fun->GetVal(2 * this->m_mirror_axis - x);
}

void ChFunctionMirror::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionMirror>();
    // serialize parent class
    ChFunction::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_operand_fun);
    archive_out << CHNVP(m_mirror_axis);
}

void ChFunctionMirror::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFunctionMirror>();
    // deserialize parent class
    ChFunction::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(m_operand_fun);
    archive_in >> CHNVP(m_mirror_axis);
}

}  // end namespace chrono
