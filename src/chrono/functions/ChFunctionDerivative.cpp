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

#include "chrono/functions/ChFunctionDerivative.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionDerivative)

ChFunctionDerivative::ChFunctionDerivative(const ChFunctionDerivative& other) {
    m_der_order = other.m_der_order;
    m_operand_fun = std::shared_ptr<ChFunction>(other.m_operand_fun->Clone());
}

double ChFunctionDerivative::GetVal(double x) const {
    return m_operand_fun->GetDer(x);
}

void ChFunctionDerivative::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionDerivative>();
    // serialize parent class
    ChFunction::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_operand_fun);
    archive_out << CHNVP(m_der_order);
}

void ChFunctionDerivative::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFunctionDerivative>();
    // deserialize parent class
    ChFunction::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(m_operand_fun);
    archive_in >> CHNVP(m_der_order);
}

}  // end namespace chrono
