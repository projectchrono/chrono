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

#include "chrono/motion_functions/ChFunctionDerive.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionDerive)

ChFunctionDerive::ChFunctionDerive(const ChFunctionDerive& other) {
    m_der_order = other.m_der_order;
    m_operand_fun = std::shared_ptr<ChFunction>(other.m_operand_fun->Clone());
}

double ChFunctionDerive::GetVal(double x) const {
    return m_operand_fun->GetDer(x);
}

void ChFunctionDerive::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunctionDerive>();
    // serialize parent class
    ChFunction::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(m_operand_fun);
    marchive << CHNVP(m_der_order);
}

void ChFunctionDerive::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunctionDerive>();
    // deserialize parent class
    ChFunction::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(m_operand_fun);
    marchive >> CHNVP(m_der_order);
}

}  // end namespace chrono
