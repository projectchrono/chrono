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

#include "chrono/motion_functions/ChFunctionOperation.h"

namespace chrono {

// Register into the object m_first_functory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionOperation)

ChFunctionOperation::ChFunctionOperation() {
    m_op_type = ChFunctionOperation::ADD;
    m_first_fun = chrono_types::make_shared<ChFunctionConst>();
    m_second_fun = chrono_types::make_shared<ChFunctionConst>();
}

ChFunctionOperation::ChFunctionOperation(const ChFunctionOperation& other) {
    m_op_type = other.m_op_type;
    m_first_fun = std::shared_ptr<ChFunction>(other.m_first_fun->Clone());
    m_second_fun = std::shared_ptr<ChFunction>(other.m_second_fun->Clone());
}

double ChFunctionOperation::GetVal(double x) const {
    double res;

    switch (m_op_type) {
        case ChFunctionOperation::ADD:
            res = m_first_fun->GetVal(x) + m_second_fun->GetVal(x);
            break;
        case ChFunctionOperation::SUB:
            res = m_first_fun->GetVal(x) - m_second_fun->GetVal(x);
            break;
        case ChFunctionOperation::MUL:
            res = m_first_fun->GetVal(x) * m_second_fun->GetVal(x);
            break;
        case ChFunctionOperation::DIV:
            res = m_first_fun->GetVal(x) / m_second_fun->GetVal(x);
            break;
        case ChFunctionOperation::POW:
            res = pow(m_first_fun->GetVal(x), m_second_fun->GetVal(x));
            break;
        case ChFunctionOperation::MAX:
            res = std::max(m_first_fun->GetVal(x), m_second_fun->GetVal(x));
            break;
        case ChFunctionOperation::MIN:
            res = std::min(m_first_fun->GetVal(x), m_second_fun->GetVal(x));
            break;
        case ChFunctionOperation::MODULO:
            res = fmod(m_first_fun->GetVal(x), m_second_fun->GetVal(x));
            break;
        case ChFunctionOperation::FABS:
            res = fabs(m_first_fun->GetVal(x));
            break;
        case ChFunctionOperation::FUNCT:
            res = m_first_fun->GetVal(m_second_fun->GetVal(x));
            break;
        default:
            res = 0;
            break;
    }
    return res;
}

void ChFunctionOperation::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunctionOperation>();
    // serialize parent class
    ChFunction::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(m_first_fun);
    marchive << CHNVP(m_second_fun);
    eChOperation_mapper mmapper;
    marchive << CHNVP(mmapper(m_op_type), "operation_type");
}

void ChFunctionOperation::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChFunctionOperation>();
    // deserialize parent class
    ChFunction::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(m_first_fun);
    marchive >> CHNVP(m_second_fun);
    eChOperation_mapper mmapper;
    marchive >> CHNVP(mmapper(m_op_type), "operation_type");
}

}  // end namespace chrono
