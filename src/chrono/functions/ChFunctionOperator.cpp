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

#include "chrono/functions/ChFunctionOperator.h"

namespace chrono {

// Register into the object m_first_functory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionOperator)

ChFunctionOperator::ChFunctionOperator() {
    m_op_type = ChFunctionOperator::ADD;
    m_first_fun = chrono_types::make_shared<ChFunctionConst>();
    m_second_fun = chrono_types::make_shared<ChFunctionConst>();
}

ChFunctionOperator::ChFunctionOperator(const ChFunctionOperator& other) {
    m_op_type = other.m_op_type;
    m_first_fun = std::shared_ptr<ChFunction>(other.m_first_fun->Clone());
    m_second_fun = std::shared_ptr<ChFunction>(other.m_second_fun->Clone());
}

double ChFunctionOperator::GetVal(double x) const {
    double res;

    switch (m_op_type) {
        case ChFunctionOperator::ADD:
            res = m_first_fun->GetVal(x) + m_second_fun->GetVal(x);
            break;
        case ChFunctionOperator::SUB:
            res = m_first_fun->GetVal(x) - m_second_fun->GetVal(x);
            break;
        case ChFunctionOperator::MUL:
            res = m_first_fun->GetVal(x) * m_second_fun->GetVal(x);
            break;
        case ChFunctionOperator::DIV:
            res = m_first_fun->GetVal(x) / m_second_fun->GetVal(x);
            break;
        case ChFunctionOperator::POW:
            res = std::pow(m_first_fun->GetVal(x), m_second_fun->GetVal(x));
            break;
        case ChFunctionOperator::MAX:
            res = std::max(m_first_fun->GetVal(x), m_second_fun->GetVal(x));
            break;
        case ChFunctionOperator::MIN:
            res = std::min(m_first_fun->GetVal(x), m_second_fun->GetVal(x));
            break;
        case ChFunctionOperator::MODULO:
            res = fmod(m_first_fun->GetVal(x), m_second_fun->GetVal(x));
            break;
        case ChFunctionOperator::FABS:
            res = fabs(m_first_fun->GetVal(x));
            break;
        case ChFunctionOperator::FUNCT:
            res = m_first_fun->GetVal(m_second_fun->GetVal(x));
            break;
        default:
            res = 0;
            break;
    }
    return res;
}

void ChFunctionOperator::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionOperator>();
    // serialize parent class
    ChFunction::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_first_fun);
    archive_out << CHNVP(m_second_fun);
    eChOperation_mapper mmapper;
    archive_out << CHNVP(mmapper(m_op_type), "operation_type");
}

void ChFunctionOperator::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFunctionOperator>();
    // deserialize parent class
    ChFunction::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(m_first_fun);
    archive_in >> CHNVP(m_second_fun);
    eChOperation_mapper mmapper;
    archive_in >> CHNVP(mmapper(m_op_type), "operation_type");
}

}  // end namespace chrono
