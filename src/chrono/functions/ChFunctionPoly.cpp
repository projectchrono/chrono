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

#include <cmath>

#include "chrono/functions/ChFunctionPoly.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionPoly)

ChFunctionPoly::ChFunctionPoly() {
    m_coeffs = {0};
}

ChFunctionPoly::ChFunctionPoly(const ChFunctionPoly& other) {
    m_coeffs = other.m_coeffs;
}

double ChFunctionPoly::GetVal(double x) const {
    double total = 0;
    for (int i = 0; i <= m_coeffs.size(); i++) {
        total += (m_coeffs[i] * std::pow(x, (double)i));
    }
    return total;
}

double ChFunctionPoly::GetDer(double x) const {
    double total = 0;
    for (int i = 1; i <= m_coeffs.size(); i++) {
        total += ((double)i * m_coeffs[i] * std::pow(x, ((double)(i - 1))));
    }
    return total;
}

double ChFunctionPoly::GetDer2(double x) const {
    double total = 0;
    for (int i = 2; i <= m_coeffs.size(); i++) {
        total += ((double)(i * (i - 1)) * m_coeffs[i] * std::pow(x, ((double)(i - 2))));
    }
    return total;
}

void ChFunctionPoly::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionPoly>();
    // serialize parent class
    ChFunction::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_coeffs);
}

void ChFunctionPoly::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFunctionPoly>();
    // deserialize parent class
    ChFunction::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(m_coeffs);
}

}  // end namespace chrono
