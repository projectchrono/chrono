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

#include "chrono/functions/ChFunctionSine.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionSine)

ChFunctionSine::ChFunctionSine(const ChFunctionSine& other) {
    m_ampl = other.m_ampl;
    m_phase = other.m_phase;
    m_angular_rate = other.m_angular_rate;
}

double ChFunctionSine::GetVal(double x) const {
    return m_ampl * std::sin(m_angular_rate * x + m_phase);
}

double ChFunctionSine::GetDer(double x) const {
    return m_angular_rate * m_ampl * std::cos(m_angular_rate * x + m_phase);
}

double ChFunctionSine::GetDer2(double x) const {
    return -m_angular_rate * m_angular_rate * m_ampl * std::sin(m_angular_rate * x + m_phase);
}

double ChFunctionSine::GetDer3(double x) const {
    return -m_angular_rate * m_angular_rate * m_angular_rate * m_ampl * std::cos(m_angular_rate * x + m_phase);
}

void ChFunctionSine::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionSine>();
    // serialize parent class
    ChFunction::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_ampl);
    archive_out << CHNVP(m_phase);
    archive_out << CHNVP(m_angular_rate);
}

void ChFunctionSine::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFunctionSine>();
    // deserialize parent class
    ChFunction::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(m_ampl);
    archive_in >> CHNVP(m_phase);
    archive_in >> CHNVP(m_angular_rate);
}

}  // end namespace chrono
