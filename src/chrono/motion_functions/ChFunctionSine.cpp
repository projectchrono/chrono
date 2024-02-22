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

#include "chrono/motion_functions/ChFunctionSine.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionSine)

ChFunctionSine::ChFunctionSine(const ChFunctionSine& other) {
    m_ampl = other.m_ampl;
    m_phase = other.m_phase;
    m_angular_rate = other.m_angular_rate;
}

double ChFunctionSine::GetVal(double x) const {
    return m_ampl * (sin(m_phase + m_angular_rate * x));
}

double ChFunctionSine::GetDer(double x) const {
    return m_ampl * m_angular_rate * (cos(m_phase + m_angular_rate * x));
}

double ChFunctionSine::GetDer2(double x) const {
    return m_ampl * -m_angular_rate * m_angular_rate * (sin(m_phase + m_angular_rate * x));
}

void ChFunctionSine::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunctionSine>();
    // serialize parent class
    ChFunction::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(m_ampl);
    marchive << CHNVP(m_phase);
    marchive << CHNVP(m_angular_rate);
}

void ChFunctionSine::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChFunctionSine>();
    // deserialize parent class
    ChFunction::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(m_ampl);
    marchive >> CHNVP(m_phase);
    marchive >> CHNVP(m_angular_rate);
}

}  // end namespace chrono
