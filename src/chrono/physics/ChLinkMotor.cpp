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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/physics/ChLinkMotor.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChLinkMotor)  NO! ABSTRACT!

ChLinkMotor::ChLinkMotor() {
	m_func = chrono_types::make_shared<ChFunction_Const>(0); // defaults to no motion.
}

ChLinkMotor::ChLinkMotor(const ChLinkMotor& other) : ChLinkMateGeneric(other) {
    m_func = other.m_func;
}

ChLinkMotor::~ChLinkMotor() {}

void ChLinkMotor::Update(double mytime, bool update_assets) {
    ChLinkMateGeneric::Update(mytime, update_assets);
    m_func->Update(mytime);
}

void ChLinkMotor::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMotor>();

    // serialize parent class
    ChLinkMateGeneric::ArchiveOut(marchive);

    // serialize all member data:
    marchive << CHNVP(m_func);
}

void ChLinkMotor::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChLinkMotor>();

    // deserialize parent class
    ChLinkMateGeneric::ArchiveIn(marchive);

    // deserialize all member data:
    marchive >> CHNVP(m_func);
}

}  // end namespace chrono
