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
    m_func = chrono_types::make_shared<ChFunctionConst>(0);  // defaults to no motion.
}

ChLinkMotor::ChLinkMotor(const ChLinkMotor& other) : ChLinkMateGeneric(other) {
    m_func = other.m_func;
}

ChLinkMotor::~ChLinkMotor() {}

void ChLinkMotor::Update(double time, bool update_assets) {
    ChLinkMateGeneric::Update(time, update_assets);
    m_func->Update(time);
}

void ChLinkMotor::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkMotor>();

    // serialize parent class
    ChLinkMateGeneric::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(m_func);
}

void ChLinkMotor::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkMotor>();

    // deserialize parent class
    ChLinkMateGeneric::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(m_func);
}

}  // end namespace chrono
