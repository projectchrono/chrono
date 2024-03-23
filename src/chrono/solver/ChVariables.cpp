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

#include "chrono/solver/ChVariables.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChVariables) \\ ABSTRACT: cannot be instantiated

ChVariables::ChVariables() : disabled(false), ndof(0), offset(0) {}

ChVariables::ChVariables(unsigned int dof) : disabled(false), ndof(dof), offset(0) {
    if (ndof > 0) {
        qb.setZero(ndof);
        fb.setZero(ndof);
    }
}

ChVariables& ChVariables::operator=(const ChVariables& other) {
    if (&other == this)
        return *this;

    disabled = other.disabled;

    qb = other.qb;
    fb = other.fb;

    ndof = other.ndof;
    offset = other.offset;

    return *this;
}

void ChVariables::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChVariables>();
    // serialize all member data:
    archive_out << CHNVP(ndof);
    archive_out << CHNVP(disabled);
    archive_out << CHNVP(offset);  // TODO: check_if_serialize_needed
}

void ChVariables::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChVariables>();
    // stream in all member data:
    archive_in >> CHNVP(ndof);
    archive_in >> CHNVP(disabled);
    archive_in >> CHNVP(offset);  // TODO: check_if_serialize_needed
}

}  // end namespace chrono
