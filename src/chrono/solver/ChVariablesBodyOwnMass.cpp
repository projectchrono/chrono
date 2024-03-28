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

#include "chrono/solver/ChVariablesBodyOwnMass.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChVariablesBodyOwnMass)

ChVariablesBodyOwnMass::ChVariablesBodyOwnMass() : mass(1), inv_mass(1), inertia(1), inv_inertia(1) {}

ChVariablesBodyOwnMass& ChVariablesBodyOwnMass::operator=(const ChVariablesBodyOwnMass& other) {
    if (&other == this)
        return *this;

    // copy parent class data
    ChVariablesBody::operator=(other);

    // copy class data
    mass = other.mass;
    inv_mass = other.inv_mass;

    inertia = other.inertia;
    inv_inertia = other.inv_inertia;

    return *this;
}

void ChVariablesBodyOwnMass::SetBodyInertia(const ChMatrix33<>& minertia) {
    inertia = minertia;
    inv_inertia = inertia.inverse();
}

void ChVariablesBodyOwnMass::SetBodyMass(const double mmass) {
    mass = mmass;
    if (mass)
        inv_mass = 1.0 / mass;
    else
        inv_mass = 1e32;
}

void ChVariablesBodyOwnMass::ComputeMassInverseTimesVector(ChVectorRef result, ChVectorConstRef vect) const {
    assert(vect.size() == ndof);
    assert(result.size() == ndof);

    // optimized unrolled operations
    result(0) = inv_mass * vect(0);
    result(1) = inv_mass * vect(1);
    result(2) = inv_mass * vect(2);
    result(3) = inv_inertia(0, 0) * vect(3) + inv_inertia(0, 1) * vect(4) + inv_inertia(0, 2) * vect(5);
    result(4) = inv_inertia(1, 0) * vect(3) + inv_inertia(1, 1) * vect(4) + inv_inertia(1, 2) * vect(5);
    result(5) = inv_inertia(2, 0) * vect(3) + inv_inertia(2, 1) * vect(4) + inv_inertia(2, 2) * vect(5);
}

void ChVariablesBodyOwnMass::AddMassTimesVector(ChVectorRef result, ChVectorConstRef vect) const {
    assert(result.size() == ndof);
    assert(vect.size() == ndof);

    // optimized unrolled operations
    result(0) += mass * vect(0);
    result(1) += mass * vect(1);
    result(2) += mass * vect(2);
    result(3) += (inertia(0, 0) * vect(3) + inertia(0, 1) * vect(4) + inertia(0, 2) * vect(5));
    result(4) += (inertia(1, 0) * vect(3) + inertia(1, 1) * vect(4) + inertia(1, 2) * vect(5));
    result(5) += (inertia(2, 0) * vect(3) + inertia(2, 1) * vect(4) + inertia(2, 2) * vect(5));
}

void ChVariablesBodyOwnMass::AddMassTimesVectorInto(ChVectorRef result, ChVectorConstRef vect, const double ca) const {
    // optimized unrolled operations
    double q0 = vect(offset + 0);
    double q1 = vect(offset + 1);
    double q2 = vect(offset + 2);
    double q3 = vect(offset + 3);
    double q4 = vect(offset + 4);
    double q5 = vect(offset + 5);
    double scaledmass = ca * mass;
    result(offset + 0) += scaledmass * q0;
    result(offset + 1) += scaledmass * q1;
    result(offset + 2) += scaledmass * q2;
    result(offset + 3) += ca * (inertia(0, 0) * q3 + inertia(0, 1) * q4 + inertia(0, 2) * q5);
    result(offset + 4) += ca * (inertia(1, 0) * q3 + inertia(1, 1) * q4 + inertia(1, 2) * q5);
    result(offset + 5) += ca * (inertia(2, 0) * q3 + inertia(2, 1) * q4 + inertia(2, 2) * q5);
}

void ChVariablesBodyOwnMass::AddMassDiagonalInto(ChVectorRef result, const double ca) const {
    result(offset + 0) += ca * mass;
    result(offset + 1) += ca * mass;
    result(offset + 2) += ca * mass;
    result(offset + 3) += ca * inertia(0, 0);
    result(offset + 4) += ca * inertia(1, 1);
    result(offset + 5) += ca * inertia(2, 2);
}

void ChVariablesBodyOwnMass::PasteMassInto(ChSparseMatrix& mat,
                                           unsigned int start_row,
                                           unsigned int start_col,
                                           const double ca) const {
    mat.SetElement(offset + start_row + 0, offset + start_col + 0, ca * mass);
    mat.SetElement(offset + start_row + 1, offset + start_col + 1, ca * mass);
    mat.SetElement(offset + start_row + 2, offset + start_col + 2, ca * mass);
    ChMatrix33<> scaledJ = inertia * ca;
    PasteMatrix(mat, scaledJ, offset + start_row + 3, offset + start_col + 3);
}

void ChVariablesBodyOwnMass::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChVariablesBodyOwnMass>();
    // serialize parent class
    ChVariablesBody::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(mass);
    archive_out << CHNVP(inertia);
}

void ChVariablesBodyOwnMass::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChVariablesBodyOwnMass>();
    // deserialize parent class
    ChVariablesBody::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(mass);
    archive_in >> CHNVP(inertia);
    SetBodyMass(mass);
    SetBodyInertia(inertia);
}

}  // end namespace chrono
