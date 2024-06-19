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

#include "chrono/solver/ChVariablesNode.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChVariablesNode)

ChVariablesNode::ChVariablesNode() : ChVariables(3), user_data(nullptr), mass(1) {}

ChVariablesNode& ChVariablesNode::operator=(const ChVariablesNode& other) {
    if (&other == this)
        return *this;

    // copy parent class data
    ChVariables::operator=(other);

    // copy class data
    user_data = other.user_data;
    mass = other.mass;

    return *this;
}

void ChVariablesNode::ComputeMassInverseTimesVector(ChVectorRef result, ChVectorConstRef vect) const {
    assert(vect.size() == ndof);
    assert(result.size() == ndof);

    // optimized unrolled operations
    double inv_mass = 1.0 / mass;
    result(0) = inv_mass * vect(0);
    result(1) = inv_mass * vect(1);
    result(2) = inv_mass * vect(2);
}

void ChVariablesNode::AddMassTimesVector(ChVectorRef result, ChVectorConstRef vect) const {
    assert(result.size() == ndof);
    assert(vect.size() == ndof);

    // optimized unrolled operations
    result(0) += mass * vect(0);
    result(1) += mass * vect(1);
    result(2) += mass * vect(2);
}

void ChVariablesNode::AddMassTimesVectorInto(ChVectorRef result, ChVectorConstRef vect, const double ca) const {
    // optimized unrolled operations
    double scaledmass = ca * mass;
    result(offset + 0) += scaledmass * vect(offset);
    result(offset + 1) += scaledmass * vect(offset + 1);
    result(offset + 2) += scaledmass * vect(offset + 2);
}

void ChVariablesNode::AddMassDiagonalInto(ChVectorRef result, const double ca) const {
    result(offset + 0) += ca * mass;
    result(offset + 1) += ca * mass;
    result(offset + 2) += ca * mass;
}

void ChVariablesNode::PasteMassInto(ChSparseMatrix& mat,
                                    unsigned int start_row,
                                    unsigned int start_col,
                                    const double ca) const {
    double scaledmass = ca * mass;
    mat.SetElement(offset + start_row + 0, offset + start_col + 0, scaledmass);
    mat.SetElement(offset + start_row + 1, offset + start_col + 1, scaledmass);
    mat.SetElement(offset + start_row + 2, offset + start_col + 2, scaledmass);
}

void ChVariablesNode::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChVariablesNode>();
    // serialize parent class
    ChVariables::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(mass);
}

void ChVariablesNode::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChVariablesNode>();
    // deserialize parent class
    ChVariables::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(mass);
    SetNodeMass(mass);
}

}  // end namespace chrono
