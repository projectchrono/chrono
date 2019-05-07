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

// Computes the product of the inverse mass matrix by a vector, and set in result: result = [invMb]*vect
void ChVariablesNode::Compute_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const {
    assert(vect.GetRows() == Get_ndof());
    assert(result.GetRows() == Get_ndof());
    // optimized unrolled operations
    double inv_mass = 1.0 / mass;
    result(0) = inv_mass * vect(0);
    result(1) = inv_mass * vect(1);
    result(2) = inv_mass * vect(2);
}

// Computes the product of the inverse mass matrix by a vector, and increment result: result += [invMb]*vect
void ChVariablesNode::Compute_inc_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const {
    assert(vect.GetRows() == Get_ndof());
    assert(result.GetRows() == Get_ndof());
    // optimized unrolled operations
    double inv_mass = 1.0 / mass;
    result(0) += inv_mass * vect(0);
    result(1) += inv_mass * vect(1);
    result(2) += inv_mass * vect(2);
}

// Computes the product of the mass matrix by a vector, and set in result: result = [Mb]*vect
void ChVariablesNode::Compute_inc_Mb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const {
    assert(result.GetRows() == vect.GetRows());
    assert(vect.GetRows() == Get_ndof());
    // optimized unrolled operations
    result(0) += mass * vect(0);
    result(1) += mass * vect(1);
    result(2) += mass * vect(2);
}

// Computes the product of the corresponding block in the
// system matrix (ie. the mass matrix) by 'vect', scale by c_a, and add to 'result'.
// NOTE: the 'vect' and 'result' vectors must already have
// the size of the total variables&constraints in the system; the procedure
// will use the ChVariable offsets (that must be already updated) to know the
// indexes in result and vect.
void ChVariablesNode::MultiplyAndAdd(ChMatrix<double>& result, const ChMatrix<double>& vect, const double c_a) const {
    assert(result.GetColumns() == 1 && vect.GetColumns() == 1);
    // optimized unrolled operations
    double scaledmass = c_a * mass;
    result(this->offset) += scaledmass * vect(this->offset);
    result(this->offset + 1) += scaledmass * vect(this->offset + 1);
    result(this->offset + 2) += scaledmass * vect(this->offset + 2);
}

// Add the diagonal of the mass matrix scaled by c_a, to 'result'.
// NOTE: the 'result' vector must already have the size of system unknowns, ie
// the size of the total variables&constraints in the system; the procedure
// will use the ChVariable offset (that must be already updated) as index.
void ChVariablesNode::DiagonalAdd(ChMatrix<double>& result, const double c_a) const {
    assert(result.GetColumns() == 1);
    result(this->offset) += c_a * mass;
    result(this->offset + 1) += c_a * mass;
    result(this->offset + 2) += c_a * mass;
}

// Build the mass matrix (for these variables) scaled by c_a, storing
// it in 'storage' sparse matrix, at given column/row offset.
// Note, most iterative solvers don't need to know mass matrix explicitly.
// Optimized: doesn't fill unneeded elements except mass.
void ChVariablesNode::Build_M(ChSparseMatrix& storage, int insrow, int inscol, const double c_a) {
    double scaledmass = c_a * mass;
    storage.SetElement(insrow + 0, inscol + 0, scaledmass);
    storage.SetElement(insrow + 1, inscol + 1, scaledmass);
    storage.SetElement(insrow + 2, inscol + 2, scaledmass);
}

void ChVariablesNode::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChVariablesNode>();
    // serialize parent class
    ChVariables::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(mass);
}

void ChVariablesNode::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChVariablesNode>();
    // deserialize parent class
    ChVariables::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(mass);
    SetNodeMass(mass);
}

}  // end namespace chrono
