//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChLcpVariablesNode.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 

#include "ChLcpVariablesNode.h"
 

namespace chrono 
{



ChLcpVariablesNode& ChLcpVariablesNode::operator=(const ChLcpVariablesNode& other)
{
	if (&other == this) return *this;

	// copy parent class data
	ChLcpVariables::operator=(other);

	// copy class data
	user_data = other.user_data;
	mass = other.mass;

	return *this;
}

/// Computes the product of the inverse mass matrix by a
/// vector, and set in result: result = [invMb]*vect
void ChLcpVariablesNode::Compute_invMb_v(ChMatrix<float>& result, const ChMatrix<float>& vect) const
{
    assert(vect.GetRows() == Get_ndof());
    assert(result.GetRows() == Get_ndof());
    // optimized unrolled operations
    double inv_mass = 1.0 / mass;
    result(0) = (float)inv_mass * vect(0);
    result(1) = (float)inv_mass * vect(1);
    result(2) = (float)inv_mass * vect(2);
};
void ChLcpVariablesNode::Compute_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const
{
    assert(vect.GetRows() == Get_ndof());
    assert(result.GetRows() == Get_ndof());
    // optimized unrolled operations
    double inv_mass = 1.0 / mass;
    result(0) = inv_mass * vect(0);
    result(1) = inv_mass * vect(1);
    result(2) = inv_mass * vect(2);
};

/// Computes the product of the inverse mass matrix by a
/// vector, and increment result: result += [invMb]*vect
void ChLcpVariablesNode::Compute_inc_invMb_v(ChMatrix<float>& result, const ChMatrix<float>& vect) const
{
    assert(vect.GetRows() == Get_ndof());
    assert(result.GetRows() == Get_ndof());
    // optimized unrolled operations
    double inv_mass = 1.0 / mass;
    result(0) += (float)inv_mass * vect(0);
    result(1) += (float)inv_mass * vect(1);
    result(2) += (float)inv_mass * vect(2);
};
void ChLcpVariablesNode::Compute_inc_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const
{
    assert(vect.GetRows() == Get_ndof());
    assert(result.GetRows() == Get_ndof());
    // optimized unrolled operations
    double inv_mass = 1.0 / mass;
    result(0) += inv_mass * vect(0);
    result(1) += inv_mass * vect(1);
    result(2) += inv_mass * vect(2);
};


/// Computes the product of the mass matrix by a
/// vector, and set in result: result = [Mb]*vect
void ChLcpVariablesNode::Compute_inc_Mb_v(ChMatrix<float>& result, const ChMatrix<float>& vect) const
{
    assert(result.GetRows() == Get_ndof());
    assert(vect.GetRows() == Get_ndof());
    // optimized unrolled operations
    result(0) += (float)mass * vect(0);
    result(1) += (float)mass * vect(1);
    result(2) += (float)mass * vect(2);
};
void ChLcpVariablesNode::Compute_inc_Mb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const
{
    assert(result.GetRows() == vect.GetRows());
    assert(vect.GetRows() == Get_ndof());
    // optimized unrolled operations
    result(0) += mass * vect(0);
    result(1) += mass * vect(1);
    result(2) += mass * vect(2);
};

/// Computes the product of the corresponding block in the 
/// system matrix (ie. the mass matrix) by 'vect', and add to 'result'. 
/// NOTE: the 'vect' and 'result' vectors must already have
/// the size of the total variables&constraints in the system; the procedure
/// will use the ChVariable offsets (that must be already updated) to know the 
/// indexes in result and vect.
void ChLcpVariablesNode::MultiplyAndAdd(ChMatrix<double>& result, const ChMatrix<double>& vect) const
{
    assert(result.GetColumns() == 1 && vect.GetColumns() == 1);
    // optimized unrolled operations
    result(this->offset) += mass * vect(this->offset);
    result(this->offset + 1) += mass * vect(this->offset + 1);
    result(this->offset + 2) += mass * vect(this->offset + 2);
}

/// Add the diagonal of the mass matrix (as a column vector) to 'result'.
/// NOTE: the 'result' vector must already have the size of system unknowns, ie
/// the size of the total variables&constraints in the system; the procedure
/// will use the ChVariable offset (that must be already updated) as index.
void ChLcpVariablesNode::DiagonalAdd(ChMatrix<double>& result) const
{
    assert(result.GetColumns() == 1);
    result(this->offset) += mass;
    result(this->offset + 1) += mass;
    result(this->offset + 2) += mass;
}

/// Build the mass matrix (for these variables) storing
/// it in 'storage' sparse matrix, at given column/row offset.
/// Note, most iterative solvers don't need to know mass matrix explicitly.
/// Optimised: doesn't fill unneeded elements except mass.
void ChLcpVariablesNode::Build_M(ChSparseMatrix& storage, int insrow, int inscol)
{
    storage.SetElement(insrow + 0, inscol + 0, mass);
    storage.SetElement(insrow + 1, inscol + 1, mass);
    storage.SetElement(insrow + 2, inscol + 2, mass);
}




// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLcpVariablesNode> a_registration_ChLcpVariablesNode;



} // END_OF_NAMESPACE____


