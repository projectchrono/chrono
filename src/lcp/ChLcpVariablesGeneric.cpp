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
//   ChLcpVariablesGeneric.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 

#include "ChLcpVariablesGeneric.h"
#include "core/ChMemory.h" // must be after system's include (memory leak debugger).


namespace chrono 
{


ChLcpVariablesGeneric& ChLcpVariablesGeneric::operator=(const ChLcpVariablesGeneric& other)
{
	if (&other == this) return *this;

	// copy parent class data
	ChLcpVariables::operator=(other);

	// copy class data

	if (other.Mmass)
	{
		if (Mmass==NULL)
			Mmass = new ChMatrixDynamic<>;
		Mmass->CopyFromMatrix(*other.Mmass);
	}
	else
	{
		if (Mmass) delete Mmass;
		Mmass=NULL;
	}

	if (other.inv_Mmass)
	{
		if (inv_Mmass==NULL)
			inv_Mmass = new ChMatrixDynamic<>;
		inv_Mmass->CopyFromMatrix(*other.inv_Mmass);
	}
	else
	{
		if (inv_Mmass) delete inv_Mmass;
		inv_Mmass=NULL;
	}


	return *this;
}

/// Computes the product of the inverse mass matrix by a
/// vector, and add to result: result = [invMb]*vect
void ChLcpVariablesGeneric::Compute_invMb_v(ChMatrix<float>& result, const ChMatrix<float>& vect) const
{
    assert(result.GetRows() == vect.GetRows());
    assert(vect.GetRows() == Get_ndof());
    result = (*inv_Mmass)*vect;
}

void ChLcpVariablesGeneric::Compute_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const
{
    assert(result.GetRows() == vect.GetRows());
    assert(vect.GetRows() == Get_ndof());
    result = (*inv_Mmass)*vect;
}

/// Computes the product of the inverse mass matrix by a
/// vector, and increment result: result += [invMb]*vect
void ChLcpVariablesGeneric::Compute_inc_invMb_v(ChMatrix<float>& result, const ChMatrix<float>& vect) const
{
    assert(result.GetRows() == vect.GetRows());
    assert(vect.GetRows() == Get_ndof());
    result += (*inv_Mmass)*vect;
}

void ChLcpVariablesGeneric::Compute_inc_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const
{
    assert(result.GetRows() == vect.GetRows());
    assert(vect.GetRows() == Get_ndof());
    result += (*inv_Mmass)*vect;
}

/// Computes the product of the mass matrix by a
/// vector, and set in result: result = [Mb]*vect
void ChLcpVariablesGeneric::Compute_inc_Mb_v(ChMatrix<float>& result, const ChMatrix<float>& vect) const
{
    assert(result.GetRows() == vect.GetRows());
    assert(vect.GetRows() == Get_ndof());
    result += (*Mmass)*vect;
}

void ChLcpVariablesGeneric::Compute_inc_Mb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const
{
    assert(result.GetRows() == vect.GetRows());
    assert(vect.GetRows() == Get_ndof());
    result += (*Mmass)*vect;
}

/// Computes the product of the corresponding block in the 
/// system matrix (ie. the mass matrix) by 'vect', and add to 'result'. 
/// NOTE: the 'vect' and 'result' vectors must already have
/// the size of the total variables&constraints in the system; the procedure
/// will use the ChVariable offsets (that must be already updated) to know the 
/// indexes in result and vect.
void ChLcpVariablesGeneric::MultiplyAndAdd(ChMatrix<double>& result, const ChMatrix<double>& vect) const
{
    assert(result.GetColumns() == 1 && vect.GetColumns() == 1);

    for (int i = 0; i < Mmass->GetRows(); i++)
    {
        double tot = 0;
        for (int j = 0; j < Mmass->GetColumns(); j++)
        {
            tot += (*Mmass)(i, j)* vect(this->offset + i);
        }
        result(this->offset + i) += tot;
    }
}

/// Add the diagonal of the mass matrix (as a column vector) to 'result'.
/// NOTE: the 'result' vector must already have the size of system unknowns, ie
/// the size of the total variables&constraints in the system; the procedure
/// will use the ChVariable offset (that must be already updated) as index.
void ChLcpVariablesGeneric::DiagonalAdd(ChMatrix<double>& result) const
{
    assert(result.GetColumns() == 1);
    for (int i = 0; i < Mmass->GetRows(); i++)
    {
        result(this->offset + i) += (*Mmass)(i, i);
    }
}


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLcpVariablesGeneric> a_registration_ChLcpVariablesGeneric;



} // END_OF_NAMESPACE____


