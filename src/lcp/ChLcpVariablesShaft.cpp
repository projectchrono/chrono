//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//


#include "ChLcpVariablesShaft.h"

namespace chrono {


ChLcpVariablesShaft& ChLcpVariablesShaft::operator=(const ChLcpVariablesShaft& other)
{
  if (&other == this) return *this;

  // copy parent class data
  ChLcpVariables::operator=(other);

  // copy class data
  m_shaft = other.m_shaft;
  m_inertia = other.m_inertia;

  return *this;
}

/// Computes the product of the inverse mass matrix by a
/// vector, and set in result: result = [invMb]*vect
void ChLcpVariablesShaft::Compute_invMb_v(ChMatrix<float>& result, const ChMatrix<float>& vect) const
{
  assert(vect.GetRows() == Get_ndof());
  assert(result.GetRows() == Get_ndof());
  result(0) = (float)m_inv_inertia * vect(0);
};
void ChLcpVariablesShaft::Compute_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const
{
  assert(vect.GetRows() == Get_ndof());
  assert(result.GetRows() == Get_ndof());
  result(0) = m_inv_inertia * vect(0);
};

/// Computes the product of the inverse mass matrix by a
/// vector, and increment result: result += [invMb]*vect
void ChLcpVariablesShaft::Compute_inc_invMb_v(ChMatrix<float>& result, const ChMatrix<float>& vect) const
{
  assert(vect.GetRows() == Get_ndof());
  assert(result.GetRows() == Get_ndof());
  result(0) += (float)m_inv_inertia * vect(0);
};
void ChLcpVariablesShaft::Compute_inc_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const
{
  assert(vect.GetRows() == Get_ndof());
  assert(result.GetRows() == Get_ndof());
  result(0) += (float)m_inv_inertia * vect(0);
};


/// Computes the product of the mass matrix by a
/// vector, and set in result: result = [Mb]*vect
void ChLcpVariablesShaft::Compute_inc_Mb_v(ChMatrix<float>& result, const ChMatrix<float>& vect) const
{
  assert(result.GetRows() == Get_ndof());
  assert(vect.GetRows() == Get_ndof());
  result(0) += (float)m_inertia * vect(0);
};
void ChLcpVariablesShaft::Compute_inc_Mb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const
{
  assert(result.GetRows() == vect.GetRows());
  assert(vect.GetRows() == Get_ndof());
  result(0) += m_inertia * vect(0);
};

/// Computes the product of the corresponding block in the 
/// system matrix (ie. the mass matrix) by 'vect', and add to 'result'. 
/// NOTE: the 'vect' and 'result' vectors must already have
/// the size of the total variables&constraints in the system; the procedure
/// will use the ChVariable offsets (that must be already updated) to know the 
/// indexes in result and vect.
void ChLcpVariablesShaft::MultiplyAndAdd(ChMatrix<double>& result, const ChMatrix<double>& vect) const
{
  assert(result.GetColumns() == 1 && vect.GetColumns() == 1);
  result(this->offset) += m_inertia * vect(this->offset);
}

/// Add the diagonal of the mass matrix (as a column vector) to 'result'.
/// NOTE: the 'result' vector must already have the size of system unknowns, ie
/// the size of the total variables&constraints in the system; the procedure
/// will use the ChVariable offset (that must be already updated) as index.
void ChLcpVariablesShaft::DiagonalAdd(ChMatrix<double>& result) const
{
  assert(result.GetColumns() == 1);
  result(this->offset) += m_inertia;
}

/// Build the mass matrix (for these variables) storing
/// it in 'storage' sparse matrix, at given column/row offset.
/// Note, most iterative solvers don't need to know mass matrix explicitly.
/// Optimised: doesn't fill unneeded elements except mass.
void ChLcpVariablesShaft::Build_M(ChSparseMatrix& storage, int insrow, int inscol)
{
  storage.SetElement(insrow + 0, inscol + 0, m_inertia);
}




// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLcpVariablesShaft> a_registration_ChLcpVariablesShaft;



} // END_OF_NAMESPACE____


