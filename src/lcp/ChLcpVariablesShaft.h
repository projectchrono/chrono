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

#ifndef CHLCPVARIABLESSHAFT_H
#define CHLCPVARIABLESSHAFT_H


#include "ChLcpVariables.h"


namespace chrono {

class ChShaft;

///  Specialized class for representing a 1-DOF item for an LCP system, that is
///  a shaft, with inertia and associated variable (rotational speed)


class ChApi ChLcpVariablesShaft : public ChLcpVariables
{
  CH_RTTI(ChLcpVariablesShaft, ChLcpVariables)

private:

  ChShaft* m_shaft;

  double m_inertia;
  double m_inv_inertia;

public:

  //
  // CONSTRUCTORS
  //

  ChLcpVariablesShaft() : ChLcpVariables(1)
  {
    m_shaft = 0;
    m_inertia = 1.0;
    m_inv_inertia = 1.0;
  }

  virtual ~ChLcpVariablesShaft() {}

  /// Assignment operator: copy from other object
  ChLcpVariablesShaft& operator=(const ChLcpVariablesShaft& other);

  //
  // FUNCTIONS
  //

  /// The number of scalar variables in the vector qb
  /// (dof=degrees of freedom)
  virtual int Get_ndof() const { return 1; }

  /// Get the inertia associated with rotation of the shaft
  double GetInertia() const { return m_inertia; }

  /// Get the inverse of the inertia associated with rotation of the shaft
  double GetInvInertia() const { return m_inv_inertia; }

  /// Set the inertia associated with rotation of the shaft
  void SetInertia(double inertia)
  {
    m_inertia = inertia;
    m_inv_inertia = 1 / inertia;
  }

  ChShaft* GetShaft() { return m_shaft; }
  void SetShaft(ChShaft* shaft) { m_shaft = shaft; }

  /// Computes the product of the inverse mass matrix by a
  /// vector, and set in result: result = [invMb]*vect
  void Compute_invMb_v(ChMatrix<float>& result, const ChMatrix<float>& vect) const;
  void Compute_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const;

  /// Computes the product of the inverse mass matrix by a
  /// vector, and increment result: result += [invMb]*vect
  void Compute_inc_invMb_v(ChMatrix<float>& result, const ChMatrix<float>& vect) const;
  void Compute_inc_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const;


  /// Computes the product of the mass matrix by a
  /// vector, and set in result: result = [Mb]*vect
  void Compute_inc_Mb_v(ChMatrix<float>& result, const ChMatrix<float>& vect) const;
  void Compute_inc_Mb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const;

  /// Computes the product of the corresponding block in the 
  /// system matrix (ie. the mass matrix) by 'vect', and add to 'result'. 
  /// NOTE: the 'vect' and 'result' vectors must already have
  /// the size of the total variables&constraints in the system; the procedure
  /// will use the ChVariable offsets (that must be already updated) to know the 
  /// indexes in result and vect.
  void MultiplyAndAdd(ChMatrix<double>& result, const ChMatrix<double>& vect) const;

  /// Add the diagonal of the mass matrix (as a column vector) to 'result'.
  /// NOTE: the 'result' vector must already have the size of system unknowns, ie
  /// the size of the total variables&constraints in the system; the procedure
  /// will use the ChVariable offset (that must be already updated) as index.
  void DiagonalAdd(ChMatrix<double>& result) const;

  /// Build the mass matrix (for these variables) storing
  /// it in 'storage' sparse matrix, at given column/row offset.
  /// Note, most iterative solvers don't need to know mass matrix explicitly.
  /// Optimised: doesn't fill unneeded elements except mass.
  void Build_M(ChSparseMatrix& storage, int insrow, int inscol);

};




} // END_OF_NAMESPACE____




#endif
