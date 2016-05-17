// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHVARIABLES_H
#define CHVARIABLES_H

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMatrixDynamic.h"
#include "chrono/core/ChSparseMatrix.h"

namespace chrono {

///  Base class for representing items which introduce
/// 'variables', that is variables 'v' (and associated masses M)
/// for a sparse representation of the problem.
///
///  The problem is described by a variational inequality VI(Z*x-d,K):
///
///  | M -Cq'|*|q|- | f|= |0| , l \in Y, C \in Ny, normal cone to Y
///  | Cq -E | |l|  |-b|  |c|
///
/// Also Z symmetric by flipping sign of l_i: |M  Cq'|*| q|-| f|=|0|
///                                           |Cq  E | |-l| |-b| |c|
/// * case linear problem:  all Y_i = R, Ny=0, ex. all bilaterals
/// * case LCP: all Y_i = R+:  c>=0, l>=0, l*c=0
/// * case CCP: Y_i are friction cones
///
/// Note, all masses and variables are assembled in
/// huge matrices, but there's no need to really
/// build such matrices, in order to exploit sparsity).
///
///  Note: in sake of highest generalization, this base
/// class does NOT include a mass submatrix (a sub part of the M
/// matrix) but just declares methods such as Compute_invMb_v(),
/// (which are used by iterative solvers) which MUST
/// be implemented by child classes. This doing, some child
/// classes too may implement all three methods without needing to
/// store entire mass submatrices, if possible, in sake of efficiency.

class ChApi ChVariables {
    CH_RTTI_ROOT(ChVariables)
  private:
    ChMatrix<>* qb;  ///< variables (accelerations, speeds, etc. depending on the problem)
    ChMatrix<>* fb;  ///< known vector (forces, or impulses, etc. depending on the problem)
    int ndof;        ///< number of degrees of freedom (number of contained scalar variables)
    bool disabled;   ///< user activation/deactivation of variables

  protected:
    int offset;  ///< offset in global q state vector (needed by some solvers)

  public:
    ChVariables() : disabled(false), ndof(0), qb(NULL), fb(NULL), offset(0) {}
    ChVariables(int m_ndof);
    virtual ~ChVariables();

    /// Assignment operator: copy from other object
    ChVariables& operator=(const ChVariables& other);

    /// Deactivates/freezes the variable (these 'frozen',
    /// variables won't be modified by the system solver).
    void SetDisabled(bool mdis) { disabled = mdis; }

    /// Tells if the variables have been deactivated (these 'frozen',
    /// variables won't be modified by the system solver).
    bool IsDisabled() const { return disabled; }

    /// Tells if these variables are currently active, in general,
    /// that is tells if they must be included into the system solver or not.
    bool IsActive() const { return !disabled; }

    /// The number of scalar variables in the vector qb
    /// (dof=degrees of freedom)
    /// *** This function MUST BE OVERRIDDEN by specialized
    /// inherited classes.
    virtual int Get_ndof() const { return ndof; }

    /// Returns reference to qb, body-relative part of degrees
    /// of freedom q in system:
    ///    | M -Cq'|*|q|- | f|= |0| ,  c>0, l>0, l*r=0;
    ///    | Cq  0 | |l|  |-b|  |c|
    ChMatrix<>& Get_qb() { return *qb; }

    /// Compute fb, body-relative part of known
    /// vector f in system.
    /// *** This function MAY BE OVERRIDDEN by specialized
    /// inherited classes (example, for impulsive multibody simulation,
    /// this may be fb=dt*Forces+[M]*previous_v ).
    ///  Another option is to set values into fb vectors, accessing
    /// them by Get_fb() from an external procedure, for each body,
    /// before starting the solver.
    virtual void Compute_fb() {}

    /// Returns reference to fb, body-relative part of known
    /// vector f in system.
    ///    | M -Cq'|*|q|- | f|= |0| ,  c>0, l>0, l*r=0;
    ///    | Cq  0 | |l|  |-b|  |c|
    /// This function can be used to set values of fb vector
    /// before starting the solver.
    ChMatrix<>& Get_fb() { return *fb; }

    /// Computes the product of the inverse mass matrix by a
    /// vector, and store in result: result = [invMb]*vect
    /// *** This function MUST BE OVERRIDDEN by specialized
    /// inherited classes
    virtual void Compute_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const = 0;

    /// Computes the product of the inverse mass matrix by a
    /// vector, and increment result: result += [invMb]*vect
    /// *** This function MUST BE OVERRIDDEN by specialized
    /// inherited classes
    virtual void Compute_inc_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const = 0;

    /// Computes the product of the mass matrix by a
    /// vector, and increment result: result = [Mb]*vect
    /// *** This function MUST BE OVERRIDDEN by specialized
    /// inherited classes
    virtual void Compute_inc_Mb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const = 0;

    /// Computes the product of the corresponding block in the
    /// system matrix (ie. the mass matrix) by 'vect', scale by c_a, and add to 'result'.
    /// NOTE: the 'vect' and 'result' vectors must already have
    /// the size of the total variables&constraints in the system; the procedure
    /// will use the ChVariable offset (that must be already updated) to know the
    /// indexes in result and vect.
    virtual void MultiplyAndAdd(ChMatrix<double>& result, const ChMatrix<double>& vect, const double c_a) const = 0;

    /// Add the diagonal of the mass matrix scaled by c_a, to 'result', as a vector.
    /// NOTE: the 'result' vector must already have the size of system unknowns, ie
    /// the size of the total variables&constraints in the system; the procedure
    /// will use the ChVariable offset (that must be already updated) as index.
    virtual void DiagonalAdd(ChMatrix<double>& result, const double c_a) const = 0;

    /// Build the mass submatrix (for these variables) multiplied by c_a, storing
    /// it in 'storage' sparse matrix, at given column/row offset.
    /// Most iterative solvers don't need to know this matrix explicitly.
    /// *** This function MUST BE OVERRIDDEN by specialized
    /// inherited classes
    virtual void Build_M(ChSparseMatrix& storage, int insrow, int inscol, const double c_a) = 0;

    /// Set offset in global q vector (set automatically by ChSystemDescriptor)
    void SetOffset(int moff) { offset = moff; }
    /// Get offset in global q vector
    int GetOffset() const { return offset; }

    virtual void ArchiveOUT(ChArchiveOut& marchive) {}
    virtual void ArchiveIN(ChArchiveIn& marchive) {}
};

}  // end namespace chrono

#endif