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

#ifndef CHVARIABLES_H
#define CHVARIABLES_H

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMatrixDynamic.h"
#include "chrono/core/ChSparseMatrix.h"

namespace chrono {

/// Base class for representing objects that introduce 'variables' (also referred as 'v') and their associated mass
/// submatrices for a sparse representation of the problem.
///
/// See ChSystemDescriptor for more information about the overall problem and data representation.
///
/// Each ChVariables object must be able to compute a mass submatrix, that will be assembled directly inside the global
/// matrix Z, (in particular inside the block H or M). Because of this there is no need for ChVariables and derived
/// classes to actually \e store their mass submatrix in memory: they just need to be able to compute it!
///
/// Moreover, in some cases, the mass submatrix is not even needed. In fact, some Chrono solvers are matrix-free. This
/// means that each ChVariables (and derived) object can be asked not to \e assemble its mass submatrix, but instead to
/// provide operation related to it. E.g. M*x, M\\x, +=M*x, and so on... Each derived class must implement these
/// methods!
///
/// Because of this, the ChVariables class does \e not include any mass submatrix by default

class ChApi ChVariables {

  private:
    ChMatrix<>* qb;  ///< variables (accelerations, speeds, etc. depending on the problem)
    ChMatrix<>* fb;  ///< known vector (forces, or impulses, etc. depending on the problem)
    int ndof;        ///< number of degrees of freedom (number of contained scalar variables)
    bool disabled;   ///< user activation/deactivation of variables

  protected:
    int offset;  ///< offset in global q state vector (needed by some solvers)

  public:
    ChVariables();
    ChVariables(int m_ndof);
    virtual ~ChVariables();

    /// Assignment operator: copy from other object
    ChVariables& operator=(const ChVariables& other);

    /// Deactivates/freezes the variable (these variables won't be modified by the system solver).
    void SetDisabled(bool mdis) { disabled = mdis; }

    /// Check if the variables have been deactivated (these variables won't be modified by the system solver).
    bool IsDisabled() const { return disabled; }

    /// Check if these variables are currently active.
    /// In general, tells if they must be included into the system solver or not.
    bool IsActive() const { return !disabled; }

    /// The number of scalar variables in the vector qb (dof=degrees of freedom)
    /// *** This function MUST BE OVERRIDDEN by specialized inherited classes.
    virtual int Get_ndof() const { return ndof; }

    /// Returns reference to qb, body-relative part of degrees of freedom q in system:
    /// <pre>
    ///    | M -Cq'|*|q|- | f|= |0| ,  c>0, l>0, l*r=0;
    ///    | Cq  0 | |l|  |-b|  |c|
    /// </pre>
    ChMatrix<>& Get_qb() { return *qb; }

    /// Compute fb, body-relative part of known vector f in system.
    /// *** This function MAY BE OVERRIDDEN by specialized inherited classes (e.g., for impulsive multibody simulation,
    /// this may be fb = dt*Forces+[M]*previous_v ).
    /// Another option is to set values into fb vectors, accessing them by Get_fb() from an external procedure,
    /// for each body, before starting the solver.
    virtual void Compute_fb() {}

    /// Returns reference to fb, body-relative part of known vector f in system.
    /// <pre>
    ///    | M -Cq'|*|q|- | f|= |0| ,  c>0, l>0, l*r=0;
    ///    | Cq  0 | |l|  |-b|  |c|
    /// </pre>
    /// This function can be used to set values of fb vector before starting the solver.
    ChMatrix<>& Get_fb() { return *fb; }

    /// Computes the product of the inverse mass matrix by a vector, and store in result: result = [invMb]*vect
    virtual void Compute_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const = 0;

    /// Computes the product of the inverse mass matrix by a vector, and increment result: result += [invMb]*vect
    virtual void Compute_inc_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const = 0;

    /// Computes the product of the mass matrix by a vector, and increment result: result = [Mb]*vect
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