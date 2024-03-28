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
#include "chrono/core/ChMatrix.h"

namespace chrono {

/// Base class for representing objects that introduce 'variables' and their associated mass submatrices.
/// Used for a sparse, distributed representation of the problem.
/// See ChSystemDescriptor for more information about the overall problem and data representation.
///
/// Each ChVariables object must be able to define a mass submatrix, that will be assembled directly inside the global
/// matrix Z (in particular, inside the block H or M). Because of this there is no need for ChVariables and derived
/// classes to actually \e store their mass submatrix in memory: they just need to be able to compute with it.
///
/// Furthermore, the system-level mass matrix is not always formed explicitly. If a matrix-free solver is used, each
/// ChVariables (and derived) object can be asked not to \e assemble its mass submatrix, but instead to provide
/// operation related to it, Ee.g. M*x, M\\x, +=M*x, and so on. Each derived class must implement these methods.
/// Because of this, the ChVariables class does \e not include any mass submatrix by default.
class ChApi ChVariables {
  public:
    ChVariables();
    ChVariables(unsigned int dof);
    virtual ~ChVariables() {}

    /// Assignment operator: copy from other object
    ChVariables& operator=(const ChVariables& other);

    /// Deactivates/freezes the variable (these variables won't be modified by the system solver).
    void SetDisabled(bool mdis) { disabled = mdis; }

    /// Check if the variables have been deactivated (these variables won't be modified by the system solver).
    bool IsDisabled() const { return disabled; }

    /// Check if these variables are currently active.
    /// In general, tells if they must be included into the system solver or not.
    bool IsActive() const { return !disabled; }

    /// The number of scalar variables in the vector qb (dof=degrees of freedom).
    unsigned int GetDOF() const { return ndof; }

    /// Get a reference to the differential states encapsulated by these variables.
    /// These variable states are part of 'q' in the system:
    /// <pre>
    ///    | M -Cq'| * |q|- | f| = |0| ,  c>0, l>0, l*r=0;
    ///    | Cq  0 |   |l|  |-b|   |c|
    /// </pre>
    ChVectorRef State() { return qb; }

    /// Get a reference to the generalized force corresponding to these variables.
    /// These variable forces are part of 'f' in the system:
    /// <pre>
    ///    | M -Cq'| * |q|- | f| = |0| ,  c>0, l>0, l*r=0;
    ///    | Cq  0 |   |l|  |-b|   |c|
    /// </pre>
    ChVectorRef Force() { return fb; }

    /// Compute the product of the inverse mass matrix by a given vector and store in result.
    /// This function must calculate `result = M^(-1) * vect` for a vector of same size as the variables state.
    virtual void ComputeMassInverseTimesVector(ChVectorRef result, ChVectorConstRef vect) const = 0;

    /// Compute the product of the mass matrix by a given vector and increment result.
    /// This function must perform the operation `result += M * vect` for a vector of same size as the variables state.
    virtual void AddMassTimesVector(ChVectorRef result, ChVectorConstRef vect) const = 0;

    /// Add the product of the mass submatrix by a given vector, scaled by ca, to result.
    /// Note: 'result' and 'vect' are system-level vectors of appropriate size. This function must index into these
    /// vectors using the offsets of each variable.
    virtual void AddMassTimesVectorInto(ChVectorRef result, ChVectorConstRef vect, const double ca) const = 0;

    /// Add the diagonal of the mass matrix, as a vector scaled by ca, to result.
    /// Note: 'result' is a system-level vector of appropriate size. This function must index into this vector using the
    /// offsets of each variable.
    virtual void AddMassDiagonalInto(ChVectorRef result, const double ca) const = 0;

    /// Write the mass submatrix for these variables into the specified global matrix at the offsets of each variable.
    /// The masses will be scaled by the given factor 'ca'. The (start_row, start_col) pair specifies the top-left
    /// corner of the system-level mass matrix in the provided matrix. Assembling the system-level sparse matrix
    /// is required only if using a direct sparse solver or for debugging/reporting purposes.
    virtual void PasteMassInto(ChSparseMatrix& mat,
                               unsigned int start_row,
                               unsigned int start_col,
                               const double ca) const = 0;

    /// Set offset in the global state vector.
    /// This offset if set automatically by the ChSystemDescriptor during set up.
    void SetOffset(unsigned int moff) { offset = moff; }

    /// Get offset in the global state vector.
    unsigned int GetOffset() const { return offset; }

    virtual void ArchiveOut(ChArchiveOut& archive_out);
    virtual void ArchiveIn(ChArchiveIn& archive_in);

  protected:
    unsigned int offset;  ///< offset in global q state vector (needed by some solvers)
    unsigned int ndof;    ///< number of degrees of freedom (number of contained scalar variables)

  private:
    ChVectorDynamic<double> qb;  ///< state variables (accelerations, speeds, etc. depending on the problem)
    ChVectorDynamic<double> fb;  ///< right-hand side force vector (forces, impulses, etc. depending on the problem)
    bool disabled;               ///< user activation/deactivation of variables
};

}  // end namespace chrono

#endif