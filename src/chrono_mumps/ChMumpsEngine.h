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
// Authors: Dario Mangoni, Radu Serban
// =============================================================================

#ifndef CHMUMPSENGINE_H
#define CHMUMPSENGINE_H

#include "chrono/core/ChMatrix.h"
#include "chrono_mumps/ChApiMumps.h"

#include "dmumps_c.h"
#define USE_COMM_WORLD -987654

/* macro s.t. indices match documentation */
#define ICNTL(I) icntl[(I)-1]
#define CNTL(I) cntl[(I)-1]
#define INFO(I) info[(I)-1]
#define INFOG(I) infog[(I)-1]
#define RINFO(I) rinfo[(I)-1]
#define RINFOG(I) rinfog[(I)-1]

namespace chrono {

/// @addtogroup mumps_module
/// @{

/// Wrapper class for the MUMPS direct linear solver.
/// This solver is not appropriate for VI and complementarity problems.
class ChApiMumps ChMumpsEngine {
  public:
    enum mumps_SYM { UNSYMMETRIC = 0, SYMMETRIC_POSDEF = 1, SYMMETRIC_GENERAL = 2 };

    enum mumps_JOB {
        INIT = -1,
        END = -2,
        ANALYZE = 1,
        FACTORIZE = 2,
        SOLVE = 3,
        ANALYZE_FACTORIZE = 4,
        FACTORIZE_SOLVE = 5,
        COMPLETE = 6
    };

    ChMumpsEngine();
    ~ChMumpsEngine();

    /// Set the number of OpenMP threads for MUMPS.
    void SetNumThreads(int num_threads);

    /// Set the problem matrix and the right-hand side.
    void SetProblem(const ChSparseMatrix& Z, ChVectorRef rhs);

    /// Set the problem matrix.
    void SetMatrix(const ChSparseMatrix& Z);

    /// Informs MUMPS of the matrix symmetry type.
    void SetMatrixSymmetry(mumps_SYM symmetry);

    /// Set the right-hand side vector.
    /// Note that it is the caller's responsibility to ensure that the size is appropriate.
    void SetRhsVector(ChVectorRef b);
    void SetRhsVector(double* b);

    /// Enable null-pivot detection in MUMPS.
    void EnableNullPivotDetection(bool val, double threshold = 0);

    /// Submit job to MUMPS.
    int MumpsCall(mumps_JOB job_call);

    /// Print a detailed description of the current INFOG array of MUMPS.
    void PrintINFOG();

    /// Get the <tt>parnum</tt>th CoNTroL parameter (CNTL)
    double GetCNTL(int parnum) { return mumps_id.CNTL(parnum); }
    /// Set the <tt>parnum</tt>th CoNTroL parameter (CNTL)
    void SetCNTL(int parnum, int parvalue) { mumps_id.CNTL(parnum) = parvalue; }

    /// Get the <tt>parnum</tt>th Integer CoNTroL parameter (ICNTL)
    int GetICNTL(int parnum) { return mumps_id.ICNTL(parnum); }
    /// Set the <tt>parnum</tt>th Integer CoNTroL parameter (ICNTL)
    void SetICNTL(int parnum, int parvalue) { mumps_id.ICNTL(parnum) = parvalue; }

    /// Get the <tt>parnum</tt>th INFOrmative flag (INFO)
    int GetINFO(int parnum) { return mumps_id.INFO(parnum); }
    /// Get the <tt>parnum</tt>th Global INFOrmative flag (INFOG)
    int GetINFOG(int parnum) { return mumps_id.INFOG(parnum); }

    /// Get the <tt>parnum</tt>th Real-valued INFOrmative flag (RINFO)
    double GetRINFO(int parnum) { return mumps_id.RINFO(parnum); }
    /// Get the <tt>parnum</tt>th Real-valued INFOrmative flag (RINFOG)
    double GetRINFOG(int parnum) { return mumps_id.RINFOG(parnum); }

    DMUMPS_STRUC_C& GetMumpsStruc() { return mumps_id; }

  private:
    DMUMPS_STRUC_C mumps_id;  ///< Mumps data structure

    std::vector<int> m_irn;   ///< row indices (in COO matrix representation)
    std::vector<int> m_jcn;   ///< column indices (in COO matrix representation)
    std::vector<double> m_a;  ///< nonzero elements (in COO matrix representation)
};

/// @} mumps_module

}  // end of namespace chrono

#endif
