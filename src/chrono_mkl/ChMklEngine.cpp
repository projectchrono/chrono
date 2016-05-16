#include <algorithm>
#include "chrono_mkl/ChMklEngine.h"

namespace chrono {
enum phase_t {
    COMPLETE = 13,
    ANALYSIS = 11,
    ANALYSIS_NUMFACTORIZATION = 12,
    NUMFACTORIZATION = 22,
    NUMFACTORIZATION_SOLVE = 23,
    SOLVE = 33,
    SOLVE_FORWARD = 331,
    SOLVE_DIAGONAL = 332,
    SOLVE_BACKWARD = 333,
    RELEASE_FACTORS = 0,
    RELEASE_ALL = -1
};

ChMklEngine::ChMklEngine(int problem_size, int matrix_type)
    : a(nullptr),
      ia(nullptr),
      ja(nullptr),
      b(nullptr),
      x(nullptr),
      last_phase_called(-1),
      /*Currently only one rhs is supported*/
      nrhs(1),    //< Number of KnownVectors
      maxfct(1),  //< Maximum number of factors with identical sparsity structure that must be kept in memory at the
                  //same time [def: 1]
      mnum(1)     //< Actual matrix for the solution phase (1 ≤ mnum ≤ maxfct) [def: 1]
{
    SetProblemSize(problem_size);
    ResetSolver(matrix_type);
}

ChMklEngine::~ChMklEngine() {
    int phase = RELEASE_ALL;
    int msglvl = 1;
    int error;
    PARDISO(pt, &maxfct, &mnum, &mtype, &phase, &n, a, ia, ja, perm.data(), &nrhs, iparm, &msglvl, b, x, &error);
    if (error)
        printf("Error while releasing memory: %d", error);
}

void ChMklEngine::SetMatrix(double* Z_values, int* Z_colIndex, int* Z_rowIndex) {
    a = Z_values;
    ja = Z_colIndex;
    ia = Z_rowIndex;
}

void ChMklEngine::SetMatrix(ChCSR3Matrix& Z) {
    a = Z.GetValuesAddress();
    ja = Z.GetColIndexAddress();
    ia = Z.GetRowIndexAddress();
    if (Z.GetSymmetry() != mtype)
        ResetSolver(Z.GetSymmetry());
    SetProblemSize(Z.GetRows());
}

void ChMklEngine::SetSolutionVector(ChMatrix<>& insx) {
    x = insx.GetAddress();
}

void ChMklEngine::SetSolutionVector(double* insx) {
    x = insx;
}

void ChMklEngine::SetKnownVector(ChMatrix<>& insf_chrono, ChMatrix<>& insb_chrono, ChMatrix<>& bdest) {
    // assures that the destination vector has the correct dimension
    if (insb_chrono.GetRows() + insf_chrono.GetRows() != bdest.GetRows())
        bdest.Resize((insb_chrono.GetRows() + insf_chrono.GetRows()), 1);

    // pastes values of insf and insb in fdest
    for (int i = 0; i < insf_chrono.GetRows(); i++)
        bdest.SetElement(i, 0, insf_chrono.GetElement(i, 0));
    for (int i = 0; i < insb_chrono.GetRows(); i++)
        bdest.SetElement(i + insf_chrono.GetRows(), 0, insb_chrono.GetElement(i, 0));

    // takes the fdest as known term of the problem
    SetKnownVector(bdest);
}

void ChMklEngine::SetProblem(ChCSR3Matrix& Z, ChMatrix<>& insb, ChMatrix<>& insx) {
    SetMatrix(Z);
    SetSolutionVector(insx);
    SetKnownVector(insb);
}

/// Tells the solver to store the permutation vector \c perm and use it in the next calls.
void ChMklEngine::UsePermutationVector(bool on_off) {
    // The perm array is not set yet; it is built by Pardiso during the next factorization phase.
    // iparm[4]=2 (perm as output) says to Pardiso to output the perm vector used by the factorization;
    // PardisoCall() will then switch iparm[4] to 1 (perm as input)
    iparm[4] = (on_off) ? 2 : 0;

    if (on_off == true) {
        resetIparmElement(30);
        resetIparmElement(35);

        perm.resize(n);
    }
}

/// Warns if a incompatible parameter has been previously set
void ChMklEngine::resetIparmElement(int iparm_num, int reset_value) {
    if (iparm[iparm_num] != reset_value) {
        iparm[iparm_num] = reset_value;

        switch (iparm_num) {
        case 3:
            printf("Preconditioned CGS has been disabled. iparm[3] = 0"); break;
        case 4:
            printf("Permutation vector has been disabled.iparm[4] = 0"); break;
        case 7:
            printf("Iterative refinement steps has been disabled. iparm[7] = 0"); break;
        case 30:
            printf("Partial solution has been disabled. iparm[30] = 0"); break;
        case 35:
            printf("Schur computation has been disabled. iparm[35] = 0"); break;
        case 59:
            printf("Mkl is now running in-core. iparm[59] = 0"); break;
        default:
            printf("WARN: IparmReset not handled");
        }
    }
}

void ChMklEngine::UsePartialSolution(int option, int start_row, int end_row) {
    assert(option == 0 || option == 1 || option == 2 || option == 3);

    iparm[30] = option;

    if (option) {

        resetIparmElement(3);
        resetIparmElement(4);
        resetIparmElement(7);
        resetIparmElement(35);
        resetIparmElement(59);

        perm.resize(n);

        if (option == 1 || option == 2) {
            for (int row_sel = 0; row_sel < n; row_sel++)
                perm[row_sel] = (b[row_sel] == 0) ? 0 : 1;
        } else if (option == 3) {
            for (int row_sel = 0; row_sel < n; row_sel++)
                perm[row_sel] = (row_sel < start_row || row_sel > end_row) ? 0 : 1;
        }
    }
}

/// The Schur complement is output on the solution vector \c x that has to be resized to \c n x \c n size;
/// The next call to Pardiso must not involve a solution phase. So no phase 33, 331, 332, 333, 23, 13 ecc...
/// Any solution phase in fact would ouput the solution on the solution vector \c x.
/// The element (\param[start_row],\param[start_row] must be the top-left element of the matrix on which the Schur
/// complement will be computed;
/// The element (\param[end_row],\param[end_row] must be the bottom-right element of the matrix on which the Schur
/// complement will be computed;
void ChMklEngine::OutputSchurComplement(int option, int start_row, int end_row) {
    iparm[35] = option;

    if (option) {
        resetIparmElement(4);
        resetIparmElement(30);

        perm.resize(n);

        assert(!(start_row == 0) && (end_row == 0));

        if (end_row == 0)
            end_row = n - 1;

        for (int row_sel = 0; row_sel < n; row_sel++)
            perm[row_sel] = (row_sel < start_row || row_sel > end_row) ? 0 : 1;
    }
}

void ChMklEngine::SetPreconditionedCGS(bool on_off, int L) {
    if (on_off) {
        int K = (mtype == 11 || mtype == 1) ? 1 : 2;
        iparm[3] = 10 * L + K;
    } else
        iparm[3] = 0;
}

int ChMklEngine::PardisoCall(int set_phase, int message_level) {
    int error;
    last_phase_called = set_phase;
    int phase_now = set_phase;
    PARDISO(pt, &maxfct, &mnum, &mtype, &phase_now, &n, a, ia, ja, perm.data(), &nrhs, iparm, &message_level, b, x,
            &error);

    if (iparm[4] == 2)
        iparm[4] = 1;

    return error;
}

void ChMklEngine::ResetSolver(int new_mat_type) {
    // After the first call to pardiso do not directly modify "pt", as that could cause a serious memory leak.
    if (new_mat_type)
        mtype = new_mat_type;
    pardisoinit(pt, &mtype, iparm);

    /*
    * NOTE: for highly indefinite symmetric matrices (e.g. interior point optimizations or saddle point problems)
    * use iparm[10] = 1 (scaling) and iparm[12] = 1 (matchings);
    */

    /* IPARM easy settings */
    iparm[0] = 1;                /* No default values for solver */
    iparm[5] = 0;                /* Write solution on u */
    iparm[11] = 0;                /* Solve with transposed/conjugate transposed matrix [def: 0, solve simply A*x=b]*/
    iparm[17] = -1;                /* Report number of nonzeros */
    iparm[18] = -1;                /* Report number of floating point operations */
    iparm[34] = 1;                /* Zero based indexing */
    iparm[26] = 0;                /* Matrix checker */
    iparm[27] = 0;                /* Double precision */
    iparm[35] = 0;                /* Schur complement matrix computation control [def:0, do not compute Schur] */
    iparm[55] = 0;                /* Diagonal and pivoting control [def:0, disabled] */
    iparm[59] = 0;                /* In-Core (OC) / Out-Of-Core (OOC) switch [def:0, IC mode] */


    /* IPARM fine settings */
    iparm[1] = 2;                /* Fill-in reducing ordering [def:2] */
    iparm[3] = 0;                /* Preconditioned CGS/CG [def:0] - HIGHLY RECOMMENDED */
    iparm[4] = 0;                /* User fill-in reducing permutation [def:0, default filling]*/
    iparm[7] = 10;                /* Maximum number of iterative refinement steps */

}

void ChMklEngine::GetResidual(double* res) const {
    mkl_cspblas_dcsrgemv("N", &n, a, ia, ja, x, res);  // performs Matrix*Solution
    for (int i = 0; i < n; i++) {
        res[i] = b[i] - res[i];  // performs: rhs - Matrix*Solution
    };
};

double ChMklEngine::GetResidualNorm(const double* res) const {
    double norm = 0;
    for (int i = 0; i < n; i++) {
        norm += res[i] * res[i];
    };
    norm = sqrt(norm);
    return norm;
};

void ChMklEngine::PrintIparmOutput() const {
    printf("\n[6] Number of iterative refinement steps performed: %d", iparm[6]);
    if (mtype == 11 || mtype == 13 || mtype == -2 || mtype == -4 || mtype == -6)
        printf("\n[13] Number of perturbed pivots: %d", iparm[13]);
    if (last_phase_called == 11 || last_phase_called == 12 || last_phase_called == 13) {
        printf("\n[14] Peak memory on symbolic factorization (kB): %d", iparm[14]);
        printf("\n[15] Permanent memory on symbolic factorization (kB): %d", iparm[15]);
        printf("\n[16] Peak memory on numerical factorization and solution (kB): %d", iparm[16]);
        printf("\nTotal peak memory consumed (kB): %d", std::max(iparm[14], iparm[15] + iparm[16]));
    }

    printf("\n[17] Number of non-zero elements in the factors: %d", iparm[17]);
    printf("\n[18] Number of floating point operations necessary to factor the matrix (^6): %d", iparm[18]);
    printf("\n[19] Number of completed CG/CGS iterations: %d", iparm[19]);
    if (mtype == -2) {
        printf("\n[21] Number of positive eigenvalues: %d", iparm[21]);
        printf("\n[22] Number of negative eigenvalues: %d\n", iparm[22]);
    }
    if (mtype == 2 || mtype == 4)
        printf("\n[29] Number of zero or negative pivots: %d", iparm[29]);
}

}  // end namespace chrono
