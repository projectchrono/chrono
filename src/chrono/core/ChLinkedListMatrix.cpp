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

#include "chrono/core/ChLinkedListMatrix.h"
#include "chrono/core/ChMath.h"

#define PIVOT_ACCEPT_TRESHOLD 0.8
#define ACCEPT_PIVOT 1e-3
#define MIN_PIVOT 1e-8
#define INFINITE_PIVOT 1.e+34

namespace chrono {

ChLinkedListMatrix::ChLinkedListMatrix(int nrows, int ncols, double fill_in)
    : ChSparseMatrix(nrows, ncols), m_determinant(0) {
    ChLinkedListMatrix::Build(fill_in);
}

ChLinkedListMatrix::ChLinkedListMatrix() : ChSparseMatrix(1, 1), m_determinant(0) {
    ChLinkedListMatrix::Build(1.0);
}

ChLinkedListMatrix::ChLinkedListMatrix(const ChMatrix<>& mat) {
    Reset(mat.GetRows(), mat.GetColumns());

    // fill matrix
    for (int r = 0; r < m_num_rows; r++) {
        ChMelement* mguess = *(elarray + r);
        for (int c = 0; c < m_num_cols; c++) {
            double el = mat.GetElement(r, c);
            if (el != 0)
                mguess = SetElement(r, c, el, mguess);
        }
    }
}

ChLinkedListMatrix::ChLinkedListMatrix(const ChLinkedListMatrix& other) : ChSparseMatrix(other) {
    Reset(other.m_num_rows, other.m_num_cols);

    for (int r = 0; r < m_num_rows; r++) {
        ChMelement* eguess = *(elarray + r);

        for (ChMelement* srowel = *(other.elarray + r); srowel != nullptr; srowel = srowel->next) {
            double val = srowel->val;
            if (val)
                eguess = SetElement(r, srowel->col, val, eguess);
        }
    }
}

ChLinkedListMatrix::~ChLinkedListMatrix() {
    mbuffer_size = 0;
    mbuffer_added = 0;

    free(elarray);
    elarray = nullptr;

    for (ChNode<ChMelement>* mnode = mbufferlist.GetHead(); mnode != nullptr; mnode = mnode->next) {
        free(mnode->data);  // deallocate memory buffers
    }

    mbufferlist.RemoveAll();  //  may be avoided, since it will automatically remove at list deletion
    mtot_size = 0;
}

void ChLinkedListMatrix::Build(double fill_in) {
    mtot_size = 0;
    mbuffer_added = 0;
    mnextel = nullptr;

    // alloc buffer of elements
    mbuffer_size = (int)(m_num_rows * m_num_cols * fill_in);
    if (m_num_rows > 5000)
        mbuffer_size = SPM_DEF_MAXELEMENTS;
    if (m_num_cols > 5000)
        mbuffer_size = SPM_DEF_MAXELEMENTS;
    if (mbuffer_size > SPM_DEF_MAXELEMENTS)
        mbuffer_size = SPM_DEF_MAXELEMENTS;
    if (mbuffer_size < m_num_rows * 2)
        mbuffer_size = m_num_rows * 2;

    ChMelement* mbuffer = (ChMelement*)calloc(mbuffer_size, sizeof(ChMelement));

    mtot_size += mbuffer_size;
    mnextel = mbuffer;
    mbufferlist.AddHead(mbuffer);

    // alloc pointers to first column
    elarray = (ChMelement**)calloc(m_num_rows, sizeof(ChMelement*));

    for (int i = 0; i < m_num_rows; i++) {
        *(elarray + i) = mnextel;                    // initialize vector of 1st col pointr pointers
        (mnextel)->Initialize(0, nullptr, nullptr, i, 0);  // initialize 1st col.elements
        mbuffer_added++;                             // increment the counter of "used" elements.
        mnextel++;
    }
}

// Copy this matrix to a dense matrix.
void ChLinkedListMatrix::CopyToMatrix(ChMatrix<>& mat) {
    mat.Reset(m_num_rows, m_num_cols);

    ChMelement* currel;
    for (int i = 0; i < m_num_rows; i++) {
        for (currel = *(elarray + i); currel != nullptr; currel = currel->next) {
            mat.SetElement(currel->row, currel->col, currel->val);
        }
    }
}

void ChLinkedListMatrix::MoreBuffer(double inflate) {
    mbuffer_size = (int)(inflate * (double)mbuffer_size);  // inflate buffer size

    if (m_num_rows < 5000)
        if ((mtot_size + mbuffer_size) > m_num_rows * m_num_cols)
            mbuffer_size = m_num_rows * m_num_cols - mtot_size;

    ChMelement* mbuffer;
    mbuffer = (ChMelement*)calloc(mbuffer_size, sizeof(ChMelement));

    mtot_size += mbuffer_size;
    mnextel = mbuffer;
    mbufferlist.AddTail(mbuffer);  // add another buffer!

    mbuffer_added = 0;
}

bool ChLinkedListMatrix::Resize(int nrows, int ncols, int nonzeros) {
    assert(false);
    return false;
}

void ChLinkedListMatrix::Reset() {
    Reset(m_num_rows, m_num_cols);
}

void ChLinkedListMatrix::Reset(int nrows, int ncols, int nonzeros) {
    // realloc 1st column array, only if needed
    if (nrows != m_num_rows) {
        if (elarray)
            free(elarray);
        elarray = (ChMelement**)calloc(nrows, sizeof(ChMelement*));
    }

    // realloc buffer (but only if needed!!! -see the if() conditions below.. - ) ,
    // eventually compacting list into a _new_ single buffer

    int mneed_buffer_size = 0;

    if (mbufferlist.Count() > 1)
        mneed_buffer_size = mtot_size;  //   sum of all list of buffers, if multiple

    if (mtot_size < nrows * 2)
        mneed_buffer_size = nrows * 2;  // ..at least two columns

    if (nrows < 5000)
        if (mtot_size > (int)(nrows * ncols * 1.2))
            mneed_buffer_size = nrows * ncols;  // ..at most completely dense

    if (mneed_buffer_size)  // ok, a new allocation must be done...
    {
        // manage reallocation of buffer...
        for (ChNode<ChMelement>* mnode = mbufferlist.GetHead(); mnode != nullptr; mnode = mnode->next) {
            free(mnode->data);  // deallocate old memory buffers
        }

        mbufferlist.RemoveAll();
        mtot_size = 0;

        mbuffer_size = mneed_buffer_size;
        ChMelement* mbuffer = (ChMelement*)calloc(mbuffer_size, sizeof(ChMelement));
        mtot_size += mbuffer_size;
        mnextel = mbuffer;
        mbufferlist.AddHead(mbuffer);  // add buffer to empty list

        mbuffer_added = 0;
    }

    m_num_rows = nrows;
    m_num_cols = ncols;

    mbuffer_added = 0;
    mnextel = (mbufferlist.GetHead())->data;  // rewind the elem. pointer

    // Just reinitialize 1st row elements
    for (int i = 0; i < m_num_rows; i++) {
        *(elarray + i) = mnextel;                    // initialize vector of 1st col pointr pointers
        (mnextel)->Initialize(0, nullptr, nullptr, i, 0);  // initialize 1st col.elements
        mbuffer_added++;                             // increment the counter of "used" elements.
        mnextel++;
    }
}

void ChLinkedListMatrix::ResetBlocks(int nrows, int ncols) {
    if ((nrows == m_num_rows) && (ncols == m_num_cols)) {
        // size doesn't change
        static ChMelement* currel;
        for (int i = 0; i < m_num_rows; i++) {
            for (currel = *(elarray + i); currel != nullptr; currel = currel->next) {
                currel->val = 0.0;
            }
        }
    } else {
        // size changes
        Reset(nrows, ncols);
    }
}

// optimized SetElement,  returning the fetched Melement*
ChMelement* ChLinkedListMatrix::SetElement(int row, int col, double val, ChMelement* guess) {
    assert(row >= 0);
    assert(col >= 0);
    assert(row < m_num_rows);
    assert(col < m_num_cols);
    assert(guess->row == row);

    ChMelement* enext;
    ChMelement* eprev;
    ChMelement* newguess;

    while (guess->col != col) {
        // forward search
        if (guess->col < col) {
            enext = guess->next;
            if (enext) {
                if (enext->col <= col)
                    guess = enext;  // .. and repeat
                else                // if (enext->col >  col)
                {
                    newguess = NewElement(val, enext, guess, row, col);
                    guess->next = newguess;
                    enext->prev = newguess;
                    return (newguess);
                }
            } else {
                newguess = NewElement(val, nullptr, guess, row, col);
                guess->next = newguess;
                return (newguess);
            }
        }

        // backward search
        if (guess->col > col) {
            eprev = guess->prev;
            if (eprev) {
                if (eprev->col >= col)
                    guess = eprev;  // .. and repeat
                else                // if (eprev->col <  col)
                {
                    newguess = NewElement(val, guess, eprev, row, col);
                    eprev->next = newguess;
                    guess->prev = newguess;
                    return (newguess);
                }
            } else {
                newguess = NewElement(val, guess, nullptr, row, col);
                guess->prev = newguess;
                return (newguess);
            }
        }

    }  // end while loop

    guess->val = val;
    return (guess);
}

// optimized GetElement,  returning the fetched Melement*
ChMelement* ChLinkedListMatrix::GetElement(int row, int col, double* val, ChMelement* guess) {
    assert(row >= 0);
    assert(col >= 0);
    assert(row < m_num_rows);
    assert(col < m_num_cols);
    assert(guess->row == row);

    ChMelement* enext;
    ChMelement* eprev;

    while (guess->col != col) {
        // forward search
        if (guess->col < col) {
            enext = guess->next;
            if (enext) {
                if (enext->col <= col)
                    guess = enext;  // .. and repeat
                else                // if (enext->col >  col)
                {
                    *val = 0;
                    return (guess);
                }
            } else {
                *val = 0;
                return (guess);
            }
        }

        // backward search
        if (guess->col > col) {
            eprev = guess->prev;
            if (eprev) {
                if (eprev->col >= col)
                    guess = eprev;  // .. and repeat
                else                // if (eprev->col <  col)
                {
                    *val = 0;
                    return (guess);
                }
            } else {
                *val = 0;
                return (guess);
            }
        }

    }  // end while loop

    *val = guess->val;
    return (guess);
}

void ChLinkedListMatrix::SetElement(int row, int col, double elem, bool overwrite) {
    ChMelement* guess;
    guess = *(elarray + row);
    SetElement(row, col, elem, guess);
}

double ChLinkedListMatrix::GetElement(int row, int col) const {
    double retv;
    ChMelement* guess;
    guess = *(elarray + row);
    const_cast<ChLinkedListMatrix*>(this)->GetElement(row, col, &retv, guess);
    return retv;
}

void ChLinkedListMatrix::SwapRows(int a, int b) {
    ChMelement* mtemp;
    mtemp = *(elarray + a);
    *(elarray + a) = *(elarray + b);  // row lists easy switched here...
    *(elarray + b) = mtemp;

    for (mtemp = *(elarray + a); mtemp != nullptr; mtemp = mtemp->next) {
        mtemp->row = a;
    }
    for (mtemp = *(elarray + b); mtemp != nullptr; mtemp = mtemp->next) {
        mtemp->row = b;
    }
}

void ChLinkedListMatrix::SwapColumns(int a, int b) {
    ChMelement* guessA;
    ChMelement* guessB;
    double valA, valB;

    for (int i = 0; i < m_num_rows; i++) {
        guessA = GetElement(i, a, &valA, *(elarray + i));
        guessB = GetElement(i, b, &valB, *(elarray + i));

        SetElement(i, a, valB, guessA);
        SetElement(i, b, valA, guessB);
    }
}

void ChLinkedListMatrix::PasteMatrix(const ChMatrix<>& matra, int insrow, int inscol, bool overwrite, bool transp) {
    int i, j;
    int maxrows = matra.GetRows();
    int maxcol = matra.GetColumns();
    ChMelement* eguess;
    double val;

    for (i = 0; i < maxrows; i++) {
        eguess = *(elarray + i + insrow);

        for (j = 0; j < maxcol; j++) {
            val = (matra.GetElement(i, j));
            if (val)
                eguess = SetElement(i + insrow, j + inscol, val, eguess);
        }
    }
}

void ChLinkedListMatrix::PasteTranspMatrix(const ChMatrix<>& matra, int insrow, int inscol) {
    int i, j;
    int maxrows = matra.GetRows();
    int maxcol = matra.GetColumns();
    ChMelement* eguess;
    double val;

    for (j = 0; j < maxcol; j++) {
        eguess = *(elarray + j + insrow);

        for (i = 0; i < maxrows; i++) {
            val = (matra.GetElement(i, j));
            if (val)
                eguess = SetElement(j + insrow, i + inscol, val, eguess);
        }
    }
}

void ChLinkedListMatrix::PasteSumMatrix(const ChMatrix<>& matra, int insrow, int inscol) {
    int i, j;
    int maxrows = matra.GetRows();
    int maxcol = matra.GetColumns();
    ChMelement* eguess;
    ChMelement* eguessread;
    double val, aval, sum;

    for (i = 0; i < maxrows; i++) {
        eguess = *(elarray + i + insrow);
        eguessread = eguess;

        for (j = 0; j < maxcol; j++) {
            val = (matra.GetElement(i, j));
            if (val) {
                eguessread = GetElement(i + insrow, j + inscol, &aval, eguessread);
                sum = val + aval;
                eguess = SetElement(i + insrow, j + inscol, sum, eguess);
            }
        }
    }
}

void ChLinkedListMatrix::PasteSumTranspMatrix(const ChMatrix<>& matra, int insrow, int inscol) {
    int i, j;
    int maxrows = matra.GetRows();
    int maxcol = matra.GetColumns();
    ChMelement* eguess;
    ChMelement* eguessread;
    double val, aval, sum;

    for (j = 0; j < maxcol; j++) {
        eguess = *(elarray + j + insrow);
        eguessread = eguess;

        for (i = 0; i < maxrows; i++) {
            val = (matra.GetElement(i, j));
            if (val) {
                eguessread = GetElement(j + insrow, i + inscol, &aval, eguessread);
                sum = val + aval;
                eguess = SetElement(j + insrow, i + inscol, sum, eguess);
            }
        }
    }
}

void ChLinkedListMatrix::PasteMatrix(const ChLinkedListMatrix& matra, int insrow, int inscol) {
    ChMelement* eguess;
    ChMelement* srowel;
    double val;

    for (auto i = 0; i < matra.m_num_rows; i++) {
        eguess = *(elarray + i + insrow);

        for (srowel = *(matra.elarray + i); srowel != nullptr; srowel = srowel->next) {
            val = srowel->val;
            if (val)
                eguess = SetElement(i + insrow, srowel->col + inscol, val, eguess);
        }
    }
}

void ChLinkedListMatrix::PasteTranspMatrix(const ChLinkedListMatrix& matra, int insrow, int inscol) {
    ChMelement* eguess;
    double val;
    // note: could be optimized for wide matra matrices..
    for (auto j = 0; j < matra.m_num_cols; j++) {
        eguess = *(elarray + j + insrow);

        for (auto i = 0; i < matra.m_num_rows; i++) {
            val = (matra.GetElement(i, j));
            if (val)
                eguess = SetElement(j + insrow, i + inscol, val, eguess);
        }
    }
}

void ChLinkedListMatrix::PasteClippedMatrix(const ChMatrix<>& matra,
                                            int cliprow,
                                            int clipcol,
                                            int nrows,
                                            int ncolumns,
                                            int insrow,
                                            int inscol,
                                            bool overwrite) {
    ChMelement* eguess;
    double val;

    for (auto i = 0; i < nrows; i++) {
        eguess = *(elarray + i + insrow);

        for (auto j = 0; j < ncolumns; j++) {
            val = (matra.GetElement(i + cliprow, j + clipcol));
            if (val)
                eguess = SetElement(i + insrow, j + inscol, val, eguess);
        }
    }
}

void ChLinkedListMatrix::PasteSumClippedMatrix(const ChMatrix<>& matra,
                                               int cliprow,
                                               int clipcol,
                                               int nrows,
                                               int ncolumns,
                                               int insrow,
                                               int inscol) {
    ChMelement* eguess;
    ChMelement* eguessread;
    double val, aval, sum;

    for (auto i = 0; i < nrows; i++) {
        eguess = *(elarray + i + insrow);
        eguessread = eguess;

        for (auto j = 0; j < ncolumns; j++) {
            val = (matra.GetElement(i + cliprow, j + clipcol));
            if (val) {
                eguessread = GetElement(i + insrow, j + inscol, &aval, eguessread);
                sum = val + aval;
                eguess = SetElement(i + insrow, j + inscol, sum, eguess);
            }
        }
    }
}

void ChLinkedListMatrix::MatrScale(double factor) {
    ChMelement* currel;

    for (int i = 0; i < m_num_rows; i++) {
        for (currel = *(elarray + i); currel != nullptr; currel = currel->next) {
            currel->val = currel->val * factor;
        }
    }
}

void ChLinkedListMatrix::Neg() {
    ChMelement* currel;

    for (int i = 0; i < m_num_rows; i++) {
        for (currel = *(elarray + i); currel != nullptr; currel = currel->next) {
            currel->val = -(currel->val);
        }
    }
}

// -----------------------------------------------------------------------------
// Solution of general linear systems using LU factorization
// -----------------------------------------------------------------------------

// LU factorization
int ChLinkedListMatrix::Setup_LU() {
    assert(m_num_rows == m_num_cols);

    ChMelement* rowel;
    ChMelement* subrowel;
    int i, k, pivrow, eqpivoted;
    double r, pivot, mval, subval, newval, leader;
    int err = 0;

    // initialize pivot index array
    m_pindices.resize(m_num_rows);
    for (int ind = 0; ind < m_num_rows; ind++) {
        m_pindices[ind] = ind;
    }

    m_determinant = 1;

    for (k = 1; k < m_num_rows; k++) {
        pivot = GetElement(k - 1, k - 1);

        if (fabs(pivot) < ACCEPT_PIVOT) {
            // pivoting is needed, so swap equations
            pivrow = BestPivotRow(k - 1);
            SwapRows(k - 1, pivrow);
            m_determinant *= -1;

            eqpivoted = m_pindices[pivrow];  // swap eq.ids in pivot array
            m_pindices[pivrow] = m_pindices[k - 1];
            m_pindices[k - 1] = eqpivoted;

            pivot = GetElement(k - 1, k - 1);  // fetch new pivot
        }

        // was unable to find better pivot: force solution to zero.and go ahead
        if (fabs(pivot) <= MIN_PIVOT) {
            pivot = INFINITE_PIVOT;
            SetElement(k - 1, k - 1, pivot);
            if (!err) {
                m_determinant = 0;
                err = (1 + m_num_rows - k);  // report deficiency
            }
        } else {
            m_determinant *= pivot;
        }

        for (i = k; i < m_num_rows; i++) {
            leader = GetElement(i, k - 1);

            if (leader) {
                r = leader / pivot;       // compute multiplier
                SetElement(i, k - 1, r);  // store the multiplier in L part!!!

                subrowel = GetElarrayMel(i);   // initialize guessed sparse elements
                rowel = GetElarrayMel(k - 1);  // for speed optimization. They are in two rows.

                // for (j=k; j < m_num_cols; j++)  where j = rowel->col
                for (; rowel != nullptr; rowel = rowel->next) {
                    if (rowel->col >= k) {
                        mval = rowel->val;
                        subrowel = GetElement(i, rowel->col, &subval, subrowel);
                        newval = subval - r * mval;
                        subrowel = SetElement(i, rowel->col, newval, subrowel);  // set the U part
                    }
                }
            }
        }
    }

    pivot = GetElement((m_num_rows - 1), (m_num_rows - 1));
    if (fabs(pivot) <= MIN_PIVOT) {
        pivot = INFINITE_PIVOT;
        SetElement(m_num_rows - 1, m_num_rows - 1, pivot);
        if (!err) {
            m_determinant = 0;
            err = 1;
        }
    } else {
        m_determinant *= pivot;
    }

    return err;
}

// Substitution using existing LU factorization
void ChLinkedListMatrix::Solve_LU(const ChMatrix<>& b, ChMatrix<>& x) {
    assert(m_num_rows == b.GetRows());
    assert(m_num_cols == x.GetRows());

    // BACKWARD substitution - L
    double xlast = b.GetElement(m_pindices[0], 0);
    x.SetElement(0, 0, xlast);

    for (int k = 1; k < m_num_rows; k++) {
        double sum = 0;

        ChMelement* rowel = GetElarrayMel(k);

        for (; (rowel != nullptr && rowel->col < k); rowel = rowel->next) {
            sum += rowel->val * (x.GetElement(rowel->col, 0));
        }

        double val = (b.GetElement(m_pindices[k], 0) - sum);
        x.SetElement(k, 0, val);
    }

    // BACKWARD substitution - U
    xlast = x.GetElement(m_num_rows - 1, 0) / GetElement(m_num_rows - 1, m_num_rows - 1);
    x.SetElement(m_num_rows - 1, 0, xlast);

    for (int k = m_num_rows - 2; k >= 0; k--) {
        double sum = 0;

        ChMelement* rowel = GetElarrayMel(k);

        for (; rowel != nullptr; rowel = rowel->next) {
            if (rowel->col >= k + 1) {
                sum += rowel->val * (x.GetElement(rowel->col, 0));
            }
        }

        double val = (x.GetElement(k, 0) - sum) / GetElement(k, k);
        x.SetElement(k, 0, val);
    }
}

// LU Factorization + substitution
int ChLinkedListMatrix::SolveGeneral(const ChMatrix<>& b, ChMatrix<>& x) {
    int err = Setup_LU();
    Solve_LU(b, x);
    return err;
}

// Support function for LU factorization
int ChLinkedListMatrix::BestPivotRow(int current) {
    double temp = 0;
    int pivotrow = current;

    for (int i = current; i < m_num_rows; i++) {
        double val = fabs(GetElement(i, current));
        if (val > temp) {
            temp = val;
            pivotrow = i;
        }
        if (temp >= PIVOT_ACCEPT_TRESHOLD)
            break;
    }
    return pivotrow;
}

// -----------------------------------------------------------------------------
// Solution of symmetric linear systems, using LDL factorization
// -----------------------------------------------------------------------------

// LDL decomposition (only upper triangular part of A is used)
int ChLinkedListMatrix::Setup_LDL() {
    assert(m_num_rows == m_num_cols);

    ChMelement* rowel;
    ChMelement* subrowel;
    int i, k, pivrow, eqpivoted;
    double r, leader, pivot, mval, subval, newval;
    int err = 0;

    // initialize pivot index array
    m_pindices.resize(m_num_rows);
    for (int ind = 0; ind < m_num_rows; ind++) {
        m_pindices[ind] = ind;
    }

    m_determinant = 1;

    for (k = 1; k < m_num_rows; k++) {
        pivot = GetElement(k - 1, k - 1);

        if (fabs(pivot) < ACCEPT_PIVOT) {
            // pivoting is needed, so ...
            pivrow = BestPivotDiag(k - 1);
            DiagPivotSymmetric(k - 1, pivrow);  // swap both column and row (only upper right!)
            m_determinant *= -1;

            // swap diagonal pivot indexes
            eqpivoted = m_pindices[pivrow];
            m_pindices[pivrow] = m_pindices[k - 1];
            m_pindices[k - 1] = eqpivoted;

            pivot = GetElement((k - 1), (k - 1));
        }

        // was unable to find better pivot: force solution to zero.and go ahead
        if (fabs(pivot) <= MIN_PIVOT) {
            pivot = INFINITE_PIVOT;
            SetElement(k - 1, k - 1, pivot);
            if (!err) {
                m_determinant = 0;
                err = (1 + m_num_rows - k);
            }
        } else {
            m_determinant *= pivot;
        }

        for (i = k; i < m_num_rows; i++) {
            leader = GetElement(k - 1, i);

            if (leader) {
                r = (leader / pivot);  // compute multiplier (mirror look A)

                subrowel = GetElarrayMel(i);   // initialize guessed sparse elements
                rowel = GetElarrayMel(k - 1);  // for speed optimization. They are in two rows.

                // advance top row element till it has (col >= k)

                // for (j=i; j < m_num_cols; j++)     only upper right part
                for (; rowel != nullptr; rowel = rowel->next) {
                    // only upper right part is filled!
                    if (rowel->col >= i) {
                        mval = rowel->val;
                        subrowel = GetElement(i, rowel->col, &subval, subrowel);
                        newval = subval - r * mval;
                        subrowel = SetElement(i, rowel->col, newval, subrowel);
                    }
                }

                SetElement((k - 1), i, r);  // now can store the multiplier in L part (mirror look! is up)
            }
        }
    }

    pivot = GetElement(m_num_rows - 1, m_num_rows - 1);
    if (fabs(pivot) <= MIN_PIVOT) {
        pivot = INFINITE_PIVOT;
        SetElement(m_num_rows - 1, m_num_rows - 1, pivot);
        if (!err) {
            m_determinant = 0;
            err = (1);
        }
    } else {
        m_determinant *= pivot;
    }

    return err;
}

// Substitution using existing LDL factorization
void ChLinkedListMatrix::Solve_LDL(const ChMatrix<>& b, ChMatrix<>& x) {
    assert(m_num_rows == b.GetRows());
    assert(m_num_rows == x.GetRows());

    // BACKWARD substitution - L
    double xlast = b.GetElement(m_pindices[0], 0);
    x.SetElement(m_pindices[0], 0, xlast);

    for (int k = 1; k < m_num_rows; k++) {
        double sum = 0;
        for (int j = 0; j < k; j++) {
            sum += GetElement(j, k) * x.GetElement(m_pindices[j], 0);
        }
        double val = b.GetElement(m_pindices[k], 0) - sum;
        x.SetElement(m_pindices[k], 0, val);
    }

    // BACKWARD substitution - D
    for (int k = 0; k < m_num_rows; k++) {
        double val = x.GetElement(m_pindices[k], 0) / GetElement(k, k);
        x.SetElement(m_pindices[k], 0, val);
    }

    // BACKWARD substitution - L'
    for (int k = m_num_rows - 2; k >= 0; k--) {
        double sum = 0;

        ChMelement* rowel = GetElarrayMel(k);

        for (; rowel != nullptr; rowel = rowel->next) {
            if (rowel->col >= k + 1) {
                sum += rowel->val * x.GetElement(m_pindices[rowel->col], 0);
            }
        }

        double val = x.GetElement(m_pindices[k], 0) - sum;
        x.SetElement(m_pindices[k], 0, val);
    }
}

// LDL Factorization + substitution
int ChLinkedListMatrix::SolveSymmetric(const ChMatrix<>& b, ChMatrix<>& x) {
    int err = Setup_LDL();
    Solve_LDL(b, x);
    return err;
}

// Support function for LDL factorization
int ChLinkedListMatrix::BestPivotDiag(int current) {
    double temp = 0;
    int i;
    int pivotrow = current;

    for (i = current; i < m_num_rows; i++) {
        if (fabs(GetElement(i, i)) > temp) {
            temp = fabs(GetElement(i, i));
            pivotrow = i;
        }
        if (temp >= PIVOT_ACCEPT_TRESHOLD)
            break;
    }
    return pivotrow;
}

// Support function for LDL factorization
// Effect only on upper-left triangle!
void ChLinkedListMatrix::DiagPivotSymmetric(int rowA, int rowB) {
    int elcol, elrow;
    double temp;
    for (elrow = 0; elrow < rowA; elrow++) {
        temp = GetElement(elrow, rowA);
        SetElement(elrow, rowA, GetElement(elrow, rowB));
        SetElement(elrow, rowB, temp);
    }
    for (elcol = rowB + 1; elcol < m_num_cols; elcol++) {
        temp = GetElement(rowA, elcol);
        SetElement(rowA, elcol, GetElement(rowB, elcol));
        SetElement(rowB, elcol, temp);
    }
    for (elcol = rowA + 1; elcol < rowB; elcol++) {
        temp = GetElement(rowA, elcol);
        SetElement(rowA, elcol, GetElement(elcol, rowB));
        SetElement(elcol, rowB, temp);
    }
    temp = GetElement(rowA, rowA);
    SetElement(rowA, rowA, GetElement(rowB, rowB));
    SetElement(rowB, rowB, temp);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

void ChLinkedListMatrix::StreamOUTsparseMatlabFormat(ChStreamOutAscii& mstream) {
    for (int ii = 0; ii < m_num_rows; ii++) {
        for (int jj = 0; jj < m_num_cols; jj++) {
            double elVal = GetElement(ii, jj);
            if (elVal || (ii + 1 == m_num_rows && jj + 1 == m_num_cols)) {
                mstream << ii + 1 << " " << jj + 1 << " " << elVal << "\n";
            }
        }
    }
}

void ChLinkedListMatrix::StreamOUT(ChStreamOutAscii& mstream) {
    mstream << "\n"
            << "Matrix " << m_num_rows << " rows, " << m_num_cols << " columns."
            << "\n";
    for (int i = 0; i < ChMin(m_num_rows, 8); i++) {
        for (int j = 0; j < ChMin(m_num_cols, 8); j++)
            mstream << GetElement(i, j) << "  ";
        if (m_num_cols > 8)
            mstream << "...";
        mstream << "\n";
    }
    if (m_num_rows > 8)
        mstream << "... \n\n";
}

}  // end namespace chrono
