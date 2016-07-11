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
    for (int r = 0; r < rows; r++) {
        ChMelement* mguess = *(elarray + r);
        for (int c = 0; c < columns; c++) {
            double el = mat.GetElement(r, c);
            if (el != 0)
                mguess = SetElement(r, c, el, mguess);
        }
    }
}

ChLinkedListMatrix::ChLinkedListMatrix(const ChLinkedListMatrix& other) {
    Reset(other.rows, other.columns);

    for (int r = 0; r < rows; r++) {
        ChMelement* eguess = *(elarray + r);

        for (ChMelement* srowel = *(other.elarray + r); srowel != NULL; srowel = srowel->next) {
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
    elarray = NULL;

    for (ChNode<ChMelement>* mnode = mbufferlist.GetHead(); mnode != NULL; mnode = mnode->next) {
        free(mnode->data);  // deallocate memory buffers
    }

    mbufferlist.RemoveAll();  //  may be avoided, since it will automatically remove at list deletion
    mtot_size = 0;
}

void ChLinkedListMatrix::Build(double fill_in) {
    mtot_size = 0;
    mbuffer_added = 0;
    mnextel = NULL;

    // alloc buffer of elements
    mbuffer_size = (int)(rows * columns * fill_in);
    if (rows > 5000)
        mbuffer_size = SPM_DEF_MAXELEMENTS;
    if (columns > 5000)
        mbuffer_size = SPM_DEF_MAXELEMENTS;
    if (mbuffer_size > SPM_DEF_MAXELEMENTS)
        mbuffer_size = SPM_DEF_MAXELEMENTS;
    if (mbuffer_size < rows * 2)
        mbuffer_size = rows * 2;

    ChMelement* mbuffer = (ChMelement*)calloc(mbuffer_size, sizeof(ChMelement));

    mtot_size += mbuffer_size;
    mnextel = mbuffer;
    mbufferlist.AddHead(mbuffer);

    // alloc pointers to first column
    elarray = (ChMelement**)calloc(rows, sizeof(ChMelement*));

    for (int i = 0; i < rows; i++) {
        *(elarray + i) = mnextel;                    // initialize vector of 1st col pointr pointers
        (mnextel)->Initialize(0, NULL, NULL, i, 0);  // initialize 1st col.elements
        mbuffer_added++;                             // increment the counter of "used" elements.
        mnextel++;
    }
}

// Copy this matrix to a dense matrix.
void ChLinkedListMatrix::CopyToMatrix(ChMatrix<>& mat) {
    mat.Reset(rows, columns);

    ChMelement* currel;
    for (int i = 0; i < rows; i++) {
        for (currel = *(elarray + i); currel != NULL; currel = currel->next) {
            mat.SetElement(currel->row, currel->col, currel->val);
        }
    }
}

void ChLinkedListMatrix::MoreBuffer(double inflate) {
    mbuffer_size = (int)(inflate * (double)mbuffer_size);  // inflate buffer size

    if (this->rows < 5000)
        if ((mtot_size + mbuffer_size) > rows * columns)
            mbuffer_size = rows * columns - mtot_size;

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
    Reset(rows, columns);
}

void ChLinkedListMatrix::Reset(int row, int col, int nonzeros) {
    // realloc 1st column array, only if needed
    if (row != rows) {
        if (elarray)
            free(elarray);
        elarray = (ChMelement**)calloc(row, sizeof(ChMelement*));
    }

    // realloc buffer (but only if needed!!! -see the if() conditions below.. - ) ,
    // eventually compacting list into a _new_ single buffer

    int mneed_buffer_size = 0;

    if (mbufferlist.Count() > 1)
        mneed_buffer_size = mtot_size;  //   sum of all list of buffers, if multiple

    if (mtot_size < row * 2)
        mneed_buffer_size = row * 2;  // ..at least two columns

    if (row < 5000)
        if (mtot_size > (int)(row * col * 1.2))
            mneed_buffer_size = row * col;  // ..at maximum row*col

    if (mneed_buffer_size)  // ok, a new allocation must be done...
    {
        // manage reallocation of buffer...
        for (ChNode<ChMelement>* mnode = mbufferlist.GetHead(); mnode != NULL; mnode = mnode->next) {
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

    rows = row;
    columns = col;

    mbuffer_added = 0;
    mnextel = (mbufferlist.GetHead())->data;  // rewind the elem. pointer

    // Just reinitialize 1st row elements
    for (int i = 0; i < rows; i++) {
        *(elarray + i) = mnextel;                    // initialize vector of 1st col pointr pointers
        (mnextel)->Initialize(0, NULL, NULL, i, 0);  // initialize 1st col.elements
        mbuffer_added++;                             // increment the counter of "used" elements.
        mnextel++;
    }
}

void ChLinkedListMatrix::ResetBlocks(int row, int col) {
    if ((row == rows) && (col == columns))  // size doesn't change
    {
        static ChMelement* currel;
        for (int i = 0; i < rows; i++) {
            for (currel = *(elarray + i); currel != NULL; currel = currel->next) {
                currel->val = 0.0;
            }
        }
    } else  // size changes
    {
        Reset(row, col);
    }
}

// optimized SetElement,  returning the fetched Melement*
ChMelement* ChLinkedListMatrix::SetElement(int row, int col, double val, ChMelement* guess) {
    assert(row >= 0);
    assert(col >= 0);
    assert(row < rows);
    assert(col < columns);
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
                newguess = NewElement(val, NULL, guess, row, col);
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
                newguess = NewElement(val, guess, NULL, row, col);
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
    assert(row < rows);
    assert(col < columns);
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

double ChLinkedListMatrix::GetElement(int row, int col) {
    double retv;
    ChMelement* guess;
    guess = *(elarray + row);
    GetElement(row, col, &retv, guess);
    return retv;
}

void ChLinkedListMatrix::SwapRows(int a, int b) {
    ChMelement* mtemp;
    mtemp = *(elarray + a);
    *(elarray + a) = *(elarray + b);  // rows lists easy switched
    *(elarray + b) = mtemp;           // here...

    for (mtemp = *(elarray + a); mtemp != NULL; mtemp = mtemp->next) {
        mtemp->row = a;
    }
    for (mtemp = *(elarray + b); mtemp != NULL; mtemp = mtemp->next) {
        mtemp->row = b;
    }
}

void ChLinkedListMatrix::SwapColumns(int a, int b) {
    ChMelement* guessA;
    ChMelement* guessB;
    double valA, valB;

    for (int i = 0; i < rows; i++) {
        guessA = GetElement(i, a, &valA, *(elarray + i));
        guessB = GetElement(i, b, &valB, *(elarray + i));

        SetElement(i, a, valB, guessA);
        SetElement(i, b, valA, guessB);
    }
}

void ChLinkedListMatrix::PasteMatrix(ChMatrix<>* matra, int insrow, int inscol, bool overwrite, bool transp) {
    int i, j;
    int maxrows = matra->GetRows();
    int maxcol = matra->GetColumns();
    ChMelement* eguess;
    double val;

    for (i = 0; i < maxrows; i++) {
        eguess = *(elarray + i + insrow);

        for (j = 0; j < maxcol; j++) {
            val = (matra->GetElement(i, j));
            if (val)
                eguess = SetElement(i + insrow, j + inscol, val, eguess);
        }
    }
}

void ChLinkedListMatrix::PasteTranspMatrix(ChMatrix<>* matra, int insrow, int inscol) {
    int i, j;
    int maxrows = matra->GetRows();
    int maxcol = matra->GetColumns();
    ChMelement* eguess;
    double val;

    for (j = 0; j < maxcol; j++) {
        eguess = *(elarray + j + insrow);

        for (i = 0; i < maxrows; i++) {
            val = (matra->GetElement(i, j));
            if (val)
                eguess = SetElement(j + insrow, i + inscol, val, eguess);
        }
    }
}

void ChLinkedListMatrix::PasteSumMatrix(ChMatrix<>* matra, int insrow, int inscol) {
    int i, j;
    int maxrows = matra->GetRows();
    int maxcol = matra->GetColumns();
    ChMelement* eguess;
    ChMelement* eguessread;
    double val, aval, sum;

    for (i = 0; i < maxrows; i++) {
        eguess = *(elarray + i + insrow);
        eguessread = eguess;

        for (j = 0; j < maxcol; j++) {
            val = (matra->GetElement(i, j));
            if (val) {
                eguessread = GetElement(i + insrow, j + inscol, &aval, eguessread);
                sum = val + aval;
                eguess = SetElement(i + insrow, j + inscol, sum, eguess);
            }
        }
    }
}

void ChLinkedListMatrix::PasteSumTranspMatrix(ChMatrix<>* matra, int insrow, int inscol) {
    int i, j;
    int maxrows = matra->GetRows();
    int maxcol = matra->GetColumns();
    ChMelement* eguess;
    ChMelement* eguessread;
    double val, aval, sum;

    for (j = 0; j < maxcol; j++) {
        eguess = *(elarray + j + insrow);
        eguessread = eguess;

        for (i = 0; i < maxrows; i++) {
            val = (matra->GetElement(i, j));
            if (val) {
                eguessread = GetElement(j + insrow, i + inscol, &aval, eguessread);
                sum = val + aval;
                eguess = SetElement(j + insrow, i + inscol, sum, eguess);
            }
        }
    }
}

void ChLinkedListMatrix::PasteMatrix(ChLinkedListMatrix* matra, int insrow, int inscol) {
    int i;
    int maxrows = matra->GetRows();
    int maxcol = matra->GetColumns();
    ChMelement* eguess;
    ChMelement* srowel;
    double val;

    for (i = 0; i < maxrows; i++) {
        eguess = *(elarray + i + insrow);

        for (srowel = *(matra->elarray + i); srowel != NULL; srowel = srowel->next) {
            val = srowel->val;
            if (val)
                eguess = SetElement(i + insrow, srowel->col + inscol, val, eguess);
        }
    }
}

void ChLinkedListMatrix::PasteTranspMatrix(ChLinkedListMatrix* matra, int insrow, int inscol) {
    int i, j;
    int maxrows = matra->GetRows();
    int maxcol = matra->GetColumns();
    ChMelement* eguess;
    double val;
    // note: could be optimized for wide matra matrices..
    for (j = 0; j < maxcol; j++) {
        eguess = *(elarray + j + insrow);

        for (i = 0; i < maxrows; i++) {
            val = (matra->GetElement(i, j));
            if (val)
                eguess = SetElement(j + insrow, i + inscol, val, eguess);
        }
    }
}

void ChLinkedListMatrix::PasteClippedMatrix(ChMatrix<>* matra,
                                            int cliprow,
                                            int clipcol,
                                            int nrows,
                                            int ncolumns,
                                            int insrow,
                                            int inscol,
                                            bool overwrite) {
    int i, j;
    int maxrows = matra->GetRows();
    int maxcol = matra->GetColumns();
    ChMelement* eguess;
    double val;

    for (i = 0; i < nrows; i++) {
        eguess = *(elarray + i + insrow);

        for (j = 0; j < ncolumns; j++) {
            val = (matra->GetElement(i + cliprow, j + clipcol));
            if (val)
                eguess = SetElement(i + insrow, j + inscol, val, eguess);
        }
    }
}

void ChLinkedListMatrix::PasteSumClippedMatrix(ChMatrix<>* matra,
                                               int cliprow,
                                               int clipcol,
                                               int nrows,
                                               int ncolumns,
                                               int insrow,
                                               int inscol) {
    int i, j;
    int maxrows = matra->GetRows();
    int maxcol = matra->GetColumns();
    ChMelement* eguess;
    ChMelement* eguessread;
    double val, aval, sum;

    for (i = 0; i < nrows; i++) {
        eguess = *(elarray + i + insrow);
        eguessread = eguess;

        for (j = 0; j < ncolumns; j++) {
            val = (matra->GetElement(i + cliprow, j + clipcol));
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

    for (int i = 0; i < rows; i++) {
        for (currel = *(elarray + i); currel != NULL; currel = currel->next) {
            currel->val = currel->val * factor;
        }
    }
}

void ChLinkedListMatrix::Neg() {
    ChMelement* currel;

    for (int i = 0; i < rows; i++) {
        for (currel = *(elarray + i); currel != NULL; currel = currel->next) {
            currel->val = -(currel->val);
        }
    }
}

// -----------------------------------------------------------------------------
// Solution of general linear systems using LU factorization
// -----------------------------------------------------------------------------

// LU factorization
int ChLinkedListMatrix::Setup_LU() {
    assert(GetRows() == GetColumns());

    ChMelement* rowel;
    ChMelement* subrowel;
    int i, k, pivrow, eqpivoted;
    double r, pivot, mval, subval, newval, leader;
    int err = 0;

    // initialize pivot index array
    m_pindices.resize(GetRows());
    for (int ind = 0; ind < rows; ind++) {
        m_pindices[ind] = ind;
    }

    m_determinant = 1;

    for (k = 1; k < rows; k++) {
        pivot = GetElement((k - 1), (k - 1));

        if (fabs(pivot) < ACCEPT_PIVOT) {
            // pivoting is needed, so swap equations
            pivrow = BestPivotRow(k - 1);
            SwapRows((k - 1), pivrow);
            m_determinant *= -1;

            eqpivoted = m_pindices[pivrow];  // swap eq.ids in pivot array
            m_pindices[pivrow] = m_pindices[k - 1];
            m_pindices[k - 1] = eqpivoted;

            pivot = GetElement((k - 1), (k - 1));  // fetch new pivot
        }

        // was unable to find better pivot: force solution to zero.and go ahead
        if (fabs(pivot) <= MIN_PIVOT) {
            pivot = INFINITE_PIVOT;
            SetElement(k - 1, k - 1, pivot);
            if (!err) {
                m_determinant = 0;
                err = (1 + rows - k);  // report deficiency
            }
        } else {
            m_determinant *= pivot;
        }

        for (i = k; i < rows; i++) {
            leader = GetElement(i, (k - 1));

            if (leader) {
                r = leader / pivot;         // compute multiplier
                SetElement(i, (k - 1), r);  // store the multiplier in L part!!!

                subrowel = GetElarrayMel(i);   // initialize guessed sparse elements
                rowel = GetElarrayMel(k - 1);  // for speed optimization. They are in two rows.

                // for (j=k; j < columns; j++)  where j = rowel->col
                for (NULL; rowel != NULL; rowel = rowel->next) {
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

    pivot = GetElement((rows - 1), (rows - 1));
    if (fabs(pivot) <= MIN_PIVOT) {
        pivot = INFINITE_PIVOT;
        SetElement(rows - 1, rows - 1, pivot);
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
    assert(GetRows() == b.GetRows());
    assert(GetRows() == x.GetRows());

    // BACKWARD substitution - L
    double xlast = b.GetElement(m_pindices[0], 0);
    x.SetElement(0, 0, xlast);

    for (int k = 1; k < rows; k++) {
        double sum = 0;

        ChMelement* rowel = GetElarrayMel(k);

        for (NULL; (rowel != NULL && rowel->col < k); rowel = rowel->next) {
            sum += rowel->val * (x.GetElement(rowel->col, 0));
        }

        double val = (b.GetElement(m_pindices[k], 0) - sum);
        x.SetElement(k, 0, val);
    }

    // BACKWARD substitution - U
    xlast = x.GetElement((rows - 1), 0) / GetElement(rows - 1, rows - 1);
    x.SetElement((rows - 1), 0, xlast);

    for (int k = (rows - 2); k >= 0; k--) {
        double sum = 0;

        ChMelement* rowel = GetElarrayMel(k);

        for (NULL; rowel != NULL; rowel = rowel->next) {
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

    for (int i = current; i < rows; i++) {
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
    assert(GetRows() == GetColumns());

    ChMelement* rowel;
    ChMelement* subrowel;
    int i, k, pivrow, eqpivoted;
    double r, leader, pivot, mval, subval, newval;
    int err = 0;

    // initialize pivot index array
    m_pindices.resize(GetRows());
    for (int ind = 0; ind < rows; ind++) {
        m_pindices[ind] = ind;
    }

    m_determinant = 1;

    for (k = 1; k < rows; k++) {
        pivot = GetElement((k - 1), (k - 1));

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
                err = (1 + rows - k);
            }
        } else {
            m_determinant *= pivot;
        }

        for (i = k; i < rows; i++) {
            leader = GetElement((k - 1), i);

            if (leader) {
                r = (leader / pivot);  // compute multiplier (mirror look A)

                subrowel = GetElarrayMel(i);   // initialize guessed sparse elements
                rowel = GetElarrayMel(k - 1);  // for speed optimization. They are in two rows.

                // advance top row element till it has (col >= k)

                // for (j=i; j < columns; j++)     only upper right part
                for (NULL; rowel != NULL; rowel = rowel->next) {
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

    pivot = GetElement((rows - 1), (rows - 1));
    if (fabs(pivot) <= MIN_PIVOT) {
        pivot = INFINITE_PIVOT;
        SetElement(rows - 1, rows - 1, pivot);
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
    assert(GetRows() == b.GetRows());
    assert(GetRows() == x.GetRows());

    // BACKWARD substitution - L
    double xlast = b.GetElement(m_pindices[0], 0);
    x.SetElement(m_pindices[0], 0, xlast);

    for (int k = 1; k < rows; k++) {
        double sum = 0;
        for (int j = 0; j < k; j++) {
            sum += (GetElement(j, k)) * (x.GetElement(m_pindices[j], 0));
        }
        double val = (b.GetElement(m_pindices[k], 0) - sum);
        x.SetElement(m_pindices[k], 0, val);
    }

    // BACKWARD substitution - D
    for (int k = 0; k < rows; k++) {
        double val = x.GetElement(m_pindices[k], 0) / GetElement(k, k);
        x.SetElement(m_pindices[k], 0, val);
    }

    // BACKWARD substitution - L'
    for (int k = (rows - 2); k >= 0; k--) {
        double sum = 0;

        ChMelement* rowel = GetElarrayMel(k);

        for (NULL; rowel != NULL; rowel = rowel->next) {
            if (rowel->col >= k + 1) {
                sum += rowel->val * (x.GetElement(m_pindices[rowel->col], 0));
            }
        }

        double val = (x.GetElement(m_pindices[k], 0) - sum);
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

    for (i = current; i < rows; i++) {
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
    for (elcol = rowB + 1; elcol < columns; elcol++) {
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
    for (int ii = 0; ii < this->GetRows(); ii++) {
        for (int jj = 0; jj < this->GetColumns(); jj++) {
            double elVal = this->GetElement(ii, jj);
            if (elVal || (ii + 1 == this->GetRows() && jj + 1 == this->GetColumns())) {
                mstream << ii + 1 << " " << jj + 1 << " " << elVal << "\n";
            }
        }
    }
}

void ChLinkedListMatrix::StreamOUT(ChStreamOutAscii& mstream) {
    mstream << "\n"
            << "Matrix " << GetRows() << " rows, " << GetColumns() << " columns."
            << "\n";
    for (int i = 0; i < ChMin(GetRows(), 8); i++) {
        for (int j = 0; j < ChMin(GetColumns(), 8); j++)
            mstream << GetElement(i, j) << "  ";
        if (GetColumns() > 8)
            mstream << "...";
        mstream << "\n";
    }
    if (GetRows() > 8)
        mstream << "... \n\n";
}

}  // end namespace chrono
