//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChLinkedListMatrix.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "core/ChMath.h"
#include "core/ChLinkedListMatrix.h"

#define PIVOT_ACCEPT_TRESHOLD 0.8
#define ACCEPT_PIVOT 0.001
#define MIN_PIVOT 0.00000001
#define INFINITE_PIVOT 1.e+34

namespace chrono {

void ChLinkedListMatrix::Build(int row, int col, double fullness) {
    rows = row;
    columns = col;

    mtot_size = 0;
    mbuffer_added = 0;
    mnextel = NULL;

    // alloc buffer of elements
    mbuffer_size = (int)(row * col * fullness);
    if (row > 5000)
        mbuffer_size = SPM_DEF_MAXELEMENTS;
    if (col > 5000)
        mbuffer_size = SPM_DEF_MAXELEMENTS;
    if (mbuffer_size > SPM_DEF_MAXELEMENTS)
        mbuffer_size = SPM_DEF_MAXELEMENTS;
    if (mbuffer_size < row * 2)
        mbuffer_size = row * 2;

    ChMelement* mbuffer = (ChMelement*)calloc(mbuffer_size, sizeof(ChMelement));

    mtot_size += mbuffer_size;
    mnextel = mbuffer;
    mbufferlist.AddHead(mbuffer);

    // alloc pointers to first column
    elarray = (ChMelement**)calloc(row, sizeof(ChMelement*));

    for (int i = 0; i < row; i++) {
        *(elarray + i) = mnextel;                    // initialize vector of 1st col pointr pointers
        (mnextel)->Initialize(0, NULL, NULL, i, 0);  // initialize 1st col.elements
        mbuffer_added++;                             // increment the counter of "used" elements.
        mnextel++;
    }
}

ChLinkedListMatrix::ChLinkedListMatrix(int row, int col, double fullness) {
    ChLinkedListMatrix::Build(row, col, fullness);
}

ChLinkedListMatrix::ChLinkedListMatrix(int row, int col) {
    ChLinkedListMatrix::Build(row, col, SPM_DEF_FULLNESS);
}

ChLinkedListMatrix::ChLinkedListMatrix() {
    ChLinkedListMatrix::Build(3, 3, 1);
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

// copy optimized also for source sparse matrix
// (copy from matrices with structural zeros does not waste space)

void ChLinkedListMatrix::CopyFromMatrix(ChMatrix<>* matra) {
    double el;
    ChMelement* mguess;

    Reset(matra->GetRows(), matra->GetColumns());

    // fill matrix
    for (int r = 0; r < rows; r++) {
        mguess = *(elarray + r);
        for (int c = 0; c < columns; c++) {
            el = matra->GetElement(r, c);
            if (el != 0)
                mguess = SetElement(r, c, el, mguess);
        }
    }
}

void ChLinkedListMatrix::CopyToMatrix(ChMatrix<>* matra) {
    matra->Reset(rows, columns);  // first, set all zeros in full matrix...

    ChMelement* currel;
    for (int i = 0; i < rows; i++) {
        for (currel = *(elarray + i); currel != NULL; currel = currel->next) {
            matra->SetElement(currel->row, currel->col, currel->val);
        }
    }
}

void ChLinkedListMatrix::CopyFromMatrix(ChLinkedListMatrix* matra) {
    static ChMelement* eguess;
    static ChMelement* srowel;
    double val;

    Reset(matra->rows, matra->columns);

    for (int r = 0; r < rows; r++) {
        eguess = *(elarray + r);

        for (srowel = *(matra->elarray + r); srowel != NULL; srowel = srowel->next) {
            val = srowel->val;
            if (val)
                eguess = SetElement(r, srowel->col, val, eguess);
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

	bool ChLinkedListMatrix::Resize(int nrows, int ncols, int nonzeros)
	{ assert(false); return 0; }

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

void ChLinkedListMatrix::PasteMatrixFloat(ChMatrix<float>* matra, int insrow, int inscol, bool overwrite, bool transp) {
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

void ChLinkedListMatrix::PasteTranspMatrixFloat(ChMatrix<float>* matra, int insrow, int inscol) {
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

void ChLinkedListMatrix::MatrMultiply(ChLinkedListMatrix* matra, ChLinkedListMatrix* matrb) {
    //**** TO DO ****
}

void ChLinkedListMatrix::MatrMultiplyT(ChLinkedListMatrix* matra, ChLinkedListMatrix* matrb) {
    //**** TO DO ****
}

void ChLinkedListMatrix::MatrTMultiply(ChLinkedListMatrix* matra, ChLinkedListMatrix* matrb) {
    //**** TO DO ****
}

void ChLinkedListMatrix::MatrAdd(ChLinkedListMatrix* matra, ChLinkedListMatrix* matrb) {
    //**** TO DO ****
}

void ChLinkedListMatrix::MatrSub(ChLinkedListMatrix* matra, ChLinkedListMatrix* matrb) {
    //**** TO DO ****
}

void ChLinkedListMatrix::MatrInc(ChLinkedListMatrix* matra) {
    //**** TO DO ****
}

void ChLinkedListMatrix::MatrScale(double factor) {
    ChMelement* currel;

    for (int i = 0; i < rows; i++) {
        for (currel = *(elarray + i); currel != NULL; currel = currel->next) {
            currel->val = currel->val * factor;
        }
    }
}

void ChLinkedListMatrix::MatrTranspose() {
    //**** TO DO ****
}

void ChLinkedListMatrix::Neg() {
    ChMelement* currel;

    for (int i = 0; i < rows; i++) {
        for (currel = *(elarray + i); currel != NULL; currel = currel->next) {
            currel->val = -(currel->val);
        }
    }
}

// Linear algebra function

int ChLinkedListMatrix::BestPivotRow(int current) {
    double temp = 0;
    int i;
    int pivotrow = current;

    for (i = current; i < rows; i++) {
        if (fabs(GetElement(i, current)) > temp) {
            temp = fabs(GetElement(i, current));
            pivotrow = i;
        }
        if (temp >= PIVOT_ACCEPT_TRESHOLD)
            break;
    }
    return pivotrow;
}

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

// Effect only on upper-left triangle!!!

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

// Solution of linear systems, optimized for sparse matrix.
// The array of pivot indexes (pivarray) is optional

int ChLinkedListMatrix::Solve_LinSys(ChMatrix<>* B,
                                 ChMatrix<>* X,
                                 int* pivarray,
                                 double* det)  // the object is the [A] matrix.
{
    ChMelement* rowel;
    ChMelement* subrowel;
    int i, k, pivrow, eqpivoted;
    double r, bi, x, sum, pivot, pivlast, mval, subval, newval, leader;
    int err = 0;

    if (pivarray)  // initialize pivot index array
    {
        for (int ind = 0; ind < rows; ind++) {
            pivarray[ind] = ind;
        }
    }

    *det = 1;

    // FORWARD reduction
    for (k = 1; k < rows; k++) {
        pivot = GetElement((k - 1), (k - 1));
        if (fabs(pivot) < ACCEPT_PIVOT) {
            pivrow = BestPivotRow(k - 1);
            SwapRows((k - 1), pivrow);
            B->SwapRows((k - 1), pivrow);
            *det = -*det;
            if (pivarray) {
                eqpivoted = pivarray[pivrow];  // swap eq.ids in pivot array
                pivarray[pivrow] = pivarray[k - 1];
                pivarray[k - 1] = eqpivoted;
            }
            pivot = GetElement((k - 1), (k - 1));
        }
        if (fabs(pivot) <= MIN_PIVOT)  // was unable to find better pivot: force solution to zero.and go ahead
        {
            //*det = *det;
            pivot = INFINITE_PIVOT;
            SetElement(k - 1, k - 1, pivot);
            if (!err)
                err = (1 + rows - k);  // report deficiency
        } else
            *det = *det * pivot;

        for (i = k; i < rows; i++) {
            leader = GetElement(i, (k - 1));

            if (leader) {
                r = leader / pivot;

                bi = B->GetElement(i, 0) - r * B->GetElement((k - 1), 0);
                B->SetElement(i, 0, bi);

                subrowel = GetElarrayMel(i);
                rowel = GetElarrayMel(k - 1);

                // while ((rowel->col<k) && (rowel->next))
                //	 rowel = rowel->next;		// advance top row element till it has (col >= k)

                for (NULL; rowel != NULL; rowel = rowel->next) {
                    if (rowel->col >= k)  // skip
                    {
                        mval = rowel->val;
                        subrowel = GetElement(i, rowel->col, &subval, subrowel);
                        newval = subval - r * mval;
                        subrowel = SetElement(i, rowel->col, newval, subrowel);
                    }
                }
            }
        }
    }

    pivlast = GetElement((rows - 1), (rows - 1));
    if (fabs(pivlast) <= MIN_PIVOT) {
        //*det = *det;
        pivlast = INFINITE_PIVOT;
        SetElement(rows - 1, rows - 1, pivlast);
        if (!err)
            err = (1);  // report deficiency
    } else
        *det = *det * pivlast;

    // BACKWARD substitution
    double xlast = B->GetElement(rows - 1, 0) / pivlast;
    X->SetElement((rows - 1), 0, xlast);

    for (k = (rows - 2); k >= 0; k--) {
        sum = 0;

        rowel = GetElarrayMel(k);

        for (NULL; rowel != NULL; rowel = rowel->next)  // for (j=(k+1); j< rows; j++)
        {
            if (rowel->col >= k + 1)  // skip L part,
            {
                sum += rowel->val * (X->GetElement(rowel->col, 0));  // sum += (GetElement(k,j))*(X->GetElement(j,0));
            }
        }

        x = (B->GetElement(k, 0) - sum) / GetElement(k, k);
        X->SetElement(k, 0, x);
    }

    return err;
}

void ChLinkedListMatrix::Solve_LinSys(ChMatrix<>* B, ChMatrix<>* X) {
    double det;
    Solve_LinSys(B, X, (int*)NULL, &det);
}

// LU decomposition and solution

int ChLinkedListMatrix::Decompose_LU(int* pivarray, double* det) {
    ChMelement* rowel;
    ChMelement* subrowel;
    int i, k, pivrow, eqpivoted;
    double r, pivot, mval, subval, newval, leader;
    int err = 0;

    if (pivarray)  // initialize pivot index array
    {
        for (int ind = 0; ind < rows; ind++) {
            pivarray[ind] = ind;
        }
    } else
        return TRUE;  // error: no pivot array.

    *det = 1;

    for (k = 1; k < rows; k++) {
        pivot = GetElement((k - 1), (k - 1));

        if (fabs(pivot) < ACCEPT_PIVOT) {
            // pivoting is needed, so swap equations
            pivrow = BestPivotRow(k - 1);
            SwapRows((k - 1), pivrow);
            *det = -*det;

            eqpivoted = pivarray[pivrow];  // swap eq.ids in pivot array
            pivarray[pivrow] = pivarray[k - 1];
            pivarray[k - 1] = eqpivoted;

            pivot = GetElement((k - 1), (k - 1));  // fetch new pivot
        }

        if (fabs(pivot) <= MIN_PIVOT)  // was unable to find better pivot: force solution to zero.and go ahead
        {
            //*det = *det;
            pivot = INFINITE_PIVOT;
            SetElement(k - 1, k - 1, pivot);
            if (!err)
                err = (1 + rows - k);  // report deficiency
        } else
            *det = *det * pivot;

        for (i = k; i < rows; i++) {
            leader = GetElement(i, (k - 1));

            if (leader) {
                r = leader / pivot;         // compute multiplier
                SetElement(i, (k - 1), r);  // store the multiplier in L part!!!

                subrowel = GetElarrayMel(i);   // initialize guessed sparse elements
                rowel = GetElarrayMel(k - 1);  // for speed optimization. They are in two rows.

                // while ((rowel->col<k) && (rowel->next))
                //	 rowel = rowel->next;		// advance top row element till it has (col >= k)

                for (NULL; rowel != NULL; rowel = rowel->next)  // for (j=k; j < columns; j++)  where j = rowel->col
                {
                    if (rowel->col >= k)  // skip
                    {
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
        //*det = *det;
        pivot = INFINITE_PIVOT;
        SetElement(rows - 1, rows - 1, pivot);
        if (!err)
            err = (1);  // report deficiency
    } else
        *det = *det * pivot;

    return err;
}

int ChLinkedListMatrix::Solve_LU(ChMatrix<>* B, ChMatrix<>* X, int* pivarray) {
    ChMelement* rowel;
    int k;
    double sum, x;

    // BACKWARD substitution - L
    double xlast = B->GetElement(pivarray[0], 0);  //  ../ GetElement(0,0) => ../1;
    X->SetElement(0, 0, xlast);

    for (k = 1; k < rows; k++) {
        sum = 0;

        rowel = GetElarrayMel(k);  // L(k,0)

        for (NULL; (rowel != NULL && rowel->col < k); rowel = rowel->next)  // (j=0; j<k; j++)
        {
            sum += rowel->val * (X->GetElement(rowel->col, 0));  // sum += (GetElement(k,j))*(X->GetElement(j,0));
        }

        x = (B->GetElement(pivarray[k], 0) - sum);  //  ../ GetElement(k,k) => ../1;
        X->SetElement(k, 0, x);
    }

    // BACKWARD substitution - U
    xlast = X->GetElement((rows - 1), 0) / GetElement(rows - 1, rows - 1);
    X->SetElement((rows - 1), 0, xlast);

    for (k = (rows - 2); k >= 0; k--) {
        sum = 0;

        rowel = GetElarrayMel(k);

        for (NULL; rowel != NULL; rowel = rowel->next)  // for (j=(k+1); j< rows; j++)
        {
            if (rowel->col >= k + 1)  // skip L part,   U row starts from (k,k+1) or further
            {
                sum += rowel->val * (X->GetElement(rowel->col, 0));  // sum += (GetElement(k,j))*(X->GetElement(j,0));
            }
        }

        x = (X->GetElement(k, 0) - sum) / GetElement(k, k);
        X->SetElement(k, 0, x);
    }

    return TRUE;
}

//////////////////////////////////////////////////////////////////////////////

//
// LDL decomposition (only upper right part of [A] is used!)
//

int ChLinkedListMatrix::Decompose_LDL(int* pivarray, double* det, int from_eq) {
    ChMelement* rowel;
    ChMelement* subrowel;
    int i, k, pivrow, eqpivoted;
    double r, leader, pivot, mval, subval, newval;
    int err = 0;

    if (pivarray)  // initialize pivot index array
    {
        for (int ind = 0; ind < rows; ind++) {
            pivarray[ind] = ind;
        }
    } else
        return TRUE;  // error: no pivot array.

    *det = 1;

    for (k = 1; k < rows; k++) {
        pivot = GetElement((k - 1), (k - 1));

        if (fabs(pivot) < ACCEPT_PIVOT) {
            // static cntpiv=0;
            // R3Error("pivot %d   %g", cntpiv, fabs(pivot));

            // pivoting is needed, so ...
            pivrow = BestPivotDiag(k - 1);
            DiagPivotSymmetric(k - 1, pivrow);  // swap both column and row (only upper right!)
            *det = -*det;                       // invert determ.sign

            eqpivoted = pivarray[pivrow];  // swap diagonal pivot indexes
            pivarray[pivrow] = pivarray[k - 1];
            pivarray[k - 1] = eqpivoted;

            pivot = GetElement((k - 1), (k - 1));
        }

        if (fabs(pivot) <= MIN_PIVOT)  // was unable to find better pivot: force solution to zero.and go ahead
        {
            pivot = INFINITE_PIVOT;
            SetElement(k - 1, k - 1, pivot);
            if (!err)
                err = (1 + rows - k);  // report deficiency
        } else
            *det = *det * pivot;

        for (i = k; i < rows; i++) {
            leader = GetElement((k - 1), i);

            if (leader) {
                r = (leader / pivot);  // compute multiplier (mirror look A)

                subrowel = GetElarrayMel(i);   // initialize guessed sparse elements
                rowel = GetElarrayMel(k - 1);  // for speed optimization. They are in two rows.

                // advance top row element till it has (col >= k)

                for (NULL; rowel != NULL; rowel = rowel->next)  // for (j=i; j < columns; j++)	// only upper right part
                {
                    if (rowel->col >= i)  // only upper right part is filled!
                    {
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
        if (!err)
            err = (1);  // report deficiency
    } else
        *det = *det * pivot;

    return err;
}

//
// Back substitution after LDL decomposition, to solve for X
//

int ChLinkedListMatrix::Solve_LDL(ChMatrix<>* B, ChMatrix<>* X, int* pivarray) {
    ChMelement* rowel;
    int j, k;
    double sum, x;

    // BACKWARD substitution - L
    double xlast = B->GetElement(pivarray[0], 0);  //  ../ GetElement(0,0) => ../1;
    X->SetElement(pivarray[0], 0, xlast);

    for (k = 1; k < rows; k++) {
        sum = 0;
        for (j = 0; j < k; j++) {
            sum += (GetElement(j, k)) * (X->GetElement(pivarray[j], 0));
        }
        x = (B->GetElement(pivarray[k], 0) - sum);  //  ../ GetElement(k,k) => ../1;
        X->SetElement(pivarray[k], 0, x);
    }
    // BACKWARD substitution - D
    for (k = 0; k < rows; k++) {
        x = X->GetElement(pivarray[k], 0) / GetElement(k, k);
        X->SetElement(pivarray[k], 0, x);
    }
    // BACKWARD substitution - L'
    for (k = (rows - 2); k >= 0; k--) {
        sum = 0;

        rowel = GetElarrayMel(k);

        for (NULL; rowel != NULL; rowel = rowel->next)  // for (j=(k+1); j< rows; j++)
        {
            if (rowel->col >= k + 1)  // skip L part,
            {
                sum += rowel->val *
                       (X->GetElement(pivarray[rowel->col], 0));  // sum += (GetElement(k,j))*(X->GetElement(j,0));
            }
        }

        x = (X->GetElement(pivarray[k], 0) - sum);  // .../ GetElement(k,k); -> ../1 ;
        X->SetElement(pivarray[k], 0, x);
    }

    return TRUE;
}

//
// Decompose and solve at once
//

int ChLinkedListMatrix::DecomposeAndSolve_LDL(ChMatrix<>* B, ChMatrix<>* X, double& mdet, int from_eq) {
    // decompose and solve at once

    assert(B->GetRows() == this->rows);
    assert(X->GetRows() == this->rows);
    assert(this->rows == this->columns);

    int* pivarray = (int*)calloc((this->rows), sizeof(int));

    int nredundancy = this->Decompose_LDL(pivarray, &mdet);
    this->Solve_LDL(B, X, pivarray);

    free(pivarray);

    return nredundancy;
}

//
// LCP solver
//

#define LCP_EPS 10e-6

//
// Decompose and solve at once (optimized for LCP iteration)
//

int ChLinkedListMatrix::DecomposeAndSolve_LDL_forLCP(ChMatrix<>* B,
                                                 ChMatrix<>* X,
                                                 double& mdet,
                                                 int i_D,
                                                 int i_C,
                                                 int n_unilaterals,
                                                 ChUnilateralData constr_data[],
                                                 int from_eq) {
    ChLinkedListMatrix tempM;
    tempM.CopyFromMatrix(this);

    // deactivate the non-clamped or not yet considered unilateral constraints
    for (int nd = 0; nd < n_unilaterals; nd++) {
        if (constr_data[nd].status != CONSTR_UNILATERAL_ON)
            tempM.SetElement(i_D + nd, i_D + nd, INFINITE_PIVOT);
    }

    tempM.DecomposeAndSolve_LDL(B, X, mdet, from_eq);

    return TRUE;
}

int ChLinkedListMatrix::DecomposeAndSolve_LDL_forLCP(ChLinkedListMatrix* Aorig,
                                                 ChLinkedListMatrix* Afact,
                                                 ChMatrix<>* B,
                                                 ChMatrix<>* X,
                                                 double& mdet,
                                                 int i_D,
                                                 int i_C,
                                                 int n_unilaterals,
                                                 ChUnilateralData constr_data[],
                                                 int from_eq,
                                                 int backup_from) {
    assert(B->GetRows() == Afact->rows);
    assert(X->GetRows() == Afact->rows);
    assert(Afact->rows == Afact->columns);
    assert(Aorig->GetRows() == Afact->rows);
    assert(Aorig->GetColumns() == Afact->columns);
    assert((from_eq == 0) || (from_eq >= backup_from));
    assert(from_eq < Afact->rows);
    assert(backup_from < Afact->rows);

    // The factorization:

    double adet;
    double* det = &adet;
    ChMelement* rowel;
    ChMelement* subrowel;
    int i, k;
    // int pivrow, eqpivoted;
    double r, leader, pivot, mval, subval, newval;
    int err = 0;

    int* pivarray = (int*)calloc((Afact->rows), sizeof(int));

    for (int indp = 0; indp < Afact->rows; indp++) {
        pivarray[indp] = indp;
    }

    int* modarray = (int*)calloc((Afact->rows), sizeof(int));

    for (int indm = 0; indm < Afact->rows; indm++) {
        modarray[indm] = false;
    }

    *det = 1;

    if (from_eq == 0) {
        Afact->CopyFromMatrix(Aorig);
    } else {
        // should be optimized?
        Afact->CopyFromMatrix(Aorig);
    }

    int begin_row = 0;

    if (from_eq > 0)
        begin_row = backup_from;

    for (k = begin_row + 1; k < Afact->rows; k++) {
        // Perform optimized backup of partially factored matrix into another sparse matrix
        if (from_eq == 0)
            if ((k - 1) == backup_from) {
                Aorig->CopyFromMatrix(Afact);
            }

        pivot = Afact->GetElement((k - 1), (k - 1));
        /*
                if (fabs(pivot) < ACCEPT_PIVOT)
                {
                    //static cntpiv=0;
                    //R3Error("pivot %d   %g", cntpiv, fabs(pivot));

                    // pivoting is needed, so ...
                    pivrow = Afact->BestPivotDiag(k-1);
                    Afact->DiagPivotSymmetric(k-1,pivrow);	// swap both column and row (only upper right!)
                    *det = - *det;					// invert determ.sign

                    eqpivoted = pivarray[pivrow];	// swap diagonal pivot indexes
                    pivarray[pivrow] =pivarray[k-1];
                    pivarray[k-1] = eqpivoted;

                    pivot = Afact->GetElement((k-1),(k-1));
                }
        */
        int row_num_D = (k - 1) - i_D;

        bool infinite_pivot = false;
        // force unactive bases to no effect, with infinite pivot
        if (row_num_D >= 0)
            if (constr_data[row_num_D].status != CONSTR_UNILATERAL_ON) {
                infinite_pivot = true;
                pivot = INFINITE_PIVOT;
                Afact->SetElement(k - 1, k - 1, pivot);
            }

        if (fabs(pivot) <= MIN_PIVOT)  // was unable to find better pivot: force solution to zero.and go ahead
        {
            infinite_pivot = true;
            pivot = INFINITE_PIVOT;
            Afact->SetElement(k - 1, k - 1, pivot);
            if (!err)
                err = (1 + Afact->rows - k);  // report deficiency
        } else
            *det = *det * pivot;

        for (i = k; i < Afact->rows; i++) {
            leader = Afact->GetElement((k - 1), i);

            if (leader != 0) {
                r = (leader / pivot);  // compute multiplier (mirror look A)

                if (!infinite_pivot) {
                    subrowel = Afact->GetElarrayMel(i);   // initialize guessed sparse elements
                    rowel = Afact->GetElarrayMel(k - 1);  // for speed optimization. They are in two rows.

                    // advance top row element till it has (col >= k)

                    for (NULL; rowel != NULL;
                         rowel = rowel->next)  // for (j=i; j < columns; j++)	// only upper right part
                    {
                        if (rowel->col >= i)  // only upper right part is filled!
                        {
                            mval = rowel->val;
                            subrowel = Afact->GetElement(i, rowel->col, &subval, subrowel);
                            newval = subval - r * mval;
                            subrowel = Afact->SetElement(i, rowel->col, newval, subrowel);
                        }
                    }
                }

                Afact->SetElement((k - 1), i, r);  // now can store the multiplier in L part (mirror look! is up)
            }
        }
    }

    pivot = Afact->GetElement((Afact->rows - 1), (Afact->rows - 1));
    if (fabs(pivot) <= MIN_PIVOT) {
        pivot = INFINITE_PIVOT;
        Afact->SetElement(Afact->rows - 1, Afact->rows - 1, pivot);
        if (!err)
            err = (1);  // report deficiency
    } else
        *det = *det * pivot;

    //
    // Back-Solve using the factored matrix

    Afact->Solve_LDL(B, X, pivarray);

    free(pivarray);
    free(modarray);

    return TRUE;
}

static void LCP_compute_DL_residuals(ChLinkedListMatrix* M,
                                     ChMatrix<>* B,
                                     ChMatrix<>* X,
                                     ChMatrix<>* D,
                                     ChMatrix<>* L,
                                     int i_D,
                                     int i_C,
                                     int n_unilaterals,
                                     ChUnilateralData constr_data[]) {
    // Recover the residuals of unilateral constraints, given body accelerations and jacobians
    // (for example, contact normal accelerations)

    D->FillElem(0);
    for (int res = 0; res < n_unilaterals; res++) {
        // the n-th D residual can be computed from jacobian [Dx] and B vector: Dn=[Dx]*x-Bn
        double sum = -B->GetElement(i_D + res, 0);
        for (int mcol = 0; mcol < i_C; mcol++)
            sum += X->GetElement(mcol, 0) * M->GetElement(i_D + res, mcol);

        // force to precise zero if constraint is clamped enough
        // (to avoid cycling because of numerical errors)
        // if (constr_data[res].status==CONSTR_UNILATERAL_ON)
        //	if (fabs(sum)<10e-6)
        //		sum = 0;

        D->SetElement(res, 0, sum);
    }

    // Recover the multipliers (ex. the reactions of constraints in acceleration problem)
    // for later easier reference.

    L->PasteClippedMatrix(X, i_D, 0, n_unilaterals, 1, 0, 0);
}

static bool LCP_compute_maxstep(ChMatrix<>* D,
                                ChMatrix<>* L,
                                ChMatrix<>* Dguess,
                                ChMatrix<>* Lguess,
                                const int idirection,
                                ChUnilateralData constr_data[],
                                int n_unilaterals,
                                int& ichanged,
                                double& step) {
    step = 1.0;
    ichanged = -1;

    double try_step;

    // Check limit caused by some constraint which must be turned OFF
    // because clamped constraints must always keep multiplier L<0

    int id;

    for (id = 0; id < n_unilaterals; id++) {
        if (constr_data[id].status == CONSTR_UNILATERAL_ON) {
            if (Lguess->GetElement(id, 0) > +LCP_EPS) {
                try_step = -L->GetElement(id, 0) / (Lguess->GetElement(id, 0) - L->GetElement(id, 0));
                if (try_step < step) {
                    step = try_step;
                    ichanged = id;
                }
            }
        }
    }

    // Check limit caused by some constraint which must be turned ON
    // because not-clamped constraints must always keep D>0

    for (id = 0; id < n_unilaterals; id++) {
        if (constr_data[id].status == CONSTR_UNILATERAL_OFF) {
            if (Dguess->GetElement(id, 0) < -LCP_EPS) {
                try_step = -D->GetElement(id, 0) / (Dguess->GetElement(id, 0) - D->GetElement(id, 0));
                if (try_step < step) {
                    step = try_step;
                    ichanged = id;
                }
            }
        }
    }

    if ((step <= 0) || (step > 1.0000001)) {
        // R3Error("  Warning, overconstrained LCP,  Step %g is not in 0..1. Changed constr.%d on %d",
        //							step, ichanged, n_unilaterals);	//***DEBUG***
        step = 0;
        ichanged = idirection;
        constr_data[ichanged].status = CONSTR_UNILATERAL_REDUNDANT;
    }

    if (ichanged == -1)
        return false;  // no limiting is required in this direction, keep step = 1;

    return true;
}

#define UNOPTIMIZED_LCP 1

int ChLinkedListMatrix::SolveLCP(ChMatrix<>* B,
                             ChMatrix<>* X,
                             int n_bilaterals,
                             int n_unilaterals,
                             int maxiters,
                             bool keep_unilateral_status,
                             ChUnilateralData constr_data[]) {
    // Check some low-level error in passing function parameters

    assert(B->GetRows() == this->rows);
    assert(X->GetRows() == this->rows);
    assert(this->rows == this->columns);

    int i_C = rows - n_unilaterals - n_bilaterals;
    int i_D = rows - n_unilaterals;

    // In case no unilateral constraints are provided, this can be readily solved as
    // a simple linear problem:

    if (n_unilaterals == 0) {
        double mdet;

        DecomposeAndSolve_LDL(B, X, mdet);

        return TRUE;
    }

    //-------------------------------------------------------------
    // Dantzig revisited algorithm
    //

    // Initialize temporary structures

    int own_constr_data = FALSE;
    int success = FALSE;

    int* pivarray = (int*)calloc((this->rows), sizeof(int));

    if (constr_data == NULL) {
        constr_data = (ChUnilateralData*)calloc((n_unilaterals), sizeof(ChUnilateralData));
        own_constr_data = TRUE;
        for (int ind = 0; ind < n_unilaterals; ind++) {
            constr_data[ind].Reset();
        }
    }

    ChMatrixDynamic<> D(n_unilaterals, 1);       // for storing residuals
    ChMatrixDynamic<> L(n_unilaterals, 1);       // for storing multipliers
    ChMatrixDynamic<> Dguess(n_unilaterals, 1);  // for storing guess residuals
    ChMatrixDynamic<> Lguess(n_unilaterals, 1);  // for storing guess multipliers
    ChMatrixDynamic<> Xguess(X->GetRows(), 1);   // for storing guess solution

    ChLinkedListMatrix Mbackup(this->rows, this->columns);  // for storing partial decomposition

    // initialize status of unilateral constraints (all constraints are unclamped).
    if (!keep_unilateral_status)
        for (int ind = 0; ind < n_unilaterals; ind++) {
            constr_data[ind].status = CONSTR_UNILATERAL_OFF;  // CONSTR_UNILATERAL_NONE;
        }

    // - compute unknowns (q, fd, etc.) using the LDL sparse solver ++++++
    //   for no unilateral constraint clamped...
    double mdet;

#ifdef UNOPTIMIZED_LCP
    DecomposeAndSolve_LDL_forLCP(B, X, mdet, i_D, i_C, n_unilaterals, constr_data);
#else
    ChLinkedListMatrix::DecomposeAndSolve_LDL_forLCP(this, &Mbackup, B, X, mdet, i_D, i_C, n_unilaterals, constr_data, 0,
                                                 i_D);
#endif

    //
    // The outer main loop
    //

    int tot_iters = 0;

    if (!keep_unilateral_status)
        while (true) {
            // - compute D and L residuals

            LCP_compute_DL_residuals(this, B, X, &D, &L, i_D, i_C, n_unilaterals, constr_data);

            // - find a residual to drive-to-zero in the main outer loop, if any

            int id;
            bool must_drive = false;

            for (id = 0; id < n_unilaterals; id++) {
                if ((constr_data[id].status != CONSTR_UNILATERAL_ON) &&
                    (constr_data[id].status != CONSTR_UNILATERAL_REDUNDANT))
                    if (D.GetElement(id, 0) < -LCP_EPS) {
                        must_drive = true;
                        break;  // will drive the residual n. 'id'.
                    }
            }

            if (must_drive == false) {
                success = TRUE;
                break;  // -> exit: all D residuals are at least >=0;
            }

            // - Now perform the inner loop, to drive-to-zero the 'id' residual:

            constr_data[id].status = CONSTR_UNILATERAL_ON;

            while (true) {
// Guess a destination, hoping that solution is reached without
// clamping or unclamping any constraint (if not, will repeat from rewound position..)
#ifdef UNOPTIMIZED_LCP
                DecomposeAndSolve_LDL_forLCP(B, &Xguess, mdet, i_D, i_C, n_unilaterals, constr_data);
#else
                ChLinkedListMatrix::DecomposeAndSolve_LDL_forLCP(this, &Mbackup, B, &Xguess, mdet, i_D, i_C, n_unilaterals,
                                                             constr_data, i_D, i_D);
#endif

                // D and L variables for the guess..

                LCP_compute_DL_residuals(this, B, &Xguess, &Dguess, &Lguess, i_D, i_C, n_unilaterals, constr_data);

                // Find the 'most limiting' inequality from path to guessed solution:

                double step;
                int ichanged;
                if (!LCP_compute_maxstep(&D, &L, &Dguess, &Lguess, id, constr_data, n_unilaterals, ichanged, step)) {
                    // move the D and L and X values to end of step (for lucky step=1)
                    X->CopyFromMatrix(Xguess);
                    D.CopyFromMatrix(Dguess);
                    L.CopyFromMatrix(Lguess);
                    break;
                }

                // move the D and L and X values to inbetween point of step (for step 0..1)
                X->LinInterpolate(*X, Xguess, step);
                D.LinInterpolate(D, Dguess, step);
                L.LinInterpolate(L, Lguess, step);

                if (constr_data[ichanged].status == CONSTR_UNILATERAL_ON) {
                    constr_data[ichanged].status = CONSTR_UNILATERAL_OFF;
                } else if (constr_data[ichanged].status == CONSTR_UNILATERAL_OFF) {
                    constr_data[ichanged].status = CONSTR_UNILATERAL_ON;
                }

                tot_iters++;
                if (maxiters)
                    if (tot_iters > maxiters)
                        break;  // -> exit without success, too many iterations;
            }

            tot_iters++;
            if (maxiters)
                if (tot_iters > maxiters)
                    break;  // -> exit without success, too many iterations;
        }

    // compute final feasability

    LCP_compute_DL_residuals(this, B, X, &Dguess, &Lguess, i_D, i_C, n_unilaterals, constr_data);
    double minD = Dguess.Min();  // test
    double maxD = Dguess.Max();  // test
    double minL = Lguess.Min();  // test
    double maxL = Lguess.Max();  // test
    double feas_error = 0.;
    for (int mc = 0; mc < Dguess.GetRows(); mc++) {
        if (fabs(Dguess.GetElement(mc, 0) * Lguess.GetElement(mc, 0)) > feas_error)
            feas_error = fabs(Dguess.GetElement(mc, 0) * Lguess.GetElement(mc, 0));
    }

    /*
    if (!success)
        R3Error("Too many iters, feas_error %g, Dn=%d, iter=%d, Dmin=%g, Dmax=%g, Lmin=%g, Lmax=%g",
                        feas_error, n_unilaterals, tot_iters, minD, maxD, minL, maxL);
    */

    free(pivarray);

    if (own_constr_data)
        free(constr_data);

    return success;
}

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


}  // END_OF_NAMESPACE____

// END
