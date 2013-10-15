//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHSPMATRIX_H
#define CHSPMATRIX_H

//////////////////////////////////////////////////
//  
//   ChSpmatrix.h
//
//   Math functions for :
//      - SPARSE MATRICES
//      - SPARSE DIAGONAL MATRICES
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChMatrix.h"
#include "core/ChLists.h"
#include "core/ChApiCE.h"

namespace chrono
{

 // default predicted density, for allocation, 0<..<1 , default 1/10 th
#define SPM_DEF_FULLNESS 0.1
 // default limit on initial number of extradiagonal elements, for allocation, default 10000
#define SPM_DEF_MAXELEMENTS 10000

///////////////////////////////////////////
// 
/// Generic element of a sparse matrix ChSparseMatrix
///

class ChApi ChMelement 
{
public:
	double val;
	ChMelement* next;
	ChMelement* prev;
	int row;
	int col;

	ChMelement(double val, ChMelement* mnext, ChMelement* mprev, int row, int col);
	void Initialize(double val, ChMelement* mnext, ChMelement* mprev, int row, int col);
};

inline ChMelement::ChMelement (double mval, ChMelement* mnext, ChMelement* mprev, int mrow, int mcol)
{
	val = mval;
	next = mnext;
	prev = mprev;
	col = mcol;
	row = mrow;
}

inline void ChMelement::Initialize (double mval, ChMelement* mnext, ChMelement* mprev, int mrow, int mcol)
{
	val = mval;
	next = mnext;
	prev = mprev;
	col = mcol;
	row = mrow;
}

//***TO DO*** move into ChLcpSimplexSolver

#define LCP_MAX_ITERS             200
#define CONSTR_UNILATERAL_NONE		0
#define CONSTR_UNILATERAL_REDUNDANT 1
#define CONSTR_UNILATERAL_OFF		2
#define CONSTR_UNILATERAL_ON	  102


class ChUnilateralData {
public:
	void Reset()	{
			//status= CONSTR_UNILATERAL_NONE;
			status= CONSTR_UNILATERAL_OFF;
			//friction = 0.;
	}

	char	status;
	//double	friction;
};


//////////////////////////////////////////////////
// SPARSE MATRIX CLASS
//
/// This class defines a sparse matrix, using the method of 
/// linked lists of non-structural-zeros per each row.
///
/// Note that this class is inherited from the ChMatrix class, 
/// however, since ChMatrix has no 'virtual' members for speed
/// reasons (especially, the Set/GetElement fundamental methods
/// are not virtual!) this means that MOST ChMatrix methods
/// must be overridden and re-implemented, even if you 
/// would stick with the implementation in the base class!
/// (here, only few of them are reimplemented, more will
/// come later in futire releases.).
/// 


class ChApi ChSparseMatrix : public ChMatrix<double> {
 private:
	ChMelement** elarray;	// array of 1st column elements 

	ChList<ChMelement> mbufferlist;		// list of pointers to Melement* buffers
	ChMelement* mnextel;				// address of next writable element in mbufferlist 

	int mtot_size;		// total allocated space for elements (maybe in noncontiguous buffers list) 
	int mbuffer_size;	// size of currently used buffer
	int mbuffer_added;	// mbuffer_added grows all time a new element is set non-zero,
						// until it reaches mbuffer_size (if so, allocation of another buffer is needed). Handled internally.

	ChMelement* NewElement (double mval, ChMelement* mnext, ChMelement* mprev, int mrow, int mcol); // used internally
	ChMelement** GetElarray () { return elarray; };
	ChMelement* GetElarrayMel(int num) {return (*(elarray+num));}
	double GetElarrayN (int num) { return (*(elarray+num))->val; };
	void SetElarrayN (double val, int num) { (*(elarray+num))->val = val;};
 public:

		// 
		// CONSTRUCTORS
		//

					/// Creates a sparse matrix with given size.
					/// fullness = predicted initial "density" of matrix,
					/// between 0..1, to have best buffer handling and
					/// memory allocations.
	ChSparseMatrix (int row, int col, double fullness);	 

					/// Creates a sparse matrix with given size.
					/// and default 'predicted' fullness as SPM_DEF_FULLNESS
	ChSparseMatrix (int row, int col);

					/// Creates a default 3x3 (a bit unuseful, use 
					/// ChSparseMatrix for _large_ matrices! :)
	ChSparseMatrix ();	
	~ChSparseMatrix ();

	void Build(int row, int col, double fullness);		///< [mostly used internally]



	void CopyFromMatrix (ChMatrix<>* matra);
	void CopyFromMatrix (ChSparseMatrix* matra);
	void CopyToMatrix	(ChMatrix<>* matra);

					// optimized SetElement,  returning the fetched Melement*
	ChMelement* SetElement (int row, int col, double val, ChMelement* guess);
					// optimized GetElement,  returning the fetched Melement*
	ChMelement* GetElement (int row, int col, double* val, ChMelement* guess);

					// Add another buffer in buffer list, bigger of last 
					// (max total buffer size = col x rows) as in newbuffer = lastbuffer * inflate  
					// (use inflate >1, suggested 2)     [mostly used internally]
	void MoreBuffer(double inflate);			


		// Overloadings of standard matrix functions:
	void Resize(int nrows, int ncols) {assert (false);}; // not implemented

	void Reset();						// reset to null matrix 
	void Reset(int row, int col);		// reset to null matrix and (if needed) changes the size.
	void ResetBlocks(int row, int col);	// if size changes, is like the above, otherwise just sets to zero the elements .
	
	void   SetElement ( int row, int col, double elem);
	double GetElement ( int row, int col);

	void SwapColumns (int a, int b);
	void SwapRows    (int a, int b);


		// Customized functions, speed-optimized for sparse matrices:

	void PasteMatrix (ChMatrix<>* matra, int insrow, int inscol);
	void PasteTranspMatrix (ChMatrix<>* matra, int insrow, int inscol);
	void PasteMatrixFloat (ChMatrix<float>* matra, int insrow, int inscol);
	void PasteTranspMatrixFloat (ChMatrix<float>* matra, int insrow, int inscol);
	void PasteMatrix (ChSparseMatrix* matra, int insrow, int inscol);
	void PasteTranspMatrix (ChSparseMatrix* matra, int insrow, int inscol);
	void PasteClippedMatrix (ChMatrix<>* matra, int cliprow, int clipcol, int nrows, int ncolumns, int insrow, int inscol);
	void PasteSumClippedMatrix (ChMatrix<>* matra, int cliprow, int clipcol, int nrows, int ncolumns, int insrow, int inscol);
	void PasteSumMatrix (ChMatrix<>* matra, int insrow, int inscol);
	void PasteSumTranspMatrix (ChMatrix<>* matra, int insrow, int inscol);

	void MatrMultiply ( ChSparseMatrix* matra, ChSparseMatrix* matrb);
	void MatrMultiplyT ( ChSparseMatrix* matra, ChSparseMatrix* matrb);
	void MatrTMultiply ( ChSparseMatrix* matra, ChSparseMatrix* matrb);
	void MatrAdd	( ChSparseMatrix* matra, ChSparseMatrix* matrb);
	void MatrSub	( ChSparseMatrix* matra, ChSparseMatrix* matrb);
	void MatrInc	( ChSparseMatrix* matra);
	void MatrScale	( double factor);
	void MatrTranspose();
	void Neg();

	// Linear algebra functions

					// Basic Gauss solver 
	int  BestPivotRow (int current);
	int  BestPivotDiag (int current);
	void DiagPivotSymmetric(int rowA, int rowB);
	int  Solve_LinSys ( ChMatrix<>* B, ChMatrix<>* X, int* pivarray, double* det);
	void Solve_LinSys ( ChMatrix<>* B, ChMatrix<>* X); // the object is the [A] matrix.

					// LU decomposition, in place. (matrix [A] is overwritten) 
					// Pivot array must exist! (it will be filled with the _row_ pivots, if any)
	int	 Decompose_LU(int* pivarray, double* det);
	int	 Solve_LU(ChMatrix<>* B, ChMatrix<>* X, int* pivarray);	
	int  Extract_LU(ChMatrix<>* L, ChMatrix<>* U);	// From this matrix, computed 'in place' with Decompose_LU(), fills the separate L and U 
							
					/// LDL decomposition, only for symmetric matrices [A]!! Note: pivarray is for 
					/// both row and col swap (full diagonal pivot)! Note: only upper part of [A] is used.
					/// The decomposition can be made to start from the 'from_eq' equation, if matrix was already decomposed and only 
					/// a basis has been modified under the right-lower part at 'from_eq, from_eq' cell (ex: in LCP solver)
	int	 Decompose_LDL(int* pivarray, double* det, int from_eq = 0);	
	int	 Solve_LDL(ChMatrix<>* B, ChMatrix<>* X, int* pivarray);
	int  Extract_LDL(ChMatrix<>* L, ChMatrix<>* D, ChMatrix<>* Lu);
	int	 DecomposeAndSolve_LDL(ChMatrix<>* B, ChMatrix<>* X, double& mdet, int from_eq = 0);
				
	int	 DecomposeAndSolve_LDL_forLCP (ChMatrix<>* B, ChMatrix<>* X, double& mdet, 
											   int i_D, int i_C, int n_unilaterals,
											   ChUnilateralData constr_data[], int from_eq = 0);
	static int DecomposeAndSolve_LDL_forLCP
											  (ChSparseMatrix* Afact,
											   ChSparseMatrix* Aorig,
												ChMatrix<>* B, ChMatrix<>* X, double& mdet, 
											   int i_D, int i_C, int n_unilaterals, 
											   ChUnilateralData constr_data[], int from_eq,
											   int backup_from);

					// LCP solver for dynamic problems: the matrix must contain the mass matrix
					// on the upper left part, bordered with the jacobians of bilateral constraints C,
					// then bordered with 'n_unilaterals' unilateral constraints D: 
					//       this      *  X   =   B 
					//
					//   | M  C'  D' |   |q	|	|Qf |
					//   | C  0   0  | * |fc| = |Qc |
					//   | D  0   0  |   |fd|   |Qd |
					//
					// Only the upper part of the matrix is modified (the one with transpose jacobians C' and D'),
					// the lower is used to store the original jacobians.
	int  SolveLCP(ChMatrix<>* B, ChMatrix<>* X, 
						   int n_bilaterals, int n_unilaterals, 
						   int maxiters = 200,
						   bool keep_unilateral_status = false,
						   ChUnilateralData constr_data[] = NULL);



					/// Method to allow serializing transient data into in ascii stream (es a file)
					/// as a Matlab sparse matrix format ( each row in file has three elements: 
					///     row,    column,    value 
					/// Note: the row and column indexes start from 1, not 0 as in C language.
	void StreamOUTsparseMatlabFormat(ChStreamOutAscii& mstream);

};


// used internally:
inline ChMelement* ChSparseMatrix::NewElement (double mval, ChMelement* mnext, ChMelement* mprev, int mrow, int mcol)
{
	if (mbuffer_added >= mbuffer_size)
	{
		MoreBuffer(2.0);
	}

	ChMelement* newel = mnextel;
	mnextel->Initialize(mval,mnext,mprev,mrow,mcol);
	
	mbuffer_added++;
	mnextel++;

	return (newel);
}




} // END_OF_NAMESPACE____


#endif
