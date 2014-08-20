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

#ifndef CHLCPVARIABLES_H
#define CHLCPVARIABLES_H

//////////////////////////////////////////////////
//
//   ChLcpVariables.h
//
//    Base class for representing a mass matrix and
//   associate variables, that are building blocks 
//   for describing the system. 
//   Used for building sparse variational problems 
//   (VI/CCP/LCP/linear problems) described by 
//   a ChLcpSystemDescriptor
//
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChApiCE.h"
#include "core/ChMatrixDynamic.h"
#include "core/ChSpmatrix.h"
#include "core/ChMemory.h" // must be after system's include (memory leak debugger).


namespace chrono
{


///  Base class for representing LCP items which introduce
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



class ChApi ChLcpVariables
{
	CH_RTTI_ROOT(ChLcpVariables)

private:
			//
			// DATA
			//
				/// the variables (accelerations, speeds, etc. depending on the problem)
	ChMatrix<>* qb;
				/// the known vector (forces, or impulses, etc. depending on the problem)
	ChMatrix<>* fb;
				/// the number of degrees of freedom (number of contained scalar variables)
	int ndof;
				/// user activation/deactivation of variables
	bool disabled;

protected:
				/// offset in global q state vector (needed by some solvers)
	int offset;

public:

			//
			// CONSTRUCTORS
			//
	ChLcpVariables()
				{
					disabled = false;
					ndof = 0;
					qb = fb = NULL;
					offset=0;
				}

	ChLcpVariables(int m_ndof)
				{
					disabled = false;
					ndof = m_ndof;
					if (Get_ndof()>0) {
						qb = new ChMatrixDynamic<>(Get_ndof(),1);
						fb = new ChMatrixDynamic<>(Get_ndof(),1);
					} else {
						qb = fb = NULL;
					}
					offset=0;
				};

	virtual ~ChLcpVariables()
				{
					delete qb;
					delete fb;
				};


				/// Assignment operator: copy from other object
	ChLcpVariables& operator=(const ChLcpVariables& other);


			//
			// FUNCTIONS
			//


				/// Tells if the variables have been deactivated (these 'frozen',
				/// variables won't be modified by the LCP system solver).
	void SetDisabled(bool mdis) { disabled = mdis; }

				/// Tells if the variables have been deactivated (these 'frozen',
				/// variables won't be modified by the LCP system solver).
	bool IsDisabled() const {return disabled;}


				/// Tells if these variables are currently active, in general,
				/// that is tells if they must be included into the LCP system solver or not.
    bool IsActive() const { return !disabled; }


				/// The number of scalar variables in the vector qb
				/// (dof=degrees of freedom)
				/// *** This function MUST BE OVERRIDDEN by specialized
				/// inherited classes.
	virtual int Get_ndof() const {return ndof;};

				/// Returns reference to qb, body-relative part of degrees
				/// of freedom q in system:
				///    | M -Cq'|*|q|- | f|= |0| ,  c>0, l>0, l*r=0;
				///    | Cq  0 | |l|  |-b|  |c|
	ChMatrix<>& Get_qb() {return *qb;};

				/// Compute fb, body-relative part of known
				/// vector f in system.
				/// *** This function MAY BE OVERRIDDEN by specialized
				/// inherited classes (example, for impulsive multibody simulation,
				/// this may be fb=dt*Forces+[M]*previous_v ).
				///  Another option is to set values into fb vectors, accessing
				/// them by Get_fb() from an external procedure, for each body,
				/// before starting the LCP solver.
	virtual void Compute_fb() {/* do nothing */};

				/// Returns reference to fb, body-relative part of known
				/// vector f in system.
				///    | M -Cq'|*|q|- | f|= |0| ,  c>0, l>0, l*r=0;
				///    | Cq  0 | |l|  |-b|  |c|
				/// This function can be used to set values of fb vector
				/// before starting the LCP solver.
	ChMatrix<>& Get_fb() {return *fb;}

				/// Computes the product of the inverse mass matrix by a
				/// vector, and store in result: result = [invMb]*vect
				/// *** This function MUST BE OVERRIDDEN by specialized
				/// inherited classes
	virtual void Compute_invMb_v(ChMatrix<float>& result, const ChMatrix<float>& vect) const = 0;
	virtual void Compute_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const = 0;

				/// Computes the product of the inverse mass matrix by a
				/// vector, and increment result: result += [invMb]*vect
				/// *** This function MUST BE OVERRIDDEN by specialized
				/// inherited classes
	virtual void Compute_inc_invMb_v(ChMatrix<float>& result, const ChMatrix<float>& vect) const = 0;
    virtual void Compute_inc_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const = 0;

				/// Computes the product of the mass matrix by a
				/// vector, and increment result: result = [Mb]*vect
				/// *** This function MUST BE OVERRIDDEN by specialized
				/// inherited classes
    virtual void Compute_inc_Mb_v(ChMatrix<float>& result, const ChMatrix<float>& vect) const = 0;
    virtual void Compute_inc_Mb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const = 0;

				/// Computes the product of the corresponding block in the 
				/// system matrix (ie. the mass matrix) by 'vect', and add to 'result'. 
				/// NOTE: the 'vect' and 'result' vectors must already have
				/// the size of the total variables&constraints in the system; the procedure
				/// will use the ChVariable offset (that must be already updated) to know the 
				/// indexes in result and vect.
	virtual void MultiplyAndAdd(ChMatrix<double>& result, const ChMatrix<double>& vect) const = 0;

				/// Add the diagonal of the mass matrix (as a column vector) to 'result'.
				/// NOTE: the 'result' vector must already have the size of system unknowns, ie
				/// the size of the total variables&constraints in the system; the procedure
				/// will use the ChVariable offset (that must be already updated) as index.
	virtual void DiagonalAdd(ChMatrix<double>& result) const = 0;

				/// Build the mass submatrix (for these variables) storing
				/// it in 'storage' sparse matrix, at given column/row offset.
				/// Most iterative solvers don't need to know this matrix explicitly.
				/// *** This function MUST BE OVERRIDDEN by specialized
				/// inherited classes
	virtual void Build_M(ChSparseMatrix& storage, int insrow, int inscol) = 0;

				/// Set offset in global q vector (set automatically by ChLcpSystemDescriptor)
	void SetOffset(int moff) {offset = moff;}
				/// Get offset in global q vector 
	int GetOffset() const {return offset;}

};




} // END_OF_NAMESPACE____


#include "core/ChMemorynomgr.h" // back to default new/delete/malloc/calloc etc. Avoid conflicts with system libs.


#endif  // END of ChLcpVariables.h
