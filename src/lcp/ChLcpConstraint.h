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

#ifndef CHLCPCONSTRAINT_H
#define CHLCPCONSTRAINT_H

//////////////////////////////////////////////////
//
//   ChLcpConstraint.h
//
//    Base class for representing a constraint for
//   sparse variational problems (VI/CCP/LCP/linear problems)
//   including inequalities, equalities, nonlinearities,
//   etc.
//
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChApiCE.h"
#include "core/ChMatrix.h"
#include "core/ChSpmatrix.h"
#include "core/ChClassRegister.h"


namespace chrono
{

// forward reference
class ChSparseMatrix;

/// Modes for constraint
enum eChConstraintMode {
		CONSTRAINT_FREE			= 0, ///< the constraint does not enforce anything
		CONSTRAINT_LOCK		    = 1, ///< the constraint enforces c_i=0;
		CONSTRAINT_UNILATERAL	= 2, ///< the constraint enforces linear complementarity c_i>=0, l_i>=0, l_1*c_i=0;
		CONSTRAINT_FRIC			= 3, ///< the constraint is one of three reactions in friction (cone complementarity problem)
};


///  Base class for representing constraints to be used
/// with variational inequality solvers, used with Linear/CCP/LCP
/// problems including inequalities, equalities, nonlinearities, etc.
/// The matrices define the variational inequality VI(Z*x-d,K):
///
///  | M -Cq'|*|q|- | f|= |0| , l \in Y, C \in Ny, normal cone to Y  
///  | Cq -E | |l|  |-b|  |c|    
///
/// Also Z symmetric by flipping sign of l_i: |M  Cq'|*| q|-| f|=|0|  
///                                           |Cq  E | |-l| |-b| |c|
/// * case linear problem:  all Y_i = R, Ny=0, ex. all bilaterals)
/// * case LCP: all Y_i = R+:  c>=0, l>=0, l*c=0)
/// * case CCP: Y_i are friction cones)
///  The jacobian matrix [Cq] is built row by row by jacobians
/// [Cq_i] of the constraints. [E] optionally includes 'cfm_i' terms 
/// on the diagonal.
///  In general, typical bilateral constraints must be solved
/// to have residual the residual c_i = 0 (unilaterals: c_i>0)
/// where the following linearization is introduced:
///      c_i= [Cq_i]*q + b_i
///
/// The base class introduces just the minimum requirements
/// for the solver, that is the basic methods which will be
/// called by the solver. It is up to the derived classes
/// to implement these methods, and to add further features..

class ChApi ChLcpConstraint
{
	CH_RTTI_ROOT(ChLcpConstraint)

protected:
			//
			// DATA
			//

				/// The 'c_i' residual of the constraint (if satisfied, c must be =0)
	double c_i;
				/// The 'l_i' lagrangian multiplier (reaction)
	double l_i;
				/// The 'b_i' right term in [Cq_i]*q+b_i=0 , note: c_i= [Cq_i]*q + b_i 
	double b_i;
				/// The constraint force mixing, if needed (usually is zero) to add some
				/// numerical 'compliance' in the constraint, that is the equation becomes:
				/// c_i= [Cq_i]*q + b_i + cfm*l_i =0;   
				/// Example, it could be   cfm = [k * h^2](^-1)   where k is stiffness
	double cfm_i;


			// FLAGS
private:
				/// Flag: the link has no formal problems (references restored correctly, etc)
	bool valid;
				/// Flag: the user can turn on/off the link easily
	bool disabled;
				/// Flag: the constraint is redundant or singular
	bool redundant;
				/// Flag: the constraint is broken (someone pulled too much..)
	bool broken;
				/// Cached active state depending on previous flags. Internal update.
	bool _active; 


protected:
				/// The mode of the constraint: free / lock / complementar
	eChConstraintMode mode;

			// Auxiliary data, (ex. for iterative solvers):

				/// The 'g_i' product [Cq_i]*[invM_i]*[Cq_i]' (+cfm)
	double g_i;

				/// offset in global "l" state vector (needed by some solvers)
	int offset;

public:

			//
			// CONSTRUCTORS
			//
	ChLcpConstraint()
					{
						c_i=0; g_i=0; b_i=0; l_i = 0; cfm_i=0;
						valid = false;
						disabled = false;
						redundant = false;
						broken = false;
						_active = true;
						mode = CONSTRAINT_LOCK;
					};

						/// Copy constructor
	ChLcpConstraint(const ChLcpConstraint& other)
					{
						c_i=other.c_i;
						g_i=other.g_i;
						b_i=other.b_i;
						l_i=other.l_i;
						cfm_i= other.cfm_i;
						valid = other.valid;
						disabled = other.disabled;
						redundant = other.redundant;
						broken = other.broken;
						mode = other.mode;
					}

	virtual ~ChLcpConstraint()
					{
					};

	virtual ChLcpConstraint* new_Duplicate () =0;


					/// Assignment operator: copy from other object
	ChLcpConstraint& operator=(const ChLcpConstraint& other);

					/// Comparison (compares anly flags, not the jacobians etc.)
	bool operator==(const ChLcpConstraint& other) const;


			//
			// FUNCTIONS
			//

				/// Tells if the constraint data is currently valid.
	virtual bool IsValid() const {return valid;}
				/// Use this function to set the valid state (child class 
				/// Children classes must use this function depending on 
				/// the result of their implementations of RestoreReference();
	virtual void SetValid(bool mon) {valid = mon; UpdateActiveFlag();}

				/// Tells if the constraint is currently turned on or off by the user.
	virtual bool IsDisabled() const { return disabled; }
				/// User can use this to enable/disable the constraint as desired
	virtual void SetDisabled(bool mon) {disabled = mon; UpdateActiveFlag();}

				/// Tells if the constraint is redundant or singular.
	virtual bool IsRedundant() const { return redundant; }
				/// Solvers may use the following to mark a constraint as redundant
	virtual void SetRedundant(bool mon) {redundant = mon; UpdateActiveFlag();}

				/// Tells if the constraint is broken, for eccess of pulling/pushing.
	virtual bool IsBroken() const { return broken; }
				/// 3rd party software can set the 'broken' status via this method
				/// (by default, constraints never break);
	virtual void SetBroken(bool mon) {broken = mon; UpdateActiveFlag();}

				/// Tells if the constraint is unilateral (typical complementarity constraint).
	virtual bool IsUnilateral()  const { return mode == CONSTRAINT_UNILATERAL; }

				/// Tells if the constraint is linear (if non linear, returns false).
	virtual bool IsLinear() const { return true; }

				/// Gets the mode of the constraint: free / lock / complementary
				/// A typical constraint has 'lock = true' by default.
	eChConstraintMode GetMode() const { return mode; }

				/// Sets the mode of the constraint: free / lock / complementary
	void SetMode(eChConstraintMode mmode) {mode = mmode; UpdateActiveFlag();}


				/// A VERY IMPORTANT function!
				/// Tells if the constraint is currently active, in general,
				/// that is tells if it must be included into the system solver or not.
				/// This method cumulates the effect of all flags (so a constraint may
				/// be not active either because 'disabled', or 'broken', o 'redundant', or not 'valid'.)
	virtual bool IsActive() const
					{
						/*
						return ( valid &&
								!disabled &&
								!redundant &&
								!broken &&
								mode!=(CONSTRAINT_FREE));
								*/ // Optimized: booleans precomputed and cached in _active.
						return _active;
					}


				/// Compute the residual of the constraint using the LINEAR 
				/// expression   c_i= [Cq_i]*q + cfm_i*l_i + b_i . For a satisfied bilateral
				/// constraint, this residual must be near zero.
	virtual double Compute_c_i() { c_i = Compute_Cq_q() + cfm_i*l_i + b_i; return c_i;};


				/// Return the residual 'c_i' of this constraint. // CURRENTLY NOT USED
	double Get_c_i() const { return c_i; }


				/// Sets the known term b_i in [Cq_i]*q + b_i =0,
				/// where: c_i = [Cq_i]*q + b_i = 0
	void Set_b_i(const double mb) {b_i = mb;}

				/// Return the known term b_i in [Cq_i]*q + b_i =0,
				/// where: c_i= [Cq_i]*q + b_i = 0
	double Get_b_i() const { return b_i; }


				/// Sets the constraint force mixing term (default=0).
				/// Adds artificial 'elasticity' to the constraint,
				/// as:   c_i= [Cq_i]*q + b_i + cfm*l_i =0;
	void Set_cfm_i(const double mcfm) {cfm_i = mcfm;}

				/// Returns the constraint force mixing term.
	double Get_cfm_i() const { return cfm_i; }


				/// Sets the 'l_i' value (constraint reaction, see 'l' vector)
	virtual void Set_l_i(double ml_i) { l_i = ml_i ;}

				/// Return the 'l_i' value (constraint reaction, see 'l' vector)
	virtual double Get_l_i() const { return l_i; }



				// -----  Functions often used by iterative solvers:

				///  This function must update jacobians and auxiliary
				/// data such as the 'g_i' product. This function is
				/// often called by LCP solvers at the beginning of the
				/// solution process.
				/// *** This function MUST BE OVERRIDDEN by specialized
				/// inherited classes, which have some jacobians!
	virtual void Update_auxiliary() { /* do nothing */ };


				/// Return the 'g_i' product , that is [Cq_i]*[invM_i]*[Cq_i]' (+cfm)
	double Get_g_i() const { return g_i; }

				/// Usually you should not use the Set_g_i function, because g_i
				/// should be automatically computed during the Update_auxiliary() .
	void Set_g_i(double m_g_i) {g_i = m_g_i;}


				///  This function must computes the product between
				/// the row-jacobian of this constraint '[Cq_i]' and the
				/// vector of variables, 'q', that is, Cq_q=[Cq_i]*q.
				///  This is used for some iterative LCP solvers.
				/// *** This function MUST BE OVERRIDDEN by specialized
				/// inherited classes! (since it will be called frequently,
				/// when iterative solvers are used, the implementation of 
				/// the [Cq_i]*q product must be AS FAST AS POSSIBLE!).
				///  It returns the result of the computation.
	virtual double Compute_Cq_q() = 0;  


				///  This function must increment the vector of variables
				/// 'q' with the quantity [invM]*[Cq_i]'*deltal,that is
				///   q+=[invM]*[Cq_i]'*deltal
				///  This is used for some iterative LCP solvers.
				/// *** This function MUST BE OVERRIDDEN by specialized
				/// inherited classes!
	virtual void Increment_q(const double deltal) = 0;

				/// Computes the product of the corresponding block in the 
				/// system matrix by 'vect', and add to 'result'. 
				/// NOTE: the 'vect' vector must already have
				/// the size of the total variables&constraints in the system; the procedure
				/// will use the ChVariable offsets (that must be already updated) to know the 
				/// indexes in result and vect; 
	virtual void MultiplyAndAdd(double& result, const ChMatrix<double>& vect) const = 0;

				/// Computes the product of the corresponding transposed block in the 
				/// system matrix (ie. the TRANSPOSED jacobian matrix C_q') by 'l', and add to 'result'. 
				/// NOTE: the 'result' vectors must already have
				/// the size of the total variables&constraints in the system; the procedure
				/// will use the ChVariable offsets (that must be already updated) to know the 
				/// indexes in result and vect; 
	virtual void MultiplyTandAdd(ChMatrix<double>& result, double l) = 0;

				/// For iterative solvers: project the value of a possible
				/// 'l_i' value of constraint reaction onto admissible orthant/set.
				/// Default behavior: if constraint is unilateral and l_i<0, reset l_i=0
				/// *** This function MAY BE OVERRIDDEN by specialized
				/// inherited classes! For example, a bilateral constraint
				/// can do nothing, a monolateral: l_i= ChMax(0., l_i);
				/// a 'boxed constraint': l_i= ChMin(ChMax(min., l_i), max); etc. etc.
	virtual void Project();

				/// Given the residual of the constraint computed as the
				/// linear map  mc_i =  [Cq]*q + b_i + cfm*l_i , returns the
				/// violation of the constraint, considering inequalities, etc.
				///   For bilateral constraint,  violation = mc_i.
				///   For unilateral constraint, violation = min(mc_i, 0),
				///   For boxed constraints or such, inherited class MAY OVERRIDE THIS!
	virtual double Violation(double mc_i);

				/// Puts the jacobian portions into the 'insrow' row of a sparse matrix,
				/// where each portion of jacobian is shifted in order to match the 
				/// offset of the corresponding ChLcpVariable. 
				/// This is used only by the ChLcpSimplex solver (iterative solvers 
				/// don't need to know jacobians explicitly)
				/// *** This function MUST BE OVERRIDDEN by specialized
				/// inherited classes!
	virtual void Build_Cq(ChSparseMatrix& storage, int insrow) =0;

				/// Same as Build_Cq, but puts the _transposed_ jacobian row as a column.
				/// *** This function MUST BE OVERRIDDEN by specialized
				/// inherited classes!
	virtual void Build_CqT(ChSparseMatrix& storage, int inscol) =0;

				/// Set offset in global q vector (set automatically by ChLcpSystemDescriptor)
	void SetOffset(int moff) {offset = moff;}

				/// Get offset in global q vector 
	int GetOffset() const {return offset;}

			//
			// STREAMING
			//

					/// Method to allow deserializing a persistent binary archive (ex: a file)
					/// into transient data.
	virtual void StreamIN(ChStreamInBinary& mstream);

					/// Method to allow serializing transient data into a persistent
					/// binary archive (ex: a file).
	virtual void StreamOUT(ChStreamOutBinary& mstream);


private: 
	void UpdateActiveFlag()
				{
					this->_active = ( valid &&
								!disabled &&
								!redundant &&
								!broken &&
								mode!=(CONSTRAINT_FREE));
				}
};





} // END_OF_NAMESPACE____






#endif  // END of ChLcpConstraint.h
