//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLINKMASK_H
#define CHLINKMASK_H

///////////////////////////////////////////////////
//
//   ChLinkMask.h
//
//
//   Array of ChConstraint() objects, used as a
//   collection of scalar constraints in ChLink()
//   objects.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <math.h>
#include <vector>

#include "core/ChMath.h"
#include "lcp/ChLcpConstraintTwoBodies.h"


namespace chrono
{



///
/// Mask structure for N scalar constraint equations between two bodies.
///

class ChApi ChLinkMask {

	CH_RTTI_ROOT(ChLinkMask);

			//
			// DATA
			//

protected:

					// Array of pointers to 'n' scalar constraint states (own by this object)
	std::vector<ChLcpConstraintTwoBodies*> constraints;


public:


	int nconstr;	// Number of scalar eq.of constraint.
					// Maybe different from effective nDOC because someone may be free/redundant/etc.

			//
			// CONSTRUCTORS
			//

				/// Build a link mask with a single constraint
				/// of class ChLcpConstraintTwoBodies().
	ChLinkMask();

				/// Build a link mask with a default array of mnconstr constraints
				/// of class ChLcpConstraintTwoBodies().
	ChLinkMask(int mnconstr);

	virtual ~ChLinkMask();

	ChLinkMask (ChLinkMask& source);
	virtual void Copy(ChLinkMask* source);
	virtual ChLinkMask* NewDuplicate();



			//
			// FUNCTIONS
			//

				/// Set references to variables of two connected bodies to all
				/// constraints at once, therefore also sets all the constraints as active.
	void SetTwoBodiesVariables(ChLcpVariables* var1, ChLcpVariables* var2);

				/// Obtain the reference to the i-th scalar constraint data
				/// in the collection link mask.
	ChLcpConstraintTwoBodies& Constr_N(int i) {assert((i>=0)&&(i<nconstr)); return *constraints[i];}

				/// Utility: to change the size of the mask, reallocating constraints
				/// (all of type ChConstraintTwo).
				/// No action if newnconstr == nconstr
	void ResetNconstr(int newnconstr);	

				/// Add a ChLcpConstraintTwoBodies to mask (NOTE: later, the constraint will
				/// be automatically deleted when the mask will be deleted)
	void AddConstraint(ChLcpConstraintTwoBodies* aconstr);

				/// To compare two masks, return TRUE if equal
	int IsEqual(ChLinkMask& mask2);  

				/// Tells if i-th equation is a unilateral constraint
	bool IsUnilateral(int i);  

				// Get the number of removed degrees of freedom (n.of costraints)

				/// Count both bilaterals and unilaterals
	int GetMaskDoc();    
				/// Count only bilaterals
	int GetMaskDoc_c();	 
				/// Count only unilaterals
	int GetMaskDoc_d(); 

				/// Get the i-th active scalar costraint (not active constr. won't be considered)
	ChLcpConstraintTwoBodies* GetActiveConstrByNum(int mnum);

				/// Sets some active constraints as redundant.
	int SetActiveRedundantByArray(int* mvector, int mcount);  


				/// Set lock =ON for costraints which were disabled because redundant
	int RestoreRedundant(); 


				/// If SetAllDisabled(true), all the constraints are temporarily turned
				/// off (inactive) at once, because marked as 'disabled'. Return n.of changed
	int SetAllDisabled(bool mdis);

				/// If SetAllBroken(true), all the constraints are temporarily turned
				/// off (broken) at once, because marked as 'broken'. Return n.of changed.
	int SetAllBroken(bool mdis);



			//
			// STREAMING
			//

					/// Method to allow deserializing a persistent binary archive (ex: a file)
					/// into transient data.
	virtual void StreamIN(ChStreamInBinary& mstream);

					/// Method to allow serializing transient data into a persistent
					/// binary archive (ex: a file).
	virtual void StreamOUT(ChStreamOutBinary& mstream);


};






///
/// Specialized ChLinkMask class, for constraint equations of
/// the ChLinkLock link.
///

class ChApi ChLinkMaskLF : public  ChLinkMask
{
	CH_RTTI(ChLinkMaskLF, ChLinkMask);

public:

			/// Create a ChLinkMaskLF which has 9 scalar constraints of
			/// class ChLcpConstraintTwoBodies(). This is useful in case it must
			/// be used for the ChLinkLock link.
	ChLinkMaskLF();

	void Copy(ChLinkMaskLF* source);
	ChLinkMask* NewDuplicate();



		/// set all mask data at once
	void SetLockMask(int x, int y, int z, int e0, int e1, int e2, int e3, int p, int d);


		/// Obtain the reference to specific scalar constraint data
		/// in the collection of this link mask.
	ChLcpConstraintTwoBodies& Constr_X()  {return *constraints[0];};
	ChLcpConstraintTwoBodies& Constr_Y()  {return *constraints[1];};
	ChLcpConstraintTwoBodies& Constr_Z()  {return *constraints[2];};
	ChLcpConstraintTwoBodies& Constr_E0() {return *constraints[3];};
	ChLcpConstraintTwoBodies& Constr_E1() {return *constraints[4];};
	ChLcpConstraintTwoBodies& Constr_E2() {return *constraints[5];};
	ChLcpConstraintTwoBodies& Constr_E3() {return *constraints[6];};
	ChLcpConstraintTwoBodies& Constr_P()  {return *constraints[7];};
	ChLcpConstraintTwoBodies& Constr_D()  {return *constraints[8];};



		// If =1, free costraints not used for bidimensional simulation (Z movement and
	    // Qx,Qy rotations (it is assumed that both markers have Z axis orthogonl to XY abs. plane)
		// If =0, return to the 3d mode.
	int Set2Dmode(int mode);


			//
			// STREAMING
			//

					/// Method to allow deserializing a persistent binary archive (ex: a file)
					/// into transient data.
	virtual void StreamIN(ChStreamInBinary& mstream);

					/// Method to allow serializing transient data into a persistent
					/// binary archive (ex: a file).
	virtual void StreamOUT(ChStreamOutBinary& mstream);
};



} // END_OF_NAMESPACE____

#endif
