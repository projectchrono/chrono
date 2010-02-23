#ifndef CHCONSTRAINT_H
#define CHCONSTRAINT_H

//////////////////////////////////////////////////
//  
//   ChConstraints.h
//
//   Class for generic constraint, (NOT MEChANICAL)
//   ex. for constrained optimizations etc. 
// 
//   Note that the 'mechanical' constraints are 
//   defined in another header, i.e. ChLink.h,
//   and they are a different stuff.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////




#include <math.h>

#include "physics/ChRef.h"


namespace chrono 
{



/// Class for basic algebraic constraints (not to be confused with
/// ChLink objects, which are complex kinematical constraints between rigid
/// bodies in 3D, containing ChLcpConstraint objects)
///
/// This is the base data for algebraic constraints. 
/// The base implemetation is basically _useless_ unless it has some 
/// inherited implementation (see other classes below)
///
/// Child classes should implement at least Update() RestoreReferences() Get_Cn().


class ChConstraint {

 private:

 protected:
			// some flags
	bool valid;
	bool disabled;

			// constraints equations in this constraint
	int  Cn; 
			// residual matrix
	ChMatrix<>* C;

 public:

	ChConstraint();
	~ChConstraint();

		/// Tells if the constraint data is currently valid.
		/// Instead of implementing it, child classes may simply
		/// set valif=false (or true) depending on the result of their
	    /// implementations of RestoreReference();
	virtual bool IsValid() {return valid;}
		
		/// Tells if the constraint is currently turned on or off by the user.
	virtual bool IsDisabled() {return disabled;}
	virtual void SetDisabled(bool mon) {disabled = mon;}

		/// Tells if the constraint is currently active, in general, 
		/// that is tells if it must be included into the system solver or not.
	virtual bool IsActive() {return (valid & !disabled & (Cn>0));}


	
		/// Returns the matrix of residuals (a column vector with Cn elements)
		/// If constraint is not active, returns NULL because no equations can be used.
	virtual ChMatrix<>* Get_C() {if (IsActive()) return C; else return NULL;}


		/// ---TO IMPLEMENT-- w.overriding
		/// Returns the number of equations in this constraints (the 
		/// size of the C residual vector)
	virtual int Get_Cn() {return 0;}
		/// Changes the number of equations in this constraints (reset the 
		/// size of the C residual vector).
	virtual int Reset_Cn(int mCn);

		/// ---TO IMPLEMENT-- w.overloading
		/// This may be overloaded by child classes. Argument should be
		/// the 'database' where the reference restoring takes place.
		/// Should return false if referencing was not possible.
		/// Should set valid=true/false depending on referencing success.
	virtual bool RestoreReferences() {return true;};

		/// ---TO IMPLEMENT-- w.overloading
		/// This MUST be overloaded by child classes.
		/// It should compute the residuals (vector 'C') of the 
		/// constraint equations, where C=0 is for satisfied constraints.
		/// Should return false if updating was not possible.
	virtual bool Update() {if (IsActive()) return true; else return false;};
};
		






/// Constraint between parameters in a ChFunction
///
/// This is the base data for all types of constraints which 
/// define relations between parameters of a ChFunction (mostly, of
/// type ChFunctionSequence).



class ChConstraint_Chf : public ChConstraint{

 private:

 protected:
	ChFunction* root_function;

	ChRefFunctionSegment target_function;

 public:
	ChConstraint_Chf();
	ChConstraint_Chf(ChFunction* mRootFunct, char* mTreeIDs);
		
		// for easy access to inner data
	ChFunction* Get_root_function() {return root_function;}
	ChFunction* Get_target_function() {return target_function.GetFunction();}
		// if needed, for accessing the target funct.reference and change the tree-IDs, for new target..
	ChRefFunctionSegment* Get_target_reference() {return &target_function;}

		// --Note: this may be overloaded by children-
		// (Child classes may need also to restore references in other encapsulated ChRef objects).
	virtual bool RestoreReferences(ChFunction* mroot);

};
		




///
/// Algebraic constraint on ChFunctions, of the type  y(T)=A
/// Impose a value of the function (or its derivative) at given time T.
///


class ChConstraint_Chf_ImposeVal : public ChConstraint_Chf{

 private:
	double T;
	double value;
	int derivation_order;

 protected:

 public:
	ChConstraint_Chf_ImposeVal()  {T=0; value=0; derivation_order=0;};
	ChConstraint_Chf_ImposeVal(ChFunction* mRootFunct, char* mTreeIDs, double mtime, double mval);
	
	double GetT()          {return T;}
	void   SetT(double mT) {T=mT; }
	double GetValue()          {return value;}
	void   SetValue(double mv) {value=mv; }
	int    GetDerivationOrder()			{return derivation_order;}
	void   SetDerivationOrder(int mo)	{derivation_order = mo; }

	int  Get_Cn() {return 1;};

	//virtual bool RestoreReferences(ChFunction* mroot);
	
	virtual bool Update();		// --> see in .cpp
};



///
/// Algebraic constraint on ChFunctions,
/// Impose continuity between two function segments
///


class ChConstraint_Chf_Continuity : public ChConstraint_Chf{

 private:
	int continuity_order;
	int interface_num;
 protected:

 public:
	ChConstraint_Chf_Continuity() {continuity_order=0; interface_num=0;};
	ChConstraint_Chf_Continuity(ChFunction* mRootFunct, char* mTreeIDs, int cont_ord, int interf_num);
	
	int    GetInterfaceNum()          {return interface_num;}
	void   SetInterfaceNum(int mi) {interface_num = mi;}
	int    GetContinuityOrder()			{return continuity_order;}
	void   SetContinuityOrder(int mc)	{continuity_order = mc; }

	int  Get_Cn() {return 1;};

	//virtual bool RestoreReferences(ChFunction* mroot);
	
	virtual bool Update();		// --> see in .cpp
};



///
/// Algebraic constraint on ChFunctions.
/// Impose handle distance along X axis
///


class ChConstraint_Chf_HorDistance : public ChConstraint_Chf{

 private:

	double distance;
	int handleA;
	int handleB;

 protected:

 public:
	ChConstraint_Chf_HorDistance() {distance=1;};
	ChConstraint_Chf_HorDistance(ChFunction* mRootFunct, char* mTreeIDs, int mhA, int mhB);

	double GetDistance()          {return distance;}
	void   SetDistance(double md) {distance = md;}

	int    GetHandleA()          {return handleA;}
	int    GetHandleB()          {return handleB;}
	void   SetHandleA(int mh) {handleA = mh;}
	void   SetHandleB(int mh) {handleB = mh;}

	//virtual bool RestoreReferences(ChFunction* mroot);
	
	virtual bool Update();		// --> see in .cpp
};



///
/// Algebraic constraint on ChFunctions.
/// Impose segment vertical separation on Y axis, between two handles
///

class ChConstraint_Chf_VertDistance : public ChConstraint_Chf{

 private:

	double distance;
	int handleA;
	int handleB;

 protected:

 public:
	ChConstraint_Chf_VertDistance() {distance=1;};
	ChConstraint_Chf_VertDistance(ChFunction* mRootFunct, char* mTreeIDs, int mhA, int mhB);

	double GetDistance()          {return distance;}
	void   SetDistance(double md) {distance = md;}

	int    GetHandleA()          {return handleA;}
	int    GetHandleB()          {return handleB;}
	void   SetHandleA(int mh) {handleA = mh;}
	void   SetHandleB(int mh) {handleB = mh;}

	//virtual bool RestoreReferences(ChFunction* mroot);
	
	virtual bool Update();		// --> see in .cpp
};



} // END_OF_NAMESPACE____


#endif  // END of ChConstraint.h 
