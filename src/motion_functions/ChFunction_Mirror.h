#ifndef CHFUNCT_MIRROR_H
#define CHFUNCT_MIRROR_H

//////////////////////////////////////////////////
//  
//   ChFunction_Mirror.h
//
//   Function objects, 
//   as scalar functions of scalar variable y=f(t)
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChFunction_Base.h"
#include "ChFunction_Const.h"


namespace chrono 
{


#define FUNCT_MIRROR	18



/// MIRROR FUNCTION:
/// y = __/\__ 
///
/// Mirrors a function about a vertical axis.

class ChApi ChFunction_Mirror : public ChFunction
{
	CH_RTTI(ChFunction_Mirror, ChFunction);
private:
	ChFunction* fa;
	double mirror_axis;			// simmetry axis position on x

public:
	ChFunction_Mirror() {mirror_axis = 0; fa = new ChFunction_Const;}
	~ChFunction_Mirror () {if (fa) delete fa;};
	void Copy (ChFunction_Mirror* source);
	ChFunction* new_Duplicate ();

	void Set_mirror_axis  (double m_axis)  {mirror_axis = m_axis;}
	double Get_mirror_axis () {return mirror_axis;}

	void Set_fa  (ChFunction* m_fa)  {fa = m_fa;}
	ChFunction* Get_fa () {return fa;}
	
	double Get_y      (double x) ;

	void Extimate_x_range (double& xmin, double& xmax);
	int Get_Type () {return (FUNCT_MIRROR);}

	int MakeOptVariableTree(ChList<chjs_propdata>* mtree);
	OPT_VARIABLES_START
		"mirror_axis",
	OPT_VARIABLES_END

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};




} // END_OF_NAMESPACE____


#endif
