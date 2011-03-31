#ifndef CHFUNCT_RAMP_H
#define CHFUNCT_RAMP_H

//////////////////////////////////////////////////
//  
//   ChFunction_Ramp.h
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


namespace chrono 
{

#define FUNCT_RAMP		1



/// LINEAR FUNCTION (like a straight ramp)
/// y = y0 + x * speed
/// 

class ChApi ChFunction_Ramp : public ChFunction
{
	CH_RTTI(ChFunction_Ramp, ChFunction);
private:
	double y0;
	double ang;
public:
	ChFunction_Ramp () {y0 = 0; ang=1;};
	ChFunction_Ramp (double m_y0, double m_ang)  {y0 = m_y0; ang = m_ang;};
	~ChFunction_Ramp () {};
	void Copy (ChFunction_Ramp* source);
	ChFunction* new_Duplicate ();

	int Get_Type () {return (FUNCT_RAMP);}			
	
			// Custom data

				/// The value for x=0;
	void	Set_y0   (double m_y0)   {y0 = m_y0;};	
	double	Get_y0 ()  {return y0;};

				/// The angular coefficient.
	void	Set_ang  (double m_ang)  {ang = m_ang;};
	double	Get_ang () {return ang;};


			// Override the Get_y(), Get_y_dx etc. functions with analytical formulas.

	double Get_y      (double x) {return (y0 + (x*ang))  ;};
	double Get_y_dx   (double x) {return (ang)			 ;};
	double Get_y_dxdx (double x) {return  0				 ;};

			// Expose the parameters which can be used for optimizations, as Javascript vars.

	OPT_VARIABLES_START
		"C",
		"ang",
	OPT_VARIABLES_END
	

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};





} // END_OF_NAMESPACE____


#endif
