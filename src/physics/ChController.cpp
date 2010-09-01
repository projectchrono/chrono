///////////////////////////////////////////////////
//
//   ChController.cpp
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <math.h>

#include "physics/ChController.h"
#include "physics/ChGlobal.h"


namespace chrono 
{ 



///   CLASS
///
/// 


ChControllerPID::ChControllerPID()
{
	this->SetIdentifier(ChGLOBALS().GetUniqueIntID()); // mark with unique ID

	P= 1.;
	I= 0.;
	D= 0.;
	
	Reset();
}

ChControllerPID::~ChControllerPID()
{
}



// Use Reset to set accumulator to zero, at beginning. For integrative part.


void ChControllerPID::Reset()
{
	this->In_int = 0.0;
	this->In_dt  = 0.0;
	this->In = 0;
	this->Out = 0;
	this->Pcomp = 0;
	this->Icomp = 0;
	this->Dcomp = 0;
	this->last_t = 0;
}


//
// Compute controller output value
//
 
double ChControllerPID::Get_Out(double new_in, double new_t)
{
	double mdt = (new_t - last_t);
	
	// please, no backward calling sequence!
	if (mdt < 0)
	{
		this->Reset();
		return 0.0;
	}

	// please, multiple calls at same time do not perform updates!
	if (mdt == 0)
	{
		return this->Out;
	}

	// OK......

	// compute derivative of in (simple Bdf differentiation)
	In_dt   = (new_in - In) / mdt;
	
	// compute integration of in  (trapezoidal rule ) 
	In_int += (new_in + In)*0.5 * mdt;

	// Go forward... synchronize time and last input recorder
	In		= new_in;
	last_t	= new_t;
	this->SetChTime(new_t);

	// ===  Compute PID components ================

	Pcomp = P * In;
	Icomp = I * In_int;
	Dcomp = D * In_dt;

	Out = Pcomp + Icomp + Dcomp;
	
	return Out;
}
			
				


} // END_OF_NAMESPACE____


////// end
