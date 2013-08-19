///////////////////////////////////////////////////
//
//   ChControllerFunc.cpp
// ------------------------------------------------
// 	 Copyright: Justin Madsen
// ------------------------------------------------
///////////////////////////////////////////////////

#include <math.h>

#include "physics/ChControllerFunc.h"
#include "physics/ChGlobal.h"


namespace chrono 
{ 



///   CLASS
///
/// 

ChControllerFunc<>::ChControllerFunc() : P(1.0), I(0), D(0)
{
	this->SetIdentifier(CHGLOBALS().GetUniqueIntID()); // mark with unique ID
	m_func = ChFunction<Function_Type>();
	t_lag = 1.0;
	y0 = 0.0;
	t0 = 0.0;
	// zero out the other vars
	Reset();
}

ChControllerFunc<>::ChControllerFunc(const double p, const double i, const double d) : P(p), I(i), D(d)
{
	this->SetIdentifier(CHGLOBALS().GetUniqueIntID()); // mark with unique ID
	m_func = ChFunction<Function_Type>();
	t_lag = 1.0;
	y0 = 0.0;
	t0 = 0.0;

	Reset();

}

template<class Function_Type>
ChControllerFunc<Function_Type>::ChControllerFunc(const double p, const double i, const double d,
		const double time_lag, Function_Type& func, double time) : P(p), I(i), D(d), t_lag(time_lag), m_func(Function_Type(func)), t0(time)
{
	this->SetIdentifier(CHGLOBALS().GetUniqueIntID()); // mark with unique ID

	Reset();
}



// Use Reset to set accumulator to zero, at beginning. For integrative part.
template<>
void ChControllerFunc<>::Reset()
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

template<class Function_Type>
void ChControllerFunc<Function_Type>::ResetFunc(Function_Type& fun, double time)
{
	this->m_func = Function_Type(fun);
	this->t0 = time;

}

double ChControllerFunc<>::Get_Func_val(double new_in, double new_t)
{
	// if we've had the same function operating for a while, eventually 
	// the time lag doesn't do anything to modify the input
	double dt = new_t - this->t0
	if( dt > this->t_lag )
		return new_in;

	// otherwise, the output will be the PID control value when the function
	// was first input, plus some increment according to the sine curve
	double used_in = y0 + m_func.Get_y( dt );
	return used_in;
}

// Compute controller output value
// note: user defined PID input, new_in, is NOT used to find PID control values
// rather, the usedInput variable will be the input, and computed based on 
// the ChFunction
double ChControllerFunc<>::Get_Out(double new_in, double new_t)
{
	double mdt = (new_t - last_t);
	
	// no backward calling sequence!
	if (mdt < 0)
	{
		this->Reset();
		return 0.0;
	}

	// multiple calls at same time do not perform updates!
	if (mdt == 0)
	{
		return this->Out;
	}

	// use the ChFunction to find the new input value to use
	this->usedInput = Get_Func_val(new_in, new_t);

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
