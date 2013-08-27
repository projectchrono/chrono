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
template<class Function_Type>
ChControllerFunc<Function_Type>::ChControllerFunc() : P(1.0), I(0), D(0), t_lag(1.0), y0_func(0.0), t0_func(0.0)
{
	this->SetIdentifier(CHGLOBALS().GetUniqueIntID()); // mark with unique ID
	m_func = ChFunction<Function_Type>();
	// zero out the other vars
	Reset();
}

template<class Function_Type>
ChControllerFunc<Function_Type>::ChControllerFunc(double p, double i, double d,
		double time_lag) : P(p), I(i), D(d), t_lag(time_lag), y0_func(0.0), t0_func(0.0)
{
	this->SetIdentifier(CHGLOBALS().GetUniqueIntID()); // mark with unique ID
	m_func = ChFunction<Function_Type>();

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

/*
template<class Function_Type>
void ChControllerFunc<Function_Type>::ResetFunc(Function_Type& fun, double time)
{
	this->m_func = Function_Type(fun);
	this->t0 = time;

}
*/

double ChControllerFunc<>::Get_Func_val(double new_t)
{
	// if we've had the same function operating for a while, eventually 
	// the time lag doesn't do anything to modify the input
	// so input = y0_func + amplitude
	double dt = new_t - this->t0_func;
	if( dt > this->t_lag || this->t0_func == 0.0) {
//		return y0_func + m_func.Get_amp();
		return this->In;
	}

	// otherwise, the output will be the PID control value when the function
	// was first input, plus some increment according to the sine curve
	double used_in = y0_func + (m_func.Get_y( dt ));
	return used_in;
}

// Compute controller output value
// note: user defined PID input, new_in, is NOT used to find PID control values
// rather, the usedInput will be the input, and computed based on 
// the ChFunction
double ChControllerFunc<>::Get_Out(double new_t)
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
	// force the PID input value to follow the defined function
	// NOTE: t0_func is = 0 before the user moves the slider
	//		so don't worry about not having a set defined function as m_func
	//		upon initialization
	double usedInput = this->Get_Func_val(new_t);

	// compute derivative of in (simple Bdf differentiation)...
//	In_dt   = (usedInput - In) / mdt;
	// ... or let the function determine the derivative
	In_dt = m_func.Get_y_dx(mdt);
	
	// compute integration of in  (trapezoidal rule ) 
	In_int += (usedInput + In)*0.5 * mdt;

	// Go forward... synchronize time and last input recorder
	In		= usedInput;
	last_t	= new_t;
	this->SetChTime(new_t);

	// ===  Compute PID components ================
	Pcomp = P * In;
	Icomp = I * In_int;
	Dcomp = D * In_dt;

	Out = Pcomp + Icomp + Dcomp;
	
	return Out;
}				

void ChControllerFunc<>::Set_newInput(double mInput, double time)
{
	double amp = mInput - this->In;
	this->m_func = ChFunction_Sine(0.0, 1.0/(4.0*(this->t_lag)), amp);
	this->y0_func = this->In;
	this->t0_func = time;

}

void ChControllerFunc<>::Set_timeLag(double mT_lag)
{
	if(mT_lag <= 0.0 )
	{
		GetLog() << "can't set a negative or zero time lag on controller, setting to 1 \n";
		this->t_lag = 1.0;
	} else {
		this->t_lag = mT_lag;
	}


}

} // END_OF_NAMESPACE____


////// end
