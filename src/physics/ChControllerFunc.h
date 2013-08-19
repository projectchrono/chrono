#ifndef CHCONTROLLERFUNC_H
#define CHCONTROLLERFUNC_H

//////////////////////////////////////////////////
//  
//   ChController.h
//
//   Class for controllers (PID etc)
//	130814 - Justin - allow the user to define a time lag,
//		so the desired controller input (mInput in Get_Out)
//		doesn't instantly get applied to the controller
//		Results in smoother motion
//		Also, 
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
#include "core/ChApiCE.h"
#include "core/ChMath.h"
#include "physics/ChObject.h"
#include "physics/ChFunction.h"



namespace chrono 
{


///
/// Class for a basic PID controller
//	Note: templated by Justin to allow for ChFunctions to drive the controller input
///
///  A basic PID controller, used as a 'black box'. Depending on 
/// input, it produces a controlled output proportional to input, 
/// input derivative by time, input integration
/// Justin's addition to this class:
//		idea is that the user will modify the input to the PID controller, but the desired
//		input isn't used immediately; rather, the desired input changes w.r.t. a sine curve
//		that is a function of time, so the change in input is smooth rather than a step change
// Usage:
// create a new PID controller as usual.
// use the Get_Out(mIn, mTime) function as normal each timestep.
//	Difference is when the desired PID value is changed by the user input
//	Must create a new ChFunction object to pass to the controller. The amplitude 
//	and phase offset is changed to something new when the user changes the PID input slider.
//		SO, two distinct places where ResetFunc() should be used
//		1) initial condition (if it isn't zero, can use constructor for this)
//		2) each time the user changes the PID slider

template<class Function_Type = ChFunction_Sine> 
class ChApi ChControllerFunc : public ChObj  
{

private:
	double In;			// internal, last input set into controller, at time 'last_t'
	double In_int;		// internal, last integral, for integrative part of controller
	double In_dt;		// internal, last derivative, for derivative part of controller
	double last_t;		// internal, last time
	double Pcomp;	// internal, 
	double Icomp;	// internal, 
	double Dcomp;	// internal, 
	double Out;		// internal, last output value
	// moved coefficients here, so can't be arbitrarily changed
	double P;	///< proportional coefficient
	double I;	///< integrative coefficient
	double D;	///< derivative coefficient
	// JCM added
	double usedInput;	// actual value passed to the controller when calling Get_Out
	double t_lag;		// time to go from 0 to 
	Function_Type m_func;	// use a ChFunction derived type (see ChFunction.h)
	double y0;		// the value of the controller when the Function was created and passed
	double t0;		// the value of time when the Function was created and passed in

public:
	ChControllerFunc();
	ChControllerFunc(const double p, const double i, const double d);
	ChControllerFunc(const double p, const double i, const double d,
		const double time_lag, Function_Type& m_fun, double time);
	//~ChControllerFunc();
	
	/// COMPUTE CONTROL value!
	/// Given an input i, returns the output o=P*i+D*di/dt+I*Int(i dt)
	/// that is o = Pcomp+Icomp+Dcomp
	/// Calls to Get_Output must be done in (possibly uniform) time steps, 
	/// otherwise remember to call Reset() before other sequences of calls.
	double Get_Out(double mInput, double mTime);

				/// Same, but just returns last computed output
	double Get_Out() {return Out;};

				// Read-only values, telling which was the components 
				// of the last outputted control, divided by proportional or
				// integrative or derivative part.
	double Get_Pcomp() {return Pcomp;}	
	double Get_Icomp() {return Icomp;}
	double Get_Dcomp() {return Dcomp;}
	double Get_In_int()  {return In_int;}
	double Get_In_dt()   {return In_dt;}
	double Get_In()   {return In;}

				/// Use Reset to set accumulator to zero, at beginning. For integrative part.
	void Reset(); 
	void ResetFunc(Function_Type& fun, double time);	// when the user moves the slider, it will result in a new def. of the function
	double Get_Func_val(double new_in, double new_t);	// use the ChFunction to find the actual val to be passed to the PID controller
};


} // END_OF_NAMESPACE____


#endif  // END of ChController.h 
