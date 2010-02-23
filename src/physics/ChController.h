#ifndef CHCONTROLLER_H
#define CHCONTROLLER_H

//////////////////////////////////////////////////
//  
//   ChController.h
//
//   Class for controllers (PID etc)
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

#include "core/ChMath.h"
#include "physics/ChObject.h"



namespace chrono 
{


///
/// Class for a basic PID controller
///
///  A basic PID controller, used as a 'black box'. Depending on 
/// input, it produces a controlled output proportional to input, 
/// input derivative by time, input integration
///

class ChControllerPID : public ChObj  {

private:
	double In;			// internal, last input set into controller, at time 'last_t'
	double In_int;		// internal, last integral, for integrative part of controller
	double In_dt;		// internal, last derivative, for derivative part of controller
	double last_t;		// internal, last time
	double Pcomp;	// internal, 
	double Icomp;	// internal, 
	double Dcomp;	// internal, 
	double Out;		// internal, last output value

public:
	ChControllerPID();
	~ChControllerPID();
	
	double P;	///< proportional coefficient
	double I;	///< integrative coefficient
	double D;	///< derivative coefficient
				
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
};


} // END_OF_NAMESPACE____


#endif  // END of ChController.h 
