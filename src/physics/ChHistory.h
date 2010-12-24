#ifndef CHHISTORY_H
#define CHHISTORY_H

//////////////////////////////////////////////////
//  
//   ChHistory.h
//
//   global state vector Y (pos. and vel of bodies)
//   time history data.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChMath.h"
#include "core/ChApiCE.h"


namespace chrono 
{



////////////////////////////////////////////////////
/// Class for time history of system vector states.
/// 
/// Example, for nsteps= 3
///
///  - Ypointer->	Y (i=-2) time=0.03
///	 -	    		Y (i=-1) time=0.05
///	 -		    	Y (i= 0) time=0.06  (Y, the current Y)
///	 -			    Y (i=+1) time=0.09	(Ynew, for predictions, etc)

class ChApi ChHistory
{
private:
	int steps;
	int vars; 
	int current_Y;		// the actual Y pointer in array
	double* times;		// array of times
	ChMatrix<>** Yvectors;// array of pointer to Y vectors

public:
	ChHistory(int nvars, int nsteps);
	~ChHistory();
	void Copy(ChHistory* source);

	void Setup(int newvars, int newsteps);
	void Restart();

		/// Returns the offset in the arrays given the
		/// cyclic position "i", ranging from and to:
		/// (-steps+2)....0....(+1)

	int Get_arr_offset (int i);


		/// Gets the address of the Y vector, for instant "i",
		/// with 0= current, -1= previous, etc.
	ChMatrix<>* Get_Y(int i);

		/// Gets/sets the time of the instant "i".
	double Get_Ytime(int i);
	void   Set_Ytime(int i, double newtime);

		/// Rotates one step further the pointer to the
		/// actual Y vector, after the user has properly set a
		/// good acceptable Y(+1), which will become Y(0) this way.
	void ForwardStep();
	void BackwardStep();
	void ProceedStep(double m_dt);

		/// shortcuts:
		/// Gets the current Y address, and the 
		/// address of the future "Ynew".(the "work in progress" Y, at
		/// time t(+1);

	double Get_now_time () { return *(times + current_Y);}
	void Set_now_time (double m_time) { *(times + current_Y) = m_time;}
	ChMatrix<>* Get_Y()		{ return *(Yvectors + current_Y);}
	ChMatrix<>* Get_Ynew()	{ return Get_Y(1);}

		/// Returns the number of correctly sequenced 
		/// recordings of history, where the step difference
	    /// is not null (neither it is higher than "stepmax").
		/// it is 1 for a hjistory just started, and it is
	    /// equal to the nsteps for a well started procedure.
	int GetNsequenced(double maxstep);


	// PREDICTION / EXTRAPOLATION / INTERPOLATION FEATURES

		///  Fills the Ynew  -that is, Y(+1)- with an extrapolation
		/// of the time-history, using polynomial interpolation of
		/// given order.
		///  The time step between current Y and Ynew must be provided,
		/// and must be different from 0.
		///  Note: if "steps" is 2, there are only two Y vectors,
		/// that is Y and Ynew, so the extrapolation have no sense, at
		/// least Y(0) and Y(-1) must exist for simple linear extrapolation.

	void PredictYnew(int order, double m_dt);

		/// As before, but does not fill the Ynew with the 
		/// prediction: here an external Y must be provided. 
	void PredictY(int order, double m_dt, ChMatrix<>* Ypredicted);

		/// As before, but the resulting predicted vector is 
		/// not needed, because it returns directly the norm of 
		/// the error between predicted and actual Ynew (which 
		/// is supposed to be just computed and with correct time).
		/// This is because it's very memory-efficient.

	double PredictionError(int order);

};


} // END_OF_NAMESPACE____

#endif
