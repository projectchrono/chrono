#ifndef CHREALTIMESTEP_H
#define CHREALTIMESTEP_H

//////////////////////////////////////////////////
//
//   ChRealtimeStep.h
//
//   Class for a timer which measure the time spent
//   in VR or game-like simulation loops, and suggests
//   a dt integration step for the physical simulation
//   for the next step.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChTimer.h"
#include "ChMath.h"

namespace chrono
{


/// Class for a timer which measure the time spent
/// in VR or game-like simulation loops, and suggests
/// a dt integration step for the physical simulation
/// for the next step. Uses high-resolution timer.
/// Note that the suggested integrations step tries
/// to keep a true real-time pace in advancing the 
/// simulation, that is the simulated time should 
/// match the real time. The suggested step may 
/// be non-constant because the overhead of the
/// simulaiton loop may vary because of varying 
/// overhead in visualization, collision or simulation.

class ChRealtimeStepTimer : public ChTimer<double>
{
public:
		/// Create the timer (outside the simulation loop, preferably
		/// just before beginning the while{} loop)
	ChRealtimeStepTimer() 
			{
				this->start();
			}

		/// Call this function INSIDE the simulation loop, just ONCE
		/// per loop, to get the suggested time for the next integration
		/// time step. If the the ChRealtimeStepTimer measured that
	    /// previous simulation step required few real-time, it will 
		/// suggest a corresponding small value for advancing the simulated 
		/// time, and viceversa will give higher values (up to a maximum 
		/// 'max_step' limit, however) if the simulation goes slow because
		/// of high CPU overhead. If the clamping value of 'max_step' is
		/// not reached, the real-time and simulated time should always match.
		/// There is also an optional 'min_step' value, which avoids too small
		/// integration steps.
		
	double SuggestSimulationStep(double max_step =0.02,		///< upper limit for step 
								 double min_step =CH_NANOTOL) ///< lower limit for step
			{
				this->stop();
				double mstep = (*this)();
				this->start();

				if (mstep<min_step) 
					return min_step;
				if (mstep>max_step) 
					return max_step;
				return mstep;
			}
};




} // END_OF_NAMESPACE____




#endif  // END of ChTimer.h
