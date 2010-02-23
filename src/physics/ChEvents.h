#ifndef CHEVENTS_H
#define CHEVENTS_H

//////////////////////////////////////////////////
//  
//   ChEvents.h
//
//   Class for simple event recorder (cyclic buffer)
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <string>

namespace chrono 
{
 

#define CHCLASS_EVENTS 43		

#define CHCLASS_EVENTS_DEFAULTNUM 100		

///
/// Class for a quick cyclic event recorder. It takes
/// really few resources and CPU cycles.
///

class ChEvents {
protected:


	int*	ebuffer;
	int		current_event;
	int		maxevents;
public:	
	ChEvents ();
	ChEvents (int m_events);
	~ChEvents ();

		// Store the event coded "m_event" and increment position
	void Record (int m_event);
	
		// Get the event "n_back" positions before the last one,
		// n_back = 0 returns the last, n_back = 1 the one before the last, etc.
	int GetN(int n_back);
		// Same, but sets events.
	void SetN(int n_back, int m_event);

		// Get the last recorded event.
	int GetLast();

	int GetMaxEvents () {return maxevents;};
		
		// Reset all events to zero, and rewind counter to beginning.
	void ResetAllEvents();
};





// Some CHRONO-specific event identifiers...

#define	CHEVENT_NONE		0
#define	CHEVENT_SETUP		1
#define	CHEVENT_UPDATE		2
#define	CHEVENT_INTSTEP		3
#define	CHEVENT_REDOSTEP	4
#define	CHEVENT_ASSEMBLY	5
#define	CHEVENT_LIMIT		6
#define	CHEVENT_R3DSTEP		7
#define CHEVENT_COMPUTEYDT  8
#define CHEVENT_TIMESTEP    9
#define CHEVENT_COLLISION  10
#define CHEVENT_OPENMONOL  11
#define	CHEVENT_REWINDCOLL 12

#define	CHEVENTS_TOTAL	   12	// ***TO UPDATE all times a new event type is added above.


} // END_OF_NAMESPACE____


#endif  // END of ChEvents.h 
