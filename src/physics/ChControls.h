#ifndef CHCONTROLS_H
#define CHCONTROLS_H

//////////////////////////////////////////////////
//  
//   ChControls.h
//
//   Class for controls
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

#include "physics/ChObject.h"


namespace chrono 
{



///
/// Basic interface class for 'controls', that are objects that change parameters
/// during the simulation, ex. to simulate PIDs, etc.
/// Must be inherited and implemented by user.
/// 


class ChApi ChControls : public ChObj 
{
								// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChControls,ChObj);

public:

	ChControls() {};
	virtual ~ChControls() {};
	
	//void Copy(ChControls* source);

			//
			// FUNCTIONS - interface to be implemeted -
			//

	virtual int ExecuteForStart() = 0;
	virtual int ExecuteForUpdate() = 0;
	virtual int ExecuteForStep() = 0;
	virtual int ExecuteFor3DStep() = 0;

			//
			// STREAMING
			//

					/// Method to allow deserializing a persistent binary archive (ex: a file)
					/// into transient data.
	void StreamIN(ChStreamInBinary& mstream);

					/// Method to allow serializing transient data into a persistent
					/// binary archive (ex: a file).
	void StreamOUT(ChStreamOutBinary& mstream);


};
		

} // END_OF_NAMESPACE____


#endif  // END of header
