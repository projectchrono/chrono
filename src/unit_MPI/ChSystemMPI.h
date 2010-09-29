#ifndef CHSYSTEMMPI_H
#define CHSYSTEMMPI_H

//////////////////////////////////////////////////
//  
//   ChSystemMPI.h
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChMpi.h"
#include "physics/ChSystem.h"
#include "ChLcpSystemDescriptorMPI.h"

namespace chrono 
{


/// Class for a specialized version of ChSystem that can be used
/// in projects that run on multiple computing nodes (MPI)

class ChSystemMPI : public ChSystem
{
public:

	ChSystemMPI();
	virtual ~ChSystemMPI();

	// Override base class functions

				/// Pushes back all ChConstraints and ChVariables contained in links,bodies,etc. 
				/// into the LCP descriptor. If the LCP descriptor is of ChLcpSystemDescriptorMPI
				/// also sets the custom data with shared variables etc.
	//virtual void LCPprepare_inject(ChLcpSystemDescriptor& mdescriptor);

private:

};



} // END_OF_NAMESPACE____


#endif  // END of ChSystemMPI.h 
