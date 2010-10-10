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
#include "ChDomainNodeMPI.h"

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

				/// Executes custom processing at the end of step. In detail,
				/// performs the MPI inter-domain exchange of objects that spill out
				/// of the domains, by streaming to binary buffers, sending them via MPI 
				/// to domains that receive them, and by deserializing and adding to receiving domain. 
	virtual void CustomEndOfStep();

	ChDomainNodeMPIlattice3D nodeMPI;

private:

};



} // END_OF_NAMESPACE____


#endif  // END of ChSystemMPI.h 
