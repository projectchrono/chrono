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

class ChApiMPI ChSystemMPI : public ChSystem
{
public:

	ChSystemMPI();
	virtual ~ChSystemMPI();

	// Override base class functions

	virtual void LCPprepare_inject(ChLcpSystemDescriptor& mdescriptor);

				/// Executes custom processing at the end of step: performs 
				///	  InterDomainSyncronizeStates()
				///   InterDomainSyncronizeStates()
				///   InterDomainSetup()   
	virtual void CustomEndOfStep();

	// Custom functions

				/// For shared items that overlap the domain boundaries, this
				/// function does some MPI communication to be sure that the 
				/// speeds and positions of shared objects (hence in multiple copies,
				/// in neighbouring domains) really have the same values. In fact
				/// different integration schemes/numerical issues in domains could
				/// lead to small differences between the n copies of the same shared item.
	virtual void InterDomainSyncronizeStates();

				/// Performs the MPI inter-domain exchange of objects that spill out
				/// of the domains, by streaming to binary buffers, sending them via MPI 
				/// to domains that receive them, and by deserializing and adding to receiving domain.
				/// Also, remove objects from domains when they entirely exit from domains. 
	virtual void InterDomainSetup();



				/// For debugging: call this function to dump all AABB bounding box information to the 
				/// specified MPI ordered file, for debugging/visualization/postprocessing etc. 
				/// The ASCII output file will have these infos at each line:
				///    rank, uniqueId, shared, xmin, ymin, zmin, xmax, ymax, zmax 
				/// where shared can be 0=no shared, 1=shared:master, 2=shared:slave
				/// NOTE: it must be called by all domains, no exceptions, becuse it contains 
				/// a MPI_File_write_ordered()
	virtual void WriteOrderedDumpAABB(ChMPIfile& output);

				/// For debugging: call this function to dump interfce data etc. as ASCII
				/// NOTE: it must be called by all domains, no exceptions, becuse it contains 
				/// a MPI_File_write_ordered()
	virtual void WriteOrderedDumpDebugging(ChMPIfile& output);


	virtual void AddSafely (ChSharedPtr<ChPhysicsItem> newitem);
		//
		// DATA
		//

	ChDomainNodeMPIlattice3D nodeMPI;

private:

};



} // END_OF_NAMESPACE____


#endif  // END of ChSystemMPI.h 
