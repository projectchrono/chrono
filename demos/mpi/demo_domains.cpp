///////////////////////////////////////////////////
//
//   Demo code about 
//
//     - distributed multi domain simulation
//
//	 CHRONO 
//   ------
//   Multibody dinamics engine
// 
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
  
     
// Include some headers used by this tutorial...

#include "physics/ChApidll.h"
#include "lcp/ChLcpVariablesGeneric.h"
#include "lcp/ChLcpVariablesBody.h"
#include "lcp/ChLcpConstraintTwoGeneric.h"
#include "core/ChLinearAlgebra.h"
#include "unit_mpi/ChMpi.h"
#include "unit_mpi/ChSystemMPI.h"
#include "unit_mpi/ChBodyMPI.h"
#include "unit_mpi/ChLcpSystemDescriptorMPI.h"
#include "unit_mpi/ChDomainLatticePartitioning.h"
#include <sstream> // TEST

// Remember to use the namespace 'chrono' because all classes 
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;



// Do some tests in a single run, inside the main() function.
// Results will be simply text-formatted outputs in the console..

int main(int argc, char* argv[])
{
	DLL_CreateGlobals(); 

	// Initialize the MPI functionality. Use the ChMPI static functions.  
	ChMPI::Init(argc,argv);


	GetLog() << "\n\n\n ----------------------------------------------\n"
			 << "Example: use MPI for a multi-domain simulation \n\n";

	// Get infos about how many processes are launched, 
	// and about the ID of this specific process.
	int numprocs = ChMPI::CommSize();
	int myid     = ChMPI::CommRank();

	if (numprocs != 12)
	{
		if (myid == 0)
		{
			GetLog() << "ERROR: you must use 12 processes! \n";
			GetLog() << "       Note that this demo must be launched only \n" 
					 << "       using the 'mpiexec', from the MPI toolset! \n";
		}
		ChMPI::Finalize();
		DLL_DeleteGlobals();
		return 0;
	}

	// Instead of using the usual ChSystem, rather use ChSystemMPI. It is
	// a specialized class for physical systems that can be used 
	// for domain decomposition with MPI communication. 

	ChSystemMPI mysystem;
 

	// Since we run multiple processes of this program, we must
	// set a 'topology' between them. Currently, cubic lattice
	// subdivision is supported. 
	// A 'partitioner' tool will help to setup this.
	int xdomains = 1;
	int ydomains = 2;
	int zdomains = 2;
	ChDomainLatticePartitioning mypartitioner(2,3,2,			// nx ny nz domains
										ChVector<>(-5,-6,-5),	// max world
										ChVector<>( 5, 2, 5) );	// min world

	mypartitioner.SetupNode(mysystem.nodeMPI, myid); // btw: take care that must be numprocs=nx*ny*nz

	ChSystemDescriptorMPIlattice3D mydescriptor(&mysystem.nodeMPI);

	mysystem.ChangeLcpSystemDescriptor(&mydescriptor);

	GetLog() << "ID=" << ChMPI::CommRank() << "  id_MPI=" << mysystem.nodeMPI.id_MPI << "\n";
	
	// Some logging for debugging..
/*
	GetLog() << "	min_box" << mysystem.nodeMPI.min_box << "\n";
	GetLog() << "	max_box" << mysystem.nodeMPI.max_box << "\n";

	GetLog() << "	   interfaces:\n";
	for (int j = 0; j<9; j++)	
		GetLog() << j << "=" << mysystem.nodeMPI.interfaces[j].id_MPI << " ";
	GetLog() << "\n";
	for (int j = 9; j<18; j++)	
		GetLog() << j << "=" << mysystem.nodeMPI.interfaces[j].id_MPI << " ";
	GetLog() << "\n";
	for (int j = 18; j<27; j++)	
		GetLog() << j << "=" << mysystem.nodeMPI.interfaces[j].id_MPI << " ";
	GetLog() << "\n";
*/

	if (myid == 0)
	{
		int added_id=0;

		GetLog() << "ID=" << mysystem.nodeMPI.id_MPI << " Adding body.. \n";
		// Create a body of the type that can cross boundaries and support 
		// MPI domain decomposition.
		ChSharedPtr<ChBodyMPI> mybody(new ChBodyMPI);

		// Set unique identifier, among entire multi-domain world.
		// Here numbering trick is to allow 2 million of bodies per domain.
		mybody->SetIdentifier(mysystem.nodeMPI.id_MPI*(2e6) + added_id);
		added_id++;

		mybody->SetCollide(true);
		mybody->GetCollisionModel()->ClearModel();
		mybody->GetCollisionModel()->AddBox(0.1,0.1,0.1, &ChVector<>(-4,-6, -0.01) );
		mybody->GetCollisionModel()->BuildModel();
		
	 	mysystem.Add(mybody);
	
		//mybody->GetCollisionModel()->SyncPosition();
		//ChVector<> vmin;
		//ChVector<> vmax;
		//mybody->GetCollisionModel()->GetAABB(vmin,vmax);
		//GetLog() << "ID=" << mysystem.nodeMPI.id_MPI << "      bbox = " << vmin << "\n" << vmax << "\n\n";
	}
	
	// Test the serializing-deserializing of objects that spill out of boundaries
	mysystem.CustomEndOfStep();
	GetLog() << "\n";
	mysystem.CustomEndOfStep();

	//GetLog() << "ID=" << mysystem.nodeMPI.id_MPI << " CustomEndOfStep.. \n";
	//mysystem.CustomEndOfStep(); // mmmhh, error because spilled body flags must be updated after deserializing..

	


	// Terminate the MPI functionality.
	ChMPI::Finalize();

	DLL_DeleteGlobals();

	return 0;
}


