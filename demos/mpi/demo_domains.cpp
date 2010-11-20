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
#include "unit_mpi/ChLcpIterativeSchwarzMPI.h"


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
					 << "       using the 'mpiexec', from the MPI toolset! \n"
					 << "       Also, the current directory must be bin/data/mpi \n";
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

	ChDomainLatticePartitioning mypartitioner(2,3,2,			// nx ny nz domains
										ChVector<>(-5,-4,-5),	// max world
										ChVector<>( 5, 2, 5) );	// min world

	mypartitioner.SetupNode(mysystem.nodeMPI, myid); // btw: please take care that must be numprocs=nx*ny*nz


	// Prepare the system with a special 'system descriptor' 
	// that is necessary when doing simulations with MPI.
	ChSystemDescriptorMPIlattice3D mydescriptor(&mysystem.nodeMPI);
	mysystem.ChangeLcpSystemDescriptor(&mydescriptor);

	// Use the Schwarz solver
	ChLcpIterativeSchwarzMPI mysolver;
	mysystem.ChangeLcpSolverSpeed(&mysolver);


	// Save on file the aabb of the boundaries of each domain, for debugging/visualization
	bool save_domain_boxes_on_file = true;
	ChMPIfile* domainfile = 0;
	if (save_domain_boxes_on_file)
	{
		//ChMPIfile::FileDelete("output\\domains.dat"); // delete prev.file, if any. Otherwise might partially overwrite
		domainfile = new ChMPIfile("output\\domains.dat", ChMPIfile::ChMPI_MODE_WRONLY | ChMPIfile::ChMPI_MODE_CREATE);
		char buffer[100];
		sprintf(buffer, "%d, %g, %g, %g, %g, %g, %g ,\n", 
						mysystem.nodeMPI.id_MPI, 
						mysystem.nodeMPI.min_box.x,	mysystem.nodeMPI.min_box.y,	mysystem.nodeMPI.min_box.z,
						mysystem.nodeMPI.max_box.x,	mysystem.nodeMPI.max_box.y,	mysystem.nodeMPI.max_box.z);
		domainfile->WriteOrdered((char*)buffer, strlen(buffer));
		delete domainfile;
	}

	// Ok, now fill the ChSystemMPI (ie. the domain assigned to this process) by
	// adding some rigid bodies, such as spheres, cubes, etc. 

	int added_id=0;
	for (int npart = 0; npart<460; npart++)
	{
		ChVector<> center(ChRandom()*2-1,ChRandom()*2-1,ChRandom()*2-1);
	
		// IMPORTANT: before adding a body, use the IsInto() check to see if the center
		// of the body is inside the domain of this process, and add only if so. 
		// No problem if the object AABB overlaps neighbouring domains. 
		if (mysystem.nodeMPI.IsInto(center))
		{
			// Create a body of the type that can cross boundaries and support 
			// MPI domain decomposition.
			ChSharedPtr<ChBodyMPI> mybody(new ChBodyMPI);

			// Set unique identifier, among entire multi-domain world. Every id works, but must be unique.
			mybody->SetIdentifier(npart);//mysystem.nodeMPI.id_MPI*(2e6) + added_id);
			added_id++;

			mybody->SetCollide(true);
			mybody->GetCollisionModel()->ClearModel();
			//mybody->GetCollisionModel()->AddBox(0.1,0.1,0.1); 
			mybody->GetCollisionModel()->AddSphere(0.1);
			mybody->GetCollisionModel()->BuildModel();
			
	 		mysystem.Add(mybody);
			mybody->SetPos( center ); // BTW here I am sure that the object AABB center is the COG position too, that might not be true in general.
			mybody->GetCollisionModel()->SyncPosition(); // really necessary?
			mybody->Update(); // really necessary?
		}

	}

	// Add a flat box as a ground

	ChVector<> groundcenter(0.01,-1.2, 0.01);
	if (mysystem.nodeMPI.IsInto(groundcenter))
	{

		ChSharedPtr<ChBodyMPI> mybody(new ChBodyMPI);
		mybody->SetIdentifier(10000 + mysystem.nodeMPI.id_MPI );

		//mybody->SetBodyFixed(true);
		mybody->SetCollide(true);
		mybody->GetCollisionModel()->ClearModel();
		mybody->GetCollisionModel()->AddBox(2,0.1,2); 
		mybody->GetCollisionModel()->BuildModel();
				
		mysystem.Add(mybody);
		mybody->SetPos( groundcenter ); 
		mybody->GetCollisionModel()->SyncPosition(); 
		mybody->Update(); 
	}


	//
	// PERFORM SOME TIME STEPS OF SIMULATION
	// 

	bool save_positions_on_file = true;

	// Initial setup
	mysystem.CustomEndOfStep();
	//***TEST***
	ChMPIfile* idebugfile = new ChMPIfile("mpi_debug_domains.txt", ChMPIfile::ChMPI_MODE_WRONLY | ChMPIfile::ChMPI_MODE_CREATE);
	mysystem.WriteOrderedDumpDebugging(*idebugfile);
	delete idebugfile;

	int totsavedsteps = 0;

	while(mysystem.GetChTime() < 2.50)
	{ 
		//GetLog() << "ID=" << ChMPI::CommRank() << "     time =" << mysystem.GetChTime() << "\n";

		if (save_positions_on_file)
		{
			char padnumber[100];
			sprintf(padnumber, "%d", (totsavedsteps+10000));
			char filename[100];

			sprintf(filename, "output\\pos%s.dat", padnumber+1);
			//ChMPIfile::FileDelete(filename); // Delete prev.,if any. Otherwise might partially overwrite
			ChMPIfile* posfile = new ChMPIfile(filename, ChMPIfile::ChMPI_MODE_WRONLY | ChMPIfile::ChMPI_MODE_CREATE);
			mysystem.WriteOrderedDumpAABB(*posfile);
			delete posfile;

			
			sprintf(filename, "output\\debug%s.dat", padnumber+1);
			//ChMPIfile::FileDelete(filename); // Delete prev.,if any. Otherwise might partially overwrite
			ChMPIfile* debugfile = new ChMPIfile(filename, ChMPIfile::ChMPI_MODE_WRONLY | ChMPIfile::ChMPI_MODE_CREATE);
			mysystem.WriteOrderedDumpDebugging(*debugfile);
			delete debugfile;
			

			++totsavedsteps;
		}

		// Advance the simulation time step 
		mysystem.DoStepDynamics( 0.02 );
	}
	


	// Terminate the MPI functionality.
	ChMPI::Finalize();

	DLL_DeleteGlobals();

	return 0;
}


