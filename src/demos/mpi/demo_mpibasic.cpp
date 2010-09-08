///////////////////////////////////////////////////
//
//   Demo code about  
// 
//     - using MPI basic features.
//
//	 NOTE! this program should be copied
//   on multiple hosts of a cluster and executed 
//   using the launcher utility of the MPICH2 
//   toolchain (ex. mpiexec or wmpiexec.exe).
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



#include "physics/ChApidll.h" 
#include "unit_MPI/ChMpi.h"

#include "mpi.h"


using namespace chrono;

 
 

int main(int argc, char* argv[])
{
	// The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed.
	DLL_CreateGlobals(); 

	CHMPI::Init(argc,argv);

	Ch_test_mpi mtest;
	mtest.run_test_matrix();
 
	CHMPI::Finalize();

	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}


