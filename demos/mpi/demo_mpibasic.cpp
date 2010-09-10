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
#include <iostream>
#include <sstream>

#include "mpi.h"


using namespace chrono;

 
 

int main(int argc, char* argv[])
{
	// The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed.
	DLL_CreateGlobals(); 

	// Initialize the MPI functionality. Use the ChMPI static functions.  
	ChMPI::Init(argc,argv);

	// Get infos about how many processes are launched, 
	// and about the ID of this specific process.
	int numprocs = ChMPI::CommSize();
	int myid     = ChMPI::CommRank();

	


	if (myid==0)
		GetLog() << "Number of processes : " << numprocs << "\n";

	if (numprocs < 2)
	{
		GetLog() << "Use at least 2 processes! \n";
		ChMPI::Finalize();
		DLL_DeleteGlobals();
		return 0;
	}


	//
	// TEST 1   -   send and receive ChMatrix<> object
	//

	if (myid==0) // sender
	{
		ChMatrixDynamic<> mmatr(3,4);
		for (int i=0; i<3; i++)
			for (int j=0; j<4; j++)
				mmatr(i,j)=i*j;
		GetLog() << "Id 0: sending ChMatrix: " << mmatr << "\n";
		ChMPI::SendMatrix(1, mmatr, ChMPI::MPI_STANDARD, false,0);
	}

	if (myid==1) // receiver
	{
		ChMatrixDynamic<> mmatr(3,4); // must already have the proper size
		ChMPIstatus mstatus;
		ChMPI::ReceiveMatrix(0, mmatr, &mstatus, false,0);
		GetLog() << "Id 1: received ChMatrix: " << mmatr << "\n";
	}


	//
	// TEST 2   -   send and receive strings
	//

	if (myid==0) // sender
	{
		std::stringstream mstrbuf(std::stringstream::in | std::stringstream::out);
		mstrbuf << "The quick lazy fog jumped on the brown dog.";
		GetLog() << "Id 0: sending string: " << mstrbuf.rdbuf()->str() << "\n";
		ChMPI::SendString(1, mstrbuf.rdbuf()->str(), ChMPI::MPI_STANDARD, false,0);
	}

	if (myid==1) // receiver
	{
		std::stringstream mstrbuf2(std::stringstream::in | std::stringstream::out);
		ChMPIstatus mstatus;
		std::string mstr;
		ChMPI::ReceiveString(0, mstr, &mstatus);
		
		GetLog() << "Id 1: received string: " << mstr << "\n";

	}



	//
	// TEST 3   -   send and receive whatever serializable chrono object
	//              i.e. objects that implements StreamIN() and StreamOUT()
	//              i.e. objects that can be saved via << and >>.
	//

	if (myid==0) // sender
	{
		std::stringstream mstream;
		ChStreamOutBinaryStream mchstreamo(&mstream);
	
		ChVector<> mv(12,23,45.34);
		mchstreamo << mv;	// serialize object to binary stream, in memory.

		GetLog() << "Id 0: sending serialized ChVector: " << mv << "\n";
		ChMPI::SendString(1, mstream.rdbuf()->str(), ChMPI::MPI_STANDARD, false,0);
	}

	if (myid==1) // receiver
	{
		ChMPIstatus mstatus;
		std::string mstr;
		ChMPI::ReceiveString(0, mstr, &mstatus);

		std::stringstream mstream;
		mstream << mstr;
		ChStreamInBinaryStream mchstreami(&mstream);

		ChVector<> mv;
		mchstreami >> mv;	// deserialize object from binary stream.
		GetLog() << "Id 1: received serialized ChVector: " << mv << "\n";
	}



	// Terminate the MPI functionality.
	ChMPI::Finalize();

	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}


