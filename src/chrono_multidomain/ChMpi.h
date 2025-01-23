// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHMPIMD_H
#define CHMPIMD_H

#include "chrono_multidomain/ChApiMultiDomain.h"
#include "chrono/core/ChMatrix.h"

namespace chrono {
namespace multidomain {

/// @addtogroup chrono_multidomain
/// @{

/// Class that wraps MPI_request structure, allowing to use
/// the ChMPI functions without having to include MPI headers.

class ChApiMultiDomain ChMPIrequest
{
public:
	ChMPIrequest();
	~ChMPIrequest();
	void* mpireq;
};

/// Class that wraps MPI_status structure, allowing to use
/// the ChMPI functions without having to include MPI headers.

class ChApiMultiDomain ChMPIstatus
{
public:
	ChMPIstatus();
	~ChMPIstatus();
	void* mpistat;
};



/// Class that defines useful utility functions based 
/// on Message Passing Interface (MPI) for cluster
/// computing. This class wraps the most common MPI 
/// functions for send/receive, expecially adapted for 
/// communicating ChMatrix objects and binary streams.

class ChApiMultiDomain ChMPI
{
public:
	/// Initialize MPI context. Must be called at the beginning of the program.
	static int Init(int* argc,char **argv[]);

	/// Terminate MPI context. Must be called at the end of the program.
	static int Finalize();

	/// Return the total number of processes
	static int CommSize();

	/// Return the ID of this process
	static int CommRank();

	/// Wait until a request is complete (for nonblocking send or receive).
	static int Wait(ChMPIrequest* mreq, ChMPIstatus* mstatus);

	/// Test if a request is complete (for nonblocking send or receive).
	static bool Test(ChMPIrequest* mreq, ChMPIstatus* mstatus);

	/// Modes for send/receive
	enum eCh_mpiCommMode{
						 MPI_STANDARD = 0,
						 MPI_BUFFERED,
						 MPI_SYNCHRONOUS,
						 MPI_READY};

	/// Send a ChMatrix to the process with rank destID.
	/// Blocking version, wrapping MPI_Send, MPI_Bsend, MPI_Rsend, MPI_Ssend   modes.
	static int SendMatrix_blocking(
							int destID,							 ///< destination rank
							ChMatrixDynamic<double>& source_matr,///< source matrix
							eCh_mpiCommMode mmode				 ///< send mode
							);	

	/// Send a ChMatrix to the process with rank destID.
	/// Non-blocking version, wrapping MPI_Isend, MPI_Ibsend, MPI_Irsend, MPI_Issend.
	static int SendMatrix_nonblocking(
							int destID,							 ///< destination rank
							ChMatrixDynamic<double>& source_matr,///< source matrix
							eCh_mpiCommMode mmode,				 ///< send mode
							ChMPIrequest* mreq = 0				 ///< if nonblocking, must use this
	);

	/// Receive a ChMatrix from the process with rank sourceID.
	/// Blocking version.
	/// NOTE: The number of columns/columns of the destination matrix 
	/// must be the same of the sourrce matrix (the user must take 
	/// care of this) otherwise an error happens. 
	static int ReceiveMatrix_blocking(
							int sourceID,						///< source rank
							ChMatrixDynamic<double>& dest_matr,	///< destination matrix (must already have proper size!)
							ChMPIstatus* mstatus				///< return status here
							);

	/// Receive a ChMatrix from the process with rank sourceID.
	/// Non-blocking version. Later, a Wait() or Test() might be needed.
	/// NOTE: The number of columns/columns of the destination matrix 
	/// must be the same of the sourrce matrix (the user must take 
	/// care of this) otherwise an error happens. 	
	static int ReceiveMatrix_nonblocking(
							int sourceID,						///< source rank
							ChMatrixDynamic<double>& dest_matr,	///< destination matrix (must already have proper size!)
							ChMPIrequest* mreq = 0				///< if nonblocking, must use this
						);


	/// Send a std::string to the process with rank destID (it
	/// could be a binary buffer too).
	/// Blocking version.
	static int SendString_blocking(
							int destID,					///< destination rank
							std::string& source_str,	///< source string
							eCh_mpiCommMode mmode		///< send mode
							);

	/// Send a std::string to the process with rank destID (it
	/// could be a binary buffer too).
	/// Non-blocking version.
	static int SendString_nonblocking(
							int destID,					///< destination rank
							std::string& source_str,	///< source string
							eCh_mpiCommMode mmode,		///< send mode
							ChMPIrequest* mreq			///< if nonblocking, must use this
	);
	static int SendString_nonblocking2(
		int destID,					///< destination rank
		std::string& source_str,	///< source string
		eCh_mpiCommMode mmode,		///< send mode
		ChMPIrequest* mreq			///< if nonblocking, must use this
	);

	/// Receive a std::string from the process with rank sourceID.
	/// Blocking version.
	/// NOTE: the non-blocking version is not yet available because it
	/// would require two steps, first a nonblocking msg to send the size, then another to receive, 
	/// now the size is get via a blocking MPI_Probe() followed by a MPI_Recv
	/// The size of the source matrix can be unknown: the destination
	/// string buffer is properly resized when receiving.
	static int ReceiveString_blocking(
							int sourceID,				///< source rank
							std::string& dest_str,		///< destination string - will be resized
							ChMPIstatus* mstatus		///< return status here
							);

	static int ReceiveString_blocking2(
		int sourceID,				///< source rank
		std::string& dest_str,		///< destination string - will be resized
		ChMPIstatus* mstatus
	);

	/// Global barrier. Calling MPI_Barrier(MPI_COMM_WORLD);
	static int Barrier();

	/// Wait all non blocking operations are finished. Calling MPI_Waitall(MPI_COMM_WORLD);
	static int WaitAll(int arraysize, ChMPIrequest requests[], ChMPIstatus statuses[]);


	/// Reduction (combines values from all processes and distributes the result back 
	/// to all processes)
	/// This cause a global synchronization. 
	enum class eCh_mpiReduceOperation {
		MPI_max = 0,
		MPI_min,
		MPI_sum,
		MPI_prod
	};
	static int ReduceAll(double send, double& received_result, eCh_mpiReduceOperation operation = eCh_mpiReduceOperation::MPI_sum);
};

class Ch_test_mpi
{
public:
	void run_test();

	//void run_test_matrix();


};


}  // end namespace multidomain
}  // end namespace chrono

#endif
