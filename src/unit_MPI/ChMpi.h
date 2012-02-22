#ifndef CHMPI_H
#define CHMPI_H

//////////////////////////////////////////////////
//  
//   ChMpi.h
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "mpi.h"
#include "unit_MPI/ChApiMPI.h"
#include "core/ChLog.h"
#include "core/ChMath.h"


namespace chrono 
{

/// Class that wraps MPI_request structure, allowing to use
/// the ChMPI functions without having to include MPI headers.

class ChApiMPI ChMPIrequest
{
public:
	ChMPIrequest();
	ChMPIrequest(ChMPIrequest const& rhs); // only for supporting std::vector, do not copy
	~ChMPIrequest();
	void* mpireq;
};

/// Class that wraps MPI_status structure, allowing to use
/// the ChMPI functions without having to include MPI headers.

class ChApiMPI ChMPIstatus
{
public:
	ChMPIstatus();
	~ChMPIstatus();
	void* mpistat;
};


/// Class that wraps a basic MPI_File object, allowing to use
/// the ChMPI functions without having to include MPI headers.

class ChApiMPI ChMPIfile
{
public:
	ChMPIfile(char* filename, int flags = CHMPI_MODE_WRONLY | CHMPI_MODE_CREATE );
	~ChMPIfile();
	void* mpifile;

	// for creation mode flags
	static const int CHMPI_MODE_RDONLY = (1L << 0);
	static const int CHMPI_MODE_RDWR   = (1L << 1);
	static const int CHMPI_MODE_WRONLY = (1L << 2);
	static const int CHMPI_MODE_CREATE = (1L << 3);

	void WriteOrdered(char* buf, int length);

	static bool FileDelete(char* filename);
};



/// Class that defines useful utility functions based 
/// on Message Passing Interface (MPI) for cluster
/// computing. This class wraps the most common MPI 
/// functions for send/receive, expecially adapted for 
/// communicating ChMatrix objects and binary streams.

class ChApiMPI ChMPI
{
public:
	/// Initialize MPI context. Must be called at the beginning of the program.
	static int Init(int argc,char *argv[]);

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
	static int SendMatrix(int destID,						///< destination rank
							ChMatrix<double>& source_matr,	///< source matrix
							eCh_mpiCommMode mmode,			///< send mode
							bool nonblocking=false,			///< set true for nonblocking (immediate) send
							ChMPIrequest* mreq= 0			///< if nonblocking=true, must use this
							);			

	/// Receive a ChMatrix from the process with rank sourceID.
	/// The number of columns/columns of the destination matrix 
	/// must be the same of the sourrce matrix (the user must take 
	/// care of this) otherwise an error happens. 
	static int ReceiveMatrix(int sourceID,					///< source rank
							ChMatrix<double>& dest_matr,	///< destination matrix (must already have proper size!)
							ChMPIstatus* mstatus,			///< return status here, for nonblocking=false (otherwise use 0)
							bool nonblocking=false,			///< set true for nonblocking (immediate) receive
							ChMPIrequest* mreq= 0			///< if nonblocking=true, must use this
							);


	/// Send a std::string to the process with rank destID (it
	/// could be a binary buffer too).
	static int SendString(int destID,						///< destination rank
							std::string& source_str,		///< source string
							eCh_mpiCommMode mmode,			///< send mode
							bool nonblocking=false,			///< set true for nonblocking (immediate) send
							ChMPIrequest* mreq= 0			///< if nonblocking=true, must use this
							);
	/// Receive a std::string from the process with rank sourceID.
	/// The size of the source matrix can be unknown: the destination
	/// string buffer is properly resized when receiving.
	static int ReceiveString(int sourceID,					///< source rank
							std::string& dest_str,			///< destination string - will be resized
							ChMPIstatus* mstatus			///< return status here
							);

	/// Send a std::vector<char> (a buffer of bytes) to the process with rank destID 
	static int SendBuffer(int destID,						///< destination rank
							std::vector<char>& source_buf,	///< source buffer
							eCh_mpiCommMode mmode,			///< send mode
							bool nonblocking=false,			///< set true for nonblocking (immediate) send
							ChMPIrequest* mreq= 0			///< if nonblocking=true, must use this
							);
	/// Receive a std::vector<char> (a buffer of bytes) from the process with rank sourceID.
	/// Optionally, the size of the buffer can be unknown: the destination
	/// vector buffer is properly resized when receiving (with small overhead due to probing).
	static int ReceiveBuffer(int sourceID,					///< source rank
							std::vector<char>& dest_buf,	///< destination buffer - will be resized
							ChMPIstatus* mstatus,			///< return status here
							int size =-1					///< size in bytes (if unknown, use -1)
							);
};




} // END_OF_NAMESPACE____


#endif  // END of ChMpi.h 
