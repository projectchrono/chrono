//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChMpi.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 

#include "mpi.h"
#include <iostream>
#include <sstream>
#include <math.h>

#include "ChMpi.h"


#ifndef CH_API_COMPILE_UNIT_MPI
#error Warning! You are compiling the MPI unit of Chrono::Engine, \
	so you need to define CH_API_COMPILE_UNIT_MPI (add that symbol \
	to the compiler defines, for all compiled files in this unit). 
#endif 


using namespace std;


namespace chrono
{


ChMPIrequest::ChMPIrequest()
{
	this->mpireq = new MPI_Request;
}
ChMPIrequest::ChMPIrequest(ChMPIrequest const& rhs)
{
	this->mpireq = new MPI_Request;
}
ChMPIrequest::~ChMPIrequest()
{
	MPI_Request* mr = static_cast<MPI_Request*>(this->mpireq);
	delete (mr); this->mpireq=0;
}

ChMPIstatus::ChMPIstatus()
{
	this->mpistat = new MPI_Status;
}

ChMPIstatus::~ChMPIstatus()
{
	MPI_Status* mr = static_cast<MPI_Status*>(this->mpistat);
	delete (mr); this->mpistat=0;
}


ChMPIfile::ChMPIfile(char* filename, int flags)
{
	this->mpifile = new MPI_File;
	
	int amode = 0;
	if (flags & CHMPI_MODE_RDONLY)	amode |= MPI_MODE_RDONLY;
	if (flags & CHMPI_MODE_RDWR)	amode |= MPI_MODE_RDWR;
	if (flags & CHMPI_MODE_WRONLY)	amode |= MPI_MODE_WRONLY;
	if (flags & CHMPI_MODE_CREATE)	amode |= MPI_MODE_CREATE;

	int rc = MPI_File_open( MPI_COMM_WORLD, filename, amode, MPI_INFO_NULL, ((MPI_File*)this->mpifile) );
    if (rc) {
       throw ChException("Unable to open MPI file.");
    }
}

ChMPIfile::~ChMPIfile()
{
	if (this->mpifile)
		MPI_File_close( (MPI_File*)this->mpifile );

	MPI_File* mf = static_cast<MPI_File*>(this->mpifile);
	delete (mf); this->mpifile=0;
}

void ChMPIfile::WriteOrdered(char* buf, int length)
{
	MPI_Status status;
	int rc = MPI_File_write_ordered( (*(MPI_File*)this->mpifile), buf, length, MPI_ChAR, &status );
	if (rc) {
       throw ChException("Error while doing MPI write ordered.");
    }
}

bool ChMPIfile::FileDelete(char* filename)
{
	int rc = MPI_File_delete( filename , MPI_INFO_NULL);
	if (!rc) 
		return true;
	if (rc == MPI_ERR_NO_SUCH_FILE)
		return false;
	else
		throw ChException("Error while doing MPI file delete");
	return false;
}




///////////



int ChMPI::Init(int argc,char *argv[])
{
	return MPI_Init(&argc,&argv);
}

int ChMPI::Finalize()
{
	return MPI_Finalize();
}

int ChMPI::CommSize()
{
	int numprocs=0;
	MPI_Comm_size(MPI_COMM_WORLD,&numprocs);
	return numprocs;
}

int ChMPI::CommRank()
{
	int myid;
	MPI_Comm_rank(MPI_COMM_WORLD,&myid);
	return myid;
}

int ChMPI::Wait(ChMPIrequest* mreq, ChMPIstatus* mstatus)
{
	return MPI_Wait( (MPI_Request*) mreq->mpireq,  (MPI_Status*) mstatus->mpistat);
}

double ChMPI::Wtime()
{
	return MPI_Wtime();
}

bool ChMPI::Test(ChMPIrequest* mreq, ChMPIstatus* mstatus)
{
	int flag;
	MPI_Test( (MPI_Request*) mreq->mpireq, &flag,  (MPI_Status*) mstatus->mpistat);
	return (flag!=0);
}

void ChMPI::Barrier()
{
	MPI_Barrier(MPI_COMM_WORLD);
}

int ChMPI::SendMatrix(int destID, 
					  ChMatrix<double>& source_matr, 
					  eCh_mpiCommMode mmode, 
					  bool nonblocking,
					  ChMPIrequest* mreq)
{
	assert ((!nonblocking && !mreq) || (nonblocking && mreq));

	struct Matrixstruct
	{
		int rows;
		int cols;
		double* vals;
	};
	Matrixstruct mmatr; 
	mmatr.rows = source_matr.GetRows();
	mmatr.cols = source_matr.GetColumns();
	mmatr.vals = source_matr.GetAddress();

	MPI_Datatype Matrixtype;
	MPI_Datatype type[3] = {MPI_INT, MPI_INT,            MPI_DOUBLE};
	int      blocklen[3] = {      1,       1, mmatr.rows*mmatr.cols};
	MPI_Aint disp[3];
	MPI_Aint base;
	// compute displacements of structure components 
	MPI_Address( &mmatr.rows, disp);
	MPI_Address( &mmatr.cols, disp+1);
	MPI_Address( mmatr.vals,  disp+2);
	base = disp[0];
	for (int i=0; i <3; i++) disp[i] -= base;
	MPI_Type_struct( 3, blocklen, disp, type, &Matrixtype);
	MPI_Type_commit( &Matrixtype);
	
	int err = 0;

	if (nonblocking == false)
	{
		switch (mmode)
		{
			case ChMPI::MPI_STANDARD:
				err = MPI_Send( &mmatr, 1, Matrixtype, destID, 1001, MPI_COMM_WORLD);
				break;
			case ChMPI::MPI_BUFFERED:
				err = MPI_Bsend( &mmatr, 1, Matrixtype, destID, 1001, MPI_COMM_WORLD);
				break;
			case ChMPI::MPI_SYNCHRONOUS:
				err = MPI_Ssend( &mmatr, 1, Matrixtype, destID, 1001, MPI_COMM_WORLD);
				break;
			case ChMPI::MPI_READY:
				err = MPI_Rsend( &mmatr, 1, Matrixtype, destID, 1001, MPI_COMM_WORLD);
				break;
			default:
				break;
		}
	}
	else
	{
		switch (mmode)
		{
			case ChMPI::MPI_STANDARD:
				err = MPI_Isend( &mmatr, 1, Matrixtype, destID, 1001, MPI_COMM_WORLD, (MPI_Request*) mreq->mpireq);
				break;
			case ChMPI::MPI_BUFFERED:
				err = MPI_Ibsend( &mmatr, 1, Matrixtype, destID, 1001, MPI_COMM_WORLD, (MPI_Request*) mreq->mpireq);
				break;
			case ChMPI::MPI_SYNCHRONOUS:
				err = MPI_Issend( &mmatr, 1, Matrixtype, destID, 1001, MPI_COMM_WORLD, (MPI_Request*) mreq->mpireq);
				break;
			case ChMPI::MPI_READY:
				err = MPI_Irsend( &mmatr, 1, Matrixtype, destID, 1001, MPI_COMM_WORLD, (MPI_Request*) mreq->mpireq);
				break;
			default:
				break;
		}
	}

	MPI_Type_free( &Matrixtype);

	return err;
}

int ChMPI::ReceiveMatrix( int sourceID, 
						 ChMatrix<double>& dest_matr, 
						 ChMPIstatus* mstatus,
						 bool nonblocking,
						 ChMPIrequest* mreq)
{
	assert ((!nonblocking && !mreq && mstatus) || (nonblocking && mreq && !mstatus));

	struct Matrixstruct
	{
		int rows;
		int cols;
		double* vals;
	};
	Matrixstruct mmatr; 
	//mmatr.rows = dest_matr.GetRows();
	//mmatr.cols = dest_matr.GetColumns();
	mmatr.vals = dest_matr.GetAddress();
	int nelements = dest_matr.GetRows() * dest_matr.GetColumns();

	MPI_Datatype Matrixtype;
	MPI_Datatype type[3] = {MPI_INT, MPI_INT,  MPI_DOUBLE};
	int      blocklen[3] = {      1,       1,	nelements};
	MPI_Aint disp[3];
	MPI_Aint base;
	// compute displacements of structure components 
	MPI_Address( &mmatr.rows, disp);
	MPI_Address( &mmatr.cols, disp+1);
	MPI_Address( mmatr.vals,  disp+2);
	base = disp[0];
	for (int i=0; i <3; i++) disp[i] -= base;
	MPI_Type_struct( 3, blocklen, disp, type, &Matrixtype);
	MPI_Type_commit( &Matrixtype);
	
	int err = 0;
	if (nonblocking == false)
	{
		err = MPI_Recv( &mmatr, 1, Matrixtype, sourceID, 1001, MPI_COMM_WORLD, (MPI_Status*)mstatus->mpistat);
	}
	else
	{
		err = MPI_Irecv( &mmatr, 1, Matrixtype, sourceID, 1001, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
	}

	//int nnelements;
	//MPI_Get_count ( (MPI_Status*)mstatus->mpistat, Matrixtype, &nnelements );
	//GetLog() << " n.elements:" << nnelements << "\n";

	MPI_Type_free( &Matrixtype);

	return err;
}




int ChMPI::SendString(int destID,					///< destination rank
							std::string& source_str,	///< source string
							eCh_mpiCommMode mmode,		///< send mode
							bool nonblocking,			///< set true for nonblocking (immediate) send
							ChMPIrequest* mreq			///< if nonblocking=true, must use this
							)
{
	assert ((!nonblocking && !mreq) || (nonblocking && mreq));

	char* data = (char*)source_str.data(); // should not access directly std::string data, but this is efficient!
	int nbytes = source_str.size();
	int err = 0;

	if (nonblocking == false)
	{
		switch (mmode)
		{
			case ChMPI::MPI_STANDARD:
				err = MPI_Send( data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD);
				break;
			case ChMPI::MPI_BUFFERED:
				err = MPI_Bsend( data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD);
				break;
			case ChMPI::MPI_SYNCHRONOUS:
				err = MPI_Ssend( data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD);
				break;
			case ChMPI::MPI_READY:
				err = MPI_Rsend( data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD);
				break;
			default:
				break;
		}
	}
	else
	{
		switch (mmode)
		{
			case ChMPI::MPI_STANDARD:
				err = MPI_Isend (  data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*) mreq->mpireq);
				break;
			case ChMPI::MPI_BUFFERED:
				err = MPI_Ibsend(  data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*) mreq->mpireq);
				break;
			case ChMPI::MPI_SYNCHRONOUS:
				err = MPI_Issend(  data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*) mreq->mpireq);
				break;
			case ChMPI::MPI_READY:
				err = MPI_Irsend(  data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*) mreq->mpireq);
				break;
			default:
				break;
		}
	}

	return err;
}



int ChMPI::ReceiveString(int sourceID,				///< source rank
							std::string& dest_str,	///< destination string - will be resized
							ChMPIstatus* mstatus
							)
{
	MPI_Status status;
	int incoming_msg_size =0;
	MPI_Probe(sourceID, 1002,	MPI_COMM_WORLD, &status);
    MPI_Get_count(&status, MPI_BYTE, &incoming_msg_size);

	dest_str.resize(incoming_msg_size);

	//GetLog() << "   ReceiveString size=" << dest_str.size() << "\n";

	void* data = (void*)dest_str.data(); // should not access directly std::string data! but this is efficient!

	int err = 0;
	//if (incoming_msg_size)
	//{
		err = MPI_Recv( data, incoming_msg_size, MPI_BYTE, sourceID, 1002, MPI_COMM_WORLD, (MPI_Status*)mstatus->mpistat);
	//}
	return err;
}



int ChMPI::SendBuffer(int destID,						///< destination rank
							std::vector<char>& source_buf,///< source buffer
							eCh_mpiCommMode mmode,		///< send mode
							bool nonblocking,			///< set true for nonblocking (immediate) send
							ChMPIrequest* mreq			///< if nonblocking=true, must use this
							)
{
	assert ((!nonblocking && !mreq) || (nonblocking && mreq));

	int nbytes = source_buf.size();
	char* data;
	if (nbytes)
		data = (char*)&(source_buf[0]); // stl vectors are assured to be sequential
	else
		data = ""; // stub, in case null length messages, stl vector has no [0] element address

	int err = 0;

	if (nonblocking == false)
	{
		switch (mmode)
		{
			case ChMPI::MPI_STANDARD:
				err = MPI_Send( data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD);
				break;
			case ChMPI::MPI_BUFFERED:
				err = MPI_Bsend( data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD);
				break;
			case ChMPI::MPI_SYNCHRONOUS:
				err = MPI_Ssend( data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD);
				break;
			case ChMPI::MPI_READY:
				err = MPI_Rsend( data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD);
				break;
			default:
				break;
		}
	}
	else
	{
		switch (mmode)
		{
			case ChMPI::MPI_STANDARD:
				err = MPI_Isend (  data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*) mreq->mpireq);
				break;
			case ChMPI::MPI_BUFFERED:
				err = MPI_Ibsend(  data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*) mreq->mpireq);
				break;
			case ChMPI::MPI_SYNCHRONOUS:
				err = MPI_Issend(  data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*) mreq->mpireq);
				break;
			case ChMPI::MPI_READY:
				err = MPI_Irsend(  data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*) mreq->mpireq);
				break;
			default:
				break;
		}
	}

	return err;
}



int ChMPI::ReceiveBuffer(int sourceID,				///< source rank
							std::vector<char>& dest_buf, ///< destination buffer - will be resized
							ChMPIstatus* mstatus,
							int size
							)
{
	MPI_Status status;
	int nbytes =0;
	if (size == -1)
	{
		MPI_Probe(sourceID, 1002,	MPI_COMM_WORLD, &status);
		MPI_Get_count(&status, MPI_BYTE, &nbytes);
	}
	else
	{
		nbytes = size;
	}

	dest_buf.resize(nbytes);

	//GetLog() << "   ReceiveBuffer size=" << nbytes << "\n";

	void* data;
	if (nbytes)
		data = (char*)&(dest_buf[0]); // stl vectors are assured to be sequential. // should not access directly std::vector data, but this is efficient!
	else
		data = (char*)""; // stub, in case null length messages, stl vector has no [0] element address
	
	int err = 0;
	//if (nbytes)
	//{
		err = MPI_Recv( data, nbytes, MPI_BYTE, sourceID, 1002, MPI_COMM_WORLD, (MPI_Status*)mstatus->mpistat);
	//}
	return err;
}








} // END_OF_NAMESPACE____


////// end
