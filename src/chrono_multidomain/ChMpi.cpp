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

#include "mpi.h"
#include <iostream>
#include <sstream>
#include <math.h>

#include "chrono_multidomain/ChMpi.h"


namespace chrono {
namespace multidomain {



using namespace std;



ChMPIrequest::ChMPIrequest()
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








int ChMPI::Init(int* argc,char **argv[])
{
	return MPI_Init(argc,argv);
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

bool ChMPI::Test(ChMPIrequest* mreq, ChMPIstatus* mstatus)
{
	int flag;
	MPI_Test( (MPI_Request*) mreq->mpireq, &flag,  (MPI_Status*) mstatus->mpistat);
	return (flag!=0);
}


int ChMPI::SendMatrix_blocking(int destID, 
					  ChMatrixDynamic<double>& source_matr,
					  eCh_mpiCommMode mmode)
{
	struct Matrixstruct
	{
		int rows;
		int cols;
		double* vals;
	};
	Matrixstruct mmatr; 
	mmatr.rows = source_matr.rows();
	mmatr.cols = source_matr.rows();
	mmatr.vals = source_matr.data();

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

	MPI_Type_free( &Matrixtype);

	return err;
}


int ChMPI::SendMatrix_nonblocking(
									int destID,
									ChMatrixDynamic<double>& source_matr,
									eCh_mpiCommMode mmode,
									ChMPIrequest* mreq)
{
	struct Matrixstruct
	{
		int rows;
		int cols;
		double* vals;
	};
	Matrixstruct mmatr;
	mmatr.rows = source_matr.rows();
	mmatr.cols = source_matr.rows();
	mmatr.vals = source_matr.data();

	MPI_Datatype Matrixtype;
	MPI_Datatype type[3] = { MPI_INT, MPI_INT,            MPI_DOUBLE };
	int      blocklen[3] = { 1,       1, mmatr.rows * mmatr.cols };
	MPI_Aint disp[3];
	MPI_Aint base;
	// compute displacements of structure components 
	MPI_Address(&mmatr.rows, disp);
	MPI_Address(&mmatr.cols, disp + 1);
	MPI_Address(mmatr.vals, disp + 2);
	base = disp[0];
	for (int i = 0; i < 3; i++) disp[i] -= base;
	MPI_Type_struct(3, blocklen, disp, type, &Matrixtype);
	MPI_Type_commit(&Matrixtype);

	int err = 0;

	switch (mmode)
	{
	case ChMPI::MPI_STANDARD:
		err = MPI_Isend(&mmatr, 1, Matrixtype, destID, 1001, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
		break;
	case ChMPI::MPI_BUFFERED:
		err = MPI_Ibsend(&mmatr, 1, Matrixtype, destID, 1001, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
		break;
	case ChMPI::MPI_SYNCHRONOUS:
		err = MPI_Issend(&mmatr, 1, Matrixtype, destID, 1001, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
		break;
	case ChMPI::MPI_READY:
		err = MPI_Irsend(&mmatr, 1, Matrixtype, destID, 1001, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
		break;
	default:
		break;
	}

	MPI_Type_free(&Matrixtype);

	return err;
}

int ChMPI::ReceiveMatrix_blocking( int sourceID, 
					     ChMatrixDynamic<double>& dest_matr,
						 ChMPIstatus* mstatus)
{
	struct Matrixstruct
	{
		int rows;
		int cols;
		double* vals;
	};
	Matrixstruct mmatr; 
	//mmatr.rows = dest_matr.GetRows();
	//mmatr.cols = dest_matr.GetColumns();
	mmatr.vals = dest_matr.data();
	int nelements = dest_matr.rows() * dest_matr.cols();

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
	
	int err = MPI_Recv( &mmatr, 1, Matrixtype, sourceID, 1001, MPI_COMM_WORLD, (MPI_Status*)mstatus->mpistat);
	
	//int nnelements;
	//MPI_Get_count ( (MPI_Status*)mstatus->mpistat, Matrixtype, &nnelements );
	//GetLog() << " n.elements:" << nnelements << "\n";

	MPI_Type_free( &Matrixtype);

	return err;
}


int ChMPI::ReceiveMatrix_nonblocking(int sourceID,
	ChMatrixDynamic<double>& dest_matr,
	ChMPIrequest* mreq)
{
	struct Matrixstruct
	{
		int rows;
		int cols;
		double* vals;
	};
	Matrixstruct mmatr;
	//mmatr.rows = dest_matr.GetRows();
	//mmatr.cols = dest_matr.GetColumns();
	mmatr.vals = dest_matr.data();
	int nelements = dest_matr.rows() * dest_matr.cols();

	MPI_Datatype Matrixtype;
	MPI_Datatype type[3] = { MPI_INT, MPI_INT,  MPI_DOUBLE };
	int      blocklen[3] = { 1,       1,	nelements };
	MPI_Aint disp[3];
	MPI_Aint base;
	// compute displacements of structure components 
	MPI_Address(&mmatr.rows, disp);
	MPI_Address(&mmatr.cols, disp + 1);
	MPI_Address(mmatr.vals, disp + 2);
	base = disp[0];
	for (int i = 0; i < 3; i++) disp[i] -= base;
	MPI_Type_struct(3, blocklen, disp, type, &Matrixtype);
	MPI_Type_commit(&Matrixtype);

	int err = MPI_Irecv(&mmatr, 1, Matrixtype, sourceID, 1001, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);

	//int nnelements;
	//MPI_Get_count ( (MPI_Status*)mstatus->mpistat, Matrixtype, &nnelements );
	//GetLog() << " n.elements:" << nnelements << "\n";

	MPI_Type_free(&Matrixtype);

	return err;
}




int ChMPI::SendString_blocking(int destID,					///< destination rank
							std::string& source_str,	///< source string
							eCh_mpiCommMode mmode		///< send mode
							)
{
	char* data = (char*)source_str.data(); // should not access directly std::string data, but this is efficient!
	int nbytes = (int)source_str.size();
	int err = 0;

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

	return err;
}

int ChMPI::SendString_nonblocking(
							int destID,					///< destination rank
							std::string& source_str,	///< source string
							eCh_mpiCommMode mmode,		///< send mode
							ChMPIrequest* mreq			///< if nonblocking=true, must use this
)
{
	char* data = (char*)source_str.data(); // should not access directly std::string data, but this is efficient!
	int nbytes = (int)source_str.size();
	int err = 0;

	switch (mmode)
	{
	case ChMPI::MPI_STANDARD:
		err = MPI_Isend(data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
		break;
	case ChMPI::MPI_BUFFERED:
		err = MPI_Ibsend(data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
		break;
	case ChMPI::MPI_SYNCHRONOUS:
		err = MPI_Issend(data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
		break;
	case ChMPI::MPI_READY:
		err = MPI_Irsend(data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
		break;
	default:
		break;
	}

	return err;
}

/*
int ChMPI::SendString_nonblocking2(
	int destID,					///< destination rank
	std::string& source_str,	///< source string
	eCh_mpiCommMode mmode,		///< send mode
	ChMPIrequest* mreq			///< if nonblocking=true, must use this
)
{
	char* data = (char*)source_str.data(); // should not access directly std::string data, but this is efficient!
	int nbytes = (int)source_str.size();
	int err = 0;

	switch (mmode)
	{
	case ChMPI::MPI_STANDARD:
		err = MPI_Isend(&nbytes, 1, MPI_INT, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
		err = MPI_Isend(data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
		break;
	case ChMPI::MPI_BUFFERED:
		err = MPI_Ibsend(&nbytes, 1, MPI_INT, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
		err = MPI_Ibsend(data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
		break;
	case ChMPI::MPI_SYNCHRONOUS:
		err = MPI_Issend(&nbytes, 1, MPI_INT, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
		err = MPI_Issend(data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
		break;
	case ChMPI::MPI_READY:
		err = MPI_Irsend(&nbytes, 1, MPI_INT, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
		err = MPI_Irsend(data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
		break;
	default:
		break;
	}

	return err;
}
*/

int ChMPI::ReceiveString_blocking(
							int sourceID,				///< source rank
							std::string& dest_str,	///< destination string - will be resized
							ChMPIstatus* mstatus
							)
{
	MPI_Status status;
	int incoming_msg_size =0;
	MPI_Probe(sourceID, 1002,	MPI_COMM_WORLD, &status);
    MPI_Get_count(&status, MPI_BYTE, &incoming_msg_size);
	//GetLog() << "   size:" << incoming_msg_size << "   \n";

	dest_str.resize(incoming_msg_size);

	void* data = (void*)dest_str.data(); // should not access directly std::string data! but this is efficient!

	int err = 0;
	if (incoming_msg_size)
	{
		err = MPI_Recv( data, incoming_msg_size, MPI_BYTE, sourceID, 1002, MPI_COMM_WORLD, (MPI_Status*)mstatus->mpistat);
	}
	return err;
}

/*
int ChMPI::ReceiveString_blocking2(
	int sourceID,				///< source rank
	std::string& dest_str,		///< destination string - will be resized
	ChMPIstatus* mstatus
) {
	int recv_length; 
	MPI_Recv(&recv_length, 1, MPI_INT, sourceID, 1002, MPI_COMM_WORLD, (MPI_Status*)mstatus->mpistat);
	dest_str.resize(recv_length); 
	int err = MPI_Recv(&dest_str[0], recv_length, MPI_BYTE, sourceID, 1002, MPI_COMM_WORLD, (MPI_Status*)mstatus->mpistat);
	return err;
}
*/

int ChMPI::Barrier() {
	return MPI_Barrier(MPI_COMM_WORLD);
}

int ChMPI::WaitAll(int arraysize, ChMPIrequest requests[], ChMPIstatus statuses[]) {
	for (int i = 0; i < arraysize; ++i)
		int res = MPI_Wait((MPI_Request*)requests[i].mpireq, (MPI_Status*)statuses[i].mpistat);
	return 1;
}


int ChMPI::ReduceAll(double send, double& received_result, eCh_mpiReduceOperation operation ) {
	int mpi_operation = MPI_NO_OP;
	switch (operation) {
	case eCh_mpiReduceOperation::MPI_max:
		mpi_operation = MPI_MAX; break;
	case eCh_mpiReduceOperation::MPI_min:
		mpi_operation = MPI_MIN; break;
	case eCh_mpiReduceOperation::MPI_sum:
		mpi_operation = MPI_SUM; break;
	case eCh_mpiReduceOperation::MPI_prod:
		mpi_operation = MPI_PROD; break;
	}

	return MPI_Allreduce(&send, &received_result, 1, MPI_DOUBLE, mpi_operation, MPI_COMM_WORLD);
}

////////////////////////////////////////////////////

double f(double);

double f(double a)
{
    return (4.0 / (1.0 + a*a));
}

int ccmain(int argc,char *argv[])
{
    int done = 0, n, myid, numprocs, i;
    double PI25DT = 3.141592653589793238462643;
    double mypi, pi, h, sum, x;
    double startwtime = 0.0, endwtime;
    int  namelen;
    char processor_name[MPI_MAX_PROCESSOR_NAME];

    MPI_Init(&argc,&argv);
    MPI_Comm_size(MPI_COMM_WORLD,&numprocs);
    MPI_Comm_rank(MPI_COMM_WORLD,&myid);
    MPI_Get_processor_name(processor_name,&namelen);

    /*
    fprintf(stdout,"Process %d of %d is on %s\n",
	    myid, numprocs, processor_name);
    fflush(stdout);
    */

    while (!done) {
        if (myid == 0) {
            fprintf(stdout, "Enter the number of intervals: (0 quits) ");
	    fflush(stdout);
            if (scanf("%d",&n) != 1) {
		fprintf( stdout, "No number entered; quitting\n" );
		n = 0;
	    }
	    startwtime = MPI_Wtime();
        }
        MPI_Bcast(&n, 1, MPI_INT, 0, MPI_COMM_WORLD);
        if (n == 0)
            done = 1;
        else {
            h   = 1.0 / (double) n;
            sum = 0.0;
            for (i = myid + 1; i <= n; i += numprocs) {
                x = h * ((double)i - 0.5);
                sum += f(x);
            }
            mypi = h * sum;
            MPI_Reduce(&mypi, &pi, 1, MPI_DOUBLE, MPI_SUM, 0, MPI_COMM_WORLD);

            if (myid == 0) {
                printf("pi is approximately %.16f, Error is %.16f\n",
                       pi, fabs(pi - PI25DT));
		endwtime = MPI_Wtime();
		printf("wall clock time = %f\n", endwtime-startwtime);	       
		fflush( stdout );
	    }
        }
    }
    MPI_Finalize();
    return 0;
}


void Ch_test_mpi::run_test()
{
	int foo=0; char* faa; faa="";
	ccmain(foo, &faa);
}



}  // end namespace multidomain
}  // end namespace chrono



