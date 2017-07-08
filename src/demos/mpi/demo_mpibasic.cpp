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
//   Demo code about
//
//     - using MPI basic features.
//
//	 NOTE! this program should be copied
//   on multiple hosts of a cluster and executed
//   using the launcher utility of the MPICH2
//   toolchain (ex. mpiexec or wmpiexec.exe).
// =============================================================================

#include "physics/ChBody.h"
#include "unit_MPI/ChMpi.h"
#include <iostream>
#include <sstream>

#include "mpi.h"

using namespace chrono;

int main(int argc, char* argv[]) {
    // Initialize the MPI functionality. Use the CHMPI static functions.
    CHMPI::Init(argc, argv);

    // Get infos about how many processes are launched,
    // and about the ID of this specific process.
    int numprocs = CHMPI::CommSize();
    int myid = CHMPI::CommRank();

    if (myid == 0)
        GetLog() << "Number of processes : " << numprocs << "\n";

    if (numprocs < 2) {
        GetLog() << "Use at least 2 processes! \n";
        CHMPI::Finalize();
        return 0;
    }

    //
    // TEST 1   -   send and receive ChMatrix<> object
    //

    if (myid == 0)  // sender
    {
        ChMatrixDynamic<> mmatr(3, 4);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 4; j++)
                mmatr(i, j) = i * j;
        GetLog() << "Id 0: sending ChMatrix: " << mmatr << "\n";
        CHMPI::SendMatrix(1, mmatr, CHMPI::MPI_STANDARD, false, 0);
    }

    if (myid == 1)  // receiver
    {
        ChMatrixDynamic<> mmatr(3, 4);  // must already have the proper size
        CHMPIstatus mstatus;
        CHMPI::ReceiveMatrix(0, mmatr, &mstatus, false, 0);
        GetLog() << "Id 1: received ChMatrix: " << mmatr << "\n";
    }

    //
    // TEST 2   -   send and receive strings
    //

    if (myid == 0)  // sender
    {
        std::stringstream mstrbuf(std::stringstream::in | std::stringstream::out);
        mstrbuf << "The quick lazy fog jumped on the brown dog.";
        std::string mstr = mstrbuf.rdbuf()->str();
        GetLog() << "Id 0: sending string: " << mstr << "\n";
        CHMPI::SendString(1, mstr, CHMPI::MPI_STANDARD, false, 0);
    }

    if (myid == 1)  // receiver
    {
        // std::stringstream mstrbuf2(std::stringstream::in | std::stringstream::out);
        CHMPIstatus mstatus;
        std::string mstr;
        CHMPI::ReceiveString(0, mstr, &mstatus);

        GetLog() << "Id 1: received string: " << mstr << "\n";
    }

    //
    // TEST 3   -   send and receive whatever serializable chrono object
    //              i.e. objects that implements StreamIN() and StreamOUT()
    //              i.e. objects that can be saved via << and >>.
    //
    /*
        if (myid==0) // sender
        {
            std::vector<char> outstream;
            ChStreamOutBinaryVector outstreamwrapper(&outstream);

            ChVector<> mv(34,11,45.34);
            outstreamwrapper << mv;	// serialize object to binary stream, in memory.

            ChBody mybody;
            outstreamwrapper.AbstractWrite(&mybody); 	// serialize object to binary stream, in memory.

            GetLog() << "Id 0: sending serialized ChVector: " << mv << "\n";
            CHMPI::SendBuffer(1, outstream, CHMPI::MPI_STANDARD, false,0);
        }

        if (myid==1) // receiver
        {
            CHMPIstatus mstatus;
            std::vector<char> instream;
            ChStreamInBinaryVector instreamwrapper(&instream);

            CHMPI::ReceiveBuffer(0, instream, &mstatus);

            ChVector<> mv;
            instreamwrapper >> mv;	// deserialize object from binary stream.

            ChObj* myobj;
            instreamwrapper.AbstractReadCreate(&myobj); // deserialize unknown object from binary stream.

            if (myobj) GetLog() << "Id 1: created obj of type:" << myobj->FactoryNameTag() << "\n";
            GetLog() << "Id 1: received serialized ChVector: " << mv << "\n";
        }
    */

    // Terminate the MPI functionality.
    CHMPI::Finalize();

    return 0;
}
