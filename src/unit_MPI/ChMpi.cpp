///////////////////////////////////////////////////
//
//   ChMpi.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "mpi.h"
#include <iostream>
#include <math.h>

#include "unit_MPI/ChMpi.h"

using namespace std;


namespace chrono
{



double f(double);

double f(double a)
{
    return (4.0 / (1.0 + a*a));
}

int ccmain()
{
    int n, myid, numprocs, i;
    double PI25DT = 3.141592653589793238462643;
    double mypi, pi, h, sum, x;
    double startwtime = 0.0, endwtime;
    int  namelen;
    char processor_name[MPI_MAX_PROCESSOR_NAME];

   // MPI::Init(argc,argv);
	MPI::Init();
    numprocs = MPI::COMM_WORLD.Get_size();
    myid     = MPI::COMM_WORLD.Get_rank();
    MPI::Get_processor_name(processor_name,namelen);

	GetLog() << "Process " << myid << " of " << numprocs << " is on " << processor_name << "\n";
 //   cout << "Process " << myid << " of " << numprocs << " is on " <<
	

    n = 10000;			/* default # of rectangles */
    if (myid == 0)
	startwtime = MPI::Wtime();

    MPI::COMM_WORLD.Bcast(&n, 1, MPI_INT, 0);

    h   = 1.0 / (double) n;
    sum = 0.0;
    /* A slightly better approach starts from large i and works back */
    for (i = myid + 1; i <= n; i += numprocs)
    {
	x = h * ((double)i - 0.5);
	sum += f(x);
    }
    mypi = h * sum;

    MPI::COMM_WORLD.Reduce(&mypi, &pi, 1, MPI_DOUBLE, MPI_SUM, 0);

    if (myid == 0) {
		endwtime = MPI::Wtime();
		GetLog() << "pi is approximately " << pi << " Error is " << fabs(pi - PI25DT) << "\n";
		GetLog() << "wall clock time = " << endwtime-startwtime << "\n";
    }

    MPI::Finalize();
    return 0;
}


void Ch_test_mpi::run_test()
{
	ccmain();
}




} // END_OF_NAMESPACE____


////// end
