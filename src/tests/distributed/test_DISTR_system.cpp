/*
 * test_DISTR_system.cpp
 *
 *  Created on: Jan 4, 2017
 *      Author: nic
 */
#include "chrono_distributed/physics/ChSystemDistr.h"
#include "chrono_distributed/physics/ChDomainDistrLong.h"
#include "chrono_distributed/collision/ChBroadphaseDistrBasic.h"
#include "chrono_distributed/collision/ChNarrowphaseDistrBasic.h"
#include "chrono_distributed/collision/ChCollisionSystemDistr.h"


#include <iostream>
#include <mpi.h>

#define MASTER 0

using namespace chrono;
using namespace std;

int my_rank;
int num_ranks;

int main(int argc, char *argv[])
{
	MPI_Init(&argc, &argv);

	cout << "Constructing the system..." << endl;
	ChSystemDistr my_sys(MPI_COMM_WORLD);
	cout << "Hello from system on rank: " << my_sys.GetMyRank() << endl;

	if (my_sys.GetMyRank() == MASTER) cout << "Assigning a domain implementation..." << endl;

	ChDomainDistrLong domain(&my_sys);
	my_sys.SetDomainImpl(&domain);

	domain.SetSimDomain(0,1,0,1,0,1);
	domain.SplitDomain();
	
	ChBroadphaseDistrBasic broadphase(&my_sys);
	my_sys.GetCollisionSystem()->SetBroadphase(&broadphase);

	ChNarrowphaseDistrBasic narrowphase(&my_sys);
	my_sys.GetCollisionSystem()->SetNarrowphase(&narrowphase);


	MPI_Finalize();
	return 0;
}
