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
#include <string>
#include <mpi.h>

#define MASTER 0

using namespace chrono;
using namespace std;

int my_rank;
int num_ranks;

void print(string msg);

int main(int argc, char *argv[])
{
	MPI_Init(&argc, &argv);

	cout << "Constructing the system...\n";
	auto my_sys = make_shared<ChSystemDistr>(MPI_COMM_WORLD);
	my_sys->SetGhostLayer(0.0);

	print("Assigning a domain implementation...\n");
	auto domain = make_shared<ChDomainDistrLong>(my_sys);
	domain->SetSimDomain(0,1,0,1,0,1);
	domain->SplitDomain();
	my_sys->SetDomainImpl(domain);

	// TODO See chrono::parallel broadphase
	print("Setting broadphase...\n");
	auto broadphase = make_shared<ChBroadphaseDistrBasic>(my_sys, 0.5);
	my_sys->GetCollisionSystem()->SetBroadphase(broadphase);

	// Return/create arrow of contactinfo structures
	// curvature 1/reff = 1/r1 + 1/r2, for DEMP
	print("Setting narrowphase...\n");
	auto narrowphase = make_shared<ChNarrowphaseDistrBasic>(my_sys);
	my_sys->GetCollisionSystem()->SetNarrowphase(narrowphase);

	// Creates and adds bodies to the system to check binning
	print("Creating bodies...\n");

	int id = 0;
	for (double i = 0.1; i < 1; i+=0.5)
	{
		for (double j = 0.1; j < 1; j+=0.5)
		{
			for (double k = 0.1; k < 1; k+=0.5)
			{
				auto body = make_shared<ChBodyDistr>();
				body->SetPos(ChVector<>(i,j,k));
				body->SetGlobalId(id++);
				my_sys->AddBody(body);
			}
		}
	}

	// Perform binning
	print("Performing binning...\n");
	broadphase->DetectPossibleCollisions();

	// Print out domain and binning information
	domain->PrintDomain();
	broadphase->PrintBins();

	MPI_Finalize();

	return 0;
}

void print(string msg)
{
	if (my_rank == MASTER)
		cout << msg;
}
