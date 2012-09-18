///////////////////////////////////////////////////
//
//   ChDomainGridPartitioning.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "mpi.h"
#include "ChMpi.h"
#include "ChDomainGridPartitioning.h"

using namespace std;


namespace chrono
{

bool ChDomainGridPartitioning::SetupNode(ChDomainNodeMPI *mnode, int myid) const
{
	// make sure the domain node is grid type, like the partitioner
	ChDomainNodeMPIgrid3D *gridnode = dynamic_cast<ChDomainNodeMPIgrid3D*>(mnode);
	if (gridnode==0){
		GetLog() << "ERROR: domain node not of grid type\n";
		return false;
	}

	// convert unique rank into x y z lattice indices. 
	int nz = (int)floor(fmod( (myid+0.0001),z_domains));
	int ny = (int)floor(fmod( (floor((double)myid /(double)(z_domains))+0.0001) , y_domains));
	int nx = (int)floor(fmod( (floor((double)myid /(double)(z_domains*y_domains))+0.0001) , x_domains));
	
	assert (nx < x_domains);
	assert (ny < y_domains);
	assert (nz < z_domains);

	// Set the AABB of domain of descriptor
	gridnode->min_box.x = this->x_split(nx);
	gridnode->max_box.x = this->x_split(nx+1);

	gridnode->min_box.y = this->y_split(ny);
	gridnode->max_box.y = this->y_split(ny+1);

	gridnode->min_box.z = this->z_split(nz);
	gridnode->max_box.z = this->z_split(nz+1);

	gridnode->SetupNode(myid,x_domains*y_domains*z_domains);

	// Set all interfaces, setting their AABB and their MPI IDs
	int iint = 0;
	for (int ix=0; ix<x_domains; ix++){
		for (int iy=0; iy<y_domains; iy++){
			for (int iz=0; iz<z_domains; iz++){
				gridnode->interfaces[iint].id_MPI = iint;
				gridnode->interfaces[iint].min_box.x = this->x_split(ix);
				gridnode->interfaces[iint].max_box.x = this->x_split(ix+1);
				gridnode->interfaces[iint].min_box.y = this->y_split(iy);
				gridnode->interfaces[iint].max_box.y = this->y_split(iy+1);
				gridnode->interfaces[iint].min_box.z = this->z_split(iz);
				gridnode->interfaces[iint].max_box.z = this->z_split(iz+1);
				iint++;
			}
		}
	}
	// Disable self reference 
	gridnode->interfaces[myid].id_MPI = -1;

	return true;
}

} // END_OF_NAMESPACE____


////// end
