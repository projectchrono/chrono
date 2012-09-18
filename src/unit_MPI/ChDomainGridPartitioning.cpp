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

bool ChDomainGridPartitioning::SetupNode(ChDomainNodeMPIgrid3D& mnode, int myid) const
{
	// convert unique rank into x y z lattice indices. 
	int nz = (int)floor(fmod( (myid+0.0001),z_domains));
	int ny = (int)floor(fmod( (floor((double)myid /(double)(z_domains))+0.0001) , y_domains));
	int nx = (int)floor(fmod( (floor((double)myid /(double)(z_domains*y_domains))+0.0001) , x_domains));
	
	assert (nx < x_domains);
	assert (ny < y_domains);
	assert (nz < z_domains);

	// Set the AABB of domain of descriptor
	mnode.min_box.x = this->x_split(nx);
	mnode.max_box.x = this->x_split(nx+1);

	mnode.min_box.y = this->y_split(ny);
	mnode.max_box.y = this->y_split(ny+1);

	mnode.min_box.z = this->z_split(nz);
	mnode.max_box.z = this->z_split(nz+1);

	mnode.SetupNode(myid,x_domains*y_domains*z_domains);

	// Set all interfaces, setting their AABB and their MPI IDs
	int iint = 0;
	for (int ix=0; ix<x_domains; ix++){
		for (int iy=0; iy<y_domains; iy++){
			for (int iz=0; iz<z_domains; iz++){
				mnode.interfaces[iint].id_MPI = iint;
				mnode.interfaces[iint].min_box.x = this->x_split(ix);
				mnode.interfaces[iint].max_box.x = this->x_split(ix+1);
				mnode.interfaces[iint].min_box.y = this->y_split(iy);
				mnode.interfaces[iint].max_box.y = this->y_split(iy+1);
				mnode.interfaces[iint].min_box.z = this->z_split(iz);
				mnode.interfaces[iint].max_box.z = this->z_split(iz+1);
				iint++;
			}
		}
	}
	// Disable self reference 
	mnode.interfaces[myid].id_MPI = -1;

	return true;
}

} // END_OF_NAMESPACE____


////// end
