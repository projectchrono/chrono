///////////////////////////////////////////////////
//
//   ChSystemMPI.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChMpi.h"
#include "ChDomainLatticePartitioning.h"

using namespace std;


namespace chrono
{

bool ChDomainLatticePartitioning::SetupNode(ChDomainNodeMPI *mnode, int myid) const
{
	// convert unique rank into x y z lattice indexes. 
	int zin = (int)floor(fmod( (myid+0.0001),z_domains));
	int yin = (int)floor(fmod( (floor((double)myid /(double)(z_domains))+0.0001) , y_domains));
	int xin = (int)floor(fmod( (floor((double)myid /(double)(z_domains*y_domains))+0.0001) , x_domains));
	
	return this->SetupNode(mnode, xin, yin, zin);

	assert (myid == mnode->id_MPI); // check that computed lattice indexing corresponds to MPI rank
}


bool ChDomainLatticePartitioning::SetupNode(ChDomainNodeMPI *mnode,
												  int nx, int ny, int nz) const
{
	// make sure the domain node is lattice type, like the partitioner
	ChDomainNodeMPIlattice3D *lnode = dynamic_cast<ChDomainNodeMPIlattice3D*>(mnode);
	if (lnode==0){
		GetLog() << "ERROR: domain node not of lattice type\n";
		return false;
	}

	assert (nx < x_domains);
	assert (ny < y_domains);
	assert (nz < z_domains);

	// Set the AABB of domain of descriptor
	lnode->min_box.x = this->x_split(nx);
	lnode->max_box.x = this->x_split(nx+1);

	lnode->min_box.y = this->y_split(ny);
	lnode->max_box.y = this->y_split(ny+1);

	lnode->min_box.z = this->z_split(nz);
	lnode->max_box.z = this->z_split(nz+1);

	// If at the boundary of the entire lattice world, set the 
	// AABB to extend to infinite along direction with no neighbour
	if (nx == 0)			lnode->min_box.x = -1e200;
	if (ny == 0)			lnode->min_box.y = -1e200;
	if (nz == 0)			lnode->min_box.z = -1e200;
	if (nx == x_domains-1) lnode->max_box.x =  1e200;
	if (ny == y_domains-1) lnode->max_box.y =  1e200;
	if (nz == z_domains-1) lnode->max_box.z =  1e200;


	// Compute MPI id of domain of descriptor 
	unsigned int IDmpi = ComputeIDfromIxIyIz(nx,ny,nz);
	lnode->id_MPI = IDmpi;

	// Set 27 interfaces, setting their AABB and their MPI IDs
	
	//mdescriptor.GetSharedInterfacesList().clear();

	int iint = 0;
	bool infinitex, infinitey, infinitez;
	for (int ix = -1; ix <= 1; ix++)
	{
		int x_int = ix + nx;
		infinitex = (x_int < 0)||(x_int >= x_domains);
		for (int iy = -1; iy <= 1; iy++)
		{
			int y_int = iy + ny;
			infinitey = (y_int < 0)||(y_int >= y_domains);
			for (int iz = -1; iz <= 1; iz++)
			{
				int z_int = iz + nz;
				infinitez = (z_int < 0)||(z_int >= z_domains);
				
				lnode->interfaces[iint].id_MPI = ComputeIDfromIxIyIz(x_int, y_int, z_int);

				// exception, disable MPI if extending infinitely at boundary
				if (infinitex || infinitey || infinitez) 
					lnode->interfaces[iint].id_MPI = -1;
				// exception, disable MPI for the 13th because it's the center domain itself
				if ((ix==0) && (iy==0) && (iz==0))
					lnode->interfaces[iint].id_MPI = -1;

				++iint;
			}
		}
	}
	// Disable self reference 
	lnode->interfaces[13].id_MPI = -1;

	return true;
}




} // END_OF_NAMESPACE____


////// end
