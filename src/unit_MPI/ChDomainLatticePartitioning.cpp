///////////////////////////////////////////////////
//
//   ChSystemMPI.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "mpi.h"
#include "ChMpi.h"
#include "ChDomainLatticePartitioning.h"

using namespace std;


namespace chrono
{



bool ChDomainLatticePartitioning::SetupDescriptor(ChSystemDescriptorMPIlattice3D& mdescriptor,
												  int nx, int ny, int nz) const
{
	assert (nx < x_domains);
	assert (ny < y_domains);
	assert (nz < z_domains);

	// Set the AABB of domain of descriptor
	mdescriptor.min_box.x = this->x_split(nx);
	mdescriptor.max_box.x = this->x_split(nx+1);

	mdescriptor.min_box.y = this->y_split(ny);
	mdescriptor.max_box.y = this->y_split(ny+1);

	mdescriptor.min_box.z = this->z_split(nz);
	mdescriptor.max_box.z = this->z_split(nz+1);

	// If at the boundary of the entire lattice world, set the 
	// AABB to extend to infinite along direction with no neighbour
	if (nx = 0)			mdescriptor.min_box.x = -1e200;
	if (ny = 0)			mdescriptor.min_box.y = -1e200;
	if (nz = 0)			mdescriptor.min_box.z = -1e200;
	if (nx = x_domains-1) mdescriptor.max_box.x =  1e200;
	if (nx = x_domains-1) mdescriptor.max_box.y =  1e200;
	if (nx = x_domains-1) mdescriptor.max_box.z =  1e200;


	// Compute MPI id of domain of descriptor (not needed?)
	unsigned int IDmpi = ComputeIDfromIxIyIz(nx,ny,nz);

	// Set the 27 interfaces, setting their AABB and their MPI IDs
	
	mdescriptor.GetSharedInterfacesList().clear();

	bool infinitex, infinitey, infinitez;
	for (int ix = -1; ix <= 1; ix++)
	{
		int x_int = ix + nx;
		infinitex = (x_int < 0)||(x_int > x_domains);
		for (int iy = -1; iy <= 1; iy++)
		{
			int y_int = iy + ny;
			infinitey = (y_int < 0)||(y_int > y_domains);
			for (int iz = -1; iz <= 1; iz++)
			{
				int z_int = iz + nz;
				infinitez = (z_int < 0)||(z_int > z_domains);
				
				// Add interface
				ChLcpSharedInterfaceMPI minterface;

				minterface.SetMPIfriend(ComputeIDfromIxIyIz(x_int, y_int, z_int));

					// exception, disable MPI if extending infinitely at boundary
				if (infinitex || infinitey || infinitez) 
					minterface.SetMPIfriend(-1);
					// exception, disable MPI for the 13th because it's the center domain itself
				if (ix && iy && iz)
					minterface.SetMPIfriend(-1);

				mdescriptor.GetSharedInterfacesList().push_back(minterface);

			}
		}
	}

	return true;
}




} // END_OF_NAMESPACE____


////// end
