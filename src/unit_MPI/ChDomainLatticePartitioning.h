#ifndef CHDOMAINLATTICEPARTITIONING_H
#define CHDOMAINLATTICEPARTITIONING_H

//////////////////////////////////////////////////
//  
//   ChDomainLatticePartitioning.h
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChMpi.h"
#include "ChDomainNodeMPI.h"

namespace chrono 
{



/// Helper class that defines how the 3D space is partitioned in 
/// different domains with a lattice, so that it can be used to
/// easily setup the topology defined by the ChSystemDescriptorMPIlattice3D 
/// objects.

class ChApiMPI ChDomainLatticePartitioning
{
public:
	ChDomainLatticePartitioning(int mx_domains, 
								int my_domains, 
								int mz_domains, 
								ChVector<> mworld_min, 
								ChVector<> mworld_max) 
		{
			x_domains=mx_domains;
			y_domains=my_domains;
			z_domains=mz_domains;
			world_min=mworld_min;
			world_max=mworld_max;
			world_size=mworld_max-mworld_min;
			x_split.Resize(x_domains+1,1);
			y_split.Resize(y_domains+1,1);
			z_split.Resize(z_domains+1,1);
			double xstep = world_size.x / ((double)x_domains);
			double ystep = world_size.y / ((double)y_domains);
			double zstep = world_size.z / ((double)z_domains);
			for ( int i = 0; i<= x_domains; i++)
				x_split(i,0)=world_min.x + i*xstep;
			for ( int i = 0; i<= y_domains; i++)
				y_split(i,0)=world_min.y + i*ystep;
			for ( int i = 0; i<= z_domains; i++)
				z_split(i,0)=world_min.z + i*zstep;
		}


		/// Setup the MPI interfaces and AABB for a MPI node oc ChDomainNodeMPIlattice3D 
		/// type, given indexes nx ny nz in the lattice. If the node is at the
		/// world min/max neighbour, it extends infinitely in that direction.
		/// Return false if out of index ranges.
	bool SetupNode(ChDomainNodeMPIlattice3D& mnode, int nx, int ny, int nz) const;

		/// Setup the MPI interfaces and AABB for a MPI node oc ChDomainNodeMPIlattice3D 
		/// type, given MPI rank index. We assume that all rank indexes will be used
		/// to setup all nodes (rank will be converted in nx ny nz indexes). This is
		/// easier to use than SetupNode(ChDomainNodeMPIlattice3D& mnode, int nx, int ny, int nz),
		/// but less flexible.
		/// If the node is at the world min/max neighbour, it extends infinitely in that direction.
		/// Return false if out of index ranges.
	bool SetupNode(ChDomainNodeMPIlattice3D& mnode, int nrank) const;

		// 
		// Utils
		//

		/// Compute the MPI id (the rank) from the x,y,z indexes, assuming all are used
	int ComputeIDfromIxIyIz(int ix, int iy, int iz) const  {return iz+(iy*z_domains)+(ix*y_domains*z_domains);}

		/// Get the size of the world (x y z size of the AABB of the entire lattice)
	ChVector<> GetWorldSize() const {return world_size;}

		/// Get the lower left corner of the AABB of the entire lattice
	ChVector<> GetWorldMin() const {return world_min;}

		/// Get the upper right corner of the AABB of the entire lattice
	ChVector<> GetWorldMax() const {return world_max;}

		/// Get the n. of domains in x
	int GetXdomains() {return x_domains;}
		/// Get the n. of domains in y
	int GetYdomains() {return y_domains;}
		/// Get the n. of domains in z
	int GetZdomains() {return z_domains;}

private:
	int x_domains;
	int y_domains;
	int z_domains;
	ChVector<> world_min;
	ChVector<> world_max;
	ChVector<> world_size;
	ChMatrixDynamic<> x_split;
	ChMatrixDynamic<> y_split;
	ChMatrixDynamic<> z_split;
};




} // END_OF_NAMESPACE____


#endif  // END of ChSystemMPI.h 
