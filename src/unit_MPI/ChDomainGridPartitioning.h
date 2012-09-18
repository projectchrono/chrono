#ifndef CHDOMAINGRIDPARTITIONING_H
#define CHDOMAINGRIDPARTITIONING_H

//////////////////////////////////////////////////
//  
//   ChDomainGridPartitioning.h
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
/// easily setup the topology defined by the ChSystemDescriptorMPIgrid3D
/// objects.

class ChApiMPI ChDomainGridPartitioning
{
public:
	ChDomainGridPartitioning(std::vector<double> x_divs,
									  std::vector<double> y_divs,
									  std::vector<double> z_divs,
									  ChVector<> mworld_min
									 )
	{
		/** Initializes the subdomain boundaries.
		  * mworld_min sets the minimum coordinate value of the system's boundaries.
		  * Each element of the x_divs vector sets the distance between subsequent
		  * boundary walls, or in other words, the width of each subdomain.
		  * Thus, the sum of the elements of x_divs is the x-component distance
		  * of the world_min position and the world_max, and the number of elements
		  * in x_divs is the number of x-partitions of the system.
		  * y_divs and z_divs are the same, except for setting the y- and z-coordinate
		  * boundaries.
		  */

			x_domains = x_divs.size();
			y_domains = y_divs.size();
			z_domains = z_divs.size();
			world_min = mworld_min;
			world_max = ChVector<>(world_min.x, world_min.y, world_min.z);
			x_split.Resize(x_domains+1, 1);
			y_split.Resize(y_domains+1, 1);
			z_split.Resize(z_domains+1, 1);

			// Set edge boundaries to negative infinity:
			x_split(0,0) = -1e200;
			y_split(0,0) = -1e200;
			z_split(0,0) = -1e200;
			
			for(int i=0; i<x_domains; i++){
				world_max.x += x_divs[i];
				x_split(i+1, 0) = world_max.x;
			}
			for(int i=0; i<y_domains; i++){
				world_max.y += y_divs[i];
				y_split(i+1, 0) = world_max.y;
			}
			for(int i=0; i<z_domains; i++){
				world_max.z += z_divs[i];
				z_split(i+1, 0) = world_max.z;
			}

			// Set edge boundaries to infinity:
			x_split(x_domains,0) = 1e200;
			y_split(y_domains,0) = 1e200;
			z_split(z_domains,0) = 1e200;

			world_size=world_max-world_min;
	}
	ChDomainGridPartitioning(std::vector<double> x_bounds,
									  std::vector<double> y_bounds,
									  std::vector<double> z_bounds
									 )
	{
		/** Initializes the subdomain boundaries.
		  * The x_bounds vector contains the x-positions of each of the boundary walls,
		  * where the element in the first index is the world minimum, the last index
		  * is the world maximum, and the middle elements are the in-between walls.
		  * The number of x-partitions, therefore, is one less than size of the x_bounds
		  * vector.
		  * It is thus necessary for the elements of x_bounds to be in ascending order.
		  * y_bounds and z_bounds are the same, except they contain y- and z-coordinate
		  * boundaries.
		  */

			x_domains = x_bounds.size() - 1;
			y_domains = y_bounds.size() - 1;
			z_domains = z_bounds.size() - 1;
			world_min = ChVector<>(x_bounds[0], y_bounds[0], z_bounds[0]);
			world_max = ChVector<>(x_bounds[x_domains], y_bounds[y_domains], z_bounds[z_domains]);
			x_split.Resize(x_domains+1, 1);
			y_split.Resize(y_domains+1, 1);
			z_split.Resize(z_domains+1, 1);

			// Set edge boundaries to infinity and negative infinity:
			x_split(0,0) = -1e200;
			y_split(0,0) = -1e200;
			z_split(0,0) = -1e200;
			x_split(x_domains,0) = 1e200;
			y_split(y_domains,0) = 1e200;
			z_split(z_domains,0) = 1e200;
			
			for(int i=1; i<x_domains; i++){
				assert (x_bounds[i] > x_bounds[i-1]);
				x_split(i, 0) = x_bounds[i];
			}
			for(int i=1; i<y_domains; i++){
				assert (y_bounds[i] > y_bounds[i-1]);
				y_split(i, 0) = y_bounds[i];
			}
			for(int i=1; i<z_domains; i++){
				assert (z_bounds[i] > z_bounds[i-1]);
				z_split(i, 0) = z_bounds[i];
			}

			world_size=world_max-world_min;
	}

		/// Setup the MPI interfaces and AABB for a MPI node of ChDomainNodeMPIgrid3D
		/// type, given MPI rank index. We assume that all rank indexes will be used
		/// to setup all nodes (rank will be converted in nx ny nz indexes). This is
		/// easier to use than SetupNode(ChDomainNodeMPIgrid3D& mnode, int nx, int ny, int nz),
		/// but less flexible.
		/// If the node is at the world min/max neighbour, it extends infinitely in that direction.
		/// Return false if out of index ranges.
	bool SetupNode(ChDomainNodeMPI *mnode, int nrank) const;

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


#endif  // END of ChDomainGridPartitioning.h
