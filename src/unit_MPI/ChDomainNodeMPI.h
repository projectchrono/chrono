#ifndef CHDOMAINNODEMPI_H
#define CHDOMAINNODEMPI_H

//////////////////////////////////////////////////
//  
//   ChDomainNodeMPI.h
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
#include <sstream>


namespace chrono 
{

/// Class for connectivity between nodes in a MPI 
/// multi-domain environment

class ChDomainNodeInterfaceMPI
{
public:
	int id_MPI;
	std::vector<char>* mstreamo;
	ChStreamOutBinaryVector* mchstreamo;
	std::vector<char>* mstreami;
	ChStreamInBinaryVector* mchstreami;

				/// Builder.
	ChDomainNodeInterfaceMPI ()
	{
		id_MPI = 0;
		mstreamo = new std::vector<char>;
		mstreami = new std::vector<char>;
		mchstreamo = new ChStreamOutBinaryVector(mstreamo);
		mchstreami = new ChStreamInBinaryVector(mstreami);
	}

	ChDomainNodeInterfaceMPI( ChDomainNodeInterfaceMPI const& rhs )
	{
		id_MPI = 0;
		mstreamo = new std::vector<char>;
		mstreami = new std::vector<char>;
		mchstreamo = new ChStreamOutBinaryVector(mstreamo);
		mchstreami = new ChStreamInBinaryVector(mstreami);
		// do not copy data, it is only for supporting std::vector
	}

				/// Destructor
	~ChDomainNodeInterfaceMPI ()
	{
		delete mstreamo;
		delete mstreami;
		delete mchstreamo;
		delete mchstreami;
	}
};


/// Class for a node in a MPI 
/// multi-domain environment

class ChDomainNodeMPI
{
public:
	int id_MPI;
	std::vector<ChDomainNodeInterfaceMPI> interfaces;
	
	ChDomainNodeMPI ()
	{
		id_MPI = 0;
	}

	~ChDomainNodeMPI ()
	{
	}
};


/// Class for a node in a MPI multi-domain environment
/// splitted as a 3D lattice

class ChDomainNodeMPIlattice3D : public ChDomainNodeMPI
{
public:
	ChVector<> min_box;
	ChVector<> max_box;
	
	ChDomainNodeMPIlattice3D ()
	{
		id_MPI = 0;
		interfaces.resize(27);
		min_box.Set(0,0,0);
		max_box.Set(0,0,0);	
	}
};






} // END_OF_NAMESPACE____


#endif  // END of ChDomainNodeMPI.h 
