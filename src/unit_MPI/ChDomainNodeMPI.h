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
#include "core/ChHashTable.h"
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
	ChHashTable<int, ChPhysicsItem*> shared_items;

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

	virtual ~ChDomainNodeMPI ()
	{
	}

	virtual bool IsInto(ChVector<>& point)= 0;
	virtual bool IsAABBinside (ChVector<>& aabbmin, ChVector<>& aabbmax)= 0;
	virtual bool IsAABBoutside(ChVector<>& aabbmin, ChVector<>& aabbmax)= 0;
	virtual bool IsAABBoverlappingInterface(int n_interface, ChVector<>& aabbmin, ChVector<>& aabbmax)= 0;
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

	virtual bool IsInto(ChVector<>& point)
	{
		if (point.x > this->min_box.x &&
			point.y > this->min_box.y &&
			point.z > this->min_box.z &&
			point.x < this->max_box.x &&
			point.y < this->max_box.y &&
			point.z < this->max_box.z ) 
			return true;
		else
			return false; 
	}

	virtual bool IsAABBinside(ChVector<>& aabbmin, ChVector<>& aabbmax)
	{
		if (aabbmin.x > this->min_box.x &&
			aabbmin.y > this->min_box.y &&
			aabbmin.z > this->min_box.z &&
			aabbmax.x < this->max_box.x &&
			aabbmax.y < this->max_box.y &&
			aabbmax.z < this->max_box.z ) 
			return true;
		else
			return false;
	}

	virtual bool IsAABBoutside(ChVector<>& aabbmin, ChVector<>& aabbmax)
	{
		if (aabbmax.x < this->min_box.x ||
			aabbmax.y < this->min_box.y ||
			aabbmax.z < this->min_box.z ||
			aabbmin.x > this->max_box.x ||
			aabbmin.y > this->max_box.y ||
			aabbmin.z > this->max_box.z ) 
			return true;
		else
			return false;
	}
	virtual bool IsAABBoverlappingInterface(int n_interface, ChVector<>& aabbmin, ChVector<>& aabbmax)
	{
		//***TO DO***
		return true;
	}
};






} // END_OF_NAMESPACE____


#endif  // END of ChDomainNodeMPI.h 
