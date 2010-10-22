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
#include "physics/ChPhysicsItem.h"
#include <sstream>
#include <functional>


namespace chrono 
{

/// Class to be used in hash table of ChDomainNodeInterfaceMPI,
/// is is small data, just a bit more than a pointer (it also has
/// info telling which side of the interface is the 'master')

class ChInterfaceItem
{
public:
	enum eChInterfaceItemType
	{
		INTERF_SLAVE = 0,
		INTERF_MASTER,
		INTERF_SLAVESLAVE
	};

	ChInterfaceItem(ChPhysicsItem*  mitem, eChInterfaceItemType ismaster) : item(mitem), type(ismaster) {};

	ChPhysicsItem*  item;
	eChInterfaceItemType	type;
};

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
	ChHashTable<int, ChInterfaceItem> shared_items;

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
		if (point.x >= this->min_box.x &&
			point.y >= this->min_box.y &&
			point.z >= this->min_box.z &&
			point.x <  this->max_box.x &&
			point.y <  this->max_box.y &&
			point.z <  this->max_box.z ) 
			return true;
		else
			return false; 
	}

	virtual bool IsAABBinside(ChVector<>& aabbmin, ChVector<>& aabbmax)
	{
		if (aabbmin.x >= this->min_box.x &&
			aabbmin.y >= this->min_box.y &&
			aabbmin.z >= this->min_box.z &&
			aabbmax.x <  this->max_box.x &&
			aabbmax.y <  this->max_box.y &&
			aabbmax.z <  this->max_box.z ) 
			return true;
		else
			return false;
	}

	virtual bool IsAABBoutside(ChVector<>& aabbmin, ChVector<>& aabbmax)
	{
		if (aabbmax.x <  this->min_box.x ||
			aabbmax.y <  this->min_box.y ||
			aabbmax.z <  this->min_box.z ||
			aabbmin.x >= this->max_box.x ||
			aabbmin.y >= this->max_box.y ||
			aabbmin.z >= this->max_box.z ) 
			return true;
		else
			return false;
	}		
	
	static const int mask_xinf = 0x1FF;
	static const int mask_xsup = 0x7FC0000;
	static const int mask_yinf = 0x1C0E07;
	static const int mask_ysup = 0x70381C0;
	static const int mask_zinf = 0x1249249;
	static const int mask_zsup = 0x4924924;

	virtual bool IsAABBoverlappingInterface(int n_interface, ChVector<>& aabbmin, ChVector<>& aabbmax)
	{
		assert (n_interface < 26);

		// Start with: overlap to all 27 domains, then refine
		int overlapflags = 0xFFFFFFF; 
		// Remove the non overlapping 9-plets of surrounding domains
		if (aabbmin.x >= this->min_box.x)
			overlapflags &= ~ mask_xinf;
		if (aabbmax.x <  this->max_box.x)
			overlapflags &= ~ mask_xsup;
		if (aabbmin.y >= this->min_box.y)
			overlapflags &= ~ mask_yinf;
		if (aabbmax.y <  this->max_box.y)
			overlapflags &= ~ mask_ysup;
		if (aabbmin.z >= this->min_box.z)
			overlapflags &= ~ mask_zinf;
		if (aabbmax.z <  this->max_box.z)
			overlapflags &= ~ mask_zsup;
		
		if ( (0x1 << n_interface) & overlapflags )
			return true;
		return false;
	}

};






} // END_OF_NAMESPACE____


#endif  // END of ChDomainNodeMPI.h 
