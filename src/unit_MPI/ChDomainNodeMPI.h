//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

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

class ChApiMPI ChInterfaceItem
{
public:
	enum eChInterfaceItemType
	{
		INTERF_SLAVE = 0,
		INTERF_MASTER,
		INTERF_SLAVESLAVE,
		INTERF_NOT_INITIALIZED
	};

	ChInterfaceItem(ChPhysicsItem*  mitem, eChInterfaceItemType ismaster) : item(mitem), type(ismaster) {};

	ChPhysicsItem*  item;
	eChInterfaceItemType	type;
};

/// Class for connectivity between nodes in a MPI 
/// multi-domain environment

class ChApiMPI ChDomainNodeInterfaceMPI
{
public:
	int id_MPI;

	// subdomain boundary information:
	ChVector<> min_box;
	ChVector<> max_box;

	std::vector<char>* mstreamo;
	ChStreamOutBinaryVector* mchstreamo;
	std::vector<char>* mstreami;
	ChStreamInBinaryVector* mchstreami;

	ChHashTable<int, ChInterfaceItem> shared_items;
	std::vector< std::pair<int,ChInterfaceItem*> > sorted_items;

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

class ChApiMPI ChDomainNodeMPI
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

	virtual bool IsInto(ChVector<> point)= 0;
	virtual bool IsIntoInterface(int n_interface, ChVector<>& point) = 0;
	virtual bool IsAABBinside (ChVector<>& aabbmin, ChVector<>& aabbmax)= 0;
	virtual bool IsAABBoutside(ChVector<>& aabbmin, ChVector<>& aabbmax)= 0;
	virtual bool IsAABBoverlappingInterface(int n_interface, ChVector<>& aabbmin, ChVector<>& aabbmax)= 0;
};


/// Class for a node in a MPI multi-domain environment
/// splitted as a 3D lattice

class ChApiMPI ChDomainNodeMPIlattice3D : public ChDomainNodeMPI
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

	virtual bool IsInto(ChVector<> point)
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

	virtual bool IsIntoInterface(int n_interface, ChVector<>& point)
	{
		assert (n_interface < 26);

		// Start with: overlap to all 27 domains, then refine
		int overlapflags = 0xFFFFFFF; 
		// Set overlapping 9-plets of surrounding domains
		if (point.x <  this->min_box.x)
			overlapflags &=   mask_xinf;
		else 
			overlapflags &= ~ mask_xinf;

		if (point.x >= this->max_box.x)
			overlapflags &=   mask_xsup;
		else
			overlapflags &= ~ mask_xsup;

		if (point.y <  this->min_box.y)
			overlapflags &=   mask_yinf;
		else
			overlapflags &= ~ mask_yinf;

		if (point.y >= this->max_box.y)
			overlapflags &=   mask_ysup;
		else
			overlapflags &= ~ mask_ysup;

		if (point.z <  this->min_box.z)
			overlapflags &=   mask_zinf;
		else
			overlapflags &= ~ mask_zinf;

		if (point.z >= this->max_box.z)
			overlapflags &=   mask_zsup;
		else
			overlapflags &= ~ mask_zsup;
		
		if ( (0x1 << n_interface) & overlapflags )
			return true;
		return false;
	}


};

/// Class for a node in a MPI multi-domain environment
/// splitted as a 3D grid, this is more general than a lattice

class ChApiMPI ChDomainNodeMPIgrid3D : public ChDomainNodeMPI
{
public:
	ChVector<> min_box;
	ChVector<> max_box;

	ChDomainNodeMPIgrid3D ()
	{
		id_MPI = 0;
		interfaces.resize(0);
		min_box.Set(0,0,0);
		max_box.Set(0,0,0);
	}

	void SetupNode(int id, int totalNodes)
	{
		id_MPI=id;
		interfaces.resize(totalNodes);
	}

	virtual bool IsInto(ChVector<> point)
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

	virtual bool IsAABBoverlappingInterface(int n_interface, ChVector<>& aabbmin, ChVector<>& aabbmax)
	{
		assert (n_interface < interfaces.size() );

		// the aabb is in the interface if it is partially in each of the domain boxes which make up the interface
		// we already know it is in this domain node box, so we only have to make sure it is partially in the other

		if (aabbmin.x >= interfaces[n_interface].max_box.x ||
			aabbmin.y >= interfaces[n_interface].max_box.y ||
			aabbmin.z >= interfaces[n_interface].max_box.z ||
			aabbmax.x <  interfaces[n_interface].min_box.x ||
			aabbmax.y <  interfaces[n_interface].min_box.y ||
			aabbmax.z <  interfaces[n_interface].min_box.z )
			return false;
		else
			return true;
	}

	virtual bool IsIntoInterface(int n_interface, ChVector<>& point)
	{
		assert (n_interface < interfaces.size() );

		if (point.x >= interfaces[n_interface].min_box.x &&
			point.y >= interfaces[n_interface].min_box.y &&
			point.z >= interfaces[n_interface].min_box.z &&
			point.x <  interfaces[n_interface].max_box.x &&
			point.y <  interfaces[n_interface].max_box.y &&
			point.z <  interfaces[n_interface].max_box.z )
			return true;
		else
			return false;
	}


};






} // END_OF_NAMESPACE____


#endif  // END of ChDomainNodeMPI.h 
