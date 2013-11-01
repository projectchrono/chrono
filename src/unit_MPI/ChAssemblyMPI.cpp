//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChAssemblyMPI.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

     
#include "ChAssemblyMPI.h"
#include "ChBodyDEMMPI.h"
#include "ChSystemMPI.h"
#include "ChLcpSystemDescriptorMPI.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChAssemblyMPI> a_registration_ChAssemblyMPI;



ChAssemblyMPI::ChAssemblyMPI ()
{
	
}



ChAssemblyMPI::~ChAssemblyMPI ()
{
	RemoveAllBodies(); 
	RemoveAllLinks();
}


static const int mask_xinf = 0x1FF;
static const int mask_xsup = 0x7FC0000;
static const int mask_yinf = 0x1C0E07;
static const int mask_ysup = 0x70381C0;
static const int mask_zinf = 0x1249249;
static const int mask_zsup = 0x4924924;


int ChAssemblyMPI::ComputeOverlapFlags(ChDomainNodeMPIlattice3D& mnode)
{
	// See if it is overlapping to one of the surrounding domains
	ChVector<> bbmin, bbmax;
	this->GetTotalAABB(bbmin,bbmax);

	if (bbmin.x < mnode.min_box.x ||
		bbmin.y < mnode.min_box.y ||
		bbmin.z < mnode.min_box.z ||
		bbmax.x > mnode.max_box.x ||
		bbmax.y > mnode.max_box.y ||
		bbmax.z > mnode.max_box.z )
	{
		// Start with: overlap to all 27 domains, then refine
		int overlapflags = 0x3FFFFFF; 
		// Remove the non overlapping 9-plets of surrounding domains
		if (bbmin.x > mnode.min_box.x)
			overlapflags &= ~ mask_xinf;
		if (bbmax.x < mnode.max_box.x)
			overlapflags &= ~ mask_xsup;
		if (bbmin.y > mnode.min_box.y)
			overlapflags &= ~ mask_yinf;
		if (bbmax.y < mnode.max_box.y)
			overlapflags &= ~ mask_ysup;
		if (bbmin.z > mnode.min_box.z)
			overlapflags &= ~ mask_zinf;
		if (bbmax.z < mnode.max_box.z)
			overlapflags &= ~ mask_zsup;
		// Not interested in 13th domain that is the central domain itself
		overlapflags &= ~ (1 << 13);

		this->last_shared = overlapflags;
		return this->last_shared;

	} // end of overlap code
	return this->last_shared;
}


void ChAssemblyMPI::Update()
{
	ChAssemblyMPI::Update(this->GetChTime());
}



							// As before, but keeps the current state.
							// Mostly used for world reference body.
void ChAssemblyMPI::Update (double mytime)
{
	ChTime = mytime;
	ClampSpeed();			// Apply limits (if in speed clamping mode) to speeds.

	if (ChSystem* syss = dynamic_cast<ChSystem*>(this->GetSystem()))
	{
		ChVector<> center;
		this->GetCenter(center);
		
		std::vector<ChBody*>::iterator ibody = bodylist.begin();					
		while (ibody != bodylist.end())
		{	
			(*ibody)->UpdateMarkers(mytime);
			if (ChBodyDEMMPI* bodd = dynamic_cast<ChBodyDEMMPI*>(*ibody))
			{
				(*bodd).UpdateForces(mytime, true);
			}
			ibody++;
		}
	}
	if (ChSystemMPI* syss = dynamic_cast<ChSystemMPI*>(this->GetSystem()))
	{
		ChVector<> center;
		this->GetCenter(center);
		
		std::vector<ChBody*>::iterator ibody = bodylist.begin();					
		while (ibody != bodylist.end())
		{	
			(*ibody)->UpdateMarkers(mytime);
			if (ChBodyDEMMPI* bodd = dynamic_cast<ChBodyDEMMPI*>(*ibody))
			{
				(*bodd).UpdateForces(mytime, (*syss).nodeMPI->IsInto(center));
			}
			ibody++;
		}
	}
	std::list<ChLink*>::iterator iterlink = linklist.begin();
	while (iterlink != linklist.end())
	{
		(*iterlink)->Update(mytime);
		iterlink++;
	}
}

//////// FILE I/O

void ChAssemblyMPI::StreamOUT(ChStreamOutBinary& mstream)
{
	/*
	// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChPhysicsItem::StreamOUT(mstream);

		// stream out all member data
	mstream << do_collide;
	mstream << do_limit_speed;
	
	mstream << max_speed;
	mstream << max_wvel;

	// 2a) write how many bodies
	mstream << (int)bodylist.size();

	// 2b) write  bodies
	std::vector<ChBody*>::iterator ibody = bodylist.begin();
	while (ibody != bodylist.end())
	{
			// write the body
		//Bpointer->StreamOUT(mstream);
		//(*ibody)->GetRTTI()->
		mstream.AbstractWriteAll(*ibody);
		//mstream.AbstractWrite(Bpointer);
		ibody++;
	}

	// 3a) write how many links
	mstream << (int)linklist.size(); 

	// 3b) write links links
	std::list<ChLink*>::iterator iterlink = linklist.begin();
	while (iterlink != linklist.end())
	{
			// Writethe link, using a special downcasting function Link_BinSave which saves also the
			// inheritance info, depending on link class inheritance from base Link*
		mstream.AbstractWrite(*iterlink);

		iterlink++;
	}
	*/
	
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChAssembly::StreamOUT(mstream);     // THIS IS NOT SUPPORTED???

		// stream out all member data
	//mstream << foo;
}

void ChAssemblyMPI::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChAssembly::StreamIN(mstream);     // THIS IS NOT SUPPORTED???

		// stream in all member data
	//mstream >> foo;
}





} // END_OF_NAMESPACE____


/////////////////////

