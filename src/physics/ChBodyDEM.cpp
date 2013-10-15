//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChBodyDEM.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

    
#include <stdlib.h>
#include <algorithm>

#include "core/ChTrasform.h"
#include "physics/ChBodyDEM.h"
#include "physics/ChGlobal.h"
#include "physics/ChSystem.h"

#include "physics/ChExternalObject.h"
#include "collision/ChCModelBulletDEM.h"
#include "core/ChLinearAlgebra.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{

using namespace collision;
using namespace geometry;


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChBodyDEM> a_registration_ChBodyDEM;



//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR SOLID BODIES


ChBodyDEM::ChBodyDEM ()
{
	collision_model=ChBodyDEM::InstanceCollisionModel();

	//kn=392400.0;
	//gn=420.0;
	kn=2e5; //2e5		//2e8
	gn=7.5e2;   //5		//15000
	kt=kn;
}


ChBodyDEM::~ChBodyDEM ()
{

}

void ChBodyDEM::Copy(ChBodyDEM* source)
{
		// copy the parent class data...
	ChBody::Copy(source);

	kn = source->kn;
	gn = source->gn;
	kt = source->kt;
}


ChCollisionModel* ChBodyDEM::InstanceCollisionModel(){
	ChCollisionModel* collision_model_t= (ChModelBulletDEM*) new ChModelBulletDEM();
	((ChModelBulletDEM*)collision_model_t)->SetBody(this);
	return collision_model_t;
}

//////// FILE I/O

void ChBodyDEM::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChBody::StreamOUT(mstream);

	mstream << kn;
	mstream << gn;
	mstream << kt;
}

void ChBodyDEM::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChBody::StreamIN(mstream);

	float ffoo;

	mstream >> ffoo;		SetSpringCoefficient(ffoo);
	mstream >> ffoo;		SetDampingCoefficient(ffoo);
	mstream >> kt;
}


#define CH_CHUNK_END_DEM 98765

int ChBodyDEM::StreamINall  (ChStreamInBinary& m_file)
{
	int mchunk = 0;

	// 1) read body class data...
 
	ChBody::StreamINall(m_file);

	m_file >> kn;
	m_file >> gn;
	m_file >> kt;

	m_file >> mchunk; 

	if (mchunk != CH_CHUNK_END_DEM) return 0;


	return 1;
}


int ChBodyDEM::StreamOUTall  (ChStreamOutBinary& m_file)
{

	// 1) read body class data...
	ChBody::StreamOUTall(m_file);

	m_file << kn;
	m_file << gn;
	m_file << kt;

	m_file << (int)CH_CHUNK_END_DEM;

	return 1;
}


} // END_OF_NAMESPACE____


/////////////////////
