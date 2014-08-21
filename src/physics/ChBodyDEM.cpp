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

#include "core/ChTransform.h"
#include "physics/ChBodyDEM.h"
#include "physics/ChGlobal.h"
#include "physics/ChSystem.h"

#include "physics/ChExternalObject.h"
#include "collision/ChCModelBulletBody.h"
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


ChBodyDEM::ChBodyDEM()
{
	collision_model->SetEnvelope(0);

	matsurface.SetNull();
	matsurfaceDEM = ChSharedPtr<ChMaterialSurfaceDEM>(new ChMaterialSurfaceDEM);
}

ChBodyDEM::ChBodyDEM(ChCollisionModel* new_collision_model)
:	ChBody(new_collision_model)
{
	collision_model->SetEnvelope(0);
	collision_model->SetBody(this);

	matsurface.SetNull();
	matsurfaceDEM = ChSharedPtr<ChMaterialSurfaceDEM>(new ChMaterialSurfaceDEM);
}

void ChBodyDEM::Copy(ChBodyDEM* source)
{
	ChBody::Copy(source);

	matsurface.SetNull();
	matsurfaceDEM = source->matsurfaceDEM;
}


//////// FILE I/O

void ChBodyDEM::StreamOUT(ChStreamOutBinary& mstream)
{
	// class version number
	mstream.VersionWrite(1);

	// serialize parent class too
	ChBody::StreamOUT(mstream);

	matsurfaceDEM->StreamOUT(mstream);
}

void ChBodyDEM::StreamIN(ChStreamInBinary& mstream)
{
	// class version number
	int version = mstream.VersionRead();

	// deserialize parent class too
	ChBody::StreamIN(mstream);

	matsurfaceDEM->StreamIN(mstream);
}


#define CH_CHUNK_END_DEM 98765

int ChBodyDEM::StreamINall(ChStreamInBinary& m_file)
{
	// TODO
	return 1;
}


int ChBodyDEM::StreamOUTall(ChStreamOutBinary& m_file)
{
	// TODO
	return 1;
}


} // END_OF_NAMESPACE____


/////////////////////
