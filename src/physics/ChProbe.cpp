//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChProbe.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include <math.h>

#include "physics/ChProbe.h"
#include "physics/ChGlobal.h"
#include "physics/ChExternalObject.h"

namespace chrono 
{



/////////////////////////////////////////////////////////
/// 
///   CLASS
///
///




ChProbe::ChProbe()
{
	this->SetIdentifier(CHGLOBALS().GetUniqueIntID()); // mark with unique ID
}

ChProbe::~ChProbe()
{
}

void ChProbe::Copy(ChProbe* source)
{
	// first copy the parent class data...
	ChObj::Copy(source);
	
	// copy other data..

}

void ChProbe::Record(double mtime)
{
	if (GetExternalObject())
		GetExternalObject()->onChronoProbeRecord(mtime);

}

void ChProbe::Reset()
{
	if (GetExternalObject())
		GetExternalObject()->onChronoProbeReset();

}


} // END_OF_NAMESPACE____

////// end
