///////////////////////////////////////////////////
//
//   ChProbe.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
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
	this->SetIdentifier(GLOBAL_Vars->GetUniqueIntID()); // mark with unique ID
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
