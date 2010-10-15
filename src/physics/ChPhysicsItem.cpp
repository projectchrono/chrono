///////////////////////////////////////////////////
//
//   ChPhysicsItem.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "physics/ChPhysicsItem.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{





// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChPhysicsItem> a_registration_ChPhysicsItem;



void ChPhysicsItem::Copy(ChPhysicsItem* source)
{
    // first copy the parent class data...
    ChObj::Copy(source);

	// copy other class data
	system=0; // do not copy - must be initialized with insertion in system.
}




void ChPhysicsItem::GetAABB(ChVector<>& bbmin, ChVector<>& bbmax)
{
	bbmin.Set(-1e200, -1e200, -1e200);
	bbmax.Set( 1e200,  1e200,  1e200);
}
				
void ChPhysicsItem::GetCenter(ChVector<>& mcenter)
{
	ChVector<> mmin, mmax;
	this->GetAABB(mmin, mmax);
	mcenter = (mmin+mmax)*0.5;
}


/////////
///////// FILE I/O
/////////



void ChPhysicsItem::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChObj::StreamOUT(mstream);

		// stream out all member data
	
}

void ChPhysicsItem::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChObj::StreamIN(mstream);

		// stream in all member data

}











} // END_OF_NAMESPACE____


