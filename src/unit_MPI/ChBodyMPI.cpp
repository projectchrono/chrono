///////////////////////////////////////////////////
//
//   ChBodyMPI.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

     
#include "ChBodyMPI.h"
#include "ChLcpSystemDescriptorMPI.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChBodyMPI> a_registration_ChBodyMPI;



ChBodyMPI::ChBodyMPI ()
{
	
}



ChBodyMPI::~ChBodyMPI ()
{
	
}


static const int mask_xinf = 0x1FF;
static const int mask_xsup = 0x7FC0000;
static const int mask_yinf = 0x1C0E07;
static const int mask_ysup = 0x70381C0;
static const int mask_zinf = 0x1249249;
static const int mask_zsup = 0x4924924;


int ChBodyMPI::ComputeOverlapFlags(ChDomainNodeMPIlattice3D& mnode)
{
	// See if it is overlapping to one of the surrounding domains
	ChVector<> bbmin, bbmax;
	this->GetCollisionModel()->GetAABB(bbmin, bbmax);

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


void ChBodyMPI::InjectVariables(ChLcpSystemDescriptor& mdescriptor)
{	
	this->Variables().SetDisabled(!this->IsActive());

	mdescriptor.InsertVariables(&this->Variables());

}




//////// FILE I/O

void ChBodyMPI::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChBody::StreamOUT(mstream);

		// stream out all member data
	//mstream << foo;
}

void ChBodyMPI::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChBody::StreamIN(mstream);

		// stream in all member data
	//mstream >> foo;
}





} // END_OF_NAMESPACE____


/////////////////////
