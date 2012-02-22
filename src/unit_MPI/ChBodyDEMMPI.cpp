///////////////////////////////////////////////////
//
//   ChBodyDEMMPI.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

     
#include "ChBodyDEMMPI.h"
#include "unit_MPI/ChSystemMPI.h"
#include "ChLcpSystemDescriptorMPI.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChBodyDEMMPI> a_registration_ChBodyDEMMPI;



ChBodyDEMMPI::ChBodyDEMMPI ()
{
	
}



ChBodyDEMMPI::~ChBodyDEMMPI ()
{
	
}


static const int mask_xinf = 0x1FF;
static const int mask_xsup = 0x7FC0000;
static const int mask_yinf = 0x1C0E07;
static const int mask_ysup = 0x70381C0;
static const int mask_zinf = 0x1249249;
static const int mask_zsup = 0x4924924;


int ChBodyDEMMPI::ComputeOverlapFlags(ChDomainNodeMPIlattice3D& mnode)
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


void ChBodyDEMMPI::InjectVariables(ChLcpSystemDescriptor& mdescriptor)
{	
	this->Variables().SetDisabled(!this->IsActive());

	mdescriptor.InsertVariables(&this->Variables());

}

void ChBodyDEMMPI::Update (double mytime)
{
	UpdateTime(mytime);
	Update();
}

void ChBodyDEMMPI::Update ()
{
	//TrySleeping();			// See if the body can fall asleep; if so, put it to sleeping
	ClampSpeed();			// Apply limits (if in speed clamping mode) to speeds.
	ComputeGyro ();			// Set the gyroscopic momentum.

		// Also update the children "markers" and
		// "forces" depending on the body current state.
	UpdateMarkers(ChTime);

	bool inc_f=false;
	if (ChSystemMPI* syss = dynamic_cast<ChSystemMPI*>(this->GetSystem()))
	{
		ChVector<> center;
		this->GetCenter(center);
		if( (*syss).nodeMPI.IsInto( center ) )
		{
			inc_f=true;
		}
	}
	UpdateForces(ChTime, inc_f);
}

void ChBodyDEMMPI::UpdateForces (double mytime, bool incr)
{
	// COMPUTE LAGRANGIAN FORCES APPLIED TO BODY

	Xforce = VNULL;
	Xtorque = VNULL;

	if (ChSystemMPI* syss = dynamic_cast<ChSystemMPI*>(this->GetSystem()))
	{
		if( incr )
		{
			// 1 - force caused by stabilizing damper ***OFF***

			// 2a- force caused by accumulation of forces in body's accumulator Force_acc
			if (Vnotnull(&Force_acc))
			{
				Xforce = Force_acc;
			}

			// 2b- force caused by accumulation of torques in body's accumulator Force_acc
			if (Vnotnull(&Torque_acc))
			{
				Xtorque = Dir_World2Body(&Torque_acc);
			}

			// 3 - accumulation of other applied forces
			std::vector<ChForce*>::iterator iforce = forcelist.begin();
			while (iforce != forcelist.end())
			{
				  // update positions, f=f(t,q)
				(*iforce)->Update (mytime);

				ChVector<> mforce;
				ChVector<> mtorque;
				(*iforce)->GetBodyForceTorque(&mforce,&mtorque);
				Xforce  += mforce;
				Xtorque += mtorque;

				iforce++;
			}

			// 4 - accumulation of script forces
			if (Vnotnull(&Scr_force))
			{
				Xforce += Scr_force;
			}
			if (Vnotnull(&Scr_torque))
			{
				Xtorque += Dir_World2Body(&Scr_torque);
			}

			Xforce += GetSystem()->Get_G_acc() * this->GetMass();
		}
	}
}


//////// FILE I/O

void ChBodyDEMMPI::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChBodyDEM::StreamOUT(mstream);     // THIS IS NOT SUPPORTED???

		// stream out all member data
	//mstream << foo;
}

void ChBodyDEMMPI::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChBodyDEM::StreamIN(mstream);     // THIS IS NOT SUPPORTED???

		// stream in all member data
	//mstream >> foo;
}





} // END_OF_NAMESPACE____


/////////////////////
