///////////////////////////////////////////////////
//
//   ChAssembly.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

    
#include <stdlib.h>
#include <algorithm>

#include "core/ChTrasform.h"
#include "physics/ChAssembly.h"
#include "physics/ChGlobal.h"
#include "physics/ChSystem.h"

#include "physics/ChExternalObject.h"
#include "collision/ChCModelBulletDEM.h"
#include "core/ChLinearAlgebra.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{

	// Hierarchy-handling functions

#define Bpointer		    (*ibody)
#define HIER_BODY_INIT      std::vector<ChBody*>::iterator ibody = bodylist.begin();
#define HIER_BODY_NOSTOP    (ibody != bodylist.end())
#define HIER_BODY_NEXT	    ibody++;

#define Lpointer		    (*iterlink)
#define HIER_LINK_INIT      std::list<ChLink*>::iterator iterlink = linklist.begin();
#define HIER_LINK_NOSTOP    (iterlink != linklist.end())
#define HIER_LINK_NEXT	    iterlink++;

using namespace collision;
using namespace geometry;


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChAssembly> a_registration_ChAssembly;



//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR SOLID BODIES


ChAssembly::ChAssembly ()
{
	do_collide = false;
	do_limit_speed = false;

	linklist.clear(); 
	bodylist.clear();

	max_speed = 0.5f;
	max_wvel  = 2.0f*float(CH_C_PI);

	SetIdentifier(CHGLOBALS().GetUniqueIntID()); // mark with unique ID

}



ChAssembly::~ChAssembly ()
{
	RemoveAllBodies(); 
	RemoveAllLinks();
}

void ChAssembly::Copy(ChAssembly* source)
{
		// copy the parent class data...
	ChPhysicsItem::Copy(source);

	do_collide = source->do_collide;
	do_limit_speed = source->do_limit_speed;

	max_speed = source->max_speed;
	max_wvel  = source->max_wvel;
}

void ChAssembly::SetSystem(ChSystem* m_system)
{
	this->system=m_system;
	HIER_BODY_INIT
	while (HIER_BODY_NOSTOP)
	{
		Bpointer->SetSystem(m_system);
		HIER_BODY_NEXT
	}
	HIER_LINK_INIT
	while (HIER_LINK_NOSTOP)
	{
		Lpointer->SetSystem(m_system);
		HIER_LINK_NEXT
	}
}

void ChAssembly::Clear()
{
	RemoveAllLinks();
	RemoveAllBodies();
}


void ChAssembly::RemoveAllBodies() 
{ 
	HIER_BODY_INIT
	while (HIER_BODY_NOSTOP)
	{
		// make sure to remove bodies from collision system before calling this.

		// nullify backward link to system
		Bpointer->SetSystem(0);	
		// this may delete the body, if none else's still referencing it..
		Bpointer->RemoveRef();
		HIER_BODY_NEXT
	}	
	bodylist.clear(); 
}


void ChAssembly::RemoveAllLinks() 
{ 

	HIER_LINK_INIT
	while (HIER_LINK_NOSTOP)
	{
		// nullify backward link to system
		Lpointer->SetSystem(0);	
		// this may delete the link, if none else's still referencing it..
		Lpointer->RemoveRef();
		HIER_LINK_NEXT
	}	
	linklist.clear(); 
}

int ChAssembly::GetDOF()
{
	int ndof = 0;

	HIER_BODY_INIT					
	while HIER_BODY_NOSTOP		
	{
		ndof+=Bpointer->GetDOF();
		HIER_BODY_NEXT
	}
	return ndof;
}

int ChAssembly::GetDOC_c()
{
	int ndoc=0;

	HIER_LINK_INIT
	while HIER_LINK_NOSTOP
	{
		ndoc   += Lpointer->GetDOC_c();
		HIER_LINK_NEXT
	}
	return ndoc;
}

int ChAssembly::GetDOC_d()
{
	int ndoc=0;

	HIER_LINK_INIT
	while HIER_LINK_NOSTOP
	{
		ndoc   += Lpointer->GetDOC_d();
		HIER_LINK_NEXT
	}
	return ndoc;
}

//// 
void ChAssembly::InjectVariables(ChLcpSystemDescriptor& mdescriptor)
{	
	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
		Bpointer->InjectVariables(mdescriptor);
		HIER_BODY_NEXT
	}
}


void ChAssembly::VariablesFbReset()
{
	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
		Bpointer->VariablesFbReset();
		HIER_BODY_NEXT
	}
}

void ChAssembly::VariablesFbLoadForces(double factor)
{
	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
		Bpointer->VariablesFbLoadForces(factor);
		HIER_BODY_NEXT
	}
}


void ChAssembly::VariablesQbLoadSpeed()
{
	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
		Bpointer->VariablesQbLoadSpeed();
		HIER_BODY_NEXT
	}
}


void ChAssembly::VariablesQbSetSpeed(double step)
{
	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
		Bpointer->VariablesQbSetSpeed(step);
		HIER_BODY_NEXT
	}
}

void ChAssembly::VariablesQbIncrementPosition(double dt_step)
{
	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
		Bpointer->VariablesQbIncrementPosition(dt_step);
		HIER_BODY_NEXT
	}
}

void ChAssembly::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	HIER_LINK_INIT
	while HIER_LINK_NOSTOP
	{
		Lpointer->InjectConstraints(mdescriptor);
		HIER_LINK_NEXT
	}
}

void ChAssembly::ConstraintsBiReset()
{
	HIER_LINK_INIT
	while HIER_LINK_NOSTOP
	{
		Lpointer->ConstraintsBiReset();
		HIER_LINK_NEXT
	}
}

void ChAssembly::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) 
{
	HIER_LINK_INIT
	while HIER_LINK_NOSTOP
	{
		Lpointer->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
		HIER_LINK_NEXT
	}
}

void ChAssembly::ConstraintsBiLoad_Ct(double factor)
{
	HIER_LINK_INIT
	while HIER_LINK_NOSTOP
	{
		Lpointer->ConstraintsBiLoad_Ct(factor);
		HIER_LINK_NEXT
	}
}

void ChAssembly::ConstraintsBiLoad_Qc(double factor)
{
	HIER_LINK_INIT
	while HIER_LINK_NOSTOP
	{
		Lpointer->ConstraintsBiLoad_Qc(factor);
		HIER_LINK_NEXT
	}
}

void ChAssembly::ConstraintsFbLoadForces(double factor)
{
	HIER_LINK_INIT
	while HIER_LINK_NOSTOP
	{
		Lpointer->ConstraintsFbLoadForces(factor);
		HIER_LINK_NEXT
	}
}

void ChAssembly::ConstraintsLoadJacobians()
{
	HIER_LINK_INIT
	while HIER_LINK_NOSTOP
	{
		Lpointer->ConstraintsLoadJacobians();
		HIER_LINK_NEXT
	}
}

void ChAssembly::ConstraintsLiLoadSuggestedSpeedSolution()
{
	HIER_LINK_INIT
	while HIER_LINK_NOSTOP
	{
		Lpointer->ConstraintsLiLoadSuggestedSpeedSolution();
		HIER_LINK_NEXT
	}
}

void ChAssembly::ConstraintsLiLoadSuggestedPositionSolution()
{
	HIER_LINK_INIT
	while HIER_LINK_NOSTOP
	{
		Lpointer->ConstraintsLiLoadSuggestedPositionSolution();
		HIER_LINK_NEXT
	}
}

void ChAssembly::ConstraintsLiFetchSuggestedSpeedSolution()
{
	HIER_LINK_INIT
	while HIER_LINK_NOSTOP
	{
		Lpointer->ConstraintsLiFetchSuggestedSpeedSolution();
		HIER_LINK_NEXT
	}
}

void ChAssembly::ConstraintsLiFetchSuggestedPositionSolution()
{
	HIER_LINK_INIT
	while HIER_LINK_NOSTOP
	{
		Lpointer->ConstraintsLiFetchSuggestedPositionSolution();
		HIER_LINK_NEXT
	}
}

void ChAssembly::ConstraintsFetch_react(double factor)
{
	HIER_LINK_INIT
	while HIER_LINK_NOSTOP
	{
		Lpointer->ConstraintsFetch_react(factor);
		HIER_LINK_NEXT
	}
}


void ChAssembly::SetNoSpeedNoAcceleration()
{
	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
		Bpointer->SetNoSpeedNoAcceleration();
		HIER_BODY_NEXT
	}
}


////
void ChAssembly::ClampSpeed()
{
	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
		Bpointer->ClampSpeed();
		HIER_BODY_NEXT
	}
}


							// UpdateALL updates the state and time
							// of the object AND the dependant (linked)
							// markers and forces.

void ChAssembly::Update()
{
	ChAssembly::Update(this->GetChTime());
}



							// As before, but keeps the current state.
							// Mostly used for world reference body.
void ChAssembly::Update (double mytime)
{
	ChTime = mytime;
	ClampSpeed();			// Apply limits (if in speed clamping mode) to speeds.

	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
		Bpointer->Update(mytime);
		HIER_BODY_NEXT
	}
	HIER_LINK_INIT
	while HIER_LINK_NOSTOP
	{
		Lpointer->Update(mytime);
		HIER_LINK_NEXT
	}
}

void ChAssembly::AddBody (ChSharedPtr<ChBody> newbody)
{
	assert(std::find<std::vector<ChBody*>::iterator>(bodylist.begin(), bodylist.end(), newbody.get_ptr())==bodylist.end());
	assert(newbody->GetSystem()==0); // should remove from other system before adding here

	newbody->AddRef();
	newbody->SetSystem (this->GetSystem());
	bodylist.push_back((newbody).get_ptr());

	// add to collision system too
	//if (newbody->GetCollide())
	//	newbody->AddCollisionModelsToSystem();
}

void ChAssembly::AddLink (ChLink* newlink)
{ 
	assert(std::find<std::list<ChLink*>::iterator>(linklist.begin(), linklist.end(), newlink)==linklist.end());

	newlink->AddRef();
	newlink->SetSystem (this->GetSystem());
	linklist.push_back(newlink);
}

void ChAssembly::AddLink (ChSharedPtr<ChLink> newlink)
{
	AddLink(newlink.get_ptr());
}

void ChAssembly::RemoveBody (ChSharedPtr<ChBody> mbody)
{
	assert(std::find<std::vector<ChBody*>::iterator>(bodylist.begin(), bodylist.end(), mbody.get_ptr() )!=bodylist.end());

	// remove from collision system
	if (mbody->GetCollide())
		mbody->RemoveCollisionModelsFromSystem(); 
 
	// warning! linear time search, to erase pointer from container.
	bodylist.erase(std::find<std::vector<ChBody*>::iterator>(bodylist.begin(), bodylist.end(), mbody.get_ptr() ) );
	
	// nullify backward link to system
	mbody->SetSystem(0);
	// this may delete the body, if none else's still referencing it..
	mbody->RemoveRef();
}

// Faster than RemoveLink because it does not require the linear time search
std::list<ChLink*>::iterator ChAssembly::RemoveLinkIter(std::list<ChLink*>::iterator& mlinkiter)
{
	// nullify backward link to system
	(*mlinkiter)->SetSystem(0);
	// this may delete the link, if none else's still referencing it..
	(*mlinkiter)->RemoveRef();

	return linklist.erase(mlinkiter);
}

void ChAssembly::RemoveLink (ChSharedPtr<ChLink> mlink)
{
	assert(std::find<std::list<ChLink*>::iterator>(linklist.begin(), linklist.end(), mlink.get_ptr() )!=linklist.end());

	// warning! linear time search, to erase pointer from container.
	linklist.remove(mlink.get_ptr());//erase(std::find<std::vector<ChBody*>::iterator>(bodylist.begin(), bodylist.end(), mbody.get_ptr() ) );
	
	// nullify backward link to system
	mlink->SetSystem(0);
	// this may delete the body, if none else's still referencing it..
	mlink->RemoveRef();
}

 
// collision stuff
void ChAssembly::SetCollide (bool mcoll)
{
	if (mcoll == do_collide) 
		return;

	if (mcoll)
	{
		SyncCollisionModels();
		this->do_collide=true;
		if (GetSystem())
		{
			AddCollisionModelsToSystem();
		}
	}
	else 
	{
		this->do_collide = false;
		if (GetSystem())
		{
			RemoveCollisionModelsFromSystem();
		}
	}
}

void ChAssembly::SyncCollisionModels()
{
	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
		Bpointer->GetCollisionModel()->SyncPosition();
		HIER_BODY_NEXT
	}
}

void ChAssembly::AddCollisionModelsToSystem() 
{
	assert(this->GetSystem());
	SyncCollisionModels();
	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
		if (Bpointer->GetCollide())
			this->GetSystem()->GetCollisionSystem()->Add(Bpointer->GetCollisionModel());
		HIER_BODY_NEXT
	}
}

void ChAssembly::RemoveCollisionModelsFromSystem() 
{
	assert(this->GetSystem());
	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
		if (Bpointer->GetCollide())
			this->GetSystem()->GetCollisionSystem()->Remove(Bpointer->GetCollisionModel());
		HIER_BODY_NEXT
	}
}



void ChAssembly::GetTotalAABB(ChVector<>& bbmin, ChVector<>& bbmax)
{
	//default infinite bb
	ChVector<> mmin(-1e200, -1e200, -1e200);
	ChVector<> mmax( 1e200,  1e200,  1e200);

	bool set = false;
	ChVector<> tmpMin;
	ChVector<> tmpMax;

	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
		if (Bpointer->GetCollisionModel())
		{
			Bpointer->GetCollisionModel()->GetAABB(tmpMin, tmpMax);
			if (!set)
			{
				mmin=tmpMin;
				mmax=tmpMax;
				set=true;
			}
			if (tmpMin.x<mmin.x)
				mmin.x=tmpMin.x;
			if (tmpMin.y<mmin.y)
				mmin.y=tmpMin.y;
			if (tmpMin.z<mmin.z)
				mmin.z=tmpMin.z;
			if (tmpMax.x>mmax.x)
				mmax.x=tmpMax.x;
			if (tmpMax.y>mmax.y)
				mmax.y=tmpMax.y;
			if (tmpMax.z>mmax.z)
				mmax.z=tmpMax.z;
		}
		HIER_BODY_NEXT
	}
	bbmin.Set(mmin.x, mmin.y, mmin.z);
	bbmax.Set(mmax.x, mmax.y, mmax.z);
}

void ChAssembly::Reference_LM_byID()
{
	ChMarker* m1;
	ChMarker* m2;

	HIER_LINK_INIT
	while HIER_LINK_NOSTOP
	{
		if (ChLinkMarkers* malink = ChDynamicCast(ChLinkMarkers,Lpointer))
		{
			m1 = SearchMarker(malink->GetMarkID1());
			m2 = SearchMarker(malink->GetMarkID2());
			malink->SetMarker1(m1);
			malink->SetMarker2(m2);
			if (m1 && m2)
			{
				Lpointer->SetValid(true);
				HIER_LINK_NEXT
			}
			else
			{
				Lpointer->SetValid(false);
				malink->SetMarkers(0,0); // however marker IDs will survive!!
				iterlink = RemoveLinkIter(iterlink); // may delete it...
			}
		}
		else
		{
			HIER_LINK_NEXT
		}
	}
}

ChMarker* ChAssembly::SearchMarker (int markID)
{
	ChMarker* candidate = NULL;
	ChMarker* res = NULL;

	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
		res = ChContainerSearchFromID<ChMarker, std::vector<ChMarker*>::iterator>
				(markID, 
				Bpointer->GetMarkerList()->begin(), 
				Bpointer->GetMarkerList()->end());
		if (res != NULL) return res;

		HIER_BODY_NEXT
	}

	return 0;
}

//////// FILE I/O

void ChAssembly::StreamOUT(ChStreamOutBinary& mstream)
{
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
	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
			// write the body
		//Bpointer->StreamOUT(mstream);
		mstream.AbstractWriteAll(Bpointer);
		//mstream.AbstractWrite(Bpointer);
		HIER_BODY_NEXT
	}

	// 3a) write how many links
	mstream << (int)linklist.size();

	// 3b) write links links
	HIER_LINK_INIT
	while HIER_LINK_NOSTOP
	{
			// Writethe link, using a special downcasting function Link_BinSave which saves also the
			// inheritance info, depending on link class inheritance from base Link*
		mstream.AbstractWrite(Lpointer);

		HIER_LINK_NEXT
	}
}

void ChAssembly::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChPhysicsItem::StreamIN(mstream);

	mstream >> do_collide;
	mstream >> do_limit_speed;
	
	mstream >> max_speed;
	mstream >> max_wvel;

	// 2a) read how many bodies
	int mnbodies = 0;
	mstream >> mnbodies;

	// 2b) read  bodies
	ChBody* newbody= NULL;
	for (int i= 0; i<mnbodies; i++)
	{
		//mstream.AbstractReadCreate(&newbody);
		mstream.AbstractReadAllCreate(&newbody);
		ChSharedPtr<ChBody> shitem(newbody);
		this->AddBody(shitem);
		/*
		ChSharedPtr<ChBody> newbody(new ChBody);
		this->AddBody(newbody);

		newbody->StreamIN(mstream);
		*/
	}

	// 3a) read how many links
	int mnlinks = 0;
	mstream >> mnlinks;

	// 3b) read  links
	ChLink* newlink= NULL;
	for (int j= 0; j<mnlinks; j++)
	{
			// read the link, using a special downcasting function Link_BinRead_Create which creates the
			// proper inherited object, depending on its class inheritance from base Link*

		mstream.AbstractReadCreate(&newlink);

		ChSharedPtr<ChLink> shlink(newlink);
		this->AddLink(shlink);
	}

	// 3c) Rebuild link pointers to markers
	this->Reference_LM_byID();
}




void ChAssembly::StreamOUTstate(ChStreamOutBinary& mstream)
{
	// Do not serialize parent classes and do not
	// implement versioning, because this must be efficient 
	// and will be used just for domain decomposition.
	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
			// write the body + child markers + forces
		Bpointer->StreamOUTstate(mstream);
		HIER_BODY_NEXT
	}
}

void ChAssembly::StreamINstate(ChStreamInBinary& mstream)
{
	// Do not serialize parent classes and do not
	// implement versioning, because this must be efficient 
	// and will be used just for domain decomposition.
	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
			// write the body + child markers + forces
		Bpointer->StreamINstate(mstream);
		HIER_BODY_NEXT
	}
}


#define CH_CHUNK_END_ASSEM 18881

int ChAssembly::StreamINall  (ChStreamInBinary& m_file)
{
	int mchunk = 0;
	ChBody* newbody= NULL;
	ChLink* newlink= NULL;

	// class version number
	int version = m_file.VersionRead();

	// 0) reset system to have no sub object child
	this->Clear();

	// 1) read system class data...
		// deserialize parent class too
	ChPhysicsItem::StreamIN(m_file);

	m_file >> do_collide;
	m_file >> do_limit_speed;

	m_file >> max_speed;
	m_file >> max_wvel;

	// 2a) read how many bodies
	int mnbodies = 0;
	m_file >> mnbodies;

	// 2b) read  bodies
	for (int i= 0; i<mnbodies; i++)
	{
		ChSharedPtr<ChBody> newbody(new ChBody);
		this->AddBody(newbody);

		if (!newbody->StreamINall(m_file)) 
			throw ChException("Cannot read body data");
	}

	// 3a) read how many links
	int mnlinks = 0;
	m_file >> mnlinks;

	// 3b) read  links
	for (int j= 0; j<mnlinks; j++)
	{
			// read the link, using a special downcasting function Link_BinRead_Create which creates the
			// proper inherited object, depending on its class inheritance from base Link*

		m_file.AbstractReadCreate(&newlink);
		if (!newlink) throw ChException("Cannot read link data");

		ChSharedPtr<ChLink> shlink(newlink);
		this->AddLink(shlink);
	}

	// 3c) Rebuild link pointers to markers
	this->Reference_LM_byID();

	m_file >> mchunk;

	if (mchunk != CH_CHUNK_END_ASSEM) return 0;

	return 1;
}


int ChAssembly::StreamOUTall  (ChStreamOutBinary& m_file)
{
	// class version number
	m_file.VersionWrite(1);

	// 1) write system class data...
		// serialize parent class too
	ChPhysicsItem::StreamOUT(m_file);

		// stream out all member data
	m_file << do_collide;
	m_file << do_limit_speed;

	m_file << max_speed;
	m_file << max_wvel;

	// 2a) write how many bodies
	m_file << (int)bodylist.size();

	// 2b) write  bodies
	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
			// write the body + child markers + forces
		if (!Bpointer->StreamOUTall(m_file)) return 0;
		HIER_BODY_NEXT
	}

	// 3a) write how many links
	m_file << (int)linklist.size(); 

	// 3b) write links links
	HIER_LINK_INIT
	while HIER_LINK_NOSTOP
	{
			// Writethe link, using a special downcasting function Link_BinSave which saves also the
			// inheritance info, depending on link class inheritance from base Link*
		m_file.AbstractWrite(Lpointer);

		HIER_LINK_NEXT
	}

	m_file << (int)CH_CHUNK_END_ASSEM;

	return 1;
}

void ChAssembly::StreamOUT(ChStreamOutAscii& mstream)
{
	//***TO DO***
}


int ChAssembly::StreamOUTall  (ChStreamOutAscii& mstream) // dump rigidbody and childrens (markers.forces)
{
	//***TO DO***

	StreamOUT (mstream);			 // 1) dump the body attrs

	return 1;
}


} // END_OF_NAMESPACE____


/////////////////////
