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

		// copy the parent class data...
	ChFrameMoving<double>::operator=(*source);

	do_collide = source->do_collide;
	do_limit_speed = source->do_limit_speed;

	max_speed = source->max_speed;
	max_wvel  = source->max_wvel;
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
		// remove from collision system
		if (Bpointer->GetCollide())
			Bpointer->RemoveCollisionModelsFromSystem(); 
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
	for (unsigned int j = 0; j < bodylist.size(); j++)
	{
		this->bodylist[j]->InjectVariables(mdescriptor);
	}
}


void ChAssembly::VariablesFbReset()
{
	for (unsigned int j = 0; j < bodylist.size(); j++)
	{
		this->bodylist[j]->Variables().Get_fb().FillElem(0.0);
	}
}

void ChAssembly::VariablesFbLoadForces(double factor)
{
	for (unsigned int j = 0; j < bodylist.size(); j++)
	{
		// add applied forces and torques (and also the gyroscopic torque and gravity!) to 'fb' vector
		this->bodylist[j]->Variables().Get_fb().PasteSumVector((this->bodylist[j]->Get_Xforce()) * factor ,0,0);
		this->bodylist[j]->Variables().Get_fb().PasteSumVector((this->bodylist[j]->Get_Xtorque()-this->bodylist[j]->Get_gyro())  * factor ,3,0);
	}
}


void ChAssembly::VariablesQbLoadSpeed()
{
	for (unsigned int j = 0; j < bodylist.size(); j++)
	{
		// set current speed in 'qb', it can be used by the LCP solver when working in incremental mode
		this->bodylist[j]->Variables().Get_qb().PasteVector(this->bodylist[j]->GetCoord_dt().pos,0,0);
		this->bodylist[j]->Variables().Get_qb().PasteVector(this->bodylist[j]->GetWvel_loc()    ,3,0);
	}
}


void ChAssembly::VariablesQbSetSpeed(double step)
{
	for (unsigned int j = 0; j < bodylist.size(); j++)
	{
		ChCoordsys<> old_coord_dt = this->bodylist[j]->GetCoord_dt();

		// from 'qb' vector, sets body speed, and updates auxiliary data
		this->bodylist[j]->SetPos_dt(   this->bodylist[j]->Variables().Get_qb().ClipVector(0,0) );
		this->bodylist[j]->SetWvel_loc( this->bodylist[j]->Variables().Get_qb().ClipVector(3,0) );

		// apply limits (if in speed clamping mode) to speeds.
		//ClampSpeed(); NO - do only per-particle, here.. (but.. really needed here?)

		// Compute accel. by BDF (approximate by differentiation);
		if (step)
		{
			this->bodylist[j]->SetPos_dtdt( (this->bodylist[j]->GetCoord_dt().pos - old_coord_dt.pos)  / step);
			this->bodylist[j]->SetRot_dtdt( (this->bodylist[j]->GetCoord_dt().rot - old_coord_dt.rot)  / step);
		}
	}
}

void ChAssembly::VariablesQbIncrementPosition(double dt_step)
{
	//if (!this->IsActive()) 
	//	return;

	for (unsigned int j = 0; j < bodylist.size(); j++)
	{
		// Updates position with incremental action of speed contained in the
		// 'qb' vector:  pos' = pos + dt * speed   , like in an Eulero step.

		ChVector<> newspeed = this->bodylist[j]->Variables().Get_qb().ClipVector(0,0);
		ChVector<> newwel   = this->bodylist[j]->Variables().Get_qb().ClipVector(3,0);

		// ADVANCE POSITION: pos' = pos + dt * vel
		this->bodylist[j]->SetPos( this->bodylist[j]->GetPos() + newspeed * dt_step);

		// ADVANCE ROTATION: rot' = [dt*wwel]%rot  (use quaternion for delta rotation)
		ChQuaternion<> mdeltarot;
		ChQuaternion<> moldrot = this->bodylist[j]->GetRot();
		ChVector<> newwel_abs = (*(this->bodylist[j]->GetA())) * newwel;
		double mangle = newwel_abs.Length() * dt_step;
		newwel_abs.Normalize();
		mdeltarot.Q_from_AngAxis(mangle, newwel_abs);
		ChQuaternion<> mnewrot = mdeltarot % moldrot;
		this->bodylist[j]->SetRot( mnewrot );
	}
}

void ChAssembly::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	std::list<ChLink*>::iterator iterlink = linklist.begin();
	while (iterlink != linklist.end())
	{
		(*iterlink)->InjectConstraints(mdescriptor);
		iterlink++;
	}
}

void ChAssembly::ConstraintsBiReset()
{
	std::list<ChLink*>::iterator iterlink = linklist.begin();
	while (iterlink != linklist.end())
	{
		(*iterlink)->ConstraintsBiReset();
		iterlink++;
	}
}

void ChAssembly::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) 
{
	std::list<ChLink*>::iterator iterlink = linklist.begin();
	while (iterlink != linklist.end())
	{
		(*iterlink)->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
		iterlink++;
	}
}

void ChAssembly::ConstraintsBiLoad_Ct(double factor)
{
	std::list<ChLink*>::iterator iterlink = linklist.begin();
	while (iterlink != linklist.end())
	{
		(*iterlink)->ConstraintsBiLoad_Ct(factor);
		iterlink++;
	}
}

void ChAssembly::ConstraintsBiLoad_Qc(double factor)
{
	std::list<ChLink*>::iterator iterlink = linklist.begin();
	while (iterlink != linklist.end())
	{
		(*iterlink)->ConstraintsBiLoad_Qc(factor);
		iterlink++;
	}
}

void ChAssembly::ConstraintsFbLoadForces(double factor)
{
	std::list<ChLink*>::iterator iterlink = linklist.begin();
	while (iterlink != linklist.end())
	{
		(*iterlink)->ConstraintsFbLoadForces(factor);
		iterlink++;
	}
}

void ChAssembly::ConstraintsLoadJacobians()
{
	std::list<ChLink*>::iterator iterlink = linklist.begin();
	while (iterlink != linklist.end())
	{
		(*iterlink)->ConstraintsLoadJacobians();
		iterlink++;
	}
}

void ChAssembly::ConstraintsLiLoadSuggestedSpeedSolution()
{
	std::list<ChLink*>::iterator iterlink = linklist.begin();
	while (iterlink != linklist.end())
	{
		(*iterlink)->ConstraintsLiLoadSuggestedSpeedSolution();
		iterlink++;
	}
}

void ChAssembly::ConstraintsLiLoadSuggestedPositionSolution()
{
	std::list<ChLink*>::iterator iterlink = linklist.begin();
	while (iterlink != linklist.end())
	{
		(*iterlink)->ConstraintsLiLoadSuggestedPositionSolution();
		iterlink++;
	}
}

void ChAssembly::ConstraintsLiFetchSuggestedSpeedSolution()
{
	std::list<ChLink*>::iterator iterlink = linklist.begin();
	while (iterlink != linklist.end())
	{
		(*iterlink)->ConstraintsLiFetchSuggestedSpeedSolution();
		iterlink++;
	}
}

void ChAssembly::ConstraintsLiFetchSuggestedPositionSolution()
{
	std::list<ChLink*>::iterator iterlink = linklist.begin();
	while (iterlink != linklist.end())
	{
		(*iterlink)->ConstraintsLiFetchSuggestedPositionSolution();
		iterlink++;
	}
}

void ChAssembly::ConstraintsFetch_react(double factor)
{
	std::list<ChLink*>::iterator iterlink = linklist.begin();
	while (iterlink != linklist.end())
	{
		(*iterlink)->ConstraintsFetch_react(factor);
		iterlink++;
	}
}


void ChAssembly::SetNoSpeedNoAcceleration()
{
	for (unsigned int j = 0; j < bodylist.size(); j++)
	{
		this->bodylist[j]->SetPos_dt(VNULL);
		this->bodylist[j]->SetWvel_loc(VNULL);
		this->bodylist[j]->SetPos_dtdt(VNULL);
		this->bodylist[j]->SetRot_dtdt(QNULL);
	}
}


////
void ChAssembly::ClampSpeed()
{
	if (this->GetLimitSpeed())
	{
		for (unsigned int j = 0; j < bodylist.size(); j++)
		{
			double w = 2.0*this->bodylist[j]->GetRot_dt().Length();
			if (w > max_wvel)
				this->bodylist[j]->SetRot_dt(this->bodylist[j]->GetRot_dt() * max_wvel/w);
		
			double v = this->bodylist[j]->GetPos_dt().Length();
			if (v > max_speed)
				this->bodylist[j]->SetPos_dt(this->bodylist[j]->GetPos_dt() * max_speed/v);
		}
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

	for (unsigned int j = 0; j < bodylist.size(); j++)
	{
		this->bodylist[j]->Update(ChTime);
	}
	std::list<ChLink*>::iterator iterlink = linklist.begin();
	while (iterlink != linklist.end())
	{
		(*iterlink)->Update(ChTime);
		iterlink++;
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
	if (newbody->GetCollide())
		newbody->AddCollisionModelsToSystem(); 
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
			for (unsigned int j = 0; j < bodylist.size(); j++)
			{
				GetSystem()->GetCollisionSystem()->Add(this->bodylist[j]->GetCollisionModel());
			}
		}
	}
	else 
	{
		this->do_collide = false;
		if (GetSystem())
		{
			for (unsigned int j = 0; j < bodylist.size(); j++)
			{
				GetSystem()->GetCollisionSystem()->Remove(this->bodylist[j]->GetCollisionModel());
			}
		}
	}
}

void ChAssembly::SyncCollisionModels()
{
	for (unsigned int j = 0; j < bodylist.size(); j++)
	{
		this->bodylist[j]->GetCollisionModel()->SyncPosition();
	}
}

void ChAssembly::AddCollisionModelsToSystem() 
{
	assert(this->GetSystem());
	SyncCollisionModels();
	for (unsigned int j = 0; j < bodylist.size(); j++)
	{
		this->GetSystem()->GetCollisionSystem()->Add(this->bodylist[j]->GetCollisionModel());
	}
}

void ChAssembly::RemoveCollisionModelsFromSystem() 
{
	assert(this->GetSystem());
	for (unsigned int j = 0; j < bodylist.size(); j++)
	{
		this->GetSystem()->GetCollisionSystem()->Remove(this->bodylist[j]->GetCollisionModel());
	}
}



void ChAssembly::GetTotalAABB(ChVector<>& bbmin, ChVector<>& bbmax)
{
	ChVector<> mmin(0.0, 0.0, 0.0);
	ChVector<> mmax(0.0, 0.0, 0.0);

	ChVector<> tmpMin;
	ChVector<> tmpMax;

	for (unsigned int j = 0; j < bodylist.size(); j++)
	{
		if (this->bodylist[j]->GetCollisionModel())
		{
			this->bodylist[j]->GetCollisionModel()->GetAABB(tmpMin, tmpMax);
			if (tmpMin.x<mmin.x)
				tmpMin.x=mmin.x;
			if (tmpMin.y<mmin.y)
				tmpMin.y=mmin.y;
			if (tmpMin.z<mmin.z)
				tmpMin.z=mmin.z;
			if (tmpMax.x>mmax.x)
				tmpMax.x=mmax.x;
			if (tmpMax.y>mmax.y)
				tmpMax.y=mmax.y;
			if (tmpMax.z>mmax.z)
				tmpMax.z=mmax.z;
		}

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

		// serialize parent class too
	ChFrameMoving<double>::StreamOUT(mstream);

		// stream out all member data
	mstream << do_collide;
	mstream << do_limit_speed;
	
	mstream << max_speed;
	mstream << max_wvel;
}

void ChAssembly::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChPhysicsItem::StreamIN(mstream);

		// deserialize parent class too
	ChFrameMoving<double>::StreamIN(mstream);

	mstream >> do_collide;
	mstream >> do_limit_speed;
	
	mstream >> max_speed;
	mstream >> max_wvel;
}




void ChAssembly::StreamOUTstate(ChStreamOutBinary& mstream)
{
	// Do not serialize parent classes and do not
	// implement versioning, because this must be efficient 
	// and will be used just for domain decomposition.
	StreamOUTall(mstream);
}

void ChAssembly::StreamINstate(ChStreamInBinary& mstream)
{
	// Do not serialize parent classes and do not
	// implement versioning, because this must be efficient 
	// and will be used just for domain decomposition.
	StreamINall(mstream);
}


#define CH_CHUNK_END 1234

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
	m_file >> *this;

	// 2a) read how many bodies
	int mnbodies = 0;
	m_file >> mnbodies;

	// 2b) read  bodies
	for (int i= 0; i<mnbodies; i++)
	{
		ChSharedBodyPtr newbody(new ChBody);
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

		ChSharedLinkPtr shlink(newlink);
		this->AddLink(shlink);
	}

	// 3c) Rebuild link pointers to markers
	this->Reference_LM_byID();

	return 1;
}


int ChAssembly::StreamOUTall  (ChStreamOutBinary& m_file)
{
	// class version number
	m_file.VersionWrite(1);

	// 1) write system class data...
	m_file << *this;

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

	m_file << (int)CH_CHUNK_END;

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
