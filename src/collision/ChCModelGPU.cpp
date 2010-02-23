//////////////////////////////////////////////////
//  
//   ChCModelGPU.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com 
// ------------------------------------------------
///////////////////////////////////////////////////
 
#ifndef CH_NOCUDA 
 
 
#include "ChCModelGPU.h" 
#include "physics/ChBody.h"
#include "physics/ChSystem.h"
#include "GIMPACT/Bullet/btGImpactCollisionAlgorithm.h"
#include "GIMPACTUtils/btGImpactConvexDecompositionShape.h"


namespace chrono 
{
namespace collision 
{



ChModelGPU::ChModelGPU()
{
	this->rad = 0.f;
	this->posX = 0.f;
	this->posY = 0.f;
	this->posZ = 0.f;
	rigidBody = 0;
}


ChModelGPU::~ChModelGPU()
{
	ClearModel();
	rad=0;
	posX=0;
	posY=0;
	posZ=0;
}

 

int ChModelGPU::ClearModel()
{
	if (GetBody()->GetSystem())
	  if (GetBody()->GetCollide())
		GetBody()->GetSystem()->GetCollisionSystem()->Remove(this);
	
	rad=0.f;
	posX = 0.f;
	posY = 0.f;
	posZ = 0.f;

	return 1;
} 


int ChModelGPU::BuildModel()
{
	assert(GetBody());
	if (GetBody()->GetSystem())
	  if (GetBody()->GetCollide())
		GetBody()->GetSystem()->GetCollisionSystem()->Add(this);

	return 1;
}

bool ChModelGPU::AddSphere(double radius,  ChVector<>* pos)
{
	rad = radius;
	if(pos==0)
	{
		posX=0.f;
		posY=0.f;
		posZ=0.f;
	}
	else
	{
		posX = (*pos).x;
		posY = (*pos).y;
		posZ = (*pos).z;
	}

	return true;
}
 
bool ChModelGPU::AddEllipsoid(double rx,  double ry,  double rz, ChVector<>* pos, ChMatrix33<>* rot)
{
	//NOT SUPPORTED
	return false;
}
 
bool ChModelGPU::AddBox(double hx, double hy, double hz, ChVector<>* pos, ChMatrix33<>* rot)
{
	//NOT SUPPORTED
	return false;
}
 
	 /// Add a cylinder to this model (default axis on Y direction), for collision purposes
bool ChModelGPU::AddCylinder (double rx, double rz, double hy, ChVector<>* pos, ChMatrix33<>* rot)
{
	//NOT SUPPORTED
	return false;
}

bool ChModelGPU::AddBarrel (double Y_low, double Y_high, double R_vert, double R_hor, double R_offset, ChVector<>* pos, ChMatrix33<>* rot)
{
	//NOT SUPPORTED
	return false;
}


	/// Add a triangle mesh to this model
bool ChModelGPU::AddTriangleMesh (const  geometry::ChTriangleMesh& trimesh,	bool is_static, bool is_convex,  ChVector<>* pos, ChMatrix33<>* rot)
{
	//NOT SUPPORTED
	return false;
}

bool ChModelGPU::AddCopyOfAnotherModel (ChCollisionModel* another)
{
	//NOT SUPPORTED
	return false;
}




ChBody* ChModelGPU::GetBody()
{
	return rigidBody;
	//OLD STUFF BELOW
	/*
	return (ChBody*) bt_collision_object->getUserPointer();
	*/
}

void ChModelGPU::SetBody(ChBody* mbo)
{
	rigidBody = mbo;
	//OLD STUFF BELOW
	/*
	bt_collision_object->setUserPointer((void*) mbo);
	*/
}



void ChModelGPU::SyncPosition()
{
	ChBody* bpointer = GetBody();
	assert(bpointer);
	assert(bpointer->GetSystem());

	posX=bpointer->GetPos().x;
	posY=bpointer->GetPos().y;
	posZ=bpointer->GetPos().z;
}

void  ChModelGPU::SetFamily(int mfamily)
{
	/*
	assert(mfamily<16);
	if (!bt_collision_object->getBroadphaseHandle()) return;

	// Trick to avoid troubles if setting mask or family when model is already overlapping to some other model
	ChCollisionSystem* mcosys =this->GetBody()->GetSystem()->GetCollisionSystem();
	short int original_mask = bt_collision_object->getBroadphaseHandle()->m_collisionFilterMask;
	mcosys->Remove(this);
	ChVector<>original_pos = this->GetBody()->GetPos();
	this->GetBody()->SetPos(ChVector<>(100,100,100)); // hopefully out of overlapping pairs
	this->SyncPosition();
	mcosys->Add(this); 

	bt_collision_object->getBroadphaseHandle()->m_collisionFilterGroup = (short int)0x1 << mfamily;

	bt_collision_object->getBroadphaseHandle()->m_collisionFilterMask = original_mask; // restore mask, cause mcosys->Add() has resetted
	this->GetBody()->SetPos(original_pos); // back to original position - future broadphase operations will create proper coll.pairs
	this->SyncPosition();	
	*/
}

int   ChModelGPU::GetFamily()
{
	/*
	int fam;
	if (!bt_collision_object->getBroadphaseHandle()) return -1;
	for (fam = 0; fam < 16; fam++)
		if (((short int)0x1 << fam) &	bt_collision_object->getBroadphaseHandle()->m_collisionFilterGroup)
			return fam;
	return fam;
	*/
	return 1;
}

void  ChModelGPU::SetFamilyMaskNoCollisionWithFamily(int mfamily)
{
	/*
	assert(mfamily<16);
	if (!bt_collision_object->getBroadphaseHandle()) return;

	// Trick to avoid troubles if setting mask or family when model is already overlapping to some other model
	ChCollisionSystem* mcosys =this->GetBody()->GetSystem()->GetCollisionSystem();
	short int original_family = bt_collision_object->getBroadphaseHandle()->m_collisionFilterGroup;
	mcosys->Remove(this);
	ChVector<>original_pos = this->GetBody()->GetPos();
	this->GetBody()->SetPos(ChVector<>(100,100,100)); // hopefully out of overlapping pairs
	this->SyncPosition();
	mcosys->Add(this); 

	short int familyflag = (short int)0x1 << mfamily;
	bt_collision_object->getBroadphaseHandle()->m_collisionFilterMask  &= ~familyflag;

	bt_collision_object->getBroadphaseHandle()->m_collisionFilterGroup = original_family; // restore group, cause mcosys->Add() has resetted
	this->GetBody()->SetPos(original_pos); // back to original position - future broadphase operations will create proper coll.pairs
	this->SyncPosition();
	*/
} 

void  ChModelGPU::SetFamilyMaskDoCollisionWithFamily(int mfamily)
{
	/*
	assert(mfamily<16);
	if (!bt_collision_object->getBroadphaseHandle()) return;

	// Trick to avoid troubles if setting mask or family when model is already overlapping to some other model
	ChCollisionSystem* mcosys =this->GetBody()->GetSystem()->GetCollisionSystem();
	short int original_family = bt_collision_object->getBroadphaseHandle()->m_collisionFilterGroup;
	mcosys->Remove(this);
	ChVector<>original_pos = this->GetBody()->GetPos();
	this->GetBody()->SetPos(ChVector<>(1000,1000,1000)); // hopefully out of overlapping pairs
	this->SyncPosition();
	mcosys->Add(this); 

	short int familyflag = (short int)0x1 << mfamily;
	bt_collision_object->getBroadphaseHandle()->m_collisionFilterMask |= familyflag;

	bt_collision_object->getBroadphaseHandle()->m_collisionFilterGroup = original_family; // restore group, cause mcosys->Add() has resetted
	this->GetBody()->SetPos(original_pos); // back to original position - future broadphase operations will create proper coll.pairs
	this->SyncPosition();
	*/
}

bool ChModelGPU::GetFamilyMaskDoesCollisionWithFamily(int mfamily)
{
	/*
	assert(mfamily<16);
	if (!bt_collision_object->getBroadphaseHandle()) return false;
	short int familyflag = (short int)0x1 << mfamily;
	return bt_collision_object->getBroadphaseHandle()->m_collisionFilterMask & familyflag;
	*/
	return true;
}


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif  // end of ! CH_NOCUDA
