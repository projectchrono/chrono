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
	this->posX = 0.f;
	this->posY = 0.f;
	this->posZ = 0.f;
	nSpheres = 0;
	colSpheres = 0;
	rMax = 0.f;
	colFam = 0;
	noCollWith = -1;
}


ChModelGPU::~ChModelGPU()
{
	//ClearModel(); not possible, would call GetPhysicsItem() that is pure virtual, enough to use instead..
	if (colSpheres) delete colSpheres;
	colSpheres = 0;
}

 

int ChModelGPU::ClearModel()
{
	if (GetPhysicsItem()->GetSystem())
	{
	  if (GetPhysicsItem()->GetCollide())
	  {
		GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Remove(this);
	  }
	}
	
	if (colSpheres) delete colSpheres;

	posX = 0.f;
	posY = 0.f;
	posZ = 0.f;

	nSpheres=0;
	rMax = 0.f;
	colFam = 0;
	noCollWith = -1;

	return 1;
} 


int ChModelGPU::BuildModel()
{
	//assert((ChBody*)GetPhysicsItem());
	if (GetPhysicsItem()->GetSystem())
	  if (GetPhysicsItem()->GetCollide())
		GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Add(this);

	return 1;
}

bool ChModelGPU::AddCompoundBody(int numSpheres, double* pX, double* pY, double* pZ, double* pR)
{
	if(numSpheres<=0)
		return false;

	nSpheres = numSpheres;
	colSpheres = new float[4*nSpheres];
	
	for(int ii=0; ii<nSpheres; ii++)
	{
		colSpheres[ii*4+0] = (float)pX[ii];
		colSpheres[ii*4+1] = (float)pY[ii];
		colSpheres[ii*4+2] = (float)pZ[ii];
		colSpheres[ii*4+3] = (float)pR[ii];

		if(((float)pR[ii])>rMax)
			rMax = (float)pR[ii];
	}

	return true;
}

bool ChModelGPU::AddSphere(double radius,  ChVector<>* pos)
{
	//add a sphere body - this collision geometry has one collision sphere at the body coordinate origin
	nSpheres = 1;
	colSpheres = new float[4*nSpheres];
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

	colSpheres[0] = 0.f; //x
	colSpheres[1] = 0.f; //y
	colSpheres[2] = 0.f; //z
	colSpheres[3] = (float) radius; //r

	rMax = (float) radius;

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


ChVector<> ChModelGPU::GetSpherePos(int ID)
{
	//ChVector<> pos(colSpheres[4*n+0],colSpheres[4*n+1],colSpheres[4*n+2]);
	ChVector<> pos;
	pos.x = colSpheres[4*ID+0];
	pos.y = colSpheres[4*ID+1];
	pos.z = colSpheres[4*ID+2];
	return pos;
}

float ChModelGPU::GetSphereR(int ID)
{
	if (ID>nSpheres)
		return -1.f;

	return colSpheres[4*ID+3];
}

void  ChModelGPU::SetFamily(int mfamily)
{
	colFam = mfamily;
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
	return colFam;
	//return 1;
}

void  ChModelGPU::SetFamilyMaskNoCollisionWithFamily(int mfamily)
{
	noCollWith = mfamily;
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
	if(noCollWith = mfamily)
	{
		noCollWith = -1;
	}

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
	return (noCollWith!=mfamily);
	/*
	assert(mfamily<16);
	if (!bt_collision_object->getBroadphaseHandle()) return false;
	short int familyflag = (short int)0x1 << mfamily;
	return bt_collision_object->getBroadphaseHandle()->m_collisionFilterMask & familyflag;
	*/
	return true;
}

int ChModelGPU::GetNoCollFamily()
{
	return noCollWith;
}


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif  // end of ! CH_NOCUDA
