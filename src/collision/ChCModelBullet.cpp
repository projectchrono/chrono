//////////////////////////////////////////////////
//  
//   ChCModelBullet.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com 
// ------------------------------------------------
///////////////////////////////////////////////////
 

 
#include "ChCModelBullet.h" 
#include "physics/ChPhysicsItem.h"
#include "physics/ChSystem.h"
#include "collision/bullet/btBulletCollisionCommon.h"
#include "GIMPACT/Bullet/btGImpactCollisionAlgorithm.h"
#include "GIMPACTUtils/btGImpactConvexDecompositionShape.h"
#include "BulletCollision/CollisionShapes/btBarrelShape.h"
#include "collision/ChCCollisionSystemBullet.h"
#include "BulletWorldImporter/btBulletWorldImporter.h"
#include "collision/ChCConvexDecomposition.h"


namespace chrono 
{



namespace collision 
{



ChModelBullet::ChModelBullet()
{
	bt_collision_object = new btCollisionObject;
	bt_collision_object->setCollisionShape(0);
	bt_collision_object->setUserPointer((void*) this);

	this->family_group = 1;
	this->family_mask  = 0xFF;

	shapes.clear();


}


ChModelBullet::~ChModelBullet()
{ 
	//ClearModel(); not possible, would call GetPhysicsItem() that is pure virtual, enough to use instead..
	shapes.clear();

	bt_collision_object->setCollisionShape(0);

	if (bt_collision_object) delete bt_collision_object;
	bt_collision_object=0;
}

 

int ChModelBullet::ClearModel()
{

	// delete previously added shapes, if collision shape(s) used by collision object
	if(shapes.size()>0)
	{
		// deletes shared pointers, so also deletes shapes if uniquely referenced
		shapes.clear(); 

		// tell to the parent collision system to remove this from collision system,
		// if still connected to a physical system 
		if (GetPhysicsItem()->GetSystem())
		  if (GetPhysicsItem()->GetCollide())
			GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Remove(this);
		
		// at the end, no collision shape
		bt_collision_object->setCollisionShape(0);
	}

	return 1;
} 


int ChModelBullet::BuildModel()
{
	//assert (GetPhysicsItem());
	
	// insert again (we assume it was removed by ClearModel!!!)
	if (GetPhysicsItem()->GetSystem())
	  if (GetPhysicsItem()->GetCollide())
		GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Add(this);

	return 1;
}

static btVector3 ChVectToBullet(const ChVector<>* pos)
{
	return btVector3((btScalar)pos->x,(btScalar)pos->y,(btScalar)pos->z);
}

static void ChPosMatrToBullet (ChVector<>* pos, ChMatrix33<>* rA, btTransform& mtrasform)
{
	btMatrix3x3 basisA( (btScalar)(*rA)(0,0), (btScalar)(*rA)(0,1), (btScalar)(*rA)(0,2),
						(btScalar)(*rA)(1,0), (btScalar)(*rA)(1,1), (btScalar)(*rA)(1,2),
						(btScalar)(*rA)(2,0), (btScalar)(*rA)(2,1), (btScalar)(*rA)(2,2));
	mtrasform.setBasis(basisA);
	mtrasform.setOrigin(btVector3(
								(btScalar)pos->x,
								(btScalar)pos->y,
								(btScalar)pos->z));
}
static void ChCoordsToBullet (ChCoordsys<>& mcoords, btTransform& mtrasform)
{
	ChMatrix33<> rA;
	rA.Set_A_quaternion(mcoords.rot);
	btMatrix3x3 basisA( (btScalar)rA(0,0), (btScalar)rA(0,1), (btScalar)rA(0,2),
						(btScalar)rA(1,0), (btScalar)rA(1,1), (btScalar)rA(1,2),
						(btScalar)rA(2,0), (btScalar)rA(2,1), (btScalar)rA(2,2));
	mtrasform.setBasis(basisA);
	mtrasform.setOrigin(btVector3(
								(btScalar)mcoords.pos.x,
								(btScalar)mcoords.pos.y,
								(btScalar)mcoords.pos.z));
}

void ChModelBullet::_injectShape(ChVector<>* pos, ChMatrix33<>* rot, btCollisionShape* mshape)
{
	ChVector<>   defpos = VNULL;
	ChMatrix33<> defrot;
	defrot.Set33Identity();

	if (!pos)
		pos = &defpos;
	if (!rot)
		rot = &defrot;

	bool centered = false;
	if ((*pos==defpos)&&(*rot==defrot))
		centered = true;

	// start_vector = ||    -- description is still empty
	if (shapes.size()==0)
	{
		if (centered)
		{
			 shapes.push_back(ChSmartPtr<btCollisionShape>(mshape)); 
			bt_collision_object->setCollisionShape(mshape);
			// end_vector=  | centered shape | 
			return;
		}
		else
		{
			btCompoundShape* mcompound = new btCompoundShape(true);
			 shapes.push_back(ChSmartPtr<btCollisionShape>(mcompound)); 
			 shapes.push_back(ChSmartPtr<btCollisionShape>(mshape)); 
			bt_collision_object->setCollisionShape(mcompound);
			btTransform mtrasform;
			ChPosMatrToBullet(pos,rot, mtrasform);
			mcompound->addChildShape(mtrasform, mshape);
			// vector=  | compound | not centered shape |
			return;
		}
	}
	// start_vector = | centered shape |    ----just a single centered shape was added
	if (shapes.size()==1)
	{
		btTransform mtrasform;
		 shapes.push_back(shapes[0]);
		 shapes.push_back(ChSmartPtr<btCollisionShape>(mshape));
		btCompoundShape* mcompound = new btCompoundShape(true);
		 shapes[0] = ChSmartPtr<btCollisionShape>(mcompound); 
		bt_collision_object->setCollisionShape(mcompound);
		ChPosMatrToBullet(&defpos, &defrot, mtrasform);
		mcompound->addChildShape(mtrasform, shapes[1].get_ptr()); 
		ChPosMatrToBullet(pos,rot, mtrasform);
		mcompound->addChildShape(mtrasform, shapes[2].get_ptr()); 
		// vector=  | compound | old centered shape | new shape | ...
		return;
	}
	// vector=  | compound | old | old.. |   ----already working with compounds..
	if (shapes.size()>1)
	{
		btTransform mtrasform;
		 shapes.push_back(ChSmartPtr<btCollisionShape>(mshape)); 
		ChPosMatrToBullet(pos,rot, mtrasform);
		btCollisionShape* mcom = shapes[0].get_ptr(); 
		((btCompoundShape*)mcom)->addChildShape(mtrasform, mshape); 
		// vector=  | compound | old | old.. | new shape | ...
		return;
	}
}

bool ChModelBullet::AddSphere(double radius,  ChVector<>* pos)
{
	// adjust default inward 'safe' margin (always as radius)
	this->SetSafeMargin((btScalar)radius);

	btSphereShape* mshape = new btSphereShape((btScalar) (radius + this->GetEnvelope()) );

	mshape->setMargin((btScalar)this->GetSuggestedFullMargin() );

	_injectShape (pos, 0, mshape);

	model_type=SPHERE;
	return true;
}
 
bool ChModelBullet::AddEllipsoid(double rx,  double ry,  double rz, ChVector<>* pos, ChMatrix33<>* rot)
{
	btScalar rad=1.0;
	btVector3 spos(0,0,0);
	double arx = rx + this->GetEnvelope();
	double ary = ry + this->GetEnvelope();
	double arz = rz + this->GetEnvelope();
	double mmargin = GetSuggestedFullMargin();
	btMultiSphereShape* mshape = new btMultiSphereShape(
						//btVector3(1,1,1),
						&spos,&rad,1);
	mshape->setLocalScaling(btVector3(  (btScalar)arx,
										(btScalar)ary,
										(btScalar)arz ));

	mshape->setMargin((btScalar)ChMin(mmargin, 0.9*ChMin(ChMin(arx,ary),arz)));

	_injectShape (	pos, rot, mshape);

	model_type=ELLIPSOID;
	return true;
}
 
bool ChModelBullet::AddBox(double hx, double hy, double hz, ChVector<>* pos, ChMatrix33<>* rot)
{
	// adjust default inward margin (if object too thin)
	this->SetSafeMargin((btScalar)ChMin(this->GetSafeMargin(), 0.2*ChMin(ChMin(hx,hy),hz)));

	double ahx = hx + this->GetEnvelope(); 
	double ahy = hy + this->GetEnvelope();
	double ahz = hz + this->GetEnvelope();
	btBoxShape* mshape = new btBoxShape(btVector3(	(btScalar)ahx, 
													(btScalar)ahy, 
													(btScalar)ahz));

	mshape->setMargin((btScalar)this->GetSuggestedFullMargin() );

	_injectShape (pos,rot, mshape);

	model_type=BOX;
	return true;
}
 
	 /// Add a cylinder to this model (default axis on Y direction), for collision purposes
bool ChModelBullet::AddCylinder (double rx, double rz, double hy, ChVector<>* pos, ChMatrix33<>* rot)
{
		// adjust default inward margin (if object too thin)
	this->SetSafeMargin((btScalar)ChMin(this->GetSafeMargin(), 0.2*ChMin(ChMin(rx,rz),(hy*0.5) )));

	double arx = rx + this->GetEnvelope();
	double arz = rz + this->GetEnvelope();
	double ahy = hy + this->GetEnvelope();
	
	btCylinderShape* mshape = new btCylinderShape(btVector3(	
						(btScalar)(arx), 
						(btScalar)(ahy), 
						(btScalar)(arz) ));
	mshape->setMargin((btScalar)this->GetSuggestedFullMargin() );

	_injectShape (pos,rot, mshape);

	model_type=CYLINDER;
	return true;
}


bool ChModelBullet::AddBarrel (double Y_low, double Y_high, double R_vert, double R_hor, double R_offset, ChVector<>* pos, ChMatrix33<>* rot)
{
		// adjust default inward margin (if object too thin)
	this->SetSafeMargin((btScalar)ChMin(this->GetSafeMargin(), 0.15*ChMin(ChMin(R_vert,R_hor),(Y_high-Y_low) )));

	btBarrelShape* mshape = new btBarrelShape(
		(btScalar)(Y_low  - this->model_envelope),  
		(btScalar)(Y_high + this->model_envelope),  
		(btScalar)(R_vert + this->model_envelope),  
		(btScalar)(R_hor  + this->model_envelope),  
		(btScalar)(R_offset) );

	mshape->setMargin((btScalar)this->GetSuggestedFullMargin() );

	_injectShape (pos,rot, mshape);

	model_type=BARREL;
	return true;
}



bool ChModelBullet::AddConvexHull (std::vector< ChVector<double> >& pointlist, ChVector<>* pos, ChMatrix33<>* rot)
{
		// adjust default inward margin (if object too thin)
	//this->SetSafeMargin((btScalar)ChMin(this->GetSafeMargin(), ... );

	btConvexHullShape* mshape = new btConvexHullShape;
	
	mshape->setMargin((btScalar)this->GetSuggestedFullMargin() );

	// ***TO DO*** shrink the convex hull by GetSafeMargin()
	for (unsigned int i = 0; i < pointlist.size(); i++)
	{
		mshape->addPoint( btVector3( 
			(btScalar)pointlist[i].x,
			(btScalar)pointlist[i].y,
			(btScalar)pointlist[i].z) );
	}

	mshape->setMargin((btScalar)this->GetSuggestedFullMargin() );
	mshape->recalcLocalAabb();
	/*
	btTransform mtr(btQuaternion(0,0,0));
	btVector3 mmin, mmax;
	mshape->getAabb(mtr,mmin,mmax);

	GetLog() << "\nAAABB min  " << (double)mmin.getX() << "   "  << (double)mmin.getY() << "   " << (double)mmin.getZ() << "\n" ;
	GetLog() << "AAABB max  " << (double)mmax.getX() << "   "  << (double)mmax.getY() << "   " << (double)mmax.getZ() << "\n" ;
	*/
	_injectShape (pos,rot, mshape);

	model_type=CONVEXHULL;
	return true;
}


// These classes inherits the Bullet triangle mesh, but adds just a single feature:
// when this shape is deleted, also the referenced triangle mesh interface is deleted.
// Hence, when a btBvhTriangleMeshShape_handlemesh is added to the list of shapes of this ChModelBullet,
// there's no need to remember to delete the mesh interface because it dies with the model, when shapes are deleted.
// This is just to avoid adding a pointer to a triangle interface in all collision models, when not needed. 

class btBvhTriangleMeshShape_handlemesh : public btBvhTriangleMeshShape
{
	btStridingMeshInterface* minterface;
public:
	btBvhTriangleMeshShape_handlemesh(btStridingMeshInterface* meshInterface) : 
			btBvhTriangleMeshShape(meshInterface ,true), 
			minterface(meshInterface) 
	{};

	~btBvhTriangleMeshShape_handlemesh()
	{
		if (minterface) delete minterface; minterface = 0; // also delete the mesh interface
	}
};

class btConvexTriangleMeshShape_handlemesh : public btConvexTriangleMeshShape
{
	btStridingMeshInterface* minterface;
public:
	btConvexTriangleMeshShape_handlemesh(btStridingMeshInterface* meshInterface) : 
			btConvexTriangleMeshShape(meshInterface), 
			minterface(meshInterface) {};

	~btConvexTriangleMeshShape_handlemesh()
	{
		if (minterface) delete minterface; minterface = 0; // also delete the mesh interface
	}
};


class btGImpactConvexDecompositionShape_handlemesh : public btGImpactConvexDecompositionShape 
{
	btStridingMeshInterface* minterface;
public:
	btGImpactConvexDecompositionShape_handlemesh(btStridingMeshInterface* meshInterface) :  
	        btGImpactConvexDecompositionShape (meshInterface, btVector3(1.f,1.f,1.f), btScalar(0.01)),
			minterface(meshInterface) 
			{
				//this->setLocalScaling(btVector3(1.f,1.f,1.f));
			};

	virtual ~btGImpactConvexDecompositionShape_handlemesh()
	{
		if (minterface) delete minterface; minterface = 0; // also delete the mesh interface
	} 
};
 
class btGImpactMeshShape_handlemesh : public btGImpactMeshShape
{
	btStridingMeshInterface* minterface;
public:
	btGImpactMeshShape_handlemesh(btStridingMeshInterface* meshInterface) : 
			btGImpactMeshShape(meshInterface), 
			minterface(meshInterface) 
			{
				//this->setLocalScaling(btVector3(1.f,1.f,1.f));
			};

	virtual ~btGImpactMeshShape_handlemesh()
	{
		if (minterface) delete minterface; minterface = 0; // also delete the mesh interface
	} 
};


	/// Add a triangle mesh to this model
bool ChModelBullet::AddTriangleMesh (const  geometry::ChTriangleMesh& trimesh,	bool is_static, bool is_convex,  ChVector<>* pos, ChMatrix33<>* rot)
{
	if (!trimesh.getNumTriangles()) 
		return false;

	btTriangleMesh* bulletMesh = new btTriangleMesh;
	for (int i=0; i<trimesh.getNumTriangles(); i++)
	{
		//bulletMesh->m_weldingThreshold = ...
		bulletMesh->addTriangle(
			ChVectToBullet(&trimesh.getTriangle(i).p1), 
			ChVectToBullet(&trimesh.getTriangle(i).p2), 
			ChVectToBullet(&trimesh.getTriangle(i).p3), 
			true); // try to remove duplicate vertices
	}
	
	if (is_static) 
	{ 
		// Here a static btBvhTriangleMeshShape should suffice, but looks like that the btGImpactMeshShape works better..
		 btCollisionShape* pShape = (btBvhTriangleMeshShape*) new btBvhTriangleMeshShape_handlemesh(bulletMesh);
	     pShape->setMargin((btScalar) this->GetSafeMargin() );
		// ((btBvhTriangleMeshShape*)pShape)->refitTree();
		//btCollisionShape* pShape = new btGImpactMeshShape_handlemesh(bulletMesh);
		//pShape->setMargin((btScalar) this->GetSafeMargin() );
		//((btGImpactMeshShape_handlemesh*)pShape)->updateBound();
		_injectShape (pos,rot, pShape);
	}
	else
	{
		if (is_convex)
		{
			btCollisionShape* pShape = (btConvexTriangleMeshShape*) new btConvexTriangleMeshShape_handlemesh(bulletMesh);
			pShape->setMargin( (btScalar) this->GetEnvelope() );
			_injectShape (pos,rot, pShape);
		} 
		else
		{
			
			/*
		      // use this (using GImpact) :
			  this->AddTriangleMeshConcave(trimesh,pos,rot);
			*/
	    
			
			   // ..or use this (using a defalt convex decomposition) : 
			ChConvexDecomposition mydecomposition;
			mydecomposition.AddTriangleMesh(trimesh);
			mydecomposition.ComputeConvexDecomposition(0, // skin width
														9, 64, // depht, max vertices in hull
														5, // concavity percent
														5, // merge treshold percent
														5, // split threshold percent
														true, // use initial island generation 
														false // use island generation (unsupported-disabled)
														);
			//mydecomposition.WriteConvexHullsAsWavefrontObj(ChStreamOutAsciiFile("test_hulls.obj")); // debug
			this->AddTriangleMeshConcaveDecomposed(mydecomposition, pos, rot);

			/*
		       // or this (using the default GI convex decomposition) :
			btCollisionShape* pShape = (btGImpactConvexDecompositionShape_handlemesh*) new btGImpactConvexDecompositionShape_handlemesh(bulletMesh);
			 pShape->setMargin( (btScalar) this->GetEnvelope() ); 
			((btGImpactMeshShape_handlemesh*)pShape)->updateBound();
			_injectShape (pos,rot, pShape);
			*/

		}
	}
	model_type=TRIANGLEMESH;
	return true;
}



bool ChModelBullet::AddTriangleMeshConcave(const  geometry::ChTriangleMesh& trimesh,	///< the concave triangle mesh
								ChVector<>* pos, ChMatrix33<>* rot ///< displacement respect to COG (optional)
								)
{
	if (!trimesh.getNumTriangles()) 
		return false;

	btTriangleMesh* bulletMesh = new btTriangleMesh;
	for (int i=0; i<trimesh.getNumTriangles(); i++)
	{
		//bulletMesh->m_weldingThreshold = ...
		bulletMesh->addTriangle(
			ChVectToBullet(&trimesh.getTriangle(i).p1), 
			ChVectToBullet(&trimesh.getTriangle(i).p2), 
			ChVectToBullet(&trimesh.getTriangle(i).p3), 
			true); // try to remove duplicate vertices
	}

	// Use the GImpact custom mesh-mesh algorithm
	btCollisionShape* pShape = (btGImpactMeshShape*) new btGImpactMeshShape_handlemesh(bulletMesh);
	pShape->setMargin( (btScalar) this->GetEnvelope() ); 
	this->SetSafeMargin(0);

	((btGImpactMeshShape_handlemesh*)pShape)->updateBound();
	_injectShape (pos,rot, pShape);

	return true;
}

bool ChModelBullet::AddTriangleMeshConcaveDecomposed(
							    ChConvexDecomposition& mydecomposition,
								ChVector<>* pos, ChMatrix33<>* rot ///< displacement respect to COG (optional)
								)
{
	// note: since the convex hulls are ot shrunk, the safe margin will be set to zero (must be set before adding them)
	this->SetSafeMargin(0);

	for (unsigned int j = 0; j< mydecomposition.GetHullCount(); j++)
	{
		std::vector< ChVector<double> > ptlist;

		CONVEX_DECOMPOSITION::ConvexHullResult result;
		if (!mydecomposition.GetDecompositionObject()->getConvexHullResult(j, result)) return false;
		for (unsigned int i=0; i<result.mVcount; i++)
		{
			ChVector<double> pt ( (double)result.mVertices[3*i+0], (double)result.mVertices[3*i+1], (double)result.mVertices[3*i+2] );
			ptlist.push_back(pt);
		}

		if (ptlist.size())
			this->AddConvexHull(ptlist,pos,rot);
	}

	return true;
}



bool ChModelBullet::AddCopyOfAnotherModel (ChCollisionModel* another)
{
	//this->ClearModel();
	this->shapes.clear(); // this will also delete owned shapes, if any, thank to shared pointers in 'shapes' vector

	this->SetSafeMargin(another->GetSafeMargin());
	this->SetEnvelope  (another->GetEnvelope());

	this->bt_collision_object->setCollisionShape(((ChModelBullet*)another)->GetBulletModel()->getCollisionShape());
	this->shapes = ((ChModelBullet*)another)->shapes;

	return true;
}





void  ChModelBullet::SetFamily(int mfamily)
{
	assert(mfamily<16);

	if (!bt_collision_object->getBroadphaseHandle()) return;

	this->SyncPosition();

	// Trick to avoid troubles if setting mask or family when model is already overlapping to some other model
	ChCollisionSystem* mcosys =this->GetPhysicsItem()->GetSystem()->GetCollisionSystem();
	//short int original_mask = bt_collision_object->getBroadphaseHandle()->m_collisionFilterMask;
	mcosys->Remove(this);

	this->family_group = (short int)0x1 << mfamily;

	ChCollisionSystemBullet* mcs = (ChCollisionSystemBullet*) mcosys;
	mcs->GetBulletCollisionWorld()->addCollisionObject(bt_collision_object, this->family_group , this->family_mask);
}

int   ChModelBullet::GetFamily()
{
	int fam;
	if (!bt_collision_object->getBroadphaseHandle()) return -1;
	for (fam = 0; fam < 16; fam++)
		if (((short int)0x1 << fam) &	bt_collision_object->getBroadphaseHandle()->m_collisionFilterGroup)
			return fam;
	return fam;
}

void  ChModelBullet::SetFamilyMaskNoCollisionWithFamily(int mfamily)
{
	assert(mfamily<16);

	if (!bt_collision_object->getBroadphaseHandle()) return;

	this->SyncPosition();

	// Trick to avoid troubles if setting mask or family when model is already overlapping to some other model
	ChCollisionSystem* mcosys =this->GetPhysicsItem()->GetSystem()->GetCollisionSystem();
	//short int original_family = bt_collision_object->getBroadphaseHandle()->m_collisionFilterGroup;
	//short int original_mask   = bt_collision_object->getBroadphaseHandle()->m_collisionFilterMask;
	mcosys->Remove(this);

	short int familyflag = (short int)0x1 << mfamily;
	this->family_mask = this->family_mask & ~familyflag;

	ChCollisionSystemBullet* mcs = (ChCollisionSystemBullet*) mcosys;
	mcs->GetBulletCollisionWorld()->addCollisionObject(bt_collision_object, this->family_group , this->family_mask);
} 

void  ChModelBullet::SetFamilyMaskDoCollisionWithFamily(int mfamily)
{
	assert(mfamily<16);

	if (!bt_collision_object->getBroadphaseHandle()) return;

	this->SyncPosition();

	// Trick to avoid troubles if setting mask or family when model is already overlapping to some other model
	ChCollisionSystem* mcosys =this->GetPhysicsItem()->GetSystem()->GetCollisionSystem();
	//short int original_family = bt_collision_object->getBroadphaseHandle()->m_collisionFilterGroup;
	//short int original_mask   = bt_collision_object->getBroadphaseHandle()->m_collisionFilterMask;
	mcosys->Remove(this);

	short int familyflag = (short int)0x1 << mfamily;
	this->family_mask = this->family_mask | familyflag;

	ChCollisionSystemBullet* mcs = (ChCollisionSystemBullet*) mcosys;
	mcs->GetBulletCollisionWorld()->addCollisionObject(bt_collision_object, this->family_group , this->family_mask);
}

bool ChModelBullet::GetFamilyMaskDoesCollisionWithFamily(int mfamily)
{
	assert(mfamily<16);
	if (!bt_collision_object->getBroadphaseHandle()) return false;
	short int familyflag = (short int)0x1 << mfamily;
	return (bt_collision_object->getBroadphaseHandle()->m_collisionFilterMask & familyflag) != 0;
}





void ChModelBullet::GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const
{
	btVector3 btmin;
	btVector3 btmax;
	if (bt_collision_object->getCollisionShape())
		bt_collision_object->getCollisionShape()->getAabb( bt_collision_object->getWorldTransform(), btmin, btmax);
	bbmin.Set(btmin.x(), btmin.y(), btmin.z());
	bbmax.Set(btmax.x(), btmax.y(), btmax.z());
}


void __recurse_add_newcollshapes(btCollisionShape* ashape, std::vector<smartptrshapes>& shapes)
{
	if (ashape)
	{
		shapes.push_back(ChSmartPtr<btCollisionShape>(ashape));

		if (ashape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE)
		{
			btCompoundShape* compoundShape = (btCompoundShape*) ashape;
			for (int shi = 0; shi < compoundShape->getNumChildShapes(); shi++)
			{
				__recurse_add_newcollshapes(compoundShape->getChildShape(shi), shapes);
			}
		}
	}
}



void ChModelBullet::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// parent class deserialize
	ChCollisionModel::StreamIN(mstream);

		// deserialize custom data:

	
	this->ClearModel(); // remove shape 

	int buffer_len=0;

	mstream >> buffer_len;

	char* mbuffer = new char[buffer_len];

	for (int mpt= 0; mpt <  buffer_len; mpt++)
		mstream >> mbuffer[mpt];

	btBulletWorldImporter import(0);//don't store info into the world
	import.setVerboseMode(false);

	if (import.loadFileFromMemory(mbuffer, buffer_len))
	{
		int numShape = import.getNumCollisionShapes();
		if (numShape)
		{
			btCollisionShape* mshape = import.getCollisionShapeByIndex(0);
			if (mshape)
				bt_collision_object->setCollisionShape(mshape);

			// Update the list of sharedpointers to newly created shapes, so that 
			// the deletion will be automatic
			__recurse_add_newcollshapes(mshape, this->shapes);
		}
	}

	delete[] mbuffer;
	
}

		
void ChModelBullet::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// parent class serialize
	ChCollisionModel::StreamOUT(mstream);

		// serialize custom data:
	
	int maxSerializeBufferSize = 1024*1024*5;	//***TO DO*** make this more efficient
	btDefaultSerializer*	serializer = new btDefaultSerializer(maxSerializeBufferSize);

	serializer->startSerialization();
	
	this->bt_collision_object->getCollisionShape()->serializeSingleShape(serializer);

	serializer->finishSerialization();

	mstream << serializer->getCurrentBufferSize();

	for (int mpt= 0; mpt <  serializer->getCurrentBufferSize(); mpt++)
		mstream << (char)(*(serializer->getBufferPointer()+mpt));

	delete serializer;

}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

