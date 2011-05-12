///////////////////////////////////////////////////
//
//   ChBodyDEM.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

    
#include <stdlib.h>
#include <algorithm>

#include "core/ChTrasform.h"
#include "physics/ChBodyDEM.h"
#include "physics/ChGlobal.h"
#include "physics/ChSystem.h"

#include "physics/ChExternalObject.h"
#include "collision/ChCModelBulletDEM.h"
#include "core/ChLinearAlgebra.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{

using namespace collision;
using namespace geometry;


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChBodyDEM> a_registration_ChBodyDEM;



//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR SOLID BODIES


ChBodyDEM::ChBodyDEM ()
{

	BFlagsSetAllOFF();		// no flags

	Xforce = VNULL;
	Xtorque = VNULL;

	collision_model=InstanceCollisionModel();

	last_coll_pos = CSYSNORM;

	SetIdentifier(ChGLOBALS().GetUniqueIntID()); // mark with unique ID

	max_speed = 0.5f;
	max_wvel  = 2.0f*float(CH_C_PI);

	kn=392400.0;
	gn=420.0;

	variables.SetUserData((void*)this);
}



ChBodyDEM::~ChBodyDEM ()
{
	if (collision_model) delete collision_model;
}

void ChBodyDEM::Copy(ChBodyDEM* source)
{
		// copy the parent class data...
	ChPhysicsItem::Copy(source);

		// copy the parent class data...
	ChFrameMoving<double>::operator=(*source);


	bflag		= source->bflag;

	variables = source->variables;
	variables.SetUserData((void*)this);

	gyro	= source->Get_gyro();

	ChTime = source->ChTime;

	collision_model->ClearModel();

	last_coll_pos = source->last_coll_pos;

	max_speed = source->max_speed;
	max_wvel  = source->max_wvel;

	kn = source->kn;
	gn = source->gn;
}


ChCollisionModel* ChBodyDEM::InstanceCollisionModel(){
	ChCollisionModel* collision_model_t= (ChModelBulletDEM*) new ChModelBulletDEM();
	((ChModelBulletDEM*)collision_model_t)->SetBody(this);
	return collision_model_t;
}


//// 
void ChBodyDEM::InjectVariables(ChLcpSystemDescriptor& mdescriptor)
{	
	this->variables.SetDisabled(!this->IsActive());

	mdescriptor.InsertVariables(&this->variables);
}


void ChBodyDEM::VariablesFbReset()
{
	this->variables.Get_fb().FillElem(0.0);
}

void ChBodyDEM::VariablesFbLoadForces(double factor)
{
	// add applied forces and torques (and also the gyroscopic torque!) to 'fb' vector
	this->variables.Get_fb().PasteSumVector( Xforce * factor ,0,0);
	this->variables.Get_fb().PasteSumVector((Xtorque - gyro)* factor ,3,0);
}


void ChBodyDEM::VariablesQbLoadSpeed()
{
	// set current speed in 'qb', it can be used by the LCP solver when working in incremental mode
	this->variables.Get_qb().PasteVector(GetCoord_dt().pos,0,0);
	this->variables.Get_qb().PasteVector(GetWvel_loc()    ,3,0);
}


void ChBodyDEM::VariablesQbSetSpeed(double step)
{
	ChCoordsys<> old_coord_dt = this->GetCoord_dt();

	// from 'qb' vector, sets body speed, and updates auxiliary data
	this->SetPos_dt(   this->variables.Get_qb().ClipVector(0,0) );
	this->SetWvel_loc( this->variables.Get_qb().ClipVector(3,0) );

	// apply limits (if in speed clamping mode) to speeds.
	ClampSpeed(); 

	// compute auxiliary gyroscopic forces
	ComputeGyro ();

	// Compute accel. by BDF (approximate by differentiation);
	if (step)
	{
		this->SetPos_dtdt( (this->GetCoord_dt().pos - old_coord_dt.pos)  / step);
		this->SetRot_dtdt( (this->GetCoord_dt().rot - old_coord_dt.rot)  / step);
	}
}

void ChBodyDEM::VariablesQbIncrementPosition(double dt_step)
{
	if (!this->IsActive()) 
		return;

	// Updates position with incremental action of speed contained in the
	// 'qb' vector:  pos' = pos + dt * speed   , like in an Eulero step.

	ChVector<> newspeed = variables.Get_qb().ClipVector(0,0);
	ChVector<> newwel   = variables.Get_qb().ClipVector(3,0);

	// ADVANCE POSITION: pos' = pos + dt * vel
	this->SetPos( this->GetPos() + newspeed * dt_step);

	// ADVANCE ROTATION: rot' = [dt*wwel]%rot  (use quaternion for delta rotation)
	ChQuaternion<> mdeltarot;
	ChQuaternion<> moldrot = this->GetRot();
	ChVector<> newwel_abs = Amatrix * newwel;
	double mangle = newwel_abs.Length() * dt_step;
	newwel_abs.Normalize();
	mdeltarot.Q_from_AngAxis(mangle, newwel_abs);
	ChQuaternion<> mnewrot = mdeltarot % moldrot;
	this->SetRot( mnewrot );
}



void ChBodyDEM::SetNoSpeedNoAcceleration()
{
	this->SetPos_dt(VNULL);
	this->SetWvel_loc(VNULL);
	this->SetPos_dtdt(VNULL);
	this->SetRot_dtdt(QNULL);
}


////
void ChBodyDEM::ClampSpeed()
{
	if (this->GetLimitSpeed())
	{
		double w = 2.0*this->coord_dt.rot.Length();
		if (w > max_wvel)
			coord_dt.rot *= max_wvel/w;

		double v = this->coord_dt.pos.Length();
		if (v > max_speed)
			coord_dt.pos *= max_speed/v;
	}
}


//// Utilities for coordinate transformations
///
Vector ChBodyDEM::Point_World2Body (Vector* mpoint)
{
	return ChFrame<double>::TrasformParentToLocal(*mpoint);
}

Vector ChBodyDEM::Point_Body2World (Vector* mpoint)
{
	return ChFrame<double>::TrasformLocalToParent(*mpoint);
}

Vector ChBodyDEM::Dir_World2Body (Vector* mpoint)
{
	return Amatrix.MatrT_x_Vect (*mpoint);
}

Vector ChBodyDEM::Dir_Body2World (Vector* mpoint)
{
	return Amatrix.Matr_x_Vect (*mpoint);
}

Vector ChBodyDEM::RelPoint_AbsSpeed(Vector* mrelpoint)
{
	return PointSpeedLocalToParent(*mrelpoint);
}

Vector ChBodyDEM::RelPoint_AbsAcc(Vector* mrelpoint)
{
	return PointAccelerationLocalToParent(*mrelpoint);
}

////
// The inertia tensor functions

void ChBodyDEM::SetInertia (ChMatrix33<>* newXInertia)
{
	variables.SetBodyInertia(newXInertia);
}

void ChBodyDEM::SetInertiaXX (Vector iner)
{
	variables.GetBodyInertia().SetElement(0,0,iner.x);
	variables.GetBodyInertia().SetElement(1,1,iner.y);
	variables.GetBodyInertia().SetElement(2,2,iner.z);
	variables.GetBodyInertia().FastInvert(&variables.GetBodyInvInertia());
}
void ChBodyDEM::SetInertiaXY (Vector iner)
{
	variables.GetBodyInertia().SetElement(0,1,iner.x);
	variables.GetBodyInertia().SetElement(0,2,iner.y);
	variables.GetBodyInertia().SetElement(1,2,iner.z);
	variables.GetBodyInertia().SetElement(1,0,iner.x);
	variables.GetBodyInertia().SetElement(2,0,iner.y);
	variables.GetBodyInertia().SetElement(2,1,iner.z);
	variables.GetBodyInertia().FastInvert(&variables.GetBodyInvInertia());
}

Vector ChBodyDEM::GetInertiaXX()
{
	ChVector<> iner;
	iner.x= variables.GetBodyInertia().GetElement(0,0);
	iner.y= variables.GetBodyInertia().GetElement(1,1);
	iner.z= variables.GetBodyInertia().GetElement(2,2);
	return iner;
}

Vector ChBodyDEM::GetInertiaXY()
{
	ChVector<> iner;
	iner.x= variables.GetBodyInertia().GetElement(0,1);
	iner.y= variables.GetBodyInertia().GetElement(0,2);
	iner.z= variables.GetBodyInertia().GetElement(1,2);
	return iner;
}

void ChBodyDEM::ComputeQInertia (ChMatrixNM<double,4,4>* mQInertia)
{
	ChMatrixNM<double,3,4> res ;
	ChMatrixNM<double,3,4> Gl  ;
	ChMatrixNM<double,4,3> GlT ;

	SetMatrix_Gl(Gl, coord.rot);
	GlT.CopyFromMatrixT(Gl);

	res.MatrMultiply (*this->GetXInertia(), Gl);
	mQInertia->MatrMultiply (GlT, res);	// [Iq]=[G'][Ix][G]
}

//////


void ChBodyDEM::To_abs_forcetorque  (Vector force, Vector appl_point, int local, Vector& resultforce, Vector& resulttorque)
{
	if (local)
	{
		// local space
		ChVector<> mforce_abs = Dir_Body2World(&force);
		resultforce = mforce_abs;
		resulttorque = Vcross (Dir_Body2World(&appl_point), mforce_abs) ;
	}
	else
	{
		// absolute space
		resultforce = force;
		resulttorque = Vcross (Vsub(appl_point, coord.pos), force) ;
	}
}
void ChBodyDEM::To_abs_torque (Vector torque, int local, Vector& resulttorque)
{
	if (local)
	{
		// local space
		resulttorque = Dir_Body2World(&torque);
	}
	else
	{
		// absolute space
		resulttorque = torque;
	}
}


void ChBodyDEM::Add_as_lagrangian_force(Vector force, Vector appl_point, int local, ChMatrixNM<double,7,1>* mQf)
{
	ChVector<> mabsforce;
	ChVector<> mabstorque;
	To_abs_forcetorque (force, appl_point, local, mabsforce, mabstorque);
	mQf->PasteSumVector(mabsforce,0,0);
	mQf->PasteSumQuaternion( ChFrame<>::GlT_x_Vect(coord.rot, Dir_World2Body(&mabstorque)) , 3,0);
}

void ChBodyDEM::Add_as_lagrangian_torque(Vector torque, int local, ChMatrixNM<double,7,1>* mQf)
{
	ChVector<> mabstorque;
	To_abs_torque (torque, local, mabstorque);
	mQf->PasteSumQuaternion( ChFrame<>::GlT_x_Vect(coord.rot, Dir_World2Body(&mabstorque)), 3,0);
}

void ChBodyDEM::From_lagrangian_to_forcetorque(ChMatrixNM<double,7,1>* mQf, Vector* mforce, Vector* mtorque)
{
	*mforce = mQf->ClipVector(0,0);
	ChQuaternion<> tempq;
	tempq = mQf->ClipQuaternion(3,0);
	*mtorque = ChFrame<>::Gl_x_Quat(coord.rot, tempq);
	*mtorque = Vmul (*mtorque, 0.25);
}

void ChBodyDEM::From_forcetorque_to_lagrangian(Vector* mforce, Vector* mtorque, ChMatrixNM<double,7,1>* mQf)
{
	mQf->PasteVector(*mforce, 0,0);
	ChQuaternion<> tempq;
	tempq = ChFrame<>::GlT_x_Vect(coord.rot, *mtorque);
	mQf->PasteQuaternion(tempq,3,0);
}

void ChBodyDEM::ComputeGyro ()
{
	ChVector<> Wvel = this->GetWvel_loc();
	gyro = Vcross ( Wvel, (variables.GetBodyInertia().Matr_x_Vect (Wvel)));	
}

void ChBodyDEM::UpdateForces (double mytime)
{
	// COMPUTE LAGRANGIAN FORCES APPLIED TO BODY

	Xforce = VNULL;
	Xtorque = VNULL;

	if (GetSystem())
	{
		Xforce += GetSystem()->Get_G_acc() * this->GetMass();
	}

}

void ChBodyDEM::UpdateTime (double mytime)
{
	ChTime = mytime;
}


void ChBodyDEM::UpdateState (Coordsys mypos, Coordsys mypos_dt)
{
	SetCoord (mypos);		// Set the state coordsys,
	SetCoord_dt (mypos_dt);	// automatically updating auxiliary variables

	//TrySleeping();			// See if the body can fall asleep; if so, put it to sleeping
	ClampSpeed();			// Apply limits (if in speed clamping mode) to speeds.
	ComputeGyro ();			// Set the gyroscopic momentum.
}


void ChBodyDEM::UpdateStateTime (Coordsys mypos, Coordsys mypos_dt, double mytime)
{
	UpdateTime (mytime);

	UpdateState (mypos, mypos_dt);
}


void ChBodyDEM::Update (Coordsys mypos, Coordsys mypos_dt, double mytime)
{
	UpdateTime (mytime);

	 mypos.rot.Normalize();
	UpdateState (mypos, mypos_dt);

	ChBodyDEM::Update();

}



							// UpdateALL updates the state and time
							// of the object AND the dependant (linked)
							// markers and forces.

void ChBodyDEM::Update()
{
	//TrySleeping();			// See if the body can fall asleep; if so, put it to sleeping 
	ClampSpeed();			// Apply limits (if in speed clamping mode) to speeds.
	ComputeGyro ();			// Set the gyroscopic momentum.

		// Also update the
		// "forces" depending on the body current state.

	UpdateForces(ChTime);
}



							// As before, but keeps the current state.
							// Mostly used for world reference body.
void ChBodyDEM::Update (double mytime)
{
				// For the body:
	UpdateTime (mytime);

	ChBodyDEM::Update();

}



void ChBodyDEM::UpdateExternalGeometry ()
{
	if (this->GetExternalObject())
		this->GetExternalObject()->onChronoChanged();
}

void ChBodyDEM::SetBodyFixed (bool mev)
{
	variables.SetDisabled(mev);
	if (mev == BFlagGet(BF_FIXED)) 
			return;
	BFlagSet(BF_FIXED, mev);
	//RecomputeCollisionModel(); // because one may use different model types for static or dynamic coll.shapes
}
 
// collision stuff
void ChBodyDEM::SetCollide (bool mcoll)
{
	if (mcoll == BFlagGet(BF_COLLIDE)) 
		return;

	if (mcoll)
	{
		SyncCollisionModels();
		BFlagSetON(BF_COLLIDE);
		if (GetSystem())
			GetSystem()->GetCollisionSystem()->Add(this->GetCollisionModel());
	}
	else 
	{
		BFlagSetOFF(BF_COLLIDE);
		if (GetSystem())
			GetSystem()->GetCollisionSystem()->Remove(this->GetCollisionModel());
	}
}

// forward reference
int coll_model_from_r3d(ChCollisionModel* chmodel, ChBody* mbody, int lod, Vector* mt, ChMatrix33<>* mr);

int ChBodyDEM::RecomputeCollisionModel()
{
	if (!GetCollide()) return FALSE; // do nothing unless collision enabled

	collision_model->ClearModel(); // ++++ start geometry definition
  
	if (this->GetExternalObject())
		this->GetExternalObject()->onAddCollisionGeometries(
					this->collision_model,
					(ChBody*)this,
					1,
					&coord.pos,
					&Amatrix);

	collision_model->BuildModel(); // ++++ complete geometry definition


	return TRUE;
}

void ChBodyDEM::SyncCollisionModels()
{
	this->GetCollisionModel()->SyncPosition();
}

void ChBodyDEM::AddCollisionModelsToSystem() 
{
	assert(this->GetSystem());
	SyncCollisionModels();
	this->GetSystem()->GetCollisionSystem()->Add(this->GetCollisionModel());
}

void ChBodyDEM::RemoveCollisionModelsFromSystem() 
{
	assert(this->GetSystem());
	this->GetSystem()->GetCollisionSystem()->Remove(this->GetCollisionModel());
}



void ChBodyDEM::GetTotalAABB(ChVector<>& bbmin, ChVector<>& bbmax)
{
	if (this->GetCollisionModel())
		this->GetCollisionModel()->GetAABB(bbmin, bbmax);
	else 
		ChPhysicsItem::GetTotalAABB(bbmin, bbmax); // default: infinite aabb
}

//////// FILE I/O

void ChBodyDEM::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChPhysicsItem::StreamOUT(mstream);

		// serialize parent class too
	ChFrameMoving<double>::StreamOUT(mstream);

		// stream out all member data
		ChVector<> vfoo=VNULL;

	mstream << kn;
	mstream << gn;

	mstream << variables.GetBodyMass();
	vfoo = GetInertiaXX();	mstream << vfoo;
	vfoo = GetInertiaXY();  mstream << vfoo;
	
	mstream << bflag;
	
	mstream << max_speed;
	mstream << max_wvel;

	this->collision_model->StreamOUT(mstream); // also 	mstream << (*this->collision_model);
}

void ChBodyDEM::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChPhysicsItem::StreamIN(mstream);

		// deserialize parent class too
	ChFrameMoving<double>::StreamIN(mstream);

	collision_model->ClearModel();

	double dfoo;
	float ffoo;
	ChVector<> vfoo;

	mstream >> ffoo;		SetSpringCoefficient(ffoo);
	mstream >> ffoo;		SetDampingCoefficient(ffoo);
	mstream >> dfoo;		SetMass(dfoo);
	mstream >> vfoo;		SetInertiaXX(vfoo);
	mstream >> vfoo;		SetInertiaXY(vfoo);
	mstream >> bflag;
	mstream >> max_speed;
	mstream >> max_wvel;
	if(this->GetBodyFixed())
		SetBodyFixed(true);
	else SetBodyFixed(false);

	this->collision_model->StreamIN(mstream); // also 	mstream >> (*collision_model);
	this->collision_model->BuildModel(); // because previously removed from ChSystem, if any.
}




void ChBodyDEM::StreamOUTstate(ChStreamOutBinary& mstream)
{
	// Do not serialize parent classes and do not
	// implement versioning, because this must be efficient 
	// and will be used just for domain decomposition.
	this->GetCoord().StreamOUT(mstream);
	this->GetCoord_dt().StreamOUT(mstream);
}

void ChBodyDEM::StreamINstate(ChStreamInBinary& mstream)
{
	// Do not serialize parent classes and do not
	// implement versioning, because this must be efficient 
	// and will be used just for domain decomposition.
	ChCoordsys<> mcoord;
	mstream >> mcoord;
	this->SetCoord(mcoord);
	mstream >> mcoord;
	this->SetCoord_dt(mcoord);

	this->Update();
	this->SyncCollisionModels();
}


#define CH_ChUNK_END 1234

int ChBodyDEM::StreamINall  (ChStreamInBinary& m_file)
{
	int mchunk = 0;

	// 1) read body class data...
 
	m_file >> *this; 

	m_file >> mchunk; 

	if (mchunk != CH_ChUNK_END) return 0;


	return 1;
}


int ChBodyDEM::StreamOUTall  (ChStreamOutBinary& m_file)
{

	// 1) read body class data...
	m_file << *this;

	m_file << (int)CH_ChUNK_END;

	return 1;
}

void ChBodyDEM::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "BODY   " << GetName() <<"\n";

	ChFrameMoving<double>::StreamOUT(mstream);

	//***TO DO***
}


int ChBodyDEM::StreamOUTall  (ChStreamOutAscii& mstream) // dump rigidbody and childrens (markers.forces)
{
	StreamOUT (mstream);			 // 1) dump the body attrs

	return 1;
}


} // END_OF_NAMESPACE____


/////////////////////
