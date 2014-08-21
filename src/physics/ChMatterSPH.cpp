//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChMatterSPH.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

    
#include <stdlib.h>
#include <algorithm>

#include "physics/ChMatterSPH.h"
#include "physics/ChSystem.h"

#include "physics/ChExternalObject.h"
#include "physics/ChProximityContainerSPH.h"
#include "collision/ChCModelBulletNode.h"
#include "core/ChLinearAlgebra.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{

using namespace collision;
using namespace geometry;


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChMatterSPH> a_registration_ChMatterSPH;

	

//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR A SPH NODE


ChNodeSPH::ChNodeSPH()
{
	this->collision_model = new ChModelBulletNode;
	
	this->UserForce = VNULL;
	this->h_rad = 0.1;
	this->coll_rad = 0.001;
	this->SetMass(0.01);
	this->volume = 0.01;
	this->density = this->GetMass()/this->volume;
	this->pressure = 0;
}

ChNodeSPH::~ChNodeSPH()
{
	delete collision_model; 
}

ChNodeSPH::ChNodeSPH (const ChNodeSPH& other) :
					ChNodeXYZ(other) 
{
	this->collision_model = new ChModelBulletNode;
	this->collision_model->AddSphere(other.coll_rad); 
	((ChModelBulletNode*)collision_model)->SetNode(
		((ChModelBulletNode*)other.collision_model)->GetNodes(),
		((ChModelBulletNode*)other.collision_model)->GetNodeId());

	this->UserForce = other.UserForce;
	this->SetKernelRadius(other.h_rad);
	this->SetCollisionRadius(other.coll_rad);
	this->SetMass(other.GetMass());
	this->volume = other.volume;
	this->density = other.density;
	this->pressure = other.pressure;

	this->variables = other.variables;
}

ChNodeSPH& ChNodeSPH::operator= (const ChNodeSPH& other)
{
	if (&other == this) 
		return *this;

	ChNodeXYZ::operator=(other);

	this->collision_model->ClearModel();
	this->collision_model->AddSphere(other.coll_rad ); 
	((ChModelBulletNode*)collision_model)->SetNode(
		((ChModelBulletNode*)other.collision_model)->GetNodes(),
		((ChModelBulletNode*)other.collision_model)->GetNodeId());
	
	this->UserForce = other.UserForce;
	this->SetKernelRadius(other.h_rad);
	this->SetCollisionRadius(other.coll_rad);
	this->SetMass(other.GetMass());
	this->volume = other.volume;
	this->density = other.density;

	this->variables = other.variables;
	
	return *this;
}

void ChNodeSPH::SetKernelRadius(double mr)
{
	h_rad = mr;
	double aabb_rad = h_rad/2; // to avoid too many pairs: bounding boxes hemisizes will sum..  __.__--*--
	((ChModelBulletNode*)this->collision_model)->SetSphereRadius(coll_rad, ChMax(0.0, aabb_rad-coll_rad) );
}
	
void ChNodeSPH::SetCollisionRadius(double mr)
{
	coll_rad = mr;
	double aabb_rad = h_rad/2; // to avoid too many pairs: bounding boxes hemisizes will sum..  __.__--*--
	((ChModelBulletNode*)this->collision_model)->SetSphereRadius(coll_rad, ChMax(0.0, aabb_rad-coll_rad) );
}



//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR SPH MATERIAL

void ChContinuumSPH::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// stream out parent class
	ChContinuumMaterial::StreamOUT(mstream);

		// stream out all member data
	mstream << this->viscosity;
	mstream << this->surface_tension;
	mstream << this->pressure_stiffness;
}

void ChContinuumSPH::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// stream in parent class
	ChContinuumMaterial::StreamIN(mstream);

		// stream in all member data
	mstream >> this->viscosity;
	mstream >> this->surface_tension;
	mstream >> this->pressure_stiffness;
}




//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR SPH NODE CLUSTER


ChMatterSPH::ChMatterSPH ()
{
	this->do_collide = false;

	this->nodes.clear();

	SetIdentifier(GetUniqueIntID()); // mark with unique ID

}


ChMatterSPH::~ChMatterSPH ()
{
	// delete nodes
	this->ResizeNnodes(0);
}



void ChMatterSPH::Copy(ChMatterSPH* source)
{
		// copy the parent class data...
	ChIndexedNodes::Copy(source);

	do_collide = source->do_collide;

	this->material = source->material;
	
	ResizeNnodes(source->GetNnodes());
}




void ChMatterSPH::ResizeNnodes(int newsize)
{
	bool oldcoll = this->GetCollide();
	this->SetCollide(false); // this will remove old particle coll.models from coll.engine, if previously added

	/*
	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		//delete (this->nodes[j]);  ***not needed since using shared ptrs
		this->nodes[j] = 0;
	}
	*/  
	this->nodes.resize(newsize);

	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		this->nodes[j] = ChSharedPtr<ChNodeSPH>(new ChNodeSPH);

		this->nodes[j]->variables.SetUserData((void*)this); // UserData unuseful in future cuda solver?
		((ChModelBulletNode*)this->nodes[j]->collision_model)->SetNode(this,j);
		this->nodes[j]->collision_model->AddSphere(0.001); //***TEST***
		this->nodes[j]->collision_model->BuildModel();
	}

	this->SetCollide(oldcoll); // this will also add particle coll.models to coll.engine, if already in a ChSystem

}


void ChMatterSPH::AddNode(ChVector<double> initial_state)
{
	ChSharedPtr<ChNodeSPH> newp (new ChNodeSPH);

	newp->SetPos(initial_state);

	this->nodes.push_back(newp);

	newp->variables.SetUserData((void*)this);  // UserData unuseful in future cuda solver?
	((ChModelBulletNode*)newp->collision_model)->SetNode(this,nodes.size()-1);
	newp->collision_model->AddSphere(0.1); //***TEST***
	newp->collision_model->BuildModel(); // will also add to system, if collision is on.
}



void ChMatterSPH::FillBox (const ChVector<> size,	
				  const double spacing,		
				  const double initial_density, 
				  const ChCoordsys<> boxcoords, 
				  const bool do_centeredcube,	
				  const double kernel_sfactor,  
				  const double randomness
				  )
{
	int samples_x = (int)(size.x/spacing);
	int samples_y = (int)(size.y/spacing);
	int samples_z = (int)(size.z/spacing);
	int totsamples = 0;

	double mrandomness= randomness;
	if (do_centeredcube)
		mrandomness = randomness*0.5;

	for (int ix = 0; ix < samples_x; ix++)
		for (int iy = 0; iy < samples_y; iy++)  
			for (int iz = 0; iz < samples_z; iz++) 
			{
				ChVector<> pos (	ix*spacing  -0.5*size.x, 
									iy*spacing  -0.5*size.y,	
									iz*spacing  -0.5*size.z);
				pos += ChVector<>(mrandomness*ChRandom()*spacing, mrandomness*ChRandom()*spacing, mrandomness*ChRandom()*spacing);
				this->AddNode(boxcoords.TransformLocalToParent(pos));
				totsamples++;

				if (do_centeredcube)
				{
					ChVector<> pos2 = pos + 0.5*ChVector<>(spacing,spacing,spacing);
					pos2 += ChVector<>(mrandomness*ChRandom()*spacing, mrandomness*ChRandom()*spacing, mrandomness*ChRandom()*spacing);
					this->AddNode(boxcoords.TransformLocalToParent(pos2));
					totsamples++;
				}
			}

	double mtotvol  = size.x * size.y * size.z;
	double mtotmass = mtotvol * initial_density;
	double nodemass = mtotmass/(double)totsamples;
	double kernelrad = kernel_sfactor*spacing;

	for (unsigned int ip = 0; ip < this->GetNnodes(); ip++)
	{
		// downcasting
		ChSharedPtr<ChNodeSPH> mnode ( this->nodes[ip] );
		assert(!mnode.IsNull());

		mnode->SetKernelRadius(kernelrad);
		mnode->SetCollisionRadius(spacing*0.1); 
		mnode->SetMass(nodemass);
	}

	this->GetMaterial().Set_density(initial_density);
}



//// 
void ChMatterSPH::InjectVariables(ChLcpSystemDescriptor& mdescriptor)
{	
	//this->variables.SetDisabled(!this->IsActive());
	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		mdescriptor.InsertVariables(&(this->nodes[j]->variables));
	}
}


void ChMatterSPH::VariablesFbReset()
{
	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		this->nodes[j]->variables.Get_fb().FillElem(0.0);
	}
}

void ChMatterSPH::VariablesFbLoadForces(double factor)
{

	// COMPUTE THE SPH FORCES HERE

	// First, find if any ChProximityContainerSPH object is present
	// in the system,

	ChProximityContainerSPH* edges =0;
	std::list<ChPhysicsItem*>::iterator iterotherphysics = this->GetSystem()->Get_otherphysicslist()->begin();
	while (iterotherphysics != this->GetSystem()->Get_otherphysicslist()->end())
	{
		if (edges=dynamic_cast<ChProximityContainerSPH*>(*iterotherphysics))
			break;
		iterotherphysics++;
	}
	assert(edges); // If using a ChMatterSPH, you must add also a ChProximityContainerSPH.
	

	// 1- Per-node initialization

	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		this->nodes[j]->UserForce = VNULL;
		this->nodes[j]->density = 0;
	}

	// 2- Per-edge initialization and accumulation of particles's density

	edges->AccumulateStep1();

	// 3- Per-node volume and pressure computation

	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		ChSharedPtr<ChNodeSPH> mnode (this->nodes[j]);
		assert(!mnode.IsNull());

		// node volume is v=mass/density
		if (mnode->density)
			mnode->volume = mnode->GetMass()/mnode->density;
		else 
			mnode->volume = 0; 

		// node pressure = k(dens - dens_0);
		mnode->pressure = this->material.Get_pressure_stiffness() * ( mnode->density - this->material.Get_density() );
	}

	// 4- Per-edge forces computation and accumulation

	edges->AccumulateStep2();


	// 5- Per-node load forces in LCP

	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		// particle gyroscopic force:
		// none.

		// add gravity 
		ChVector<> Gforce = GetSystem()->Get_G_acc() * this->nodes[j]->GetMass();
		ChVector<> TotForce = this->nodes[j]->UserForce + Gforce; 

		// downcast
		ChSharedPtr<ChNodeSPH> mnode ( this->nodes[j]);
		assert (!mnode.IsNull());

		mnode->variables.Get_fb().PasteSumVector(TotForce * factor ,0,0);
	}

}

void ChMatterSPH::VariablesQbLoadSpeed()
{
	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		// set current speed in 'qb', it can be used by the LCP solver when working in incremental mode
		this->nodes[j]->variables.Get_qb().PasteVector(this->nodes[j]->GetPos_dt(),0,0);
	}
}

void ChMatterSPH::VariablesFbIncrementMq()
{
	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		this->nodes[j]->variables.Compute_inc_Mb_v(this->nodes[j]->variables.Get_fb(), this->nodes[j]->variables.Get_qb());
	}
}

void ChMatterSPH::VariablesQbSetSpeed(double step)
{
	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		ChVector<> old_pos_dt = this->nodes[j]->GetPos_dt();

		// from 'qb' vector, sets body speed, and updates auxiliary data
		this->nodes[j]->SetPos_dt(   this->nodes[j]->variables.Get_qb().ClipVector(0,0) );

		// Compute accel. by BDF (approximate by differentiation);
		if (step)
		{
			this->nodes[j]->SetPos_dtdt( (this->nodes[j]->GetPos_dt() - old_pos_dt)  / step);
		}
	}
}

void ChMatterSPH::VariablesQbIncrementPosition(double dt_step)
{
	//if (!this->IsActive()) 
	//	return;

	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		// Updates position with incremental action of speed contained in the
		// 'qb' vector:  pos' = pos + dt * speed   , like in an Eulero step.

		ChVector<> newspeed = this->nodes[j]->variables.Get_qb().ClipVector(0,0);

		// ADVANCE POSITION: pos' = pos + dt * vel
		this->nodes[j]->SetPos( this->nodes[j]->GetPos() + newspeed * dt_step);
	}


}



//////////////


void ChMatterSPH::SetNoSpeedNoAcceleration()
{
	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		this->nodes[j]->SetPos_dt(VNULL);
		this->nodes[j]->SetPos_dtdt(VNULL);
	}
}





//////






void ChMatterSPH::Update()
{
	ChMatterSPH::Update(this->GetChTime());
}

						
void ChMatterSPH::Update (double mytime)
{	
		// Inherit time changes of parent class
	ChPhysicsItem::Update(mytime);

	//TrySleeping();		// See if the body can fall asleep; if so, put it to sleeping 
	//ClampSpeed();			// Apply limits (if in speed clamping mode) to speeds.
	
}



void ChMatterSPH::UpdateExternalGeometry ()
{
	if (this->GetExternalObject())
		this->GetExternalObject()->onChronoChanged();
}

 
// collision stuff
void ChMatterSPH::SetCollide (bool mcoll)
{

	if (mcoll == this->do_collide) 
		return;

	if (mcoll)
	{
		this->do_collide = true;
		if (GetSystem())
		{
			for (unsigned int j = 0; j < nodes.size(); j++)
			{
				GetSystem()->GetCollisionSystem()->Add(this->nodes[j]->collision_model);
			}
		}
	}
	else 
	{
		this->do_collide = false;
		if (GetSystem())
		{
			for (unsigned int j = 0; j < nodes.size(); j++)
			{
				GetSystem()->GetCollisionSystem()->Remove(this->nodes[j]->collision_model);
			}
		}
	}
}

void ChMatterSPH::SyncCollisionModels()
{
	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		this->nodes[j]->collision_model->SyncPosition();
	}
}

void ChMatterSPH::AddCollisionModelsToSystem() 
{
	assert(this->GetSystem());
	SyncCollisionModels();
	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		this->GetSystem()->GetCollisionSystem()->Add(this->nodes[j]->collision_model);
	}
}

void ChMatterSPH::RemoveCollisionModelsFromSystem() 
{
	assert(this->GetSystem());
	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		this->GetSystem()->GetCollisionSystem()->Remove(this->nodes[j]->collision_model);
	}
}


////

void ChMatterSPH::UpdateParticleCollisionModels()
{
	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		this->nodes[j]->collision_model->ClearModel();
		//***TO DO*** UPDATE RADIUS OF SPHERE? this->nodes[j]->collision_model->AddCopyOfAnotherModel(this->particle_collision_model);
		this->nodes[j]->collision_model->BuildModel();
	}
}




//////// FILE I/O

void ChMatterSPH::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChIndexedNodes::StreamOUT(mstream);

		// stream out all member data
	mstream << this->material;

	//***TO DO*** stream nodes

}

void ChMatterSPH::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChIndexedNodes::StreamIN(mstream);


		// stream in all member data

	mstream >> this->material;

	//***TO DO*** unstream nodes
}






} // END_OF_NAMESPACE____


/////////////////////
