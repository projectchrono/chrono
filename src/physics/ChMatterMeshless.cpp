///////////////////////////////////////////////////
//
//   ChMatterMeshless.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

    
#include <stdlib.h>
#include <algorithm>

#include "physics/ChMatterMeshless.h"
#include "physics/ChSystem.h"

#include "physics/ChExternalObject.h"
#include "physics/ChProximityContainerMeshless.h"
#include "collision/ChCModelBulletNode.h"
#include "core/ChLinearAlgebra.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{

using namespace collision;
using namespace geometry;


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChMatterMeshless> a_registration_ChMatterMeshless;

	

//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR A MESHLESS NODE


ChNodeMeshless::ChNodeMeshless()
{
	this->collision_model = new ChModelBulletNode;
	
	this->pos_ref = VNULL;
	this->UserForce = VNULL;
	this->h_rad = 0.1;
	this->coll_rad = 0.001;
	this->SetMass(0.01);
	this->volume = 0.01;
	this->density = this->GetMass()/this->volume;
	this->hardening = 0;
}

ChNodeMeshless::~ChNodeMeshless()
{
	delete collision_model; 
}

ChNodeMeshless::ChNodeMeshless (const ChNodeMeshless& other) :
					ChNodeBase(other) 
{
	this->collision_model = new ChModelBulletNode;
	this->collision_model->AddSphere(other.coll_rad); 
	((ChModelBulletNode*)collision_model)->SetNode(
		((ChModelBulletNode*)other.collision_model)->GetNodes(),
		((ChModelBulletNode*)other.collision_model)->GetNodeId());

	this->pos_ref = other.pos_ref;
	this->UserForce = other.UserForce;
	this->SetKernelRadius(other.h_rad);
	this->SetCollisionRadius(other.coll_rad);
	this->SetMass(other.GetMass());
	this->volume = other.volume;
	this->density = other.density;
	this->hardening = other.hardening;
	
	this->t_strain = other.t_strain;
	this->p_strain = other.p_strain;
	this->e_strain = other.e_strain;
	this->e_stress = other.e_stress;

	this->variables = other.variables;
}

ChNodeMeshless& ChNodeMeshless::operator= (const ChNodeMeshless& other)
{
	if (&other == this) 
		return *this;

	ChNodeBase::operator=(other);

	this->collision_model->ClearModel();
	this->collision_model->AddSphere(other.coll_rad ); 
	((ChModelBulletNode*)collision_model)->SetNode(
		((ChModelBulletNode*)other.collision_model)->GetNodes(),
		((ChModelBulletNode*)other.collision_model)->GetNodeId());
	
	this->pos_ref = other.pos_ref;
	this->UserForce = other.UserForce;
	this->SetKernelRadius(other.h_rad);
	this->SetCollisionRadius(other.coll_rad);
	this->SetMass(other.GetMass());
	this->volume = other.volume;
	this->density = other.density;
	this->hardening = other.hardening; 

	this->t_strain = other.t_strain;
	this->p_strain = other.p_strain;
	this->e_strain = other.e_strain;
	this->e_stress = other.e_stress;

	this->variables = other.variables;
	
	return *this;
}

void ChNodeMeshless::SetKernelRadius(double mr)
{
	h_rad = mr;
	double aabb_rad = h_rad/2; // to avoid too many pairs: bounding boxes hemisizes will sum..  __.__--*--
	((ChModelBulletNode*)this->collision_model)->SetSphereRadius(coll_rad, ChMax(0.0, aabb_rad-coll_rad) );
}
	
void ChNodeMeshless::SetCollisionRadius(double mr)
{
	coll_rad = mr;
	double aabb_rad = h_rad/2; // to avoid too many pairs: bounding boxes hemisizes will sum..  __.__--*--
	((ChModelBulletNode*)this->collision_model)->SetSphereRadius(coll_rad, ChMax(0.0, aabb_rad-coll_rad) );
}






//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR Meshless NODE CLUSTER


ChMatterMeshless::ChMatterMeshless ()
{
	this->do_collide = false;
	
	this->material.Set_E(1000000);
	this->material.Set_v(0.2);
	this->material.Set_density(1000);
	
	this->viscosity = 0.0;

	this->nodes.clear();

	SetIdentifier(GLOBAL_Vars->GetUniqueIntID()); // mark with unique ID

}


ChMatterMeshless::~ChMatterMeshless ()
{
	// delete nodes
	this->ResizeNnodes(0);
}



void ChMatterMeshless::Copy(ChMatterMeshless* source)
{
		// copy the parent class data...
	ChIndexedNodes::Copy(source);

	do_collide = source->do_collide;
	
	ResizeNnodes(source->GetNnodes());
}




void ChMatterMeshless::ResizeNnodes(int newsize)
{
	bool oldcoll = this->GetCollide();
	this->SetCollide(false); // this will remove old particle coll.models from coll.engine, if previously added

	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		delete (this->nodes[j]);
		this->nodes[j] = 0;
	}

	this->nodes.resize(newsize);

	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		this->nodes[j] = new ChNodeMeshless;

		this->nodes[j]->variables.SetUserData((void*)this); // UserData unuseful in future cuda solver?
		((ChModelBulletNode*)this->nodes[j]->collision_model)->SetNode(this,j);
		this->nodes[j]->collision_model->AddSphere(0.001); //***TEST***
		this->nodes[j]->collision_model->BuildModel();
	}

	this->SetCollide(oldcoll); // this will also add particle coll.models to coll.engine, if already in a ChSystem

}


void ChMatterMeshless::AddNode(ChVector<double> initial_state)
{
	ChNodeMeshless* newp = new ChNodeMeshless;

	newp->SetPos(initial_state);
	newp->SetPosReference(initial_state);

	this->nodes.push_back(newp);

	newp->variables.SetUserData((void*)this);  // UserData unuseful in future cuda solver?
	((ChModelBulletNode*)newp->collision_model)->SetNode(this,nodes.size()-1);
	newp->collision_model->AddSphere(0.1); //***TEST***
	newp->collision_model->BuildModel(); // will also add to system, if collision is on.
}




void ChMatterMeshless::FillBox (const ChVector<> size,	
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
				this->AddNode(boxcoords.TrasformLocalToParent(pos));
				totsamples++;

				if (do_centeredcube)
				{
					ChVector<> pos2 = pos + 0.5*ChVector<>(spacing,spacing,spacing);
					pos2 += ChVector<>(mrandomness*ChRandom()*spacing, mrandomness*ChRandom()*spacing, mrandomness*ChRandom()*spacing);
					this->AddNode(boxcoords.TrasformLocalToParent(pos2));
					totsamples++;
				}
			}

	double mtotvol  = size.x * size.y * size.z;
	double mtotmass = mtotvol * initial_density;
	double nodemass = mtotmass/(double)totsamples;
	double kernelrad = kernel_sfactor*spacing;

	for (int ip = 0; ip < this->GetNnodes(); ip++)
	{
		ChNodeMeshless* mnode = (ChNodeMeshless*)&(this->GetNode(ip));
		mnode->SetKernelRadius(kernelrad);
		mnode->SetCollisionRadius(spacing*0.1); 
		mnode->SetMass(nodemass);
	}

	this->GetMaterial().Set_density(initial_density);
}




//// 
void ChMatterMeshless::InjectVariables(ChLcpSystemDescriptor& mdescriptor)
{	
	//this->variables.SetDisabled(!this->IsActive());
	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		mdescriptor.InsertVariables(&(this->nodes[j]->variables));
	}
}


void ChMatterMeshless::VariablesFbReset()
{
	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		this->nodes[j]->variables.Get_fb().FillElem(0.0);
	}
}

void ChMatterMeshless::VariablesFbLoadForces(double factor)
{

	// COMPUTE THE MESHLESS FORCES HERE

	// First, find if any ChProximityContainerMeshless object is present
	// in the system,

	ChProximityContainerMeshless* edges =0;
	std::list<ChPhysicsItem*>::iterator iterotherphysics = this->GetSystem()->Get_otherphysicslist()->begin();
	while (iterotherphysics != this->GetSystem()->Get_otherphysicslist()->end())
	{
		if (edges=dynamic_cast<ChProximityContainerMeshless*>(*iterotherphysics))
			break;
		iterotherphysics++;
	}
	assert(edges); // If using a ChMatterMeshless, you must add also a ChProximityContainerMeshless.
	

	// 1- Per-node initialization

	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		this->nodes[j]->J.FillElem(0.0);
		this->nodes[j]->Amoment.FillElem(0.0);
		this->nodes[j]->t_strain.FillElem(0.0);
		this->nodes[j]->e_stress.FillElem(0.0); 
		this->nodes[j]->UserForce = VNULL;
		this->nodes[j]->density = 0;
	}

	// 2- Per-edge initialization and accumulation of values in particles's J, Amoment, m_v, density

	edges->AccumulateStep1();

	// 3- Per-node inversion of A and computation of strain stress

	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		ChNodeMeshless* mnode = this->nodes[j];

		// node volume is v=mass/density
		if (mnode->density>0)
			mnode->volume = mnode->GetMass()/mnode->density;
		else 
			mnode->volume = 0;

		// Compute A inverse
		ChMatrix33<> M_tmp = mnode->Amoment;
		double det = M_tmp.FastInvert(&mnode->Amoment);
		if (fabs(det)<0.00003) 
		{
			mnode->Amoment.FillElem(0); // deactivate if not possible to invert
			mnode->e_strain.FillElem(0.0); // detach
		}
		else
		{
			// Compute J = ( A^-1 * [dwg | dwg | dwg] )' + I
			M_tmp.MatrMultiply(mnode->Amoment, mnode->J);
			M_tmp.Element(0,0) +=1; 
			M_tmp.Element(1,1) +=1;
			M_tmp.Element(2,2) +=1;	
			mnode->J.CopyFromMatrixT(M_tmp);

			// Compute step strain tensor  epsilon = J'*J - I
			ChMatrix33<> mtensor;
			mtensor.MatrMultiply(M_tmp, mnode->J);
			mtensor.Element(0,0)-=1;
			mtensor.Element(1,1)-=1;
			mtensor.Element(2,2)-=1;

			// Compute elastic stress tensor  sigma= C*epsilon
			ChStrainTensor<> step_e_strain;
			step_e_strain.ConvertFromMatrix(mtensor); 
			mnode->t_strain.MatrAdd(mnode->e_strain, step_e_strain);
			this->GetMaterial().ComputeElasticStress(mnode->e_stress, mnode->t_strain);
			mnode->e_stress.ConvertToMatrix(mtensor);

			// Precompute 2*v*J*sigma*A^-1
			mnode->FA = mnode->J * (mtensor * (mnode->Amoment));
			mnode->FA.MatrScale(2*mnode->volume);
		}
	}

	// 4- Per-edge force transfer from stress, and add also viscous forces

	edges->AccumulateStep2();


	// 5- Per-node load force

	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		// particle gyroscopic force:
		// none.

		// add gravity 
		ChVector<> Gforce = GetSystem()->Get_G_acc() * this->nodes[j]->GetMass();
		ChVector<> TotForce = this->nodes[j]->UserForce + Gforce; 

		ChNodeMeshless* mnode = this->nodes[j];

		mnode->variables.Get_fb().PasteSumVector(TotForce * factor ,0,0);
		
	}

}


void ChMatterMeshless::VariablesQbLoadSpeed()
{
	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		// set current speed in 'qb', it can be used by the LCP solver when working in incremental mode
		this->nodes[j]->variables.Get_qb().PasteVector(this->nodes[j]->GetPos_dt(),0,0);
	}
}


void ChMatterMeshless::VariablesQbSetSpeed(double step)
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

void ChMatterMeshless::VariablesQbIncrementPosition(double dt_step)
{
	//if (!this->IsActive()) 
	//	return;
 
	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		ChNodeMeshless* mnode = this->nodes[j];

		// Increment total elastic tensor, instead of absorbing it in plastic tensor as in Muller paper.
		this->nodes[j]->e_strain = this->nodes[j]->t_strain; 
		this->nodes[j]->pos_ref = this->nodes[j]->pos;

		// Integrate plastic flow 
		ChStrainTensor<> plasticflow;
		this->material.ComputePlasticStrainFlow(plasticflow, mnode->e_strain);
		double dtpfact = dt_step* this->material.Get_flow_rate();
		if (dtpfact>1.0)
			dtpfact = 1.0; // clamp if dt is larger than plastic flow duration

		this->nodes[j]->p_strain.MatrInc(plasticflow*dtpfact);
		this->nodes[j]->e_strain.MatrDec(plasticflow*dtpfact);

	}

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


void ChMatterMeshless::SetNoSpeedNoAcceleration()
{
	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		this->nodes[j]->SetPos_dt(VNULL);
		this->nodes[j]->SetPos_dtdt(VNULL);
	}
}





//////






void ChMatterMeshless::Update()
{
	ChMatterMeshless::Update(this->GetChTime());
}

						
void ChMatterMeshless::Update (double mytime)
{	
		// Inherit time changes of parent class
	ChPhysicsItem::Update(mytime);

	//TrySleeping();		// See if the body can fall asleep; if so, put it to sleeping 
	//ClampSpeed();			// Apply limits (if in speed clamping mode) to speeds.
	
}



void ChMatterMeshless::UpdateExternalGeometry ()
{
	if (this->GetExternalObject())
		this->GetExternalObject()->onChronoChanged();
}

 
// collision stuff
void ChMatterMeshless::SetCollide (bool mcoll)
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

void ChMatterMeshless::SyncCollisionModels()
{
	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		this->nodes[j]->collision_model->SyncPosition();
	}
}

void ChMatterMeshless::AddCollisionModelsToSystem() 
{
	assert(this->GetSystem());
	SyncCollisionModels();
	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		this->GetSystem()->GetCollisionSystem()->Add(this->nodes[j]->collision_model);
	}
}

void ChMatterMeshless::RemoveCollisionModelsFromSystem() 
{
	assert(this->GetSystem());
	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		this->GetSystem()->GetCollisionSystem()->Remove(this->nodes[j]->collision_model);
	}
}


////

void ChMatterMeshless::UpdateParticleCollisionModels()
{
	for (unsigned int j = 0; j < nodes.size(); j++)
	{
		this->nodes[j]->collision_model->ClearModel();
		//***TO DO*** UPDATE RADIUS OF MeshlessERE? this->nodes[j]->collision_model->AddCopyOfAnotherModel(this->particle_collision_model);
		this->nodes[j]->collision_model->BuildModel();
	}
}




//////// FILE I/O

void ChMatterMeshless::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChIndexedNodes::StreamOUT(mstream);

		// stream out all member data
	mstream << this->material;

	//***TO DO*** stream nodes

}

void ChMatterMeshless::StreamIN(ChStreamInBinary& mstream)
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
