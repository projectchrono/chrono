//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File authors: Andrea Favali, Alessandro Tasora


#include "chrono/core/ChMath.h"
#include "chrono/physics/ChObject.h"
#include "chrono/physics/ChLoad.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChNodeFEAxyz.h"
#include "chrono_fea/ChElementTetra_4.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>
#include <functional> 

using namespace std;

namespace chrono  {
namespace fea {

void ChMesh::SetupInitial()
{
  n_dofs = 0;
  n_dofs_w = 0;

  for (unsigned int i=0; i< vnodes.size(); i++)
  {
    if (!vnodes[i]->GetFixed())
    {
      //    - count the degrees of freedom
      n_dofs += vnodes[i]->Get_ndof_x();
      n_dofs_w += vnodes[i]->Get_ndof_w();
    }
  }

  for (unsigned int i=0; i< velements.size(); i++)
  {
    //    - precompute matrices, such as the [Kl] local stiffness of each element, if needed, etc.
    velements[i]->SetupInitial(GetSystem());
  }

}


void ChMesh::Relax ()
{
  for (unsigned int i=0; i< vnodes.size(); i++)
  {
    //    - "relaxes" the structure by setting all X0 = 0, and null speeds
    vnodes[i]->Relax();
  }
}


void ChMesh::SetNoSpeedNoAcceleration()
{ 
  for (unsigned int i=0; i< vnodes.size(); i++)
  {
    //    -  set null speeds, null accelerations
    vnodes[i]->SetNoSpeedNoAcceleration();
  }
}

void ChMesh::AddNode(std::shared_ptr<ChNodeFEAbase> m_node) {
    this->vnodes.push_back(m_node);
}

void ChMesh::AddElement(std::shared_ptr<ChElementBase> m_elem) {
    this->velements.push_back(m_elem);
}

void ChMesh::ClearElements() {
    velements.clear();
    vcontactsurfaces.clear();
}

void ChMesh::ClearNodes() {
    velements.clear();
    vnodes.clear();
    vcontactsurfaces.clear();
}

void ChMesh::AddContactSurface(std::shared_ptr<ChContactSurface> m_surf) {
    m_surf->SetMesh(this);
    this->vcontactsurfaces.push_back(m_surf);
}

void ChMesh::ClearContactSurfaces() {
    vcontactsurfaces.clear();
}

void ChMesh::AddMeshSurface(std::shared_ptr<ChMeshSurface> m_surf) {
    m_surf->SetMesh(this);
    this->vmeshsurfaces.push_back(m_surf);
}

/// This recomputes the number of DOFs, constraints,
/// as well as state offsets of contained items
void ChMesh::Setup() {
    n_dofs = 0;
    n_dofs_w = 0;

    for (unsigned int i = 0; i < vnodes.size(); i++) {
        if (!vnodes[i]->GetFixed()) {
            vnodes[i]->NodeSetOffset_x(this->GetOffset_x() + n_dofs);
            vnodes[i]->NodeSetOffset_w(this->GetOffset_w() + n_dofs_w);

            //    - count the degrees of freedom
            n_dofs += vnodes[i]->Get_ndof_x();
            n_dofs_w += vnodes[i]->Get_ndof_w();
        }
    }
}

// Updates all time-dependant variables, if any...
// Ex: maybe the elasticity can increase in time, etc.
void ChMesh::Update(double m_time, bool update_assets) {
    // Parent class update
    ChIndexedNodes::Update(m_time, update_assets);

    for (unsigned int i = 0; i < velements.size(); i++) {
        //    - update auxiliary stuff, ex. update element's rotation matrices if corotational..
        velements[i]->Update();
    }
}

void ChMesh::SyncCollisionModels() {
  for (unsigned int j = 0; j < vcontactsurfaces.size(); j++) {
    this->vcontactsurfaces[j]->SurfaceSyncCollisionModels();
  }
}

void ChMesh::AddCollisionModelsToSystem() {
  assert(this->GetSystem());
  SyncCollisionModels();
  for (unsigned int j = 0; j < vcontactsurfaces.size(); j++) {
    this->vcontactsurfaces[j]->SurfaceAddCollisionModelsToSystem(this->GetSystem());
  }
}

void ChMesh::RemoveCollisionModelsFromSystem() {
  assert(this->GetSystem());
  for (unsigned int j = 0; j < vcontactsurfaces.size(); j++) {
    this->vcontactsurfaces[j]->SurfaceRemoveCollisionModelsFromSystem(this->GetSystem());
  }
}


//// STATE BOOKKEEPING FUNCTIONS

void ChMesh::IntStateGather(
    const unsigned int off_x,		///< offset in x state vector
    ChState& x,						///< state vector, position part
    const unsigned int off_v,		///< offset in v state vector
    ChStateDelta& v,				///< state vector, speed part
    double& T)						///< time
{
  unsigned int local_off_x=0;
  unsigned int local_off_v=0;
  for (unsigned int j = 0; j < vnodes.size(); j++)
  {
    if (!vnodes[j]->GetFixed())
    {
      vnodes[j]->NodeIntStateGather(	off_x+local_off_x,
          x,
          off_v+local_off_v,
          v,
          T);
      local_off_x += vnodes[j]->Get_ndof_x();
      local_off_v += vnodes[j]->Get_ndof_w();
    }
  }

  T = this->GetChTime();
}

void ChMesh::IntStateScatter(
    const unsigned int off_x,		///< offset in x state vector
    const ChState& x,				///< state vector, position part
    const unsigned int off_v,		///< offset in v state vector
    const ChStateDelta& v,			///< state vector, speed part
    const double T) 				///< time
{
  unsigned int local_off_x=0;
  unsigned int local_off_v=0;
  for (unsigned int j = 0; j < vnodes.size(); j++)
  {
    if (!vnodes[j]->GetFixed())
    {
      vnodes[j]->NodeIntStateScatter(	off_x+local_off_x,
          x,
          off_v+local_off_v,
          v,
          T);
      local_off_x += vnodes[j]->Get_ndof_x();
      local_off_v += vnodes[j]->Get_ndof_w();
    }
  }

  this->Update(T);
}

void ChMesh::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a)
{
  unsigned int local_off_a=0;
  for (unsigned int j = 0; j < vnodes.size(); j++)
  {
    if (!vnodes[j]->GetFixed())
    {
      vnodes[j]->NodeIntStateGatherAcceleration(	off_a+local_off_a,  a);
      local_off_a += vnodes[j]->Get_ndof_w();
    }
  }
}

void ChMesh::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a)
{
  unsigned int local_off_a=0;
  for (unsigned int j = 0; j < vnodes.size(); j++)
  {
    if (!vnodes[j]->GetFixed())
    {
      vnodes[j]->NodeIntStateScatterAcceleration(	off_a+local_off_a, a);
      local_off_a += vnodes[j]->Get_ndof_w();
    }
  }
}

void ChMesh::IntStateIncrement(
    const unsigned int off_x,		///< offset in x state vector
    ChState& x_new,					///< state vector, position part, incremented result
    const ChState& x,				///< state vector, initial position part
    const unsigned int off_v,		///< offset in v state vector
    const ChStateDelta& Dv)  		///< state vector, increment
{
  unsigned int local_off_x=0;
  unsigned int local_off_v=0;
  for (unsigned int j = 0; j < vnodes.size(); j++)
  {
    if (!vnodes[j]->GetFixed())
    {
      vnodes[j]->NodeIntStateIncrement(	off_x+local_off_x,
          x_new,
          x,
          off_v+local_off_v,
          Dv);
      local_off_x += vnodes[j]->Get_ndof_x();
      local_off_v += vnodes[j]->Get_ndof_w();
    }
  }
}

void ChMesh::IntLoadResidual_F(
    const unsigned int off,		 ///< offset in R residual (not used here! use particle's offsets)
    ChVectorDynamic<>& R,		 ///< result: the R residual, R += c*F
    const double c				 ///< a scaling factor
)
{
  // applied nodal forces
  unsigned int local_off_v=0;
  for (unsigned int j = 0; j < vnodes.size(); j++)
  {
    if (!vnodes[j]->GetFixed())
    {
      this->vnodes[j]->NodeIntLoadResidual_F(	off+local_off_v,
          R,
          c);
      local_off_v += vnodes[j]->Get_ndof_w();
    }
  }

  // internal forces
#pragma omp parallel for
    for (int ie = 0; ie < this->velements.size(); ie++) {
        this->velements[ie]->EleIntLoadResidual_F(R, c);
    }

    // Apply gravity loads without the need of adding
    // a ChLoad object to each element: just instance here a single ChLoad and reuse
    // it for all 'volume' objects.
    if (automatic_gravity_load) {
        std::shared_ptr<ChLoadableUVW> mloadable;  // still null
        auto common_gravity_loader = std::make_shared<ChLoad<ChLoaderGravity>>(mloadable);
        common_gravity_loader->loader.Set_G_acc(this->GetSystem()->Get_G_acc());
        common_gravity_loader->loader.SetNumIntPoints(num_points_gravity);

        for (unsigned int ie = 0; ie < this->velements.size(); ie++) {
            if (mloadable = std::dynamic_pointer_cast<ChLoadableUVW>(this->velements[ie])) {
                if (mloadable->GetDensity()) {
                    // temporary set loader target and compute generalized forces term
                    common_gravity_loader->loader.loadable = mloadable;
                    common_gravity_loader->ComputeQ(0, 0);
                    common_gravity_loader->LoadIntLoadResidual_F(R, c);
                }
            }
        }
  }
}


void ChMesh::IntLoadResidual_Mv(
    const unsigned int off,		 ///< offset in R residual
    ChVectorDynamic<>& R,		 ///< result: the R residual, R += c*M*v
    const ChVectorDynamic<>& w,  ///< the w vector
    const double c				 ///< a scaling factor
)
{
  // nodal masses
  unsigned int local_off_v=0;
  for (unsigned int j = 0; j < vnodes.size(); j++)
  {
    if (!vnodes[j]->GetFixed())
    {
      vnodes[j]->NodeIntLoadResidual_Mv(	off+local_off_v,  R, w, c);
      local_off_v += vnodes[j]->Get_ndof_w();
    }
  }

  // internal masses
  for (unsigned int ie = 0; ie < this->velements.size(); ie++)
  {
    this->velements[ie]->EleIntLoadResidual_Mv(R, w, c);
  }
}

void ChMesh::IntToLCP(
    const unsigned int off_v,			///< offset in v, R
    const ChStateDelta& v,
    const ChVectorDynamic<>& R,
    const unsigned int off_L,			///< offset in L, Qc
    const ChVectorDynamic<>& L,
    const ChVectorDynamic<>& Qc
)
{
  unsigned int local_off_v=0;
  for (unsigned int j = 0; j < vnodes.size(); j++)
  {
    if (!vnodes[j]->GetFixed())
    {
      vnodes[j]->NodeIntToLCP(off_v + local_off_v,  v, R);
      local_off_v += vnodes[j]->Get_ndof_w();
    }
  }
}

void ChMesh::IntFromLCP(
    const unsigned int off_v,			///< offset in v
    ChStateDelta& v,
    const unsigned int off_L,			///< offset in L
    ChVectorDynamic<>& L
)
{
  unsigned int local_off_v=0;
  for (unsigned int j = 0; j < vnodes.size(); j++)
  {
    if (!vnodes[j]->GetFixed())
    {
      vnodes[j]->NodeIntFromLCP(off_v + local_off_v,  v);
      local_off_v += vnodes[j]->Get_ndof_w();
    }
  }
}




//// LCP SOLVER

void ChMesh::InjectKRMmatrices(ChLcpSystemDescriptor& mdescriptor) 
{
  for (unsigned int ie = 0; ie < this->velements.size(); ie++)
    this->velements[ie]->InjectKRMmatrices(mdescriptor);
}

void ChMesh::KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor)
{
#pragma omp parallel for 
  for (int ie = 0; ie < this->velements.size(); ie++)
    this->velements[ie]->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
}

void ChMesh::VariablesFbReset()
{
  for (unsigned int ie = 0; ie < this->vnodes.size(); ie++)
    this->vnodes[ie]->VariablesFbReset();
}

void ChMesh::VariablesFbLoadForces(double factor)
{
  // applied nodal forces
  for (unsigned int in = 0; in < this->vnodes.size(); in++)
    this->vnodes[in]->VariablesFbLoadForces(factor);

  // internal forces
  for (unsigned int ie = 0; ie < this->velements.size(); ie++)
    this->velements[ie]->VariablesFbLoadInternalForces(factor);
}

void ChMesh::VariablesQbLoadSpeed() 
{
  for (unsigned int ie = 0; ie < this->vnodes.size(); ie++)
    this->vnodes[ie]->VariablesQbLoadSpeed();
}

void ChMesh::VariablesFbIncrementMq() 
{
  // nodal masses
  for (unsigned int ie = 0; ie < this->vnodes.size(); ie++)
    this->vnodes[ie]->VariablesFbIncrementMq();

  // internal masses
  for (unsigned int ie = 0; ie < this->velements.size(); ie++)
    this->velements[ie]->VariablesFbIncrementMq();
}

void ChMesh::VariablesQbSetSpeed(double step) 
{
  for (unsigned int ie = 0; ie < this->vnodes.size(); ie++)
    this->vnodes[ie]->VariablesQbSetSpeed(step);
}

void ChMesh::VariablesQbIncrementPosition(double step)
{
  for (unsigned int ie = 0; ie < this->vnodes.size(); ie++)
    this->vnodes[ie]->VariablesQbIncrementPosition(step);
}

void ChMesh::InjectVariables(ChLcpSystemDescriptor& mdescriptor)
{
  for (unsigned int ie = 0; ie < this->vnodes.size(); ie++)
    this->vnodes[ie]->InjectVariables(mdescriptor);

  //mdescriptor.InsertVariables(&this->vnodes[ie]->Variables());
}






} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


