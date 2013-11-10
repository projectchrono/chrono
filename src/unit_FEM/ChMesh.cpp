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


#include "core/ChMath.h"
#include "physics/ChObject.h"
#include "ChMesh.h"

namespace chrono 
{
namespace fem
{



void ChMesh::SetupInitial()
{
	n_dofs = 0;

	for (unsigned int i=0; i< vnodes.size(); i++)
	{
			//    - count the degrees of freedom 
		n_dofs += vnodes[i]->Get_ndof();
	}

	for (unsigned int i=0; i< velements.size(); i++)
	{
			//    - precompute matrices, such as the [Kl] local stiffness of each element, if needed, etc.
		velements[i]->SetupInitial();
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


void ChMesh::AddNode (ChNodeFEMbase& m_node)
{
	this->vnodes.push_back(&m_node);
}

void ChMesh::AddElement (ChElementBase& m_elem)
{
	this->velements.push_back(&m_elem);
}

void ChMesh::ClearElements ()
{
	velements.clear();
}

void ChMesh::ClearNodes ()
{
	velements.clear();
	vnodes.clear();
}





// Updates all time-dependant variables, if any...
// Ex: maybe the elasticity can increase in time, etc.

void ChMesh::UpdateTime (double m_time)
{
	ChIndexedNodes::Update(m_time);
	
	for (unsigned int i=0; i< velements.size(); i++)
	{
			//    - update auxiliary stuff, ex. update element's rotation matrices if corotational..
		velements[i]->Update();
	}

}


void ChMesh::InjectKRMmatrices(ChLcpSystemDescriptor& mdescriptor) 
{
	for (unsigned int ie = 0; ie < this->velements.size(); ie++)
		this->velements[ie]->InjectKRMmatrices(mdescriptor);
}

void ChMesh::KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor)
{
	for (unsigned int ie = 0; ie < this->velements.size(); ie++)
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
		mdescriptor.InsertVariables(&this->vnodes[ie]->Variables());
}






} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

