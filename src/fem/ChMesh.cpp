///////////////////////////////////////////////////
//
//   ChMesh.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChMath.h"
#include "physics/ChObject.h"
#include "fem/ChQuadra.h"
#include "fem/ChFem.h"
#include "fem/ChMesh.h"

namespace chrono 
{
namespace fem
{

//////////////////////////////////////
//////////////////////////////////////
//
// CLASS FOR FINITE ELEMENT MESH

ChFMesh::ChFMesh(int m_type)
{
	type = m_type;

	n_nodes = 0;
	n_coords = 0;
	n_elements =0;

	node_list = NULL;
	elem_list = NULL;

	E = 1000;
	v = 0.3;

	density = 1000; 
}

ChFMesh::~ChFMesh()
{
	Free_elements();
	Free_nodes();
}

void ChFMesh::Free_elements ()
{
	if (elem_list != NULL) { KillList ((ChObj**)&elem_list);}
}

void ChFMesh::Free_nodes ()
{
	if (node_list != NULL) { KillList ((ChObj**)&node_list);}
}

void ChFMesh::Add_node (ChFnode* m_node)
{
	// queue a node into nodelist;
	m_node->AddToList ((ChObj**) &node_list);
}

void ChFMesh::Add_element (ChFelem* m_elem)
{
	m_elem->AddToList ((ChObj**) &elem_list);
}


void ChFMesh::Set_E (double m_E)
{
	E = m_E;
	G = E/(2*(1+v));	// fixed v, E, get G
	l = (v*E)/((1+v)*(1-2*v));	// Lame's constants l,u
	u = G/2;
}

void ChFMesh::Set_v (double m_v)
{
	v = m_v;
	G = E/(2*(1+v));	// fixed v, E, get G
	l = (v*E)/((1+v)*(1-2*v));	// Lame's constants l,u 
	u = G/2;
}

void ChFMesh::Set_G (double m_G)
{
	G = m_G;
	v = (E/(2*G))-1;	// fixed G, E, get v
	l = (v*E)/((1+v)*(1-2*v)); // Lame's constants l,u
	u = G/2;
}


// Setup() is used justat the beginning of the simulation,
// after the user has added all the nodes and elements 
// to the mesh, with AddNode and AddElement
// It does the following:
//
// 1) - "relaxes" the structure by setting all X0 = 0
//    - count the nodes
// 2) - build [A] for all elements
//    - sets [A0]=[A] for all elements
//    - count elements
//    - sets the [E] for elements
//    - sets the [Kl] of each element

void ChFMesh::Setup ()
{
	n_nodes= 0; 
	n_elements = 0;
	n_coords = 0;

	ChFnode* Npointer = node_list;
	while (Npointer != NULL)
	{
			//    - count the nodes
		n_nodes ++;
			//    - count the degrees of freedom 
			// **** TO DO *** maybe there are unused knots!
		n_coords += Npointer->Get_dof();
			//    - "relaxes" the structure by setting all X0 = 0, and null speeds
		Npointer->Relax();

		Npointer = (ChFnode*) Npointer->GetNext();
	}

	ChFelem* Epointer = elem_list;
	while (Epointer != NULL)
	{
			//    - count the elements
		n_elements ++;

			//    - build [A] for all elements
		Epointer->Update_A();

			//    - sets [A0]=[A] for all elements
		Epointer->Setup_A0();

			//    - sets the [E] for elements
		Epointer->Compute_Ematr (E, v, G, l, u);

			//    - sets the [Kl] of each element
		Epointer->Compute_Kl();

		Epointer = (ChFelem*) Epointer->GetNext();
	}

}



// Updates all time-dependant variables, if any...
// Ex: maybe the elasticity can increase in time, etc.

void ChFMesh::UpdateTime (double m_time)
{
	ChTime = m_time;

	// *** TO DO: time dependant variables E,G,v, etc.
}


// Update_A()  is used at each simulation step, or during 
// the iterations of non-linear problems.
// It updates the local coordsystem [A] of the element, 
// depending on the new position of its knots.

void ChFMesh::Update_A ()
{
	ChFelem* Epointer = elem_list;
	while (Epointer != NULL)
	{
			//    - update [A] for all elements
		Epointer->Update_A();

		Epointer = (ChFelem*) Epointer->GetNext();
	}

}


// This more complete version of the Update function does the following:
// - it sets new position  X    for all nodes, fetching from a global state vector,
// - it sets new speeds    X_dt for all nodes, fetching from a global state vector,
// - it updates the local time
// - it updates the [A] coordinate matrix
// - updates time-dependant functions, if any... 


void ChFMesh::UpdateALL (ChMatrix<>* state_vector, int X_offset, int Xdt_offset, double m_time)
{
	int iX_offset   = X_offset;
	int iXdt_offset = Xdt_offset;
	
	// -) For all nodes, fetch the new states:
	ChFnode* Npointer = node_list;
	while (Npointer != NULL)
	{
			//    - sets the position of the knot
		Npointer->X = state_vector->ClipVector(iX_offset,0);

			//    - sets the speed of the knot
		Npointer->X_dt = state_vector->ClipVector(iXdt_offset,0);

			//    - increment the offset 
			// **** TO DO *** maybe there are unused knots!
		iX_offset   += Npointer->Get_dof();
		iXdt_offset += Npointer->Get_dof();


		Npointer = (ChFnode*) Npointer->GetNext();
	}

		// -) For all elements:

	Update_A();	// updates all [A]

		// -) Update time
	UpdateTime (m_time);

}


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

