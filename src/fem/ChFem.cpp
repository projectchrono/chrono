///////////////////////////////////////////////////
//
//   ChFem.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 


#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <float.h>
#include <memory.h>

#include "core/ChMath.h"
#include "core/ChTrasform.h"
#include "physics/ChObject.h"
#include "fem/ChFem.h"
#include "fem/ChQuadra.h"

namespace chrono 
{
namespace fem
{


//////////////////////////////////////
//////////////////////////////////////

// CLASS FOR FINITE ELEMENTS

// 0) Base FEM class

ChFelem::ChFelem()
{
}


ChFelem::~ChFelem()
{
}

void ChFelem::SetNode (int index, ChFnode* node)
{
	*(np_list + index) = node;
}


ChFnode* ChFelem::GetNode (int index)
{
	return (*(np_list + index));
}


void ChFelem::Setup_A0()
{
	Update_A();				// build a new coordinate matrix

	A0->CopyFromMatrix(*A);	//  [A0] = [A]
}


void ChFelem::Relax()
{
	int i;
	for (i= 0; i< Get_nodes(); i++)
	{
		GetNode(i)->Relax();			// 1)  X0= X
	}

	Setup_A0();							// 2) [A0]=[A]
}


Vector ChFelem::Get_NodeLocX0 (int index)
{
	return ( ChTrasform<>::TrasformParentToLocal (GetNode(index)->X0,	// old absolute point! 
							 	 GetNode(0)->X0,	    // old origin
								 *Get_A0()) );			// old csys
}

Vector ChFelem::Get_NodeLocX (int index)
{
	return ( ChTrasform<>::TrasformParentToLocal (GetNode(index)->X,		// actual absolute point! 
							 	 GetNode(0)->X,			// actual origin
								 *Get_A()) );			// actual csys
}

Vector ChFelem::Get_NodeLocDisplacement (int index)
{
			// local displacement  {dl}={Xl}-{X0l}
	return (Vsub (Get_NodeLocX (index), 
				  Get_NodeLocX0 (index)));
}



// Computes the static vector of displacements, in local coords,
// used by other calculus routines.

void ChFelem::Compute_Dl ()
{
	int i;
	Vector dl;

	for (i= 0; i< Get_nodes(); i++)
	{
		dl = Get_NodeLocDisplacement (i);
		Get_Dl()->PasteVector(dl, i*3, 0); 
	}
}

// Computes the vector of speed of knots,  in local coords,

void ChFelem::Compute_Dl_dt ()
{
	int i;
	Vector dl_dt;

	for (i= 0; i< Get_nodes(); i++)
	{
			// {Dl_dt}=[A]'{Dw_dt} 
		dl_dt = Get_A()->MatrT_x_Vect(GetNode(i)->X_dt);
		Get_Dl_dt()->PasteVector(dl_dt, i*3, 0); 
	}
}





void ChFelem::StrainToAbsCoords (ChMatrix<>* loc_strain, ChMatrix<>* abs_strain)
{
	ChMatrixNM<double,6,6> Te;  // the transformation matrix for strain
	ChMatrix33<>* Aa;

	Aa= Get_A();				// the actual coordinate of the element

	Te.SetElement (0,0, Aa->GetElement(0,0) * Aa->GetElement(0,0));
	Te.SetElement (0,1, Aa->GetElement(0,1) * Aa->GetElement(0,1));
	Te.SetElement (0,2, Aa->GetElement(0,2) * Aa->GetElement(0,2));
	Te.SetElement (0,3, Aa->GetElement(0,0) * Aa->GetElement(0,1));
	Te.SetElement (0,4, Aa->GetElement(0,1) * Aa->GetElement(0,2));
	Te.SetElement (0,5, Aa->GetElement(0,2) * Aa->GetElement(0,0));

	Te.SetElement (1,0, Aa->GetElement(1,0) * Aa->GetElement(1,0));
	Te.SetElement (1,1, Aa->GetElement(1,1) * Aa->GetElement(1,1));
	Te.SetElement (1,2, Aa->GetElement(1,2) * Aa->GetElement(1,2));
	Te.SetElement (1,3, Aa->GetElement(1,0) * Aa->GetElement(1,1));
	Te.SetElement (1,4, Aa->GetElement(1,1) * Aa->GetElement(1,2));
	Te.SetElement (1,5, Aa->GetElement(1,2) * Aa->GetElement(1,0));

	Te.SetElement (2,0, Aa->GetElement(2,0) * Aa->GetElement(2,0));
	Te.SetElement (2,1, Aa->GetElement(2,1) * Aa->GetElement(2,1));
	Te.SetElement (2,2, Aa->GetElement(2,2) * Aa->GetElement(2,2));
	Te.SetElement (2,3, Aa->GetElement(2,0) * Aa->GetElement(2,1));
	Te.SetElement (2,4, Aa->GetElement(2,1) * Aa->GetElement(2,2));
	Te.SetElement (2,5, Aa->GetElement(2,2) * Aa->GetElement(2,0));

	Te.SetElement (3,0, Aa->GetElement(0,0) * Aa->GetElement(1,0) * 2);
	Te.SetElement (3,1, Aa->GetElement(0,1) * Aa->GetElement(1,1) * 2);
	Te.SetElement (3,2, Aa->GetElement(0,2) * Aa->GetElement(1,2) * 2);
	Te.SetElement (3,3, Aa->GetElement(0,0) * Aa->GetElement(1,1) + Aa->GetElement(1,0) * Aa->GetElement(0,1));
	Te.SetElement (3,4, Aa->GetElement(0,1) * Aa->GetElement(1,2) + Aa->GetElement(1,1) * Aa->GetElement(0,2));
	Te.SetElement (3,5, Aa->GetElement(0,2) * Aa->GetElement(1,0) + Aa->GetElement(1,2) * Aa->GetElement(0,0));

	Te.SetElement (4,0, Aa->GetElement(1,0) * Aa->GetElement(2,0) * 2);
	Te.SetElement (4,1, Aa->GetElement(1,1) * Aa->GetElement(2,1) * 2);
	Te.SetElement (4,2, Aa->GetElement(1,2) * Aa->GetElement(2,2) * 2);
	Te.SetElement (4,3, Aa->GetElement(1,0) * Aa->GetElement(2,1) + Aa->GetElement(2,0) * Aa->GetElement(1,1));
	Te.SetElement (4,4, Aa->GetElement(1,1) * Aa->GetElement(2,2) + Aa->GetElement(2,1) * Aa->GetElement(1,2));
	Te.SetElement (4,5, Aa->GetElement(1,2) * Aa->GetElement(2,0) + Aa->GetElement(2,2) * Aa->GetElement(1,0));

	Te.SetElement (5,0, Aa->GetElement(2,0) * Aa->GetElement(0,0) * 2);
	Te.SetElement (5,1, Aa->GetElement(2,1) * Aa->GetElement(0,1) * 2);
	Te.SetElement (5,2, Aa->GetElement(2,2) * Aa->GetElement(0,2) * 2);
	Te.SetElement (5,3, Aa->GetElement(2,0) * Aa->GetElement(0,1) + Aa->GetElement(0,0) * Aa->GetElement(2,1));
	Te.SetElement (5,4, Aa->GetElement(2,1) * Aa->GetElement(0,2) + Aa->GetElement(0,1) * Aa->GetElement(2,2));
	Te.SetElement (5,5, Aa->GetElement(2,2) * Aa->GetElement(0,0) + Aa->GetElement(0,2) * Aa->GetElement(2,0));

	abs_strain->MatrMultiply (Te, *loc_strain);	 // %%% performs transformation
}


void ChFelem::StressToAbsCoords (ChMatrix<>* loc_stress, ChMatrix<>* abs_stress)
{
	ChMatrixNM<double,6,6> To;  // the transformation matrix for stresses
	ChMatrix33<>* Aa;

	Aa= Get_A();				// the actual coordinate of the element

	To.SetElement (0,0, Aa->GetElement(0,0) * Aa->GetElement(0,0));
	To.SetElement (0,1, Aa->GetElement(0,1) * Aa->GetElement(0,1));
	To.SetElement (0,2, Aa->GetElement(0,2) * Aa->GetElement(0,2));
	To.SetElement (0,3, Aa->GetElement(0,0) * Aa->GetElement(0,1) *2);
	To.SetElement (0,4, Aa->GetElement(0,1) * Aa->GetElement(0,2) *2);
	To.SetElement (0,5, Aa->GetElement(0,2) * Aa->GetElement(0,0) *2);

	To.SetElement (1,0, Aa->GetElement(1,0) * Aa->GetElement(1,0));
	To.SetElement (1,1, Aa->GetElement(1,1) * Aa->GetElement(1,1));
	To.SetElement (1,2, Aa->GetElement(1,2) * Aa->GetElement(1,2));
	To.SetElement (1,3, Aa->GetElement(1,0) * Aa->GetElement(1,1) *2);
	To.SetElement (1,4, Aa->GetElement(1,1) * Aa->GetElement(1,2) *2);
	To.SetElement (1,5, Aa->GetElement(1,2) * Aa->GetElement(1,0) *2);

	To.SetElement (2,0, Aa->GetElement(2,0) * Aa->GetElement(2,0));
	To.SetElement (2,1, Aa->GetElement(2,1) * Aa->GetElement(2,1));
	To.SetElement (2,2, Aa->GetElement(2,2) * Aa->GetElement(2,2));
	To.SetElement (2,3, Aa->GetElement(2,0) * Aa->GetElement(2,1) *2);
	To.SetElement (2,4, Aa->GetElement(2,1) * Aa->GetElement(2,2) *2);
	To.SetElement (2,5, Aa->GetElement(2,2) * Aa->GetElement(2,0) *2);

	To.SetElement (3,0, Aa->GetElement(0,0) * Aa->GetElement(1,0));
	To.SetElement (3,1, Aa->GetElement(0,1) * Aa->GetElement(1,1));
	To.SetElement (3,2, Aa->GetElement(0,2) * Aa->GetElement(1,2));
	To.SetElement (3,3, Aa->GetElement(0,0) * Aa->GetElement(1,1) + Aa->GetElement(1,0) * Aa->GetElement(0,1));
	To.SetElement (3,4, Aa->GetElement(0,1) * Aa->GetElement(1,2) + Aa->GetElement(1,1) * Aa->GetElement(0,2));
	To.SetElement (3,5, Aa->GetElement(0,2) * Aa->GetElement(1,0) + Aa->GetElement(1,2) * Aa->GetElement(0,0));

	To.SetElement (4,0, Aa->GetElement(1,0) * Aa->GetElement(2,0));
	To.SetElement (4,1, Aa->GetElement(1,1) * Aa->GetElement(2,1));
	To.SetElement (4,2, Aa->GetElement(1,2) * Aa->GetElement(2,2));
	To.SetElement (4,3, Aa->GetElement(1,0) * Aa->GetElement(2,1) + Aa->GetElement(2,0) * Aa->GetElement(1,1));
	To.SetElement (4,4, Aa->GetElement(1,1) * Aa->GetElement(2,2) + Aa->GetElement(2,1) * Aa->GetElement(1,2));
	To.SetElement (4,5, Aa->GetElement(1,2) * Aa->GetElement(2,0) + Aa->GetElement(2,2) * Aa->GetElement(1,0));

	To.SetElement (5,0, Aa->GetElement(2,0) * Aa->GetElement(0,0));
	To.SetElement (5,1, Aa->GetElement(2,1) * Aa->GetElement(0,1));
	To.SetElement (5,2, Aa->GetElement(2,2) * Aa->GetElement(0,2));
	To.SetElement (5,3, Aa->GetElement(2,0) * Aa->GetElement(0,1) + Aa->GetElement(0,0) * Aa->GetElement(2,1));
	To.SetElement (5,4, Aa->GetElement(2,1) * Aa->GetElement(0,2) + Aa->GetElement(0,1) * Aa->GetElement(2,2));
	To.SetElement (5,5, Aa->GetElement(2,2) * Aa->GetElement(0,0) + Aa->GetElement(0,2) * Aa->GetElement(2,0));

	abs_stress->MatrMultiply (To, *loc_stress);	 // %%% performs transformation
}


void ChFelem::Compute_LocStrainStress (Vector* par, ChMatrix<>* loc_strain, ChMatrix<>* loc_stress)
{
    // ------- TO GET STRAIN:

	Compute_Be (par->x, par->y, par->z);	// updates Be for given parameter coordinates

			// local strain is   {el}=[Be]{dl}  , with {dl} local displacements
	loc_strain->MatrMultiply (*Get_Be(), *Get_Dl());


	// ------- TO GET STRESS:

	if (loc_stress != NULL)
	{
			// local stress is   {ol}=[E]{el}
		loc_stress->MatrMultiply(*Get_E(), *loc_strain);
	}
}


void ChFelem::Compute_Fr ()
{
	// 1) in local coords: {Frl} = [Kl]{Dl} 
	Get_Fr()->MatrMultiply (*Get_Kl(), *Get_Dl());

	// 2) transform the forces into world coordinates
	int i;
	Vector m_force;

	for (i= 0; i< Get_nodes(); i++)
	{
			// pick the coordinates from full vector
		m_force = Get_Fr()->ClipVector(i*3, 0);
			// transform {Fr}=[A]{Frl}
		m_force = Get_A()->Matr_x_Vect (m_force);
			// push the coords back into modifyed vector,
			// and go on with next 3 coords.
		Get_Fr()->PasteVector(m_force, i*3, 0); 
	}
}






} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

