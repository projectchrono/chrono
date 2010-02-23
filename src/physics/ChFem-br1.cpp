///////////////////////////////////////////////////
//
//   ChFem-br1.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <float.h>
#include <memory.h>

#include "core/ChMath.h"
#include "core/ChTrasform.h"
#include "physics/ChObject.h"
#include "physics/ChQuadra.h"
#include "physics/ChFem.h"
#include "physics/ChFem-br1.h"


namespace chrono 
{

//////////////////////////////////////
//////////////////////////////////////
//
// CLASS FOR 3D BRICK ELEMENT "BR1"

		//initialize static variables -only few matrices allocated, used
		// in common by all "BrickLinear" elements, etc.
ChMatrix<>* ChFelem_BrickLinear::Ematr = NULL;
ChMatrix<>* ChFelem_BrickLinear::N = NULL;
ChMatrix<>* ChFelem_BrickLinear::Bi = NULL;
ChMatrix<>* ChFelem_BrickLinear::Be = NULL;
ChMatrix<>* ChFelem_BrickLinear::Bx = NULL;
ChMatrix<>* ChFelem_BrickLinear::C = NULL;
ChMatrix33<>* ChFelem_BrickLinear::J = NULL;
ChMatrix33<>* ChFelem_BrickLinear::Jinv = NULL;
ChMatrix<>* ChFelem_BrickLinear::BEBj = NULL;
ChMatrix<>* ChFelem_BrickLinear::tempM = NULL;
ChMatrix<>* ChFelem_BrickLinear::Dl = NULL;
ChMatrix<>* ChFelem_BrickLinear::Dl_dt = NULL;
ChMatrix<>* ChFelem_BrickLinear::Fr = NULL;
double  ChFelem_BrickLinear::Jdet = 0;
ChVector<>  ChFelem_BrickLinear::par(0,0,0);
int     ChFelem_BrickLinear::fem_instanced = 0;


ChFelem_BrickLinear::ChFelem_BrickLinear()
{
	Kl = new ChMatrixDynamic<> (24,24);
	A  = new ChMatrix33<>;
	A0 = new ChMatrix33<>;

	// static matrices: allocate only first time.
	if (fem_instanced == 0)
	{
		Ematr = new ChMatrixDynamic<> (6,6);
		N  = new ChMatrixDynamic<> (1,8);
		Bi = new ChMatrixDynamic<> (3,8);
		Bx = new ChMatrixDynamic<> (3,8);
		Be = new ChMatrixDynamic<> (6,24);
		C  = new ChMatrixDynamic<> (8,3);
		J  = new ChMatrix33<>;
		Jinv = new ChMatrix33<>;
		Dl    = new ChMatrixDynamic<> (24,1);
		Dl_dt = new ChMatrixDynamic<> (24,1);
		Fr   = new ChMatrixDynamic<> (24,1);
		tempM = new ChMatrixDynamic<> (6,24); 	// E rows x Be columns !
		BEBj = new ChMatrixDynamic<> (24,24);
	}

	fem_instanced++;

	volume = 0;
	mass= 0;
				// Alloc memory for 8 pointers to nodes
	np_list = (ChFnode**) calloc (8, sizeof(ChFnode*));
}

ChFelem_BrickLinear::~ChFelem_BrickLinear()
{
	delete A;
	delete A0;
	delete Kl;

	free (np_list);  // frees the array of pointers to nodes !

	fem_instanced-- ;

	if (fem_instanced == 0)		// free the static matrices
	{
		delete Ematr;
		delete N;
		delete Bi;
		delete Bx;
		delete Be;
		delete C;
		delete J;
		delete Jinv;
		delete tempM;
		delete BEBj;
		delete Dl;
		delete Dl_dt;
		delete Fr;
	}
}

void ChFelem_BrickLinear::Compute_Ematr (double E, double v, double G, double l, double u)
{
	double d1 = l + 2*u;	// upper diag terms
	double d2 = 2*u;		// lower diag terms

	Ematr->SetElement(0,0,d1);
	Ematr->SetElement(1,1,d1);
	Ematr->SetElement(2,2,d1);

	Ematr->SetElement(3,3,d2);
	Ematr->SetElement(4,4,d2);
	Ematr->SetElement(5,5,d2);

	Ematr->SetElement(0,1,l);
	Ematr->SetElement(0,2,l);
	Ematr->SetElement(1,0,l);
	Ematr->SetElement(1,2,l);
	Ematr->SetElement(2,0,l);
	Ematr->SetElement(2,1,l);	
}

void ChFelem_BrickLinear::Compute_C()
{
	int i;
	/* static */ Vector rel_point;
	int rows = Get_nodes();

	for (i=0; i<rows; i++)
	{
		// transform each point into relative coords 'A'
		rel_point = ChTrasform<>::TrasformParentToLocal (GetNode(i)->X,	// absolute point! 
												GetNode(0)->X,	// origin
												*Get_A());		// coords orientation

		C->SetElement (i,0, rel_point.x);
		C->SetElement (i,1, rel_point.y);
		C->SetElement (i,2, rel_point.z);
	}
}

void ChFelem_BrickLinear::Compute_N (double i, double j, double k) 
{
		N->SetElement(0,0, (1-i)*(1-j)*(1-k)/8 );
		N->SetElement(0,1, (1+i)*(1-j)*(1-k)/8 );
		N->SetElement(0,2, (1-i)*(1+j)*(1-k)/8 );
		N->SetElement(0,3, (1+i)*(1+j)*(1-k)/8 );
		N->SetElement(0,4, (1-i)*(1-j)*(1+k)/8 );
		N->SetElement(0,5, (1+i)*(1-j)*(1+k)/8 );
		N->SetElement(0,6, (1-i)*(1+j)*(1+k)/8 );
		N->SetElement(0,7, (1+i)*(1+j)*(1+k)/8 );
}

void ChFelem_BrickLinear::Compute_Bi (double i, double j, double k)
{
    // set shape derivatives [Bi], in param.coords
									// dN/di
		Bi->SetElement(0,0, -(1-j)*(1-k)/8 );
		Bi->SetElement(0,1, +(1-j)*(1-k)/8 );
		Bi->SetElement(0,2, -(1+j)*(1-k)/8 );
		Bi->SetElement(0,3, +(1+j)*(1-k)/8 );
		Bi->SetElement(0,4, -(1-j)*(1+k)/8 );
		Bi->SetElement(0,5, +(1-j)*(1+k)/8 );
		Bi->SetElement(0,6, -(1+j)*(1+k)/8 );
		Bi->SetElement(0,7, +(1+j)*(1+k)/8 );
									// dN/dj				
		Bi->SetElement(1,0, -(1-i)*(1-k)/8 );
		Bi->SetElement(1,1, -(1+i)*(1-k)/8 );
		Bi->SetElement(1,2, +(1-i)*(1-k)/8 );
		Bi->SetElement(1,3, +(1+i)*(1-k)/8 );
		Bi->SetElement(1,4, -(1-i)*(1+k)/8 );
		Bi->SetElement(1,5, -(1+i)*(1+k)/8 );
		Bi->SetElement(1,6, +(1-i)*(1+k)/8 );
		Bi->SetElement(1,7, +(1+i)*(1+k)/8 );
									// dN/dk
		Bi->SetElement(2,0, -(1-i)*(1-j)/8 );
		Bi->SetElement(2,1, -(1+i)*(1-j)/8 );
		Bi->SetElement(2,2, -(1-i)*(1+j)/8 );
		Bi->SetElement(2,3, -(1+i)*(1+j)/8 );
		Bi->SetElement(2,4, +(1-i)*(1-j)/8 );
		Bi->SetElement(2,5, +(1+i)*(1-j)/8 );
		Bi->SetElement(2,6, +(1-i)*(1+j)/8 );
		Bi->SetElement(2,7, +(1+i)*(1+j)/8 );
}

void ChFelem_BrickLinear::Compute_Bx (double i, double j, double k)
{
	// set shape derivatives [Bx], in relative coords,
	// calling internally the previous function Set_Bi, for current [C]

	Compute_Bi (i, j, k);			// 1- Builds [Bi]

	J->MatrMultiply ( *Bi, *C);		// 2- Builds [J]= [Bi][C]
	Jdet = J->FastInvert(Jinv);		// 3- Builds [Jinv] = [J]'
	Bx->MatrMultiply ( *Jinv, *Bi);	// 4- Builds [Bx] = [Jinv][Bi]
}


void ChFelem_BrickLinear::Compute_Be (double i, double j, double k)
{
	int nod, offs;

	nod= 0;

	Compute_Bx (i,j,k);			// %%%  Compute Bx (so also Bi) 

	for(nod= 0; nod < 8; nod++)
	{
		offs=nod*3;
		Be->SetElement (0,0+offs ,Bx->GetElement(0,nod)); //Nx
		Be->SetElement (1,1+offs ,Bx->GetElement(1,nod)); //Ny
		Be->SetElement (2,2+offs ,Bx->GetElement(2,nod)); //Nz
		Be->SetElement (3,0+offs ,Bx->GetElement(1,nod)); //Ny
		Be->SetElement (3,1+offs ,Bx->GetElement(0,nod)); //Nx
		Be->SetElement (4,1+offs ,Bx->GetElement(2,nod)); //Nz 
		Be->SetElement (4,2+offs ,Bx->GetElement(1,nod)); //Ny
		Be->SetElement (5,0+offs ,Bx->GetElement(2,nod)); //Nz
		Be->SetElement (5,2+offs ,Bx->GetElement(0,nod)); //Nx
	}
}

// Computes the argument of the 3d integral which
// gives the [K]l by quadrature, for current [C]
// Remember!!! Update the "par" vector of 3 parameters, just before
// calling it, they  cannot be passed as argumants for strange reasons.
// It is like    BEBj = BEBj(par)

ChMatrix<>*  ChFelem_BrickLinear::Compute_BEBj (double i, double j, double k)
{

	Compute_Be (i, j, k);					// 1- Builds  [Be]  from current [Bi]->[Bx]->..

	tempM->MatrMultiply ( *Ematr, *Be );
	BEBj->MatrTMultiply  ( *Be, *tempM );

	BEBj->MatrScale(Jdet);					// 2- [BEBj] = [B]'[E][B]*j

	return (BEBj);
}



void ChFelem_BrickLinear::Compute_Kl()
{
	Compute_C();			// builds the coordinate matrix
							// once for Kl calculation

	ChQuadrature m_integrator; // creates the integrator object and integrate:
	m_integrator.GaussIntegrate (
				this,			// FEM object to integrate
				Kl,				// result matrix pointer
				1,1,1,			// integration order in ijk
				3);				// 3 dimensions
}

void ChFelem_BrickLinear::Update_A()
{
	/*static*/ Vector vc, vx, vy, vz;

		// for brick element, use the points of a corner, rectified, to build coordsys.
	vc = GetNode(0)->X;
	vx = Vnorm (Vsub (GetNode(1)->X, vc));
	vy = Vnorm (Vcross (Vsub(GetNode(4)->X, vc),vx));
	vz = Vcross (vx, vy);

	A->Set_A_axis(vx,vy,vz);
}


} // END_OF_NAMESPACE____

