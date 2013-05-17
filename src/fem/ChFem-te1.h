#ifndef CHFEM_TE1_H
#define CHFEM_TE1_H

//////////////////////////////////////////////////
//  
//   ChFem-te1.h
//
//   TETRAHEDRON LINEAR
//   Definition of the "te1" (tetrahedron 3d linear) 
//   solid element, inherited from the basic 
//   "Felem" class.  ***OBSOLETE***
//  
//   
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <math.h>
#include <float.h>
#include <memory.h>

#include "core/ChLog.h"
#include "core/ChMath.h"
#include "fem/ChFem.h"


namespace chrono 
{
namespace fem
{


#define CHCLASS_FELEM_TETRALINEAR 22

// INHERIT ALL THE FINITE ELEMENTS FROM THE BASE CLASS "Felem", PROVIDING THE
// SPECIFIC IMPLEMENTATIONS OF THE VIRTUAL FUNCTIONS.
// The most important functions to provide for each custom FEM are:
// - Compute_Kl    (to provide the stiffness matrix in local coords)
// - Update_A      (to get a coordsys from a FEM, given the knots)
// Lot of other functions can be added, for intermediate calculus,
// and lot of variables can be implemented (we suggest to implement
// most of matrices as static objects, whenever possible, just to
// waste as few memory as possible.

/// Tetrahedron, 3d constant stress, for Finite Element methods.
/// Uses the parametric element theory.

class ChApi ChFelem_TetraLinear : public ChFelem
{
private:
	static ChMatrix<>* Ematr;
	static ChMatrix<>* N;
	static ChMatrix<>* Bi;
	static ChMatrix<>* Bx;
	static ChMatrix<>* Be;
	static ChMatrix<>* C;
	static ChMatrix33<>* J;
	static ChMatrix33<>* Jinv;
	static ChMatrix<>* tempM;
	static ChMatrix<>* BEBj;
	static ChMatrix<>* Dl;
	static ChMatrix<>* Dl_dt;
	static ChMatrix<>* Fr;
	static double Jdet;
	static ChVector<> par;
	static int fem_instanced;	
public:

	ChFelem_TetraLinear();
	~ChFelem_TetraLinear();


	int Get_type () {return FELEM_TETRA_LINEAR;};
	int Get_dof	 () {return 12;}
	int Get_nodes () {return 4;}
	
	ChMatrix<>* Get_E() {return Ematr;}		
	ChMatrix<>* Get_C() {return C;}
	ChMatrix<>* Get_N()  {return N;}
	ChMatrix<>* Get_Be() {return Be;}
	ChMatrix<>* Get_Dl() {return Dl;}
	ChMatrix<>* Get_Fr() {return Fr;}

	void Compute_Ematr (double E, double v, double G, double l, double u);
	void Compute_C ();								///< set coords [C] matrix
	void Compute_N	(double i, double j, double k);	///< set shape functions, [N]=[N(i,j,k)]
	void Compute_Bi (double i, double j, double k); ///< set shape derivatives, in param.space
	void Compute_Bx (double i, double j, double k); ///< set shape derivatives, in world space
	void Compute_Be (double i, double j, double k); ///< set Be from Bx, for elasticity problem
	ChMatrix<>* Compute_BEBj (double i, double j, double k); ///< computes the argument of Gauss integral

	void Compute_Kl();					///< computes the Kl (stiffness matrix) of this element

	void Update_A();					///< builds the actual coordsys from the displaced vertex position
};


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif