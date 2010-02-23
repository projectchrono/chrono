#ifndef CHFEM_H
#define CHFEM_H

//////////////////////////////////////////////////
//
//   ChFem.h
//
//   Finite elements definition with class for
//   mesh of f.e. too.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
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
#include "physics/ChObject.h"


namespace chrono
{


/// Class for a generic finite element node 
/// in 3D space  ***OBSOLETE***

class ChFnode : public ChObj
{
public:
	Vector X;		///< actual position
	Vector X_dt;	///< actual speed
	Vector X0;		///< original position
	double mass;	///< mass

	int Get_dof () {return (3);} // xyz = 3 dof
	void Relax () { X0 = X; X_dt.x=0; X_dt.y=0; X_dt.z=0; }
};



//////////////////////////////////////
//////////////////////////////////////

#define CHCLASS_ChFELEM 20

//////////////////////////////////////
//
// CLASS FOR FINITE ELEMENT

#define FELEM_BRICK_LINEAR		0
#define FELEM_BRICK_QUADRIC		1
#define FELEM_SHELL_LINEAR		2
#define FELEM_SHELL_QUADRIC		3
#define FELEM_ROD				4
#define FELEM_BAR				5
#define FELEM_TETRA_LINEAR		6
#define FELEM_TETRA_QUADRIC		7


/// Base class for a generic finete element, with stiffness
/// matrix, local coordinate system, mass, etc. ***OBSOLETE***

class ChFelem : public ChObj
{
protected:

	ChMatrixDynamic<>* Kl;		// the stiffness matrix in local coords
	ChMatrix33<>* A;		// the actual coordinate system
	ChMatrix33<>* A0;		// the original coordinate system
	double volume;	// the volume
	double mass;	// the mass
	ChFnode** np_list;// the array of pointers to nodes

public:

	ChFelem();
	~ChFelem();


	void SetNode (int index, ChFnode* node);
	ChFnode* GetNode (int index);

	virtual int Get_type () {return 0;}
	virtual int Get_dof () {return 0;}
	virtual int Get_nodes() {return 0;}

	double Get_volume () {return volume;}
	double Get_mass () {return mass;}

	Vector Get_NodeLocX0 (int index);			///< position of node in local coordsys at beginning (last relax)
	Vector Get_NodeLocX (int index);			///< position of node in local coordsys, in this instant.
	Vector Get_NodeLocDisplacement (int index); ///< local displacement {Xl}-{X0l} from beginning, in local csys.


	ChMatrix33<>* Get_A()  {return A;}
	ChMatrix33<>* Get_A0() {return A0;}
	ChMatrixDynamic<>* Get_Kl() {return Kl;}

	virtual ChMatrix<>* Get_E()  =0;
	virtual ChMatrix<>* Get_C()  =0;
	virtual ChMatrix<>* Get_N()  =0;
	virtual ChMatrix<>* Get_Be() =0;
	virtual ChMatrix<>* Get_Dl() =0;
	virtual ChMatrix<>* Get_Dl_dt() =0;
	virtual ChMatrix<>* Get_Fr() =0;



			/// Fills the static vector of local displacements
	void Compute_Dl	();

			/// Fills the static vector of local node speeds
	void Compute_Dl_dt	();


			/// Compute the constitutive matrix
			/// - the matrix will be considered always in _local_ coordinate
	virtual void Compute_Ematr (double E, double v, double G, double l, double u) =0;


			/// Compute the shape function N=N(i,j,k) for given coordinate values

	virtual void Compute_N	(double i, double j, double k) =0;


			/// Compute the shape derivative, i.e. strains "e" are:  {e}=[Be]{d}
			/// where {d} is the vector of displacement, knot by knot

	virtual void Compute_Be	(double i, double j, double k) =0;

			/// Compute  BEBj, the argument of the integral which provides the Kl matrix.
			/// This function is called by the Gauss integrator.

	virtual ChMatrix<>* Compute_BEBj (double i, double j, double k) =0;

			/// Compute stiffness matrix [K]l for given position of nodes.
			/// - Be sure to have set the E matrix
			/// - The matrix is in _local_ coordinates
			/// - Check the element topology: it must have all nodes properly set

	virtual void Compute_Kl() =0;


			/// Compute the strain and stress at given position (at given local parametric coordinates)
			/// - The strain/stresses are in _local_ coordinates, you may want to transform them
			///   into world coords later (see the related functions)
			/// - If pointer to loc_stress is NULL, it won't be computed.
			/// - Use it _after_ you have calculated the displacements of the knots, having
			///   performed some kind of analysis on the system (static or dynamic), with proper Kl.
			/// - the stress matrix is a column-vector (6 rows) with Ox, Oy, Oz, Txy, Tyz, Txz.
			/// - the strain matrix is a column-vector (6 rows) with Ex, Ey, Ez, Yxy, Yyz, Yxz.

	virtual void Compute_LocStrainStress (Vector* par, ChMatrix<>* loc_strain, ChMatrix<>* loc_stress); // -> see cpp


			/// Transformations from local to absolute coords, of 3d stress and 3d strain
	void StressToAbsCoords (ChMatrix<>* loc_stress, ChMatrix<>* abs_stress);
	void StrainToAbsCoords (ChMatrix<>* loc_strain, ChMatrix<>* abs_strain);


			/// Compute the internal reactions "Fr" which should be applied
			/// to the nodes to keep the deformed element in the current shape.
			/// You can imagine the element as a "complex spring" which is applying
	        /// forces  f=-Fr at the nodes.
			/// - The displacement vector must be already computed
			/// - the stiffness matrix Kl must be already computed.
			/// - the forces are in absolute coordinates.

	virtual void Compute_Fr ();


	//virtual void Compute_Kw(Matrix* m_Kw) {};	// project local stiffness into world-coordinates,

	 		/// updates [A] depending on knot positions
	virtual void Update_A() =0;

			/// setup [A0]=[A] at beginning, it "relaxes" the coordsys.
	void Setup_A0();

			/// Relax both nodes and coordsys; {X0}={X}   [A0]=[A]
	void Relax();

};


} // END_OF_NAMESPACE____


#endif






