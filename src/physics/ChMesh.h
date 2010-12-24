#ifndef CHMESH_H
#define CHMESH_H

//////////////////////////////////////////////////
//  
//   ChMesh.h  ***OBSOLETE***
//
//   Class for mesh of finite elements
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


#include <stdlib.h>
#include <math.h>

#include "physics/ChObject.h"
#include "core/ChMath.h"


namespace chrono 
{


#define CHCLASS_FMESH 2

//////////////////////////////////////
//////////////////////////////////////

// CLASS FOR FINITE ELEMENT MESH
//
/// Class which defines a mesh of finite elements of class ChFelem,
/// between nodes of class  ChFnode.  ***OBSOLETE***
///

class ChApi ChFMesh : public ChObj 
{
private:

	int type;		// the type of finite element

	int n_nodes;		// number of nodes
	int n_coords;		// number of degrees of freedom 
	int n_elements;		// number of elements

	int Mnumber;
	int offset;			// offset in state vector

	ChFnode*   node_list;	// linklist of nodes
	ChFelem*	 elem_list;	// linklist of elements

	double E;			// Young Modulus
	double v;			// Poisson ratio
	double G;			// shear modulus
	double l;			// Lame's modulus
	double u;			// rigidity modulus

	double density;		// density
	double volume;		// volume
	double mass;		// total mass

public:

	ChFMesh(int m_type);
	~ChFMesh();

	void Add_node (ChFnode* m_node);
	void Add_element (ChFelem* m_elem);
	void Free_nodes ();
	void Free_elements ();

	void Set_E (double m_E);
	void Set_v (double m_v);
	void Set_G (double m_G);
	void Set_density (double m_density) {density = m_density;}

	double Get_E () {return E;}
	double Get_v () {return v;}
	double Get_G () {return G;}
	double Get_l () {return l;}
	double Get_u () {return u;}
	double Get_density () {return density;}
	double Get_volume  () {return volume;}
	double Get_mass	   () {return mass;}

	int Get_Nnodes () {return n_nodes;}
	int Get_Ncoords () {return n_coords;}
	int Get_Nelements () {return n_elements;}

	void SetOffset (int myOff) {offset = myOff;}
	int  GetOffset () {return offset;}

/*
	void Compute_Kw (ChMatrix<>* m_Kw);		// compute full [K]w stiffness matrix
	void Compute_R	(ChMatrix<>* m_R);		// compute [R] nodal reactions
*/

	void Setup ();				// - X0 = X for all nodes
								// - build all [A0] and [A]
								// - build [E]
								// - build all [Kl] */

	void UpdateTime(double m_time);

	void Update_A ();			// - updates all [A]

		// Updates all the positions and speed of nodes,
		// then updates the local coordsystems [A] of the elements.
	void UpdateALL (ChMatrix<>* state_vector, int X_offset, int Xdt_offset, double m_time);
};



} // END_OF_NAMESPACE____
#endif 
