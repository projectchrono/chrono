#ifndef CHMESH_H
#define CHMESH_H

//////////////////////////////////////////////////
//  
//   ChMesh.h 
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

#include "physics/ChPhysicsItem.h"
#include "fem/ChContinuumMaterial.h"


namespace chrono 
{
namespace fem
{



/// Class which defines a mesh of finite elements of class ChFelem,
/// between nodes of class  ChFnode.  ***NEW, EXPERIMENTAL ***

class ChApi ChMesh : //public ChPhysicsItem , 
					 public ChIndexedNodes
{
private:

	std::vector<ChNodeFEMbase*>	 vnodes;	//  nodes
	std::vector<ChElementBase*>	 velements;	//  elements

	unsigned int n_dofs; // total degrees of freedom
	//double volume;	 // total volume
	//double mass;		 // total mass

public:

	ChMesh() { n_dofs = 0;};
	~ChMesh() {};

	void AddNode (ChNodeFEMbase& m_node);
	void AddElement (ChElementBase& m_elem);
	void ClearNodes ();
	void ClearElements ();
	
				/// Access the N-th node 
	virtual ChNodeBase& GetNode(unsigned int n) {return *((ChNodeBase*)vnodes[n]);};
				/// Access the N-th element 
	virtual ChElementBase& GetElement(unsigned int n) {return *velements[n];};

	unsigned int GetNnodes () {return vnodes.size();}
	unsigned int GetNelements () {return velements.size();}
	unsigned int GetNdof () {return n_dofs;}

				/// - Computes the total number of degrees of freedom
				/// - Precompute auxiliary data, such as (local) stiffness matrices Kl, if any, etc
	void Setup ();				

				/// Set reference position of nodes as current position, for all nodes.
	void Relax ();			

				/// Update time dependent data, for all elements. 
	void UpdateTime(double m_time);

				/// Updates all [A] coord.systems for all (corotational) elements.
				/// Not needed? Can be done directly when calling KmatricesLoad() ...
	//void UpdateRotation ();			


			//
			// LCP SYSTEM FUNCTIONS        for interfacing all elements with LCP solver
			//

				/// Tell to a system descriptor that there are items of type
				/// ChLcpKstiffness in this object (for further passing it to a LCP solver)
				/// Basically does nothing, but maybe that inherited classes may specialize this.
	virtual void InjectKmatrices(ChLcpSystemDescriptor& mdescriptor);

				/// Adds the current stiffness K and damping R matrices in encapsulated
				/// ChLcpKstiffness item(s), if any. The K and R matrices are load with scaling 
				/// values Kfactor and Rfactor. 
	virtual void KmatricesLoad(double Kfactor, double Rfactor);


				/// Sets the 'fb' part (the known term) of the encapsulated ChLcpVariables to zero.
	virtual void VariablesFbReset();

				/// Adds the current forces (applied to item) into the
				/// encapsulated ChLcpVariables, in the 'fb' part: qf+=forces*factor
	virtual void VariablesFbLoadForces(double factor=1.);

				/// Initialize the 'qb' part of the ChLcpVariables with the 
				/// current value of speeds. Note: since 'qb' is the unknown of the LCP, this
				/// function sems unuseful, however the LCP solver has an option 'add_Mq_to_f', that
				/// takes [M]*qb and add to the 'fb' term before starting (this is often needed in
				/// the Anitescu time stepping method, for instance); this explains the need of this method..
	virtual void VariablesQbLoadSpeed();

				/// Fetches the item speed (ex. linear and angular vel.in rigid bodies) from the
				/// 'qb' part of the ChLcpVariables and sets it as the current item speed.
				/// If 'step' is not 0, also should compute the approximate acceleration of
				/// the item using backward differences, that is  accel=(new_speed-old_speed)/step.
				/// Mostly used after the LCP provided the solution in ChLcpVariables.
	virtual void VariablesQbSetSpeed(double step=0.);

				/// Increment item positions by the 'qb' part of the ChLcpVariables,
				/// multiplied by a 'step' factor.
				///     pos+=qb*step
				/// If qb is a speed, this behaves like a single step of 1-st order
				/// numerical integration (Eulero integration).
	virtual void VariablesQbIncrementPosition(double step);

				/// Tell to a system descriptor that there are variables of type
				/// ChLcpVariables in this object (for further passing it to a LCP solver)
				/// Basically does nothing, but maybe that inherited classes may specialize this.
	virtual void InjectVariables(ChLcpSystemDescriptor& mdescriptor);


};






//////////////////////////////////////
//////////////////////////////////////  OBSOLETE STUFF!!!!   OBSOLETE STUFF!!!!   OBSOLETE STUFF!!!!   OBSOLETE STUFF!!!!   OBSOLETE STUFF!!!! 


// CLASS FOR FINITE ELEMENT MESH   ***OBSOLETE***
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

	ChFnode*	 node_list;	// linklist of nodes
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
} // END_OF_NAMESPACE____

#endif 
