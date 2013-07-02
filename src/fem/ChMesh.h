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
#include "fem/ChNodeFEMbase.h"
#include "fem/ChElementBase.h"

namespace chrono 
{
namespace fem
{



/// Class which defines a mesh of finite elements of class ChFelem,
/// between nodes of class  ChFnode. 

class ChApi ChMesh : public ChIndexedNodes
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
	virtual ChNodeBase* GetNode(unsigned int n) {return vnodes[n];};
				/// Access the N-th element 
	virtual ChElementBase* GetElement(unsigned int n) {return velements[n];};

	unsigned int GetNnodes () {return vnodes.size();}
	unsigned int GetNelements () {return velements.size();}
	virtual  int GetDOF () {return n_dofs;}

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



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif 
