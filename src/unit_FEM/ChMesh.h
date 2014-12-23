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

#ifndef CHMESH_H
#define CHMESH_H


#include <stdlib.h>
#include <math.h>

#include "physics/ChIndexedNodes.h"
#include "physics/ChContinuumMaterial.h"
#include "ChNodeFEMbase.h"
#include "ChElementBase.h"

namespace chrono 
{

/// Namespace with classes for the FEM unit.
namespace fem
{



/// Class which defines a mesh of finite elements of class ChFelem,
/// between nodes of class  ChFnode. 

class ChApiFem ChMesh : public ChIndexedNodes
{
			// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChMesh,ChIndexedNodes);

private:

	std::vector< ChSharedPtr<ChNodeFEMbase> >	 vnodes;	//  nodes
	std::vector< ChSharedPtr<ChElementBase> >	 velements;	//  elements

	unsigned int n_dofs;	// total degrees of freedom
	unsigned int n_dofs_w;	// total degrees of freedom, derivative (Lie algebra)


public:

	ChMesh() { n_dofs = 0; n_dofs_w = 0; };
	~ChMesh() {};

	void AddNode    ( ChSharedPtr<ChNodeFEMbase> m_node);
	void AddElement ( ChSharedPtr<ChElementBase> m_elem);
	void ClearNodes ();
	void ClearElements ();
	
				/// Access the N-th node 
	virtual ChSharedPtr<ChNodeBase>    GetNode(unsigned int n) {return vnodes[n];};
				/// Access the N-th element 
	virtual ChSharedPtr<ChElementBase> GetElement(unsigned int n) {return velements[n];};

	unsigned int GetNnodes () {return (unsigned int) vnodes.size();}
	unsigned int GetNelements () {return (unsigned int) velements.size();}
	virtual  int GetDOF () {return n_dofs;}
	virtual  int GetDOF_w() { return n_dofs_w; }

				/// - Computes the total number of degrees of freedom
				/// - Precompute auxiliary data, such as (local) stiffness matrices Kl, if any, for each element.
	void SetupInitial ();				

				/// Set reference position of nodes as current position, for all nodes.
	void Relax ();

				/// Set no speed and no accelerations in nodes (but does not change reference positions)
    void SetNoSpeedNoAcceleration();

				/// Update time dependent data, for all elements. 
				/// Updates all [A] coord.systems for all (corotational) elements.
	void Update(double m_time);
			

				/// Load tetahedrons from .node and .ele files as saved by TetGen.
				/// The file format for .node (with point# starting from 1) is:
				///   [# of points] [dimension (only 3)] [# of attributes (only 0)] [markers (only 0)]
				///   [node #] [x] [y] [z]
				///   [node #] [x] [y] [z]   etc.
				/// The file format for .ele (with tet# starting from 1) is:
				///   [# of tetahedrons] [dimension (only 4 supported)] [# of attributes (only 0)]
				///   [tet #] [node #] [node #] [node #] [node #]
				///   [tet #] [node #] [node #] [node #] [node #]   etc.
				/// If you pass a material inherited by ChContinuumElastic, nodes with 3D motion are used, and corotational elements.
				/// If you pass a material inherited by ChContinuumPoisson3D, nodes with scalar field are used (ex. thermal, electrostatics, etc)
	void LoadFromTetGenFile(const char* filename_node,  ///< name of the .node file
						    const char* filename_ele,   ///< name of the .ele  file
							ChSharedPtr<ChContinuumMaterial> my_material); ///< material for the created tetahedrons
	
				/// Load tetahedrons, if any, saved in a .inp file for Abaqus.
	void LoadFromAbaqusFile(const char* filename, 
							ChSharedPtr<ChContinuumMaterial> my_material,
							std::vector< std::vector< ChSharedPtr<ChNodeFEMbase> > >& node_sets);

		//
		// STATE FUNCTIONS
		//

				// (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
	virtual void IntStateGather(const unsigned int off_x,	ChState& x,	const unsigned int off_v, ChStateDelta& v,	double& T);	
	virtual void IntStateScatter(const unsigned int off_x,	const ChState& x, const unsigned int off_v,	const ChStateDelta& v,	const double T);
	virtual void IntStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x,	const unsigned int off_v, const ChStateDelta& Dv); 
	virtual void IntLoadResidual_F(const unsigned int off,	ChVectorDynamic<>& R, const double c );
	virtual void IntLoadResidual_Mv(const unsigned int off,	ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c);
	virtual void IntToLCP(const unsigned int off_v,	const ChStateDelta& v, const ChVectorDynamic<>& R, const unsigned int off_L, const ChVectorDynamic<>& L, const ChVectorDynamic<>& Qc);
	virtual void IntFromLCP(const unsigned int off_v, ChStateDelta& v, const unsigned int off_L, ChVectorDynamic<>& L);

			//
			// LCP SYSTEM FUNCTIONS        for interfacing all elements with LCP solver
			//

				/// Tell to a system descriptor that there are items of type
				/// ChLcpKblock in this object (for further passing it to a LCP solver)
				/// Basically does nothing, but maybe that inherited classes may specialize this.
	virtual void InjectKRMmatrices(ChLcpSystemDescriptor& mdescriptor);

				/// Adds the current stiffness K and damping R and mass M matrices in encapsulated
				/// ChLcpKblock item(s), if any. The K, R, M matrices are added with scaling 
				/// values Kfactor, Rfactor, Mfactor.  
	virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor);


				/// Sets the 'fb' part (the known term) of the encapsulated ChLcpVariables to zero.
	virtual void VariablesFbReset();

				/// Adds the current forces (applied to item) into the
				/// encapsulated ChLcpVariables, in the 'fb' part: qf+=forces*factor
	virtual void VariablesFbLoadForces(double factor=1.);

				/// Initialize the 'qb' part of the ChLcpVariables with the 
				/// current value of speeds. Note: since 'qb' is the unknown of the LCP, this
				/// function seems unuseful, unless used before VariablesFbIncrementMq()
	virtual void VariablesQbLoadSpeed();

				/// Adds M*q (masses multiplied current 'qb') to Fb, ex. if qb is initialized
				/// with v_old using VariablesQbLoadSpeed, this method can be used in 
				/// timestepping schemes that do: M*v_new = M*v_old + forces*dt
	virtual void VariablesFbIncrementMq();

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
