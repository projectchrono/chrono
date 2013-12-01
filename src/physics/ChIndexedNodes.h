//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHINDEXEDNODES_H
#define CHINDEXEDNODES_H

//////////////////////////////////////////////////
//
//   ChIndexedNodes.h
//
//   Interface class for clusters of points that can
//   be accessed with an index.
//   Must be inherited by children classes.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <math.h>

#include "core/ChShared.h"
#include "physics/ChPhysicsItem.h"
#include "lcp/ChLcpVariablesBodyOwnMass.h"


namespace chrono
{

using namespace collision;



// Forward references (for parent hierarchy pointer)

class ChSystem;


/// Class for a node, that has some degrees of 
/// freedom and that contain a proxy to the solver.

class ChApi ChNodeBase : public ChShared  
{
public:
	ChNodeBase ();
	virtual ~ChNodeBase ();

	ChNodeBase (const ChNodeBase& other); // Copy constructor
	ChNodeBase& operator= (const ChNodeBase& other); //Assignment operator

					//
					// FUNCTIONS
					//


			// Access the 'LCP variables' of the node. To be implemented in children classes
	virtual ChLcpVariables& Variables() =0; 

			/// Get the number of degrees of freedom
	int Get_ndof() { return this->Variables().Get_ndof();}


			//
			// Functions for interfacing to the LCP solver
			//

				/// Sets the 'fb' part (the known term) of the encapsulated ChLcpVariables to zero.
	virtual void VariablesFbReset() { Variables().Get_fb().FillElem(0.0); }

				/// Adds the current forces (applied to node) into the
				/// encapsulated ChLcpVariables, in the 'fb' part: qf+=forces*factor
	virtual void VariablesFbLoadForces(double factor=1.) {};

				/// Initialize the 'qb' part of the ChLcpVariables with the 
				/// current value of speeds. 
	virtual void VariablesQbLoadSpeed() {};

				/// Adds M*q (masses multiplied current 'qb') to Fb, ex. if qb is initialized
				/// with v_old using VariablesQbLoadSpeed, this method can be used in 
				/// timestepping schemes that do: M*v_new = M*v_old + forces*dt
	virtual void VariablesFbIncrementMq() {};

				/// Fetches the item speed (ex. linear velocity, in xyz nodes) from the
				/// 'qb' part of the ChLcpVariables and sets it as the current item speed.
				/// If 'step' is not 0, also should compute the approximate acceleration of
				/// the item using backward differences, that is  accel=(new_speed-old_speed)/step.
				/// Mostly used after the LCP provided the solution in ChLcpVariables.
	virtual void VariablesQbSetSpeed(double step=0.) {};

				/// Increment node positions by the 'qb' part of the ChLcpVariables,
				/// multiplied by a 'step' factor.
				///     pos+=qb*step
				/// If qb is a speed, this behaves like a single step of 1-st order
				/// numerical integration (Eulero integration).
	virtual void VariablesQbIncrementPosition(double step) {};


};


/// Class for a single 'point' node, that has 3 DOF degrees of
/// freedom and a mass.

class ChApi ChNodeXYZ : public ChNodeBase 
{
public:
	ChNodeXYZ ();
	virtual ~ChNodeXYZ ();

	ChNodeXYZ (const ChNodeXYZ& other); // Copy constructor
	ChNodeXYZ& operator= (const ChNodeXYZ& other); //Assignment operator

					//
					// FUNCTIONS
					//

			// Position of the node - in absolute csys.
	ChVector<> GetPos() {return pos;}
			// Position of the node - in absolute csys.
	void SetPos(const ChVector<>& mpos) {pos = mpos;}

			// Velocity of the node - in absolute csys.
	ChVector<> GetPos_dt() {return pos_dt;}
			// Velocity of the node - in absolute csys.
	void SetPos_dt(const ChVector<>& mposdt) {pos_dt = mposdt;}

			// Acceleration of the node - in absolute csys.
	ChVector<> GetPos_dtdt() {return pos_dtdt;}
			// Acceleration of the node - in absolute csys.
	void SetPos_dtdt(const ChVector<>& mposdtdt) {pos_dtdt = mposdtdt;}

			// Get mass of the node. To be implemented in children classes
	virtual double GetMass() const = 0;
			// Set mass of the node. To be implemented in children classes
	virtual void SetMass(double mm) = 0;

					//
					// DATA
					// 
	ChVector<> pos;		
	ChVector<> pos_dt;
	ChVector<> pos_dtdt;
};


/// Interface class for clusters of points that can
/// be accessed with an index.
/// Must be inherited by children classes.

class ChApi ChIndexedNodes : public ChPhysicsItem
{
						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChIndexedNodes,ChPhysicsItem);

private:
			//
	  		// DATA
			//

public:

			//
	  		// CONSTRUCTORS
			//

				/// Build a cluster of nodes 
	ChIndexedNodes ();

				/// Destructor
	~ChIndexedNodes ();


			//
	  		// FUNCTIONS
			//

				/// Get the number of nodes
	virtual unsigned int GetNnodes() =0;

				/// Access the N-th node 
	virtual ChSharedPtr<ChNodeBase> GetNode(unsigned int n) =0;

				/// Add a new node to the particle cluster, passing a 
				/// vector as initial position.
//	virtual void AddNode(ChVector<double> initial_state) =0;

				/// Resize the node cluster. Also clear the state of 
				/// previously created particles, if any.
//	virtual void ResizeNnodes(int newsize) =0;

				/// Number of coordinates of the node cluster
//	virtual int GetDOF  ()   {return 3*GetNnodes();} 



			//
			// STREAMING
			//


				/// Method to allow deserializing a persistent binary archive (ex: a file)
				/// into transient data.
	void StreamIN(ChStreamInBinary& mstream);

				/// Method to allow serializing transient data into a persistent
				/// binary archive (ex: a file).
	void StreamOUT(ChStreamOutBinary& mstream);


};




typedef ChSharedPtr<ChIndexedNodes> ChSharedIndexedNodesPtr;



} // END_OF_NAMESPACE____


#endif
