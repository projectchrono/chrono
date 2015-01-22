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
// File authors: Alessandro Tasora

#ifndef CHMATTERMESHLESS_H
#define CHMATTERMESHLESS_H

//////////////////////////////////////////////////
//
//   ChMatterMeshless.h
//
//   Class for clusters of nodes that can 
//   simulate a visco-elasto-plastic deformable solid 
//   using the approach in Mueller ("Point based.." paper)
//   that is with a 'meshless' FEM approach.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <math.h>

#include "ChApiFEM.h"
#include "physics/ChIndexedNodes.h"
#include "physics/ChNodeXYZ.h"
#include "collision/ChCCollisionModel.h"
#include "lcp/ChLcpVariablesNode.h"
#include "physics/ChContinuumMaterial.h"

namespace chrono
{

// Forward references (for parent hierarchy pointer)
class ChSystem;

namespace fem
{

using namespace collision;






/// Class for a single node in the meshless FEM  cluster

class ChApiFem ChNodeMeshless : public ChNodeXYZ  
{
public:
	ChNodeMeshless();
	~ChNodeMeshless();

	ChNodeMeshless (const ChNodeMeshless& other); // Copy constructor
	ChNodeMeshless& operator= (const ChNodeMeshless& other); //Assignment operator

					//
					// FUNCTIONS
					//

			// Reference (initial) position of the node - in absolute csys.
			// Note that the simulation algorithm might reset this once in a while, exp. for highly plastic objects.
	ChVector<> GetPosReference() {return pos_ref;}
			// Reference (initial) position of the node - in absolute csys. 
	void SetPosReference(const ChVector<>& mpos) {pos_ref = mpos;}

			// Get the kernel radius (max. radius while checking surrounding particles)
	double GetKernelRadius() {return h_rad;}
	void SetKernelRadius(double mr);

			// Set collision radius (for colliding with bodies, boundaries, etc.)
	double GetCollisionRadius() {return coll_rad;}
	void SetCollisionRadius(double mr);

			// Set the mass of the node
	void SetMass(double mmass) {this->variables.SetNodeMass(mmass);}
			// Get the mass of the node
	double GetMass() const {return variables.GetNodeMass();}

			// Access the 'LCP variables' of the node
	ChLcpVariables& Variables() {return variables;}

					//
					// DATA
					// 
	ChVector<> pos_ref; 
	
	ChMatrix33<> Amoment;
	ChMatrix33<> J;
	ChMatrix33<> FA;

	ChStrainTensor<> t_strain; //
	ChStrainTensor<> p_strain; // plastic strain
	ChStrainTensor<> e_strain; // elastic strain
	ChStressTensor<> e_stress; // stress

	ChLcpVariablesNode	variables;
	ChCollisionModel*	collision_model;

	ChVector<> UserForce;		

	double volume; 
	double density;
	double h_rad;
	double coll_rad;
	double hardening;
};



/// Class for clusters of nodes that can 
/// simulate a visco-elasto-plastic deformable solid 
/// using the approach in Mueller ("Point based.." 2004 paper)
/// that is with a 'meshless' FEM approach.

class ChApiFem ChMatterMeshless : public ChIndexedNodes
{
						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChMatterMeshless,ChIndexedNodes);

private:
			//
	  		// DATA
			//
	
						// The nodes: 
	std::vector< ChSharedPtr<ChNodeMeshless> > nodes;				

	//ChContinuumPlasticVonMises material;
	ChSharedPtr<ChContinuumElastoplastic> material; //* ChContinuumDruckerPrager material; //***TEST***

	double viscosity;

	bool do_collide;

public:

			//
	  		// CONSTRUCTORS
			//

				/// Build a cluster of nodes for Meshless and meshless FEM.
				/// By default the cluster will contain 0 particles.
	ChMatterMeshless ();

				/// Destructor
	~ChMatterMeshless ();

				/// Copy from another ChMatterMeshless. 
	void Copy(ChMatterMeshless* source);


			//
	  		// FLAGS
			//


				/// Enable/disable the collision for this cluster of particles.
				/// After setting ON, remember RecomputeCollisionModel()
				/// before anim starts (it is not automatically
				/// recomputed here because of performance issues.)
	void  SetCollide (bool mcoll);
	bool  GetCollide() {return do_collide;}

			// STATISTICS  - override these in child classes if needed
			// 

				/// Get the number of scalar coordinates (variables), if any, in this item 
	virtual int GetDOF  ()   {return 3*this->GetNnodes();}

			//
	  		// FUNCTIONS
			//

				/// Get the number of nodes
	unsigned int GetNnodes() {return (unsigned int) nodes.size();}

				/// Access the N-th node 
	ChSharedPtr<ChNodeBase> GetNode(unsigned int n) { assert(n<nodes.size()); return nodes[n];}
				
				/// Resize the node cluster. Also clear the state of 
				/// previously created particles, if any.
	void ResizeNnodes(int newsize);

				/// Add a new node to the particle cluster, passing a 
				/// vector as initial position.
	void AddNode(ChVector<double> initial_state);



		//
		// LCP INTERFACE
		//

			 // Override/implement LCP system functions of ChPhysicsItem
			 // (to assembly/manage data for LCP system solver)

	void VariablesFbReset();

	void VariablesFbLoadForces(double factor=1.);

	void VariablesQbLoadSpeed();

	void VariablesFbIncrementMq();

	void VariablesQbSetSpeed(double step=0.);

	void VariablesQbIncrementPosition(double step);

	virtual void InjectVariables(ChLcpSystemDescriptor& mdescriptor);



			   // Other functions

				/// Set no speed and no accelerations (but does not change the position)
	void SetNoSpeedNoAcceleration();

			
				/// Synchronize coll.models coordinates and bounding boxes to the positions of the particles.
	virtual void SyncCollisionModels();
	virtual void AddCollisionModelsToSystem();
	virtual void RemoveCollisionModelsFromSystem();

	void UpdateParticleCollisionModels();


				/// Access the material
	ChSharedPtr<ChContinuumElastoplastic>&  GetMaterial() {return material;}
				/// Change the default material (by default it is a ChContinuumPlasticVonMises )
				/// with a new one, that you create and handle with smart pointer, so you do not have to worry about deletion.
	void ReplaceMaterial(ChSharedPtr<ChContinuumElastoplastic> newmaterial);

				/// Set the Newtonian viscosity of the material
	void SetViscosity(double mvisc) { viscosity=mvisc;}
				/// Get the Newtonian viscosity of the material
	double GetViscosity() {return viscosity;}

				/// Initialize the material as a prismatic region filled with nodes,
				/// initially well ordered as a lattice. This is a helper function
				/// so that you avoid to create all nodes one by one with many calls
				/// to AddNode() .
	void FillBox (const ChVector<> size,	///< x,y,z sizes of the box to fill (better if integer multiples of spacing)
				  const double spacing,		///< the spacing between two near nodes
				  const double initial_density, ///< density of the material inside the box, for initialization of node's masses
				  const ChCoordsys<> cords = CSYSNORM, ///< position and rotation of the box
				  const bool do_centeredcube =false,   ///< if false, array is simply cubic, if true is centered cubes (highest regularity)
				  const double kernel_sfactor =2.2,  ///< the radius of kernel of the particle is 'spacing' multiplied this value
				  const double randomness = 0.0	///< randomness of the initial distribution lattice, 0...1
				  );


			//
			// UPDATE FUNCTIONS
			//

				/// Update all auxiliary data of the particles 
	virtual void Update (double mytime);
				/// Update all auxiliary data of the particles
	virtual void Update ();


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



// A shortcut..
typedef ChSharedPtr<ChMatterMeshless> ChSharedMatterMeshlessPtr;



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif
