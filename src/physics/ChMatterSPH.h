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

#ifndef CHMATTERSPH_H
#define CHMATTERSPH_H

//////////////////////////////////////////////////
//
//   ChMatterSPH.h
//
//   Class for clusters of points that can 
//   simulate a fluid or an elastic / plastic
//   solid with the SPH Smooth Particle Hydrodynamics
//   approach, that is with a 'meshless' FEM approach.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <math.h>

#include "physics/ChIndexedNodes.h"
#include "physics/ChNodeXYZ.h"
#include "physics/ChContinuumMaterial.h"
#include "collision/ChCCollisionModel.h"
#include "lcp/ChLcpVariablesNode.h"


namespace chrono
{


// Forward references (for parent hierarchy pointer)

class ChSystem;


/// Class for a single node in the SPH cluster
/// (it does not define mass, inertia and shape becuase those
/// data are shared between them)

class ChApi ChNodeSPH : public ChNodeXYZ  
{
public:
	ChNodeSPH();
	~ChNodeSPH();

	ChNodeSPH (const ChNodeSPH& other); // Copy constructor
	ChNodeSPH& operator= (const ChNodeSPH& other); //Assignment operator

					//
					// FUNCTIONS
					//


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
	
	ChLcpVariablesNode  variables;

	collision::ChCollisionModel*  collision_model;

	ChVector<> UserForce;

	double volume; 
	double density;
	double h_rad;
	double coll_rad;
	double pressure;
};



/// Class for SPH fluid material, with basic property 
/// of uncompressible fluid. 

class ChApi ChContinuumSPH : public fem::ChContinuumMaterial
{
private:

	double viscosity;
	double surface_tension;
	double pressure_stiffness;

public:

			/// Create a continuum isothropic elastoplastic material,
			/// where you can define also plastic and elastic max. stress (yeld limits
			/// for transition elastic->blastic and plastic->fracture).
	ChContinuumSPH(double m_refdensity = 1000, double mviscosity = 0.1, double mtension= 0) 
				: viscosity(mviscosity), surface_tension(mtension), pressure_stiffness(100), ChContinuumMaterial(m_refdensity) {};

	virtual ~ChContinuumSPH() {};
	

			/// Set the viscosity, in [Pa s] units. 
	void   Set_viscosity (double mvisc) {viscosity = mvisc;}
			/// Get the viscosity.
	double Get_viscosity () {return viscosity;}

			/// Set the surface tension coefficient. 
	void   Set_surface_tension (double mten) {surface_tension = mten;}
			/// Get the surface tension coefficient.
	double Get_surface_tension() {return surface_tension;}

			/// Set the pressure stiffness (should be infinite for water
			/// or other almost-incompressible fluids, but too large 
			/// values can cause numerical troubles).
	void   Set_pressure_stiffness (double mst) {pressure_stiffness = mst;}
			/// Set the pressure stiffness.
	double Get_pressure_stiffness() {return pressure_stiffness;}


				/// Method to allow deserializing 
	void StreamIN(ChStreamInBinary& mstream);

				/// Method to allow serializing 
	void StreamOUT(ChStreamOutBinary& mstream);

};




/// Class for clusters of point nodes that can 
/// simulate a fluid or an elastic / plastic
/// solid with the SPH Smooth Particle Hydrodynamics
/// approach, that is with a 'meshless' FEM approach.

class ChApi ChMatterSPH : public ChIndexedNodes
{
						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChMatterSPH,ChIndexedNodes);

private:
			//
	  		// DATA
			//
	
						// The nodes: 
	std::vector< ChSharedPtr<ChNodeSPH> > nodes;				

	ChContinuumSPH material;

	bool do_collide;

public:

			//
	  		// CONSTRUCTORS
			//

				/// Build a cluster of nodes for SPH and meshless FEM.
				/// By default the cluster will contain 0 particles.
	ChMatterSPH ();

				/// Destructor
	~ChMatterSPH ();

				/// Copy from another ChMatterSPH. 
	void Copy(ChMatterSPH* source);


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

				/// Sets the 'fb' part of the encapsulated ChLcpVariablesBody to zero.
	void VariablesFbReset();

				/// Adds the current forces applied to body (including gyroscopic torque) in
				/// encapsulated ChLcpVariablesBody, in the 'fb' part: qf+=forces*factor
	void VariablesFbLoadForces(double factor=1.);

				/// Initialize the 'qb' part of the ChLcpVariablesBody with the 
				/// current value of body speeds. Note: since 'qb' is the unknown of the LCP, this
				/// function seems unuseful, unless used before VariablesFbIncrementMq()
	void VariablesQbLoadSpeed();

				/// Adds M*q (masses multiplied current 'qb') to Fb, ex. if qb is initialized
				/// with v_old using VariablesQbLoadSpeed, this method can be used in 
				/// timestepping schemes that do: M*v_new = M*v_old + forces*dt
	void VariablesFbIncrementMq();

				/// Fetches the body speed (both linear and angular) from the
				/// 'qb' part of the ChLcpVariablesBody (does not updates the full body&markers state)
				/// and sets it as the current body speed.
				/// If 'step' is not 0, also computes the approximate acceleration of
				/// the body using backward differences, that is  accel=(new_speed-old_speed)/step.
				/// Mostly used after the LCP provided the solution in ChLcpVariablesBody .
	void VariablesQbSetSpeed(double step=0.);

				/// Increment body position by the 'qb' part of the ChLcpVariablesBody,
				/// multiplied by a 'step' factor.
				///     pos+=qb*step
				/// If qb is a speed, this behaves like a single step of 1-st order
				/// numerical integration (Eulero integration).
				/// Does not automatically update markers & forces.
	void VariablesQbIncrementPosition(double step);


				/// Tell to a system descriptor that there are variables of type
				/// ChLcpVariables in this object (for further passing it to a LCP solver)
				/// Basically does nothing, but maybe that inherited classes may specialize this.
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
	ChContinuumSPH&  GetMaterial() {return material;}
	
				/// Initialize the fluid as a prismatic region filled with nodes,
				/// initially well ordered as a lattice. This is a helper function
				/// so that you avoid to create all nodes one by one with many calls
				/// to AddNode() .
	void FillBox (const ChVector<> size,	///< x,y,z sizes of the box to fill (better if integer multiples of spacing)
				  const double spacing,		///< the spacing between two near nodes
				  const double initial_density, ///< density of the material inside the box, for initialization of node's masses
				  const ChCoordsys<> cords = CSYSNORM, ///< position and rotation of the box
				  const bool do_centeredcube =true,	///< if false, array is simply cubic, if true is centered cubes (highest regularity)
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




typedef ChSharedPtr<ChMatterSPH> ChSharedMatterSPHPtr;



} // END_OF_NAMESPACE____


#endif
