#ifndef CHPARTICLESCLONES_H
#define CHPARTICLESCLONES_H

//////////////////////////////////////////////////
//
//   ChParticlesClones.h
//
//   Class for clusters of particle 'clones', that is many
//   rigid objects that share the same shape and mass.
//   This can be used to make granular flows. 
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

#include "physics/ChIndexedParticles.h"
#include "collision/ChCCollisionModel.h"
#include "lcp/ChLcpVariablesBodySharedMass.h"


namespace chrono
{

using namespace collision;



// Forward references (for parent hierarchy pointer)

class ChSystem;


/// Class for a single particle clone in the ChParticlesClones cluster
/// (it does not define mass, inertia and shape becuase those
/// data are _shared_ between them)

class ChApi ChAparticle : public ChParticleBase  
{
public:
	ChAparticle();
	~ChAparticle();

	ChAparticle (const ChAparticle& other); // Copy constructor
	ChAparticle& operator= (const ChAparticle& other); //Assignment operator
	
		// Access the 'LCP variables' of the node
	virtual ChLcpVariables& Variables() {return variables;}


	ChLcpVariablesBodySharedMass	variables;
	ChCollisionModel*				collision_model;
	ChVector<> UserForce;		
	ChVector<> UserTorque;		
};


/// Class for clusters of 'clone' particles, that is many
/// rigid objects with the same shape and mass.
/// This can be used to make granular flows, where
/// you have thousands of objects with the same shape.
/// In fact, a single ChParticlesClones object can
/// be more memory-efficient than many ChBody objects,
/// because they share many features, such as mass and
/// collision shape.
/// If you have N different families of shapes in your 
/// granular simulations (ex. 50% of particles are large
/// spheres, 25% are small spheres and 25% are polihedrons)
/// you can simply add three ChParticlesClones objects to the 
/// ChSystem. This would be more efficient anyway than
/// creating all shapes as ChBody.

class ChApi ChParticlesClones : public ChIndexedParticles 
{
						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChParticlesClones,ChIndexedParticles);

private:
			//
	  		// DATA
			//
	
						// The particles: 
	std::vector<ChAparticle*> particles;				
	
						// Shared mass of particles
	ChSharedMassBody		 particle_mass;

						// Sample collision model
	ChCollisionModel*		 particle_collision_model;


	bool do_collide;
	bool do_limit_speed;
	bool do_sleep;

					// Collective data for surface contact and impact:

	float impactC;		// impact restitution coefficient
	float impactCt;		// tangential impact restitution coefficient
	float k_friction;	// kinematic friction coefficient for surface contact
	float s_friction;	// static friction for rest-contact (sticking)
	float rolling_friction;  // rolling friction 
	float spinning_friction; // spinning friction 

	float max_speed;	// limit on linear speed (useful for VR & videagames)
	float max_wvel;		// limit on angular vel. (useful for VR & videagames)

	float  sleep_time;
	float  sleep_minspeed;
	float  sleep_minwvel;
	float  sleep_starttime;

public:

			//
	  		// CONSTRUCTORS
			//

				/// Build a cluster of particles.
				/// By default the cluster will contain 0 particles.
	ChParticlesClones ();

				/// Destructor
	~ChParticlesClones ();

				/// Copy from another ChParticlesClones. 
	void Copy(ChParticlesClones* source);


			//
	  		// FLAGS
			//


				/// Enable/disable the collision for this cluster of particles.
				/// After setting ON, remember RecomputeCollisionModel()
				/// before anim starts (it is not automatically
				/// recomputed here because of performance issues.)
	void  SetCollide (bool mcoll);
	bool  GetCollide() {return do_collide;}

				/// Trick. Set the maximum linear speed (beyond this limit it will
				/// be clamped). This is useful in virtual reality and real-time
				/// simulations, because it reduces the risk of bad collision detection.
				/// The realism is limited, but the simulation is more stable.
	void SetLimitSpeed    (bool mlimit) { do_limit_speed = mlimit;};
	bool GetLimitSpeed()  {return do_limit_speed;};



			//
	  		// FUNCTIONS
			//

				/// Get the number of particles
	unsigned int GetNparticles() {return particles.size();}

				/// Access the N-th particle 
	ChParticleBase& GetParticle(unsigned int n) { assert(n<particles.size()); return *particles[n];}
				
				/// Resize the particle cluster. Also clear the state of 
				/// previously created particles, if any.
				/// NOTE! Define the sample collision shape using GetCollisionModel()->...
				/// before adding particles!
	void ResizeNparticles(int newsize);

				/// Add a new particle to the particle cluster, passing a 
				/// coordinate system as initial state.
				/// NOTE! Define the sample collision shape using GetCollisionModel()->...
				/// before adding particles!
	void AddParticle(ChCoordsys<double> initial_state = CSYSNORM);



			 // Override/implement LCP system functions of ChPhysicsItem
			 // (to assembly/manage data for LCP system solver)

				/// Sets the 'fb' part of the encapsulated ChLcpVariablesBody to zero.
	void VariablesFbReset();

				/// Adds the current forces applied to body (including gyroscopic torque) in
				/// encapsulated ChLcpVariablesBody, in the 'fb' part: qf+=forces*factor
	void VariablesFbLoadForces(double factor=1.);

				/// Initialize the 'qb' part of the ChLcpVariablesBody with the 
				/// current value of body speeds. Note: since 'qb' is the unknown of the LCP, this
				/// function sems unuseful, however the LCP solver has an option 'add_Mq_to_f', that
				/// takes [M]*qb and add to the 'fb' term before starting (this is often needed in
				/// the Anitescu time stepping method, for instance); this explains the need of this method..
	void VariablesQbLoadSpeed();


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

				/// Acess the collision model for the collision engine: this is the 'sample'
				/// collision model that is used by all particles.
				/// To get a non-null pointer, remember to SetCollide(true), before.
	ChCollisionModel* GetCollisionModel() {return particle_collision_model;}
			
				/// Synchronize coll.models coordinates and bounding boxes to the positions of the particles.
	virtual void SyncCollisionModels();
	virtual void AddCollisionModelsToSystem();
	virtual void RemoveCollisionModelsFromSystem();

				/// After you added collision shapes to the sample coll.model (the one
				/// that you access with GetCollisionModel() ) you need to call this
				/// function so that all collision models of particles will reference the sample coll.model.
	void UpdateParticleCollisionModels();


				/// The normal restitution coefficient, for collisions.
				/// Should be in 0..1 range.
	float  GetImpactC() {return impactC;}
	void   SetImpactC(float mval) {impactC = mval;}

				/// The tangential restitution coefficient, for collisions.
	float  GetImpactCt() {return impactCt;}
	void   SetImpactCt(float mval) {impactCt = mval;}

				/// The kinetic friction coefficient. Usually in 0..1 range, rarely above.
	float  GetKfriction() {return k_friction;}
	void   SetKfriction(float mval) {k_friction = mval;}

				/// The static friction coefficient. Usually a bit higher than kinetic coeff.
	float  GetSfriction() {return s_friction;}
	void   SetSfriction(float mval) {s_friction = mval;}

				/// Set both static friction and kinetic friction at once, with same value
	void  SetFriction(float mval) {SetSfriction(mval); SetKfriction(mval);}

				/// The rolling friction (rolling parameter, it has the dimension of a length). 
				/// Rolling resistant torque is Tr <= (normal force) * (this parameter)
				/// Usually a very low value.
				/// Note! a non-zero value will make the simulation 2x slower! Also, the
				/// GPU solver currently does not support rolling friction. Default: 0.
	float  GetRollingFriction() {return rolling_friction;}
	void   SetRollingFriction(float mval) {rolling_friction = mval;}

				/// The spinning friction (it has the dimension of a length). 
				/// Spinning resistant torque is Ts <= (normal force) * (this parameter)
				/// Usually a very low value. 
				/// Note! a non-zero value will make the simulation 2x slower! Also, the
				/// GPU solver currently does not support spinning friction. Default: 0.
	float  GetSpinningFriction() {return spinning_friction;}
	void   SetSpinningFriction(float mval) {spinning_friction = mval;}

				/// Mass of each particle. Must be positive.
	void   SetMass (double newmass) { if (newmass>0.) this->particle_mass.SetBodyMass(newmass);}
	double GetMass() {return this->particle_mass.GetBodyMass();}

				/// Set the inertia tensor of each particle
	void SetInertia (ChMatrix33<>* newXInertia);
				/// Set the diagonal part of the inertia tensor of each particle
	void SetInertiaXX (Vector iner);
				/// Get the diagonal part of the inertia tensor of each particle
	Vector GetInertiaXX();
				/// Set the extradiagonal part of the inertia tensor of each particle
				/// (xy, yz, zx values, the rest is symmetric)
	void SetInertiaXY (Vector iner);
				/// Get the extradiagonal part of the inertia tensor of each particle
				/// (xy, yz, zx values, the rest is symmetric)
	Vector GetInertiaXY();


				/// Trick. Set the maximum linear speed (beyond this limit it will
				/// be clamped). This is useful in virtual reality and real-time
				/// simulations, because it reduces the risk of bad collision detection.
				/// This speed limit is active only if you set  SetLimitSpeed(true);
	void   SetMaxSpeed(float m_max_speed) {max_speed = m_max_speed;}
	float  GetMaxSpeed () {return max_speed;}

				/// Trick. Set the maximum angualar speed (beyond this limit it will
				/// be clamped). This is useful in virtual reality and real-time
				/// simulations, because it reduces the risk of bad collision detection.
				/// This speed limit is active only if you set  SetLimitSpeed(true);
	void   SetMaxWvel(float m_max_wvel) {max_wvel = m_max_wvel;}
	float  GetMaxWvel () {return max_wvel;}

				/// When this function is called, the speed of particles is clamped
				/// into limits posed by max_speed and max_wvel  - but remember to
				/// put the body in the SetLimitSpeed(true) mode.
	void ClampSpeed();

				/// Set the amount of time which must pass before going automatically in
				/// sleep mode when the body has very small movements.
	void   SetSleepTime(float m_t) {sleep_time = m_t;}
	float GetSleepTime () {return sleep_time;}

				/// Set the max linear speed to be kept for 'sleep_time' before freezing.
	void   SetSleepMinSpeed(float m_t) {sleep_minspeed = m_t;}
	float GetSleepMinSpeed () {return sleep_minspeed;}

				/// Set the max linear speed to be kept for 'sleep_time' before freezing.
	void   SetSleepMinWvel(float m_t) {sleep_minwvel = m_t;}
	float GetSleepMinWvel () {return sleep_minwvel;}




			//
			// UPDATE FUNCTIONS
			//

				/// Update all auxiliary data of the particles 
	virtual void Update (double mytime);
				/// Update all auxiliary data of the particles
	virtual void Update ();

				/// Tells to the associated external object ChExternalObject() ,if any,
				/// that its 3D shape must be updated in order to syncronize to ChBody
				/// coordinates
	void UpdateExternalGeometry ();


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




typedef ChSharedPtr<ChParticlesClones> ChSharedParticlesClonesPtr;



} // END_OF_NAMESPACE____


#endif
