#ifndef CHSHAFT_H
#define CHSHAFT_H

//////////////////////////////////////////////////
//
//   ChShaft.h
//
//   Class for one-degree-of-freedom part, that is
//   shafts that can be used to build 1D models
//   of power trains. This is more efficient than 
//   simulating power trains modeled full 3D ChBody
//   objects. 
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

//#include "core/ChShared.h"
#include "physics/ChPhysicsItem.h"
#include "lcp/ChLcpVariablesGeneric.h"



namespace chrono
{

// Forward references (for parent hierarchy pointer)

class ChSystem;


///
///  Class for one-degree-of-freedom mechanical parts with associated
///  inertia (mass, or J moment of intertial for rotating parts). 
///  In most cases these represent shafts that can be used to build 1D models
///  of power trains. This is more efficient than simulating power trains 
///  modeled with full 3D ChBody objects. 
///

class ChShaft : public ChPhysicsItem {

						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChShaft,ChPhysicsItem);

private:
			//
	  		// DATA
			//

	double torque;		// The torque acting on shaft (force, if used as linear DOF)

	double pos;			// angle 
	double pos_dt;		// angular velocity
	double pos_dtdt;	// angular acceleration

	double inertia;		// the J moment of inertia (or mass, if used as linear DOF)

						// used as an interface to the LCP solver.
	ChLcpVariablesGeneric variables;

	float max_speed;	// limit on linear speed (useful for VR & videagames)

	float  sleep_time;
	float  sleep_minspeed;
	float  sleep_minwvel;
	float  sleep_starttime;

	bool fixed;
	bool limitspeed;
	bool sleeping;
	bool use_sleeping;

public:

			//
	  		// CONSTRUCTORS
			//

				/// Build a shaft.
	ChShaft ();
				/// Destructor
	~ChShaft ();

				/// Copy from another ChShaft. 
	void Copy(ChShaft* source);


			//
	  		// FLAGS
			//


				/// Sets the 'fixed' state of the shaft. If true, it does not rotate
				/// despite constraints, forces, etc.
	void SetShaftFixed (bool mev) {this->fixed = mev; variables.SetDisabled(mev);}
	bool GetShaftFixed() {return this->fixed;}
				/// Trick. Set the maximum shaft velocity (beyond this limit it will
				/// be clamped). This is useful in virtual reality and real-time
				/// simulations.
				/// The realism is limited, but the simulation is more stable.
	void SetLimitSpeed    (bool mlimit) { this->limitspeed = mlimit;}
	bool GetLimitSpeed()  {return this->limitspeed;};

				/// Trick. If use sleeping= true, shafts which do not rotate
				/// for too long time will be deactivated, for optimization.
				/// The realism is limited, but the simulation is faster.
	void SetUseSleeping    (bool ms) { this->use_sleeping = ms;}
	bool GetUseSleeping()  {return this->use_sleeping;}

				/// Force the shaft in sleeping mode or not (usually this state change is not
				/// handled by users, anyway, because it is mostly automatic).
	void SetSleeping    (bool ms) { this->sleeping = ms;}
				/// Tell if the shaft is actually in sleeping state.
	bool GetSleeping()  {return this->sleeping;}

				/// Put the shaft in sleeping state if requirements are satisfied.
	bool TrySleeping();

				/// Tell if the body is active, i.e. it is neither fixed to ground nor
				/// it is in sleep mode.
	bool IsActive() {return !(sleeping || fixed);}


			//
	  		// FUNCTIONS
			//

				/// Number of coordinates of the shaft
	virtual int GetDOF  ()   {return 1;}

				/// Returns reference to the encapsulated ChLcpVariables,
	ChLcpVariablesGeneric& Variables() {return variables;}

			 // Override/implement LCP system functions of ChPhysicsItem
			 // (to assembly/manage data for LCP system solver)

				/// Sets the 'fb' part of the encapsulated ChLcpVariables to zero.
	void VariablesFbReset();

				/// Adds the current torques in the 'fb' part: qf+=torques*factor
	void VariablesFbLoadForces(double factor=1.);

				/// Initialize the 'qb' part of the ChLcpVariables with the 
				/// current value of shaft speed. Note: since 'qb' is the unknown of the LCP, this
				/// function sems unuseful, however the LCP solver has an option 'add_Mq_to_f', that
				/// takes [M]*qb and add to the 'fb' term before starting (this is often needed in
				/// the Anitescu time stepping method, for instance); this explains the need of this method..
	void VariablesQbLoadSpeed();

				/// Fetches the shaft speed from the 'qb' part of the ChLcpVariables (does not 
				/// updates the full shaft state) and sets it as the current shaft speed.
				/// If 'step' is not 0, also computes the approximate acceleration of
				/// the shaft using backward differences, that is  accel=(new_speed-old_speed)/step.
	void VariablesQbSetSpeed(double step=0.);

				/// Increment shaft position by the 'qb' part of the ChLcpVariables,
				/// multiplied by a 'step' factor.
				///     pos+=qb*step
	void VariablesQbIncrementPosition(double step);


				/// Tell to a system descriptor that there are variables of type
				/// ChLcpVariables in this object (for further passing it to a LCP solver)
	virtual void InjectVariables(ChLcpSystemDescriptor& mdescriptor);




			   // Other functions

				/// Set no speed and no accelerations (but does not change the position)
	void SetNoSpeedNoAcceleration();

				/// Set the torque applied to the shaft 
	void   SetAppliedTorque(double mtorque) { this->torque = mtorque;}
				/// Get the torque applied to the shaft 
	double GetAppliedTorque() {return torque;}

				/// Set the angular position 
	void   SetPos(double mp) { this->pos = mp;}
				/// Get the angular position 
	double GetPos() {return this->pos;}
	
				/// Set the angular velocity 
	void   SetPos_dt(double mp) { this->pos_dt = mp;}
		 		/// Get the angular velocity 
	double GetPos_dt() {return this->pos_dt;}

				/// Set the angular acceleration 
	void   SetPos_dtdt(double mp) { this->pos_dtdt = mp;}
				/// Get the angular acceleration
	double GetPos_dtdt() {return this->pos_dtdt;}

				/// Inertia of the shaft. Must be positive.
				/// Try not to mix bodies with too high/too low values of mass, for numerical stability.
	void   SetInertia (double newJ);
	double GetInertia() {return this->inertia;}



				/// Trick. Set the maximum velocity (beyond this limit it will
				/// be clamped). This is useful in virtual reality and real-time
				/// simulations, to increase robustness at the cost of realism.
				/// This limit is active only if you set  SetLimitSpeed(true);
	void   SetMaxSpeed(float m_max_speed) {max_speed = m_max_speed;}
	float  GetMaxSpeed () {return max_speed;}

				/// When this function is called, the speed of the shaft is clamped
				/// into limits posed by max_speed and max_wvel  - but remember to
				/// put the shaft in the SetLimitSpeed(true) mode.
	void   ClampSpeed();

				/// Set the amount of time which must pass before going automatically in
				/// sleep mode when the shaft has very small movements.
	void   SetSleepTime(float m_t) {sleep_time = m_t;}
	float  GetSleepTime () {return sleep_time;}

				/// Set the max linear speed to be kept for 'sleep_time' before freezing.
	void   SetSleepMinSpeed(float m_t) {sleep_minspeed = m_t;}
	float  GetSleepMinSpeed () {return sleep_minspeed;}

				/// Set the max linear speed to be kept for 'sleep_time' before freezing.
	void   SetSleepMinWvel(float m_t) {sleep_minwvel = m_t;}
	float  GetSleepMinWvel () {return sleep_minwvel;}

	


			//
			// UPDATE FUNCTIONS
			//

				/// Update all auxiliary data of the shaft at given time
	virtual void Update (double mytime);


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



typedef ChSharedPtr<ChShaft> ChSharedShaftPtr;



} // END_OF_NAMESPACE____


#endif
