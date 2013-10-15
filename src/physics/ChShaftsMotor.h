//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHSHAFTSMOTOR_H
#define CHSHAFTSMOTOR_H

//////////////////////////////////////////////////
//
//   ChShaftsMotor.h
//
//   Class for defining a motor (a torque) between
//   two one-degree-of-freedom parts, that is,
//   shafts that can be used to build 1D models
//   of power trains. This is more efficient than 
//   simulating power trains modeled full 3D ChBody
//   objects. 
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChShaftsCouple.h"
#include "lcp/ChLcpConstraintTwoGeneric.h"



namespace chrono
{

///  Class for defining a 'transmission ratio' (a 1D gear) 
///  between two one-degree-of-freedom parts, that is,
///  shafts that can be used to build 1D models
///  of power trains. This is more efficient than 
///  simulating power trains modeled with full 3D ChBody
///  objects. 

class ChApi ChShaftsMotor : public ChShaftsCouple {

						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChShaftsMotor,ChShaftsCouple);

private:
			//
	  		// DATA
			//

	double motor_torque;	

	double motor_set_rot;
	double motor_set_rot_dt;

	double torque_react1;					
	double torque_react2;

						// used as an interface to the LCP solver.
	ChLcpConstraintTwoGeneric constraint;
	float cache_li_speed;	// used to cache the last computed value of multiplier (solver warm starting)
	float cache_li_pos;		// used to cache the last computed value of multiplier (solver warm starting)	

public:

			//
	  		// CONSTRUCTORS
			//

				/// Constructor.
	ChShaftsMotor ();
				/// Destructor
	~ChShaftsMotor ();

				/// Copy from another ChShaftsMotor. 
	void Copy(ChShaftsMotor* source);


			//
	  		// FLAGS
			//

			//
	  		// FUNCTIONS
			//

				/// Number of scalar constraints 
	virtual int GetDOC_c  () {return 0;}


			// Override/implement LCP system functions of ChShaftsCouple
			// (to assembly/manage data for LCP system solver

	virtual void InjectConstraints(ChLcpSystemDescriptor& mdescriptor);
	virtual void ConstraintsBiReset();
	virtual void ConstraintsBiLoad_C(double factor=1., double recovery_clamp=0.1, bool do_clamp=false);
	virtual void ConstraintsBiLoad_Ct(double factor=1.);
	virtual void ConstraintsFbLoadForces(double factor=1.);
	virtual void ConstraintsLoadJacobians();
	virtual void ConstraintsLiLoadSuggestedSpeedSolution();
	virtual void ConstraintsLiLoadSuggestedPositionSolution();
	virtual void ConstraintsLiFetchSuggestedSpeedSolution();
	virtual void ConstraintsLiFetchSuggestedPositionSolution();
	virtual void ConstraintsFetch_react(double factor=1.);


			   // Other functions

				/// Use this function after gear creation, to initialize it, given  
				/// two shafts to join. 
				/// Each shaft must belong to the same ChSystem. 
	virtual int Initialize(ChSharedPtr<ChShaft>& mshaft1, ///< first  shaft to join
						   ChSharedPtr<ChShaft>& mshaft2  ///< second shaft to join 
						   );


	enum eCh_shaftsmotor_mode {
		MOT_MODE_ROTATION = 0,
		MOT_MODE_SPEED,
		MOT_MODE_TORQUE
		} motor_mode;

				/// Se the motor mode. The options are that you impose
				/// the relative torque between the two shafts, 
				/// or their relative rotation phase, or
				/// their relative speed, but one mode excludes the others. 
	void SetMotorMode(eCh_shaftsmotor_mode mmode) {motor_mode = mmode;}

				/// Set the motor torque applied between shaft2 and shaft1. 
				/// So, if fixed, shaft1 can be considered the reference, or the 'truss'.
				/// (The torque is applied with opposite sign to shaft 1).
				/// Note: use this only when in MOT_MODE_TORQUE !!
	void   SetMotorTorque(double mt) { assert(motor_mode == MOT_MODE_TORQUE); this->motor_torque = mt;}

				/// Get the motor torque applied between shaft2 and shaft1. 
	double GetMotorTorque() { return this->motor_torque;}

				/// Get the reaction torque exchanged between the two shafts,
				/// considered as applied to the 1st axis.
	double GetTorqueReactionOn1() {return  (GetMotorTorque());}

				/// Get the reaction torque exchanged between the two shafts,
				/// considered as applied to the 2nd axis.
	double GetTorqueReactionOn2() {return -(GetMotorTorque());}

				/// Set the motor rotation phase between shaft2 and shaft1. 
				/// If the rotation is not constant, you also must use SetMotorRot_dt()
				/// Note: use this only when in MOT_MODE_ROTATION !
	void   SetMotorRot(double mt) 
					{   assert(motor_mode == MOT_MODE_ROTATION); 
						this->motor_set_rot = mt; }

				/// Set the motor rotation speed between shaft2 and shaft1.
				/// Note: use this only when in MOT_MODE_ROTATION or MOT_MODE_SPEED !
	void   SetMotorRot_dt(double mt) 
					{   assert((motor_mode == MOT_MODE_ROTATION)||(motor_mode == MOT_MODE_SPEED));
						this->motor_set_rot_dt = mt; }

				/// Get the actual angle rotation of the motor, in terms of phase of shaft 1 respect to 2.
	double GetMotorRot() {return (this->shaft1->GetPos() - this->shaft2->GetPos());}
				/// Get the actual speed of the motor, in terms of speed of shaft 1 respect to 2.
	double GetMotorRot_dt() {return (this->shaft1->GetPos_dt() - this->shaft2->GetPos_dt());}
				/// Get the actual acceleration of the motor, in terms of accel. of shaft 1 respect to 2.
	double GetMotorRot_dtdt() {return (this->shaft1->GetPos_dtdt() - this->shaft2->GetPos_dtdt());}

			//
			// UPDATE FUNCTIONS
			//

				/// Update all auxiliary data of the gear transmission at given time
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



typedef ChSharedPtr<ChShaftsMotor> ChSharedShaftsMotorPtr;



} // END_OF_NAMESPACE____


#endif
