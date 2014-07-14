//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLINKENGINE_H
#define CHLINKENGINE_H

///////////////////////////////////////////////////
//
//   ChLinkEngine.h
//
//
//   Classes for rotating actuators (motors)
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "physics/ChLinkLock.h"
#include "physics/ChShaft.h"
#include "lcp/ChLcpConstraintTwoGeneric.h"


namespace chrono
{

// Unique link identifier, for detecting type faster than with rtti.
#define LNK_ENGINE		31


///
/// Class for links representing engines between two rigid bodies.
/// Note that the engine can be in 'impose relative rotation' mode,
/// as well as in 'impose speed' etc. It can also be used to represent
/// an engine with a torque/speed custom curve. etc.
///

class ChApi ChLinkEngine : public ChLinkLock {

	CH_RTTI(ChLinkEngine,ChLinkLock);

protected:
	ChFunction* rot_funct;	// rotation(t) function
	ChFunction* spe_funct;	// speed(t) function
	ChFunction* tor_funct;	// torque(t) function
	ChFunction* torque_w;	// torque(w) function

	int	learn;				// if TRUE, the actuator does not apply constraint, just
							// records the motion into its rot_function.

	int impose_reducer;		// if true, speed torque or rotation are imposed to the fast (motor) shaft, before reducer!

	double mot_rot;			// current rotation (read only)
	double mot_rot_dt;		// current ang speed (read only)
	double mot_rot_dtdt;	// current ang acc  (read only)
	double mot_torque;		// current motor torque (read only)
	double mot_rerot;		// current rotation (read only)  before reducer
	double mot_rerot_dt;	// current ang speed (read only) before reducer
	double mot_rerot_dtdt;	// current ang acc  (read only)  before reducer
	double mot_retorque;	// current motor torque (read only) before reducer

	double mot_tau;			// motor: transmission ratio
	double mot_eta;			// motor: transmission efficiency
	double mot_inertia;		// motor: inertia (added to system)

	int eng_mode;			// mode of controlling the motor (by rotation, speed etc.)

	int shaft_mode;			// mode of imposing constraints on extra (non-z) degrees of freedom

	ChFunction* rot_funct_x;	// rotation(t) function for keyframe polar motor
	ChFunction* rot_funct_y;	// rotation(t) function for keyframe polar motor
	double last_r3time;			// internal:for backward differentiation to compute speed in keyframe mode
	double last_r3mot_rot;		// internal:for backward differentiation to compute speed in keyframe mode
	double last_r3mot_rot_dt;	// internal:for backward differentiation to compute speed in keyframe mode
	Quaternion last_r3relm_rot; // internal:for backward differentiation to compute speed in keyframe mode
	Quaternion last_r3relm_rot_dt;	// internal
	Quaternion keyed_polar_rotation;// internal

	ChShaft	innershaft1;		// used in ENG_MODE_TO_POWERTRAIN_SHAFT
	ChShaft	innershaft2;		// ''      ''
	ChLcpConstraintTwoGeneric innerconstraint1; // ''     ''
	ChLcpConstraintTwoGeneric innerconstraint2; // ''     ''
	double cache_li_speed1;
	double cache_li_pos1;
	double torque_react1;
	double cache_li_speed2;
	double cache_li_pos2;
	double torque_react2;

public:
						// builders and destroyers
	ChLinkEngine ();
	virtual ~ChLinkEngine ();
	virtual void Copy(ChLinkEngine* source);
	virtual ChLink* new_Duplicate ();	// always return base link class pointer


							// UPDATING FUNCTIONS - "lin.act. link" custom implementations

							/// Updates motion laws, etc. for the impose rotation / impose speed modes
	virtual void UpdateTime (double mytime);
							/// Updates torque for the impose torque mode
	virtual void UpdateForces (double mytime);
							/// Updates the r3d time, so perform differentiation for computing speed in case of keyframed motion

	virtual void UpdatedExternalTime (double prevtime, double time);

                            /// Sets up the markers associated with the engine link
    virtual void ChLinkEngine::SetUpMarkers(ChMarker* mark1, ChMarker* mark2);

			// data get/set
	ChFunction* Get_rot_funct() {return rot_funct;};
	void Set_rot_funct(ChFunction* m_funct);
	ChFunction* Get_spe_funct() {return spe_funct;};
	void Set_spe_funct(ChFunction* m_funct);
	ChFunction* Get_tor_funct() {return tor_funct;};
	void Set_tor_funct(ChFunction* m_funct);
	ChFunction* Get_torque_w_funct() {return torque_w;};
	void Set_torque_w_funct(ChFunction* m_funct);

	ChFunction* Get_rot_funct_x() {return rot_funct_x;};
	void Set_rot_funct_x(ChFunction* m_funct);
	ChFunction* Get_rot_funct_y() {return rot_funct_y;};
	void Set_rot_funct_y(ChFunction* m_funct);
	Quaternion GetKeyedPolarRotation(){return keyed_polar_rotation;};
	void SetKeyedPolarRotation(Quaternion mq);

	int   Get_learn() {return learn;};
	void  Set_learn(int mset);
	int   Get_impose_reducer() {return impose_reducer;};
	void  Set_impose_reducer(int mset) {impose_reducer = mset;};
	int   Get_eng_mode() {return eng_mode;};
	void  Set_eng_mode(int mset);
        enum eCh_eng_mode {
		ENG_MODE_ROTATION = 0,
		ENG_MODE_SPEED,
		ENG_MODE_TORQUE,
		ENG_MODE_KEY_ROTATION,
		ENG_MODE_KEY_POLAR,
		ENG_MODE_TO_POWERTRAIN_SHAFT
		};
	int   Get_shaft_mode() {return shaft_mode;};
	void  Set_shaft_mode(int mset);
        enum eCh_shaft_mode	{
		ENG_SHAFT_LOCK = 0,		// shafts of motor and user (markers 1 and 2) are stiffly joined 
		ENG_SHAFT_PRISM,		// shafts of motor and user (markers 1 and 2) can shift along shaft (Z axis) 
		ENG_SHAFT_OLDHAM,		// shafts of motor and user (markers 1 and 2) may be parallel shifting on X and Y
		ENG_SHAFT_UNIVERSAL,	// not yet used
		ENG_SHAFT_CARDANO		// not yet used
		};
	double Get_mot_rot() {return mot_rot;}
	double Get_mot_rot_dt() {return mot_rot_dt;}
	double Get_mot_rot_dtdt() {return mot_rot_dtdt;}
	double Get_mot_torque() {return mot_torque;}
	double Get_mot_rerot() {return mot_rerot;}
	double Get_mot_rerot_dt() {return mot_rerot_dt;}
	double Get_mot_rerot_dtdt() {return mot_rerot_dtdt;}
	double Get_mot_retorque() {return mot_retorque;}
	void Set_mot_tau(double mtau) {mot_tau = mtau;}
	double Get_mot_tau() {return mot_tau;}
	void Set_mot_eta(double meta) {mot_eta = meta;}
	double Get_mot_eta() {return mot_eta;}
	void Set_mot_inertia(double min) {mot_inertia = min;}
	double Get_mot_inertia() {return mot_inertia;}

			// Access the inner 1D shaft connected to the rotation of body1 about dir of motor shaft, 
			// if in CH_ENG_MODE_TO_POWERTRAIN_SHAFT. The shaft can be
			// connected to other shafts with ChShaftsClutch or similar items.
	ChShaft* GetInnerShaft1() {return &this->innershaft1;}
			// Access the inner 1D shaft connected to the rotation of body2 about dir of motor shaft, 
			// if in CH_ENG_MODE_TO_POWERTRAIN_SHAFT. The shaft can be
			// connected to other shafts with ChShaftsClutch or similar items.
	ChShaft* GetInnerShaft2() {return &this->innershaft2;}
			// Get the torque between body 1 and inner shaft 1.
			// Note: use only if in CH_ENG_MODE_TO_POWERTRAIN_SHAFT.
	double GetInnerTorque1() {return this->torque_react1;}
			// Get the torque between body 2 and inner shaft 2.
			// Note: use only if in CH_ENG_MODE_TO_POWERTRAIN_SHAFT.
	double GetInnerTorque2() {return this->torque_react2;}

			// Overload LCP system functions of ChPhysicsItem
			// (beyond the base link implementations, it also have to 
			// add the constraint coming from the inner shaft etc.)
	virtual void InjectConstraints(ChLcpSystemDescriptor& mdescriptor);
	virtual void ConstraintsBiReset();
	virtual void ConstraintsBiLoad_C(double factor=1., double recovery_clamp=0.1, bool do_clamp=false);
	virtual void ConstraintsBiLoad_Ct(double factor=1.);
	virtual void ConstraintsLoadJacobians();
	virtual void ConstraintsLiLoadSuggestedSpeedSolution();
	virtual void ConstraintsLiLoadSuggestedPositionSolution();
	virtual void ConstraintsLiFetchSuggestedSpeedSolution();
	virtual void ConstraintsLiFetchSuggestedPositionSolution();
	virtual void ConstraintsFetch_react(double factor=1.);
	virtual void InjectVariables(ChLcpSystemDescriptor& mdescriptor);
	virtual void VariablesFbReset();
	virtual void VariablesFbLoadForces(double factor=1.);
	virtual void VariablesQbLoadSpeed();
	virtual void VariablesFbIncrementMq();
	virtual void VariablesQbSetSpeed(double step=0.);
	virtual void VariablesQbIncrementPosition(double step);



							// STREAMING
	virtual void StreamIN(ChStreamInBinary& mstream);
	virtual void StreamOUT(ChStreamOutBinary& mstream);

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
