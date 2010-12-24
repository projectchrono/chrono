#ifndef CHLINKLINACTUATOR_H
#define CHLINKLINACTUATOR_H

///////////////////////////////////////////////////
//
//   ChLinkLinActuator.h
//
//
//   Classes for linear actuators.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "physics/ChLinkLock.h"


namespace chrono
{

// Unique link identifier, for detecting type faster than with rtti.
#define LNK_LINACTUATOR	27


///
/// Class for linear actuators between two markers,
/// as the actuator were joined with two spherical
/// bearing at the origin of the two markers.
///

class ChApi ChLinkLinActuator : public ChLinkLock {

	CH_RTTI(ChLinkLinActuator,ChLinkLock);

protected:
	ChFunction* dist_funct;	// distance function
	int	learn;				// if TRUE, the actuator does not apply constraint, just
							// records the motion into its dist_function.
	double offset;			// distance offset

	double mot_tau;			// motor: transmission ratio
	double mot_eta;			// motor: transmission efficiency
	double mot_inertia;		// motor: inertia (added to system)
	ChFunction* mot_torque;	// motor: recorder of torque
	ChFunction* mot_rot;	// motor: recorder of motor rotation

	double mot_rerot;		// current rotation (read only)  before reducer
	double mot_rerot_dt;	// current ang speed (read only) before reducer
	double mot_rerot_dtdt;	// current ang acc  (read only)  before reducer
	double mot_retorque;	// current motor torque (read only) before reducer

public:
						// builders and destroyers
	ChLinkLinActuator ();
	virtual ~ChLinkLinActuator ();
	virtual void Copy(ChLinkLinActuator* source);
	virtual ChLink* new_Duplicate ();	// always return base link class pointer


							// UPDATING FUNCTIONS - "lin.act. link" custom implementations

							// Updates motion laws, marker positions, etc.
	virtual void UpdateTime (double mytime);

			// data get/set
	ChFunction* Get_dist_funct() {return dist_funct;};
	void Set_dist_funct(ChFunction* m_funct);
	int   Get_learn() {return learn;};
	void  Set_learn(int mset);
	double  Get_lin_offset() {return offset;};
	void  Set_lin_offset(double mset) {offset = mset;}

	void Set_mot_tau(double mtau) {mot_tau = mtau;}
	double Get_mot_tau() {return mot_tau;}
	void Set_mot_eta(double meta) {mot_eta = meta;}
	double Get_mot_eta() {return mot_eta;}
	void Set_mot_inertia(double min) {mot_inertia = min;}
	double Get_mot_inertia() {return mot_inertia;}
	ChFunction* Get_motrot_funct() {return mot_rot;};
	void Set_motrot_funct(ChFunction* m_funct);
	ChFunction* Get_mottorque_funct() {return mot_torque;};
	void Set_mottorque_funct(ChFunction* m_funct);

			// easy fetching of motor-reduced moments or angle-speed-accel.
	double Get_mot_rerot() {return mot_rerot;}
	double Get_mot_rerot_dt() {return mot_rerot_dt;}
	double Get_mot_rerot_dtdt() {return mot_rerot_dtdt;}
	double Get_mot_retorque() {return mot_retorque;}


							// STREAMING
	virtual void StreamIN(ChStreamInBinary& mstream);
	virtual void StreamOUT(ChStreamOutBinary& mstream);

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
