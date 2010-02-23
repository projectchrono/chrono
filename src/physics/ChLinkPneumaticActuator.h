#ifndef CHLINKPNEUMATICACTUATOR_H
#define CHLINKPNEUMATICACTUATOR_H

///////////////////////////////////////////////////
//
//   ChLinkPneumaticActuator.h
//
//
//   Classes for pneumatic actuators between two
//   bodies.
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
#include "pneumatica/assepneumatico.h"



namespace chrono
{

using namespace chrono::pneumatics;

// Unique link identifier, for detecting type faster than with rtti.
#define LNK_PNEUMATIC	33


///
/// Class for pneumatic linear actuators between two markers,
/// as the piston were joined with two spherical
/// bearing at the origin of the two markers.
///

class ChLinkPneumaticActuator : public ChLinkLock {

	CH_RTTI(ChLinkPneumaticActuator,ChLinkLock);

protected:
							// pointer to internal structure with all pneumatic variables
	AssePneumatico* pneuma;
							// marker distance for zero stroke
	double offset;
							// read-only vars  (updated in UpdateXyz() functions!!)
	double pA;		// pressure chamber A
	double pB;		// pressure chamber B
	double pA_dt;	// d/dt pressure chamber A
	double pB_dt;	// d/dt pressure chamber B
	double pneu_F;	// applied force

	double last_force_time;	// internal

public:
						// builders and destroyers
	ChLinkPneumaticActuator ();
	virtual ~ChLinkPneumaticActuator ();
	virtual void Copy(ChLinkPneumaticActuator* source);
	virtual ChLink* new_Duplicate ();	// always return base link class pointer


							// UPDATING FUNCTIONS - "lin. pneumatic actuator" custom implementations

							// Updates motion laws, marker positions, etc.
	virtual void UpdateTime (double mytime);
							// Updates forces etc.
	virtual void UpdateForces (double mytime);

							// DATA GET/ SET
										// for straight access to all internal pneumatic data
	AssePneumatico* Get_pneuma() {return this->pneuma;};
										// after setting internal pneumatic datas in 'pneuma'
	void SetupActuator()			{pneuma->SetupAssePneumatico();};

										// joints offset for zero length stroke
	double  Get_lin_offset() {return offset;};
	void    Set_lin_offset(double mset);
										// cylinder stroke
	double  Get_pneu_L() {return pneuma->Get_L();};
	void    Set_pneu_L(double mset);


										// state
	double Get_pA() {return pA;}
	double Get_pB() {return pB;}
	double Get_pA_dt() {return pA_dt;}
	double Get_pB_dt() {return pB_dt;}
										// actual force
	double Get_pneu_F() {return pneu_F;}
										// actual position & speed
	double Get_pneu_pos()	 {return relM.pos.x - this->offset;};
	double Get_pneu_pos_dt() {return relM_dt.pos.x;};

										// shortcuts for valve commands
	void Set_ComA(double ma)	{pneuma->Set_ComA(ma);};
	double Get_ComA()			{return pneuma->Get_ComA();};
	void Set_ComB(double ma)	{pneuma->Set_ComB(ma);};
	double Get_ComB()			{return pneuma->Get_ComB();};

	double Get_pneu_R() {return sqrt(pneuma->Get_A() / CH_C_PI);}
	void Set_pneu_R(double mr) { pneuma->Set_A(mr*mr*CH_C_PI); pneuma->SetupAssePneumatico();}


							// STREAMING
	virtual void StreamIN(ChStreamInBinary& mstream);
	virtual void StreamOUT(ChStreamOutBinary& mstream);

};





//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
