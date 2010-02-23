#ifndef _PISTONE_3_2_H			//*****
#define _PISTONE_3_2_H			//*****

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: pistone_3_2.h
//
// Descrizione		: pistone comandato con 2 valvole 3-2 
//					  
//					  
//
// Autore			: Hermes Giberti
// Data				: Marzo 2002
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


#ifndef NULL
 #define NULL 0
#endif

#include "pistone.h"
#include "valvola_3_2.h"


namespace chrono 
{
namespace pneumatics
{


/// A specialized pneumatic piston actuator, with two 3-2 valves

class pistone_3_2 : public pistone {

public:
	valvola_3_2 *valva, *valvb;
	

public:
	pistone_3_2();
	virtual ~pistone_3_2();
	
	virtual void SetupValvole();

	virtual valvola_3_2 *GetValvA() {return valva;};
	virtual valvola_3_2 *GetValvB() {return valvb;};
	
	virtual void SetStatoPistone();
	
	virtual void SetComando(double mycomandoA, double mycomandoB);
	
	virtual void SetStatoSist(double *s);


};


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____




#endif							//*****

