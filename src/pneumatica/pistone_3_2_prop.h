///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: pistone_3_2_prop.h
//
// Descrizione		: pistone comandato con 2 valvole 3-2 proporzionali
//					  
//					  
//
// Autore			: Hermes Giberti
// Data				: Marzo 2002
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#ifndef _PISTONE_3_2_PROP_H			//*****
#define _PISTONE_3_2_PROP_H			//*****
#ifndef NULL
 #define NULL 0
#endif

#include "pistone.h"
#include "valvola_3_2_prop.h"


namespace chrono 
{
namespace pneumatics
{

/// Specialized pneumatic piston, exploiting two 3-2 proportional
/// valves.


class pistone_3_2_prop : public pistone {

private:
	valvola_3_2_prop *valva, *valvb;
	

public:
	pistone_3_2_prop();
	virtual ~pistone_3_2_prop();
	
	virtual void SetupValvole();

	virtual valvola_3_2_prop *GetValvA() {return valva;};
	virtual valvola_3_2_prop *GetValvB() {return valvb;};
	
	virtual void SetComando(double mycomandoA, double mycomandoB);
	
	virtual void SetStatoPistone();
	
	virtual void SetStatoSist(double *s);
};


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif							//*****

