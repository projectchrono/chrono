#ifndef _SIS_ATTUATORE_3_2_PROP_H			//*****
#define _SIS_ATTUATORE_3_2_PROP_H			//*****

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: sis_attuatore_3_2_prop.h
//
// Descrizione		: 
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

#include "pistone_3_2_prop.h"
#include "sistema.h"


namespace chrono 
{
namespace pneumatics
{

/// Pneumatic actuator system, with two 3-2 proportional valves.

class sis_attuatore_3_2_prop : public sistema {

public:
	pistone_3_2_prop *mypistone;
	
	
	//controllore *mycontrollore;
	//gen_funzioni *myfunzione;

public:
	sis_attuatore_3_2_prop();	//{mypistone = NULL; mypistone = new pistone_3_2();};
	virtual ~sis_attuatore_3_2_prop();	// {if(mypistone) delete mypistone; mypistone = NULL;};

	virtual void InizializzaSistema();
	
	virtual void Salva(int myi);
	
	virtual void ScriviFile();

	virtual void SetStato();	// Qui agirà il controllore?????

	virtual void SetDerivataStato();

	virtual void Comanda(double *com);

	virtual void derivate(double *s, double *ds); // metodo che serve a RK4


};



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif									//*****

