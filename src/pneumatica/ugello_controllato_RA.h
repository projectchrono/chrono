#ifndef _UGELLO_CONTROLLATO_RA_H			//*****
#define _UGELLO_CONTROLLATO_RA_H			//*****

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: ugello_controllato_RA.h
//
// Descrizione		: RA -> dallo scarico all'utilizzatore
//
//
//
// Autore			: Hermes Giberti
// Data				: Marzo 2002
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////




#include "ugello_controllato_PA.h"


namespace chrono
{
namespace pneumatics
{


/// Class defining pneumatic nozzles (outro).

class ugello_controllato_RA : public ugello_controllato_PA {

protected:

public:
	ugello_controllato_RA() {};
	virtual ~ugello_controllato_RA() {};

	virtual double ConduttanzaIn();
	virtual double ConduttanzaOut();
	virtual double BetaIn();
	virtual double BetaOut();
			// Funzioni che definiscono la conduttanza e il beta

};



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif									//*****
