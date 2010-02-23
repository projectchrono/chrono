#ifndef _UGELLO_CONTROLLATO_PA_H			//*****
#define _UGELLO_CONTROLLATO_PA_H			//*****

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: ugello_controllato_PA.h
//
// Descrizione		: PA -> dal serbatoio all'utilizzatore
//
//
//
// Autore			: Hermes Giberti
// Data				: Marzo 2002
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////



#include "ugello_controllato.h"


namespace chrono
{
namespace pneumatics
{


/// Class defining pneumatic nozzles (into).


class ugello_controllato_PA : public ugello_controllato {

protected:
	double comando;
			// comando fornito all'ugello

public:
	ugello_controllato_PA() {comando = 0;};
	virtual ~ugello_controllato_PA() {};

	virtual void SetComando(double mycomando) {comando = mycomando;};
	virtual double GetComando() {return comando;};
			// Assegna il comando all'ugello

	virtual double ConduttanzaIn();
	virtual double ConduttanzaOut();
	virtual double BetaIn();
	virtual double BetaOut();
			// Funzioni che definiscono la conduttanza e il beta

};


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____



#endif									//*****
