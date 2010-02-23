#ifndef _UGELLO_CONTROLLATO_H			//*****
#define _UGELLO_CONTROLLATO_H			//*****

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: ugello_controllato.h
//
// Descrizione		:
//
//
//
// Autore			: Hermes Giberti
// Data				: Marzo 2002
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


#include "ugello.h"



namespace chrono
{
namespace pneumatics
{


/// Class defining controlled pneumatic nozzles.

class ugello_controllato : public ugello {

protected:
	double comando;
			// comando fornito all'ugello

public:
	ugello_controllato() {comando = 0;};
	virtual ~ugello_controllato() {};

	virtual void SetComando(double mycomando) {comando = mycomando;};
	virtual double GetComando() {return comando;};
			// Assegna il comando all'ugello

	virtual double ConduttanzaIn();
	virtual double ConduttanzaOut();
	virtual double BetaIn();
	virtual double BetaOut();
			// Funzioni che definiscono la conduttanza e il beta

	virtual double GetConduttanza();
			// Decide in funzione del verso quale C usare
	virtual double GetBeta();
			// Decide in funzione del verso quale b usare

};



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____



#endif									//*****
