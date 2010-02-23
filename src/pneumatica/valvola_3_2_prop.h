#ifndef _VALVOLA_3_2_PROP_H
#define _VALVOLA_3_2_PROP_H

//////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: valvola_3_2_prop.h
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

#include "valvola_3_2.h"



namespace chrono
{
namespace pneumatics
{

/// Class defining a proportional 3-ways valve (3-2), with
/// proportional actuation.

class valvola_3_2_prop : public valvola_3_2 {

protected :
	double comando;
	double comandomin, comandomax, comandochius;

public:
	valvola_3_2_prop();
	virtual ~valvola_3_2_prop();

	void SetComando (double mycomando) { comando = mycomando;};
	double GetComando() {return comando;};

	void SetComandoMin (double mycomandomin) { comandomin = mycomandomin;};
	double GetComandoMin() {return comandomin;};
	void SetComandoMax (double mycomandomax) { comandomax = mycomandomax;};
	double GetComandoMax() {return comandomax;};
	void SetComandoChius (double mycomandochius) { comandochius = mycomandochius;};
	double GetComandoChius() {return comandochius;};

	virtual void SetupUgelli();

	void ApplicaComando();
	virtual double Portata();

};



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____



#endif						//*****
