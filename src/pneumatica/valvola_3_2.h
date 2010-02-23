#ifndef _VALVOLA_3_2_H			//*****
#define _VALVOLA_3_2_H			//*****

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: valvola_3_2.h
//
// Descrizione		: Valvola a tre vie, del tipo ON/OFF:
//					  P-> Alimentazione	R-> Scarico		A-> Utilizzatore
//
//
// Autore			: Hermes Giberti
// Data				: Marzo 2002
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


#ifndef NULL
 #define NULL 0
#endif

#include <math.h>
#include "ugello.h"



namespace chrono
{
namespace pneumatics
{

/// Class defining the 3-ways valve (3-2), with on-off behaviour.

class valvola_3_2 {

protected:
	ugello *ug1, *ug2;
			// Puntatori ad ugello
	double pp, pr, pa;
			// pp= pressione all'alimentazione  pr= pres.allo scarico  pa= pres.all'utilizzatore
	double portata;
			// portata che attraversa la valvola
public:
	double b,beta;

public:
	enum StatoValvola { scarico = -1, chiusa = 0, alimentazione = 1} stato;

public:

	valvola_3_2();
	virtual ~valvola_3_2();

	virtual void SetupUgelli();

	void SetPp (double mypp) { pp= mypp;};
	double GetPp () {return pp;};
	void SetPr (double mypr) { pr= mypr;};
	double GetPr () {return pr;};
	void SetPa (double mypa) { pa= mypa;};
	double GetPa () {return pa;};

	double GetPortata() {return portata;};

	ugello *GetUgello1() {return ug1;};
	ugello *GetUgello2() {return ug2;};

	void SetStato (StatoValvola mystato) { stato = mystato;};
	StatoValvola GetStato () {return stato;};

	void SetupValvola(double mci1, double mco1, double mbi1, double mbo1, double mci2, double mco2, double mbi2, double mbo2, double mpp, double mpr, double mpa);
	// Ugello 1: mci1, mco1, mbi1, mbo1; Ugello 2: mci2, mco2, mbi2, mbo2; Condizioni iniziali: mpp, mpr, mpa


	/*virtual*/ void Funzionamento();
	virtual double Portata();


};


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif						//*****
