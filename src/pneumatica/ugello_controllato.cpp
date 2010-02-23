///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: ugello_controllato.cpp
//
// Descrizione		: 
//					  
//					  
//
// Autore			: Hermes Giberti
// Data				: Marzo 2002
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


#include "ugello_controllato.h"
#include <math.h>


namespace chrono 
{
namespace pneumatics
{


double ugello_controllato::GetConduttanza()
			// Sceglie in funzione del verso il valore della conduttanza
{	
	if (verso == flusso_in) {return ConduttanzaIn();}; return ConduttanzaOut();
}

double ugello_controllato::GetBeta()
			// Sceglie in funzione del verso il valore di beta
{
	if (verso == flusso_in) {return BetaIn();}; return BetaOut();
}



double ugello_controllato::ConduttanzaIn()
			// Definisce la conduttanza in ingresso in funzione della tensione
{
	return ci*comando;
}

double ugello_controllato::ConduttanzaOut()
			// Definisce la conduttanza in uscita in funzione della tensione
{
	return co*comando;
}

double ugello_controllato::BetaIn()
			// Definisce il beta in ingresso in funzione della tensione
{
	return bi*comando;
}

double ugello_controllato::BetaOut()
			// Definisce il beta in uscita in funzione della tensione
{
	return bo*comando;
}


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

