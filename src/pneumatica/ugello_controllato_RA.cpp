///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: ugello_controllato_RA.cpp
//
// Descrizione		: RA -> dallo scarico all'utilizzatore
//					  
//					  
//
// Autore			: Hermes Giberti
// Data				: Marzo 2002
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


#include "ugello_controllato_RA.h"
#include <math.h>


namespace chrono 
{
namespace pneumatics
{


double ugello_controllato_RA::ConduttanzaIn()
			// Definisce la conduttanza in ingresso in funzione della tensione
{
	return /*2.6e-8*(comando/5-1);*/(2.6e-8)*comando;
	//-4.71e-17*pow(comando,3)+4.18e-14*pow(comando,2)+3.42e-11*comando-2.63e-9;
}

double ugello_controllato_RA::ConduttanzaOut()
			// Definisce la conduttanza in uscita in funzione della tensione
{
	return /*2.4e-8*(comando/5-1);*/(2.4e-8)*comando;
	//-3.76e-17*pow(comando,3)+2.8e-14*pow(comando,2)+3.55e-11*comando-2.76e-9;
}

double ugello_controllato_RA::BetaIn()
			// Definisce il beta in ingresso in funzione della tensione
{
	return  /*3.8e-1*(comando/5-1);*/(3.8e-1)*comando;
	//-4.5e-12*pow(comando,4)+1.2e-8*pow(comando,3)-1.1e-5*pow(comando,2)+3.8e-3*comando-4.0e-2;
}

double ugello_controllato_RA::BetaOut()
			// Definisce il beta in uscita in funzione della tensione
{
	return  /*3.8e-1*(comando/5-1);*/(3.8e-1)*comando;
	//-8.5e-12*pow(comando,4)+2.0e-8*pow(comando,3)-1.6e-5*pow(comando,2)+4.7e-3*comando-1.5e-2;
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

