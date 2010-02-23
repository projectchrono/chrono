///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: ugello_controllato_PA.cpp
//
// Descrizione		: PA -> dal serbatoio all'utilizzatore
//					  
//					  
//
// Autore			: Hermes Giberti
// Data				: Marzo 2002
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


#include "ugello_controllato_PA.h"
#include <math.h>


namespace chrono 
{
namespace pneumatics
{


double ugello_controllato_PA::ConduttanzaIn()
			// Definisce la conduttanza in ingresso in funzione della tensione
{
	return  /*2.5e-8*(comando/5-1);*/ /*2.5e-8/1000*comando;*/ (2.5e-8)*comando;
	//-5.35e-17*pow(comando,3)+6.7e-14*pow(comando,2)+1.23e-11*comando-1.83e-9;
}

double ugello_controllato_PA::ConduttanzaOut()
			// Definisce la conduttanza in uscita in funzione della tensione
{
	return /*2.4e-8*(comando/5-1);*/ /*2.4e-8/1000*comando;*/ (2.4e-8)*comando;
	//-5.69e-17*pow(comando,3)+6.61e-14*pow(comando,2)+1.74e-11*comando-3.23e-9;
}

double ugello_controllato_PA::BetaIn()
			// Definisce il beta in ingresso in funzione della tensione
{
	return /*3.5e-1*(comando/5-1);*/(3.5e-1)*comando;
	//-3.5e-12*pow(comando,4)+9.2e-9*pow(comando,3)-8.3e-6*pow(comando,2)+2.9e-3*comando-1.2e-3;
	
}

double ugello_controllato_PA::BetaOut()
			// Definisce il beta in uscita in funzione della tensione
{
	return /*3.5e-1*(comando/5-1);*/(3.5e-1)*comando;
	//1.7e-9*pow(comando,3)-2.8e-6*pow(comando,2)+1.2e-3*comando+1.9e-1;
	
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

