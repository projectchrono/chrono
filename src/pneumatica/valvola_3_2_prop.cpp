///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: valvola_3_2_prop.cpp
//
// Descrizione		: 
//					  
//					  
//
// Autore			: Hermes Giberti
// Data				: Marzo 2002
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include "valvola_3_2_prop.h"
#include "ugello_controllato_PA.h"
#include "ugello_controllato_RA.h"
 

namespace chrono 
{
namespace pneumatics
{


valvola_3_2_prop::valvola_3_2_prop()
{
	comando=comandomin=comandomax=comandochius=0;
		
	comandomin =1000;
	comandochius =0;
	comandomax =-1000;
	

	// non metto niente perchè uso il costruttore della classe base....
}

valvola_3_2_prop::~valvola_3_2_prop()
{
	if(ug1) delete ug1;
	ug1 = NULL;
	if(ug2) delete ug2;
	ug2 = NULL;
}

void valvola_3_2_prop::SetupUgelli()
			// Definisce come costruire gli ugelli componenti la valvola
{
	ug1 = new ugello_controllato_PA();
	ug2 = new ugello_controllato_RA();
}


void valvola_3_2_prop::ApplicaComando()

{
	if (comando>comandomax) comando=comandomax;
	else if (comando<comandomin) comando= comandomin;

	if (comando < (comandochius+1e-3) && comando > (comandochius-1e-3)) // comando < (comandochius+1e-3) && comando < (comandochius-1e-3)
		stato = chiusa;
	else if ((comando < comandochius))
		{
		((ugello_controllato_PA*)ug1) -> SetComando((comandochius-comando)/fabs(comandomin-comandochius)); //SetComando(comandochius-comando);
		stato = alimentazione;
		}
	else if ((comando > comandochius))
		{
		((ugello_controllato_RA*)ug2) -> SetComando((comando-comandochius)/fabs(comandomax-comandochius)); //SetComando(comando-comandochius);
		stato = scarico;
		}
	//else if (comando == comandochius) stato = chiusa;
}

double valvola_3_2_prop::Portata()
{
ApplicaComando();
Funzionamento();		// scrive anche in portata la portata del momento
//portata = (ug1->Portata() + ug2->Portata());

return portata;
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

