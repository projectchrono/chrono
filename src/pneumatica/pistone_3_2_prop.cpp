//////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: pistone_3_2_prop.cpp
//
// Descrizione		: pistone comandato con 2 valvole 3-2 proporzionali
//					  
//					  
//
// Autore			: Hermes Giberti
// Data				: Marzo 2002
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


#include "pistone_3_2_prop.h"


namespace chrono 
{
namespace pneumatics
{


pistone_3_2_prop::pistone_3_2_prop()
			// Costruttore di pistone_3_2_prop
{
	valva = valvb = NULL;
	// Ricordarsi di chiamare il metodo SetupValvole() per creare i due oggetti
}

pistone_3_2_prop::~pistone_3_2_prop()
			// Distruttore di pistone
{
	if(valva) delete valva;
	valva = NULL;
	if(valvb) delete valvb;
	valvb = NULL;
	
}

void pistone_3_2_prop::SetupValvole()
		// Decide a seconda della classe quale valvole usare
{
	valva = new valvola_3_2_prop();
	valva->SetupUgelli();
	valvb = new valvola_3_2_prop();
	valvb->SetupUgelli();
}

void pistone_3_2_prop::SetComando(double mycomandoA, double mycomandoB)
			// Scrivo il comando nella variabile comando delle valvole, la funzione portata si occupa del resto
{
	valva->SetComando(mycomandoA);	// Scrive il comando nella variabile comando e basta
	valvb->SetComando(mycomandoB);
}

void pistone_3_2_prop::SetStatoPistone()
			// Setta lo stato del sistema e si preoccupa dei vari controlli sulle variabili
{
	if(statopistone[0]>l)
	{
		statopistone[0]=l;
		statopistone[1]=0;
	}

	if(statopistone[0]<0)
	{
		statopistone[0]=0;
		statopistone[1]=0;
	}
	
	valva->SetPa(statopistone[2]);
	valvb->SetPa(statopistone[3]);

	ca->SetP(statopistone[2]);	
	ca->SetG(valva->Portata());				
	ca->SetV(wa+statopistone[0]*a);
	ca->SetV1(statopistone[1]*a);
	
	cb->SetP(statopistone[3]);	
	cb->SetG(valvb->Portata());				
	cb->SetV(wb+(l-statopistone[0])*a*alfa);
	cb->SetV1(-statopistone[1]*a*alfa);

}


void pistone_3_2_prop::SetStatoSist(double *s)

{
	valva->SetPa((*(s+2)));
	valvb->SetPa((*(s+3)));

	ca->SetP((*(s+2)));	
	ca->SetG(valva->Portata());				
	ca->SetV(wa+(*(s))*a);
	ca->SetV1((*(s+1))*a);
	
	cb->SetP((*(s+3)));	
	cb->SetG(valvb->Portata());				
	cb->SetV(wb+(l-(*(s)))*a*alfa);
	cb->SetV1(-(*(s+1))*a*alfa);
}


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


