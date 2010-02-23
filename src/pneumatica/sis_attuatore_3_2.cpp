///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: sis_attuatore_3_2.cpp
//
// Descrizione		: 
//					  
//					  
//
// Autore			: Hermes Giberti
// Data				: Marzo 2002
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


#include "sis_attuatore_3_2.h"


namespace chrono 
{
namespace pneumatics
{


sis_attuatore_3_2::sis_attuatore_3_2()
			// Costruttore
{
	mypistone = NULL;
	mypistone = new pistone_3_2();

	//mycontrollore = NULL;
	//mycontrollore = new controllore();
}

sis_attuatore_3_2::~sis_attuatore_3_2()
			// Distruttore
{
	if(mypistone) delete mypistone;
	mypistone = NULL;

	//if(mycontrollore) delete mycontrollore;
	//mycontrollore = NULL;
}



void sis_attuatore_3_2::InizializzaSistema()
			// Metodi che inizializza il sistema(condizioni iniziali nel sistema)
{
	
	memcpy(stato,mypistone->statopistone,(n*sizeof(double)));
	memcpy(derivatastato,mypistone->derivatastatopistone,(n*sizeof(double)));

}


void sis_attuatore_3_2::SetStato()
			
{
	// scarico = -1, chiusa = 0, alimentazione = 1
	//SetComando(1,-1);
//double com;
//com = mycontrollore->proporzionale(1.4, *(stato+1));
//mypistone->SetComando(com,-com);

	//memcpy(stato[0],mysistema->GetStato(),(n*sizeof(double)));

	memcpy(mypistone->statopistone,stato,(n*sizeof(double)));
	//Copio da stato del sistema a stato del pistone (statopistone)
	/*mypistone->statopistone[0] = *(stato);
	mypistone->statopistone[1] = *(stato+1);
	mypistone->statopistone[2] = *(stato+2);
	mypistone->statopistone[3] = *(stato+3);*/
	
	
	mypistone->SetStatoPistone(); // Qui faccio i controlli funzionali e il controllore?????

}


void sis_attuatore_3_2::SetDerivataStato()
			//
{
	mypistone->SetDerivataStatoPistone();
	memcpy(derivatastato,mypistone->derivatastatopistone,(n*sizeof(double)));
	
	//*(derivatastato)=mypistone->derivatastatopistone[0];
	//*(derivatastato+1)=mypistone->derivatastatopistone[1];
	//*(derivatastato+2)=mypistone->derivatastatopistone[2];
	//*(derivatastato+3)=mypistone->derivatastatopistone[3];
}


void sis_attuatore_3_2::derivate(double *s, double *ds)

{
	mypistone->SetStatoDer(s,ds);
}


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____



