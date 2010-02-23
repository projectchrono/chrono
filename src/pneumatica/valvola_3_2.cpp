///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: valvola_3_2.cpp
//
// Descrizione		: Valvola a tre vie, del tipo ON/OFF:
//					  P-> Alimentazione	R-> Scarico		A-> Utilizzatore
//					  
//
// Autore			: Hermes Giberti
// Data				: Marzo 2002
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


#include "valvola_3_2.h"


namespace chrono 
{
namespace pneumatics
{


valvola_3_2::valvola_3_2()
			// Costruttore di valvola_3_2
{
	stato=chiusa;
	pp=pr=pa=portata=0;

	b=beta=0;
	
	ug1=ug2=NULL;
	//SetupUgelli();
}

valvola_3_2::~valvola_3_2()
			// Distruttore di valvola_3_2
{
	if(ug1) delete ug1;
	ug1 = NULL;
	if(ug2) delete ug2;
	ug2 = NULL;
}

void valvola_3_2::SetupUgelli()
			// Definisce come costruire gli ugelli componenti la valvola
{
	ug1 = new ugello();
	ug2 = new ugello();
}



void valvola_3_2::SetupValvola(double mci1, double mco1, double mbi1, double mbo1, double mci2, double mco2, double mbi2, double mbo2, double mpp, double mpr, double mpa)
			// Inizializza la valvola			
			// Ugello 1: mci1, mco1, mbi1, mbo1;
			// Ugello 2: mci2, mco2, mbi2, mbo2;
			// Condizioni iniziali: mpp, mpr, mpa
{
	ug1 -> SetCi(mci1);
	ug1 -> SetCo(mco1);
	ug1 -> SetBi(mbi1);
	ug1 -> SetBo(mbo1);

	ug2 -> SetCi(mci2);
	ug2 -> SetCo(mco2);
	ug2 -> SetBi(mbi2);
	ug2 -> SetBo(mbo2);

	SetPp(mpp);
	SetPr(mpr);
	SetPa(mpa);
}


void valvola_3_2::Funzionamento()
			// Definisce il funzionamento della valvola
{
			// L'oggetto ug1 rappresenta il canale di Alimentazione
	ug1 -> SetPmonte(pp);
	ug1 -> SetPvalle(pa);

			// L'oggetto ug2 rappresenta il canale di Scarico
	ug2 -> SetPmonte(pr);
	ug2 -> SetPvalle(pa);

	switch (stato) {
	case scarico:
		ug1->SetIsopen(false);
		ug2->SetIsopen(true);
		
		portata = (ug1->Portata() + ug2->Portata());
		b=ug2->GetB();
		beta=ug2->GetBe();
		
		break;
	case chiusa:
		ug1->SetIsopen(false);
		ug2->SetIsopen(false);

		portata = 0;		//(ug1->Portata() + ug2->Portata())
		b=0;		//ug2->GetB(); potrebbe essere utile vedere il b a valvola chiusa
		beta=0;		//ug2->GetBe();

		break;
	case alimentazione:	
		ug1->SetIsopen(true);
		ug2->SetIsopen(false);

		portata = (ug1->Portata() + ug2->Portata());
		b=ug1->GetB();
		beta=ug1->GetBe();

		break;
	}

}


double valvola_3_2::Portata()
			// Restituisce la portata transitante nella valvola
{
	Funzionamento();
	//portata = (ug1->Portata() + ug2->Portata()); 
	return portata;
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

