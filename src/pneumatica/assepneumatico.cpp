///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: assepneumatico.cpp
//
// Descrizione		: Questa classe implementa tutte le funzionalità di un asse
//					  pneumatico provvisto di due valvole proporzionali in 
//					  controllo di portata.
//
// Autore			: Hermes Giberti
// Data				: Marzo 2002
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#include "assepneumatico.h"


namespace chrono 
{
namespace pneumatics
{


///////////////////////////////////////////////////////////////////////////////
AssePneumatico::AssePneumatico()
{
	myattuatore = new sis_attuatore_3_2_prop();
	myattuatore->mypistone->SetupValvole();

	com =new double[2]; memset(com,0x0,sizeof(double)*2);


	// Attribuisco alle variabili che definiscono il sistema dei valori sensati
	ci=2.75e-8;	//0.0000000275; // Conduttanza in ingresso
	co=3.12e-8;	//0.0000000312;	// Conduttanza in uscita
	bi=0.44157; // Beta in ingresso
	bo=0.29748;	// Beta in uscita
	
	
	pva=601325;		//101325; //Pressione nella camera a
	pma=601325;		//601325; // pressione di alimentazione della camera a
	pvb=681325;		//601325; // pressione nella camera b //** Per non farlo partire
	pmb=601325;		//601325; // pressione di alimentazione della camera b
	ps =101325;		// pressione di scarico
	
	
	l=0.5;			// Corsa dello stelo
	wa =0.0000042;	//4.2e-6; //0.002*0.1	// Volume morto camera a
	wb =0.000013;	//1.3e-5;				// Volume morto camera b
	a=0.002;					// Superficie dello stantuffo
	alfa=0.9;					// Coefficiente di riduzione superficie dello stantuffo
	gamma=59.6;					// Coefficiente di smorzamento viscoso							
	mstelo=5.7;					// Massa dello stelo
	
	valvA_min = 0;
	valvA_close = 5;
	valvA_max = 10;
	
	valvB_min = 0;
	valvB_close = 5;
	valvB_max = 10;

	
	// Inizializzo il sistema
	// Considero le valvole come simmetriche
	(myattuatore->mypistone->GetValvA())->SetupValvola(ci, co, bi, bo, ci, co, bi, bo, pma, ps, pva);
	(myattuatore->mypistone->GetValvB())->SetupValvola(ci, co, bi, bo, ci, co, bi, bo, pmb, ps, pvb);

	myattuatore->mypistone->SetupPistone(ps,l,wa,wb,a,alfa,gamma,mstelo);
	
	// Inizizializzo lo stato del sitema
	*(myattuatore->mypistone->GetStatoPistone())=0;		
	*(myattuatore->mypistone->GetStatoPistone()+1)=0;
	*(myattuatore->mypistone->GetStatoPistone()+2)=pva;
	*(myattuatore->mypistone->GetStatoPistone()+3)=pvb;
	
	//******************************************************************************
	// Impongo il range di funzionamento delle valvole
	(myattuatore->mypistone->GetValvA())->SetComandoMin(valvA_min);
	(myattuatore->mypistone->GetValvA())->SetComandoChius(valvA_close);
	(myattuatore->mypistone->GetValvA())->SetComandoMax(valvA_max);
	
	
	(myattuatore->mypistone->GetValvB())->SetComandoMin(valvB_min);
	(myattuatore->mypistone->GetValvB())->SetComandoChius(valvB_close);
	(myattuatore->mypistone->GetValvB())->SetComandoMax(valvB_max);
	
	// Condizioni iniziali delle valvole
	myattuatore->mypistone->SetComando(valvA_close ,valvB_close); //0 0

	// Initialize own command array. Alex
	com[0] = valvA_close;
	com[1] = valvB_close;

	// Alloca memoria per le varibili di sistema
	myattuatore->SetSistema();

};

///////////////////////////////////////////////////////////////////////////////
AssePneumatico::~AssePneumatico()
{
	if(myattuatore) delete myattuatore;
		myattuatore = NULL;
}

///////////////////////////////////////////////////////////////////////////////
void AssePneumatico::SetupAssePneumatico(void)
{
	(myattuatore->mypistone->GetValvA())->SetupValvola(ci, co, bi, bo, ci, co, bi, bo, pma, ps, pva);
	(myattuatore->mypistone->GetValvB())->SetupValvola(ci, co, bi, bo, ci, co, bi, bo, pmb, ps, pvb);

	myattuatore->mypistone->SetupPistone(ps,l,wa,wb,a,alfa,gamma,mstelo);
	
	// Inizizializzo lo stato del sitema
	
	//***NOT WANTED??? Alex
	/*

	*(myattuatore->mypistone->GetStatoPistone())=0;		
	*(myattuatore->mypistone->GetStatoPistone()+1)=0;
	*(myattuatore->mypistone->GetStatoPistone()+2)=pva;
	*(myattuatore->mypistone->GetStatoPistone()+3)=pvb;

	*/	


	//******************************************************************************
	// Impongo il range di funzionamento delle valvole
	(myattuatore->mypistone->GetValvA())->SetComandoMin(valvA_min);
	(myattuatore->mypistone->GetValvA())->SetComandoChius(valvA_close);
	(myattuatore->mypistone->GetValvA())->SetComandoMax(valvA_max);
	
	
	(myattuatore->mypistone->GetValvB())->SetComandoMin(valvB_min);
	(myattuatore->mypistone->GetValvB())->SetComandoChius(valvB_close);
	(myattuatore->mypistone->GetValvB())->SetComandoMax(valvB_max);
	
	// Condizioni iniziali delle valvole

	// ***NOT WANTED???  Alex
	//myattuatore->mypistone->SetComando(valvA_close ,valvB_close); //0 0
};

///////////////////////////////////////////////////////////////////////////////
//void AssePneumatico::Set_P(double p1, double p2)
//{
//
//};
//
///////////////////////////////////////////////////////////////////////////////
void AssePneumatico::Get_P(double *p1, double *p2)
{

};

///////////////////////////////////////////////////////////////////////////////
//void AssePneumatico::Set_Pos(double x, double x_dt)
//{
//
//};
//
///////////////////////////////////////////////////////////////////////////////
void AssePneumatico::Get_Pos(double *x, double *x_dt)
{

};

///////////////////////////////////////////////////////////////////////////////
void AssePneumatico::Update(void)
{
	// Inizizializzo lo stato del sitema
	*(myattuatore->GetStato())=pos;			// Spostamento
	*(myattuatore->GetStato()+1)=vel;		// Velocità
	*(myattuatore->GetStato()+2)=pva;		// Pressione camera a
	*(myattuatore->GetStato()+3)=pvb;		// Pressione camera b

	// Setto lo stato
	myattuatore->SetStato();

	myattuatore->SetDerivataStato();

	myattuatore->InizializzaSistema();

	(myattuatore->comandi)= com;
	myattuatore->Comanda(myattuatore->comandi);
}; 


///////////////////////////////////////////////////////////////////////////////
double AssePneumatico::Get_F(void)
{	
	return myattuatore->mypistone->Forza();
};

///////////////////////////////////////////////////////////////////////////////
void AssePneumatico::Get_P_dt(double *p1_dt, double *p2_dt)
{
	*(p1_dt)=myattuatore->mypistone->PresA1();
	*(p2_dt)=myattuatore->mypistone->PresB1();
};


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


