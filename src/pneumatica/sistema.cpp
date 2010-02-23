///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: sistema.cpp
//
// Descrizione		: 
//					  
//					  
//
// Autore			: Hermes Giberti
// Data				: Marzo 2002
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


#include "sistema.h"
#include "math.h"


namespace chrono 
{
namespace pneumatics
{


sistema::sistema()
			// Costruttore di sistema
{
	n=nc=k=0;
	t=setpoint=0;
	stato=derivatastato=NULL; 

	
}

sistema::~sistema()
			// Distruttore di sistema
{

/*
	delete [] datisalv.tempo;
	delete [] datisalv.setpoint;
	delete [] datisalv.posizione;
	delete [] datisalv.setpointvel;
	delete [] datisalv.velocita;
	delete [] datisalv.setpointacc;
	delete [] datisalv.accelerazione;

	delete [] datisalv.pres_a;
	delete [] datisalv.pres_b;
	delete [] datisalv.der_pres_a;
	delete [] datisalv.der_pres_b;

	//delete [] datisalv.portata_a;
	//delete [] datisalv.portata_b;
	//delete [] datisalv.b_a;
	//delete [] datisalv.beta_a;
	//delete [] datisalv.b_b;
	//delete [] datisalv.beta_b;

	//delete [] datisalv.pidvel_p;
	//delete [] datisalv.pidvel_d;
	//delete [] datisalv.pidpos_p;
	//delete [] datisalv.pidpos_d;

	delete [] datisalv.supsliding;
	delete [] datisalv.m1_e;
	delete [] datisalv.m2_e1;
	delete [] datisalv.e2;
	
	delete [] datisalv.e;
	delete [] datisalv.e1;

	delete [] datisalv.segno_s;
	delete [] datisalv.segno_se;
	delete [] datisalv.segno_se1;
	delete [] datisalv.segno_se2;

	delete [] datisalv.u_k1;
	delete [] datisalv.u_k2;
	delete [] datisalv.u_eta;

	delete [] datisalv.u_n;
	delete [] datisalv.u_l;
	delete [] datisalv.u_2;

	delete [] datisalv.comando_a;
	delete [] datisalv.comando_b;
*/
}

int sistema::NSistema()
			// Metodo che restituisce il numero di variabili da integrare.
{
	n=4;
	return n;
}

int sistema::NcSistema()
			// Metodo che restitusce il numero di comandi;
{
	nc=2;
	return nc;
}

void sistema::SetSistema()
			// Metodo che alloca memoria per stato e derivatastato
{
	NSistema();	// Chiamo questo metodo che scrive in n un numero e poi uso n
	NcSistema(); // Scrive in nc il numero dei comandi utilizzati da sistema 
	
	stato = new double[n];				memset(stato,0x0,sizeof(double)*n);
	derivatastato = new double[n];		memset(derivatastato,0x0,sizeof(double)*n);
	// Aggiungo a nc 6 per poter contenere i contributi del PID cfr->controllore.cpp
	comandi = new double[nc+20];			memset(comandi,0x0,sizeof(double)*(nc+20));
	
//OCIO //Per salvare i dati bisogna inizializzare num
	// Alloco memoria per salvare i dati della simulazione -> num è la dimensione 
/*	
	datisalv.tempo = new double[num];				memset(datisalv.tempo,0x0,sizeof(double)*(num));
	datisalv.setpoint = new double[num];			memset(datisalv.setpoint,0x0,sizeof(double)*(num));
	datisalv.posizione = new double[num];			memset(datisalv.posizione,0x0,sizeof(double)*(num));
	datisalv.setpointvel = new double[num];			memset(datisalv.setpointvel,0x0,sizeof(double)*(num));
	datisalv.velocita = new double[num];			memset(datisalv.velocita,0x0,sizeof(double)*(num));
	datisalv.setpointacc = new double[num];			memset(datisalv.setpointacc,0x0,sizeof(double)*(num));
	datisalv.accelerazione = new double[num];		memset(datisalv.accelerazione,0x0,sizeof(double)*(num));
	
	datisalv.pres_a = new double[num];				memset(datisalv.pres_a,0x0,sizeof(double)*(num));
	datisalv.pres_b = new double[num];				memset(datisalv.pres_b,0x0,sizeof(double)*(num));
	datisalv.der_pres_a = new double[num];			memset(datisalv.der_pres_a,0x0,sizeof(double)*(num));
	datisalv.der_pres_b = new double[num];			memset(datisalv.der_pres_b,0x0,sizeof(double)*(num));
	
	datisalv.portata_a = new double[num];			memset(datisalv.portata_a,0x0,sizeof(double)*(num));
	datisalv.portata_b = new double[num];			memset(datisalv.portata_b,0x0,sizeof(double)*(num));
	datisalv.b_a = new double[num];				memset(datisalv.b_a,0x0,sizeof(double)*(num));
	datisalv.beta_a = new double[num];			memset(datisalv.beta_a,0x0,sizeof(double)*(num));
	datisalv.b_b = new double[num];				memset(datisalv.b_b,0x0,sizeof(double)*(num));
	datisalv.beta_b = new double[num];			memset(datisalv.beta_b,0x0,sizeof(double)*(num));

	//datisalv.pidvel_p = new double[num];			memset(datisalv.pidvel_p,0x0,sizeof(double)*(num));
	//datisalv.pidvel_d = new double[num];			memset(datisalv.pidvel_d,0x0,sizeof(double)*(num));
	//datisalv.pidpos_p = new double[num];			memset(datisalv.pidpos_p,0x0,sizeof(double)*(num));
	//datisalv.pidpos_d = new double[num];			memset(datisalv.pidpos_d,0x0,sizeof(double)*(num));
	
	
	datisalv.supsliding = new double[num];			memset(datisalv.supsliding,0x0,sizeof(double)*(num));
	datisalv.m1_e = new double[num];				memset(datisalv.m1_e,0x0,sizeof(double)*(num));
	datisalv.m2_e1 = new double[num];				memset(datisalv.m2_e1,0x0,sizeof(double)*(num));
	datisalv.e2 = new double[num];					memset(datisalv.e2,0x0,sizeof(double)*(num));

	datisalv.e = new double[num];					memset(datisalv.e,0x0,sizeof(double)*(num));
	datisalv.e1 = new double[num];					memset(datisalv.e1,0x0,sizeof(double)*(num));

	datisalv.segno_s = new double[num];				memset(datisalv.segno_s,0x0,sizeof(double)*(num));
	datisalv.segno_se = new double[num];			memset(datisalv.segno_se,0x0,sizeof(double)*(num));
	datisalv.segno_se1 = new double[num];			memset(datisalv.segno_se1,0x0,sizeof(double)*(num));
	datisalv.segno_se2 = new double[num];			memset(datisalv.segno_se2,0x0,sizeof(double)*(num));

	datisalv.u_k1 = new double[num];				memset(datisalv.u_k1,0x0,sizeof(double)*(num));
	datisalv.u_k2 = new double[num];				memset(datisalv.u_k2,0x0,sizeof(double)*(num));
	datisalv.u_eta = new double[num];				memset(datisalv.u_eta,0x0,sizeof(double)*(num));

	datisalv.u_n = new double[num];					memset(datisalv.u_n,0x0,sizeof(double)*(num));
	datisalv.u_l = new double[num];					memset(datisalv.u_l,0x0,sizeof(double)*(num));
	datisalv.u_2 = new double[num];					memset(datisalv.u_2,0x0,sizeof(double)*(num));
	
	
	datisalv.comando_a = new double[num];			memset(datisalv.comando_a,0x0,sizeof(double)*(num));
	datisalv.comando_b = new double[num];			memset(datisalv.comando_b,0x0,sizeof(double)*(num));
*/
}



void sistema::Salva(int myi)
			// Metodo che salva i dati sul disco
{
	/*k=0;
	file<<t<<" ";
	while (k<n) {file<<*(stato+k) <<" "; k++;}
	file<<"\n";*/
	// Scrivendo come sopra scrivo tutto lo stato qualsiasi sia la sua lunghezza
/*
	dataout<<t<<" "<<*(stato)<<" "<<*(stato+1)<<" "<<*(derivatastato+1)<<" "<<*(stato+2)<<" "<<*(stato+3)<<" "<<*(comandi)<<" "<<*(comandi+1)<<"\n";
*/
}
 
void sistema::ScriviFile()
			// Metodo che scrive su file i dati salvati
{

}

void sistema::SetStato()
			//metodo che setta lo stato
{

}


void sistema::SetDerivataStato()
			//metodo che setta lo stato derivato
{

}

void sistema::derivate(double *s, double *ds)

{

}


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


