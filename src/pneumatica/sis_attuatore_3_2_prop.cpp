///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: sis_attuatore_3_2_prop.cpp
//
// Descrizione		: 
//					  
//					  
//
// Autore			: Hermes Giberti
// Data				: Marzo 2002
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


#include "sis_attuatore_3_2_prop.h"


namespace chrono 
{
namespace pneumatics
{


sis_attuatore_3_2_prop::sis_attuatore_3_2_prop()
			// Costruttore
{
	mypistone = NULL;
	mypistone = new pistone_3_2_prop();

	
	//mycontrollore = NULL;
	//mycontrollore = new controllore();

	//myfunzione = NULL;
	//myfunzione = new gen_funzioni();


}


sis_attuatore_3_2_prop::~sis_attuatore_3_2_prop()
			// Distruttore
{
	if(mypistone) delete mypistone;
	mypistone = NULL;

	//if(mycontrollore) delete mycontrollore;
	//mycontrollore = NULL;

	//if(myfunzione) delete myfunzione;
	//myfunzione = NULL;
}


void sis_attuatore_3_2_prop::InizializzaSistema()
			// Metodi che inizializza il sistema(condizioni iniziali nel sistema)
{	
	memcpy(stato,mypistone->statopistone,(n*sizeof(double)));
	memcpy(derivatastato,mypistone->derivatastatopistone,(n*sizeof(double)));
}

void sis_attuatore_3_2_prop::SetStato()
			// Seta lo stato tenendo conto dell'effetto del controllore			
{
	// Scarico da 0 a 1000, Chiusa = 0, Alimentazione da -1000 a 0
	// SetComando(valvola A,valvola B);
	
	//double setA,coA;
	//setA=coA=0;
	//setA = (myfunzione->Rampa((GetT())));
	//coA = (mycontrollore->proporzionale(setA, *(stato+1)));
	
	//mypistone->SetComando(coA,-500);

	memcpy(mypistone->statopistone,stato,(n*sizeof(double)));
	//Copio da stato del sistema a stato del pistone (statopistone)
	
	mypistone->SetStatoPistone(); // Qui faccio i controlli funzionali e il controllore?????
	
}


void sis_attuatore_3_2_prop::SetDerivataStato()
			//
{
	mypistone->SetDerivataStatoPistone();
	memcpy(derivatastato,mypistone->derivatastatopistone,(n*sizeof(double)));
}


void sis_attuatore_3_2_prop::derivate(double *s, double *ds)

{
	mypistone->SetStatoDer(s,ds);
}


void sis_attuatore_3_2_prop::Comanda(double *com)

{
	mypistone->SetComando(*(com),*(com+1));
}


void sis_attuatore_3_2_prop::Salva(int myi)
			// Metodo che salva i dati sul disco
{
	/*k=0;
	file<<t<<" ";
	while (k<n) {file<<*(stato+k) <<" "; k++;}
	file<<"\n";
	// Scrivendo come sopra scrivo tutto lo stato qualsiasi sia la sua lunghezza

	dataout
		<<t<<" "<<setpoint
		<<" "<<*(stato)<<" "<<*(stato+1)<<" "<<*(derivatastato+1)
		<<" "<<*(stato+2)<<" "<<*(stato+3)<<" "<<*(derivatastato+2)<<" "<<*(derivatastato+3)
		
		<<" "<<mypistone->GetValvA()->GetPortata()<<" "<<mypistone->GetValvB()->GetPortata()
		
		<<" "<<mypistone->GetValvA()->b<<" "<<mypistone->GetValvA()->beta
		<<" "<<mypistone->GetValvB()->b<<" "<<mypistone->GetValvB()->beta
		
		<<" "<<*(comandi)<<" "<<*(comandi+1)<<"\n";
	*/
	
	if (myi < num) {

	datisalv.tempo[myi] =t;
	datisalv.setpoint[myi] =setpoint;
	datisalv.posizione[myi] = *(stato);
	datisalv.setpointvel[myi] =setpointvel;
	datisalv.velocita[myi] = *(stato+1);
	datisalv.setpointacc[myi] =setpointacc;
	datisalv.accelerazione[myi] = *(derivatastato+1);
	
	datisalv.pres_a[myi] = *(stato+2);
	datisalv.pres_b[myi] = *(stato+3);
	datisalv.der_pres_a[myi] = *(derivatastato+2);
	datisalv.der_pres_b[myi] = *(derivatastato+3);
	
	datisalv.portata_a[myi]= mypistone->GetValvA()->GetPortata();
	datisalv.portata_b[myi] = mypistone->GetValvB()->GetPortata();
	datisalv.b_a[myi] = mypistone->GetValvA()->b;
	datisalv.beta_a[myi] = mypistone->GetValvA()->beta;
	datisalv.b_b[myi] = mypistone->GetValvB()->b;
	datisalv.beta_b[myi] = mypistone->GetValvB()->beta;
/*	
	// Anello di posizione
	datisalv.pidvel_p[myi] =*(comandi+5);
			//datisalv.pidvel_i[myi] =*(comandi+6);  OCIO manca dato nella struttura dati
	datisalv.pidvel_d[myi] =*(comandi+7);
	// Anello di velocità
	datisalv.pidpos_p[myi] =*(comandi+2);
			//datisalv.pidpos_i[myi] =*(comandi+3);  OCIO manca dato nella struttura dati
	datisalv.pidpos_d[myi] =*(comandi+4);
	
*/
	datisalv.supsliding[myi] =*(comandi + 2);
	datisalv.m1_e[myi] =*(comandi + 3);
	datisalv.m2_e1[myi] =*(comandi + 4);
	datisalv.e2[myi] =*(comandi + 5);

	datisalv.e[myi] =*(comandi + 6);
	datisalv.e1[myi] =*(comandi + 7);

	datisalv.segno_s[myi] =*(comandi + 8);
	datisalv.segno_se[myi] =*(comandi + 9);
	datisalv.segno_se1[myi] =*(comandi + 10);
	datisalv.segno_se2[myi] =*(comandi + 11);

	datisalv.u_k1[myi] =*(comandi + 12);
	datisalv.u_k2[myi] =*(comandi + 13);
	datisalv.u_eta[myi] =*(comandi + 14);

	datisalv.u_n[myi] =*(comandi + 15);
	datisalv.u_l[myi] =*(comandi + 16);
	datisalv.u_2[myi] =*(comandi + 17);

	datisalv.comando_a[myi] =(mypistone->GetValvA())->GetComando();  //*(comandi);
	datisalv.comando_b[myi] =(mypistone->GetValvB())->GetComando();  //*(comandi+1);
	}

}

void sis_attuatore_3_2_prop::ScriviFile()
			// Metodo che scrive i dati su disco
{
	// Tolgo il primo punto perchè ho setpvel e setpacc che fanno casino!!
	/*
	for (j=1;j<num;j++) 
	{
		dataout
			<<datisalv.tempo[j]<<" \t"				//1
			<<datisalv.setpoint[j]<<" \t"			//2
			<<datisalv.posizione[j]<<" \t"			//3
			<<datisalv.setpointvel[j]<<" \t"		//4
			<<datisalv.velocita[j]<<" \t"			//5
			<<datisalv.setpointacc[j]<<" \t"		//6
			<<datisalv.accelerazione[j]<<" \t"		//7
			
			<<datisalv.pres_a[j]<<" \t"				//8
			<<datisalv.pres_b[j]<<" \t"				//9
			<<(datisalv.pres_a[j]-datisalv.pres_b[j])<<" \t"			//10 //<<datisalv.der_pres_a[j]<<" \t"			//10
			<<datisalv.der_pres_b[j]<<" \t"			//11
			
			<<datisalv.portata_a[j]<<" \t"			//12
			<<datisalv.portata_b[j]<<" \t"			//13
			<<datisalv.b_a[j]<<" \t"				//14
			<<datisalv.beta_a[j]<<" \t"				//15
			<<datisalv.b_b[j]<<" \t"				//16
			<<datisalv.beta_b[j]<<" \t"				//17
			
			//<<datisalv.pidvel_p[j]<<" \t"			//1
			//<<datisalv.pidvel_d[j]<<" \t"			//1
			//<<datisalv.pidpos_p[j]<<" \t"			//1
			//<<datisalv.pidpos_d[j]<<" \t"			//1

			<<datisalv.supsliding[j]<<" \t"			//18
			<<datisalv.m1_e[j]<<" \t"				//19
			<<datisalv.m2_e1[j]<<" \t"				//20
			<<datisalv.e2[j]<<" \t"					//21

			<<datisalv.e[j]<<" \t"					//22
			<<datisalv.e1[j]<<" \t"					//23

			<<datisalv.segno_s[j]<<" \t"			//24
			<<datisalv.segno_se[j]<<" \t"			//25
			<<datisalv.segno_se1[j]<<" \t"			//26
			<<datisalv.segno_se2[j]<<" \t"			//27

			<<datisalv.u_k1[j]<<" \t"				//28
			<<datisalv.u_k2[j]<<" \t"				//29
			<<datisalv.u_eta[j]<<" \t"				//30

			<<datisalv.u_n[j]<<" \t"				//31
			<<datisalv.u_l[j]<<" \t"				//32
			<<datisalv.u_2[j]<<" \t"				//33
			
			<<datisalv.comando_a[j]<<" \t"			//34
			<<datisalv.comando_b[j]<<"\n";			//35
		
		// Escludo in questo modo la parte finale della simulazione se ha valori senza senso
		// ... credo...
		if (datisalv.tempo[j+1]==0 || datisalv.setpoint[j+1]< 0) j=num;
	}
	*/
}


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

