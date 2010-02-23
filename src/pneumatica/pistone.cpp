///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: pistone.cpp
//
// Descrizione		: 
//					  
//					  
//
// Autore			: Hermes Giberti
// Data				: Marzo 2002
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


#include "pistone.h"

namespace chrono 
{
namespace pneumatics
{

pistone::pistone()
			// Costruttore di pistone
{
	l=a=alfa=ga=gb=wa=wb=ps=gamma=mstelo=Dx1=fp=fs=fd=0;
	
	ca = cb = NULL;
	
	//****** Parametri relativi all'atrito
	Dx1=0.00001;
	fs=0;//51.85;		// [N]
	fd=0;//21.34;		// [N]
	//************************************

	memset(statopistone,0x0,sizeof(statopistone));
	memset(derivatastatopistone,0x0,sizeof(derivatastatopistone));
	
	ca = new volume();
	cb = new volume();

}

pistone::~pistone()
			// Distruttore di pistone
{
	if(ca) delete ca;
	ca = NULL;
	if(cb) delete cb;
	cb = NULL;
}

void pistone::SetupPistone(double mps, double ml, double mwa, double mwb, double ma, double malfa, /*double mfa,*/ double mgamma, double mmstelo)
			// Pressione di scarico mps (è la stessa che metto nel setup_valvola)
			// Lunghezza corsa ml, Volume morto camera A e camera B = mwa,mwb
			// Sezione ma, Coeff.di riduzione sezione malfa, Forza attrito mfa;
			// Coeff.attrito viscoso mgamma, Massa stelo mmstelo;
{
	l=ml;	
	a=ma;
	alfa=malfa;
	
	wa=mwa;
	wb=mwb;

	ps=mps;

	gamma=mgamma;
	mstelo=mmstelo;
}

double pistone::Forza()
			// Restituisce il valore della forza che il pistone imprime
{	
	fp=(statopistone[2]*a) - (statopistone[3]*a*alfa) - (ps*a*(1-alfa));
	
	if(statopistone[1]<Dx1 && statopistone[1]>-Dx1)
	{
		if((fabs(fp)-fs)<0)
		{
			statopistone[1]=derivatastatopistone[0]=0;
			return 0;
		}
	} 
	//if(statopistone[1]<0) fd=-fd;
	// ritorno il valore facendo il conto dul segno della forza dinamica 
	return (fp - gamma*statopistone[1] - (statopistone[1]>0 ? fd :-fd )); 
}

void pistone::SetStatoPistone()
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
	
	ca->SetP(statopistone[2]);	
	ca->SetG(ga);				// ATTENZIONE ga deve essere settata dalla valvola
	ca->SetV(wa+statopistone[0]*a);
	ca->SetV1(statopistone[1]*a);
	
	cb->SetP(statopistone[3]);	
	cb->SetG(gb);				// ATTENZIONE gb deve essere settata dalla valvola
	cb->SetV(wb+(l-statopistone[0])*a*alfa);
	cb->SetV1(-statopistone[1]*a*alfa);
}

void pistone::SetDerivataStatoPistone()
			// Setta lo stato derivato
{
	derivatastatopistone[0]=statopistone[1];
	double ap;
	ap=(Forza()-mstelo*0)/mstelo;		// + Contributo forza peso
	
	if(statopistone[0]>=l)
	{
		if(ap>=0) 
		{
			derivatastatopistone[1]=0;
		}
		else 
		{
			derivatastatopistone[1]=ap;
		}
	}
	else if(statopistone[0]<=0)
	{
		if(ap>0) 
		{
			derivatastatopistone[1]=ap;
		}
		else 
		{
			derivatastatopistone[1]=0;
		}
	}
	else if(statopistone[0]<l) 
		derivatastatopistone[1]=ap;

	derivatastatopistone[2]=PresA1();
	derivatastatopistone[3]=PresB1();
}


void pistone::SetStatoDer(double *s, double *ds)
			// Aggiorna la deriv. dello stato senza fare controlli, usando variabili d'appoggio
{
	SetStatoSist(s);
	
	*(ds)=*(s+1); //statopistone[1];
	double ap;
	ap=Forza()/mstelo;
	*(ds+1)=ap;
	*(ds+2)=PresA1();
	*(ds+3)=PresB1();
}

void pistone::SetStatoSist(double *s)
			// Aggiorna lo stato nelle variabili d'appoggio senza fare controlli 
{
	ca->SetP((*(s+2)));	
	ca->SetG(ga);				// ATTENZIONE ga deve essere settata dalla valvola
	ca->SetV(wa+(*(s))*a);
	ca->SetV1((*(s+1))*a);
	
	cb->SetP((*(s+3)));	
	cb->SetG(gb);				// ATTENZIONE gb deve essere settata dalla valvola
	cb->SetV(wb+(l-(*(s)))*a*alfa);
	cb->SetV1(-(*(s+1))*a*alfa);
}


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


