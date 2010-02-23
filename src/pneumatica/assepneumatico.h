#ifndef _ASSEPNEUMATICO_H			//*****
#define _ASSEPNEUMATICO_H			//*****

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: assepneumatico.h
//
// Descrizione		: Questa classe implementa tutte le funzionalità di un asse
//					  pneumatico provvisto di due valvole proporzionali in
//					  controllo di portata.
//
// Autore			: Hermes Giberti
// Data				: Marzo 2002
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////



#include <math.h>
#include "sis_attuatore_3_2.h"
#include "sis_attuatore_3_2_prop.h"


namespace chrono
{
/// Namespace with classes for the simulation of pneumatic devices
namespace pneumatics
{


/// This class provides the functionality to simulate a single
/// pneumatic actuator (i.e. a pneumatic cylinder) with two 3-2
/// valves and two nozzles.

class AssePneumatico {

private:
	// Puntatore al sistema pneumatico
	sis_attuatore_3_2_prop *myattuatore;
	// Variabili che definisco il sistema
	double ci,co,bi,bo;
	double ps,pma,pmb;
	double l,wa,wb,a,alfa,gamma,mstelo;
	double valvA_min, valvA_close, valvA_max;
	double valvB_min, valvB_close, valvB_max;

	double pva, pvb;	// Pressioni nelle due camere del cilindro
	double pos, vel;	// Posizione e velocità dello stantuffo
	double *com;		// Puntatore all'array contenente i comandi da dare alle due valvole

public:
	AssePneumatico();
	virtual ~AssePneumatico();


	// FUNZIONI PER L'INIZZIALIZZAZIONE DEL SISTEMA

	// Conduttanza ingresso
	void Set_Ci(double myci){ci=myci; };
	double Get_Ci(void){return ci;};

	// Conduttanza uscita
	void Set_Co(double myco){co=myco; };
	double Get_Co(void){return co;};

	// Beta ingresso
	void Set_Bi(double mybi){bi=mybi; };
	double Get_Bi(void){return bi;};

	// Beta uscita
	void Set_Bo(double mybo){bo=mybo; };
	double Get_Bo(void){return bo;};

	// Pressione allo scarico
	void Set_Ps(double myps){ps=myps; };
	double Get_Ps(void){return ps;};

	// Pressione di alimentazione camera a
	void Set_Pma(double mypma){pma=mypma; };
	double Get_Pma(void){return pma;};

	// Pressione di alimentazione camera b
	void Set_Pmb(double mypmb){pmb=mypmb; };
	double Get_Pmb(void){return pmb;};

	// Lunghezza della corsa
	void Set_L(double myl){l=myl; };
	double Get_L(void){return l;};

	// Volume morto camera a
	void Set_Wa(double mywa){wa=mywa; };
	double Get_Wa(void){return wa;};

	// Volume morto camera b
	void Set_Wb(double mywb){wb=mywb; };
	double Get_Wb(void){return wb;};

	// Superficie dello stantuffo
	void Set_A(double mya){a=mya; };
	double Get_A(void){return a;};

	// Coefficiente di riduzione della superficie dello stantuffo
	void Set_Alfa(double myalfa){alfa=myalfa; };
	double Get_Alfa(void){return alfa;};

	// Coefficiente di attrito viscoso
	void Set_Gamma(double mygamma){gamma=mygamma; };
	double Get_Gamma(void){return gamma;};

	// Massa dello stelo
	void Set_Mstelo(double mymstelo){mstelo=mymstelo; };
	double Get_Mstelo(void){return mstelo;};

	// Setto il comando di apertura minima della valvola
	void Set_ValvA_min(double myvalvA_min){valvA_min=myvalvA_min;};
	double Get_ValvA_min(void){return valvA_min;};

	// Setto il comando di chiusura della valvola
	void Set_ValvA_close(double myvalvA_close){valvA_close=myvalvA_close;};
	double Get_ValvA_close(void){return valvA_close;};

	// Setto il comando di apertura massima della valvola
	void Set_ValvA_max(double myvalvA_max){valvA_max=myvalvA_max;};
	double Get_ValvA_max(void){return valvA_max;};

	// Setto il comando di apertura minima della valvola
	void Set_ValvB_min(double myvalvB_min){valvB_min=myvalvB_min;};
	double Get_ValvB_min(void){return valvB_min;};

	// Setto il comando di chiusura della valvola
	void Set_ValvB_close(double myvalvB_close){valvB_close=myvalvB_close;};
	double Get_ValvB_close(void){return valvB_close;};

	// Setto il comando di apertura massima della valvola
	void Set_ValvB_max(double myvalvB_max){valvB_max=myvalvB_max;};
	double Get_ValvB_max(void){return valvB_max;};

	/////////////////////////////////////////////////////////////
	// FUNZIONI DI SCAMBIO CON **CHRONO** durante integrazione
	//

		// pressioni					<---
	void Set_P(double p1, double p2){pva=p1; pvb=p2;};
	void Get_P(double *p1, double *p2);	   // nothing

		// posizione e velocità stelo	<---
	void Set_Pos(double x, double x_dt){pos=x; vel=x_dt;};
	void Get_Pos(double *x, double *x_dt); // nothing

		// comandi valvole				<---
	void Set_Com(double com1, double com2){*(com)=com1; *(com+1)=com2;};
	void Get_Com(double *com1, double *com2){*(com1)=*(com); *(com2)=*(com+1);};
	void Set_ComA(double com1){*(com+0)=com1;};
	void Set_ComB(double com2){*(com+1)=com2;};
	double Get_ComA(){return *(com+0);};
	double Get_ComB(){return *(com+1);};

		// Da usare per inizializzarare il sistema con valori
		// differenti da quelli de default (cfr costruttore)
	void SetupAssePneumatico(void);

		// Calcola F e P_dt
	void Update(void);

		// Forza F				--->
	double Get_F(void);

		// Derivata pressioni	--->
	void Get_P_dt(double *p1_dt, double *p2_dt);

		// TO DO
	void Integrate(double t_old, double t_new){};

	//
	//
	///////////////////////////////////////////////////////////////
};


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif								//*****
