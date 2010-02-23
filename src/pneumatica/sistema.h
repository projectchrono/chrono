///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: sistema.h
//
// Descrizione		: 
//					  
//					  
//
// Autore			: Hermes Giberti
// Data				: Marzo 2002
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


#ifndef _SISTEMA_H			//*****
#define _SISTEMA_H			//*****

#ifndef NULL
 #define NULL 0
#endif

#include <stdlib.h>
#include <fstream>
#include <string.h>


namespace chrono 
{
namespace pneumatics
{

/// A pneumatic system. This class can be specialized for 
/// representing more specific systems (for example a piston
/// with valves, etc.).

class sistema {

public:
	int n,nc,k;		// n = numero di elementi di stato, nc = num di comandi,k = contatore in salva
	int num;		// num = numero di punti da salvare
	double t;		// t = tempo (nell'integratore non è definito)
	double dt;
	double *stato, *derivatastato;
	double *comandi;
	double setpoint;
	double setpointvel;
	double setpointacc;
	
	struct save {
		double *tempo;
		double *setpoint;
		double *posizione;
		double *setpointvel;
		double *velocita;
		double *setpointacc;
		double *accelerazione;
		
		double *pres_a;
		double *pres_b;
		double *der_pres_a;
		double *der_pres_b;
		
		double *portata_a;
		double *portata_b;
		double *b_a;
		double *beta_a;
		double *b_b;
		double *beta_b;
	
		//double *pidvel_p;
		//double *pidvel_d;
		//double *pidpos_p;
		//double *pidpos_d;

		double *supsliding;
		double *m1_e;
		double *m2_e1;
		double *e2;

		double *e;
		double *e1;

		double *segno_s;
		double *segno_se;
		double *segno_se1;
		double *segno_se2;

		double *u_k1;
		double *u_k2;
		double *u_eta;

		double *u_n;
		double *u_l;
		double *u_2;
		
		double *comando_a;
		double *comando_b;
	} datisalv;		// Struttura che contiene di dati da salvare in memoria

public:
	//ofstream dataout;		//crea un stream di output

public:
	sistema();
	virtual ~sistema();
	
	void SetNum (int mynum) { num= mynum;};
	int GetNum () {return num;};

	void SetSetpoint (double mysetpoint) { setpoint= mysetpoint;};
	double GetSetpoint () {return setpoint;};

	void SetSetpointVel (double mysetpointvel) { setpointvel= mysetpointvel;};
	double GetSetpointVel () {return setpointvel;};

	void SetSetpointAcc (double mysetpointacc) { setpointacc= mysetpointacc;};
	double GetSetpointAcc () {return setpointacc;};
	
	void SetT (double myt) { t= myt;};
	double GetT () {return t;};
	
	void SetDt (double mydt) { dt= mydt;};
	double GetDt () {return dt;};

	virtual int NSistema();
			// Metodo che restitusce il numero di variabili da integrare (dimensione dello stato);
	virtual int NcSistema();
			// Metodo che restitusce il numero di comandi;
	
	virtual void SetSistema();
			// Metodo che alloca memoria per stato e derivatastato
	virtual void InizializzaSistema() {};
	
	virtual void Salva(int myi);
			// Metodo che salva i dati i memoria
	virtual void ScriviFile();
			// Metodi che scrive su file i dati salvati
	
	virtual void Comanda(double *com) {};
			// Metodo che distribusce i comandi su i vari oggetti di sistema
	
	virtual double *GetStato() {return stato;};				//metodo che restituisce un puntare a stato;
	virtual void SetStato();	// Qui faccio i controlli funzionali e il controllore?????

	virtual double *GetDerivataStato() {return derivatastato;};		//metodo che restituisce un puntare a derivatastato;
	virtual void SetDerivataStato();

	virtual double *GetComandi() {return comandi;};		//metodo che restituisce un puntare a comandi;
	virtual void SetComandi(){};

	
	virtual void derivate(double *s, double *ds);


};


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____



#endif						//*****
