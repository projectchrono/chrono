#ifndef _UGELLO_H			//*****
#define _UGELLO_H			//*****

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: ugello.h
//
// Descrizione		: 
//					  
//					  
//
// Autore			: Hermes Giberti
// Data				: Marzo 2002
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


#include <string.h>


namespace chrono 
{
namespace pneumatics
{

/// Class defining pneumatic nozzles.

class ugello {

protected:
	double ci, co, bi, bo;	
			// ci=Conduttanzain ingresso; co=Conduttanza in uscita [m^3/(s*Pa)]
			// bi=beta in ingresso; bo=beta in uscita
	double pvalle, pmonte;
			// Pvalle=Pressione a valle dell'ugello; Pmonte=pressione a monte
	bool isopen;
			// Variabile che definisce se la valvola è aperta o chiusa

	double Q, pres;			
			// Portata massica [kg/s]
	double fi;
			// Funzione che discrimina fra flusso sonico e subsonico (fattore sonico)
	double rho;
			// Densità normale dell'aria [kg/m^3]
	double t0;
			// Temperatura in condizioni normali [K]
	double tambiente;
			// Temperatura ambiente [K], (25°C)


	double b;			//beta istantaneo 
	double beta;		//beta della valvola


public:
	enum TipoFlusso {flusso_out = 0, flusso_in = 1} verso;
			// verso definisce la direzione del flusso flusso_out = 0, flusso_in =1

public:
	ugello()  {isopen = true;verso = flusso_in; ci=co=bi=bo=pvalle=pmonte=.0;
				rho=1.225;t0=293.15;tambiente=298; b=beta=0;};
	virtual ~ugello() {};

	void SetCi(double myci) {ci = myci;};
	double GetCi() {return ci;};
	void SetCo(double myco) {co = myco;};
	double GetCo() {return co;};
	void SetBi(double mybi) {bi = mybi;};
	double GetBi() {return bi;};
	void SetBo(double mybo) {bo = mybo;};
	double GetBo() {return bo;};
	
	double GetB() {return b;};
	double GetBe() {return beta;};
	
	void SetPvalle(double mypvalle) { pvalle= mypvalle;};
	double GetPvalle() {return pvalle;};
	void SetPmonte(double mypmonte) { pmonte= mypmonte;};
	double GetPmonte() {return pmonte;};
	
	void SetIsopen(bool myisopen) {isopen = myisopen;};
	bool GetIsopen() {return isopen;};
	
	virtual double GetConduttanza() { if (verso == flusso_in) {return ci;}; return co;};				
			// Decide in funzione del verso quale C usare
	virtual double GetBeta() { if (verso == flusso_in) {return bi;}; return bo;};						
			// Decide in funzione del verso quale b usare
	double Portata();	
			// Restituisce il valore della portata
	double SetDirezione();
			// Impone la diresione del flusso a seconda delle pressioni a cavallo della valvola

	

};


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____



#endif						//*****
