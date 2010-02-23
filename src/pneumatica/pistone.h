#ifndef _PISTONE_H			//*****
#define _PISTONE_H			//*****

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: pistone.h
//
// Descrizione		: 
//					  
//					  
//
// Autore			: Hermes Giberti
// Data				: Marzo 2002
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////



#ifndef NULL
 #define NULL 0
#endif

#include "volume.h"
#include <string.h>

namespace chrono 
{
namespace pneumatics
{

/// This class implements the details of a pneumatic piston
/// with given stroke, chambers,  etc. 

class pistone {

public:	
	double l, a, alfa;
			// l = corsa del pistone, a= sezione stantuffo, alfa=coeff.di riduzione area;
	double wa, wb;
			// wolumi morti delle due camere;
	double ga, gb;
			// ga = portata valvola A, gb = portata valvola B;
	double  ps;
			// ps = pressione di scarico;
	double gamma, mstelo;
			// coeff.di attrito viscoso, massa dello stelo;

	double statopistone[4];
			// [0]=spostamento; [1]=velocità; [2]=pressione A; [3]=pressione B;
	double derivatastatopistone[4];
			// [0]=velocità; [1]=accelerazione; [2]=derivata press A; [3]=derivata press B;
	
	double Dx1,fp,fs,fd;
			// Dx1 =intervallo velocità, fp = forza dovuta alle pressioni
			// fs = forza d'attrito statico, fd = forza d'attrito dinamico;
	
	volume *ca, *cb;
			// puntatori che permetteranno di creare la camera A e la B

public:
	pistone();
	virtual ~pistone();

	void SetL (double myl) { l = myl;};
	double GetL () {return l;};
	void SetA (double mya) { a = mya;};
	double GetA () {return a;};
	void SetAlfa (double myalfa) { alfa = myalfa;};
	double GetAlfa () {return alfa;};

	void SetWa (double mywa) { wa = mywa;};
	double GetWa () {return wa;};
	void SetWb (double mywb) { wb = mywb;};
	double GetWb () {return wb;};

	void SetGa (double myga) { ga = myga;};
	double GetGa () {return ga;};
	void SetGb (double mygb) { gb = mygb;};
	double GetGb () {return gb;};
	
	void SetPs (double myps) { ps = myps;};
	double GetPs () {return ps;};
	
	void SetGamma (double mygamma) { gamma = mygamma;};
	double GetGamma () {return gamma;};
	void SetMstelo (double mymstelo) { mstelo = mymstelo;};
	double GetMstelo () {return mstelo;};
	
	volume *GetCa() {return ca;};
	volume *GetCb() {return cb;};
	
	
	virtual void SetupPistone(double mps, double ml, double mwa, double mwb, double ma, double malfa, double mgamma, double mmstelo);
			// Metodo che inizializza il pistone
	
	virtual double *GetStatoPistone() {return statopistone;};
			// Restituisce un puntatore a statopistone
	virtual void SetStatoPistone();
			// Aggiorna lo stato del sistema
	virtual double *GetDerivataStatoPistone() {return derivatastatopistone;};
			// Restituisce un puntatore a derivatastatopistone
	virtual void SetDerivataStatoPistone();
			// Riempie l'array derivatastatopistone con il nuovo derivatastatopistone
			
	virtual void SetStatoDer(double *s, double *ds);
	virtual void SetStatoSist(double *s);
			// Questi due metotodi settano lo stato e lo stato derivato senza controlli

	double Forza();
			// Restituisce la forza dello stantuffo
	double PresA1() {return (ca -> Pressione1());};
			// Restituisce la derivata della pressione nella camera a
	double PresB1() {return (cb -> Pressione1());};
			// Restituisce la derivata della pressione nella camera b
	
};


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif						//*****
