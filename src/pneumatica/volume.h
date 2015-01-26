#ifndef _VOLUME_H			//*****
#define _VOLUME_H			//*****

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: volume.h
//
// Descrizione		:
//
//
//
// Autore			: Hermes Giberti
// Data				: Marzo 2002
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////




#include <math.h>


namespace chrono
{
namespace pneumatics
{


/// Class defining the properties of a pneumatic volume of
/// air, with density, pressure, etc.

class volume {

private:
	double rho, n, p0;
			// rho = densità normale dell'aria [kg/m^3], p0 = pressione normale dell'aria
			// n = coefficiente della politropica
	double g, p, v, v1;
			// rispettivamente portata, pressione, volume, derivata del volume;

public:
	volume() {rho=1.225; p0=101325, n = 1;g=p=v=v1=0;};
	virtual ~volume() {};

	void SetG (double myg) { g = myg;};
	double GetG () {return g;};
	void SetP (double myp) { p = myp;};
	double GetP () {return p;};
	void SetV (double myv) { v = myv;};
	double GetV () {return v;};
	void SetV1 (double myv1) { v1 = myv1;};
	double GetV1 () {return v1;};

	double Pressione1() {return ( (g-rho* pow((p/p0),(1/n)) *v1)*n*p0/(rho*v)*pow((p0/p),((1-n)/n)) ) ;}

};



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____



#endif						//*****
