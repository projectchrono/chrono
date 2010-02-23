///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// File				: ugello.cpp
//
// Descrizione		: 
//					  
//					  
//
// Autore			: Hermes Giberti
// Data				: Marzo 2002
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


#include "ugello.h"
#include <math.h>


namespace chrono 
{
namespace pneumatics
{


double ugello::Portata()
			//Restituisce il valore della portata assegnate le pressioni a cavallo dell'ugello
{
	
	/*double Q, pres;			
			// Portata massica [kg/s]
	double fi;
			// Funzione che discrimina fra flusso sonico e subsonico (fattore sonico)
	double rho;
			// Densità normale dell'aria [kg/m^3]
	double t0;
			// Temperatura in condizioni normali [K]
	double tambiente;
			// Temperatura ambiente [K], (25°C)
	rho=1.225;
	t0=293.15;
	tambiente=298;*/
	

	if (!isopen) { return 0;};
			// Verifica se la valvola è aperta o no
	
	b=SetDirezione();	// Beta temporaneo
	beta = GetBeta();	// Beta del ugello per quel comando

	if (beta < b)
		{ fi=sqrt(1.0-( pow((b-beta) / (1.0-beta) , 2.0)  ) );}
			// condizioni subsoniche
	else
		{fi=1;}
			// condizioni soniche
	
	(verso==flusso_in) ? (pres=pmonte) : (pres=-pvalle);

	Q= GetConduttanza()*pres*fi*rho*sqrt(t0/tambiente);
				// Calcola la portata attribuebdo a pres il segno e il valore corretto
	return Q;
		
}


double ugello::SetDirezione()
			// Impone una direzione al flusso a seconda delle pressioni a cavallo della valvola
{
	double btemp;
			// pM = pressione maggiore, pm = pressione minore, btemp = beta del momento

	if((pmonte/pvalle)>1)
		{
		btemp = pvalle/pmonte;
		verso = flusso_in;
		}
	else
		{
		btemp = pmonte/pvalle;
		verso = flusso_out;
		}
	
    return btemp;
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


