// Sistema Pneumatico costituito da un attuatore e da due valvole 3/2

//******************************************************************************
#include "assepneumatico.h"

#include <math.h>
#include <fstream>
#include <stdlib.h>


//******************************************************************************

void main(void)
{
	double forza;
	AssePneumatico *pneum;
	pneum = new AssePneumatico();

	for(int i=0;i<100;i++)
	{
		pneum->Update();
		forza=pneum->Get_F();

		pneum->Set_P(601325,601325);
	}
	
	return;
}

//******************************************************************************