//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

//////////////////////////////////////////////////
//  
//   ChIntegrator.cpp
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "physics/ChIntegrator.h"

namespace chrono 
{


ChIntegrator::ChIntegrator()
{
	method = INTEG_EULEROSTD_EXP;
}

ChIntegrator::~ChIntegrator()
{
}

void ChIntegrator::Copy(ChIntegrator* source)
{
	// first copy the parent class data...
	ChObj::Copy(source);

	// copy own data
	method = source->method;

}


int ChIntegrator::ODEintegrate_step (
					int n_eq,				// number of equations
					ChMatrix<>* Y,				// the current state (also will contain resulting state after integration)
					double& t,				// the starting time (also will contain final time ater integration)
					int getdy(ChMatrix<>* m_Y, double m_t, ChMatrix<>* m_dYdt, void* m_userdata), // function which computes dy/dt=f(y,t), returning TRUE if OK, FALSE if error
					double h,				// step length.
					double& int_error,		// if possible, returns here the local error extimation
					void* userdata)			// additional user data
{
	//
	// initialize stuff..
	//

	int_error = 0.0;
	
	int i;
	int m_ok = TRUE;

	double a2=0.2,a3=0.3,a4=0.6,a5=1.0,a6=0.875,b21=0.2,
		b31=3.0/40.0,b32=9.0/40.0,b41=0.3,b42 = -0.9,b43=1.2,
		b51 = -11.0/54.0, b52=2.5,b53 = -70.0/27.0,b54=35.0/27.0,
		b61=1631.0/55296.0,b62=175.0/512.0,b63=575.0/13824.0,
		b64=44275.0/110592.0,b65=253.0/4096.0,c1=37.0/378.0,
		c3=250.0/621.0,c4=125.0/594.0,c6=512.0/1771.0,
		dc5 = -277.00/14336.0;
	double dc1=c1-2825.0/27648.0,dc3=c3-18575.0/48384.0,
		dc4=c4-13525.0/55296.0,dc6=c6-0.25;

	//
	// get number of equations...
	//

	int nequations = Y->GetRows();
	if (nequations != n_eq) 
				return FALSE;

	//
	// Initialize and reset temporary vectors, to match number
	// of equations
	//

	ChMatrixDynamic<> Ydt;
	ChMatrixDynamic<> Ytemp;
	ChMatrixDynamic<> Ynew;
	ChMatrixDynamic<> Yk1;
	ChMatrixDynamic<> Yk2;
	ChMatrixDynamic<> Yk3;
	ChMatrixDynamic<> Yk4;
	ChMatrixDynamic<> Yk5;
	ChMatrixDynamic<> Yk6;

	Ydt.Reset	(nequations, 1);
	Ytemp.Reset (nequations, 1);
	Yk1.Reset	(nequations, 1);
	Yk2.Reset	(nequations, 1);
	Yk3.Reset	(nequations, 1);
	Yk4.Reset	(nequations, 1);
	Yk5.Reset	(nequations, 1);
	Yk6.Reset	(nequations, 1);
	Ynew.Reset	(nequations, 1);


	// 
	// Perform step integration
	//

	switch (this->method)
	{


	case INTEG_EULEROSTD_EXP:

				// 1-- dY/dt   at current time: t0 

		m_ok=(*getdy)(Y, t,			&Yk1, userdata);	// >>>>>>>>>>>>>>>>>>>>>>> Yk1

		Yk1.MatrScale(h);

		Ynew.MatrAdd (*Y, Yk1);
		
		break;



	case INTEG_EULEROMOD_EXP:

				// 1- Yk1 = Ydt (Y, t0)
		m_ok=(*getdy)(Y, t,		&Yk1, userdata);		// >>>>>>>>>>>>>>>>>>>>>>> Yk1


				// 2- Yk2 = Ydt (Y+(dt/2)*Ydt,  t+dt/2)
		for (i = 0; i < nequations; i++)
			Ytemp.SetElement(i,0, 
						Y->GetElement(i,0) +
						h*0.5* Ydt.GetElement(i,0)  );

		m_ok=(*getdy)(&Ytemp, t+h/2,	&Yk2, userdata);	// >>>>>>>>>>>>>>>>>>>>>>> Yk2 


				// 3- Ynew = Y + (Yk2 * dt)
		for (i = 0; i < nequations; i++)
			Ynew.SetElement(i,0, 
						Y->GetElement(i,0) +
						h* Yk2.GetElement(i,0)  );
	
		break;



	case INTEG_HEUN_EXP:

				// 1- Yk1 = Ydt (Y, t0)
		m_ok=(*getdy)(Y, t,		&Yk1, userdata);			// >>>>>>>>>>>>>>>>>>>>>>> Yk1


				// 2- Yk2 = Ydt (Y+ dt*Ydt,  t+dt)
		for (i = 0; i < nequations; i++)
			Ytemp.SetElement(i,0, 
						Y->GetElement(i,0) +
						h * Yk1.GetElement(i,0)  );

		m_ok=(*getdy)(&Ytemp, t+h,	&Yk2, userdata);		// >>>>>>>>>>>>>>>>>>>>>>> Yk2 

				// 3- Ynew= Y + (Yk1 * dt/2) + (Yk1 * dt/2)
		for (i = 0; i < nequations; i++)
			Ynew.SetElement(i,0, 
						Y->GetElement(i,0) +
						h*0.5* Yk1.GetElement(i,0) +
						h*0.5* Yk2.GetElement(i,0) );

		break;



	case INTEG_KUTTA_EXP:

		// 1-- dY/dt   at current time: t0 
		m_ok=(*getdy)(Y, t,			&Yk1, userdata);	// >>>>>>>>>>>>>>>>>>>>>>> Yk1

		// 2-- dY/dt   at time: t0 +a2*h
		for (i = 0; i < nequations; i++)
			Ytemp.SetElement(i,0, 
						Y->GetElement(i,0) +
						b21*h* Yk1.GetElement(i,0)  );

		m_ok=(*getdy)(&Ytemp, t +a2*h,	&Yk2, userdata);	// >>>>>>>>>>>>>>>>>>>>>>> Yk2

		// 3-- dY/dt   at time: t0 +a3*h
		for (i = 0; i < nequations; i++)
			Ytemp.SetElement(i,0, 
						Y->GetElement(i,0) +
						h*( b31*Yk1.GetElement(i,0) +
						    b32*Yk2.GetElement(i,0)  )   );

		m_ok=(*getdy)(&Ytemp, t +a3*h,	&Yk3, userdata);	// >>>>>>>>>>>>>>>>>>>>>>> Yk3


		// 4-- dY/dt   at time: t0 +a4*h
		for (i = 0; i < nequations; i++)
			Ytemp.SetElement(i,0, 
						Y->GetElement(i,0) +
						h*( b41*Yk1.GetElement(i,0) +
						    b42*Yk2.GetElement(i,0) +
							b43*Yk3.GetElement(i,0)  )   );
		
		m_ok=(*getdy)(&Ytemp, t +a4*h,	&Yk4, userdata);	// >>>>>>>>>>>>>>>>>>>>>>> Yk4
		

		// 5-- dY/dt   at time: t0 +a5*h
		for (i = 0; i < nequations; i++)
			Ytemp.SetElement(i,0, 
						Y->GetElement(i,0) +
						h*( b51*Yk1.GetElement(i,0) +
						    b52*Yk2.GetElement(i,0) +
						    b53*Yk3.GetElement(i,0) +
							b54*Yk4.GetElement(i,0)  )   );

		m_ok=(*getdy)(&Ytemp, t +a5*h,	&Yk5, userdata);	// >>>>>>>>>>>>>>>>>>>>>>> Yk5


		// 6-- dY/dt   at time: t0 +a6*h
		for (i = 0; i < nequations; i++)
			Ytemp.SetElement(i,0, 
						Y->GetElement(i,0) +
						h*( b61*Yk1.GetElement(i,0) +
						    b62*Yk2.GetElement(i,0) +
						    b63*Yk3.GetElement(i,0) +
							b64*Yk4.GetElement(i,0) +
							b65*Yk5.GetElement(i,0)  )   );

		m_ok=(*getdy)(&Ytemp, t +a6*h,	&Yk6, userdata);	// >>>>>>>>>>>>>>>>>>>>>>> Yk6


		// Ynew = ..  acumulate to set incremented new Y state 
		for (i = 0; i < nequations; i++)
			Ynew.SetElement(i,0,
						Y->GetElement(i,0) +
						h*( c1*Yk1.GetElement(i,0) +
							c3*Yk3.GetElement(i,0) +
							c4*Yk4.GetElement(i,0) +
							c6*Yk6.GetElement(i,0)	)	);

				// -- compute local error
		for (i = 0; i < nequations; i++)
			int_error += ChMax(int_error, fabs(
					    h*( dc1*Yk1.GetElement(i,0) +
							dc3*Yk3.GetElement(i,0) +
							dc4*Yk4.GetElement(i,0) +
							dc5*Yk5.GetElement(i,0) +
							dc6*Yk6.GetElement(i,0)	 )    
							));
		break;



	case INTEG_RK_EXP:

		// 1-- dY/dt   at current time: t0 

		m_ok=(*getdy)(Y, t,			&Yk1, userdata);	// >>>>>>>>>>>>>>>>>>>>>>> Yk1


		// 2-- dY/dt   at time: t0 + h/2
		for (i = 0; i < nequations; i++)
			Ytemp.SetElement(i,0, 
						Y->GetElement(i,0) +
						0.5 * h * Yk1.GetElement(i,0)  );

		m_ok=(*getdy)(&Ytemp, t +h/2,	&Yk2, userdata);	// >>>>>>>>>>>>>>>>>>>>>>> Yk2


		// 3-- dY/dt   at time: t0 + h/2
		for (i = 0; i < nequations; i++)
			Ytemp.SetElement(i,0, 
						Y->GetElement(i,0) +
						0.5 * h * Yk2.GetElement(i,0)  );

		m_ok=(*getdy)(&Ytemp, t +h/2,	&Yk3, userdata);	// >>>>>>>>>>>>>>>>>>>>>>> Yk3


		// 4-- dY/dt   at time: t0 + h
		for (i = 0; i < nequations; i++)
			Ytemp.SetElement(i,0, 
						Y->GetElement(i,0) +
							  h * Yk3.GetElement(i,0)  );
				
		m_ok=(*getdy)(&Ytemp, t +h,		&Yk4, userdata);	// >>>>>>>>>>>>>>>>>>>>>>> Yk4



		// Ynew = ... acumulate to set incremented new Y state
		for (i = 0; i < nequations; i++)
			Ynew.SetElement(i,0,
						Y->GetElement(i,0) +
						h*( (1.0/6.0)*Yk1.GetElement(i,0) +
							(1.0/3.0)*Yk2.GetElement(i,0) +
							(1.0/3.0)*Yk3.GetElement(i,0) +
							(1.0/6.0)*Yk4.GetElement(i,0)	)	);

		break;





	default:
		
		m_ok = FALSE;
		break;
	}

	//
	// Go forward.. (UPDATE TIME AND STATE)
	//

	Y->CopyFromMatrix(Ynew);
	t = t+h;


	return (m_ok);
}


} // END_OF_NAMESPACE____
