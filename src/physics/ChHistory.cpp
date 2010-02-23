///////////////////////////////////////////////////
//
//   ChHistory.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChHistory.h"


namespace chrono
{

ChHistory::ChHistory(int nvars, int nsteps)
{
	if (nsteps <1) {nsteps = 1;}

	steps = nsteps;
	vars = nvars;
	current_Y = 0;

	times = (double*) calloc (steps, sizeof(double));
	Yvectors = (ChMatrix<>**) calloc (steps, sizeof(ChMatrix<>*));

	for (int i=0; i < steps; ++i)
	{
		*(times + i) = 0;
		ChMatrix<>* m_Y = new ChMatrixDynamic<>(vars,1);
		*(Yvectors + i) = m_Y;
	}
}


ChHistory::~ChHistory()
{
	for (int i=0; i < steps; ++i)
	{
		delete *(Yvectors + i);
	}

	free (Yvectors);
	free (times);
}

void ChHistory::Setup(int newvars, int newsteps)
{
			// delete old matrices
    int i;

	for (i=0; i < steps; ++i)
	{
		delete *(Yvectors + i);
	}

	free (Yvectors);
	free (times);

			// instance new ones
	steps = newsteps;
	vars = newvars;
	current_Y = 0;

	times = (double*) calloc (steps, sizeof(double));
	Yvectors = (ChMatrix<>**) calloc (steps, sizeof(ChMatrix<>*));

	for (i=0; i < steps; ++i)
	{
		*(times + i) = 0;
		*(Yvectors +i) = new ChMatrixDynamic<>(vars,1);
	}
}

void ChHistory::Copy(ChHistory* source)
{
	Setup ( source->vars, source->steps);
	current_Y = source->current_Y;

	for (int i=0; i < steps; ++i)
	{
		*(times +i) = source->times[i];
		(*(Yvectors +i))->CopyFromMatrix(**(source->Yvectors + i));
	}
}


void ChHistory::Restart()
{
	for (int i=0; i < steps; ++i)
	{
		*(times + i) = 0;
	}
}


int ChHistory::Get_arr_offset (int i)
{
		// range only in  -(steps-2) ...0..1
	if (i < (2-steps)) {i = (2-steps);}
	if (i >	   1	 ) {i = 1;}

	int selected;
	selected = (current_Y + i);

		// modular cycling
	if (selected >= steps) { selected = selected - steps;}
	if (selected < 0)	   { selected = steps + selected;}

	return (selected);
}


ChMatrix<>* ChHistory::Get_Y(int i)
{
	int sel;
	sel = Get_arr_offset(i);

	return *(Yvectors + sel);
}

double ChHistory::Get_Ytime(int i)
{
	int sel;
	sel = Get_arr_offset(i);

	return *(times + sel);
}

void ChHistory::Set_Ytime(int i, double newtime)
{
	int sel;
	sel = Get_arr_offset(i);

	*(times + sel) = newtime;
}





void ChHistory::ForwardStep()
{
	current_Y++;	// increments the offset of one,

	if (current_Y >= steps)
	{
		current_Y = 0;	// if end of array, cycle back!
	}
}

void ChHistory::BackwardStep()
{
	current_Y--;	// increments the offset of one,

	if (current_Y < 0)
	{
		current_Y = (steps -1);	// if begin of array, go to end!
	}
}

void ChHistory::ProceedStep(double m_dt)
{
	double newtime;

	if (m_dt>0)
	{
		newtime= (Get_Ytime(0) + m_dt);
		ForwardStep();
		Set_now_time (newtime);
	}

	if (m_dt<0)
	{
		newtime= (Get_Ytime(0) + m_dt);
		BackwardStep();
		Set_now_time (newtime);
	}
}


int ChHistory::GetNsequenced(double maxstep)
{
	int sequence = 1;
	double timeA, timeB;
	int i;
	for (i= 0; i <= (steps-2); ++i)
	{
		timeA = Get_Ytime (-i);
		timeB = Get_Ytime (-i-1);

		if ((timeA == timeB) ||
		    ((timeA - timeB) > (maxstep + 0.00000001)) ||
		    (timeA <  timeB)  ) break;

		sequence++;
	}
	return sequence;
}



// POLYNOMIAL EXTRAPOLATION AND INTERPOLATION FUNCTIONS
// FOR PREDICTIONS

void ChHistory::PredictY(int order, double m_dt, ChMatrix<>* Ypredicted)
{
	int nJ, nI;
	double newX, iX, jX;

	// build Lagrange interpolation coefficients

	newX = Get_now_time() + m_dt;

	if (order < 1) order = 1;
	if (order >= steps) {order = (steps - 1);}

	int points = order +1;

	ChMatrix<>* li = new ChMatrixDynamic<>(1, points);

	for (nJ = 0; nJ < points; nJ++)
	{
		double lu = 1;
		double ll = 1;
		jX = Get_Ytime(nJ - order);

		for (nI = 0; nI < points; nI++)
		{
			if (nI != nJ)
			{
				iX = Get_Ytime(nI - order);
				lu *= (newX - iX);
				ll *= (jX - iX);
			}
		}

		li->SetElement(0,nJ, (lu/ll));
	}

	// fill the predicted Y with extrapolation
	int ivar, istep;
	double newel;

	for (ivar = 0; ivar < vars ; ivar++)
	{
		newel = 0;
		for (istep= 0; istep < points; istep++)
		{
			newel += (Get_Y(istep - order)->GetElement(ivar,0) *
					  (li->GetElement(0,istep)));
		}
		Ypredicted->SetElement (ivar, 0,newel);
	}

}

void ChHistory::PredictYnew(int order, double m_dt)
{
	if (order > (steps-2)) {order = (steps - 2);}

	// fill the Ynew with the prediction
	PredictY (order, m_dt, Get_Ynew());

	// set the new time for extrapolated Ynew:
	Set_Ytime (1, (Get_now_time() + m_dt));
}


double ChHistory::PredictionError(int order)
{
	int nJ, nI;
	double newX, iX, jX;

	// build Lagrange interpolation coefficients

	newX = Get_Ytime(1);

	if (order < 1) order = 1;
	if (order > (steps-2)) {order = (steps - 2);}
	if (order == 0) return 0; // at least linear interpolation (order >= 1)...

	int points = order +1;

	ChMatrix<>* li = new ChMatrixDynamic<>(1, points);

	for (nJ = 0; nJ < points; nJ++)
	{
		double lu = 1;
		double ll = 1;
		jX = Get_Ytime(nJ - order);

		for (nI = 0; nI < points; nI++)
		{
			if (nI != nJ)
			{
				iX = Get_Ytime(nI - order);
				lu *= (newX - iX);
				ll *= (jX - iX);
			}
		}

		li->SetElement(0,nJ, (lu/ll));
	}

	// fill the predicted Y with extrapolation
	int ivar, istep;
	double newel;
	double m_error = 0;
	ChMatrix<>* m_newY = Get_Ynew();

	for (ivar = 0; ivar < vars ; ivar++)
	{
		newel = 0;
		for (istep= 0; istep < points; istep++)
		{
			newel += (Get_Y(istep - order)->GetElement(ivar,0) *
					  (li->GetElement(0,istep)));
		}
		m_error += pow ((newel - m_newY->GetElement(ivar, 0)),2);
	}

	return sqrt(m_error);
}



} // END_OF_NAMESPACE____

