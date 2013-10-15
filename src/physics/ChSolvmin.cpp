//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010, 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

//////////////////////////////////////////////////
//
//   ChSolvmin.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#if !defined(__APPLE__)
#include <malloc.h>
#endif
#include "core/ChLog.h"
#include "physics/ChSolvmin.h"



namespace chrono
{





void null_entry_solv_opt(double x[],double g[]){}

///////////////////////////////////////////////////////////
///
/// Ch_optimizer
///           Base optimization engine member functions
///

// build
ChOptimizer::ChOptimizer()
{
	strcpy (err_message, "");
	C_vars = 0;
	afunction = 0;
	afunctionGrad = 0;

	xv = NULL;
	xv_sup = NULL;
	xv_inf = NULL;

	minimize = FALSE;
	opt_fx= 0;
	error_code = 0;
	fx_evaluations = 0;
	grad_evaluations = 0;
	grad_step = 1.e-12;

	break_funct = NULL;
	break_cycles = 10;
	break_cyclecounter = 0;
	user_break = 0;
}


// destroy
ChOptimizer::~ChOptimizer()
{

}


// copy
void ChOptimizer::Copy(ChOptimizer* source)
{
	// first copy the parent class data...
	ChObj::Copy(source);

	// copy error message
	strcpy (err_message, source->err_message);

	minimize = source->minimize;
	error_code = source->error_code;
	fx_evaluations = source->fx_evaluations;
	grad_evaluations = source->grad_evaluations;
	grad_step = source->grad_step;
	C_vars = source->C_vars;

	xv = source->xv;
	xv_sup = source->xv_sup;
	xv_inf = source->xv_inf;

	break_funct = source->break_funct;
	break_cycles = source->break_cycles;
	break_cyclecounter = source->break_cyclecounter;
	user_break = source->user_break;

}


// Evaluates the function ("function" string) in database, with given state
// of variables.

double ChOptimizer::Eval_fx(double x[])
{
	ChMatrixDynamic<> Vin	(this->C_vars,1);
	ChMatrixDynamic<> Vout	(1,1);

	for(int i=0; i<Vin.GetRows(); i++) Vin(i,0)=x[i];
	this->afunction->Eval(Vout, Vin);

	this->fx_evaluations++;

	return Vout(0,0);
}



// Evaluates the function ("function" string) in database, with given state
// of variables.


void ChOptimizer::Eval_grad(double x[], double gr[])
{
	if (this->afunctionGrad)
	{
		ChMatrixDynamic<> Vin	(this->C_vars,1);
		ChMatrixDynamic<> Vout	(this->C_vars,1);

		for(int i=0; i<Vin.GetRows(); i++) Vin(i,0)=x[i];
		this->afunctionGrad->Eval(Vout, Vin);
		for(int i=0; i<Vin.GetRows(); i++) gr[i]=Vout(i,0);
	}
	else
	{
		// ------ otherwise use BDF  ---------------------
		int mtotvars = GetNumOfVars();
		double oldval;
		double mf, mfd;

		// %%%%%%%%%%%%%  Evaluate central value of function
		mf = Eval_fx(x);

		for (int mvar = 0; mvar < mtotvars; mvar++)
		{
			oldval = x[mvar];
			// increment one variable
			x[mvar]=oldval + this->grad_step;
			mfd = Eval_fx(x);
			// %%%%%%%%%%%%% compute gradient by BDF
			gr[mvar]= ((mfd-mf)/(this->grad_step));
			// back to original value
			x[mvar]=oldval;
		}
	}
	this->grad_evaluations++;		// increment the counter of total number of gradient evaluations
}


double ChOptimizer::Eval_fx(const ChMatrix<>* x)
{
	ChMatrixDynamic<> out(1,1);
	this->afunction->Eval(out, *x);
	this->fx_evaluations++;
	return out(0,0);
}

void   ChOptimizer::Eval_grad(const ChMatrix<>* x, ChMatrix<>* gr)
{
	//Eval_grad(x->GetAddress(), gr->GetAddress());
	if (this->afunctionGrad)
	{
		this->afunctionGrad->Eval(*gr, *x);
	}
	else
	{
		// ------ otherwise use BDF  ---------------------
		int mtotvars = GetNumOfVars();
		double oldval;
		double mf, mfd;

		// Evaluate central value of function
		mf = Eval_fx(x);
		ChMatrixDynamic<> mdx(*x);

		for (int mvar = 0; mvar < mtotvars; mvar++)
		{
			oldval = (*x)(mvar);
			// increment one variable
			mdx(mvar)=oldval + this->grad_step;
			mfd = Eval_fx(x);
			// compute gradient by BDF
			(*gr)(mvar)= ((mfd-mf)/(this->grad_step));
			// back to original value
			mdx(mvar)=oldval;
		}
	}
}



//// breaking

void  ChOptimizer::DoBreakCheck()
{
	break_cyclecounter++;
	if (break_cyclecounter > break_cycles)
		if (this->break_funct)
		{
			break_cyclecounter = 0;
			user_break = break_funct();
		}
}



//// OPTIMIZE FUNCTION

int ChOptimizer::PreOptimize()
{
	// reset the counter of number of evaluations.
	fx_evaluations = 0;
	grad_evaluations = 0;

	// reset breakers
	user_break = 0;
	break_cyclecounter = 0;

	// reset error message
	strcpy (err_message, "");

	// check count vars
	int nv = GetNumOfVars();
	if (nv<1) { error_code = OPT_ERR_NOVARS;
				strcpy (err_message, "Error: no variables defined");
				return FALSE;}

	return TRUE;
}

int ChOptimizer::DoOptimize()
{
	//////
	//////  Must be implemented by child class!!!
	//////  ..............................
	return TRUE;
}

int ChOptimizer::PostOptimize()
{
	// *** TO DO ***//
	return TRUE;
}


// DO all the tree steps in sequence
//

int ChOptimizer::Optimize()
{
	if (!this->PreOptimize())  return FALSE;	// 1-
	if (!this->DoOptimize())   return FALSE;	// 2-
	if (!this->PostOptimize()) return FALSE;	// 3-
	return TRUE;
}



///////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
///
/// Ch_local_optimizer
///           Local optimization engine member functions
///


//// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
/////
////  EVALUATION OF OBJECTIVE FUNCTION ( C function called by solvopt() function )


double CalcFO (double x[],void *  idData)
{

	ChOptimizerLocal* moptimizer = (ChOptimizerLocal*) idData;

		// evaluate function and change sign, because local optimizer
		// uses the solvopt() function which by default minimizes (we
		// maximize by default
	return -(moptimizer->Eval_fx(x));
}

void showVarFun() { };

//// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$


// build
ChOptimizerLocal::ChOptimizerLocal()
{
	// first, build parent class
	ChOptimizer::ChOptimizer();

	initial_step = 1.0;
	arg_tol = 1.e-6;
	fun_tol = 1.e-7;
	maxiters = 50;
	maxevaluations = 200;
	gamma = 2.001e0;
	dilation = 2.5;
	gradstep = 1.e-100;

	iters_done = 0;
}


// destroy
ChOptimizerLocal::~ChOptimizerLocal()
{
	// first, delete parent class
	// Ch_optimizer::~Ch_optimizer();
}


// copy
void ChOptimizerLocal::Copy(ChOptimizerLocal* source)
{
	// first, copy  parent class
	ChOptimizer::Copy(source);

	initial_step = source->initial_step;
	arg_tol = source->arg_tol;
	fun_tol = source->fun_tol;
	maxiters = source->maxiters;
	maxevaluations = source->maxevaluations;
	gamma = source->gamma;
	dilation = source->dilation;
	gradstep = source->gradstep;

	iters_done = source->iters_done;
}






//// OPTIMIZE FUNCTION  , locally with pseudo-NR method


int ChOptimizerLocal::DoOptimize()
{
 	double solvopt_options[12];
	double nstep;
	int err_code = 0;

	// count vars
	int nv = GetNumOfVars();
	if (nv<1) { error_code = OPT_ERR_NOVARS; return FALSE;}

	//nstep = fabs(initial_step);
	//if (minimize) nstep = nstep * (-1);

	nstep = - fabs(initial_step); // nstep always negative,
								  // always minimize otherwise solvopt() is buggy.

	error_code = OPT_ERR_OK;

		// set vector of options
	solvopt_options[0] = nstep;
	solvopt_options[1] = arg_tol;
	solvopt_options[2] = fun_tol;
	solvopt_options[3] = maxiters;
	solvopt_options[4] = 0;
	solvopt_options[5] = gamma;
	solvopt_options[6] = dilation;
	solvopt_options[7] = gradstep;
	solvopt_options[8] = 0;
	solvopt_options[9] = 0;
	solvopt_options[10] = 0;
	solvopt_options[11] = maxevaluations;

	int everyTotStep = 1;
	int breakCicle = 0;

	if (!this->C_vars)
	{
			// allocate variables vector
		if (xv)
			delete [] xv;
		xv = new double[nv+3];  // or...[nv] :-)

			// populate vector with starting approximation
			// system->vector
	    //	System_to_Vars(xv);
	}

	double inires = this->Eval_fx(xv);

		// call solver !!!!

	opt_fx = solvopt(nv, //numero di dimensioni
                  xv, //punto iniziale
                  CalcFO, //funzione che calcola la funzione obiettivo di intersect
                  null_entry_solv_opt, //nessuna gradiente
                  solvopt_options,
                  (void *)this,
                  showVarFun,
                  everyTotStep,
                  &breakCicle,
				  err_code);

		// return auxiliary results and infos
	fx_evaluations = (long)solvopt_options[9];
	grad_evaluations = (long)solvopt_options[10];
	iters_done = (long)solvopt_options[8];


		// parse the error message
	/*
	switch (err_code)
	{
	case 0: sprintf (err_message, "OK: objective function optimized in %d steps", iters_done); break;
	case 2: strcpy  (err_message, "Error: insufficient memory for NR optimization"); break;
	case 3: strcpy  (err_message, "Error: objective function equals infinity"); break;
	case 4: strcpy  (err_message, "Error: zero gradient in local optimization"); break;
	case 7: strcpy  (err_message, "Warning: number of iterations exceeded the limit"); break;
	case 8: strcpy  (err_message, "Warning: user imposed stop"); break;
	}
	*/

		// delete variables vector
	if (!this->C_vars)
	{
		// Vars_to_System(xv);
		if (xv)
			delete [] xv;
		xv = NULL;
	}

	return TRUE;
}





///////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
///
/// Ch_genetic_optimizer
///           global optimization engine - member functions
///

// build/destroy genotypes objects (the individuals)

ChGenotype::ChGenotype(int number_of_genes)
{
	genes = new ChMatrixDynamic<>(number_of_genes, 1);	// instance matrix of genes
	fitness = 0;
	rel_fitness = 0;
	cum_fitness = 0;
	need_eval  = TRUE;
}

ChGenotype::~ChGenotype()
{
	delete (genes); genes = NULL;
}

void ChGenotype::Copy(ChGenotype* source)
{
	genes->CopyFromMatrix(*source->genes);
	fitness = source->fitness;
	rel_fitness = source->rel_fitness;
	cum_fitness = source->cum_fitness;
	need_eval = source->need_eval;
}


// build/destroy the optimizer engine

ChOptimizerGenetic::ChOptimizerGenetic()
{
	// first, build parent class
	//Ch_optimizer::Ch_optimizer();

	// initialize
	popsize = 50;
	population = NULL;
	best_indiv = new ChGenotype(1);
	max_generations = 100;
	selection = SELEC_ROULETTEBEST;
	crossover = CROSSOVER_BLEND;
	mutation = MUTATION_UNIFORM;
	elite = ELITE_FALSE;
	crossv_change = CRO_CHANGE_NULL;
	crossv_changeto = CROSSOVER_BLEND;
	crossv_changewhen = 40;
	average = 0;
	stdeviation = 0;
	min_fitness = 0;
	max_fitness = 0;
	generations_done = 0;
	mutation_prob = 0.006;
	crossover_prob = 0.3;
	speciation_mating = FALSE;
	incest_taboo = TRUE;
	replacement = REPLA_PARENTS;
	eugenetics = 0.0;

	stop_by_stdeviation = FALSE;
	stop_stdeviation = 0.0001;
	stop_by_fitness = FALSE;
	stop_fitness = 0;

	his_average = NULL;
	his_stdeviation = NULL;
	his_maxfitness = NULL;
	his_minfitness = NULL;
}

// destroy
ChOptimizerGenetic::~ChOptimizerGenetic()
{
	// first, delete parent class
	// Ch_optimizer::~Ch_optimizer();

	// delete the instance of best individual
	delete best_indiv;
}


// copy
void ChOptimizerGenetic::Copy(ChOptimizerGenetic* source)
{
	// first, copy  parent class
	ChOptimizer::Copy(source);

	population = NULL;
	best_indiv->Copy(source->best_indiv);

	popsize = source->popsize;
	max_generations = source->max_generations;
	selection = source->selection;
	crossover = source->crossover;
	mutation = source->mutation;
	mutation_prob = source->mutation_prob;
	crossover_prob = source->crossover_prob;
	elite = source->elite;
	crossv_change =  source->crossv_change ;
	crossv_changeto =  source->crossv_changeto ;
	crossv_changewhen =  source->crossv_changewhen ;
	speciation_mating = source->speciation_mating;
	incest_taboo = source->incest_taboo;
	replacement = source->replacement;
	eugenetics = source->eugenetics;

	stop_by_stdeviation = source->stop_by_stdeviation;
	stop_stdeviation = source->stop_stdeviation;
	stop_by_fitness = source->stop_by_fitness;
	stop_fitness = source->stop_fitness;

	average = 0;
	stdeviation = 0;
	min_fitness = 0;
	max_fitness = 0;
	generations_done = 0;

	his_average = NULL;
	his_stdeviation = NULL;
	his_maxfitness = NULL;
	his_minfitness = NULL;
}





// Genetic operations

int ChOptimizerGenetic::CreatePopulation(ChGenotype**& my_population, int my_popsize)
{
	// create the array of pointers;
	my_population = (ChGenotype**)  calloc (my_popsize, sizeof(ChGenotype*));

	// create the individuals (the genotypes are set to default)
	ChGenotype* mygen;
	int nvars = GetNumOfVars();
	for (int i = 0; i < my_popsize; i++ )
	{
		mygen = new ChGenotype(nvars);
		my_population[i] = mygen;
	}
	return TRUE;
}

int ChOptimizerGenetic::DeletePopulation(ChGenotype**& my_population, int my_popsize)
{
	if (my_population == NULL) return FALSE;

	// delete all the individuals (the genotypes)
	ChGenotype* mygen;
	for (int i = 0; i<my_popsize; i++)
	{
		mygen = my_population[i];
		delete (mygen);
	}

	// delete the array of pointers
	free(my_population);
	my_population = NULL;

	return TRUE;
}

int ChOptimizerGenetic::InitializePopulation()
{
	int nv= GetNumOfVars();
	int mind;
	int varindex;
	double mvalue, a1, a2;

	if (population == NULL) return FALSE;
	if (population[0]->genes->GetRows() != nv) return FALSE;

	for (mind = 0; mind < popsize; mind++)
	{
		varindex = 0;
		population[mind]->need_eval = TRUE;
		for (varindex = 0; varindex <nv ;varindex ++)
		{
			a1 = this->xv_inf[varindex];		// the random value of each variable
			a2 = this->xv_sup[varindex];		// must lie within the max/min limits
			mvalue = a1 + (ChRandom() * (a2-a1));
			population[mind]->genes->SetElement(varindex,0, mvalue);
			varindex++;
		}
	}

	return TRUE;
}

double ChOptimizerGenetic::ComputeFitness(ChGenotype* mygen)
{
	int nv= GetNumOfVars();
	int mvar;
	double mfitness= 0;
	// create temporary array of variables for function-parameter-passing purposes.
	double* myvars = (double*) calloc (nv, sizeof(double));

	// fill the vector of variables as double* array
	for (mvar = 0; mvar <nv; mvar++)
		{ myvars[mvar] = mygen->genes->GetElement(mvar,0); }

			// impose these variables to the system,
			//  Vars_to_System(myvars);
	// evaluate functional,
	mfitness = this->Eval_fx(myvars);	// ++++++ HERE THE FITNESS IS EVALUATED

	mygen->fitness = mfitness;
	// set flag for speed reasons..
	mygen->need_eval = FALSE;

	free (myvars); // delete the array of variables

	return mfitness;
}

int ChOptimizerGenetic::ComputeAllFitness()
{
	int nv= GetNumOfVars();
	int mvar, mind;
	// create temporary array of variables for function-parameter-passing purposes.
	double* myvars = (double*) calloc (nv, sizeof(double));

	for (mind = 0; mind < popsize; mind++)
	{
		if (population[mind]->need_eval)
		{
			// fill the vector of variables as double* array
			for (mvar = 0; mvar <nv; mvar++)
				{ myvars[mvar] = population[mind]->genes->GetElement(mvar,0); }
				 // impose these variables to the system,
				 // Vars_to_System(myvars);
			// evaluate functional,
			population[mind]->fitness = this->Eval_fx(myvars);	// ++++++ HERE THE FITNESS IS EVALUATED
			// set flag for speed reasons..
			population[mind]->need_eval = FALSE;
			// user break?
			if (this->user_break) break;
		}
	}
	free (myvars); // delete the array of variables

	return TRUE;
}


ChGenotype* ChOptimizerGenetic::Select_best(ChGenotype** my_population)
{
	ChGenotype* mselected = my_population[0];
	double mbestfitness =  my_population[0]->fitness;

	for (int i = 0; i < popsize; i++)
	{
		if (fabs(my_population[i]->fitness) != OPT_IMPOSSIBLE)
			if (my_population[i]->fitness > mbestfitness)
			{
				mselected  = my_population[i];
				mbestfitness = my_population[i]->fitness;
			}
	}
	// default .
	return mselected;
}

ChGenotype* ChOptimizerGenetic::Select_worst(ChGenotype** my_population)
{
	ChGenotype* mselected = my_population[0];
	double mworstfitness = my_population[0]->fitness;

	for (int i = 0; i < popsize; i++)
	{
		if (fabs(my_population[i]->fitness) != OPT_IMPOSSIBLE)
			if (my_population[i]->fitness < mworstfitness)
			{
				mselected  = my_population[i];
				mworstfitness = my_population[i]->fitness;
			}
	}
	// default .
	return mselected;
}

double ChOptimizerGenetic::Get_fitness_interval(ChGenotype** my_population)
{
	return (Select_best(my_population)->fitness - Select_worst(my_population)->fitness);
}


int ChOptimizerGenetic::PopulationStats(double& average, double& max, double& min, double& stdeviation)
{
	average = 0;
	max = -999999999;
	min = +999999999;
	stdeviation = 0;
	double variance = 0;
	int i;
	int sumind = 0;

	for (i = 0; i < popsize; i++)
	{
		if (fabs(population[i]->fitness) != OPT_IMPOSSIBLE)
		{
			sumind ++;
			average += population[i]->fitness;
			if (population[i]->fitness > max) max = population[i]->fitness;
			if (population[i]->fitness < min) min = population[i]->fitness;
		}
	}
	average = average / (double)sumind;

	for (i = 0; i < popsize; i++)
	{
		if (fabs(population[i]->fitness) != OPT_IMPOSSIBLE)
		{
			variance += pow((population[i]->fitness - average), 2);
		}
	}
	variance = variance / (double)(sumind-1);
	stdeviation = sqrt(variance);

	return TRUE;
}


ChGenotype* ChOptimizerGenetic::Select_roulette(ChGenotype** my_population)
{
	ChGenotype* mselected = my_population[0];

	// compute sum of all fitness, relative to worst.
	double msum = 0;
	double norm_fitness;
	double minf_fit = Select_worst(my_population)->fitness;
	double msup_fit = Select_best(my_population)->fitness;
	double clamp_fit = minf_fit + this->eugenetics*(msup_fit - minf_fit);

    int i;
	for (i = 0; i < popsize; i++)
	{
		if (fabs(my_population[i]->fitness) != OPT_IMPOSSIBLE)
		{
			norm_fitness = my_population[i]->fitness - clamp_fit;
			if (norm_fitness < 0) norm_fitness = 0;
			msum = msum + norm_fitness;
		}
	}

	double number = ChRandom() * msum;	// the sorted-out number!
	double partialsum = 0;
	for (i = 0; i < popsize; i++)
	{
		if (fabs(my_population[i]->fitness) != OPT_IMPOSSIBLE)
		{
			norm_fitness = my_population[i]->fitness - clamp_fit;
			if (norm_fitness < 0) norm_fitness = 0;
			partialsum = partialsum + norm_fitness;
			if (partialsum >= number)
				return my_population[i]; // the number was ok.
		}
	}
	// default .
	return mselected;
}


int ChOptimizerGenetic::Selection()
{
	// create the selected population:
	ChGenotype** selected_population;
	CreatePopulation(selected_population, popsize);
	int i;

	// move the good elements into the new temp array of selected elements
	switch (this->selection)
	{
	case SELEC_ROULETTE:
		for (i = 0; i < popsize; i++)
			selected_population[i]->Copy(Select_roulette(population));
		break;
	case SELEC_ROULETTEBEST:
		for (i = 0; i < popsize; i++)
			selected_population[i]->Copy(Select_roulette(population));
		Select_worst(selected_population)->Copy(Select_best(population));
		break;
	default:
		for (i = 0; i < popsize; i++)
			selected_population[i]->Copy(Select_roulette(population));
	}

	// set the tmp_population as new population, deleting the old.
	DeletePopulation(population, popsize);
	population = selected_population;

	return TRUE;
}


int ChOptimizerGenetic::ApplyCrossover (ChGenotype* par1, ChGenotype* par2, ChGenotype& child1, ChGenotype& child2)
{
	int nvars = par1->genes->GetRows();
	int mv;
	double fen1, fen2, newfen1, newfen2, w1, w2;
	ChMatrixDynamic<> mtemp (nvars,1);

	switch (this->crossover)
	{
	case CROSSOVER_DISABLED:
		// do not perform crossover, return same as parent without need of evaluation
		child1.Copy(par1);
		child2.Copy(par2);
		child1.need_eval = FALSE;
		child2.need_eval = FALSE;
		return TRUE; // %%%%

	case CROSSOVER_ARITMETIC:
		// 'Aritmetic' crossover:
		// average of fenotypes with random wheight
		for (mv = 0; mv < nvars; mv++)
		{
			w1 = ChRandom();
			w2 = ChRandom();
			fen1 = par1->genes->GetElement(mv, 0);
			fen2 = par2->genes->GetElement(mv, 0);
			newfen1 = fen1 * w1 + fen2 * (1-w1);
			newfen2 = fen1 * w2 + fen2 * (1-w2);
			child1.genes->SetElement(mv,0, newfen1);
			child2.genes->SetElement(mv,0, newfen2);
		}
		break;

	case CROSSOVER_BLEND:
		// 'Blend' crossover:
		// linear average of two fenotypes with constant weights 0.3 and 0.7 (children are linear
		// interpolation of parents 0...0.3...0.7...1)
		w1 = 0.3;
		w2 = 0.7;
		for (mv = 0; mv < nvars; mv++)
		{
			fen1 = par1->genes->GetElement(mv, 0);
			fen2 = par2->genes->GetElement(mv, 0);
			newfen1 = fen1 * w1 + fen2 * (1-w1);
			newfen2 = fen1 * w2 + fen2 * (1-w2);
			child1.genes->SetElement(mv,0, newfen1);
			child2.genes->SetElement(mv,0, newfen2);
		}
		break;
	case CROSSOVER_BLEND_RANDOM:
		// 'Blend random' crossover:
		// linear average of two fenotypes with random weights 0.3 and 0.7 (children are linear
		// interpolation of parents 0...rnd...rnd...1)
		w1 = ChRandom();
		w2 = ChRandom();
		for (mv = 0; mv < nvars; mv++)
		{
			fen1 = par1->genes->GetElement(mv, 0);
			fen2 = par2->genes->GetElement(mv, 0);
			newfen1 = fen1 * w1 + fen2 * (1-w1);
			newfen2 = fen1 * w2 + fen2 * (1-w2);
			child1.genes->SetElement(mv,0, newfen1);
			child2.genes->SetElement(mv,0, newfen2);
		}
		break;
	case CROSSOVER_HEURISTIC:
		// 'heuristic crossover' extrapolates the child in the direction of
		// the parent with best fitness
		ChGenotype* lead_par;
		ChGenotype* slave_par;
		if (par1->fitness >= par2->fitness)
			{lead_par = par1; slave_par = par2;}
		else
			{lead_par = par2; slave_par = par1;}

		for (mv = 0; mv < nvars; mv++)
		{
			fen1 = lead_par->genes->GetElement(mv, 0);
			fen2 = slave_par->genes->GetElement(mv, 0);

			w1 = ChRandom();
			newfen1 = fen1 + w1*(fen1-fen2);
			if (newfen1 > this->xv_sup[mv]) newfen1 = this->xv_sup[mv];
			if (newfen1 < this->xv_inf[mv]) newfen1 = this->xv_inf[mv];
			child1.genes->SetElement(mv,0, newfen1);

			w2 = ChRandom();
			newfen2 = fen1 + w2*(fen1-fen2);
			if (newfen2 > this->xv_sup[mv]) newfen2 = this->xv_sup[mv];
			if (newfen2 < this->xv_inf[mv]) newfen2 = this->xv_inf[mv];
			child2.genes->SetElement(mv,0, newfen2);
		}
		break;

	default:
		break;
	}

	// set flags: the chromosomes has been changed,
	// and fitness should be computed!
	child1.need_eval = TRUE;
	child2.need_eval = TRUE;

	return TRUE;
}

int ChOptimizerGenetic::Crossover()
{
	int nv= GetNumOfVars();
	ChGenotype* par1;
	ChGenotype* par2;
	ChGenotype child1(nv);
	ChGenotype child2(nv);
	ChGenotype bestparent(nv);
	int selnum1 = 0;
	int selnum2 = 0;


	par1= NULL;
	par2= NULL;
/*
	int marriage;
	int ind1, ind2;
	int maxmarriages = (int)(((double)popsize)*crossover_prob);
	for (marriage = 0; marriage < maxmarriages; marriage++)
	{
		ind1 = (int)(floor(ChRandom()*(double)popsize));
		ind2 = (int)(floor(ChRandom()*(double)popsize));
		par1 = population[ind1];
		par2 = population[ind2];
		// perform the crossover to find child1, child2
		ApplyCrossover (par1, par2, child1, child2);
		// replace parents with children data
		par1->Copy(&child1);
		par2->Copy(&child2);
	}
*/

	for (int i = 0; i < popsize; i++)
	{
		if (ChRandom() <= this->crossover_prob)
		{
			if ((par1 == NULL)&&(par2 == NULL))
				par1 = population[i];
			else
				if ((par1 != NULL)&&(par2 == NULL))
					par2 = population[i];
		}

		if ((par1 != NULL)&&(par2 != NULL))
		{
///if (par1->genes->Equals(par2->genes))
////cout << "\n                    --incest";
			// perform the crossover to find child1, child2
			ApplyCrossover (par1, par2, child1, child2);

			// replacement of children data  ##########
			switch (replacement) {
			case REPLA_PARENTS:
				par1->Copy(&child1);
				par2->Copy(&child2);
				break;
			case REPLA_WORST:
				Select_worst(this->population)->Copy(&child1);
				Select_worst(this->population)->Copy(&child2);
				break;
			default:
				break;
			}

			// restore pointer for next marriage in for loop...
			par1 = NULL;
			par2 = NULL;
		}
	}

	return TRUE;
}


int ChOptimizerGenetic::Mutation()
{
	int nv= GetNumOfVars();
	double a1, a2, mutval;
	int had_mutation;

 	for (int i = 0; i < popsize; i++)
	{
		had_mutation = FALSE;
		for (int mvar = 0; mvar < nv; mvar++)
		{
			if (ChRandom() <= this->mutation_prob)
			{
				// MUTATION of the fenotype variable...
				// Find the variable structure in variable linked list, with limits
				a1 = this->xv_inf[mvar];	// the random value of each variable
				a2 = this->xv_sup[mvar];	// must lie within the max/min
				// Perform mutation:
				switch (this->mutation)
				{
				case MUTATION_UNIFORM:
					mutval = a1 + ChRandom() * (a2- a1);
					break;
				case MUTATION_BOUNDARY:
					if (ChRandom() < 0.5) mutval = a1;
					else                  mutval = a2;
					break;
				}

				// store the mutated allele:
				population[i]->genes->SetElement(mvar,0,mutval);

				// the chromosomes have been changed, remember that
				// new fitness will be computed:
				population[i]->need_eval = TRUE;

				had_mutation = TRUE;
			}
		}
		if (had_mutation) this->mutants++;
	}


	return TRUE;
}


int ChOptimizerGenetic::LogOut(int filelog)
{
	if (his_average)
		if (his_average->GetRows()>=generations_done)
			his_average->SetElement(generations_done-1,0,average);
	if (his_stdeviation)
		if (his_stdeviation->GetRows()>=generations_done)
			his_stdeviation->SetElement(generations_done-1,0,stdeviation);
	if (his_maxfitness)
		if (his_maxfitness->GetRows()>=generations_done)
			his_maxfitness->SetElement(generations_done-1,0,max_fitness);
	if (his_minfitness)
		if (his_minfitness->GetRows()>=generations_done)
			his_minfitness->SetElement(generations_done-1,0,min_fitness);

	if (filelog)
	{
		ChLog::eChLogLevel oldfilemode = GetLog().GetCurrentLevel();
		GetLog().SetCurrentLevel(ChLog::CHMESSAGE);

		GetLog() << "\n\nGENERATION ------- n. ";
		GetLog() << (int)generations_done;
		GetLog() << "\n  STATISTICS:";
		GetLog() << "\n     max:   ";		GetLog() << max_fitness;
		GetLog() << "\n     min:   ";		GetLog() << min_fitness;
		GetLog() << "\n     media: ";		GetLog() << average;
		GetLog() << "\n     stdev: ";		GetLog() << stdeviation;
		GetLog() << "\n  Fitness of best individual ever born: ";
		GetLog() << best_indiv->fitness;
		GetLog() << "\n";

		GetLog().SetCurrentLevel(oldfilemode);
	}
	return TRUE;
}


int ChOptimizerGenetic::DoOptimize()
{
	GetLog() << "\n\n\nGENETIC OPTIMIZATION STARTED.............\n\n";

	int nv= GetNumOfVars();

	// allocate -if needed- the upper-lower boundaries arrays
/*
	if (!this->C_vars)
	{
		this->xv_sup = (double*) calloc (nv, sizeof(double));
		this->xv_inf = (double*) calloc (nv, sizeof(double));
		int mvar = 0;

		for (ChOptVar* Vovar = optvarlist; Vovar != NULL; Vovar = (ChOptVar*) Vovar->GetNext())
		{
			this->xv_sup[mvar] = Vovar->GetLimSup();
			this->xv_inf[mvar] = Vovar->GetLimInf();
			mvar++;
		}
	}
*/

	this->CreatePopulation(population, popsize);
	this->InitializePopulation();

	this->mutants = 0;

		// avoid complete streaming of log during optimization;
	ChLog::eChLogLevel oldfilemode = GetLog().GetCurrentLevel();
	GetLog().SetCurrentLevel(ChLog::CHQUIET);


	// -- COMPUTE FITNESS  (for all individuals of brand-new population, 1st time)
	this->ComputeAllFitness();

	//
	// THE CYCLE OF OPTIMIZATION,
	// GENERATION BY GENERATION
	//
	for (generations_done = 1; generations_done <= max_generations; generations_done++)
	{
		if (crossv_change == CRO_CHANGE_DATE)
			if(generations_done > crossv_changewhen) crossover = crossv_changeto;

		// -- store the best individual of old population
		best_indiv->Copy(Select_best(population));

		// -- set the value of best found value of FX
		opt_fx = best_indiv->fitness;

		// -- statistics...
		PopulationStats(average, max_fitness, min_fitness, stdeviation);

		// -- log
		LogOut(TRUE);


		// -- break cycle conditions
		if (stop_by_stdeviation)
			if (stdeviation <= stop_stdeviation)
			{
				sprintf (err_message, "OK, imposed standard deviation reached in %d generations", generations_done);
				break;
			}
		if (stop_by_fitness)
			if (max_fitness >= stop_fitness)
			{
				sprintf (err_message, "OK, imposed max fitness reached in %d generations", generations_done);
				break;
			}

		// -- user break?
		if (this->user_break)
		{
			if (*err_message == 0)
				sprintf (err_message, "OK, user break");
			break;
		}


		// -- CROSSOVER
		this->Crossover();

		// -- MUTATION
		this->Mutation();

		// -- COMPUTE FITNESS
		this->ComputeAllFitness();

		// -- SELECTION
		this->Selection();

		if (elite == ELITE_TRUE)
		{
			if (Select_best(population)->fitness < best_indiv->fitness)
			{
				Select_worst(population)->Copy(best_indiv);
			}
		}

	}

	GetLog().SetCurrentLevel(oldfilemode);


	if (generations_done >= max_generations)
		sprintf (err_message, "OK, all %d generations done", max_generations);

	// on termination, reset the system at the value of the fenotypes of the best indiv.

	GetLog() << "\n\nGENETIC OPTIMIZATION TERMINATED.";

	GetLog() << "\nResetting the system to the best genotype-variables found:";

	// resetting system  (or xv[] vector) to best individual
	int mvar;
	double* myvars = (double*) calloc (nv, sizeof(double));
	for (mvar = 0; mvar <nv; mvar++)
		{
			myvars[mvar] = best_indiv->genes->GetElement(mvar,0);
			GetLog() << "\n   "; 
			GetLog() << myvars[mvar];
		}
	
	memcpy (this->xv, myvars, (sizeof(double) * nv));
	//	Vars_to_System(myvars);

	free (myvars); // delete the array of variables

	GetLog() << "\n with fitness (objective fx obtained) equal to = ";
	GetLog() << best_indiv->fitness;
	GetLog() << "\n\n\n";

	// delete the population, useful anymore...
	this->DeletePopulation(population, popsize);

	if (!this->C_vars)
	{
		free (this->xv_sup);
		free (this->xv_inf);
	}

	return TRUE;
}






///////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
///
/// Ch_gradb_optimizer
///           Local optimization engine member functions
///


// build
ChOptimizerGradient::ChOptimizerGradient()
{
	// first, build parent class
	ChOptimizer::ChOptimizer();

	initial_step	=1;
	arg_tol		= 1.e-6;
	fun_tol		= 1.e-7;
	maxevaluations	= 800;
	maxgradients	= 50;
	maxdilationsteps	= 8;
	maxbisections	= 10;
	dilation	= 2.1;
	do_conjugate = FALSE;
}


// destroy
ChOptimizerGradient::~ChOptimizerGradient()
{
}


// copy
void ChOptimizerGradient::Copy(ChOptimizerGradient* source)
{
	// first, copy  parent class
	ChOptimizer::Copy(source);

	initial_step = source->initial_step;
	arg_tol = source->arg_tol;
	fun_tol = source->fun_tol;
	maxevaluations = source->maxevaluations;
	maxgradients = source->maxgradients;
	maxdilationsteps = source->maxdilationsteps;
	maxbisections = source->maxbisections;
	dilation = source->dilation;
	do_conjugate = source->do_conjugate;
}









//// OPTIMIZE FUNCTION  , locally with pseudo-NR method


int ChOptimizerGradient::DoOptimize()
{
	double nstep = initial_step;
	int dilationstep;
	int bisections;
	double fx, fxguess, fxmid, fx1, fx2, lastgood, normGold, normGnew;
	int update_grad;

	// count vars
	int nv = GetNumOfVars();

	nstep = initial_step;

	// allocate variables vector
	ChMatrixDynamic<> mX(nv,1);		// the variables
	ChMatrixDynamic<> mXguess(nv,1);// the guessed variables
	ChMatrixDynamic<> mG(nv,1);		// the gradient
	ChMatrixDynamic<> mGcn(nv,1);	// the conjugategradient
	ChMatrixDynamic<> mGcnold(nv,1);// the old conjugate gradient
	ChMatrixDynamic<> mDx(nv,1);	// the increment
	ChMatrixDynamic<> mXmid(nv,1);	// the bisection center
	ChMatrixDynamic<> mX1(nv,1);	// the bisection left
	ChMatrixDynamic<> mX2(nv,1);	// the bisection right

	// populate vector with starting approximation
	// system->vector

	for (int i = 0; i < nv; i++) { mX(i,0) = this->xv[i]; };
	//System_to_Vars(&mX);

	fx = Eval_fx(&mX);

	update_grad = TRUE;

	// perform iterative optimization..

	while(TRUE)
	{
		// #### compute gradient here;
		Eval_grad(&mX, &mG);

		// correction for conjugate gradient method?
		if (this->do_conjugate)
		{
			normGnew = mG.NormTwo();
			mGcn.CopyFromMatrix(mG);
			if (grad_evaluations > 1)
			{
				mGcnold.MatrScale((normGnew/normGold));
				mGcn.MatrInc(mGcnold);	// update conj.gradient direction
			}
			mGcnold.CopyFromMatrix(mGcn);
			normGold = normGnew;
			mG.CopyFromMatrix(mGcn);	// use the conjugate gradient!
		}

		// Forward dilation search;
		dilationstep = 0;
		lastgood = fx;
		while(TRUE)
		{
			if (dilationstep > maxdilationsteps) break;
			dilationstep++;

			// compute new fx position;
			mDx.CopyFromMatrix(mG);
			mDx.MatrScale(nstep);
			mXguess.MatrAdd(mX, mDx);

			fxguess = Eval_fx(&mXguess);

			if (fxguess <= lastgood)
			{
				break;
			}

			nstep *= dilation;
			lastgood = fxguess;
		}

		// Backward bisection search;
		bisections = 0;
		int gone;

		mDx.CopyFromMatrix(mG);// initialize mid
		mDx.MatrScale(nstep*0.5);
		mXmid.MatrAdd(mX, mDx);

		fxmid = Eval_fx(&mXmid);

		while(TRUE)
		{
			if (bisections > maxbisections) break;
			bisections++;
			if (fx_evaluations > maxevaluations) 	break;

			mDx.CopyFromMatrix(mG);// initialize left
			mDx.MatrScale(nstep*0.25);
			mX1.MatrAdd(mX, mDx);

			fx1 = Eval_fx(&mX1);

			mDx.CopyFromMatrix(mG);// initialize right
			mDx.MatrScale(nstep*0.75);
			mX2.MatrAdd(mX, mDx);

			fx2 = Eval_fx(&mX2);

			gone = FALSE;
			if ((fx1 <= fxmid)&&(fxmid <= fx2)&&(gone==FALSE))
			{
				mX.CopyFromMatrix(mXmid);		fx = fxmid;
				mXmid.CopyFromMatrix(mX2);		fxmid = fx2;
				gone = TRUE;
			}
			if ((fx1 >= fxmid)&&(fxmid >= fx2)&&(gone==FALSE))
			{
				mXguess.CopyFromMatrix(mXmid);	fxguess = fxmid;
				mXmid.CopyFromMatrix(mX1);		fxmid = fx1;
				gone = TRUE;
			}
			if ((fx1 <= fxmid)&&(fxmid >= fx2)&&(gone==FALSE))
			{
				mX.CopyFromMatrix(mX1);		fx = fx1;
				mXguess.CopyFromMatrix(mX2);	fxguess = fx2;
				gone = TRUE;
			}
			if (gone==FALSE)
			{
				nstep *= 0.25;
				break;
			}

			if (fabs(fxmid-fx)<= fun_tol)
			{
//cout << "\n              ---- ";
				sprintf (err_message, "OK,function tolerance reached.");
				goto end_opt;
			}
			// ***TO DO*** stop after "arg_tol" passed

			// half step, then repeat
			nstep *= 0.5;
		}

		if ((fx1 >= fx)&&(fx1 >= fxmid)&&(fx1 >= fx2)&&(fx1 >= fxguess))
			mX.CopyFromMatrix(mX1);
		if ((fx2 >= fx)&&(fx2 >= fxmid)&&(fx2 >= fx1)&&(fx2 >= fxguess))
			mX.CopyFromMatrix(mX2);
		if ((fxmid >= fx)&&(fxmid >= fx1)&&(fxmid >= fx2)&&(fxmid >= fxguess))
			mX.CopyFromMatrix(mXmid);
		if ((fxguess >= fx)&&(fxguess >= fx1)&&(fxguess >= fx2)&&(fxguess >= fxmid))
			mX.CopyFromMatrix(mXguess);

		fx = Eval_fx(mX.GetAddress());

		if (this->fx_evaluations > maxevaluations)
		{
//cout << "\n          limit on fx evals----= ";
			sprintf (err_message, "OK, limit on max number of fx evaluations reached in %d steps", this->grad_evaluations);
			break;
		}
		if (this->grad_evaluations > maxgradients)
		{
//cout << "\n          limit on max gradients ---- ";
			sprintf (err_message, "OK, limit on max number of %d gradient evaluations reached.", this->grad_evaluations);
			break;
		}
	}

	goto end_opt;

end_opt:
	// ***TO DO*** set the system at best variables among fx, fxguess, fx1, fx2;
	this->opt_fx = fx;

	if (this->C_vars)
	{
		for (int i = 0; i < nv; i++) { this->xv[i] = mX(i,0);};
	}  // copy result into vector xv

	return TRUE;
}




///////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
///
/// Ch_hybrid_optimizer
///           Hybrid (genetic+local) optimization engine
///


// build
ChOptimizerHybrid::ChOptimizerHybrid()
{
	// first, build parent class
	ChOptimizer::ChOptimizer();

	genetic_opt = new ChOptimizerGenetic();
	local_opt = new ChOptimizerLocal();

	current_phase = 0;
	use_genetic = TRUE;
	use_local = TRUE;
}


// destroy
ChOptimizerHybrid::~ChOptimizerHybrid()
{
	// delete the two incapsulated optimizers;
	delete genetic_opt;
	delete local_opt;
}


// copy
void ChOptimizerHybrid::Copy(ChOptimizerHybrid* source)
{
	// first, copy  parent class
	ChOptimizer::Copy(source);

	current_phase = source->current_phase;
	use_genetic = source->use_genetic;
	use_local = source->use_local;

	// copy everything also in the two optmimizers
	genetic_opt->Copy(source->genetic_opt);
	local_opt->Copy(source->local_opt);
}






//// OPTIMIZE FUNCTION  ,

int ChOptimizerHybrid::DoOptimize()
{

	// set common optimization settings
	genetic_opt->minimize = this->minimize;
	local_opt->minimize = this->minimize;
	genetic_opt->SetNumOfVars(this->C_vars);
	local_opt->SetNumOfVars(this->C_vars);
	genetic_opt->SetXv(this->xv);
	genetic_opt->SetXv_sup(this->xv_sup);
	genetic_opt->SetXv_inf(this->xv_inf);
	local_opt->SetXv(this->xv);
	local_opt->SetXv_sup(this->xv_sup);
	local_opt->SetXv_inf(this->xv_inf);

	genetic_opt->break_funct = this->break_funct;
	local_opt->break_funct = this->break_funct;
	genetic_opt->break_cycles = this->break_cycles;
	local_opt->break_cycles = this->break_cycles;


	// 1)  optimization with genetic method

	if (this->use_genetic)
	{
		this->current_phase = 1;

		if (!genetic_opt->Optimize())
		{
			strcpy (this->err_message, genetic_opt->err_message);
			return FALSE;
		}

		this->fx_evaluations = genetic_opt->fx_evaluations;
		this->grad_evaluations = genetic_opt->grad_evaluations;
		this->opt_fx = genetic_opt->opt_fx;
	}

	// 2)  optimization with gradient method

	if (this->use_local)
	{
		this->current_phase = 2;

		if (!local_opt->Optimize())
		{
			strcpy (this->err_message, local_opt->err_message);
			return FALSE;
		}

		this->fx_evaluations = genetic_opt->fx_evaluations + local_opt->fx_evaluations;
		this->grad_evaluations = genetic_opt->grad_evaluations + local_opt->grad_evaluations;
		this->opt_fx = local_opt->opt_fx;
	}

	this->current_phase = 0;

	// set class values after optimization
	strcpy (this->err_message, "Ok, hybrid optimization has been terminated");


	// on termination, reset the system at the value of the fenotypes of the best indiv.
	GetLog() << "\n\nHYBRID OPTIMIZATION TERMINATED.";
	GetLog() << "\nCurrent system variables after optimization:";
	int nv = this->GetNumOfVars();
	double* myvars = (double*) calloc (nv, sizeof(double));
	//this->System_to_Vars(myvars);
	for (int mvar = 0; mvar <nv; mvar++)
		{
			GetLog() << "\n   "; 
			GetLog() << myvars[mvar];
		}
	free (myvars); // delete the array of variables

	GetLog() << "\n with objective fx obtained equal to = ";
	GetLog() << this->opt_fx;
	GetLog() << "\n\n\n";

	return TRUE;
}


void  ChOptimizerHybrid::SetObjective (ChFx* mformula)
{
	ChOptimizer::SetObjective(mformula);
	genetic_opt->SetObjective(mformula);
	local_opt->SetObjective(mformula);
}
void  ChOptimizerHybrid::SetObjectiveGrad (ChFx* mformula)
{
	ChOptimizer::SetObjectiveGrad(mformula);
	genetic_opt->SetObjectiveGrad(mformula);
	local_opt->SetObjectiveGrad(mformula);
}

void ChOptimizerHybrid::SetNumOfVars(int mv)
{
	C_vars = mv;
	genetic_opt->SetNumOfVars(mv);
	local_opt->SetNumOfVars(mv);
};




//###################################################################
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
///  NR OPTIMIZER FUNCTION


#define slv_err1 "\nSolvOpt error: x has to be a vector of dimension >=2\n"
#define slv_err2 "\nSolvOpt error: insufficient memory available\n"
#define slv_err3 "\nSolvopt error: objective function equals infinity\n"
#define slv_wrn1 "\nSolvOpt warning: ZERO GRADIENT is returned"
#define slv_wrn2 "\nSolvOpt Warning: Resetting"
#define slv_wrn3 "\nSolvOpt Warning: trying random point"
#define slv_wrn4 "\nSolvOpt warning: Number of iterations exceeded the limit\n"
#define slv_wrn5 "\nSolvOpt warning: user imposed stop \n"
#define slv_wrn6 "\nSolvOpt warning: Number of function evaluations exceeded the limit\n"



double solvopt(unsigned int n,
               double x[],
               double  fun(double x[],void * idData),
               void  grad(double x[],double g[]),
               double options[],
               void *  idData,
               void (*showDisplay)(),
               int showEvery,
               int *breakCicle,
			   int& err_code)
{
/*
  solvopt  returns the optimum function value.

  Arguments to the function:
  x       is the n-vector, the coordinates of the starting point
          at a call to the subroutine and the optimizer at regular return,
  fun     is the entry name of an external function which computes the value
          of the function 'fun' at a point x.
          synopsis: double fun(double x[])
  grad    is the entry name of an external function which computes the gradient
          vector of the function 'fun' at a point x.
          synopsis: void grad(double x[],double g[])
  options is a vector of optional parameters (see the description in SOLVOPT.H).
        Returned optional values:
        options[8], the number of iterations, options[8]<0 means
                    an error occured
        options[9], the number of objective function evaluations, and
        options[10],the number of gradient evaluations.
  idData rappresenta un dato a 32 bit per contenere particolari informazioni
  showVarFun puntatore a funzione void (*)() per il display delle variabili
         chiamata in ogni ciclo
  showEvery ogni quanti cilci chiamare la funzione di show
    breakCicle puntatore ad intero, se diventa alto il ciclo viene interrotto



____________________________________________________________________________*/

      double default_options[12]=
             {-1.e0,1.e-6,1.e-7,5.e3,0.e0,2.001e0,2.5e0,1.e-100,0.e0,0.e0,0.e0, 1000.0};
      //void null_entry();
      unsigned short gammadj, app, reset, termin;
      double f,fi,f1,fopt,d,d1,dd,gamma,alpha,w,h1,h,hp,y,evstep;
      double dx,e2g,e2gt,e2z,e2g1,gnp=5.1e0,xrb,xlb;
      const double zero=0.e0, one=1.e0, eps=1.e-15, maxf=1.e100;
      double  *B;               /* space transformation matrix (allocatable)      */
      double *g,*g0,*g1,*gt,*z,*x1; /* allocatable working arrays                     */
      //unsigned short k=0,           /* iteration counter                              */
      unsigned int   k=0,           /* iteration counter                              */
                     kcheck=0,      /* reset check counter                            */
                     ld,            /* iteraion number to print intermediate results  */
                     i,j,           /* loop counters                                  */
                     k1=0,k2=0,     /* 1-D search step counters                       */
                     kstop=0,       /* termination counter                            */
                     nstop=0,       /* zero-gradient events counter                   */
                     kg=0,kstore=3,nzero=0,
                     nsteps[3]={0,0,0}; /* counters used for step size change         */
      int countShowFun=0;

	  err_code = 0;		// alex

/* Check the dimension: */

      //if (n<2)
      if (n<1) //paolo, 28/11/98
      { //printf (slv_err1);
        options[8]=-one;
        return(0.);
      }

/* Allocate the memory for working arrays: */

      B=(double *)calloc(n*n+10,sizeof(double));
      g=(double *)calloc(n+10,sizeof(double));
     g0=(double *)calloc(n+10,sizeof(double));
     g1=(double *)calloc(n+10,sizeof(double));
     gt=(double *)calloc(n+10,sizeof(double));
      z=(double *)calloc(n+10,sizeof(double));
     x1=(double *)calloc(n+10,sizeof(double));

      if (B==NULL||g==NULL||g0==NULL||g1==NULL||gt==NULL||z==NULL||x1==NULL)
      {
         //printf (slv_err2);
         options[8]=-2.e0;
         return(0.);
      }

/* ANALIZE THE ARGUMENTS PASSED
   User-supplied gradients: */
      if (grad==null_entry_solv_opt) app=1; else app=0;

/* Norm of vector x:*/
      dx=zero; for (i=0;i<n;i++) dx+=x[i]*x[i]; dx=sqrt(dx);

/* Default values for options */
      for (i=0;i<=7;i++) if (options[i]==zero) options[i]=default_options[i];

      if (fabs(options[0])<=eps*dx)
      {
		  if (options[0]<zero) options[0]=-1.e-6*dx; else options[0]=1.e-6*dx;
      }
          if (options[1]<=eps) options[1]=default_options[1];
          if (options[2]<=eps) options[2]=default_options[2];
          if (options[5]==default_options[5]) gammadj=1;
          else
          {  gammadj=0;
             if (options[5]<1.2e0) options[5]=1.2e0;
             else {if (options[5]>3.e0) options[5]=3.e0;}
          }
          if (options[6]<1.5e0) options[6]=1.5e0;
       options[8]=default_options[8];
       options[9]=default_options[9];
       options[10]=default_options[10];

/* Minimize resp. maximize the objective function:*/
      if(options[0]<zero)h1=-one; else h1=one;
/* Multiplier for the matrix for the inverse of the space dilation: */
      w=one/options[6]-one;
/* Set other control parameters: */
      gamma=options[5];
      if (options[4]<zero && options[4]!=-one) options[4]=zero;
      ld=(unsigned int)floor(options[4]+0.1);

	  if (options[11]<=0.0) { options[11]=default_options[11];};

/*--------------------------------------------------------------------
RESETTING CYCLE */

   while (1)
   {

/* COMPUTE THE OBJECTIVE FUNCTION: */

      f=fun(x,idData);  options[9]+=one;	// +++++++++
      if (fabs(f)>maxf)
      {  //if (options[4]!=-one)
         //  ; //printf (slv_err3);
         options[8]=k;
		 err_code = 3;
		 goto endrun;
      }

/* COMPUTE THE GRADIENT: */
      if (app)
      {     dd=ChMax(1.e-7/pow((k+1),1.5),options[7]);
            for (i=0;i<n;i++)
            {  d=dd*ChMax(fabs(x[i]),1.e-4);
               y=x[i]; x[i]=x[i]+d;
			   fi=fun(x,idData);			// ++++++++
			   g[i]=(fi-f)/d; x[i]=y;
            }
            options[9]=options[9]+n;
			if (options[9]>=options[11]) {err_code = 11; goto endrun;}
      }
      else  { grad(x,g); options[10]=options[10]+one; }
/* Store the gradient in a new array: */
      for (i=0;i<n;i++) g1[i]=g[i];
/* Compute the norm of the gradient: */
      e2g=zero; for (i=0;i<n;i++) e2g+=g[i]*g[i]; e2g=sqrt(e2g);
/* Check the norm of the gradient:   */
      if (e2g<eps)
      {  // if (options[4]!=-one)
		 //  ; //printf (slv_wrn1);
		 err_code = 4;
         options[8]=k;
		 goto endrun;
      }
/*-----------------------------------------------------------------
  INITIAL TRIAL STEP SIZE and RE-INITIALIZATION of B:  */

      if (k==0) { h=options[0]/(log10(e2g+one)/log10(2.e0));  dx=fabs(h)*3.e0;}
      for (i=0;i<n;i++) { for (j=0;j<n;j++) B[i*n+j]=zero; B[i*n+i]=one; }



/*-----------------------------------------------------------------
  ITERATION CYCLE   */

   while (1)
   {
	  k+=1; kcheck+=1;

/* Gradient in the transformed space (gt) and the difference (z): */
      e2gt=zero; e2z=zero;
      for (i=0;i<n;i++)
      {  d=zero; for (j=0;j<n;j++) d+=B[j+i*n]*g[j];
         gt[i]=d;       e2gt+=d*d;
         z[i]=d-g1[i];  e2z+=z[i]*z[i];
      }
      e2gt=sqrt(e2gt); e2z=sqrt(e2z);

/* Adjust gamma  */

      if (gammadj) gamma=pow(1.2e0,ChMax(one,log10(e2g+one)));

/* Calculate alpha: */

      alpha=options[6];
      d=zero; for (i=0;i>n;i++) d+=g[i]*gt[i];  d/=e2g*e2gt;
      if (d<-0.999e0) alpha*=1.2e0;
      if (gamma>2.5e0 && gammadj) alpha+=(gamma-2.5e0)/2.e0;
      w=one/alpha-one;

/* Check the norm of the difference: */

      if (e2z>1.e-31) for (i=0;i<n;i++) z[i]/=e2z;
      else for (i=0;i<n;i++) z[i]=zero;

/* Make a space transformation:
   g1=gt+w*(z*gt')*z: */
        d=zero; for (i=0;i<n;i++) d+=z[i]*gt[i];
        e2g1=zero; d*=w;
        for (i=0;i<n;i++)
        { g1[i]=gt[i]+d*z[i];
          e2g1+=g1[i]*g1[i];
        } e2g1=sqrt(e2g1);

/*-----------------------------------------------------------------
  CHECK FOR THE NEED OF RESETTING   */

      reset=1;
      for (i=0;i<n;i++)
         {if (fabs(g[i])>eps) if (fabs(gt[i])/fabs(g[i])>=eps) reset=0;}
      if (e2z==zero && kcheck>2) reset=1;
      if (reset)
      {   // if (options[4]!=-one)
		  //	printf (slv_wrn2);
		  kcheck=0; h=h1*dx/6.e0; k=k-1; kg=0; for (i=0;i<3;i++) nsteps[i]=0;
          kstop=0;  exit(0);
      }

      for (i=0;i<n;i++) gt[i]=g1[i]/e2g1;

/* New inverse matrix: B = B ( I + (1/alpha -1)zz' ) */

      for (i=0;i<n;i++)
      {    d=zero;   for (j=0;j<n;j++) d+=B[j*n+i]*z[j];
           d*=w;     for (j=0;j<n;j++) B[j*n+i]+=d*z[j];
      }

/* Gradient in the non-transformed space: g0 = B' * gt   */
      for (i=0;i<n;i++)
      {    d=zero;   for (j=0;j<n;j++) d+=B[j*n+i]*gt[j];
           g0[i]=d;
      }

/* STORE THE CURRENT VALUES AND SET THE COUNTERS FOR 1-D SEARCH  */

      for (i=0;i<n;i++) z[i]=x[i]; hp=h; fopt=f; k1=0; k2=0;

/* 1-D SEARCH   */

      while (1)
      {  for (i=0;i<n;i++) x1[i]=x[i]; f1=f;

/* Next point:   */
         for (i=0;i<n;i++) x[i]+=hp*g0[i];

/* COMPUTE THE FUNCTION VALUE AT A POINT:  */

         f=fun(x,idData);  options[9]+=one;		// ++++++++
		 if (options[9]>=options[11]) {err_code = 11; goto endrun;}
         if (fabs(f)>maxf)
         {  // if (options[4]!=-one)
            //		printf (slv_err3);
			err_code = 3;
            options[8]=k;
			goto endrun;
         }

/* CHECK FOR THE NEED TO USE A SMALLER STEP   */

         if (fabs(f)<eps) { d=f/eps; d1=f1/eps; } else { d=f; d1=f1;}
         if (d1<zero) dd=-one; else {if (d1>zero) dd=one; else dd=zero; }
         if ( h1*d  <  h1*pow(gamma,dd)*d1 )
            {if (e2g>zero) { k2+=1; for (i=0;i<n;i++) x[i]=x1[i]; hp/=gnp; f=f1; k1=0; }}
         else
         {
/* Has the 1-D optimizer been passed?   */
            if (h1*d<=h1*d1) break;
/* Adjust a trial step size   */
            if (k2==0)  k1+=1;
            if (k1>=20) hp*=2.e0;
            else
            {   if (k1>=10) hp*=1.25e0;
                else
                {  if (k1>=5)  hp*=1.1e0;
                   else if (k1>=2)   hp*=1.05e0;
         }  }   }
      }
/* ------------------------  End of 1-D search  ------------------  */

/* Store the number of steps made and calculate the average number  */

      if (kg<kstore) kg+=1;
      if (kg>=2) for (i=kg-1;i>0;i--) nsteps[i]=nsteps[i-1];
      nsteps[0]=k1;
      j=0; for (i=0;i<kg;i++) j+=nsteps[i]; evstep=j/kg;

/* Adjust the trial step size  */

      if (evstep<1.75e0)       h/=pow(1.7e0,2.e0-evstep);
      else if (evstep>2.25e0)  h*=pow(1.1e0,evstep-one);
      dx=zero; for (i=0;i<n;i++) dx+=(z[i]-x[i])*(z[i]-x[i]); dx=sqrt(dx);

/* COMPUTE THE GRADIENT */

      if (app) {
            dd=ChMax(1.e-7/pow((k+1),1.5),options[7]);
            for (i=0;i<n;i++) {
               d=dd*ChMax(fabs(x[i]),1.e-4);
               y=x[i]; x[i]=x[i]+d;
			   fi=fun(x,idData);			// +++++++++
			   g[i]=(fi-f)/d;   x[i]=y;
            }
            options[9]=options[9]+n;
			if (options[9]>=options[11]) {err_code = 11; goto endrun;}
      }
      else  { grad(x,g); options[10]=options[10]+one; }

/* Compute the norm of the gradient:   */

      e2g=zero; for (i=0;i<n;i++) e2g+=g[i]*g[i]; e2g=sqrt(e2g);

/* Check the norm of the gradient: */

    if (e2g<eps)
    {   // if (options[4]!=-one)
        //    printf (slv_wrn1);
        nzero+=1;

      if (nzero<=10)
      {
         //g=g0; //prima era cosi' Se voleno fare una copia la facciamo con un for
         for(i=0;i<n;i++)  //paolo, 29/11/98
           g[i]=g0[i];

         e2g=zero; for (i=0;i<n;i++) e2g+=g[i]*g[i]; e2g=sqrt(e2g);
         dx=fabs(h);
         if (k>nzero+1) h*=2.e0;    /*Enlarge step size to pass through*/
         else
         {    h/=2.e0;              /*Reduce step size to find better X*/
              for (i=0;i<n;i++) x[i]=x1[i];
              for (i=0;i<n;i++) z[i]=x1[i]+h;
         }
      }
      else
      {  /* Use a random point: */
          // if (options[4]!=-one)
          //       printf(slv_wrn3);
          if (k>nzero+1) for (i=0;i<n;i++) x[i]=-x[i];
          else for (i=0;i<n;i++) x[i]*=.9e0;
          nzero = 0;

          /* COMPUTE THE OBJECTIVE FUNCTION: */
          f=fun(x,idData);  options[9]+=one;		// ++++++++
		  if (options[9]>=options[11]) {err_code = 11; goto endrun;}
          if (fabs(f)>maxf)
          {  //if (options[4]!=-one)
             //   printf (slv_err3);
			 err_code = 3;
             options[8]=k;
			 goto endrun;
          }

          /* COMPUTE THE GRADIENT: */
          if (app) {
                dd=ChMax(1.e-7/pow((k+1),1.5),options[7]);
                for (i=0;i<n;i++) {
                   d=dd*ChMax(fabs(x[i]),1.e-4);
                   y=x[i]; x[i]=x[i]+d;
				   fi=fun(x,idData);			// ++++++++
				   g[i]=(fi-f)/d; x[i]=y;
                }
                 options[9]=options[9]+n;
				 if (options[9]>=options[11]) {err_code = 11; goto endrun;}
          }
          else  { grad(x,g); options[10]=options[10]+one; }
          /* Compute the norm of the gradient:   */
          e2g=zero; for (i=0;i<n;i++) e2g+=g[i]*g[i]; e2g=sqrt(e2g);
          /* Check the norm of the gradient: */
          if (e2g<eps)
          { // if (options[4]!=-one)
            //     printf (slv_wrn1);
			err_code = 4;
            options[8]=k;
			goto endrun;
          }
       }  /* nzero check   */
     }  /* zero gradient  */
/*-----------------------------------------------------------------
  DISPLAY THE CURRENT VALUES: */
       if (k==ld)
       {  /*printf ("\nIteration # ..... Function Value ..... "
                  "Step Value ..... Gradient Norm"
                  "\n     %5i     %13.5g      %13.5g       %13.5g",k,f,dx,e2g);
          */
         ld=(unsigned int)(k+floor(options[4]+.1));
       }
/*-----------------------------------------------------------------
  CHECK THE STOPPING CRITERIA: */
       xrb=zero; for (i=0;i<n;i++) if (xrb<fabs(x[i])) xrb=fabs(x[i]);
       if (xrb>1.e-8) xlb=ChMax(1.e-8,1.e-8/xrb);
       else xlb=1.e-8;
       /* CRITERION FOR THE ARGUMENT :*/
       termin=1;
       for (i=0;i<n;i++)
       {
		   if (fabs(x[i])>xlb)
           {
			 //if (fabs(z[i]-x[i])/fabs(x[i])>=options[1])	// modified by alex in the absolute err form...
			 if (fabs(z[i]-x[i]) >= options[1])
             {
				termin=0;
				break;
             }
           }
       }
       if (termin)
		   kstop+=1;
       /* CRITERION FOR THE FUNCTION:  */
       if (termin && ( (app && kstop>=1) || app==0  ) )
       {
		  if (fabs(f)>options[2]*options[2])
          {
			 // if (fabs((f-fopt)/f)<options[2] )	// modified by alex in the absolute err form...
			 if (fabs(f-fopt)<options[2] )
             {
				 options[8]=k;
				 goto endrun;
             }
          }
          else
          {
			  options[8]=k;
			  goto endrun;
          }
       }
/*-------------------------------------------------------------------
  LIMIT ON ITERATIONS  */

       if (k>(options[3]-0.1))
       {  options[8]=k;
          //printf (slv_wrn4);
		  err_code = 7;
          goto endrun;
       }

         //paolo 10/01/99
         if( showDisplay != NULL && showEvery > 0) {
           if( ++countShowFun > showEvery ) {
             (*showDisplay)();
             countShowFun = 0;
           }
         }

         if( breakCicle != NULL )
           if( *breakCicle ) {
             options[8]=-3;
			 err_code = 8;
			 goto endrun;
           }

		 // Ale 2002
		 ChOptimizer* moptim = (ChOptimizer*) idData;
		 if( moptim->user_break) {
			 options[8]=-3;
			 err_code = 8;
			 goto endrun;
		 }


     } /* enddo for the iteration cycle */
   }  /*  enddo for the resetting cycle */

  endrun:

  /* deallocate working arrays: */
     free(x1);
     free(z);
     free(gt);
     free(g1);
     free(g0);
     free(g);
     free(B);

  return(f);
}



} // END_OF_NAMESPACE____

// eof

