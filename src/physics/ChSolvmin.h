#ifndef CHSOLVMIN_H
#define CHSOLVMIN_H

//////////////////////////////////////////////////
//
//   ChSolvmin.h
//
//   Math functions and 'optimization engine object' for :
//      - LOCAL NONLINEAR OPTIMIZATION
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChOptvar.h"


// forward reference
struct JSScript;

namespace chrono
{



#define OPT_ERR_OK				0
#define OPT_ERR_NOVARS			1
#define OPT_ERR_NOMEMORY		2
#define OPT_ERR_INFINITY		3
#define OPT_ERR_CANNOTEVALFX	4
#define OPT_ERR_CANNOTEVALVAR	5
#define OPT_ERR_INVALIDSYS		6

#define OPT_IMPOSSIBLE		+999999
#define OPT_PENALTY_POS 	+999998
#define OPT_PENALTY_NEG 	-999998



#define CHCLASS_OPTIMIZER 24

///
/// Base class for multi-variable optimization.
///

class ChOptimizer : public ChObj {

protected:
	char function[300];		// the objective formula to be maximized, as ASCII expression

	ChOptVar* optvarlist;	// list of variables to be optimized;
	void* database;			// points to an object derived by ChObj (ex: a PSystem )which can
							// evaluate both variables and objective function [obsolete]

	int C_vars;				// If NULL, the number of optimized variables is automatically
							// computed from list of ASCII variables (see "optvarlist" above),
							// otherwise must be set >0 to use the following C evaluation of formulas..

							/// The user can provide the evaluation fx also in form of
							/// a generic C function of N parameters (passed as a vector double[] p),
							/// which will be used if C_vars >0 and by providing the function "*funct",
							/// otherwise NULL for parsing of the ASCII formula "function".
	double (*func)(double p[], void* my_data);				// function evaluation

							/// Same as above, but to compute gradient. If NULL (as default),
							/// the gradient is obtained numerically by differentiation.
	void (*dfunc)(double p[], double dp[], void* my_data);	// gradient evaluation

	void* my_data;			// Optional data to be passed to the *func and *dfunc
	double* xv;			// Vector of variables, for C function above, also 1st approximation.
	double* xv_sup;		// When using C fx eval, these are the hi/lo limits for the variables.
	double* xv_inf;		// these are not used by all optimizer, and can be NULL for gradient, for example, but needed for genetic.

	JSScript* fx_script;	// JavaScript script coming from compilation of function[]   {internal}

public:

	// ------ DATA

	bool minimize;				///< default = false; just maximize
 	double grad_step;			///< default = 1.e-12; step size for evaluation of gradient

	double opt_fx;				///< best resulting value of objective function

	char err_message[200];		///< the ok/warning/error messages are written here
	int error_code;
	long fx_evaluations;		///< number of function evaluations
	long grad_evaluations;		///< number of gradient evaluations


	int (*break_funct)();	///< if not null, this function is called each 'break_cycles' evaluations
	int break_cycles;		///< how many fx evaluations per check
	int user_break;			///< if break_funct() reported TRUE, this flag is ON, and optimizers should exit all cycles
	int break_cyclecounter; ///< internal


	// ------ FUCTIONS

	ChOptimizer();
	virtual ~ChOptimizer();
	virtual void Copy(ChOptimizer* source);

				/// Sets the objective function to maximize, as ASCII interpreted formula
				/// Such fourmula will be interpreted by the "database" object.
	virtual void SetObjective (char* mformula);
	char* GetObjective () {return function;};

				/// Sets the database which will be accessed by interpreted formulas [obsolete]
	void SetDatabase (void* newdb) {database = newdb; };
	void* GetDatabase () {return database;};

				/// Sets the optimization variables
	virtual void AddOptVar (ChOptVar* newvar);
	virtual void RemoveOptVar (ChOptVar* newvar);

	virtual ChOptVar* GetVarList() {return optvarlist;};
	virtual void SetVarList(ChOptVar* mv) {optvarlist = mv;};

	virtual int  CompileOptVar();	// to speed up code..


				/// returns the number of optimization variables set.
	virtual int  GetNumOfVars();

				/// Set the number of optimization variables.
				/// note: if you use the ChOptVar "ascii" variables,
				/// this is not needed -the count is automatic- but you
				/// MUST set it > 0 if you want to use the "C" evaluation of *funct()!!!
	virtual void  SetNumOfVars(int mv) {C_vars = mv;};

				/// Set the C function which will be used for fx evaluation
				/// instead of parsing of "ascii" objective formula.
	virtual void  SetObjective (double (*m_func)(double p[], void* my_data)) {func = m_func;};
	virtual void  SetDObjective (void (*m_dfunc)(double p[], double dp[], void* my_data)) {dfunc = m_dfunc;};

				// Gets the vector of variables, if C function is used
	double* GetXv() {return xv;};
	double* GetXv_sup() {return xv_sup;};
	double* GetXv_inf() {return xv_inf;};
	void SetXv(double* mx) {xv = mx;};
	void SetXv_sup(double* mx) {xv_sup = mx;};
	void SetXv_inf(double* mx) {xv_inf = mx;};

				/// Function optional argument: the "my_data" generic pointer..
	void* GetMyData() {return my_data;};
	void SetMyData(void* mmy_data) {my_data= mmy_data;};


				/// The multibody system "database" gets the current state of
				/// variables. Ret. null if can't set values
	int Vars_to_System(double  x[]);
	int Vars_to_System(ChMatrix<>* x);

				/// The variables gets their corresponding values in multibody system.
				/// Return null if can't get values
	int System_to_Vars(double  x[]);
	int System_to_Vars(ChMatrix<>* x);

				/// Returns the value of the functional, for given state of variables
				/// and with the given "database" multibody system. Here evaluates the string "function".
	double Eval_fx(double x[]);
	double Eval_fx(ChMatrix<>* x);

				/// Computes the gradient of objective function, for given state of variables.
				/// The gradient is stored into gr vector.
	void   Eval_grad(double x[], double gr[]);
	void   Eval_grad(ChMatrix<>* x, ChMatrix<>* gr);


				/// Performs the optimization of the ChSystem pointed by "database"
				/// (or whatever object which can evaluate the string "function" and the "optvarlist")
				/// using the current parameters. Returns false if some error occured.
	virtual int PreOptimize();	// <- this just makes some tests, allocations, and compilations..
	virtual int DoOptimize();	// <- THIS IS THE STEP WHICH COMPUTES THE OPTIMAL xv[] AND MUST BE IMPLEMENTED BY CHILDREN CLASSES!
	virtual int PostOptimize();	// <- this just makes deallocations and sets the system as xv[]

				/// Does the three steps in sequence PreOptimize, DoOptimize, PostOptimize.
				/// The derived classes shouldn't need the definition of this method, because
				/// they just have to implement the DoOptimize.
	virtual int Optimize();

				/// Each break_cycles number of times this fx is called, the function break_funct() is
				/// evaluated (if any) and if positive, the variable user_break becomes true.
	void DoBreakCheck();


};



////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#define CHCLASS_LOCOPTIMIZER 25

///
/// Inherited class for local optimization
/// with the method of pseudo-Newton
///

class ChOptimizerLocal : public ChOptimizer
{
public:

	// ------ DATA

	// the parameters for optimization
	double initial_step;		// def = 1; initial trial step size
	double arg_tol;				// argument rel.error, def 1.e-6
	double fun_tol;				// function rel error, def 1.e-7
	int    maxiters;			// max number of iterations, def 50;
	int    maxevaluations;		// max number of fx evaluations, def 200;
	double gamma;				// gamma for line search, def 2.001;
	double dilation;			// space dilation coeff, def. 2.5
	double gradstep;			// gradient evaluation step, def 1.e-100

	// results and messages
	long iters_done;			// number of iterations performed (<0 = err)


	// ------ FUNCTIONS

	ChOptimizerLocal();
	virtual ~ChOptimizerLocal();
	virtual void Copy(ChOptimizerLocal* source);

		// Performs the optimization of the PSystem pointed by "database"
		// (or whatever object which can evaluate the string "function" and the "optvarlist")
		// using the current parameters. Returns false if some error occured.
	virtual int DoOptimize();


};





////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////


#define CHCLASS_GENETICOPTIMIZER 26

/// Class for individuals of the population;
/// for the ChOptimizerGenetic optimization engine.

class ChGenotype {
public:
	ChMatrix<>* genes;		// genetic information (variables defining the individual)
	double fitness;		// fitness value
	double rel_fitness;	// relative fitness
	double cum_fitness;	// cumulative fitness
	int	   need_eval;	// genes changed, need recomputation of fitness;

	ChGenotype(int number_of_genes);
	~ChGenotype();
	void Copy(ChGenotype* source);
};


///
/// Inherited class for global optimization
/// with the genetic method (evolutive simulation).
///

class ChOptimizerGenetic : public ChOptimizer
{
public:

	// ------ DATA

	// the parameters for optimization
	int	popsize;				// def = 100; initial population size in array "population"
	ChGenotype** population;	// Array of pointers to population genotypes (NOTE! created and
								// deleted just within the function Optimize(); null if outside.
	ChGenotype* best_indiv;		// The copy of the individual with best fitness found in latest
								// optimization. It always exist, even after termination of optimization.
	int	max_generations;		// max number of generations to perform
	int selection;				// reproduction (selection) type (see codes)
	int	crossover;				// crossover type (see codes)
	int mutation;				// mutation type;
	int elite;					// if true, best parent is always kept after crossover
	int crossv_change;			// see codes above, if NULL the crossover type is always the same
	int crossv_changeto;		// the type of new crossover if using crossv_change
	long crossv_changewhen;		// generation number, when the "change of crossover type" takes place
	double mutation_prob;		// probability of mutation, 0..1, default = 0.001
	double crossover_prob;		// crossover probability, default = 0.3;
	int speciation_mating;		// if TRUE, marriage happens between similar individuals;
	int incest_taboo;			// if TRUE, avoids marriage between individuals too similar
	int replacement;			// see codes
	double eugenetics;			// range (0..1); if 0, no eugenetics, otherwise is the clamping for fitness (normalized in 0..1)

	int stop_by_stdeviation;	// if true...		(def: false)
	double stop_stdeviation;	// stop search if stdeviation becomes lower than this value (def.0)
	int stop_by_fitness;		// if true...		(def. false)
	double stop_fitness;		// stop search if fitness of best individual exceed this value (def.0)

	// results and messages
	double average;				// the average fitness of individuals
	double stdeviation;			// the unbiased standard deviation (=sqrt(variance)) of fitness
	double min_fitness;
	double max_fitness;
	long generations_done;		// number of generations performed
	long mutants;				// mutated individuals since start of optimizations


	// History of statistic values.  (pointers to external values, not automatically creates/deleted)
	// If set = NULL (as by default) nothing is recorded.
	// If they point to external vectors (matrix with one column), they will
	// be filled generation by generation.
	ChMatrix<>* his_average;
	ChMatrix<>* his_stdeviation;
	ChMatrix<>* his_maxfitness;
	ChMatrix<>* his_minfitness;


	// ------ FUCTIONS

	ChOptimizerGenetic();
	virtual ~ChOptimizerGenetic();
	virtual void Copy(ChOptimizerGenetic* source);

		// The optimization procedure.
		// Performs the optimization of the PSystem pointed by "database"
		// using the current parameters. Returns false if some error occured.
	virtual int DoOptimize();

		// Handling of populations
	int CreatePopulation(ChGenotype**& my_population, int my_popsize);
	int DeletePopulation(ChGenotype**& my_population, int my_popsize);
		// Genetic operations on population, used internally by "optimize".
	ChGenotype* Select_roulette(ChGenotype** my_population);	// returns an individual from my_population, using roulette method
	ChGenotype* Select_best(ChGenotype** my_population);		// returns an individual from my_population, using roulette method
	ChGenotype* Select_worst(ChGenotype** my_population);		// returns an individual from population, using roulette method
	double Get_fitness_interval(ChGenotype** my_population);	// returns fitness max - fitness min
	double ComputeFitness(ChGenotype*);		// compute fitness for given genotype
	int ApplyCrossover (ChGenotype* par1, ChGenotype* par2, ChGenotype& child1, ChGenotype& child2);  // apply crossover to two parents, results in childrens
	int InitializePopulation();	// all population genotypes are initialized with random values of variables
 	int ComputeAllFitness();	// all individuals get their fitness values.
	int Selection();			// the population is selected, according to "selection" parameter.
	int Crossover();			// new alleles are obtained by crossover of individuals.
	int Mutation();				// some mutations are performed on the population.
	int PopulationStats(double& average, double& max, double& min, double& stdeviation);
	int LogOut(int filelog);		// outputs generation stats to stramLOG file (if filelog TRUE) and to console

};

enum eChGeneticSelection{
	SELEC_ROULETTE = 0,
	SELEC_ROULETTEBEST,
	SELEC_NORMGEOMETRIC,
	SELEC_TOURNAMENT,
};
enum eChGeneticCrossover{
	CROSSOVER_ARITMETIC = 0,
	CROSSOVER_BLEND,
	CROSSOVER_BLEND_RANDOM,
	CROSSOVER_HEURISTIC,
	CROSSOVER_DISABLED,
};
enum eChGeneticChange{				
	CRO_CHANGE_NULL = 0,
	CRO_CHANGE_DATE,
	CRO_CHANGE_SLOWLY,
};
enum eChGeneticMutation{
	MUTATION_UNIFORM = 0,
	MUTATION_BOUNDARY,
};
enum eChGeneticEliteMode{
	ELITE_FALSE = 0,
	ELITE_TRUE,
};
enum eChGeneticReplaceMode{				
	REPLA_PARENTS = 0,
	REPLA_WORST,
};




////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#define CHCLASS_GRADBOPTIMIZER 32

///
/// Inherited class for local optimization
/// with the cheap method of gradient and bisection
///

class ChOptimizerGradient : public ChOptimizer
{
public:

	// ------ DATA

	// the parameters for optimization
	double initial_step;		// def = 1; initial trial step size
	double arg_tol;				// argument rel.error, def 1.e-6
	double fun_tol;				// function rel error, def 1.e-7
	int    maxevaluations;		// max number of fx evaluations, def 800;
	int    maxgradients;		// max number of gradient evaluations, def 50;
	int	   maxdilationsteps;	// max number of 'exploring' steps in forward gradient direction (def 8);
	int    maxbisections;			// number of bisections in climbing interval, before recomputing gradient (def 10);
	double dilation;			// space dilation coeff, def. 2
	int	   do_conjugate;		// if true, corrects the gradient by conjugate method (default= off)


	// ------ FUNCTIONS

	ChOptimizerGradient();
	virtual ~ChOptimizerGradient();
	virtual void Copy(ChOptimizerGradient* source);

		// Performs the optimization of the PSystem pointed by "database"
		// (or whatever object which can evaluate the string "function" and the "optvarlist")
		// using the current parameters. Returns false if some error occured.
	virtual int DoOptimize();

};




////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

#define CHCLASS_HYBRIDOPTIMIZER 33

/// Inherited class for genetic optimization followed by
/// a refinement with the method of gradient, one after the other.
///  Parameters must be set for the two incapsulated optimizers
/// genetic_opt and  gradient_opt,  that is  for example:
/// my_hybrid->genetic_opt->maxgenerations = 200; etc...
///  However, the optimization variables, the system and the objective function
/// must be added to the hybrid optimizer itself, which in turn will
/// automatically set them for the genetic_opt and gradient_opt.


class ChOptimizerHybrid : public ChOptimizer
{
public:
	// ------ DATA

			// The gradient optimizer. Created/deallocated automatically.
	ChOptimizerGenetic*	genetic_opt;
			// The gradient optimizer. Created/deallocated automatically.
	ChOptimizerLocal*		local_opt;

	int current_phase;	// 0 = null, 1 = genetic running, 2 = local running
	bool use_genetic;	// def. true;
	bool use_local;		// def. true;

	// ------ FUNCTIONS

	ChOptimizerHybrid();
	virtual ~ChOptimizerHybrid();
	virtual void Copy(ChOptimizerHybrid* source);

	void SetObjective (char* mformula);
	void SetObjective (double (*m_func)(double p[], void* my_data));
	void SetDObjective (void (*m_dfunc)(double p[], double dp[], void* my_data));

	void SetNumOfVars(int mv);
	void SetDatabase (void* newdb);
	void AddOptVar (ChOptVar* newvar);
	void RemoveOptVar (ChOptVar* newvar);


		// Performs the optimization of the PSystem pointed by "database"
		// (or whatever object which can evaluate the string "function" and the "optvarlist")
		// using the current parameters. Returns false if some error occured.
	virtual int DoOptimize();


};






////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////
// A 3rd party function which performs local optimization
//

void null_entry_solv_opt(double x[],double g[]);

double solvopt(unsigned int n,
               double x[],
               double  fun(double x[],void * idData),
               void  grad(double x[],double g[]),
               double options[],
               void *  idData,
               void (*showDisplay)(),
               int showEvery,
               int *breakCicle,
			   int& err_code);

     /*
              solvopt_options[0]= H, where sign(H)=-1 resp. sign(H)=+1 means minimize
                          resp. maximize FUN and H itself is a factor for the
                          initial trial step size (options[0]=-1 by default),
              solvopt_options[1]= relative error for the argument
                          in terms of the max-norm (1.e-6 by default),
              solvopt_options[2]= relative error for the function value (1.e-7 by default),
              solvopt_options[3]= limit for the number of iterations (5000 by default),
              solvopt_options[4]= control of the display of intermediate results and
                          error resp. warning messages (default value is 0,
                          i.e., no intermediate output but error and warning
                          messages),
             @solvopt_options[5]= constant gamma used by the line search procedure
                          (options[5]=2.001 by default),
             @solvopt_options[6]= the coefficient of space dilation (2.5 by default),
             @solvopt_options[7]= lower bound for the stepsize used for the difference
                          approximation of gradients (1.e-100 by default).
						   (@ ... changes should be done with care)

              options[8], return the number of iterations, options[8]<0 means
                            an error occured
              options[9], return the number of objective function evaluations, and
              options[10],return the number of gradient evaluations.
			  options[11],limit on the maximum number of fx evaluations. No default.
      */



} // END_OF_NAMESPACE____

#endif
