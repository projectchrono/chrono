// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHSOLVMIN_H
#define CHSOLVMIN_H

#include "chrono/physics/ChFx.h"
#include "chrono/physics/ChObject.h"

// forward reference
struct JSScript;

namespace chrono {

/// Base class for multi-variable optimization.

class ChApi ChOptimizer : public ChObj {
  protected:
    ChFx* afunction;      ///< the function to be maximized
    ChFx* afunctionGrad;  ///< the gradient of the function to be maximized, or null for default BDF.

    int C_vars;  ///< number of input variables

    double* xv;      ///< Vector of variables, also 1st approximation.
    double* xv_sup;  ///< These are the hi/lo limits for the variables,
    double* xv_inf;  ///< these are not used by all optimizer, and can be NULL for gradient, but needed for genetic.

  public:
    bool minimize;     ///< default = false; just maximize
    double grad_step;  ///< default = 1.e-12; step size for evaluation of gradient

    double opt_fx;  ///< best resulting value of objective function

    char err_message[200];  ///< the ok/warning/error messages are written here
    int error_code;
    long fx_evaluations;    ///< number of function evaluations
    long grad_evaluations;  ///< number of gradient evaluations

    int (*break_funct)();    ///< if not null, this function is called each 'break_cycles' evaluations
    int break_cycles;        ///< how many fx evaluations per check
    int user_break;          ///< if break_funct() reported true, this flag is ON, and optimizers should exit all cycles
    int break_cyclecounter;  ///< internal

    ChOptimizer();
    ChOptimizer(const ChOptimizer& other);
    virtual ~ChOptimizer() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChOptimizer* Clone() const override { return new ChOptimizer(*this); }

    /// Sets the objective function to maximize
    virtual void SetObjective(ChFx* mformula) { afunction = mformula; }
    virtual ChFx* GetObjective() const { return afunction; }

    /// Sets the objective function gradient (not mandatory,
    /// because if not set, the default bacward differentiation is used).
    virtual void SetObjectiveGrad(ChFx* mformula) { afunctionGrad = mformula; }
    virtual ChFx* GetObjectiveGrad() const { return afunctionGrad; }

    /// Set the number of optimization variables.
    /// Note: this must be set properly as the number of variables used in the objective function!
    virtual void SetNumOfVars(int mv) { C_vars = mv; }
    /// Returns the number of optimization variables.
    virtual int GetNumOfVars() const { return C_vars; }

    // Gets the vector of variables
    double* GetXv() const { return xv; };
    double* GetXv_sup() const { return xv_sup; }
    double* GetXv_inf() const { return xv_inf; }
    void SetXv(double* mx) { xv = mx; }
    void SetXv_sup(double* mx) { xv_sup = mx; }
    void SetXv_inf(double* mx) { xv_inf = mx; }

    /// Returns the value of the functional, for given state of variables
    /// and with the given "database" multibody system. Here evaluates the string "function".
    double Eval_fx(double x[]);
    double Eval_fx(const ChMatrix<>* x);

    /// Computes the gradient of objective function, for given state of variables.
    /// The gradient is stored into gr vector.
    void Eval_grad(double x[], double gr[]);
    void Eval_grad(const ChMatrix<>* x, ChMatrix<>* gr);

    /// Performs the optimization of the ChSystem pointed by "database"
    /// (or whatever object which can evaluate the string "function" and the "optvarlist")
    /// using the current parameters. Returns false if some error occured.
    /// This function just makes some tests, allocations, and compilations..
    virtual bool PreOptimize();
    ///  This function computes the optimal xv[].
    /// It must be implemented by derived classes
    //<- THIS IS THE STEP WHICH COMPUTES THE OPTIMAL xv[] AND MUST BE IMPLEMENTED BY CHILDREN CLASSES!
    virtual bool DoOptimize();
    /// Finalization and cleanup
    virtual bool PostOptimize();

    /// Does the three steps in sequence PreOptimize, DoOptimize, PostOptimize.
    /// The derived classes shouldn't need the definition of this method, because
    /// they just have to implement the DoOptimize.
    virtual bool Optimize();

    /// Each break_cycles number of times this fx is called, the function break_funct() is
    /// evaluated (if any) and if positive, the variable user_break becomes true.
    void DoBreakCheck();
};

// --------------------------------------------------------------------------------------

/// Class for local optimization with the pseudo-Newton method

class ChApi ChOptimizerLocal : public ChOptimizer {
  public:
    // the parameters for optimization
    double initial_step;  // def = 1; initial trial step size
    double arg_tol;       // argument rel.error, def 1.e-6
    double fun_tol;       // function rel error, def 1.e-7
    int maxiters;         // max number of iterations, def 50;
    int maxevaluations;   // max number of fx evaluations, def 200;
    double gamma;         // gamma for line search, def 2.001;
    double dilation;      // space dilation coeff, def. 2.5
    double gradstep;      // gradient evaluation step, def 1.e-100

    // results and messages
    long iters_done;  // number of iterations performed (<0 = err)

    ChOptimizerLocal();
    ChOptimizerLocal(const ChOptimizerLocal& other);
    virtual ~ChOptimizerLocal() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChOptimizerLocal* Clone() const override { return new ChOptimizerLocal(*this); }

    // Performs the optimization of the PSystem pointed by "database"
    // (or whatever object which can evaluate the string "function" and the "optvarlist")
    // using the current parameters. Returns false if some error occured.
    virtual bool DoOptimize() override;
};

// -----------------------------------------------------------------------------

/// Class for individuals of the population; for the ChOptimizerGenetic optimization engine.

class ChApi ChGenotype {
  public:
    ChMatrix<>* genes;   ///< genetic information (variables defining the individual)
    double fitness;      ///< fitness value
    double rel_fitness;  ///< relative fitness
    double cum_fitness;  ///< cumulative fitness
    bool need_eval;      ///< genes changed, need recomputation of fitness;

    ChGenotype(int number_of_genes);
    ChGenotype(const ChGenotype& other);
    ~ChGenotype();

    ChGenotype* Clone() const { return new ChGenotype(*this); }

    //// RADU
    //// TODO: get rid of this function!!!!  REPLACE with a proper assignment operator
    void Copy(ChGenotype* source);
};

// -----------------------------------------------------------------------------

/// Class for global optimization with the genetic method (evolutive simulation).

class ChApi ChOptimizerGenetic : public ChOptimizer {
  public:
    enum class SelectionType {
        ROULETTE,
        ROULETTEBEST,
        NORMGEOMETRIC,
        TOURNAMENT,
    };
    enum class CrossoverType {
        ARITMETIC,
        BLEND,
        BLEND_RANDOM,
        HEURISTIC,
        DISABLED,
    };
    enum class CrossoverChangeType {
        NO_CHANGE,
        DATE,
        SLOWLY,
    };
    enum class MutationType {
        UNIFORM,
        BOUNDARY,
    };
    enum class ReplaceMode {
        PARENTS = 0,
        WORST,
    };

    int popsize;                        ///< def = 100; initial population size in array "population"
    ChGenotype** population;            ///< Array of pointers to population genotypes
    ChGenotype* best_indiv;             ///< Copy of the individual with best fitness found in latest optimization.
    int max_generations;                ///< max number of generations to perform
    SelectionType selection;            ///< reproduction (selection) type
    CrossoverType crossover;            ///< crossover type
    MutationType mutation;              ///< mutation type;
    bool elite;                    ///< if true, best parent is always kept after crossover
    CrossoverChangeType crossv_change;  ///< see codes above, if NO_CHANGE the crossover type is always the same
    CrossoverType crossv_changeto;      ///< the type of new crossover if using crossv_change
    long crossv_changewhen;             ///< generation number, when the "change of crossover type" takes place
    double mutation_prob;               ///< probability of mutation, 0..1, default = 0.001
    double crossover_prob;              ///< crossover probability, default = 0.3;
    bool speciation_mating;             ///< if true, marriage happens between similar individuals;
    bool incest_taboo;                  ///< if true, avoids marriage between individuals too similar
    ReplaceMode replacement;            ///< replacement mode
    double eugenetics;  ///< range (0..1); if 0, no eugenetics, otherwise clamp for fitness (normalized in 0..1)

    bool stop_by_stdeviation;  ///< if true...		(def: false)
    double stop_stdeviation;   ///< stop search if stdeviation becomes lower than this value (def.0)
    bool stop_by_fitness;      ///< if true...		(def. false)
    double stop_fitness;       ///< stop search if fitness of best individual exceed this value (def.0)

    // results and messages
    double average;      ///< the average fitness of individuals
    double stdeviation;  ///< the unbiased standard deviation (=sqrt(variance)) of fitness
    double min_fitness;
    double max_fitness;
    long generations_done;  ///< number of generations performed
    long mutants;           ///< mutated individuals since start of optimizations

    // History of statistic values.  (pointers to external values, not automatically creates/deleted)
    // If set = NULL (as by default) nothing is recorded.
    // If they point to external vectors (matrix with one column), they will
    // be filled generation by generation.
    ChMatrix<>* his_average;
    ChMatrix<>* his_stdeviation;
    ChMatrix<>* his_maxfitness;
    ChMatrix<>* his_minfitness;

    ChOptimizerGenetic();
    ChOptimizerGenetic(const ChOptimizerGenetic& other);
    virtual ~ChOptimizerGenetic();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChOptimizerGenetic* Clone() const override { return new ChOptimizerGenetic(*this); }

    // The optimization procedure.
    // Performs the optimization of the PSystem pointed by "database"
    // using the current parameters. Returns false if some error occured.
    virtual bool DoOptimize() override;

    // Handling of populations
    bool CreatePopulation(ChGenotype**& my_population, int my_popsize);
    bool DeletePopulation(ChGenotype**& my_population, int my_popsize);

    // Genetic operations on population, used internally by "optimize".

    // returns an individual from my_population, using roulette method
    ChGenotype* Select_roulette(ChGenotype** my_population);
    // returns an individual from my_population, using roulette method
    ChGenotype* Select_best(ChGenotype** my_population);
    // returns an individual from population, using roulette method
    ChGenotype* Select_worst(ChGenotype** my_population);
    // returns fitness max - fitness min
    double Get_fitness_interval(ChGenotype** my_population);
    // compute fitness for given genotype
    double ComputeFitness(ChGenotype*);
    // apply crossover to two parents, results in childrens
    void ApplyCrossover(ChGenotype* par1, ChGenotype* par2, ChGenotype& child1, ChGenotype& child2);
    // all population genotypes are initialized with random values of variables
    bool InitializePopulation();
    // all individuals get their fitness values.
    bool ComputeAllFitness();
    // the population is selected, according to "selection" parameter.
    void Selection();
    // new alleles are obtained by crossover of individuals.
    void Crossover();
    // some mutations are performed on the population.
    void Mutation();
    void PopulationStats(double& average, double& max, double& min, double& stdeviation);
    // outputs generation stats to stramLOG file (if filelog is true) and to console
    void LogOut(bool filelog);
};

// -----------------------------------------------------------------------------

/// Class for local optimization with the cheap method of gradient and bisection

class ChApi ChOptimizerGradient : public ChOptimizer {
  public:
    // the parameters for optimization
    double initial_step;   ///< def = 1; initial trial step size
    double arg_tol;        ///< argument rel.error, def 1.e-6
    double fun_tol;        ///< function rel error, def 1.e-7
    int maxevaluations;    ///< max number of fx evaluations, def 800;
    int maxgradients;      ///< max number of gradient evaluations, def 50;
    int maxdilationsteps;  ///< max number of 'exploring' steps in forward gradient direction (def 8);
    int maxbisections;     ///< number of bisections in climbing interval, before recomputing gradient (def 10);
    double dilation;       ///< space dilation coeff, def. 2
    bool do_conjugate;     ///< if true, corrects the gradient by conjugate method (default= off)

    ChOptimizerGradient();
    ChOptimizerGradient(const ChOptimizerGradient& other);
    virtual ~ChOptimizerGradient() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChOptimizerGradient* Clone() const override { return new ChOptimizerGradient(*this); }

    // Performs the optimization of the PSystem pointed by "database"
    // (or whatever object which can evaluate the string "function" and the "optvarlist")
    // using the current parameters. Returns false if some error occured.
    virtual bool DoOptimize() override;
};

// -----------------------------------------------------------------------------

/// Class for genetic optimization followed by
/// a refinement with the method of gradient, one after the other.
/// Parameters must be set for the two incapsulated optimizers
/// genetic_opt and  gradient_opt,  that is  for example:
/// my_hybrid->genetic_opt->maxgenerations = 200; etc...
/// However, the optimization variables, the system and the objective function
/// must be added to the hybrid optimizer itself, which in turn will
/// automatically set them for the genetic_opt and gradient_opt.

class ChApi ChOptimizerHybrid : public ChOptimizer {
  public:
    // The gradient optimizer. Created/deallocated automatically.
    ChOptimizerGenetic* genetic_opt;
    // The gradient optimizer. Created/deallocated automatically.
    ChOptimizerLocal* local_opt;

    int current_phase;  ///< 0 = null, 1 = genetic running, 2 = local running
    bool use_genetic;   ///< def. true;
    bool use_local;     ///< def. true;

    ChOptimizerHybrid();
    ChOptimizerHybrid(const ChOptimizerHybrid& other);
    virtual ~ChOptimizerHybrid();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChOptimizerHybrid* Clone() const override { return new ChOptimizerHybrid(*this); }

    virtual void SetObjective(ChFx* mformula) override;
    virtual void SetObjectiveGrad(ChFx* mformula) override;

    void SetNumOfVars(int mv) override;

    // Performs the optimization of the PSystem pointed by "database"
    // (or whatever object which can evaluate the string "function" and the "optvarlist")
    // using the current parameters. Returns false if some error occured.
    virtual bool DoOptimize() override;
};

// ----------------------------------------------------------------------------

/// A 3rd party function which performs local optimization

ChApi void null_entry_solv_opt(double x[], double g[]);

ChApi double solvopt(unsigned int n,
                     double x[],
                     double fun(double x[], void* idData),
                     void grad(double x[], double g[]),
                     double options[],
                     void* idData,
                     void (*showDisplay)(),
                     int showEvery,
                     int* breakCicle,
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

}  // end namespace chrono

#endif
