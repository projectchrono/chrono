#ifndef CHSOLVER_H
#define CHSOLVER_H

//////////////////////////////////////////////////
//  
//   ChSolver.h
//
//   Math functions  for :
//      - NEWTON NONLINEAR SOLUTION
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


namespace chrono 
{


////////////////////////////////////////////////
// LINEAR SYSTEM ITERATIVE SOLVER 



// ***TO DO***


////////////////////////////////////////////////
// NON LINEAR SOLVER


// ***TO DO***

#define CHCLASS_SOLVER 24

class ChSolver : public ChObj {

protected: 

	//ChOptVar* functionlist;	// list of functions to be solved, as JAVASCRIPT expressions	

	ChOptVar* varlist;		// list of variables to be solved, as JAVASCRIPT expressions;


	int C_vars;				// If NULL, the number of optimized variables is automatically
							// computed from list of ASCII variables (see "optvarlist" above),
							// otherwise must be set >0 to use the following C evaluation of formulas..

							// The user can provide the evaluation fxs also in form of
							// a generic C function of N parameters (passed as a vector double[] p),
							// returning N residuals (stored into vector double[] r),
							// which will be used if C_vars >0 and by providing the function "*funct",
							// otherwise NULL for parsing of the ASCII formulas "function".
	double (*func)(ChMatrix<>* x, ChMatrix<>* r, void* my_data);		
							// Optional: a function which returns directly the jacobian
							// matrix J of the equations, at position q. If this function is NULL, the jacobian
							// will be computed numerically (but it will be slower).
	double (*Jfunc)(ChMatrix<>* x, ChMatrix<>* J, void* my_data);		
	

							// Optional data to be passed to the *func and *Jfunc 
	void* my_data;			


	ChMatrix<>* xv;	    // Vector of variables, for C function above, also 1st approximation.
	ChMatrix<>* rv;	    // Vector of variables, for C function above


	// internal 

	//JSScript* fx_script;	// JavaScript script coming from compilation of function[]   {internal}


public: 

	// ------ DATA
					
 	double grad_step;			// def = 1.e-12; step size for evaluation of gradient 

	char err_message[200];		// the ok/warning/error messages are written here
	int error_code;

	long fx_evaluations;		// number of function evaluations
	long jacobian_evaluations;	// number of jacobian evaluations


	int (*break_funct)();	// if not null, this function is called each 'break_cycles' evaluations
	int break_cycles;		// how many fx evaluations per check
	int user_break;			// if break_funct() reported TRUE, this flag is ON, and optimizers should exit all cycles
	int break_cyclecounter; // internal


	// ------ FUCTIONS

	ChSolver();
	virtual ~ChSolver();
	virtual void Copy(ChSolver* source);



				// Sets the optimization variables
	virtual void AddOptVar (ChOptVar* newvar);
	virtual void RemoveOptVar (ChOptVar* newvar);
	virtual ChOptVar* GetVarList() {return optvarlist;};
	virtual void SetVarList(ChOptVar* mv) {optvarlist = mv;};
	virtual int  CompileOptVar();	// to speed up code..

				// returns the number of optimization variables set.
	virtual int  GetNumOfVars();


				// Set the number of solution variables.
				// note: if you use the ChOptVar "ascii" variables, 
				// this is not needed -the count is automatic- but you 
				// MUST set it > 0 if you want to use the "C" evaluation of *funct()!!!
	virtual void  SetNumOfVars(int mv) {C_vars = mv;};
				
				// set the C function which will be used for fx evaluation
				// instead of parsing of "ascii" objective formula.
	virtual void  SetObjective (double (*m_func)(double p[], double r[], void* my_data)) {func = m_func;};

				// gets the vector of variables, if C function is used
	double* GetXv() {return xv;};

	void SetXv(double* mx) {xv = mx;};


				// function aoptional argument: the "my_data" generic pointer..
	void* GetMyData() {return my_data;};
	void SetMyData(void* mmy_data) {my_data= mmy_data;};


		// the multibody system "database" gets the current state of variables. Ret. null if can't set values
	int Vars_to_System(double  x[]);
	int Vars_to_System(ChMatrix<>* x);
		// the variables gets their corresponding values in multibody system. Ret. null if can't get values
	int System_to_Vars(double  x[]);
	int System_to_Vars(ChMatrix<>* x);


		// Returns the value of the residuals in r vector, for given state of variables 
		// and with the given "database" multibody system. Here evaluates the string "function". 
		// Return null if some function cannot be evaluated.
	int Eval_fx(double x[], ChMatrix<>* r);
	int Eval_fx(ChMatrix<>* x, ChMatrix<>* r);

		// Computes the gradient of objective function, for given state of variables.
		// The gradient is stored into gr vector. 
	void   Eval_jacobian(double x[], double gr[][]);
	void   Eval_jacobian(ChMatrix<>* x, ChMatrix<>* gr);


		// Performs the solution
		// using the current parameters. Returns false if some error occured.
	virtual int PreSolve();	// <- this just makes some tests, allocations, and compilations..
	virtual int DoSolve();	// <- THIS IS THE STEP WHICH COMPUTES THE OPTIMAL xv[] AND MUST BE IMPLEMENTED BY CHILDREN CLASSES!	
	virtual int PostSolve();// <- this just makes deallocations and sets the system as xv[]
		

		// Does the three steps in sequence PreOptimize, DoOptimize, PostOptimize.
		// The derived classes shouldn't need the definition of this method, because
		// they just have to implement the DoOptimize. 
	virtual int Solve();  

		// each break_cycles number of times this fx is called, the function break_funct() is 
		// evaluated (if any) and if positive, the variable user_break becomes true.
	void DoBreakCheck(); 

	
};




} // END_OF_NAMESPACE____

#endif