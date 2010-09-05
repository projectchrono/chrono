///////////////////////////////////////////////////
//
//   ChJs_optimizer.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChJs_utils.h"
#include "ChJs_optimizer.h"
#include "physics/ChSolvmin.h"
#include "unit_JS/ChOptvar.h"
#include "physics/ChGlobal.h"

//*** DISABLED!!!***
#ifdef __12321__ 

namespace chrono
{


//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   OPTIMIZER class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec ChOptimizer_props[] = {
    {"objective",         0,	JSPROP_ENUMERATE},
    {"opt_fx",		      1,	JSPROP_ENUMERATE | JSPROP_READONLY},
	{"minimize",		  2,	JSPROP_ENUMERATE},
	{"n_eval_fx",		  3,	JSPROP_ENUMERATE | JSPROP_READONLY},
	{"n_eval_grad",		  4,	JSPROP_ENUMERATE | JSPROP_READONLY},
	{"n_vars",			  5,	JSPROP_ENUMERATE | JSPROP_READONLY},
    {0}
};

GET_JS_PARSE_BEGIN(ChOptimizer_get, ChOptimizer*)
	GET_JS_PROP (0,  chjs_from_string(cx,vp,this_data->GetObjective()) )
	GET_JS_PROP (1,  chjs_from_double(cx,vp,this_data->opt_fx) )
	GET_JS_PROP (2,  chjs_from_int(cx,vp,this_data->minimize) )
	GET_JS_PROP (3,  chjs_from_int(cx,vp,this_data->fx_evaluations) )
	GET_JS_PROP (4,  chjs_from_int(cx,vp,this_data->grad_evaluations) )
	GET_JS_PROP (5,  chjs_from_int(cx,vp,this_data->GetNumOfVars()) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(ChOptimizer_set, ChOptimizer*)
	SET_JS_PROP (0,	&chjs_string, this_data->SetObjective(chjs_to_string(cx,vp)) )
	SET_JS_PROP (2,	&chjs_int,	  this_data->minimize =chjs_to_int(cx,vp)  )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_FUNCTION(jsOptimize, ChOptimizer*, 0)
  //PARCHK(0, &chjs_double);
  int ok= 0;
  ok = this_data->Optimize();
  if (ok)
	*rval = JSVAL_VOID;
  else
	chjs_from_string(cx,rval,this_data->err_message);
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsAddVar, ChOptimizer*, 3)
  PARCHK(0, &chjs_string);
  PARCHK(1, &chjs_double);
  PARCHK(2, &chjs_double);
  ChOptVar* mopv = new ChOptVar;
  mopv->SetVarName(chjs_to_string(cx, argv+0));
  mopv->SetLimInf(chjs_to_double(cx, argv+1));
  mopv->SetLimSup(chjs_to_double(cx, argv+2));
  this_data->AddOptVar(mopv);
  *rval = JSVAL_VOID;
DEF_JS_FUNEND


// ------- Method list -------------------------------
//
static JSFunctionSpec ChOptimizer_methods[] = {
	{"Optimize",		jsOptimize,			0},
	{"AddVar",			jsAddVar,			0},
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_ChOptimizer = {
    "ChOptimizer", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  ChOptimizer_get,  ChOptimizer_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   JS_FinalizeStub,
};


////////////////////////////////////////////////////////////////////
//
// INITIALIZATION
//
JSObject* ChJS_InitClass_ChOptimizer(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob,
				parent,					// parent prototype (parent class)
				&chjs_ChOptimizer,		// this class
				NULL, 0,				// constructor fx and parameters
				ChOptimizer_props, ChOptimizer_methods,
				NULL, NULL);
	return ret;
}



//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   LOCAL OPTIMIZER class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec ChLocalOptimizer_props[] = {
    {"max_iters",          0,	JSPROP_ENUMERATE},
    {"max_evaluations",    1,	JSPROP_ENUMERATE},
    {"tol_arg",		       2,	JSPROP_ENUMERATE},
    {"tol_fx",		       3,	JSPROP_ENUMERATE},
    {"initial_step",	   4,	JSPROP_ENUMERATE},
    {"grad_step",		   5,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(ChLocalOptimizer_get, ChOptimizerLocal*)
	GET_JS_PROP (0,  chjs_from_int(cx,vp,this_data->maxiters) )
	GET_JS_PROP (1,  chjs_from_int(cx,vp,this_data->maxevaluations) )
	GET_JS_PROP (2,  chjs_from_double(cx,vp,this_data->arg_tol) )
	GET_JS_PROP (3,  chjs_from_double(cx,vp,this_data->fun_tol) )
	GET_JS_PROP (4,  chjs_from_double(cx,vp,this_data->initial_step) )
	GET_JS_PROP (5,  chjs_from_double(cx,vp,this_data->grad_step) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(ChLocalOptimizer_set, ChOptimizerLocal*)
	SET_JS_PROP (0,	&chjs_int,	  this_data->maxiters =chjs_to_int(cx,vp)  )
	SET_JS_PROP (1,	&chjs_int,	  this_data->maxevaluations =chjs_to_int(cx,vp)  )
	SET_JS_PROP (2,	&chjs_double, this_data->arg_tol =chjs_to_double(cx,vp)  )
	SET_JS_PROP (3,	&chjs_double, this_data->fun_tol =chjs_to_double(cx,vp)  )
	SET_JS_PROP (4,	&chjs_double, this_data->initial_step = chjs_to_double(cx,vp)  )
	SET_JS_PROP (5,	&chjs_double, this_data->grad_step = chjs_to_double(cx,vp)  )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_BUILDER (ChLocalOptimizer_construct, ChOptimizerLocal*)
  this_data = new ChOptimizerLocal;
DEF_JS_BUILDEND

ChJS_FINALIZER (ChLocalOptimizer_finalize, ChOptimizerLocal*)



// ------- Method list -------------------------------
//
static JSFunctionSpec ChLocalOptimizer_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_ChLocalOptimizer = {
    "ChLocalOptimizer", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  ChLocalOptimizer_get,  ChLocalOptimizer_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,		   ChLocalOptimizer_finalize,
};


////////////////////////////////////////////////////////////////////
//
// INITIALIZATION
//
JSObject* ChJS_InitClass_ChLocalOptimizer(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob,
				parent,					// parent prototype (parent class)
				&chjs_ChLocalOptimizer,		// this class
				ChLocalOptimizer_construct, 0,	// constructor fx and parameters
				ChLocalOptimizer_props, ChLocalOptimizer_methods,
				NULL, NULL);
	return ret;
}


//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   GENETIC OPTIMIZER class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec ChGeneticOptimizer_props[] = {
    {"popsize",	          0,	JSPROP_ENUMERATE},
    {"max_generations",   1,	JSPROP_ENUMERATE},
    {"selection",		  2,	JSPROP_ENUMERATE},
    {"crossover",		  3,	JSPROP_ENUMERATE},
    {"crossover_prob",	  4,	JSPROP_ENUMERATE},
    {"mutation",		  5,	JSPROP_ENUMERATE},
    {"mutation_prob",	  6,	JSPROP_ENUMERATE},
    {"elite",			  7,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(ChGeneticOptimizer_get, ChOptimizerGenetic*)
	GET_JS_PROP (0,  chjs_from_int(cx,vp,this_data->popsize) )
	GET_JS_PROP (1,  chjs_from_int(cx,vp,this_data->max_generations) )
	GET_JS_PROP (2,  chjs_from_int(cx,vp,this_data->selection) )
	GET_JS_PROP (3,  chjs_from_int(cx,vp,this_data->crossover) )
	GET_JS_PROP (4,  chjs_from_double(cx,vp,this_data->crossover_prob) )
	GET_JS_PROP (5,  chjs_from_int(cx,vp,this_data->mutation) )
	GET_JS_PROP (6,  chjs_from_double(cx,vp,this_data->mutation_prob) )
	GET_JS_PROP (7,  chjs_from_int(cx,vp,this_data->elite) )
	GET_JS_PROP (8,  chjs_from_int(cx,vp,this_data->replacement) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(ChGeneticOptimizer_set, ChOptimizerGenetic*)
	SET_JS_PROP (0,	&chjs_int,	  this_data->popsize =chjs_to_int(cx,vp)  )
	SET_JS_PROP (1,	&chjs_int,	  this_data->max_generations =chjs_to_int(cx,vp)  )
	SET_JS_PROP (2,	&chjs_int,	  this_data->selection =chjs_to_int(cx,vp)  )
	SET_JS_PROP (3,	&chjs_int,	  this_data->crossover =chjs_to_int(cx,vp)  )
	SET_JS_PROP (4,	&chjs_double, this_data->crossover_prob =chjs_to_double(cx,vp)  )
	SET_JS_PROP (5,	&chjs_int,	  this_data->mutation =chjs_to_int(cx,vp)  )
	SET_JS_PROP (6,	&chjs_double, this_data->mutation_prob =chjs_to_int(cx,vp)  )
	SET_JS_PROP (7,	&chjs_int,	  this_data->elite =chjs_to_int(cx,vp)  )
	SET_JS_PROP (8,	&chjs_int,	  this_data->replacement =chjs_to_int(cx,vp)  )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_BUILDER (ChGeneticOptimizer_construct, ChOptimizerGenetic*)
  this_data = new ChOptimizerGenetic;
DEF_JS_BUILDEND

ChJS_FINALIZER (ChGeneticOptimizer_finalize, ChOptimizerGenetic*)



// ------- Method list -------------------------------
//
static JSFunctionSpec ChGeneticOptimizer_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_ChGeneticOptimizer = {
    "ChGeneticOptimizer", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  ChGeneticOptimizer_get,  ChGeneticOptimizer_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,		   ChGeneticOptimizer_finalize,
};

static JSConstDoubleSpec genetic_constants[] = {
	{SELEC_ROULETTE,		"SELEC_ROULETTE",		0, {0,0,0}},
	{SELEC_ROULETTEBEST,	"SELEC_ROULETTEBEST",	0, {0,0,0}},
	{SELEC_NORMGEOMETRIC,	"SELEC_NORMGEOMETRIC",	0, {0,0,0}},
	{SELEC_TOURNAMENT,		"SELEC_TOURNAMENT",		0, {0,0,0}},
	{CROSSOVER_ARITMETIC,	"CROSSOVER_ARITMETIC",	0, {0,0,0}},
	{CROSSOVER_BLEND,		"CROSSOVER_BLEND",		0, {0,0,0}},
	{CROSSOVER_BLEND_RANDOM,"CROSSOVER_BLEND_RANDOM", 0, {0,0,0}},
	{CROSSOVER_HEURISTIC,	"CROSSOVER_HEURISTIC",	0, {0,0,0}},
	{CROSSOVER_DISABLED,	"CROSSOVER_DISABLED",	0, {0,0,0}},
	{CRO_CHANGE_NULL,		"CRO_CHANGE_NULL",		0, {0,0,0}},
	{CRO_CHANGE_DATE,		"CRO_CHANGE_DATE",		0, {0,0,0}},
	{CRO_CHANGE_SLOWLY,		"CRO_CHANGE_SLOWLY",	0, {0,0,0}},
	{MUTATION_UNIFORM,		"MUTATION_UNIFORM",		0, {0,0,0}},
	{MUTATION_BOUNDARY,		"MUTATION_BOUNDARY",	0, {0,0,0}},
	{ELITE_FALSE,			"ELITE_FALSE",			0, {0,0,0}},
	{ELITE_TRUE,			"ELITE_TRUE",			0, {0,0,0}},
	{REPLA_PARENTS,			"REPLA_PARENTS",		0, {0,0,0}},
	{REPLA_WORST,			"REPLA_WORST",			0, {0,0,0}},
    {0,0,0,{0,0,0}}
};

////////////////////////////////////////////////////////////////////
//
// INITIALIZATION
//
JSObject* ChJS_InitClass_ChGeneticOptimizer(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob,
				parent,					// parent prototype (parent class)
				&chjs_ChGeneticOptimizer,		// this class
				ChGeneticOptimizer_construct, 0,	// constructor fx and parameters
				ChGeneticOptimizer_props, ChGeneticOptimizer_methods,
				NULL, NULL);

	if (!JS_DefineConstDoubles(cx, ret, genetic_constants))
     return NULL;

	return ret;
}


//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   HYBRID OPTIMIZER class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec ChHybridOptimizer_props[] = {
    {"genetic_opt",	      0,	JSPROP_ENUMERATE | JSPROP_READONLY},
    {"local_opt",	      1,	JSPROP_ENUMERATE | JSPROP_READONLY},
    {0}
};


GET_JS_PARSE_BEGIN(ChHybridOptimizer_get, ChOptimizerHybrid*)
	GET_JS_PROP (0,  chjs_from_data(cx,vp,this_data->genetic_opt, &chjs_ChGeneticOptimizer ) )
	GET_JS_PROP (1,  chjs_from_data(cx,vp,this_data->local_opt, &chjs_ChLocalOptimizer ) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(ChHybridOptimizer_set, ChOptimizerHybrid*)
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_BUILDER (ChHybridOptimizer_construct, ChOptimizerHybrid*)
  this_data = new ChOptimizerHybrid;
DEF_JS_BUILDEND

ChJS_FINALIZER (ChHybridOptimizer_finalize, ChOptimizerHybrid*)



// ------- Method list -------------------------------
//
static JSFunctionSpec ChHybridOptimizer_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_ChHybridOptimizer = {
    "ChHybridOptimizer", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  ChHybridOptimizer_get,  ChHybridOptimizer_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,		    ChHybridOptimizer_finalize,
};


////////////////////////////////////////////////////////////////////
//
// INITIALIZATION
//
JSObject* ChJS_InitClass_ChHybridOptimizer(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob,
				parent,					// parent prototype (parent class)
				&chjs_ChHybridOptimizer,		// this class
				ChHybridOptimizer_construct, 0,	// constructor fx and parameters
				ChHybridOptimizer_props, ChHybridOptimizer_methods,
				NULL, NULL);

	return ret;
}


} // END_OF_NAMESPACE____

#endif