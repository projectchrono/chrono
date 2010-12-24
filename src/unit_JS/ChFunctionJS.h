#ifndef CHFUNCTJS_H
#define CHFUNCTJS_H

//////////////////////////////////////////////////
//  
//   ChFunctionJS.h
//
//   Function object that uses Javascript parsing
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChFunction.h"
#include "unit_JS/ChApiJS.h"


struct JSScript; // forward reference;


namespace chrono 
{



class ChApiJS ChFunctionOptvarToolsJS
{
public:
	static int OptVariablesToVector(ChFunction* funct, ChMatrix<>* mv, int offset);
	static int VectorToOptVariables(ChFunction* funct, ChMatrix<>* mv, int offset);
};




#define FUNCT_JSCRIPT	14

#define CHF_JSCRIPT_STRING_LEN 200


////////////////////////////////////////////
/// JAVASCRIPT FUNCTION:
/// y = javascript evaluation of function y=f(x)  
/// 

class ChApiJS ChFunction_Jscript : public ChFunction
{
	CH_RTTI(ChFunction_Jscript, ChFunction);

private:
	char js_command[CHF_JSCRIPT_STRING_LEN];			// jscript command or file
	JSScript *js_script;								// jscript compiled script, if any
	int js_error;							// set to true when can't compile/execute (reset false by next SetCommand())
public:
	ChFunction_Jscript();
	~ChFunction_Jscript() {};
	void Copy (ChFunction_Jscript* source);
	ChFunction* new_Duplicate ();

		/// Set the javascript command to be executed each time Get_y() is called. 
	void Set_Command  (char* m_command);
		/// Get the javascript command to be executed each time Get_y() is called. 
	char* Get_Command  ()  {return js_command;};
	
		/// Set/initialize a javascript variable, which can be used by the js command. 
	void Set_Variable (char* variablename, double value);

		/// Get false if no javascript parsing error.
	int Get_js_error  ()  {return js_error;};

	double Get_y      (double x);

	int Get_Type () {return (FUNCT_JSCRIPT);}

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};


} // END_OF_NAMESPACE____


#endif
