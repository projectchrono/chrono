#ifndef CHPARSER_H
#define CHPARSER_H

//////////////////////////////////////////////////
//  
//   ChParser.h
//
//   Class for quick parsing of textual formulas  
//   and handling of variables
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

//***OBSOLETE***

#include "physics/ChFormule.h"
#include "physics/ChObject.h"

namespace chrono 
{


#define CHP_ERRMSG_LEN 100

//***OBSOLETE***
//###########################################################################
//
// CLASS FOR PARSER
//
/// This class defines a compact and fast parser/compiler, that is an 
/// object which can transform simple 'programs' or formulas in form of
/// strings into compiled stack-programs, for fast execution/evaluation.
///
/// The syntax of formulas/programs is very limitesd (+/-*, sin cos etc.)
/// but enough for basic operations.
///
/// Do not confuse this feature with the MUCH more advanced (but slower) 
/// javascript embedding of Chrono objects!.
//***OBSOLETE***

#define CHCLASS_PARSER 30		

class ChParser : public ChObj {
private:
	ChEvStacks* stacks;
	ChObj* database;
	ChNode<ChTag>* instrp;
	ChNode<ChVar>* datap;
	ChNode<ChTag>* goto_instrp;
	char error_m[ChP_ERRMSG_LEN];
	double fv[10]; // some space for variables
public:	
	ChParser ();
	ChParser (ChObj* mdatabase);
	~ChParser () { delete stacks;};

		/// Database for interpreted formulas [obsolete]
	void SetDatabase (ChObj* mdatabase) {database= mdatabase;};
	ChObj* GetDatabase () {return database;};

	ChEvStacks* GetStacks() {return stacks;};
	void ClearStacks() {if (stacks) stacks->Clear();};

		// set max sizes of the three stacks (data, program, working)
	void SetStackSizes(int ds, int ps, int ws);

		// sets up the error message of the parser.
	void MakeErrorMsg (char* message, char* badcommand);
		// Gets the string with error message
	char* GetErrorMsg () {return error_m;};

		// Own translating functions ,for basic general math operations 
		// like sum, add, divide, etc., which are "global" and which does not need databases.
				// Translate token into tag structure
	int Translate (char* string, char* string_end, ChTag& ret_tag);
				// Translate tag into Ch_var 
	int ParseTag (ChTag mtag, ChVar* retv, ChNode<ChVar>* param, int set);

		// Compile a string, building the three stacks. 
		// Note that after the three stacks has been compiled, the execution can be
		// performed with Execute().
		// (This fuction is recursive, use Compile() instead!)
	int REC_Compile(char* string, char* endstring);

		/// Compilation of a text formula into a program (stored in "stacks" data of
		/// this class, ready to be executed later) 
	int Compile (char* string);

		/// After you have correctly compiled a formula string, you can execute
		/// the program memorized in stacks. Returns the pointer to the first variable
		/// of the working stack, which most likely contains the result of evaluation. 
	int Execute();

		/// Evaluates a string (both compile and execute in sequence)  !!!!!
		/// Result is TRUE if no errors. 
	int Evaluate(char* string);


		/// Shortcut for easy-to-use evaluation of a formula which
		/// is expected to return a float (results passed as references among arguments)
	int Execute(double& result);

		/// Shortcut for easy-to-use evaluation of a formula which
		/// is expected to return a float (results passed as references among arguments)
	int Evaluate(char* string, double& result);
};



} // END_OF_NAMESPACE____

#endif  // END of ChParser.h 
