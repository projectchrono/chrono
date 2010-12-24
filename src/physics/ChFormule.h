#ifndef CHFORMULE_H
#define CHFORMULE_H

//////////////////////////////////////////////////
//
//   ChFormule.h
//
//   Class for quick parsing/compilation of textual
//   formulas and handling of variables
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "core/ChApiCE.h"
#include "core/ChLists.h"
#include "physics/ChStack.h"
#include "physics/ChFormule.h"

namespace chrono
{



// CLASS IDENTIFIER  - unique ID number codes-
// All type identifiers start with CHCLASS_...
// Each important class header defines its own.

// current CHCLASS_MAXVAL max value =  53
#define CHCLASS_NONE	0		// this is for void variable
#define CHCLASS_FLOAT   11		// this is for float variable
#define CHCLASS_INTEGER 12
#define CHCLASS_STRING  27
#define CHCLASS_POINTER 28		// this is for generic pointer
#define CHCLASS_STRINGP 29		// this is for generic pointer to string


//###########################################################################
//
// THESAURUS ITEM  class
//
/// Each object can have a "lexicon thesaurus", ie, a static global array
/// of "thesaurus items" which can be used for the translation of string
/// tokens into Ch_Tag identifiers (see below).

class ChThesItem {
public:
	char*   name;		// string identifier of token

	int		write;		// can be written, not only read.
	int		ret_type;	// return type (class identifier)
	int		par1_type;	// 1st parameter: type (class identifier), null if none
	int		par2_type;	// 2nd parameter: type (class identifier), null if none
	int		par3_type;	// 3rd parameter: type (class identifier), null if none
};



//###########################################################################
//
// CLASS FOR FUNCTION TAGS
//
/// Functions can be translated into tag structures during "interpretation"
/// of strings. These can be used to build a stack and to access object's inner data (see
/// the "translate" functions of objects).

class ChApi ChTag {
public:
	int		tagID;		// unique data identifier of tag in own class (1,2,3,...)
	int		classID;	// unique class identifier
	void*	object;		// the address of the object

	int		write;		// can be written, not only read.
	int		ret_type;	// return type (class identifier)
	int		par1_type;	// 1st parameter: type (class identifier), null if none
	int		par2_type;	// 2nd parameter: type (class identifier), null if none
	int		par3_type;	// 3rd parameter: type (class identifier), null if none

	ChTag();
	ChTag(int mtagID, int mclassID, void* mobject, ChThesItem mthesa);
	~ChTag() {};
	void Set(int mtagID, int mclassID, void* mobject, ChThesItem mthesa);
	void Copy(ChTag* mtag) {*this=*mtag;};
};



//###########################################################################
//
// VARIABLES:   BASE CLASS FOR HANDLING STACK VALUES
//
/// Variables for general-purpose type handling in parsed formula evaluations.
/// Formulas will correspond to staks ChStack of operations and ChVar objects.


class ChApi ChVar {
public:
			//
			// DATA
			//
					/// Pointer to generic variable, (data)
					/// i.e. double/int/float/vector/object, etc
	void*	varp;
					/// Needed for speed reason, varp points to this double if a simple
					/// "double" type is needed, to avoid allocation/deallocation.
	double  mdouble;
					/// Identifier of type of variable. The creator
					/// must care to set it.
	int		classtype;

					/// Size of data pointed by varp. If set NULL, varp points to
					/// mdouble or other stuff which should not be freed.
					/// so the delete() does not free it. Otherwise
					/// destruction will free(varp).
	int		size;

			//
			// FUNCTIONS
			//

	ChVar ();
	ChVar (int msize);
	~ChVar ();
	ChVar (const ChVar&);
	void Copy(const ChVar*);

	ChVar& operator=(const ChVar& other)	
					{if (&other == this) return *this; this->Copy(&other); return *this; }

					/// Clean() restores everything as a new Ch_var, and
					/// frees the memory block, if any.
	void Clear();

			// shortcuts:
					/// Cleans the var and sets the double value. (you can get: mdouble = *(double*)varp)
	void ResetAsDouble(double myval);
					/// Cleans the var and sets the integer value. (you can get: minteger = *(int*)varp)
	void ResetAsInt(int myval);
					/// Cleans the var and sets (and copies) a vector value, (you can get: mvector = *(Vector*)varp)
	void ResetAsVector(void* myval);
					/// Cleans the var and sets the pointer to string, without copying (you can get: char* = (char*)varp)
	void ResetAsStringPointer(char* myval);
					/// Cleans the var and sets a generic pointer (you can get: mpointer = varp)
	void ResetAsPointer(void* myval);
					/// Cleans the var and sets the variable as void. (ex: it is the return value of a "void function()")
	void ResetAsVoid();
};




//###########################################################################
//
// CLASS FOR EVALUATION STACKS
//
/// The three stacks for evaluations of formulas and Chrono programs.
/// Each formula, after parsing and 'compilation', will correspond to
/// three stacks (program stack, work stack, data stack).

class ChApi ChEvStacks {
public:
	ChStack<ChTag>* prog_stack;
	ChStack<ChVar>* work_stack;
	ChStack<ChVar>* data_stack;

	ChEvStacks(long psize, long wsize, long dsize);
	~ChEvStacks();
	void Clear();
};






// FUNCTIONS (not belonging to a class)


			// Compares the two strings "string" and "compare",
			// Returns TRUE if they are equal.
			// Note that the control is performed on the characters between
			// "string" and "endstring", (last char excluded).
			// Return FALSE if they does not match (also if number of characters
			// of "compare" is not = endstring-string.)
int token_match (char* string, char* endstring, char* compare);

			// Starting from "string", counts the lenght of the token atom,
			// - until the -> symbol or ...
			// - the space or ..
			// - end of string.  are found.
int token_atom_lenght (char* string, char* string_end);

			// changes the "string" pointer moving it right after the token-atom
			// separator, i.e. after the first "->" found between string..endstring
int goto_next_atom (char*& string, char*& endstring);

			// Starting from the parenthesis char pointed by "stringp",
			// returns the number of chars to reach other matching parenthesis in
			// the interval string..endstring
			// Returns 0 if no matchng parenthesis found in the interval.
int match_parenthesis (char* string, char* endstring);
			// Same, but this is for { ... }
int match_brace (char* string, char* endstring);


			// Returns TRUE if the string between string..endstring
int is_a_number (char* string, char* endstring);

			// Given a token-string and a "thesaurus" in form of null-ended Ch_ThesItem array,
			// tries to get the corresponding "ret_tag". It's an utility function.
			// Returns null if error, true if correctly parsed.
int TranslateWithThesaurus (char* string, char* string_end,
								ChThesItem* thes, void* obj, int classid, ChTag& ret_tag);


			// Some shortcuts for handling variables and tokens,
			// easier and easier versions.
int Get_Var_by_Token (char* string, char* string_end, ChVar& myret, void* database);
int Set_Var_by_Token (char* string, char* string_end, ChVar& myret, void* database);

int Get_Var_by_Name (char* string, ChVar& myret, void* database);
int Set_Var_by_Name (char* string, ChVar& myret, void* database);

int Get_Float_by_Name (char* string, double& myfloat, void* database);
int Set_Float_by_Name (char* string, double& myfloat, void* database);


			// Evaluates a text-based formula, like "2+(3*4)+cos(34)"
			// Returns the result variable of Ch_var type, with proper type.
ChVar Ch_EvalVar (char* string, char* endstring, void* database);

			// Same as before but no need to set endstring.
ChVar Ch_EvalVar (char* string, void* database);

			// Some "easy to use" shortcuts, with double/int
			// automatic casting. Result passed by reference "result".
			// Return TRUE for succesfull parsing, NULL otherwise.
int	Ch_Eval (char* string, void* database, int& result);
int	Ch_Eval (char* string, void* database, double& result);


} // END_OF_NAMESPACE____

#endif  // END of ChFormule.h
