///////////////////////////////////////////////////
//
//   ChFormule.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <float.h>
#include <memory.h>
#include <ctype.h>

#include "core/ChLists.h"
#include "physics/ChObject.h"
#include "physics/ChFormule.h"

namespace chrono
{


enum {
		OP_NONE = 0,
		OP_MUL,
		OP_DIV,
		OP_ADD,
		OP_SUB,
		OP_POW,
};


////////////////////////////////////////////////////////
// Ch_Tag
//          tag class

ChTag::ChTag()
{
	tagID = 0;
	classID = 0;
	object = NULL;
	write = FALSE;
	ret_type = CHCLASS_NONE;
	par1_type = CHCLASS_NONE;
	par2_type = CHCLASS_NONE;
	par3_type = CHCLASS_NONE;
}

ChTag::ChTag(int mtagID, int mclassID, void* mobject, ChThesItem mthesa)
{
	tagID = mtagID;
	classID = mclassID;
	object = mobject;
	write =     mthesa.write;
	ret_type =  mthesa.ret_type;
	par1_type = mthesa.par1_type;
	par2_type = mthesa.par2_type;
	par3_type = mthesa.par3_type;
}

void ChTag::Set(int mtagID, int mclassID, void* mobject, ChThesItem mthesa)
{
	tagID = mtagID;
	classID = mclassID;
	object = mobject;
	write =     mthesa.write;
	ret_type =  mthesa.ret_type;
	par1_type = mthesa.par1_type;
	par2_type = mthesa.par2_type;
	par3_type = mthesa.par3_type;
}


////////////////////////////////////////////////////////
// Ch_var
//          Build-destroy Ch_var object

ChVar::ChVar()
{
	classtype = CHCLASS_NONE;
	varp = NULL;
	size = 0;
	mdouble = 0;
}

ChVar::ChVar (int msize)
{
	classtype = CHCLASS_NONE;
	varp = NULL;
	mdouble = 0;
	size = 0;
	if (msize)
	{
		varp = calloc(msize,1);
		if (varp)
			size = msize;
		else
			size = 0;
	}
}

ChVar::~ChVar ()
{
	if ((size)&&(varp))
	{
		free(varp);
	}
}

ChVar::ChVar (const ChVar& source)
{
	classtype = CHCLASS_NONE;
	varp = NULL;
	size = 0;
	mdouble = 0;

	varp = source.varp;
	classtype = source.classtype;
	mdouble = source.mdouble;

	if (source.varp == &source.mdouble)
	{
		varp = (void*)&mdouble;
		size = 0;
		return;
	}

	if (source.size)
	{
		varp = calloc(source.size, 1);
		size = source.size;
		memcpy (varp, source.varp, size);
		return;
	}
}

void ChVar::Copy(const ChVar* source)
{
	if ((size)&&(varp))
	{
		free(varp);
	}

	classtype = CHCLASS_NONE;
	varp = NULL;
	size = 0;
	mdouble = 0;

	varp = source->varp;
	classtype = source->classtype;
	mdouble = source->mdouble;

	if (source->varp == &source->mdouble)
	{
		varp = (void*)&mdouble;
		size = 0;
		return;
	}

	if (source->size)
	{
		varp = calloc(source->size, 1);
		size = source->size;
		memcpy (varp, source->varp, size);
		return;
	}
}

			// Clean() restores everything as a new Ch_var, and frees the memory block, if any.
void ChVar::Clear()
{
	if ((size)&&(varp))
	{
		free(varp); varp = NULL;
		size = 0;
	}
	classtype = CHCLASS_NONE;
	varp = NULL;
	size = 0;
	mdouble = 0;
}
				// Cleans the var and sets the double value.
void ChVar::ResetAsDouble(double myval)
{
	Clear();
	mdouble = myval;
	varp = &mdouble;
	classtype = CHCLASS_FLOAT;
}
				// Cleans the var and sets the integer value.
void ChVar::ResetAsInt(int myval)
{
	Clear();
	mdouble = (double)myval;
	varp = &mdouble;
	classtype = CHCLASS_INTEGER;
}

void ChVar::ResetAsStringPointer(char* myval)
{
	Clear();
	varp = (void*)myval;
	classtype = CHCLASS_STRINGP;
}

void ChVar::ResetAsPointer(void* myval)
{
	Clear();
	varp = myval;
	classtype = CHCLASS_POINTER;
}

void ChVar::ResetAsVoid()
{
	Clear();
}

void ChVar::ResetAsVector(void* myval)
{
	Clear();
	varp = calloc(sizeof(double),3);
	size = (sizeof(double)) * 3;
	memcpy (varp, myval, size);
	classtype = CHCLASS_VECTOR;
}

//Ch_var Ch_var::operator= (Ch_var source)
//{
//	Ch_var mres;
//	mres(source);
//}

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////


ChEvStacks::ChEvStacks(long psize, long wsize, long dsize)
{
	prog_stack = new ChStack<ChTag>(psize);
	work_stack = new ChStack<ChVar>(wsize);
	data_stack = new ChStack<ChVar>(dsize);
}

ChEvStacks::~ChEvStacks()
{
	delete prog_stack;
	delete work_stack;
	delete data_stack;
}

void ChEvStacks::Clear()
{
	prog_stack->Clear();
	work_stack->Clear();
	data_stack->Clear();
}







////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

int Get_Var_by_Token (char* string, char* string_end, ChVar& myret, void* database)
{
	ChTag mytag;
	if ( ((ChObj*)database)->Translate(string,string_end, mytag) )
		return ( ((ChObj*)mytag.object)->ParseTag(mytag, &myret, NULL, 0) );
	else return FALSE;
}

int Set_Var_by_Token (char* string, char* string_end, ChVar& myret, void* database)
{
	ChTag mytag;
	if ( ((ChObj*)database)->Translate(string,string_end, mytag) )
		return ( ((ChObj*)mytag.object)->ParseTag(mytag, &myret, NULL, 1) );
	else return FALSE;
}

int Get_Var_by_Name (char* string, ChVar& myret, void* database)
{
	char* string_end = string + strlen(string);
	return Get_Var_by_Token (string, string_end, myret, database);
}

int Set_Var_by_Name (char* string, ChVar& myret, void* database)
{
	char* string_end = string + strlen(string);
	return Set_Var_by_Token (string, string_end, myret, database);
}

int Get_Float_by_Name (char* string, double& myfloat, void* database)
{
	ChVar myret;
	if (Get_Var_by_Name (string, myret, database))
		if (myret.classtype == CHCLASS_FLOAT)
		{
			myfloat = myret.mdouble;
			return TRUE;
		}
	return FALSE;
}

int Set_Float_by_Name (char* string, double& myfloat, void* database)
{
	ChVar myret;
	myret.ResetAsDouble(myfloat);
	return (Set_Var_by_Name (string, myret, database));
}


////////////////////////////////////////////////////////


int token_match (char* string, char* endstring, char* compare)
{
	while (TRUE)
	{
		if ((string == endstring)&&(*compare ==0)) return TRUE;
		if ((*compare==0)||(*string==0)) return FALSE;  // if strings terminates before end
		// Char by char comparison, until end of a string, or until 'endstring' reached
		if (*string != *compare) return FALSE;
		string++; compare ++;
	}
	return TRUE;
}

int token_atom_lenght (char* string, char* string_end)
{
	int nchars = 0;
	char* strp = string;
	while (*strp != 0)
	{
		if (memcmp(strp, " ",1) == 0) break;
		if (memcmp(strp, "->",2) == 0) break;
		nchars++;
		strp++;
	}
	if (nchars> (string_end - string)) nchars = (string_end - string);
	return nchars;
}

int goto_next_atom (char*& string, char*& endstring)
{

	if (token_match(string, string+2, (char*)"->"))
	{
		string = string +2;
		if (string > endstring) string = endstring;
		return TRUE;
	}
	return FALSE;
}

int match_parenthesis (char* string, char* endstring)
{
	int nchars = 0;
	int level = 1;
	while (*string != 0)
	{
		nchars++;
		string++;
		if (*string == *"(")
		{
			level++;
		}
		if (*string == *")")
		{
			level--;
			if (level == 0) break;
		}
		if (string == endstring) return 0; // no parenthesis found
		if (*string == 0) return 0; // end string but no parenthesis found
	}
	return nchars;
}

int match_brace (char* string, char* endstring)
{
	int nchars = 0;
	int level = 1;
	while (*string != 0)
	{
		nchars++;
		string++;
		if (*string == *"{")
		{
			level++;
		}
		if (*string == *"}")
		{
			level--;
			if (level == 0) break;
		}
		if (string == endstring) return 0; // no parenthesis found
		if (*string == 0) return 0; // end string but no parenthesis found
	}
	return nchars;
}

int is_a_number (char* string, char* endstring)
{
	int nchars = 0;
	int npoints = 0;
	int mantissa = 0;
	int sign = 0;
	while ((*string != 0)&&(string!= endstring))
	{
		nchars++;

		if ((*string == *"e")||(*string == *"E")) {mantissa++; sign =0;}
		if (mantissa>=2) return FALSE;

		if (*string == *".") npoints++;
		if (npoints>=2) return FALSE;

		if ((*string == *"+")||(*string == *"-")) sign++;
		if (sign>=2) return FALSE;

		if (!(    isdigit(*string)
				|| (*string == *"E")
				|| (*string == *"e")
				|| (*string == *".")
				|| (*string == *"+")
				|| (*string == *"-")
			 )) return FALSE;
		string++;
	}
	if (nchars) return TRUE; else return FALSE;
}


ChVar Ch_EvalVar (char* string, char* endstring, void* database)
{
	ChVar op1, op2;
	char* strpoint;
	int opcode = OP_NONE;

	// 1- Remove enclosing brackets  ()
	//

	if ((memcmp(string, "(",1) == 0) && (memcmp(endstring, ")",1) == 0))
	{
	    ChVar mv;
		mv = Ch_EvalVar(string+1, endstring-1, database);
	    return mv;
		//return Ch_EvalVar (string+1, endstring-1, database); // %%%%% ---recurse
	}

	// 2- Handle arithmetic binary operations
	//

	strpoint = string;
	while (strpoint != endstring)
	{
		// skip ext. parenthesis
		if (memcmp(strpoint, "(",1) == 0)
		{
			int skipped = match_parenthesis (strpoint, endstring);
			if (skipped)
				strpoint =	strpoint + skipped + 1;
			else
				{ break; }// no matching parenthesis!!
		}
		if (memcmp(strpoint, "*",1) == 0) {opcode = OP_MUL; break;}
		if (memcmp(strpoint, "/",1) == 0) {opcode = OP_DIV; break;}
		if (memcmp(strpoint, "+",1) == 0) {opcode = OP_ADD; break;}
		if (memcmp(strpoint, "-",1) == 0) {opcode = OP_SUB; break;}
		strpoint++;
	}
	if (opcode != OP_NONE)
	{
	//	if ((op1.classtype == CHCLASS_FLOAT)&&(op2.classtype == CHCLASS_FLOAT))
	//	{
			ChVar res(sizeof(double)); // instance temp return data
			res.classtype = CHCLASS_FLOAT;
			switch (opcode)
			{
			case OP_MUL:
				*(double*)res.varp  =
					*(double*)(Ch_EvalVar(string, strpoint, database)).varp *
					*(double*)(Ch_EvalVar(strpoint+1, endstring, database)).varp;
				break;
			case OP_DIV:
				*(double*)res.varp  =
					*(double*)(Ch_EvalVar(string, strpoint, database)).varp /
					*(double*)(Ch_EvalVar(strpoint+1, endstring, database)).varp;
				break;
			case OP_ADD:
				*(double*)res.varp  =
					*(double*)(Ch_EvalVar(string, strpoint, database)).varp +
					*(double*)(Ch_EvalVar(strpoint+1, endstring, database)).varp;
				break;
			case OP_SUB:
				*(double*)res.varp  =
					*(double*)(Ch_EvalVar(string, strpoint, database)).varp -
					*(double*)(Ch_EvalVar(strpoint+1, endstring, database)).varp;
				break;
			default: break;
			}
			return res;  // >>>>>>  RETURN
	//	}
	}


	// 3- Parse numbers.. 12345789.43etc  if possible
	//

	double flo= atof (string);
	if ((flo < 99999999)&&(flo > -99999999))
	{
		ChVar res(sizeof(double));
		res.classtype = CHCLASS_FLOAT;
		*(double*)res.varp = flo;
		return res;
	}

	ChVar res; res.classtype = CHCLASS_NONE; // no parsing was possible, so return error
	return res;
}


ChVar Ch_EvalVar (char* string, void* database)
{
	char* stringend = string + strlen (string);
	ChVar mv;
	mv = Ch_EvalVar ( string, stringend,  database);
	return mv;
}

// Some "easy to use" shortcuts, with double/int
// automatic casting
int		Ch_Eval (char* string, void* database, int& result)
{
	result = 0;
	ChVar ret;
	ret = Ch_EvalVar (string, database); // copy from returned data.

	if (ret.classtype == CHCLASS_NONE) return FALSE; // could not parse
	else
	{
		if (ret.classtype == CHCLASS_INTEGER)
		{	result = *(int*)ret.varp;
			return TRUE;}
		if (ret.classtype == CHCLASS_FLOAT)	// cast needed
		{	double mdouble = *(double*)ret.varp;
			result = (int) mdouble;
			return TRUE; }
	}
	return FALSE;
}

int	Ch_Eval (char* string, void* database, double& result)
{
	result = 0;
	ChVar ret;

	ret = Ch_EvalVar (string, database);

	if (ret.classtype == CHCLASS_NONE) return FALSE; // could not parse
	else
	{
		if (ret.classtype == CHCLASS_FLOAT)
		{
			result = *(double*)ret.varp;
			return TRUE;}
		if (ret.classtype == CHCLASS_INTEGER)	// cast needed
		{	double mint = *(int*)ret.varp;
			result = (double) mint;
			return TRUE; }
	}
	return FALSE;
}



////// Evaluation with thesaurus

int TranslateWithThesaurus (char* string, char* string_end,
							ChThesItem* thes, void* obj, int classid,
							ChTag& ret_tag)
{
	int len;

	if (string >= string_end) 		 // was end of buffer:  " ..->mybody"
	{
		ret_tag.object = obj;
		ret_tag.classID = classid;
		ret_tag.tagID = 0;
		ret_tag.ret_type = classid;
		ret_tag.par1_type = ret_tag.par2_type = ret_tag.par3_type = CHCLASS_NONE;
		return TRUE;
	}

	len = token_atom_lenght (string, string_end);

	// Cycle all items of the specified thesaurus,.
	int item = 0;
	while (thes[item].name)
	{
		if (token_match(string, string+len, thes[item].name))
		{
			ret_tag.Set(item+1, classid, obj, thes[item]);
			return TRUE;	// found matching token! :-)
		}
		item++;
	}

	// Too bad, at the end there was no matching....
	ret_tag.object = 0;
	ret_tag.tagID = -2;
	ret_tag.classID = CHCLASS_NONE;
	ret_tag.ret_type= CHCLASS_NONE;
	return FALSE;
}


} // END_OF_NAMESPACE____

////// end

