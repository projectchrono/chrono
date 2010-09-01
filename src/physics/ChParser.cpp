///////////////////////////////////////////////////
//
//   ChParser.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

//***OBSOLETE***
 

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <memory.h>

#include "physics/ChParser.h"

namespace chrono
{

//###########################################################################
//
// CLASS FOR PARSER
//
// The object which can transform strings into compiled stack-programs


static ChThesItem parser_thesaurus[] = {
    {"sqrt",		0, CHCLASS_FLOAT,	CHCLASS_FLOAT, 0, 0},				// 0
	{"add",			0, CHCLASS_FLOAT,	CHCLASS_FLOAT, CHCLASS_FLOAT, 0},
	{"move_dw",		0, NULL,			0, 0, 0},
	{"multiply",	0, CHCLASS_FLOAT,	CHCLASS_FLOAT, CHCLASS_FLOAT, 0},
	{"divide",		0, CHCLASS_FLOAT,	CHCLASS_FLOAT, CHCLASS_FLOAT, 0},
	{"subtract",	0, CHCLASS_FLOAT,	CHCLASS_FLOAT, CHCLASS_FLOAT, 0},
	{"pow",			0, CHCLASS_FLOAT,	CHCLASS_FLOAT, CHCLASS_FLOAT, 0},
	{"isequal",		0, CHCLASS_FLOAT,	CHCLASS_FLOAT, CHCLASS_FLOAT, 0},
	{"ismajor",		0, CHCLASS_FLOAT,	CHCLASS_FLOAT, CHCLASS_FLOAT, 0},
	{"isminor",		0, CHCLASS_FLOAT,	CHCLASS_FLOAT, CHCLASS_FLOAT, 0},
	{"sin",			0, CHCLASS_FLOAT,	CHCLASS_FLOAT, 0, 0},				// 10
	{"cos",			0, CHCLASS_FLOAT,	CHCLASS_FLOAT, 0, 0},
	{"asin",		0, CHCLASS_FLOAT,	CHCLASS_FLOAT, 0, 0},
	{"acos",		0, CHCLASS_FLOAT,	CHCLASS_FLOAT, 0, 0},
	{"tan",			0, CHCLASS_FLOAT,	CHCLASS_FLOAT, 0, 0},
	{"atan",		0, CHCLASS_FLOAT,	CHCLASS_FLOAT, 0, 0},
	{"log",			0, CHCLASS_FLOAT,	CHCLASS_FLOAT, 0, 0},
	{"exp",			0, CHCLASS_FLOAT,	CHCLASS_FLOAT, 0, 0},
	{"fv0",			1, CHCLASS_FLOAT,	0, 0, 0},
	{"fv1",			1, CHCLASS_FLOAT,	0, 0, 0},
	{"fv2",			1, CHCLASS_FLOAT,	0, 0, 0},							// 20
	{"fv3",			1, CHCLASS_FLOAT,	0, 0, 0},
	{"fv4",			1, CHCLASS_FLOAT,	0, 0, 0},
	{"fv5",			1, CHCLASS_FLOAT,	0, 0, 0},
	{"store",		0, NULL,			CHCLASS_POINTER, CHCLASS_POINTER, 0},
	{"not",			0, CHCLASS_FLOAT,	CHCLASS_FLOAT,   0, 0},
	{"notequal",	0, CHCLASS_FLOAT,	CHCLASS_FLOAT, CHCLASS_FLOAT, 0},
	{"ifgoto",		0, NULL,			CHCLASS_FLOAT, CHCLASS_POINTER, CHCLASS_POINTER},
	{"notifgoto",	0, NULL,			CHCLASS_FLOAT, CHCLASS_POINTER, CHCLASS_POINTER},
	{"goto",		0, NULL,			CHCLASS_POINTER, CHCLASS_POINTER, 0},
	{"fv6",			1, CHCLASS_FLOAT,	0, 0, 0},							// 30
	{"fv7",			1, CHCLASS_FLOAT,	0, 0, 0},
	{"fv8",			1, CHCLASS_FLOAT,	0, 0, 0},
	{"fv9",			1, CHCLASS_FLOAT,	0, 0, 0},
    {NULL,}
};


ChParser::ChParser ()
{
	database = NULL;
	stacks = new ChEvStacks(200, 200, 200);
	instrp = stacks->prog_stack->GetList()->GetHead();
	datap = stacks->data_stack->GetList()->GetHead();
	memset(fv, 0, (sizeof(double)*10));
};

ChParser::ChParser (ChObj* mdatabase)
{
	database = mdatabase;
	stacks = new ChEvStacks(1000,1000,1000);
	instrp = stacks->prog_stack->GetList()->GetHead();
	datap = stacks->data_stack->GetList()->GetHead();
	memset(fv, 0, (sizeof(double)*10));
};


void ChParser::SetStackSizes(int ds, int ps, int ws)
{
	ClearStacks();
	stacks->data_stack->Set_maxsize(ds);
	stacks->prog_stack->Set_maxsize(ps);
	stacks->work_stack->Set_maxsize(ws);
	instrp = stacks->prog_stack->GetList()->GetHead();
	datap = stacks->data_stack->GetList()->GetHead();
}

void ChParser::MakeErrorMsg (char* message, char* badcommand)
{
	char* str = message;
	int sc = 0; int seq =0;
	int i;
	for (i = 0; i < (ChP_ERRMSG_LEN-1); i++)
	{
		if ((str[sc] == 0)&&(seq)) break;
		if (str[sc] == 0) {str = badcommand; sc = 0; seq++;}
		error_m[i] = str[sc];
		sc++;
	}
	error_m[i] = 0;
	error_m[(ChP_ERRMSG_LEN-1)] = 0;
}


// PARSING TAGS

int ChParser::Translate (char* string, char* string_end,
							  ChTag& ret_tag)
{

	// A -first try to translate using own vocabulary
	if (TranslateWithThesaurus (string, string_end,
							parser_thesaurus, this, CHCLASS_PARSER,
							ret_tag))
		return TRUE;

	// B -ask the parent class to parse it, if not found.
	if (ChObj::Translate(string, string_end, ret_tag))
		return TRUE;

	// C -try to parse database object (example, a PSystem object), if provided
	if (database)
		if (database->Translate(string, string_end, ret_tag))
			return TRUE;

	// D -try to parse numbers
	if (is_a_number(string, string_end))
	{
		double flo= atof (string);
		ChVar* mydata = new ChVar;	// create a Ch_var to store the number
		mydata->ResetAsDouble(flo);
		stacks->data_stack->Push(mydata); // put the var in data stack.
		// set return tag as "move_dw";
		ret_tag.Set(3, CHCLASS_PARSER, this, parser_thesaurus[2]);
		return TRUE;
	}

	return FALSE;	// no parsing was available
}


int ChParser::ParseTag (ChTag mtag, ChVar* retv, ChNode<ChVar>* param, int set)
{
	double result;
	ChVar* mvar;
	ChVar* newvar;
	ChObj* myobj;

	// A - first, try with parent class
	if (ChObj::ParseTag (mtag, retv, param, set)) return TRUE;

	// B - then try with this..
	if (mtag.classID == CHCLASS_PARSER)
	{
		if (!set)	// GET values
		{
			switch (mtag.tagID)
			{
			case 1:	// sqrt
				result = sqrt(*(double*)(param->data->varp));
				retv->ResetAsDouble(result); return TRUE;
			case 2: // add
				result = (*(double*)(param->data->varp)) + (*(double*)(param->next->data->varp));
				retv->ResetAsDouble(result); return TRUE;
			case 3: // moved_dw
				mvar = datap->data; //stacks->data_stack->GetTop();
				newvar = new ChVar;
				newvar->Copy(mvar);
				stacks->work_stack->Push(newvar);
				datap= datap->next; //stacks->data_stack->Pop();
				retv->ResetAsVoid();
				return TRUE;
			case 4: // multiply
				result = (*(double*)(param->data->varp)) * (*(double*)(param->next->data->varp));
				retv->ResetAsDouble(result); return TRUE;
			case 5:	// divide
				result = (*(double*)(param->data->varp)) / (*(double*)(param->next->data->varp));
				retv->ResetAsDouble(result); return TRUE;
			case 6: // subtract
				result = (*(double*)(param->data->varp)) - (*(double*)(param->next->data->varp));
				retv->ResetAsDouble(result); return TRUE;
			case 7: // pow
				result = pow((*(double*)(param->data->varp)),(*(double*)(param->next->data->varp)));
				retv->ResetAsDouble(result); return TRUE;
			case 8: // isequal
				result = ((*(double*)(param->data->varp))==(*(double*)(param->next->data->varp)));
				retv->ResetAsDouble(result); return TRUE;
			case 9: // ismajor
				result = ((*(double*)(param->data->varp))>(*(double*)(param->next->data->varp)));
				retv->ResetAsDouble(result); return TRUE;
			case 10: // isminor
				result = ((*(double*)(param->data->varp))<(*(double*)(param->next->data->varp)));
				retv->ResetAsDouble(result); return TRUE;
			case 11: // sin
				result = sin((*(double*)(param->data->varp)));
				retv->ResetAsDouble(result); return TRUE;
			case 12: // cos
				result = cos((*(double*)(param->data->varp)));
				retv->ResetAsDouble(result); return TRUE;
			case 13: // asin
				result = asin((*(double*)(param->data->varp)));
				retv->ResetAsDouble(result); return TRUE;
			case 14: // acos
				result = acos((*(double*)(param->data->varp)));
				retv->ResetAsDouble(result); return TRUE;
			case 15: // tan
				result = tan((*(double*)(param->data->varp)));
				retv->ResetAsDouble(result); return TRUE;
			case 16: // atan
				result = atan((*(double*)(param->data->varp)));
				retv->ResetAsDouble(result); return TRUE;
			case 17: // log
				result = log((*(double*)(param->data->varp)));
				retv->ResetAsDouble(result); return TRUE;
			case 18: // exp
				result = exp((*(double*)(param->data->varp)));
				retv->ResetAsDouble(result); return TRUE;
			case 19: // fv0
				retv->ResetAsDouble(fv[0]); return TRUE;
			case 20: // fv1
				retv->ResetAsDouble(fv[1]); return TRUE;
			case 21: // fv2
				retv->ResetAsDouble(fv[2]); return TRUE;
			case 22: // fv3
				retv->ResetAsDouble(fv[3]); return TRUE;
			case 23: // fv4
				retv->ResetAsDouble(fv[4]); return TRUE;
			case 24: // fv5
				retv->ResetAsDouble(fv[5]); return TRUE;
			case 25: // store   A=B
				myobj = (ChObj*)((ChTag*)(param->data->varp))->object;
				myobj->ParseTag(*(ChTag*)(param->data->varp),	// tag to set
								param->next->data,		// value data to set (Ch_var)
								NULL,		// no params
								TRUE);		// SET tag mode
				retv->ResetAsVoid();
				return TRUE;
			case 26: // not
				result = !(*(double*)(param->data->varp));
				retv->ResetAsDouble(result); return TRUE;
			case 27: // notequal
				result = ((*(double*)(param->data->varp))!=(*(double*)(param->next->data->varp)));
				retv->ResetAsDouble(result); return TRUE;
			case 28: // ifgoto
				if((*(double*)(param->data->varp))!=0)
				{
					this->goto_instrp = (ChNode<ChTag>*)(param->next->data->varp);
					this->datap  = (ChNode<ChVar>*)(param->next->next->data->varp);
				}
				retv->ResetAsVoid(); return TRUE;
			case 29: // notifgoto
				if((*(double*)(param->data->varp))==0)
				{
					this->goto_instrp = (ChNode<ChTag>*)(param->next->data->varp);
					this->datap  = (ChNode<ChVar>*)(param->next->next->data->varp);
				}
				retv->ResetAsVoid(); return TRUE;
			case 30: // goto
				this->goto_instrp = (ChNode<ChTag>*)(param->data->varp);
				this->datap  = (ChNode<ChVar>*)(param->next->data->varp);
				retv->ResetAsVoid(); return TRUE;
			case 31: // fv6
				retv->ResetAsDouble(fv[6]); return TRUE;
			case 32: // fv7
				retv->ResetAsDouble(fv[7]); return TRUE;
			case 33: // fv8
				retv->ResetAsDouble(fv[8]); return TRUE;
			case 34: // fv9
				retv->ResetAsDouble(fv[9]); return TRUE;
			default:
				return FALSE;
			}
		}
		else		// SET values
		{
			switch (mtag.tagID)
			{
			case 19: // fv0
				fv[0] = (*(double*)retv->varp); return TRUE;
			case 20: // fv1
				fv[1] = (*(double*)retv->varp); return TRUE;
			case 21: // fv2
				fv[2] = (*(double*)retv->varp); return TRUE;
			case 22: // fv3
				fv[3] = (*(double*)retv->varp); return TRUE;
			case 23: // fv4
				fv[4] = (*(double*)retv->varp); return TRUE;
			case 24: // fv5
				fv[5] = (*(double*)retv->varp); return TRUE;
			case 31: // fv6
				fv[6] = (*(double*)retv->varp); return TRUE;
			case 32: // fv7
				fv[7] = (*(double*)retv->varp); return TRUE;
			case 33: // fv8
				fv[8] = (*(double*)retv->varp); return TRUE;
			case 34: // fv9
				fv[9] = (*(double*)retv->varp); return TRUE;
			default:
				return FALSE;
			}
		}
	}

	// C -try to parse database object (example, a PSystem object), if provided
	if (database)
		if (database->ParseTag (mtag, retv, param, set))
			return TRUE;

	return FALSE;	// sorry, no parent class nor child objects could parse it.
}



/////// RECURSIVE COMPILATION

enum {
	PARID_NULL = 0,
	PARID_PLUS,
	PARID_MINUS,
	PARID_MUL,
	PARID_DIV,
	PARID_FX,
	PARID_MINOR,
	PARID_MAJOR,
	PARID_ISEQUAL,
	PARID_STORE,
	PARID_COMMA,
	PARID_BICOMMA,
	PARID_WHILE,
	PARID_NOT,
	PARID_NOTEQUAL,
	PARID_POINTER,
};


int ChParser::REC_Compile (char* string, char* endstring)
{
	char* strpoint;
	ChTag* tag_command;

	//  Remove spaces
	while (*string == *" ")
		string++;
	while (*(endstring-1) == *" ")
		endstring--;

	//
	// 1- Remove enclosing brackets  ()  if any
	//

	if ( (*string == *"(" ) && ( *(endstring-1) == *")" ) )
	{
		return REC_Compile (string+1, endstring-1); // %%%%% ---recurse
	}

	if ( (*string == *"{" ) && ( *(endstring-1) == *"}" ) )
	{
		return REC_Compile (string+1, endstring-1); // %%%%% ---recurse
	}

	//  Remove spaces
	while (*string == *" ")
		string++;
	while (*(endstring-1) == *" ")
		endstring--;




	//
	// 2- Handle syntax: find operator with lowest precedence
	//

	char *sA_from, *sA_to, *sB_from, *sB_to = NULL;
	int precedence = 100;
	int opcode = PARID_NULL;
	sA_from  = string;
	sA_to = endstring;
	ChNode<ChVar>* mydatap;
	ChNode<ChTag>* myinstrp;

	strpoint = string;
	while (strpoint < endstring)
	{
		// handle parenthesis
		if (*strpoint == *"(" )
		{
			int skipped = match_parenthesis (strpoint, endstring);

			if (skipped == NULL)
			{
				MakeErrorMsg("No matching parenthesis: ", strpoint); // ** ERROR
				return FALSE;
			}
			if ((strpoint + skipped)==(endstring-1)) // the syntax was "fx(yyyy)"
			{
				if (precedence>=40)   // fx(yyy)
				{
					opcode = PARID_FX;
					precedence = 40;
					sA_from = string; sA_to = strpoint;
					sB_from = strpoint+1 ; sB_to = endstring-1;
					break;
				}
			}
			if ((skipped)&&(opcode!=PARID_FX))	 // skipping the parenthesis
			{
				strpoint = strpoint + skipped + 1;
				if (strpoint == endstring) break;
			};
		}

		// handle "while" cycle.
		if (token_match(strpoint, strpoint+5, "while"))
		{
			char* wistrpoint = strpoint+5;
			while (*wistrpoint == *" ") wistrpoint++; // skip spaces
			if (*wistrpoint != *"(")
			{
				MakeErrorMsg("After 'while' there must be a (..) condition: ", wistrpoint); // ** ERROR
				return FALSE;
			}
			int skipped = match_parenthesis (wistrpoint, endstring);
			if (skipped == NULL)
			{
				MakeErrorMsg("No matching ')' brace: ", wistrpoint); // ** ERROR
				return FALSE;
			}
			char* wisA_from = wistrpoint+1;
			char* wisA_to = wistrpoint+skipped;
			wistrpoint = wistrpoint+skipped+1;
			while (*wistrpoint == *" ") wistrpoint++; // skip spaces
			if (*wistrpoint != *"{")
			{
				MakeErrorMsg("After 'while()' there must be an opening '{' brace: ", wistrpoint); // ** ERROR
				return FALSE;
			}
			int skipcycle = match_brace (wistrpoint, endstring);
			if (skipcycle == NULL)
			{
				MakeErrorMsg("No matching '}' brace: ", wistrpoint); // ** ERROR
				return FALSE;
			}
			if (precedence>=38)
			{
				opcode = PARID_WHILE;
				precedence = 45;
				sA_from = wisA_from;
				sA_to = wisA_to;
				sB_from = wistrpoint+1 ;
				sB_to = wistrpoint+skipcycle;
				wistrpoint = wistrpoint + skipcycle + 1;
				if (wistrpoint == endstring) break;
				strpoint = wistrpoint;
			}
		}

		if ((*strpoint == *";")&&(precedence>5))
		{
			opcode = PARID_BICOMMA;
			precedence = 5;
			sB_from = string; sB_to = strpoint;
			sA_from = strpoint+1 ; sA_to = endstring;
		}

		if ((*strpoint == *",")&&(precedence>10))
		{
			opcode = PARID_COMMA;
			precedence = 10;
			sA_from = string; sA_to = strpoint;
			sB_from = strpoint+1 ; sB_to = endstring;
		}

		if ((token_match(strpoint, strpoint+2, "=="))&&(precedence>15))
		{
			opcode = PARID_ISEQUAL;
			precedence = 15;
			sA_from = string; sA_to = strpoint;
			sB_from = strpoint+2 ; sB_to = endstring;
		}
		if ((token_match(strpoint, strpoint+2, "->"))&&(precedence>15))
		{
			opcode = PARID_POINTER;
			precedence = 50;
			//sA_from = string; sA_to = strpoint;
			//sB_from = strpoint+2 ; sB_to = endstring;
		}
		if ((token_match(strpoint, strpoint+2, "!="))&&(precedence>15))
		{
			opcode = PARID_NOTEQUAL;
			precedence = 15;
			sA_from = string; sA_to = strpoint;
			sB_from = strpoint+2 ; sB_to = endstring;
		}
		if ((*strpoint == *">")&&(precedence>15)&&(opcode != PARID_POINTER) )
		{
			opcode = PARID_MAJOR;
			precedence = 15;
			sA_from = string; sA_to = strpoint;
			sB_from = strpoint+1 ; sB_to = endstring;
		}
		if ((*strpoint == *"<")&&(precedence>15))
		{
			opcode = PARID_MINOR;
			precedence = 15;
			sA_from = string; sA_to = strpoint;
			sB_from = strpoint+1 ; sB_to = endstring;
		}
		if ((*strpoint == *"!")&&(precedence>16)&&(opcode != PARID_NOTEQUAL) )
		{
			opcode = PARID_NOT;
			precedence = 16;
			sA_from = strpoint+1; sA_to = endstring;
		}

		if ((*strpoint == *"=")&&(precedence>12)&&(opcode != PARID_ISEQUAL)&&(opcode != PARID_NOTEQUAL) )
		{
			opcode = PARID_STORE;
			precedence = 12;
			sA_from = string; sA_to = strpoint;
			sB_from = strpoint+1 ; sB_to = endstring;
		}

		if ((*strpoint == *"+")&&(precedence>20))
		{
			opcode = PARID_PLUS;
			precedence = 20;
			sA_from = string; sA_to = strpoint;
			sB_from = strpoint+1 ; sB_to = endstring;
		}
		if ((*strpoint == *"-")&&(precedence>20)&&(opcode != PARID_POINTER))
		{
			opcode = PARID_MINUS;
			precedence = 20;
			sA_from = string; sA_to = strpoint;
			sB_from = strpoint+1 ; sB_to = endstring;
		}
		if ((*strpoint == *"*")&&(precedence>22))
		{
			opcode = PARID_MUL;
			precedence = 22;
			sA_from = string; sA_to = strpoint;
			sB_from = strpoint+1 ; sB_to = endstring;
		}
		if ((*strpoint == *"/")&&(precedence>22))
		{
			opcode = PARID_DIV;
			precedence = 22;
			sA_from = string; sA_to = strpoint;
			sB_from = strpoint+1 ; sB_to = endstring;
		}

		strpoint++;
	}

	//
	// 3- Branch the parsing
	//
	ChVar* mydatatag = NULL;
	ChTag* mynewtag = NULL;
	ChVar* mydatatagEndInstr = NULL;
	ChVar* mydatatagEndData = NULL;
	int oldins, olddata = NULL;

	switch (opcode)
	{
	case PARID_FX:
			return (  (REC_Compile(sA_from,  sA_to )) 	 // fx
					&&(REC_Compile(sB_from,  sB_to )));  // yyyy %%%%% ---recurse
	case PARID_BICOMMA:
			return (  (REC_Compile(sA_from,  sA_to ))
					&&(REC_Compile(sB_from,  sB_to )));
	case PARID_COMMA:
			return (  (REC_Compile(sA_from,  sA_to ))
					&&(REC_Compile(sB_from,  sB_to )));
	case PARID_ISEQUAL:
			tag_command = new ChTag;
			tag_command->Set(8, CHCLASS_PARSER, this, parser_thesaurus[7]);
			stacks->prog_stack->Push(tag_command); // &&&&&&&& create command
			return (  (REC_Compile(sA_from,  sA_to ))
					&&(REC_Compile(sB_from,  sB_to )));
	case PARID_MAJOR:
			tag_command = new ChTag;
			tag_command->Set(9, CHCLASS_PARSER, this, parser_thesaurus[8]);
			stacks->prog_stack->Push(tag_command); // &&&&&&&& create command
			return (  (REC_Compile(sA_from,  sA_to ))
					&&(REC_Compile(sB_from,  sB_to )));
	case PARID_MINOR:
			tag_command = new ChTag;
			tag_command->Set(10, CHCLASS_PARSER, this, parser_thesaurus[9]);
			stacks->prog_stack->Push(tag_command); // &&&&&&&& create command
			return (  (REC_Compile(sA_from,  sA_to ))
					&&(REC_Compile(sB_from,  sB_to )));
	case PARID_PLUS:
			tag_command = new ChTag;
			tag_command->Set(2, CHCLASS_PARSER, this, parser_thesaurus[1]);
			stacks->prog_stack->Push(tag_command); // &&&&&&&& create command
			return (  (REC_Compile(sA_from,  sA_to ))
					&&(REC_Compile(sB_from,  sB_to )));
	case PARID_MINUS:
			tag_command = new ChTag;
			tag_command->Set(6, CHCLASS_PARSER, this, parser_thesaurus[5]);
			stacks->prog_stack->Push(tag_command); // &&&&&&&& create command
			return (  (REC_Compile(sA_from,  sA_to ))
					&&(REC_Compile(sB_from,  sB_to )));
	case PARID_MUL:
			tag_command = new ChTag;
			tag_command->Set(4, CHCLASS_PARSER, this, parser_thesaurus[3]);
			stacks->prog_stack->Push(tag_command); // &&&&&&&& create command
			return (  (REC_Compile(sA_from,  sA_to ))
					&&(REC_Compile(sB_from,  sB_to )));
	case PARID_DIV:
			tag_command = new ChTag;
			tag_command->Set(5, CHCLASS_PARSER, this, parser_thesaurus[4]);
			stacks->prog_stack->Push(tag_command); // &&&&&&&& create command
			return (  (REC_Compile(sA_from,  sA_to ))
					&&(REC_Compile(sB_from,  sB_to )));
	case PARID_NOT:
			tag_command = new ChTag;
			tag_command->Set(26, CHCLASS_PARSER, this, parser_thesaurus[25]);
			stacks->prog_stack->Push(tag_command); // &&&&&&&& create command
			return (REC_Compile(sA_from,  sA_to ));
	case PARID_NOTEQUAL:
			tag_command = new ChTag;
			tag_command->Set(27, CHCLASS_PARSER, this, parser_thesaurus[26]);
			stacks->prog_stack->Push(tag_command); // &&&&&&&& create command
			return (  (REC_Compile(sA_from,  sA_to ))
					&&(REC_Compile(sB_from,  sB_to )));
	case PARID_STORE:
			tag_command = new ChTag;
			tag_command->Set(25, CHCLASS_PARSER, this, parser_thesaurus[24]);
			stacks->prog_stack->Push(tag_command); // &&&&&&&& create "store" command
			oldins = this->stacks->prog_stack->Get_used();
			olddata = this->stacks->data_stack->Get_used();

			if (!REC_Compile(sA_from,  sA_to ))
				return FALSE;
			if (((this->stacks->prog_stack->Get_used() - oldins) != 1 ) ||
				(this->stacks->data_stack->Get_used() != olddata )  )
				{
					MakeErrorMsg("syntax error at the left side of '=' sign: ", string);
					return FALSE; }// ** ERROR

			mynewtag = new ChTag;
			*mynewtag = *(stacks->prog_stack->GetTop());
			stacks->prog_stack->Pop(); // remove the tag after copying it in the Ch_var of the data stack.

			mydatatag = new ChVar;	// Create a Ch_var to store the tag.
			mydatatag->ResetAsPointer(mynewtag);
			mydatatag->size = sizeof(ChTag);  // deleting var now will delete pointed tag too.
			stacks->data_stack->Push(mydatatag); // put the var in data stack.

			tag_command = new ChTag;
			tag_command->Set(3, CHCLASS_PARSER, this, parser_thesaurus[2]);
			stacks->prog_stack->Push(tag_command); // &&&&&&&& create "move dw" command

			if (!REC_Compile(sB_from,  sB_to ))
				return FALSE;	// compile things to store into tag-variable
			return TRUE;

	case PARID_WHILE:
			myinstrp = stacks->prog_stack->GetList()->GetHead(); // store data/program pointer before inserting the while loop
			mydatap = stacks->data_stack->GetList()->GetHead();

			tag_command = new ChTag;
			 tag_command->Set(30, CHCLASS_PARSER, this, parser_thesaurus[29]);
			 stacks->prog_stack->Push(tag_command); // &&&&&&&& create "goto" command for the repeat of loop

			mydatatagEndInstr = new ChVar;	// Create a Ch_var to store the instr.pointer.
			 stacks->data_stack->Push(mydatatagEndInstr); // put the var in data stack.
			tag_command = new ChTag;
			 tag_command->Set(3, CHCLASS_PARSER, this, parser_thesaurus[2]);
			 stacks->prog_stack->Push(tag_command); // &&&&&&&& create "move dw" command

			mydatatagEndData = new ChVar;	// Create a Ch_var to store the data.pointer.
			 stacks->data_stack->Push(mydatatagEndData); // put the var in data stack.
			tag_command = new ChTag;
			 tag_command->Set(3, CHCLASS_PARSER, this, parser_thesaurus[2]);
			 stacks->prog_stack->Push(tag_command); // &&&&&&&& create "move dw" command


			if (!REC_Compile(sB_from,  sB_to ))   // compile 'while' loop code
				return FALSE;

			tag_command = new ChTag;
			 tag_command->Set(29, CHCLASS_PARSER, this, parser_thesaurus[28]);
			 stacks->prog_stack->Push(tag_command); // &&&&&&&& create "notifgoto" command

			if (!REC_Compile(sA_from,  sA_to ))   // compile code returning condition
				return FALSE;

			mydatatag = new ChVar;	// Create a Ch_var to store the instr.pointer.
			 mydatatag->ResetAsPointer(myinstrp);
			 stacks->data_stack->Push(mydatatag); // put the var in data stack.
			tag_command = new ChTag;
			 tag_command->Set(3, CHCLASS_PARSER, this, parser_thesaurus[2]);
			 stacks->prog_stack->Push(tag_command); // &&&&&&&& create "move dw" command

			mydatatag = new ChVar;	// Create a Ch_var to store the data.pointer.
			 mydatatag->ResetAsPointer(mydatap);
			 stacks->data_stack->Push(mydatatag); // put the var in data stack.
			tag_command = new ChTag;
			 tag_command->Set(3, CHCLASS_PARSER, this, parser_thesaurus[2]);
			 stacks->prog_stack->Push(tag_command); // &&&&&&&& create "move dw" command

			mydatatagEndInstr->ResetAsPointer(stacks->prog_stack->GetList()->GetHead());
			mydatatagEndData->ResetAsPointer(stacks->data_stack->GetList()->GetHead());

			return TRUE;

	case PARID_NULL:
		break;
	default:
		break;
	}


	//    If the parser has not splitted recursively...
	// 4- Try to parse as variables  "bodyA->mass"  or numbers "12345.67" etc. if possible
	//    using the translation function of this class and the one of the database.

	ChTag mytag;
	if (Translate (string, endstring, mytag))
	{
		tag_command = new ChTag;
		*tag_command = mytag;
		stacks->prog_stack->Push(tag_command); // &&&&&&&&&&&& create command
		return TRUE;
	}

	MakeErrorMsg("syntax error: ", string); // ** ERROR compilation not possible

	return FALSE;
}


int ChParser::Compile (char* string)
{

	MakeErrorMsg("","");

	ClearStacks();

	return REC_Compile (string, string+strlen(string));

}



int ChParser::Execute()
{
	MakeErrorMsg("","");

	// Initialize instruction pointer at the beginning of progr.stack
	instrp = stacks->prog_stack->GetList()->GetHead();
	datap = stacks->data_stack->GetList()->GetHead();

	// Kill results on the work stack
	stacks->work_stack->Clear();

	// Perform all the instructions in stack, in sequence
	while (instrp != NULL)
	{
		// the pointer instr_nostep will be set by instructions which handle instruction pointer like ifgoto, goto. If NULL, normal pointer increment.
		goto_instrp = NULL;

		// instance a ch-variable to put on work_stack as return value
		ChVar* retvar = new ChVar();

		// Set the return variable by EXECUTION of the command on prog_stack
		if (!(this->ParseTag(
					   *(instrp->data),	// current Ch_Tag instruction
						retvar,	   	    // pointer to return value variable
						stacks->work_stack->GetList()->GetHead(), // pointer to node in list of parameters (top of work_stack)
						0) ) )
		{
			char buff[50];
			sprintf(buff, "ID: %d, CLASS: %d", instrp->data->tagID, instrp->data->classID);
			MakeErrorMsg("Tag not supported: ", buff); // ** ERROR
			return FALSE;
		}

		//if ((instrp == NULL)||(datap==NULL))  return TRUE; // a 'goto' has moved to end of program

		// delete the used parameters from work_stack
		if (instrp->data->par1_type != NULL) stacks->work_stack->Pop();
		if (instrp->data->par2_type != NULL) stacks->work_stack->Pop();
		if (instrp->data->par3_type != NULL) stacks->work_stack->Pop();

		// push the return variable (if not void) on the return work_stack
		if (retvar->classtype != CHCLASS_NONE)
			stacks->work_stack->Push(retvar);
		else
			delete retvar;

		// move instruction node to next instruction;
		if (goto_instrp)
			instrp = goto_instrp;
		else
			instrp = instrp->next;
	}

	return TRUE; // stacks->work_stack->GetTop();
}




////////   shortcuts, easy-to-use, etc...

int ChParser::Execute(double& result)
{
	if (!Execute()) return FALSE;
	if (stacks->work_stack->GetTop() == NULL)
	{
		MakeErrorMsg ("Function does not return value","");
		return FALSE;
	}
	if (stacks->work_stack->GetTop()->classtype != CHCLASS_FLOAT)
	{
		MakeErrorMsg ("Function return type is not a float point number","");
		return FALSE;
	}
	else result = stacks->work_stack->GetTop()->mdouble;

	return TRUE;
}

int ChParser::Evaluate(char* string)
{
	if (!Compile(string)) return FALSE;
	if (!Execute()) return FALSE;
	return TRUE;
}

int ChParser::Evaluate(char* string, double& result)
{
	if (!Compile(string)) return FALSE;
	if (!Execute(result)) return FALSE;
	result = result;
	return TRUE;
}


} // END_OF_NAMESPACE____

////// end
