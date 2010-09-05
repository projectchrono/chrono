//////////////////////////////////////////////////
//
//   ChFx.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "unit_JS/ChFxJavascript.h"
#include "unit_JS/ChJs_Engine.h"


namespace chrono
{


		/// Create the function wrapper.
ChFxJavascript::ChFxJavascript()
		{
			strcpy (function, "");
			strcpy (err_message, "");
			optvarlist = 0;
			fx_script = 0;
		}

ChFxJavascript::~ChFxJavascript()
		{
			if (optvarlist != 0) { ChObj::KillList ((ChObj**)&optvarlist);}
			if (fx_script)
				JS_DestroyScript(CHGLOBALS_JS().chjsEngine->cx, fx_script);
			fx_script = 0;
		}

int ChFxJavascript::SetFunction (char* mformula)
		{
			strcpy (function, mformula);

			// compile function
			if (fx_script)
				JS_DestroyScript(CHGLOBALS_JS().chjsEngine->cx, fx_script);
			fx_script = JS_CompileScript(
							CHGLOBALS_JS().chjsEngine->cx,
							CHGLOBALS_JS().chjsEngine->jglobalObj,
							this->function,
							strlen(this->function),
							"Optimizing function", 0);

			if (!fx_script)  {
				sprintf (err_message, "Error: Javascript objective function can't be compiled.");
				return 0; }

			// compile the optimization variables to speed up the code...
			if (!CompileInVar())	{
				sprintf (err_message, "Error: variables can't be compiled.");
				return 0;	}

			return 1;
		}

void ChFxJavascript::AddInVar (ChOptVar* newvar)
		{
			newvar->AddToList ((ChObj**) &optvarlist);
		}
void ChFxJavascript::RemoveInVar (ChOptVar* newvar)
		{
			newvar->RemoveFromList((ChObj**) &optvarlist);
		}

int ChFxJavascript::GetNumOfVars()
		{
			int nvars = 0;
			ChOptVar* Vlistsource = optvarlist;
			while (Vlistsource != NULL)
			{
				if (Vlistsource->GetLock()== FALSE)
					nvars++;
				Vlistsource = (ChOptVar*) Vlistsource->GetNext();
			}
			return nvars;
		}

int  ChFxJavascript::CompileInVar() 	// to speed up code..
		{
			int i = 1;
			for (ChOptVar* myvar = GetVarList(); myvar != NULL; myvar = (ChOptVar*) myvar->GetNext())
			{
				if (!myvar->Compile( 0 ))  // anyway, Compile() not yet implemented
				{
						sprintf (err_message, "Error: bad syntax in variable n.%d, cannot set&fetch float scalar value", i);
						return 0;
				}
				i++;
			}
			return i-1;
		}

		/// INTERFACE: 
		/// Evaluate A=f(B) 
void ChFxJavascript::Eval  (ChMatrix<>& A,	///< results here
						const ChMatrix<>& B		///< input here
					   )
		{
			// -1-
			// Update the system with the variables in X state:
			this->Vars_to_System(&B);

			// -2-
			// Execute the objective function and gets the value.
			// It assumes that the function to be evaluated is alredy compiled,
			// (it has been already compiled at the beginning of the optimization
			// thank to Ch_optimizer::Optimize()  function).

			jsval rval;
			JSBool ok;
			double fx = 0;

			ok = JS_ExecuteScript(
							CHGLOBALS_JS().chjsEngine->cx,
							CHGLOBALS_JS().chjsEngine->jglobalObj,
							this->fx_script,
							&rval);

			if (ok)
			{
				ok = JS_ValueToNumber(CHGLOBALS_JS().chjsEngine->cx, rval, &fx);
			}
			if (!ok)
			{
				sprintf (err_message, "Error: cannot execute Javascript optimization function.");
			}

			// Set the return value
			A(0,0) = fx;
		}

int ChFxJavascript::Vars_to_System(double x[])
{
	ChOptVar* mvar = GetVarList();

	int index = 0;
	while (mvar != NULL)
	{
		mvar->SetVarValue( x[index], 0 );

		index++;
		mvar = (ChOptVar*) mvar->GetNext();
	}
	return TRUE;
}

int ChFxJavascript::System_to_Vars(double x[])
{
	ChOptVar* mvar = GetVarList();

	int index = 0;
	while (mvar != NULL)
	{
		x[index] = mvar->GetVarValue( 0 );

		index++;
		mvar = (ChOptVar*) mvar->GetNext();
	}
	return TRUE;
}

int ChFxJavascript::Vars_to_System(const ChMatrix<>* x)
{
	ChOptVar* mvar = GetVarList();

	int index = 0;
	while (mvar != NULL)
	{
		mvar->SetVarValue( x->GetElement(index,0), 0 );
		index++;
		mvar = (ChOptVar*) mvar->GetNext();
	}
	return TRUE;
}

int ChFxJavascript::System_to_Vars(ChMatrix<>* x)
{
	ChOptVar* mvar = GetVarList();

	int index = 0;
	while (mvar != NULL)
	{
		x->SetElement(index,0, mvar->GetVarValue( 0 ));
		index++;
		mvar = (ChOptVar*) mvar->GetNext();
	}
	return TRUE;
}




} // END_OF_NAMESPACE____

// eof

