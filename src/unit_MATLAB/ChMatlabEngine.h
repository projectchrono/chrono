#ifndef CHMATLABENGINE_H
#define CHMATLABENGINE_H

///////////////////////////////////////////////////
//  
//   ChMatlabEngine.h
//
//   Use this header if you want to exploit Matlab
//   from Chrono::Engine programs.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChMath.h"

// include also the Matlab header..
#include "engine.h"



namespace chrono 
{


/// Class for accessing the Matlab engine with a C++ wrapper.
/// When a ChMatlabEngine object is instanced, a Matlab engine
/// is started (assuming Matlab is properly installed and its
/// dll are available on the system) and following funcitons can
/// be used to copy variables from/to chrono::engine, and
/// to execute commands in Matlab. Useful also to exploit the
/// powerful plotting features of Matlab.
/// Note! to compile programs that include this header, your 
/// makefile must be properly configured to set the Matlab SDK
/// directory, so that matlab headers and libs can be compiled/linked
/// Also, if you run an executable that used this header on a 
/// system that has no Matlab installed, a 'missing dll' error
/// will pop up as soon as you try to start the program.

class ChMatlabEngine 
{
private:
		// 
		// DATA
		// 
	Engine *ep;

public:
		// 
		// FUNCTIONS
		// 

	ChMatlabEngine()
	{
		if (!(ep = engOpen("-automation \0"))) 
		{
			throw ChException("Can't start MATLAB engine");
		}
	}

	~ChMatlabEngine()
	{
		if (ep) engClose(ep); ep = 0;	
	}
		/// Return pointer to internal Matlab engine (avoid using it directly,
		/// if you can use other functions of this class that 'wrap' it.)
	Engine* GetEngine() {return ep;}

		/// Evaluate a Matlab instruction. If error happens while executing, returns false.
	bool Eval(char* mstring)
	{
		if (engEvalString(ep, mstring) == 0) return true;
		else return false;
	}
		/// Set visibility of GUI matlab window.
	bool SetVisible(bool mvis)
	{
		if (engSetVisible(ep, mvis) == 0) return true;
		else return false;
	}

		/// Put a matrix in Matlab environment, specifying its name as variable.
		/// If a variable with the same name already exist, it is overwritten.
	bool PutVariable(const ChMatrix<double>& mmatr, char* varname)
	{
		mxArray *T = NULL;
		T = mxCreateDoubleMatrix(mmatr.GetRows(), mmatr.GetColumns(), mxREAL);
		memcpy((char *) mxGetPr(T), (char *) mmatr.GetAddress(), mmatr.GetRows()*mmatr.GetColumns()*sizeof(double));
		engPutVariable(ep, varname, T);
		mxDestroyArray(T);
		return true;
	}
		/// Fetch a matrix from Matlab environment, specifying its name as variable.
		/// The used matrix must be of ChMatrixDynamic<double> type because 
		/// it might undergo resizing.
	bool GetVariable(ChMatrixDynamic<double>& mmatr, char* varname)
	{
		mxArray* T = engGetVariable(ep, varname);
		if (T)
		{
			mwSize  ndi = mxGetNumberOfDimensions(T);
			if (ndi != 2) 
			{
				mxDestroyArray(T);
				return false;
			}
			const mwSize* siz = mxGetDimensions(T);
			mmatr.Resize(siz[0], siz[1]);
			memcpy((char *) mmatr.GetAddress(), (char *) mxGetPr(T), mmatr.GetRows()*mmatr.GetColumns()*sizeof(double));
			mxDestroyArray(T);
			return true;
		}
		mxDestroyArray(T);
		return false;
	}


};





} // END_OF_NAMESPACE____


#endif
