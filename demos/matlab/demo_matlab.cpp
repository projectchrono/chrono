///////////////////////////////////////////////////
//
//   Demo code about 
//
//     - how to call Matlab from Chrono::Engine
//
//	 CHRONO    
//   ------
//   Multibody dinamics engine
//  
// ------------------------------------------------ 
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------ 
///////////////////////////////////////////////////
 
  
 
#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "matlab_interface/ChMatlabEngine.h"


// Use the namespace of Chrono

using namespace chrono;


 
int main(int argc, char* argv[])
{
	// The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed.
	DLL_CreateGlobals();

	// Better put the Matlab stuff inside a try{}, since it may throw exception if 
	// the engine is not started (because Matlab not properly installed)
	try
	{
		// This is the object that you can use to access the Matlab engine.
		// As soon as created, it loads the Matlab engine (if troubles happen, it
		// throws exception).
		ChMatlabEngine matlab_engine;

		
		GetLog() << "PERFORM TESTS OF MATLAB<->CHRONOENGINE INTERACTION\n\n";

		// 
		// EXAMPLE 1: execute a Matlab command
		//

		GetLog() << "- Execute plotting command from Chrono::Engine...\n\n";

		matlab_engine.Eval("z=peaks(25); \
						    surf(z);  \
							colormap(jet); \
							pause(4); \
							");

		// 
		// EXAMPLE 2: pass a Chrono matrix to Matlab
		//

		GetLog() << "- Send some data to Matlab for operations and plotting...\n\n";

		ChMatrixDynamic<> m_time(30,1);
		ChMatrixDynamic<> m_sine(30,1);
		for(int i=0; i<30; i++)
		{
			m_time(i,0)= ((double)i/30.)*5.;
			m_sine(i,0)= sin ( m_time(i,0) *2.);
		}
		matlab_engine.PutVariable(m_time, "m_time");
		matlab_engine.PutVariable(m_sine, "m_sine");
		matlab_engine.Eval("figure; plot(m_time,m_sine);");

		// 
		// EXAMPLE 3: pass a Matlab matrix to Chrono
		//

		GetLog() << "- Fetch some data from Matlab...\n\n";

		matlab_engine.Eval("m_matr=[0:0.1:5]';");

		ChMatrixDynamic<double> m_matr;
		matlab_engine.GetVariable(m_matr, "m_matr");
		GetLog() << m_matr;

		// Wait some seconds before closing all

		matlab_engine.Eval("pause(4)");

	}
	catch (ChException mex)
	{
		GetLog() << mex.what(); // Print error on console, if Matlab did not start.
	}
 
	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}
  
