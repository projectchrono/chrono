///////////////////////////////////////////////////
//
//   Demo code about 
//
//     - how to use the ChFunction objects to
//       easily define y=f(t) functions.
//     - how to call Matlab from Chrono::Engine
//       (in detail, to use Matlab to plot data).
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
#include "physics/ChFunction.h"
#include "unit_MATLAB/ChMatlabEngine.h"


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
		GetLog() << "PERFORM TESTS OF MATLAB<->CHRONOENGINE INTERACTION\n\n";
		GetLog() << "(please wait few seconds: Matlab engine must be loaded)\n\n";

		// This is the object that you can use to access the Matlab engine.
		// As soon as created, it loads the Matlab engine (if troubles happen, it
		// throws exception).
		ChMatlabEngine matlab_engine;

		

		// 
		// EXAMPLE 1: create a ramp ChFunction, set properties, evaluate it.
		//

		GetLog() << "==== Test 1...\n\n";

		ChFunction_Ramp f_ramp;

		f_ramp.Set_ang(0.1);	// set angular coefficient;
		f_ramp.Set_y0(0.4);		// set y value for x=0;

		 // Evaluate y=f(x) function at a given x value, using Get_y() :
		double y	= f_ramp.Get_y(10);
		 // Evaluate derivative df(x)/dx at a given x value, using Get_y_dx() :
		double ydx	= f_ramp.Get_y_dx(10);

		GetLog() << "   ChFunction_Ramp at x=0: y=" << y << "  dy/dx=" << ydx << "\n\n";


		// 
		// EXAMPLE 2: plot a sine ChFunction
		//

		GetLog() << "==== Test 2...\n\n";

		ChFunction_Sine f_sine;

		f_sine.Set_amp(2);		// set amplitude;
		f_sine.Set_freq(0.9);	// set frequency;

		 // Evaluate y=f(x) function along 100 x points:
		ChMatrixDynamic<> x_array(100,1);
		ChMatrixDynamic<> y_array(100,1);
		ChMatrixDynamic<> ydx_array(100,1);
		ChMatrixDynamic<> ydxdx_array(100,1);
		for (int i=0; i<100; i++)
		{
			double x = (double)i/50.0;
			x_array(i)= x;
			y_array(i)= f_sine.Get_y(x);
			ydx_array(i)= f_sine.Get_y_dx(x);
			ydxdx_array(i)= f_sine.Get_y_dxdx(x);
		}
		
		 // Send resulting vectors of values to Matlab 
		matlab_engine.PutVariable(x_array, "x_array");
		matlab_engine.PutVariable(y_array, "y_array");
		matlab_engine.PutVariable(ydx_array, "ydx_array");
		matlab_engine.PutVariable(ydxdx_array, "ydxdx_array");
	
		 // Plot with Matlab 'plot' command
		matlab_engine.Eval("figure;  plot(x_array,y_array,'k');");
		matlab_engine.Eval("hold on; plot(x_array,ydx_array,'g');");
		matlab_engine.Eval("grid on; plot(x_array,ydxdx_array,'r');");
		matlab_engine.Eval("legend ('y','dy/dx','ddy/dxdx');");

		GetLog() << "Press a key to finish... \n";
		getchar(); // pause until key..
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
  
