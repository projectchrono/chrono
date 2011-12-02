//////////////////////////////////////////////////
//  
//   ChModuleCore.i
//
//   SWIG configuration file.
//   This is processed by SWIG to create the C::E
//   wrapper for Python.
//
///////////////////////////////////////////////////



// Define the module to be used in Python when typing 
//  'import ChronoEngine_PYTHON_core as chrono'


%module(directors="1") ChronoEngine_PYTHON_core


// Turn on the documentation of members, for more intuitive IDE typing

%feature("autodoc", "1");


// Include other .i configuration files for SWIG. 
// These are divided in many .i files, each per a
// different c++ class, when possible.

%include "std_string.i"
%include "typemaps.i"

//  core/  classes
%include "ChException.i"
%include "ChHashFunction.i"
%include "ChCoordsys.i" 
%include "ChQuaternion.i" 
%include "ChVector.i" 
%include "ChFrame.i" 
%include "ChFrameMoving.i"
%include "ChLinearAlgebra.i"
%include "ChStream.i"
%include "ChLog.i"
%include "ChMathematics.i"
%include "ChMatrix.i"
%include "ChTimer.i"
%include "ChRealtimeStep.i"
%include "ChTrasform.i"
%include "ChShared.i"

// motion_functions/   classes
%include "ChFunction_Base.i"

//  physics/  classes
%include "ChObject.i"
%include "ChPhysicsItem.i"
%include "ChMaterialSurface.i"
%include "ChBody.i"
%include "ChBodyAuxRef.i"
%include "ChMarker.i"
%include "ChForce.i"
%include "ChSystem.i"
%include "ChContactContainerBase.i"

// collision/   classes
%include "ChCollisionModel.i"




//
// ADDITIONAL C++ FUNCTIONS / CLASSES THAT ARE USED ONLY FOR PYTHON WRAPPER
//

%inline %{

	// Create a custom ChLog class for logging directly in the Python shell,
	// because the default ChLog was redirecting to std::cout that is not 
	// necessarily the console display of python.
namespace chrono
{
class ChLogPython : public ChLog 
{
public:
	ChLogPython() {}
	virtual ~ChLogPython() {};
			/// Redirect output stream to file wrapper.
	virtual void	Output(const char* data, int n) 
		{ 
				char buffer[1000];
				if (n>999) 
					n=999;
				strncpy(buffer, data, n);
				buffer[n]=0;
				PySys_WriteStdout(buffer);
		}
private:
};
};

%}




//
// INITIALIZATION CODE THAT IS EXECUTED AT THE STARTING OF TEH PYTHON UNIT
//

%init %{

		// Create a custom logger to be used all times the GetLog() 
		// funciton is used in C::E to print something. 
	static chrono::ChLogPython static_cout_logger;
	SetLog(static_cout_logger);

%}


//
// ADD PYTHON CODE
//

/*
%pythoncode %{

%}
*/