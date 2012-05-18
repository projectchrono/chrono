///////////////////////////////////////////////////
//
//   ChPython.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 

#include "unit_PYTHON/ChPython.h"
#include <Python.h>
#include <iostream>
#include <sstream>

#ifndef CH_API_COMPILE_UNIT_PYPARSER
#error Warning! You are compiling the PYTHON unit of Chrono::Engine, \
	so you need to define CH_API_COMPILE_UNIT_PYTHON (add that symbol \
	to the compiler defines, for all compiled files in this unit). 
#endif 



namespace chrono
{



ChPythonEngine::ChPythonEngine()
{
	Py_Initialize();
}

ChPythonEngine::~ChPythonEngine()
{
	Py_Finalize();
}

void ChPythonEngine::Run(const char* program)
{
	/*
	// Redirect stdout to one std::string buffer.
	std::streambuf *msgOutBuffer, *msgOutBufferBackup;
	std::stringstream smessageOutput;
	msgOutBufferBackup = std::cout.rdbuf();
	msgOutBuffer = smessageOutput.rdbuf();
	std::cout.rdbuf(msgOutBuffer);
	 */
	// Redirect stderr to one std::string buffer.
	std::streambuf *msgErrBuffer, *msgErrBufferBackup;
	std::stringstream smessageError;
	msgErrBufferBackup = std::cerr.rdbuf();
	msgErrBuffer = smessageError.rdbuf();
	std::cerr.rdbuf(msgErrBuffer);
	 
	if (!PyRun_SimpleString(program))
	{
		PyErr_Print();
		throw ChException(smessageError.str());
	}

	// Set to previous channels!
	/*
	std::cout.rdbuf(msgOutBufferBackup);
	*/
	std::cerr.rdbuf(msgErrBufferBackup);
}
		
bool ChPythonEngine::GetFloat(const char* variable, double& return_val)
{
	PyObject * module = PyImport_AddModule("__main__"); // borrowed reference
	assert(module);                                     // __main__ should always exist

	PyObject * dictionary = PyModule_GetDict(module);   // borrowed reference
	assert(dictionary);                                 // __main__ should have a dictionary

	PyObject * result = PyDict_GetItemString(dictionary, variable);   // borrowed reference
	if (!result) 
		return false;

	if (PyFloat_Check(result))
	{
		return_val = PyFloat_AS_DOUBLE(result);
		return true;
	}
	if (PyLong_Check(result)) 
	{
		return_val = (double)PyLong_AS_LONG(result); // cast py int to c++ double - it's ok anyway
		return true;
	}

	return false;
}
			
void ChPythonEngine::SetFloat(const char* variable, const double val)
{
	std::ostringstream sstream;
	sstream << variable << "=" << val <<"\n";

	this->Run(sstream.str().c_str());
}


bool ChPythonEngine::GetInteger(const char* variable, int& return_val)
{
	PyObject * module = PyImport_AddModule("__main__"); // borrowed reference
	assert(module);                                     // __main__ should always exist

	PyObject * dictionary = PyModule_GetDict(module);   // borrowed reference
	assert(dictionary);                                 // __main__ should have a dictionary

	PyObject * result = PyDict_GetItemString(dictionary, variable);   // borrowed reference
	if (!result) 
		return false;

	if (PyLong_Check(result)) 
	{
		return_val = PyLong_AS_LONG(result); // cast py int to c++ double - it's ok anyway
		return true;
	}

	return false;
}
			
void ChPythonEngine::SetInteger(const char* variable, const int val)
{
	std::ostringstream sstream;
	sstream << variable << "=" << val <<"\n";

	this->Run(sstream.str().c_str());
}




} // END_OF_NAMESPACE____


////// end
