// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#include "chrono_pyparser/ChSwigutils.h"

#include "chrono_pyparser/ChPython.h"
#include <Python.h>
#include <iostream>
#include <sstream>
#include <typeinfo>

namespace chrono {

ChPythonEngine::ChPythonEngine() {
    Py_Initialize();
}

ChPythonEngine::~ChPythonEngine() {
    Py_Finalize();
}

void ChPythonEngine::Run(const char* program) {
    // Redirect stderr to one std::string buffer.

    std::streambuf *msgErrBuffer, *msgErrBufferBackup;
    std::stringstream smessageError;
    msgErrBufferBackup = std::cerr.rdbuf();
    msgErrBuffer = smessageError.rdbuf();
    std::cerr.rdbuf(msgErrBuffer);

    if (PyRun_SimpleString(program) != 0) {
        PyErr_Print();
        throw ChException(smessageError.str());
    }

    // Set to previous channels!

    // std::cout.rdbuf(msgOutBufferBackup);

    std::cerr.rdbuf(msgErrBufferBackup);
}

bool ChPythonEngine::GetFloat(const char* variable, double& return_val) {
    PyObject* module = PyImport_AddModule("__main__");  // borrowed reference
    assert(module);                                     // __main__ should always exist

    PyObject* dictionary = PyModule_GetDict(module);  // borrowed reference
    assert(dictionary);                               // __main__ should have a dictionary

    PyObject* result = PyDict_GetItemString(dictionary, variable);  // borrowed reference
    if (!result)
        return false;

    if (PyFloat_Check(result)) {
        return_val = PyFloat_AS_DOUBLE(result);
        return true;
    }
    if (PyLong_Check(result)) {
        return_val = (double)PyLong_AsLong(result);  // cast py int to c++ double - it's ok anyway
        return true;
    }

    return false;
}

void ChPythonEngine::SetFloat(const char* variable, const double val) {
    std::ostringstream sstream;
    sstream << variable << "=" << val << "\n";

    this->Run(sstream.str().c_str());
}

bool ChPythonEngine::GetInteger(const char* variable, int& return_val) {
    PyObject* module = PyImport_AddModule("__main__");  // borrowed reference
    assert(module);                                     // __main__ should always exist

    PyObject* dictionary = PyModule_GetDict(module);  // borrowed reference
    assert(dictionary);                               // __main__ should have a dictionary

    PyObject* result = PyDict_GetItemString(dictionary, variable);  // borrowed reference
    if (!result)
        return false;

    if (PyLong_Check(result)) {
        return_val = PyLong_AsLong(result);
        return true;
    }

    return false;
}

void ChPythonEngine::SetInteger(const char* variable, const int val) {
    std::ostringstream sstream;
    sstream << variable << "=" << val << "\n";

    this->Run(sstream.str().c_str());
}

bool ChPythonEngine::GetBool(const char* variable, bool& return_val) {
    PyObject* module = PyImport_AddModule("__main__");  // borrowed reference
    assert(module);                                     // __main__ should always exist

    PyObject* dictionary = PyModule_GetDict(module);  // borrowed reference
    assert(dictionary);                               // __main__ should have a dictionary

    PyObject* result = PyDict_GetItemString(dictionary, variable);  // borrowed reference
    if (!result)
        return false;

    if (PyBool_Check(result)) {
        return_val = (PyLong_AsLong(result) == 0) ? false : true;
        return true;
    }

    return false;
}

void ChPythonEngine::SetBool(const char* variable, const bool val) {
    std::ostringstream sstream;
    sstream << variable << "=" << val << "\n";

    this->Run(sstream.str().c_str());
}

bool ChPythonEngine::GetString(const char* variable, std::string& return_val) {
    PyObject* module = PyImport_AddModule("__main__");  // borrowed reference
    assert(module);                                     // __main__ should always exist

    PyObject* dictionary = PyModule_GetDict(module);  // borrowed reference
    assert(dictionary);                               // __main__ should have a dictionary

    PyObject* result = PyDict_GetItemString(dictionary, variable);  // borrowed reference
    if (!result)
        return false;

    if (PyBytes_Check(result)) {
        char* ret_string = PyBytes_AsString(result);
        return_val = ret_string;
        return true;
    }
    if (PyUnicode_Check(result)) {
        PyObject* mascii = PyUnicode_AsASCIIString(result);

        char* ret_string = PyBytes_AsString(mascii);
        return_val = ret_string;

        Py_DECREF(mascii);
        return true;
    }

    return false;
}

void ChPythonEngine::SetString(const char* variable, std::string& val) {
    std::ostringstream sstream;
    sstream << variable << "='" << val << "'\n";

    this->Run(sstream.str().c_str());
}

/*
typedef struct {
  PyObject ob_base;
  void *ptr;
  void *ty;
  int own;
  PyObject *next;

} SwigPyObject;
*/

void ChPythonEngine::ImportSolidWorksSystem(const char* solidworks_py_file, ChSystem& msystem) {
    std::ostringstream sstream;

    // sstream << "from " << std::string(solidworks_py_file) << " import exported_items\n";

    sstream << "import builtins  \n";
    sstream << "import imp  \n";
    sstream << "import os  \n";
    sstream << "mdirname, mmodulename= os.path.split('" << std::string(solidworks_py_file) << "')  \n";
    sstream << "builtins.exported_system_relpath = mdirname + '/'  \n";
    sstream << "fp, pathname, description = imp.find_module(mmodulename,[builtins.exported_system_relpath])  \n";
    sstream << "try:  \n";
    sstream << "    imported_mod = imp.load_module('imported_mod', fp, pathname, description)  \n";
    sstream << "finally:  \n";
    sstream << "    if fp:  \n";
    sstream << "        fp.close()  \n";
    sstream << "exported_items = imported_mod.exported_items  \n";

    this->Run(sstream.str().c_str());

    PyObject* module = PyImport_AddModule("__main__");  // borrowed reference
    if (!module)
        throw ChException("ERROR. No Python __main__ module?");

    PyObject* dictionary = PyModule_GetDict(module);  // borrowed reference
    if (!dictionary)
        throw ChException("ERROR. No Python dictionary?");

    PyObject* result = PyDict_GetItemString(dictionary, "exported_items");  // borrowed reference
    if (!result)
        throw ChException("ERROR. Missing Python object 'exported_items' in SolidWorks file");

    if (PyList_Check(result)) {
        int nitems = (int)PyList_Size(result);
        // GetLog() << "N.of list items: " << nitems << "\n";
        for (int i = 0; i < nitems; i++) {
            PyObject* mobj = PyList_GetItem(result, i);
            if (mobj) {
                // GetLog() << "   Python type: " << mobj->ob_type->tp_name << "\n";

                SwigPyObject* mswigobj = SWIG_Python_GetSwigThis(mobj);
                if (mswigobj) {
                    void* objptr = mswigobj->ptr;
                    std::shared_ptr<ChPhysicsItem>* pt_to_shp = (std::shared_ptr<ChPhysicsItem>*)objptr;

                    /// Add the ChPhysicsItem to the ChSystem
                    msystem.Add((*pt_to_shp));
                } else {
                    throw ChException(
                        "ERROR. Only shared pointers to ChPhysicsItem subclasses can be inside exported_items.");
                }
            }
        }

        msystem.Setup();
        msystem.Update();

    } else {
        throw ChException("ERROR. exported_items python object is not a list.");
    }
}

}  // namespace chrono

