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

#include <Python.h>
#include <iostream>
#include <sstream>
#include <typeinfo>

#include "chrono_parsers/ChSwigutils.h"
#include "chrono_parsers/ChParserPython.h"

namespace chrono {
namespace parsers {

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
        throw std::runtime_error(smessageError.str());
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

void ChPythonEngine::SetList(const char* variable, const std::vector<double>& val) {
    std::ostringstream sstream;
    sstream << variable << "="
            << "[";
    for (int i = 0; i < val.size(); ++i)
        sstream << (double)val[i] << ",";
    sstream << "]"
            << "\n";
    this->Run(sstream.str().c_str());
}

bool ChPythonEngine::GetList(const char* variable, std::vector<double>& return_val) {
    PyObject* module = PyImport_AddModule("__main__");  // borrowed reference
    assert(module);                                     // __main__ should always exist

    PyObject* dictionary = PyModule_GetDict(module);  // borrowed reference
    assert(dictionary);                               // __main__ should have a dictionary

    PyObject* result = PyDict_GetItemString(dictionary, variable);  // borrowed reference
    if (!result)
        return false;

    if (PyList_Check(result)) {
        int length = (int)PyObject_Length(result);
        return_val.clear();  // initially clear for safety
        for (int i = 0; i < length; ++i) {
            auto item = PyList_GetItem(result, i);
            if (PyLong_Check(item)) {
                double tmp = PyLong_AsDouble(
                    item);  // whether list item is a py integer or a number like 3.0, interpret it as a double
                return_val.push_back(tmp);
            } else if (PyFloat_Check(item)) {
                double tmp = PyFloat_AS_DOUBLE(item);
                return_val.push_back(tmp);
            }
        }
        return true;
    }

    return false;
}

void ChPythonEngine::ImportSolidWorksSystem(const std::string& solidworks_py_file, ChSystem& msystem) {
    std::ostringstream sstream;

    sstream << "import builtins\n";
    sstream << "import sys\n";
    sstream << "import os\n";
    sstream << "mpath = \"" << solidworks_py_file << "\"\n";
    sstream << "mdirname, mmodulename = os.path.split(mpath)\n";
    sstream << "builtins.exported_system_relpath = mdirname + '/'\n";
    sstream << "try:\n";
    sstream << "    if sys.version_info[0] == 3 and sys.version_info[1] >= 5:\n";
    sstream << "        import importlib.util\n";
    sstream << "        spec = importlib.util.spec_from_file_location(mmodulename, mpath)\n";
    sstream << "        imported_mod = importlib.util.module_from_spec(spec)\n";
    sstream << "        sys.modules[mmodulename] = imported_mod\n";
    sstream << "        spec.loader.exec_module(imported_mod)\n";
    sstream << "    elif sys.version_info[0] == 3 and sys.version_info[1] < 5:\n";
    sstream << "        import importlib.machinery\n";
    sstream << "        loader = importlib.machinery.SourceFileLoader(mmodulename, mpath)\n";
    sstream << "        imported_mod = loader.load_module()\n";
    sstream << "    else:\n";
    sstream << "        raise Exception(\"Python version not supported. Please upgrade it.\")\n";
    sstream << "except Exception as e:\n";
    sstream << "    print(f\"Error loading module: {e}\")\n";
    sstream << "exported_items = imported_mod.exported_items\n";
    this->Run(sstream.str().c_str());

    PyObject* module = PyImport_AddModule("__main__");  // borrowed reference
    if (!module)
        throw std::runtime_error("ERROR: no Python __main__ module?");

    PyObject* dictionary = PyModule_GetDict(module);  // borrowed reference
    if (!dictionary)
        throw std::runtime_error("ERROR: no Python dictionary?");

    PyObject* result = PyDict_GetItemString(dictionary, "exported_items");  // borrowed reference
    if (!result)
        throw std::runtime_error("ERROR: missing Python object 'exported_items' in SolidWorks file");

    if (PyList_Check(result)) {
        int nitems = (int)PyList_Size(result);

        for (int i = 0; i < nitems; i++) {
            PyObject* mobj = PyList_GetItem(result, i);
            if (mobj) {
                SwigPyObject* mswigobj = SWIG_Python_GetSwigThis(mobj);
                if (mswigobj) {
                    void* objptr = mswigobj->ptr;
                    std::shared_ptr<ChPhysicsItem>* pt_to_shp = (std::shared_ptr<ChPhysicsItem>*)objptr;

                    /// Add the ChPhysicsItem to the ChSystem
                    msystem.Add((*pt_to_shp));
                } else {
                    throw std::runtime_error(
                        "ERROR: only shared pointers to ChPhysicsItem subclasses can be inside exported_items.");
                }
            }
        }

        msystem.Setup();
        msystem.Update(0.0, true);

    } else {
        throw std::runtime_error("ERROR: exported_items python object is not a list.");
    }
}

}  // end namespace parsers
}  // namespace chrono
