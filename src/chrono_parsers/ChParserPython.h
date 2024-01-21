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

#ifndef CH_PARSER_PYTHON_H
#define CH_PARSER_PYTHON_H

#include "chrono_parsers/ChApiParsers.h"

#include "chrono/core/ChLog.h"
#include "chrono/physics/ChSystemNSC.h"

namespace chrono {
namespace parsers {

/// @addtogroup parsers_module
/// @{

/// Class for a Python parser.
/// This is an interpreter that can parse Python programs, from a single formula up to large programs.
class ChApiParsers ChPythonEngine {
  public:
    /// Create a Python parser, an interpreter that can parse Python programs.
    /// NOTE: currently only one instance at a time can be created.
    ChPythonEngine();

    ~ChPythonEngine();

    /// Execute a program.
    /// If fails, it throws an exception, so it is wise to put it inside try..catch blocks.
    void Run(const char* program);

    /// Retrieve a value of an existing floating point variable.
    /// Returns false if unsuccesfull.
    bool GetFloat(const char* variable, double& return_val);

    /// Set a value of a floating point variable. If a variable with the same
    /// name is existing, it is overwritten, otherwise it is created (in __main__ namespace)
    void SetFloat(const char* variable, const double val);

    /// Retrieve a value of an existing integer variable.
    /// Returns false if unsuccesfull.
    bool GetInteger(const char* variable, int& return_val);

    /// Set a value of a integer variable. If a variable with the same
    /// name is existing, it is overwritten, otherwise it is created (in __main__ namespace)
    void SetInteger(const char* variable, const int val);

    /// Retrieve a value of an existing bool variable.
    /// Returns false if unsuccesfull.
    bool GetBool(const char* variable, bool& return_val);

    /// Set a value of a bool variable. If a variable with the same
    /// name is existing, it is overwritten, otherwise it is created (in __main__ namespace)
    void SetBool(const char* variable, const bool val);

    /// Retrieve a value of an existing string variable.
    /// Returns false if unsuccesfull.
    bool GetString(const char* variable, std::string& return_val);

    /// Set a value of a string variable. If a variable with the same
    /// name is existing, it is overwritten, otherwise it is created (in __main__ namespace)
    void SetString(const char* variable, std::string& val);

    /// Set a value of a numeric list variable. If a variable with the same
    /// name is existing, it is overwritten, otherwise it is created (in __main__ namespace)
    void SetList(const char* variable, const std::vector<double>& val);

    /// Retrieve a value of an existing numeric list variable.
    /// Returns false if unsuccesfull.
    bool GetList(const char* variable, std::vector<double>& return_val);

    /// Load a .py file as it is saved by the SolidWorks add-in exporter.
    /// Such a .py file is a Python program that creates an equivalent mechanism in Chrono and contains a sequence of
    /// Chrono physics item object (bodies, links, etc.).  This function populates the specified Chrono system with all
    /// these Chrono objects.
    /// This function may throw on error, so it should be called in a try-catch block
    void ImportSolidWorksSystem(const std::string& solidworks_py_file, ChSystem& msystem);
};

/// @} parsers_module

}  // end namespace parsers
}  // namespace chrono

#endif
