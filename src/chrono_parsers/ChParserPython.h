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
    /// Throws an exception in case of failure, so it recommended to enclose this in a try..catch block.
    void Run(const std::string& program);

    /// Set a value of a floating point variable. If a variable with the same
    /// name is existing, it is overwritten, otherwise it is created (in __main__ namespace)
    void SetFloat(const std::string& variable, const double val);

    /// Retrieve a value of an existing floating point variable.
    /// Returns false if unsuccessful.
    bool GetFloat(const std::string& variable, double& return_val);

    /// Set a value of a integer variable.
    /// If a variable with the same name is existing, it is overwritten, otherwise it is created (in __main__ namespace)
    void SetInteger(const std::string& variable, const int val);

    /// Retrieve a value of an existing integer variable.
    /// Returns false if unsuccessful.
    bool GetInteger(const std::string& variable, int& return_val);

    /// Set a value of a bool variable.
    /// If a variable with the same name is existing, it is overwritten, otherwise it is created (in __main__ namespace)
    void SetBool(const std::string& variable, const bool val);

    /// Retrieve a value of an existing bool variable.
    /// Returns false if unsuccessful.
    bool GetBool(const std::string& variable, bool& return_val);

    /// Set a value of a string variable.
    /// If a variable with the same name is existing, it is overwritten, otherwise it is created (in __main__ namespace)
    void SetString(const std::string& variable, std::string& val);

    /// Retrieve a value of an existing string variable.
    /// Returns false if unsuccessful.
    bool GetString(const std::string& variable, std::string& return_val);

    /// Set a numeric list variable from a given std::vector.
    /// If a variable with the same name is existing, it is overwritten, otherwise it is created (in __main__ namespace)
    void SetList(const std::string& variable, const std::vector<double>& val);

    /// Retrieve an existing numeric list variable and store it in given std::vector.
    /// Returns false if unsuccessful.
    bool GetList(const std::string& variable, std::vector<double>& return_val);

    /// Set a numeric list variable from a given ChVectorDynamic<>.
    /// If a variable with the same name is existing, it is overwritten, otherwise it is created (in __main__ namespace)
    void SetList(const std::string& variable, const ChVectorDynamic<double>& val);

    /// Retrieve an existing numeric list variable and store it in given ChVectorDynamic<>.
    /// Returns false if unsuccessful.
    bool GetList(const std::string& variable, ChVectorDynamic<double>& return_val);

    /// Set a numeric list variable from a given ChMatrixDynamic<>.
    /// If a variable with the same name is existing, it is overwritten, otherwise it is created (in __main__ namespace)
    void SetMatrix(const std::string& variable, const ChMatrixDynamic<double>& val);

    /// Retrieve an existing numeric list variable and store it in given ChMatrixDynamic<>.
    /// Returns false if unsuccessful.
    bool GetMatrix(const std::string& variable, ChMatrixDynamic<double>& return_val);

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
