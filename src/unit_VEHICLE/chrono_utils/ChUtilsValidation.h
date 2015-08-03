// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// A set of utility classes and functions for validation.
//
// =============================================================================

#ifndef CH_UTILS_VALIDATION_H
#define CH_UTILS_VALIDATION_H

#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <valarray>
#include <cmath>
#include <numeric>
#include <algorithm>

#include "utils/ChApiUtils.h"


namespace chrono {
namespace utils {

/// Norm types for validation
enum ChNormType {
  L2_NORM,
  RMS_NORM,
  INF_NORM
};

/// Vector of data file headers.
typedef std::vector<std::string> Headers;

/// Vector of data points.
typedef std::valarray<double>    DataVector;

/// Data table.
typedef std::vector<DataVector>  Data;

///
/// This class provides functionality for validation of simulation results.
/// It provides functions for processing either two data files (simulation and
/// reference) and calculating the norms of the column differences, or for
/// processing a single simulation data file and calculating the norms of its
/// columns.
/// In either case, it is assumed that the first column in a data file contains
/// time values.  This column is never processed.
///
class CH_UTILS_API ChValidation
{
public:

  ChValidation() {}
  ~ChValidation() {}

  /// Read the data from the specified files and process it.
  /// Excluding the first column (which must contain identical values in the two
  /// input files), we subtract the data in corresponding columns in the two
  /// files are and calculate the norms of the difference vectors.
  bool Process(
    const std::string& sim_filename,    ///< name of the file with simulation results
    const std::string& ref_filename,    ///< name of the file with reference data
    char               delim = '\t'     ///< delimiter (default TAB)
    );

  /// Read the data in the specified file and process it.
  /// We calculate the vector norms of all columns except the first one.
  bool Process(
    const std::string& sim_filename,    ///< name of the file with simulation results
    char               delim = '\t'     ///< delimiter (default TAB)
    );

  /// Return the number of data columns.
  size_t GetNumColumns() const { return m_num_cols; }
  /// Return the number of rows.
  size_t GetNumRows() const { return m_num_rows; }

  /// Return the headers in the simulation data file.
  const Headers& GetHeadersSimData() const { return m_sim_headers; }
  /// Return the simulation data.
  const Data& GetSimData() const { return m_sim_data; }

  /// Return the headers in the reference data file.
  const Headers& GetHeadersRefData() const { return m_ref_headers; }
  /// Return the reference data.
  const Data& GetRefData() const { return m_ref_data; }

  /// Return the L2 norm for the specified column.
  double GetL2norm(size_t col) const { return m_L2_norms[col]; }
  /// Return the RMS norm for the specified column.
  double GetRMSnorm(size_t col) const { return m_RMS_norms[col]; }
  /// Return the infinity norm for the specified column.
  double GetINFnorm(size_t col) const { return m_INF_norms[col]; }

  /// Return the L2 norms for all columns.
  const DataVector& GetL2norms() const { return m_L2_norms; }
  /// Return the RMS norm for all columns.
  const DataVector& GetRMSnorms() const { return m_RMS_norms; }
  /// Return the infinity norms for all columns.
  const DataVector& GetINFnorms() const { return m_INF_norms; }

  /// Read the specified data file.
  /// The file is assumed to be delimited by the specified character.
  /// The return value is the actual number of data points read from the file.
  static size_t ReadDataFile(
    const std::string& filename,        ///< [in] name of the data file
    char               delim,           ///< [in] delimiter
    Headers&           headers,         ///< [out] vector of column header strings
    Data&              data             ///< [out] table of data values
    );

private:

  double L2norm(const DataVector& v);
  double RMSnorm(const DataVector& v);
  double INFnorm(const DataVector& v);

  size_t m_num_cols;
  size_t m_num_rows;

  Headers m_sim_headers;
  Headers m_ref_headers;

  Data    m_sim_data;
  Data    m_ref_data;

  DataVector m_L2_norms;
  DataVector m_RMS_norms;
  DataVector m_INF_norms;
};

// -----------------------------------------------------------------------------
// Free function declarations
// -----------------------------------------------------------------------------

///
/// Compare the data in the two specified files.
/// The comparison is done using the specified norm type and tolerance. The
/// function returns true if the norms of all column differences are below the
/// given tolerance and false otherwise.
/// It is assumed that the input files are TAB-delimited.
///
CH_UTILS_API
bool Validate(
          const std::string& sim_filename,
          const std::string& ref_filename,
          ChNormType         norm_type,
          double             tolerance,
          DataVector&        norms
          );

///
/// Validation of a constraint violation data file.
/// The validation is done using the specified norm type and tolerance. The
/// function returns true if the norms of all columns, excluding the first one,
/// are below the given tolerance and false otherwise.
/// It is assumed that the input file is TAB-delimited.
///
CH_UTILS_API
bool Validate(
          const std::string& sim_filename,
          ChNormType         norm_type,
          double             tolerance,
          DataVector&        norms
          );

// -----------------------------------------------------------------------------
// Global functions for accessing the reference validation data.
// -----------------------------------------------------------------------------

/// Set the path to the reference validation data directory.
/// (ATTENTION: not thread safe)
CH_UTILS_API void SetValidationDataPath(const std::string& path);

/// Obtain the current path to the reference validation data directory.
/// (thread safe)
CH_UTILS_API const std::string& GetValidationDataPath();

/// Obtain the complete path to the specified filename.
/// The given filename is assumed to be relative to the reference validation
/// data directory.
/// (thread safe)
CH_UTILS_API std::string GetValidationDataFile(const std::string& filename);


} // namespace utils
} // namespace chrono


#endif
