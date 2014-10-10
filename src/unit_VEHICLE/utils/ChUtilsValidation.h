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
#include <functional>

#include "utils/ChApiUtils.h"


namespace chrono {
namespace utils {

/// Vector of data file headers.
typedef std::vector<std::string> Headers;

/// Vector of data points.
typedef std::valarray<double>    DataVector;

/// Data table.
typedef std::vector<DataVector>  Data;

///
/// This class provides functionality for comparing two time series.
///
  class CH_UTILS_API ChValidation {
  public:
    ChValidation() {}
    ~ChValidation() {}

    /// Read the data from the specified files and process it.
    bool Process(
      const std::string& sim_filename,    ///< name of the file with simulation results
      const std::string& ref_filename,    ///< name of the file with reference data
      size_t             num_data_points, ///< number of data points
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

    /// Return the difference L2 norm for the specified column.
    double GetDiffL2norm(size_t col) const { return m_L2_norms[col]; }
    /// Return the difference RMS norm for the specified column.
    double GetDiffRMSnorm(size_t col) const { return m_RMS_norms[col]; }
    /// Return the difference infinity norm for the specified column.
    double GetDiffINFnorm(size_t col) const { return m_INF_norms[col]; }

    /// Return the difference L2 norms for all columns.
    const DataVector& GetDiffL2norms() const { return m_L2_norms; }
    /// Return the difference RMS norm for all columns.
    const DataVector& GetDiffRMSnorms() const { return m_RMS_norms; }
    /// Return the difference infinity norms for all columns.
    const DataVector& GetDiffINFnorms() const { return m_INF_norms; }

    /// Return all metrics for the specified column.
    void GetDiffMetrics(
      size_t  col,        ///< data column number
      double& L2_norm,    ///< difference L2 norm
      double& RMS_norm,   ///< difference RMS norm
      double& Inf_norm    ///< difference infinity norm
      ) const;

    /// Read the specified data file.
    /// The file is assumed to be delimited by the specified character.
    /// The return value is the actual number of data points read from the file.
    static size_t ReadDataFile(
      const std::string& filename,        ///< [in] name of the data file
      char               delim,           ///< [in] delimiter
      size_t             num_data_points, ///< [in] number of data points
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




} // namespace utils
} // namespace chrono


#endif
