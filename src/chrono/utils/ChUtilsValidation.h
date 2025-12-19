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
#include <cmath>
#include <numeric>
#include <algorithm>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMatrix.h"

namespace chrono {
namespace utils {

/// This class provides functionality for validation of simulation results.
/// It provides functions for processing either two data files (simulation and reference) and calculating the norms of
/// the column differences, or for processing a single simulation data file and calculating the norms of its columns. In
/// either case, it is assumed that the first column in a data file contains time values; this column is never
/// processed.
class ChApi ChValidation {
  public:
    /// Norm types for validation.
    enum class NormType { L2, RMS, INF };

    /// Vector of data file headers.
    typedef std::vector<std::string> Headers;

    /// Vector of data points.
    typedef ChVectorDynamic<> DataVector;

    /// Data table.
    typedef std::vector<DataVector> Data;

    ChValidation() {}
    ~ChValidation() {}

    /// Read the data from the specified files and process it.
    /// Ignoring the first column (which must contain identical values in the two input files), subtract the data in
    /// corresponding columns in the two files are and calculate the norms of the difference vectors.
    bool Process(const std::string& sim_filename,  ///< name of the file with simulation results
                 const std::string& ref_filename   ///< name of the file with reference data
    );

    /// Process the data in the two specified structures.
    /// Ignoring the first column (which must contain identical values in the two input structures), subtract the data
    /// in corresponding columns in the two structures are and calculate the norms of the difference vectors.
    bool Process(const Data& sim_data,  ///< simulation data strcture
                 const Data& ref_data   ///< reference data structure
    );

    /// Read the data in the specified file and process it.
    /// Calculate the vector norms of all columns except the first one.
    bool Process(const std::string& sim_filename);

    /// Process the data in the specified structure.
    /// Calculate the vector norms of all columns except the first one.
    bool Process(const Data& sim_data  ///< simulation data structure
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

    // Utility wrapper functions

    /// Read the specified data file and return a Data object.
    /// The file is assumed to be space-delimited.
    static Data ReadDataFile(const std::string& filename,  ///< [in] name of the data file
                             Headers& headers              ///< [out] vector of column header strings
    );

    /// Compare the data in the two specified files.
    /// The comparison is done using the specified norm type and tolerance. The function returns true if the norms of
    /// all column differences are below the given tolerance and false otherwise. It is assumed that the input files are
    /// space-delimited.
    static bool Test(const std::string& sim_filename,
                     const std::string& ref_filename,
                     NormType norm_type,
                     double tolerance,
                     DataVector& norms);

    /// Compare the data in the two specified structures
    /// The comparison is done using the specified norm type and tolerance. The function returns true if the norms of
    /// all column differences are below the given tolerance and false otherwise.
    static bool Test(const Data& sim_data,
                     const Data& ref_data,
                     NormType norm_type,
                     double tolerance,
                     DataVector& norms);

    /// Validation of a constraint violation data file.
    /// The validation is done using the specified norm type and tolerance. The function returns true if the norms of
    /// all columns, excluding the first one, are below the given tolerance and false otherwise. It is assumed that the
    /// input file is space-delimited.
    static bool Test(const std::string& sim_filename, NormType norm_type, double tolerance, DataVector& norms);

    /// Validation of a constraint violation data structure.
    /// The validation is done using the specified norm type and tolerance. The function returns true if the norms of
    /// all columns, excluding the first one, are below the given tolerance and false otherwise.
    static bool Test(const Data& sim_data, NormType norm_type, double tolerance, DataVector& norms);

  private:
    size_t m_num_cols;
    size_t m_num_rows;

    Headers m_sim_headers;
    Headers m_ref_headers;

    Data m_sim_data;
    Data m_ref_data;

    DataVector m_L2_norms;
    DataVector m_RMS_norms;
    DataVector m_INF_norms;
};

// -----------------------------------------------------------------------------
// Global functions for accessing the reference validation data.
// -----------------------------------------------------------------------------

/// Set the path to the reference validation data directory.
/// (ATTENTION: not thread safe)
ChApi void SetValidationDataPath(const std::string& path);

/// Obtain the current path to the reference validation data directory.
/// (thread safe)
ChApi const std::string& GetValidationDataPath();

/// Obtain the complete path to the specified filename.
/// The given filename is assumed to be relative to the reference validation data directory.
/// (thread safe)
ChApi std::string GetValidationDataFile(const std::string& filename);

}  // namespace utils
}  // namespace chrono

#endif
