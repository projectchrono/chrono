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

#include <iterator>

#include "chrono/utils/ChUtils.h"
#include "chrono/utils/ChValidation.h"

namespace chrono {
namespace utils {

// -----------------------------------------------------------------------------

bool ChValidation::Process(const std::string& sim_filename, const std::string& ref_filename) {
    // Read the simulation results file.
    m_sim_data = ReadDataFile(sim_filename, m_sim_headers);
    m_num_cols = m_sim_data.size();
    m_num_rows = m_sim_data[0].size();

    // Read the reference data file.
    m_ref_data = ReadDataFile(ref_filename, m_ref_headers);
    auto num_ref_cols = m_ref_data.size();
    auto num_ref_rows = (size_t)m_ref_data[0].size();

    // Resize the arrays of norms to zero length
    // (needed if we return with an error below)
    m_L2_norms.resize(0);
    m_RMS_norms.resize(0);
    m_INF_norms.resize(0);

    // Perform some sanity checks.
    if (m_num_cols != num_ref_cols) {
        std::cout << "ERROR: the number of columns in the two files is different:" << std::endl;
        std::cout << "   File " << sim_filename << " has " << m_num_cols << " columns" << std::endl;
        std::cout << "   File " << ref_filename << " has " << num_ref_cols << " columns" << std::endl;
        return false;
    }
    size_t n = m_num_rows;
    if (m_num_rows > num_ref_rows) {
        std::cout << "WARNING: simulation data has more time points than reference data:" << std::endl;
        std::cout << "   Simulation file " << sim_filename << " has " << m_num_rows << " data points" << std::endl;
        std::cout << "   Reference file " << ref_filename << " has " << num_ref_rows << " data points" << std::endl;
        std::cout << "   Processing up to t = " << m_ref_data[0].tail(1) << std::endl;
        n = num_ref_rows;
    }

    // Ensure that the first columns (time) are the same.
    if ((m_sim_data[0].head(n) - m_ref_data[0].head(n)).lpNorm<2>() > 1e-10) {
        std::cout << "ERROR: time sequences do not match." << std::endl;
        return false;
    }

    // Resize arrays of norms.
    m_L2_norms.resize(m_num_cols - 1);
    m_RMS_norms.resize(m_num_cols - 1);
    m_INF_norms.resize(m_num_cols - 1);

    // Calculate norms of the differences.
    for (size_t col = 0; col < m_num_cols - 1; col++) {
        m_L2_norms[col] = (m_sim_data[col + 1].head(n) - m_ref_data[col + 1].head(n)).lpNorm<2>();
        m_RMS_norms[col] = (m_sim_data[col + 1].head(n) - m_ref_data[col + 1].head(n)).rmsNorm();
        m_INF_norms[col] = (m_sim_data[col + 1].head(n) - m_ref_data[col + 1].head(n)).lpNorm<Eigen::Infinity>();
    }

    return true;
}

bool ChValidation::Process(const Data& sim_data, const Data& ref_data) {
    // Resize the arrays of norms to zero length
    // (needed if we return with an error below)
    m_L2_norms.resize(0);
    m_RMS_norms.resize(0);
    m_INF_norms.resize(0);

    // Simulation data
    m_num_cols = sim_data.size();
    m_num_rows = (size_t)sim_data[0].size();
    if (m_num_cols < 1) {
        std::cout << "ERROR: no values in simulation data structure." << std::endl;
        return false;
    }

    // Reference data
    auto num_ref_cols = ref_data.size();
    auto num_ref_rows = (size_t)ref_data[0].size();

    // Perform some sanity checks.
    if (m_num_cols != num_ref_cols) {
        std::cout << "ERROR: the number of columns in the two structures is different:" << std::endl;
        std::cout << "   Simulation data has " << m_num_cols << " columns" << std::endl;
        std::cout << "   Reference data has " << ref_data.size() << " columns" << std::endl;
        return false;
    }
    size_t n = m_num_rows;
    if (m_num_rows > num_ref_rows) {
        std::cout << "WARNING: simulation data has more time points than reference data:" << std::endl;
        std::cout << "   Simulation data has " << m_num_rows << " data points" << std::endl;
        std::cout << "   Reference data has " << ref_data[0].size() << " data points" << std::endl;
        std::cout << "   Processing up to t = " << ref_data[0].tail(1) << std::endl;
        n = num_ref_rows;
    }

    // Cache simulation and reference data. Set headers to empty values.
    m_sim_data.resize(m_num_cols);
    m_ref_data.resize(m_num_cols);
    m_sim_headers.resize(m_num_cols);
    m_ref_headers.resize(m_num_cols);
    for (size_t col = 0; col < m_num_cols; col++) {
        m_sim_data[col] = sim_data[col];
        m_ref_data[col] = ref_data[col];
        m_ref_headers[col] = "";
    }

    // Ensure that the first columns (time) are the same.
    if ((m_sim_data[0].head(n) - m_ref_data[0].head(n)).lpNorm<2>() > 1e-10) {
        std::cout << "ERROR: time sequences do not match." << std::endl;
        return false;
    }

    // Resize arrays of norms.
    m_L2_norms.resize(m_num_cols - 1);
    m_RMS_norms.resize(m_num_cols - 1);
    m_INF_norms.resize(m_num_cols - 1);

    // Calculate norms of the differences.
    for (size_t col = 0; col < m_num_cols - 1; col++) {
        m_L2_norms[col] = (m_sim_data[col + 1].head(n) - m_ref_data[col + 1].head(n)).lpNorm<2>();
        m_RMS_norms[col] = (m_sim_data[col + 1].head(n) - m_ref_data[col + 1].head(n)).rmsNorm();
        m_INF_norms[col] = (m_sim_data[col + 1].head(n) - m_ref_data[col + 1].head(n)).lpNorm<Eigen::Infinity>();
    }

    return true;
}

bool ChValidation::Process(const std::string& sim_filename) {
    // Read the simulation results file.
    m_sim_data = ReadDataFile(sim_filename, m_sim_headers);
    m_num_cols = m_sim_data.size();
    m_num_rows = m_sim_data[0].size();

    // Resize arrays of norms.
    m_L2_norms.resize(m_num_cols - 1);
    m_RMS_norms.resize(m_num_cols - 1);
    m_INF_norms.resize(m_num_cols - 1);

    // Calculate norms of the column vectors.
    for (size_t col = 0; col < m_num_cols - 1; col++) {
        m_L2_norms[col] = (m_sim_data[col + 1]).lpNorm<2>();
        m_RMS_norms[col] = (m_sim_data[col + 1]).rmsNorm();
        m_INF_norms[col] = (m_sim_data[col + 1]).lpNorm<Eigen::Infinity>();
    }

    return true;
}

bool ChValidation::Process(const Data& sim_data) {
    // Resize the arrays of norms to zero length
    // (needed if we return with an error below)
    m_L2_norms.resize(0);
    m_RMS_norms.resize(0);
    m_INF_norms.resize(0);

    // Read the simulation results file.
    m_num_cols = sim_data.size();
    m_num_rows = sim_data[0].size();
    if (m_num_cols < 1) {
        std::cout << "ERROR: no values in simulation data structure." << std::endl;
        return false;
    }

    // Cache simulation data. Set headers to empty values.
    m_sim_data.resize(m_num_cols);
    m_sim_headers.resize(m_num_cols);
    for (size_t col = 0; col < m_num_cols; col++) {
        m_sim_data[col] = sim_data[col];
        m_sim_headers[col] = "";
    }

    // Resize arrays of norms.
    m_L2_norms.resize(m_num_cols - 1);
    m_RMS_norms.resize(m_num_cols - 1);
    m_INF_norms.resize(m_num_cols - 1);

    // Calculate norms of the column vectors.
    for (size_t col = 0; col < m_num_cols - 1; col++) {
        m_L2_norms[col] = (m_sim_data[col + 1]).lpNorm<2>();
        m_RMS_norms[col] = (m_sim_data[col + 1]).rmsNorm();
        m_INF_norms[col] = (m_sim_data[col + 1]).lpNorm<Eigen::Infinity>();
    }

    return true;
}

// -----------------------------------------------------------------------------

ChValidation::Data ChValidation::CreateData(const std::vector<std::vector<double>>& columns) {
    if (columns.empty())
        return Data();

    auto num_cols = columns.size();
    auto num_rows = columns[0].size();
    Data data(num_cols);
    for (size_t col = 0; col < num_cols; col++) {
        std::vector<double>& tmp = const_cast<std::vector<double>&>(columns[col]);
        data[col] = Eigen::Map<ChVectorDynamic<>>(tmp.data(), num_rows);
    }

    return data;
}

// -----------------------------------------------------------------------------

ChValidation::Data ChValidation::ReadDataFile(const std::string& filename, Headers& headers) {
    std::ifstream ifile(filename);
    std::string line;

    // Count the number of lines in the file then rewind the input file stream
    size_t num_lines = std::count(std::istreambuf_iterator<char>(ifile), std::istreambuf_iterator<char>(), '\n');
    ifile.seekg(0, ifile.beg);

    // Skip top comment lines (lines starting with '#')
    size_t num_comments = 0;
    while (true) {
        std::getline(ifile, line);
        if (line[0] == '#')
            num_comments++;
        else
            break;
    }

    // Read the column headers (assumed space-separated)
    {
        std::stringstream iss(line);
        std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(),
                  std::back_inserter(headers));
    }

    // Create and load data
    size_t num_data_points = num_lines - num_comments - 1;
    size_t num_cols = headers.size();
    Data data(num_cols);
    for (size_t col = 0; col < num_cols; col++)
        data[col].resize(num_data_points);

    // Read the actual data, one line at a time.
    size_t row = 0;
    while (std::getline(ifile, line)) {
        std::stringstream iss(line);
        for (size_t col = 0; col < num_cols; col++)
            iss >> data[col][row];
        row++;
    }
    ChAssertAlways(row == num_data_points);

    return data;
}

// -----------------------------------------------------------------------------
// Compare the data in the two specified files.
// The comparison is done using the specified norm type and tolerance. The
// function returns true if the norms of all column differences are below the
// given tolerance and false otherwise.
// It is assumed that the input files are TAB-delimited.
// -----------------------------------------------------------------------------
bool ChValidation::Test(const std::string& sim_filename,
                        const std::string& ref_filename,
                        NormType norm_type,
                        double tolerance,
                        DataVector& norms) {
    ChValidation validator;

    if (!validator.Process(sim_filename, ref_filename))
        return false;

    size_t num_cols = validator.GetNumColumns() - 1;
    norms.resize(num_cols);

    switch (norm_type) {
        case NormType::L2:
            norms = validator.GetL2norms();
            break;
        case NormType::RMS:
            norms = validator.GetRMSnorms();
            break;
        case NormType::INF:
            norms = validator.GetINFnorms();
            break;
    }

    for (size_t col = 0; col < num_cols; col++) {
        if (norms[col] > tolerance)
            return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
// Compare the data in the two specified structures
// The comparison is done using the specified norm type and tolerance. The
// function returns true if the norms of all column differences are below the
// given tolerance and false otherwise.
// -----------------------------------------------------------------------------
bool ChValidation::Test(const Data& sim_data,
                        const Data& ref_data,
                        NormType norm_type,
                        double tolerance,
                        DataVector& norms) {
    ChValidation validator;

    if (!validator.Process(sim_data, ref_data))
        return false;

    size_t num_cols = validator.GetNumColumns() - 1;
    norms.resize(num_cols);

    switch (norm_type) {
        case NormType::L2:
            norms = validator.GetL2norms();
            break;
        case NormType::RMS:
            norms = validator.GetRMSnorms();
            break;
        case NormType::INF:
            norms = validator.GetINFnorms();
            break;
    }

    for (size_t col = 0; col < num_cols; col++) {
        if (norms[col] > tolerance)
            return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
// Validation of a constraint violation data file.
// The validation is done using the specified norm type and tolerance. The
// function returns true if the norms of all columns, excluding the first one,
// are below the given tolerance and false otherwise.
// It is assumed that the input file is TAB-delimited.
// -----------------------------------------------------------------------------
bool ChValidation::Test(const std::string& sim_filename, NormType norm_type, double tolerance, DataVector& norms) {
    ChValidation validator;

    if (!validator.Process(sim_filename))
        return false;

    size_t num_cols = validator.GetNumColumns() - 1;
    norms.resize(num_cols);

    switch (norm_type) {
        case NormType::L2:
            norms = validator.GetL2norms();
            break;
        case NormType::RMS:
            norms = validator.GetRMSnorms();
            break;
        case NormType::INF:
            norms = validator.GetINFnorms();
            break;
    }

    for (size_t col = 0; col < num_cols; col++) {
        if (norms[col] > tolerance)
            return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
// Validation of a constraint violation data structure.
// The validation is done using the specified norm type and tolerance. The
// function returns true if the norms of all columns, excluding the first one,
// are below the given tolerance and false otherwise.
// -----------------------------------------------------------------------------
bool ChValidation::Test(const Data& sim_data, NormType norm_type, double tolerance, DataVector& norms) {
    ChValidation validator;

    if (!validator.Process(sim_data))
        return false;

    size_t num_cols = validator.GetNumColumns() - 1;
    norms.resize(num_cols);

    switch (norm_type) {
        case NormType::L2:
            norms = validator.GetL2norms();
            break;
        case NormType::RMS:
            norms = validator.GetRMSnorms();
            break;
        case NormType::INF:
            norms = validator.GetINFnorms();
            break;
    }

    for (size_t col = 0; col < num_cols; col++) {
        if (norms[col] > tolerance)
            return false;
    }

    return true;
}

std::string ChValidation::GetNormTypeAsString(NormType type) {
    switch (type) {
        case NormType::L2:
            return "L2";
        case NormType::RMS: 
          return "RMS";
        case NormType::INF:
            return "L-infinity";
    }
    return "unknown";
}

// -----------------------------------------------------------------------------
// Functions for manipulating the validation data directory
// -----------------------------------------------------------------------------

static std::string validation_data_path("../data/");

// Set the path to the directory containing reference validation data.
void SetValidationDataPath(const std::string& path) {
    validation_data_path = path;
}

// Obtain the current path to the directory containing reference validation data.
const std::string& GetValidationDataPath() {
    return validation_data_path;
}

// Obtain the complete path to the specified filename, given relative to the
// directory containing reference validation data.
std::string GetValidationDataFile(const std::string& filename) {
    return validation_data_path + filename;
}

}  // namespace utils
}  // namespace chrono