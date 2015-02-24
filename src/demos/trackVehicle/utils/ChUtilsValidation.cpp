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

#include "ChUtilsValidation.h"

namespace chrono {
namespace utils {



// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
bool ChValidation::Process(const std::string& sim_filename,
                           const std::string& ref_filename,
                           char               delim)
{
  // Read the simulation results file.
  m_num_rows = ReadDataFile(sim_filename, delim, m_sim_headers, m_sim_data);
  m_num_cols = m_sim_headers.size();

  // Read the reference data file.
  size_t num_ref_rows = ReadDataFile(ref_filename, delim, m_ref_headers, m_ref_data);

  // Resize the arrays of norms to zero length
  // (needed if we return with an error below)
  m_L2_norms.resize(0);
  m_RMS_norms.resize(0);
  m_INF_norms.resize(0);

  // Perform some sanity checks.
  if (m_num_cols != m_ref_headers.size()) {
    std::cout << "ERROR: the number of columns in the two files is different:" << std::endl;
    std::cout << "   File " << sim_filename << " has " << m_num_cols << " columns" << std::endl;
    std::cout << "   File " << ref_filename << " has " << m_ref_headers.size() << " columns" << std::endl;
    return false;
  }

  if (m_num_rows != num_ref_rows) {
    std::cout << "ERROR: the number of rows in the two files is different:" << std::endl;
    std::cout << "   File " << sim_filename << " has " << m_num_rows << " columns" << std::endl;
    std::cout << "   File " << ref_filename << " has " << num_ref_rows << " columns" << std::endl;
    return false;
  }

  // Ensure that the first columns (time) are the same.
  if (L2norm(m_sim_data[0] - m_ref_data[0]) > 1e-10) {
    std::cout << "ERROR: time sequences do not match." << std::endl;
    return false;
  }

  // Resize arrays of norms.
  m_L2_norms.resize(m_num_cols - 1);
  m_RMS_norms.resize(m_num_cols - 1);
  m_INF_norms.resize(m_num_cols - 1);

  // Calculate norms of the differences.
  for (size_t col = 0; col < m_num_cols - 1; col++) {
    m_L2_norms[col] = L2norm(m_sim_data[col + 1] - m_ref_data[col + 1]);
    m_RMS_norms[col] = RMSnorm(m_sim_data[col + 1] - m_ref_data[col + 1]);
    m_INF_norms[col] = INFnorm(m_sim_data[col + 1] - m_ref_data[col + 1]);
  }

  return true;
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
bool ChValidation::Process(const std::string& sim_filename,
                           char               delim)
{
  // Read the simulation results file.
  m_num_rows = ReadDataFile(sim_filename, delim, m_sim_headers, m_sim_data);
  m_num_cols = m_sim_headers.size();

  // Resize arrays of norms.
  m_L2_norms.resize(m_num_cols - 1);
  m_RMS_norms.resize(m_num_cols - 1);
  m_INF_norms.resize(m_num_cols - 1);

  // Calculate norms of the column vectors.
  for (size_t col = 0; col < m_num_cols - 1; col++) {
    m_L2_norms[col] = L2norm(m_sim_data[col + 1]);
    m_RMS_norms[col] = RMSnorm(m_sim_data[col + 1]);
    m_INF_norms[col] = INFnorm(m_sim_data[col + 1]);
  }

  return true;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChValidation::L2norm(const DataVector& v)
{
  return std::sqrt((v * v).sum());
}

double ChValidation::RMSnorm(const DataVector& v)
{
  return std::sqrt((v * v).sum() / v.size());
}

double ChValidation::INFnorm(const DataVector& v)
{
   return std::abs(v).max();
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
size_t ChValidation::ReadDataFile(const std::string& filename,
                                  char               delim,
                                  Headers&           headers,
                                  Data&              data)
{
  std::ifstream ifile(filename.c_str());
  std::string   line;

  // Count the number of lines in the file then rewind the input file stream.
  size_t num_lines = std::count(std::istreambuf_iterator<char>(ifile),
                                std::istreambuf_iterator<char>(), '\n');
  ifile.seekg(0, ifile.beg);

  size_t num_data_points = num_lines - 3;

  // Skip the first two lines.
  std::getline(ifile, line);
  std::getline(ifile, line);

  // Read the line with column headers.
  std::getline(ifile, line);
  std::stringstream iss1(line);
  std::string col_header = "";

  while (std::getline(iss1, col_header, delim))
    headers.push_back(col_header);

  size_t num_cols = headers.size();

  // Resize data
  data.resize(num_cols);
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

  return row;
}


// -----------------------------------------------------------------------------
// Compare the data in the two specified files.
// The comparison is done using the specified norm type and tolerance. The
// function returns true if the norms of all column differences are below the
// given tolerance and false otherwise.
// It is assumed that the input files are TAB-delimited.
// -----------------------------------------------------------------------------
bool Validate(const std::string& sim_filename,
              const std::string& ref_filename,
              ChNormType         norm_type,
              double             tolerance,
              DataVector&        norms
              )
{
  ChValidation validator;

  if (!validator.Process(sim_filename, ref_filename))
    return false;

  size_t num_cols = validator.GetNumColumns() - 1;
  norms.resize(num_cols);

  switch (norm_type) {
  case L2_NORM:  norms = validator.GetL2norms(); break;
  case RMS_NORM: norms = validator.GetRMSnorms(); break;
  case INF_NORM: norms = validator.GetINFnorms(); break;
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
bool Validate(const std::string& sim_filename,
              ChNormType         norm_type,
              double             tolerance,
              DataVector&        norms)
{
  ChValidation validator;

  if (!validator.Process(sim_filename))
    return false;

  size_t num_cols = validator.GetNumColumns() - 1;
  norms.resize(num_cols);

  switch (norm_type) {
  case L2_NORM:  norms = validator.GetL2norms(); break;
  case RMS_NORM: norms = validator.GetRMSnorms(); break;
  case INF_NORM: norms = validator.GetINFnorms(); break;
  }

  for (size_t col = 0; col < num_cols; col++) {
    if (norms[col] > tolerance)
      return false;
  }

  return true;
}


}  // namespace utils
}  // namespace chrono