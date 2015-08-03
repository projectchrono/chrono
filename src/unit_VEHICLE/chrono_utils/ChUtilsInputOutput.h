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
// A set of utility classes and functions for I/O.
//
// =============================================================================

#ifndef CH_UTILS_INOUT_H
#define CH_UTILS_INOUT_H

#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

#include "physics/ChSystem.h"
#include "assets/ChColor.h"

#include "utils/ChApiUtils.h"
#include "utils/ChUtilsCreators.h"


namespace chrono {
namespace utils {


// -----------------------------------------------------------------------------
// CSV_writer
//
// Simple class to output to a Comma-Separated Values file.
// -----------------------------------------------------------------------------
class CH_UTILS_API CSV_writer {
public:
  explicit CSV_writer(const std::string& delim = ",") : m_delim(delim) {}

  CSV_writer(const CSV_writer& source) : m_delim(source.m_delim)
  {
    // Note that we do not copy the stream buffer (as then it would be shared!)
    m_ss.copyfmt(source.m_ss);          // copy all data
    m_ss.clear(source.m_ss.rdstate());  // copy the error state
  }

  ~CSV_writer() {}

  void write_to_file(const std::string& filename,
                     const std::string& header = "")
  {
    std::ofstream ofile(filename.c_str());
    ofile << header;
    ofile << m_ss.str();
    ofile.close();
  }

  const std::string&  delim() const { return m_delim; }
  std::ostringstream& stream() { return m_ss; }

  template <typename T>
  CSV_writer& operator<< (const T& t)                          { m_ss << t << m_delim; return *this; }

  CSV_writer& operator<<(std::ostream& (*t)(std::ostream&))    { m_ss << t; return *this; }
  CSV_writer& operator<<(std::ios& (*t)(std::ios&))            { m_ss << t; return *this; }
  CSV_writer& operator<<(std::ios_base& (*t)(std::ios_base&))  { m_ss << t; return *this; }

private:
  std::string m_delim;
  std::ostringstream m_ss;
};

inline CSV_writer& operator<< (CSV_writer& out, const ChVector<>& v)
{
  out << v.x << v.y << v.z;
  return out;
}

inline CSV_writer& operator<< (CSV_writer& out, const ChQuaternion<>& q)
{
  out << q.e0 << q.e1 << q.e2 << q.e3;
  return out;
}

inline CSV_writer& operator<< (CSV_writer& out, const ChColor& c)
{
  out << c.R << c.G << c.B;
  return out;
}


// -----------------------------------------------------------------------------
// Free function declarations
// -----------------------------------------------------------------------------

// Write to a CSV file pody position, orientation, and (optionally) linear and
// angular velocity. Optionally, only active bodies are processed.
CH_UTILS_API
void WriteBodies(ChSystem*          system,
                 const std::string& filename,
                 bool               active_only = false,
                 bool               dump_vel = false,
                 const std::string& delim = ",");

// Create a CSV file with a checkpoint...
CH_UTILS_API
bool WriteCheckpoint(ChSystem*          system,
                     const std::string& filename);

// Read a CSV file with a checkpoint...
CH_UTILS_API
void ReadCheckpoint(ChSystem*          system,
                    const std::string& filename);

// Write CSV output file for PovRay.
// Each line contains information about one visualization asset shape, as
// follows:
//    index, x, y, z, e0, e1, e2, e3, type, geometry
// where 'geometry' depends on 'type' (an enum).
CH_UTILS_API
void WriteShapesPovray(ChSystem*          system,
                       const std::string& filename,
                       bool               body_info = true,
                       const std::string& delim = ",");

// Write the triangular mesh from the specified OBJ file as a macro in a PovRay
// include file. The output file will be "[out_dir]/[mesh_name].inc". The mesh
// vertices will be tramsformed to the frame with specified offset and
// orientation.
CH_UTILS_API
void WriteMeshPovray(const std::string&    obj_filename,
                     const std::string&    mesh_name,
                     const std::string&    out_dir,
                     const ChColor&        color = ChColor(0.4f, 0.4f, 0.4f),
                     const ChVector<>&     pos = ChVector<>(0, 0, 0),
                     const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0));


} // namespace utils
} // namespace chrono


#endif
