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
// Utility classes and functions for input/output file operations.
//
// CSV_Writer
//  class that encapsulates functionality for writing files in Comma Separated
//  Values format.
//
// WriteCheckpoint and ReadCheckpoint
//  these functions write and read, respectively, a checkpoint file.
//  Limitations:
//    - it is assumed that the visualization asset geometry exctly matches the
//      contact geometry.
//    - only a subset of contact shapes are currently supported
//
// WriteShapesPovray
//  this function writes a CSV file appropriate for processing with a POV-Ray
//  script.
//
// =============================================================================

#ifndef CH_UTILS_INOUT_H
#define CH_UTILS_INOUT_H

#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChBezierCurve.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtilsCreators.h"

namespace chrono {
namespace utils {

// -----------------------------------------------------------------------------
// CSV_writer
//
// Simple class to output to a Comma-Separated Values file.
// -----------------------------------------------------------------------------
class ChApi CSV_writer {
 public:
  explicit CSV_writer(const std::string& delim = ",") : m_delim(delim) {}

  CSV_writer(const CSV_writer& source) : m_delim(source.m_delim) {
    // Note that we do not copy the stream buffer (as then it would be shared!)
    m_ss.copyfmt(source.m_ss);          // copy all data
    m_ss.clear(source.m_ss.rdstate());  // copy the error state
  }

  ~CSV_writer() {}

  void write_to_file(const std::string& filename, const std::string& header = "") const {
    std::ofstream ofile(filename.c_str());
    ofile << header;
    ofile << m_ss.str();
    ofile.close();
  }

  const std::string& delim() const { return m_delim; }
  std::ostringstream& stream() { return m_ss; }

  template <typename T>
  CSV_writer& operator<<(const T& t) {
    m_ss << t << m_delim;
    return *this;
  }

  CSV_writer& operator<<(std::ostream& (*t)(std::ostream&)) {
    m_ss << t;
    return *this;
  }
  CSV_writer& operator<<(std::ios& (*t)(std::ios&)) {
    m_ss << t;
    return *this;
  }
  CSV_writer& operator<<(std::ios_base& (*t)(std::ios_base&)) {
    m_ss << t;
    return *this;
  }

 private:
  std::string m_delim;
  std::ostringstream m_ss;
};

template <typename T>
inline CSV_writer& operator<<(CSV_writer& out, const ChVector<T>& v) {
  out << v.x() << v.y() << v.z();
  return out;
}

template <typename T>
inline CSV_writer& operator<<(CSV_writer& out, const ChQuaternion<T>& q) {
  out << q.e0() << q.e1() << q.e2() << q.e3();
  return out;
}

inline CSV_writer& operator<<(CSV_writer& out, const ChColor& c) {
  out << c.R << c.G << c.B;
  return out;
}

// -----------------------------------------------------------------------------
// Free function declarations
// -----------------------------------------------------------------------------

// This function dumps to a CSV file pody position, orientation, and optionally
// linear and angular velocity. Optionally, only active bodies are processed.
ChApi
void WriteBodies(ChSystem* system,
                 const std::string& filename,
                 bool active_only = false,
                 bool dump_vel = false,
                 const std::string& delim = ",");

// Create a CSV file with a checkpoint...
ChApi
bool WriteCheckpoint(ChSystem* system, const std::string& filename);

// Read a CSV file with a checkpoint...
ChApi
void ReadCheckpoint(ChSystem* system, const std::string& filename);

// Write CSV output file for PovRay.
// Each line contains information about one visualization asset shape, as
// follows:
//    index, x, y, z, e0, e1, e2, e3, type, geometry
// where 'geometry' depends on 'type' (an enum).
ChApi
void WriteShapesPovray(ChSystem* system,
                       const std::string& filename,
                       bool body_info = true,
                       const std::string& delim = ",");

// Write the specified mesh as a macro in a PovRay include file. The output file
// will be "[out_dir]/[mesh_name].inc". The mesh vertices will be transformed to
// the frame with specified offset and orientation.
ChApi void WriteMeshPovray(geometry::ChTriangleMeshConnected trimesh,
                           const std::string& mesh_name,
                           const std::string& out_dir,
                           const ChColor& color = ChColor(0.4f, 0.4f, 0.4f),
                           const ChVector<>& pos = ChVector<>(0, 0, 0),
                           const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                           bool smoothed = false);

// Write the triangular mesh from the specified OBJ file as a macro in a PovRay
// include file. The output file will be "[out_dir]/[mesh_name].inc". The mesh
// vertices will be transformed to the frame with specified offset and
// orientation.
ChApi
void WriteMeshPovray(const std::string& obj_filename,
                     const std::string& mesh_name,
                     const std::string& out_dir,
                     const ChColor& color = ChColor(0.4f, 0.4f, 0.4f),
                     const ChVector<>& pos = ChVector<>(0, 0, 0),
                     const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0));

// Write the Bezier curve from
ChApi void WriteCurvePovray(const ChBezierCurve& curve,
                            const std::string& curve_name,
                            const std::string& out_dir,
                            double radius = 0.03,
                            const ChColor& col = ChColor(0.8f, 0.8f, 0.2f));

}  // namespace utils
}  // namespace chrono

#endif
