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
// WriteVisualizationAssets
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
#include <functional>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChBezierCurve.h"
#include "chrono/assets/ChColor.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
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
        ofile << header << std::endl;
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

template <typename T>
inline CSV_writer& operator<<(CSV_writer& out, const std::vector<T>& vec) {
    for (const auto& v : vec)
        out << v;
    return out;
}

// -----------------------------------------------------------------------------
// Free function declarations
// -----------------------------------------------------------------------------

/// This function dumps to a CSV file pody position, orientation, and optionally linear and angular velocity.
/// Optionally, only active bodies are processed.
ChApi void WriteBodies(ChSystem* system,
                       const std::string& filename,
                       bool active_only = false,
                       bool dump_vel = false,
                       const std::string& delim = ",");

/// Create a CSV file with a checkpoint.
ChApi bool WriteCheckpoint(ChSystem* system, const std::string& filename);

/// Read a CSV file with a checkpoint.
ChApi void ReadCheckpoint(ChSystem* system, const std::string& filename);

/// Write CSV output file with camera information for off-line visualization.
/// The output file includes three vectors, one per line, for camera position, camera target (look-at point), and camera
/// up vector, respectively.
ChApi void WriteCamera(const std::string& filename,
                       const ChVector<>& cam_location,
                       const ChVector<>& cam_target,
                       const ChVector<>& camera_upvec,
                       const std::string& delim = ",");

/// Write CSV output file with body and asset information for off-line visualization.
/// (1) The first line of the output file contains:
/// - the number of bodies (0 if no body information is included below)
/// - the number of visual assets
/// - the number of links
/// - the number of TSDA spring-dampers
/// (2) If the number of bodies is not zero, the next block of lines includes information about the rigid bodies. Each
/// line contains 9 values:
/// - the body identifier
/// - a flag (1 or 0) indicating whether the body is active or not
/// - the body position expressed in global frame
/// - the body orientation (as a quaternion, expressed in global frame)
/// (3) The next block of lines includes information about the visual assets. Each line contains:
/// - the identifier of the associated body
/// - a flag (1 or 0) indicating whether the body is active or not
/// - the asset position expresssed in global frame
/// - the asset orientation (as a quaternion, expressed in global frame)
/// - the visualization asset type (an enum value)
/// - geometry information depending on the type above
/// (4) The next block of lines includes information about the joints. Each line contains:
/// - the joint type (an enum value)
/// - joint position and orientation information depending on the type above
/// (5) The next block of lines includes information about TSDA elements. Each line contains:
/// - the type of visual asset (0 for segment or 1 for coil)
/// - start point position
/// - end point position
ChApi void WriteVisualizationAssets(ChSystem* system,               ///< containg system
                                    const std::string& filename,    ///< output file name
                                    bool body_info = true,          ///< include body state information
                                    const std::string& delim = ","  ///< CSV delimitator
);

/// Write CSV output file with body and asset information for off-line visualization.
/// This version uses a discriminator function to select which bodies and assets are included in the output.
/// The function selector receives as argument a body reference and must return 'true' if the body and its assets should
/// be included in the output file and 'false' otherwise.
ChApi void WriteVisualizationAssets(ChSystem* system,                             ///< containg system
                                    const std::string& filename,                  ///< output file name
                                    std::function<bool(const ChBody&)> selector,  ///< select bodies
                                    bool body_info = true,                        ///< include body state information
                                    const std::string& delim = ","                ///< CSV delimitator
);

/// Write the specified mesh as a macro in a PovRay include file. The output file will be "[out_dir]/[mesh_name].inc".
/// The mesh vertices will be transformed to the frame with specified offset and orientation.
ChApi void WriteMeshPovray(geometry::ChTriangleMeshConnected& trimesh,
                           const std::string& mesh_name,
                           const std::string& out_dir,
                           const ChColor& color = ChColor(0.4f, 0.4f, 0.4f),
                           const ChVector<>& pos = ChVector<>(0, 0, 0),
                           const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                           bool smoothed = false);

/// Write the triangular mesh from the specified OBJ file as a macro in a PovRay include file. The output file will be
/// "[out_dir]/[mesh_name].inc". The mesh vertices will be transformed to the frame with specified offset and
/// orientation.
ChApi bool WriteMeshPovray(const std::string& obj_filename,
                           const std::string& mesh_name,
                           const std::string& out_dir,
                           const ChColor& color = ChColor(0.4f, 0.4f, 0.4f),
                           const ChVector<>& pos = ChVector<>(0, 0, 0),
                           const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0));

/// Write the specified Bezier curve as a macro in a PovRay include file.
ChApi void WriteCurvePovray(const ChBezierCurve& curve,
                            const std::string& curve_name,
                            const std::string& out_dir,
                            double radius = 0.03,
                            const ChColor& col = ChColor(0.8f, 0.8f, 0.2f));

}  // namespace utils
}  // namespace chrono

#endif
