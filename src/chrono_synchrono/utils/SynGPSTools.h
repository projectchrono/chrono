// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// Class wrapping a vector into a GPS coordinate, along with helper functions
// to translate BezierCurves between ChVectors and GPS coordinates. There is
// some overlap between the functions here and those in ChGPSSensor, in the
// future they will share the same codebase, but for now take care when using
// both of them.
//
// =============================================================================

#ifndef SYN_FRAMEWORK_H
#define SYN_FRAMEWORK_H

#include "chrono/core/ChVector3.h"
#include "chrono/core/ChBezierCurve.h"

#include "chrono_vehicle/ChTerrain.h"

#include "chrono_synchrono/agent/SynAgent.h"

#ifdef CHRONO_SENSOR
    #include "chrono_sensor/sensors/ChGPSSensor.h"
#else
    #define EARTH_RADIUS 6371000.0  // [meters]
#endif

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_utils
/// @{

/// @brief Wrapper class around vector stores GPS points as (lat, long, alt) in degrees
class SYN_API GPScoord {
  public:
    /// Constructs a GPScoord with a default z value
    GPScoord(double x, double y, double z = 0) : m_vector(x, y, z) {}

    /// Access to components
    /// Primarily for cleaner and more understandable code
    const double lat() const { return m_vector.x(); }
    const double lon() const { return m_vector.y(); }
    const double alt() const { return m_vector.z(); }

    /// Access to components with conversions
    const double lat_rad() const { return lat() * CH_DEG_TO_RAD; }
    const double lon_rad() const { return lon() * CH_DEG_TO_RAD; }

    ChVector3d GetVector() const { return m_vector; }
    void SetVector(ChVector3d vector) { m_vector = vector; }

  private:
    ChVector3d m_vector;
};

/// @brief Holds a SynTerrain along with the GPS coordinate mapped to the origin of the vector space
class SYN_API SynGPSTools {
  public:
    /// Construct a SynGPSTools object with the specified origin and attached terrain
    SynGPSTools(const GPScoord& origin, std::shared_ptr<vehicle::ChTerrain> terrain);

    /// Destructor
    ~SynGPSTools();

    /// @brief Optionally pass in a vertical offset for the points and whether the path should be a closed curve
    std::shared_ptr<ChBezierCurve> CurveFromGPS(std::vector<GPScoord>& gps_points,
                                                double vert_offset = 0,
                                                bool closed = 0);

    /// @brief Generate a ChBezierCurve from gps waypoints specified through a file
    /// Optionally pass in a vertical offset for the points and whether the path should be a closed curve
    std::shared_ptr<ChBezierCurve> CurveFromGPS(const std::string& filename, double vert_offset = 0, bool closed = 0);

    /// @brief Convert GPS coordinate to 3D cartesian point with an optionally specified height
    /// @param gps the point to convert
    /// @param height optional z/altitude-offset, defaults to 0.5
    ChVector3d To3DCartesian(const GPScoord& gps, double height = 0.5) const;

  private:
    const std::shared_ptr<vehicle::ChTerrain> m_terrain;  ///< handle to the terrain attached to this framework

    GPScoord m_origin;    ///< origin associated with this GPS Tool
    double m_lat_origin;  ///< easy access for latitude of the origin
    double m_lon_origin;  ///< easy access for the longitude of the origin
    double m_cos_origin;  ///< cosine of the latitude of the origin that is used for conversions
};

/// @} synchrono_utils

}  // namespace synchrono
}  // namespace chrono

#endif
