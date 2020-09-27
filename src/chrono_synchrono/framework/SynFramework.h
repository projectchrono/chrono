#ifndef SYN_FRAMEWORK_H
#define SYN_FRAMEWORK_H

#include "chrono_synchrono/SynApi.h"
#include "chrono_synchrono/agent/SynAgent.h"
#include "chrono_synchrono/terrain/SynTerrain.h"

#include "chrono/core/ChVector.h"
#include "chrono/core/ChBezierCurve.h"

#ifdef CHRONO_SENSOR
#include "chrono_sensor/ChGPSSensor.h"
#else
// [meters]
#define EARTH_RADIUS 6371000.0
#endif  // SENSOR

using namespace chrono;

namespace chrono {
namespace synchrono {

/// GPScoord is a wrapper class around ChVector
/// Stores GPS points as (lat, long, alt)
/// All coordinates are in degrees
class SYN_API GPScoord : public ChVector<> {
  public:
    /// Constructs a GPScoord with a default z value
    GPScoord(double x, double y, double z = 0) : ChVector<>(x, y, z) {}

    /// Access to components
    /// Primarily for cleaner and more understandable code
    double lat() { return x(); }
    double lon() { return y(); }
    double alt() { return z(); }
    const double lat() const { return x(); }
    const double lon() const { return y(); }
    const double alt() const { return z(); }

    /// Access to components with conversions
    double lat_rad() { return lat() * CH_C_DEG_TO_RAD; }
    double lon_rad() { return lon() * CH_C_DEG_TO_RAD; }
    const double lat_rad() const { return lat() * CH_C_DEG_TO_RAD; }
    const double lon_rad() const { return lon() * CH_C_DEG_TO_RAD; }
};

class SYN_API SynFramework {
  public:
    /// Construct a SynFramework object with the specified origin and attached terrain
    SynFramework(const GPScoord& origin, std::shared_ptr<SynTerrain> terrain);

    /// Destructor
    ~SynFramework();

    /// Generate a ChBezierCurve from gps waypoints
    /// Optionally pass in a vertical offset for the points and whether the path should be a closed curve
    std::shared_ptr<ChBezierCurve> CurveFromGPS(std::vector<GPScoord>& gps_points,
                                                double vert_offset = 0,
                                                bool closed = 0);

    /// Generate a ChBezierCurve from gps waypoints specified through a file
    /// Optionally pass in a vertical offset for the points and whether the path should be a closed curve
    std::shared_ptr<ChBezierCurve> CurveFromGPS(const std::string& filename, double vert_offset = 0, bool closed = 0);

    /// Convert GPS coordinate to 3D cartesian point with an optionally specified height
    ChVector<> To3DCartesian(const GPScoord& gps, double height = 0.5) const;

  private:
    const std::shared_ptr<SynTerrain> m_terrain;  ///< handle to the SynTerrain attached to this framework

    GPScoord m_origin;    ///< origin associated with this framework
    double m_lat_origin;  ///< easy access for latitude of the origin
    double m_lon_origin;  ///< easy access for the longitude of the origin
    double m_cos_origin;  ///< cosine of the latitude of the origin that is used for conversions
};

}  // namespace synchrono
}  // namespace chrono

#endif
