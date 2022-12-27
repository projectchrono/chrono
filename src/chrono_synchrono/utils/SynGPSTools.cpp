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
// Class wrapping a ChVector into a GPS coordinate, along with helper functions
// to translate BezierCurves between ChVectors and GPS coordinates. There is
// some overlap between the functions here and those in ChGPSSensor, in the
// future they will share the same codebase, but for now take care when using
// both of them.
//
// =============================================================================

#include "chrono_synchrono/utils/SynGPSTools.h"

#include "chrono_synchrono/utils/SynLog.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace chrono {
namespace synchrono {

SynGPSTools::SynGPSTools(const GPScoord& origin, std::shared_ptr<ChTerrain> terrain)
    : m_terrain(terrain) , m_origin(origin) {
    // Store origin info in radians, from degree-based GPScoord
    m_lat_origin = origin.lat_rad();
    m_lon_origin = origin.lon_rad();
    m_cos_origin = std::cos(m_lat_origin);
}
SynGPSTools::~SynGPSTools() {}

std::shared_ptr<ChBezierCurve> SynGPSTools::CurveFromGPS(std::vector<GPScoord>& gps_points,
                                                         double vert_offset,
                                                         bool closed) {
    std::vector<ChVector<>> bezier_points;
    bezier_points.reserve(gps_points.size());
    for (const auto& gps_point : gps_points)
        bezier_points.push_back(To3DCartesian(gps_point, vert_offset));

    bool is_already_closed = (gps_points.back().GetVector() - gps_points.front().GetVector()).Length() < 1e-6;
    if (closed && !is_already_closed)
        bezier_points.push_back(bezier_points.front());

    return chrono_types::make_shared<ChBezierCurve>(bezier_points);
}

// File must be in the following format:
//  1: <num_waypoints>  <num_cols>
//  2: lat_0            lon_0
//  3: lat_1            lon_1
//  4: ...              ...
// TODO: Support files with altitude
std::shared_ptr<ChBezierCurve> SynGPSTools::CurveFromGPS(const std::string& filename, double vert_offset, bool closed) {
    // Open input file stream
    std::ifstream ifile;
    std::string line;
    try {
        ifile.exceptions(std::ios::failbit | std::ios::badbit | std::ios::eofbit);
        ifile.open(filename.c_str());
    } catch (std::exception&) {
        throw ChException("Cannot open input file");
    }

    // Read number of knots and type of curve
    size_t num_waypoints;
    size_t num_cols;

    {
        std::getline(ifile, line);
        std::istringstream iss(line);
        iss >> num_waypoints >> num_cols;
        iss.precision(9);
    }

    if (num_cols == 2) {
        // Read points as GPS waypoints
        std::vector<ChVector<>> waypoints;
        waypoints.reserve(num_waypoints);
        for (size_t i = 0; i < num_waypoints; i++) {
            double lat, lon;

            std::getline(ifile, line);
            std::istringstream iss(line);
            iss >> lat >> lon;

            waypoints.push_back(To3DCartesian(GPScoord(lat, lon), vert_offset));
        }

        // Close the path, if necessary
        bool is_already_closed = (waypoints.back() - waypoints.front()).Length() < 1e-6;
        if (closed && !is_already_closed)
            waypoints.push_back(waypoints.front());

        ifile.close();
        return chrono_types::make_shared<ChBezierCurve>(waypoints);
    }

    if (num_cols == 3 || num_cols == 9) {
        // TODO: This method should actually support numcols=3 b/c of altitude
        SynLog() << "SynGPSTools::CurveFromGPS: File specified actually describes a ChBezierCurve, not a GPS curve, "
                    "reading anyways"
                 << "\n";

        ifile.close();
        return ChBezierCurve::read(filename);
    }

    // Not the expected number of columns.  Close the file and throw an exception.
    ifile.close();
    throw ChException("Invalid input file");
}

// TODO Add support for altitude
ChVector<> SynGPSTools::To3DCartesian(const GPScoord& gps, double height) const {
    auto lat_rad = gps.lat_rad();
    auto lon_rad = gps.lon_rad();

    // x is East, y is North
    auto x = EARTH_RADIUS * (lon_rad - m_lon_origin) * m_cos_origin;
    auto y = EARTH_RADIUS * (lat_rad - m_lat_origin);

    auto z = m_terrain->GetHeight({x, y, 0}) + height;
    return {x, y, z};
}

}  // namespace synchrono
}  // namespace chrono
