
// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist
// =============================================================================
//
// GPS utils
//
// =============================================================================

#include "chrono/utils/ChConstants.h"

#include "chrono_sensor/utils/ChGPSUtils.h"

namespace chrono {
namespace sensor {

void Cartesian2GPS(ChVector3d& coords, ChVector3d& ref) {
    // convert from cartesian to gps coordinates assuming a sphere
    double lat = (coords.y() / EARTH_RADIUS) * 180.0 / chrono::CH_PI + ref.y();
    double lon = (coords.x() / (EARTH_RADIUS * cos(lat * chrono::CH_PI / 180.0))) * 180.0 / chrono::CH_PI + ref.x();
    double alt = coords.z() + ref.z();

    if (lon < -180.0) {
        lon = lon + 360.0;
    } else if (lon > 180.0) {
        lon = lon - 360.0;
    }
    coords = chrono::ChVector3d({lon, lat, alt});
}

void GPS2Cartesian(ChVector3d& coords, ChVector3d& ref) {
    double lon = coords.x();
    double lat = coords.y();
    double alt = coords.z();

    double x = ((lon - ref.x()) * chrono::CH_PI / 180.0) * (EARTH_RADIUS * cos(lat * chrono::CH_PI / 180.0));
    double y = ((lat - ref.y()) * chrono::CH_PI / 180.0) * EARTH_RADIUS;
    double z = alt - ref.z();

    coords = chrono::ChVector3d({x, y, z});
}

void Cartesian2ENU(ChVector3d& coords, ChVector3d& ref) {
    // convert from cartesian to gps coordinates assuming a sphere
    double lat = ref.y();
    double lon = ref.x();
    ////double alt = ref.z();

    double x = coords.x();
    double y = coords.y();
    double z = coords.z();

    double clat = cos(lat * chrono::CH_PI / 180.0);
    double slat = sin(lat * chrono::CH_PI / 180.0);
    double clon = cos(lon * chrono::CH_PI / 180.0);
    double slon = sin(lon * chrono::CH_PI / 180.0);

    double dx = x - ref.x();
    double dy = y - ref.y();
    double dz = z - ref.z();

    coords.x() = -slon * dx + clon * dy;
    coords.y() = -slat * clon * dx - slat * slon * dy + clat * dz;
    coords.z() = clat * clon * dx + clat * slon * dy + slat * dz;
}

void ENU2Cartesian(ChVector3d& coords, ChVector3d& ref) {
    // convert from cartesian to gps coordinates assuming a sphere
    double lat = ref.y();
    double lon = ref.x();
    ////double alt = ref.z();

    double x = coords.x();
    double y = coords.y();
    double z = coords.z();

    double clat = cos(lat * chrono::CH_PI / 180.0);
    double slat = sin(lat * chrono::CH_PI / 180.0);
    double clon = cos(lon * chrono::CH_PI / 180.0);
    double slon = sin(lon * chrono::CH_PI / 180.0);

    double dx = -slon * x - slat * clon * y + clat * clon * z;
    double dy = clon * x - slat * slon * y + clat * slon * z;
    double dz = clat * y + slat * z;

    coords.x() = dx + ref.x();
    coords.y() = dy + ref.y();
    coords.z() = dz + ref.z();
}

void GPS2ENU(ChVector3d& coords, ChVector3d& ref) {
    GPS2Cartesian(coords, ref);
    Cartesian2ENU(coords, ref);
}

void ENU2GPS(ChVector3d& coords, ChVector3d& ref) {
    ENU2Cartesian(coords, ref);
    Cartesian2GPS(coords, ref);
}

}  // namespace sensor
}  // namespace chrono
