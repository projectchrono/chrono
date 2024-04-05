// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include <fstream>
#include <iostream>
#include <sstream>

#include "chrono/collision/ChCollisionShapeConvexHull.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionShapeConvexHull)
CH_UPCASTING(ChCollisionShapeConvexHull, ChCollisionShape)

ChCollisionShapeConvexHull::ChCollisionShapeConvexHull() : ChCollisionShape(Type::CONVEXHULL) {}

ChCollisionShapeConvexHull::ChCollisionShapeConvexHull(std::shared_ptr<ChContactMaterial> material,
                                                       const std::vector<ChVector3d>& points)
    : ChCollisionShape(Type::CONVEXHULL, material) {
    this->points = points;
}

std::vector<std::shared_ptr<ChCollisionShapeConvexHull>> ChCollisionShapeConvexHull::Read(
    std::shared_ptr<ChContactMaterial> material,
    const std::string& filename) {
    // Open input file stream
    std::ifstream ifile;
    std::string line;
    try {
        ifile.open(filename);
    } catch (const std::exception&) {
        std::cerr << "ChCollisionShapeConvexHull::Read - cannot open input file " << filename << std::endl;
        throw std::invalid_argument("Cannot open input file");
    }

    std::vector<std::shared_ptr<ChCollisionShapeConvexHull>> hulls;
    std::vector<ChVector3d> points;

    while (std::getline(ifile, line)) {
        if (line.size() == 0 || line[0] == '#')
            continue;

        if (line.rfind("hull", 0) == 0) {
            if (points.size() > 0)
                hulls.push_back(chrono_types::make_shared<ChCollisionShapeConvexHull>(material, points));
            points.clear();
        }

        std::istringstream iss(line);
        ChVector3d p;
        iss >> p.x() >> p.y() >> p.z();
        points.push_back(p);
    }

    if (points.size() > 0)
        hulls.push_back(chrono_types::make_shared<ChCollisionShapeConvexHull>(material, points));

    return hulls;
}

void ChCollisionShapeConvexHull::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChCollisionShapeConvexHull>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(points);
}

void ChCollisionShapeConvexHull::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChCollisionShapeConvexHull>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(points);
}

}  // end namespace chrono
