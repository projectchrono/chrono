// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nevindu Batagoda
// =============================================================================
//
// A class to contain a volumetric object renderd using NVDB Voxels. 
// Inherits elements from ChBody and ChBox
//
// =============================================================================


#include "chrono_sensor/optix/ChNVDBVolume.h"

namespace chrono {
namespace sensor {

ChNVDBShape::ChNVDBShape() {
    SetMutable(false);
}

ChNVDBShape::ChNVDBShape(const ChBox& box) : gbox(box) {
    SetMutable(false);
}

void ChNVDBShape::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChNVDBShape>();
    // serialize parent class
    ChVisualShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(gbox);
}

void ChNVDBShape::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChNVDBShape>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(gbox);
}

ChNVDBVolume::ChNVDBVolume(double Xsize,
                           double Ysize,
                           double Zsize,
                           double density,
                           bool visualize)
    : ChBody() {
    SetupBody(Xsize, Ysize, Zsize, density, visualize);
}

void ChNVDBVolume::SetupBody(double Xsize,
                                 double Ysize,
                                 double Zsize,
                                 double density,
                                 bool visualize
                                 ) {
    double mmass = density * (Xsize * Ysize * Zsize);

    SetMass(mmass);
    SetInertiaXX(ChVector3d((1.0 / 12.0) * mmass * (pow(Ysize, 2) + pow(Zsize, 2)),
                                  (1.0 / 12.0) * mmass * (pow(Xsize, 2) + pow(Zsize, 2)),
                                  (1.0 / 12.0) * mmass * (pow(Xsize, 2) + pow(Ysize, 2))));
    if (visualize) {
        auto vshape = chrono_types::make_shared<ChNVDBShape>();
        vshape->GetBoxGeometry().SetLengths(ChVector3d(Xsize, Ysize, Zsize));
        auto vmodel = chrono_types::make_shared<ChVisualModel>();
        vmodel->AddShape(vshape);
        this->AddVisualModel(vmodel);
    }
}

}  // namespace sensor
} //namespace chrono