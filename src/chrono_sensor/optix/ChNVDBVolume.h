
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
// A class to contain a volumetric object rendered using NVDB voxels.
//
// =============================================================================

#ifndef CH_NVDB_VOLUME_H
#define CH_NVDB_VOLUME_H

#include "chrono/core/ChApiCE.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChBox.h"
#include "chrono_sensor/ChApiSensor.h"

namespace chrono {
namespace sensor {

/// Volumetric object using NanoVDB voxels.
class CH_SENSOR_API ChNVDBShape : public ChVisualShape {
  public:
    ChNVDBShape();
    ChNVDBShape(const ChBox& box);
    ~ChNVDBShape() {}

    /// Access the box geometry.
    ChBox& GetBoxGeometry() { return gbox; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    ChBox gbox;
};

/// Body for NanoVDB visualization volume.
/// When building Chrono::Sensor, enable CH_USE_SENSOR_NVDB to link NanoVDB library to Chrono::Sensor.
/// Currently this can only be used to render Chrono::FSI SPH simulations. Use this in conjunction with
/// ChOptixScene::SetFSIParticles() and ChOptixScene::SetFSINumFSIParticles() to render Chrono::FSI SPH simulations.
class CH_SENSOR_API ChNVDBVolume : public ChBody {
  public:
    /// Creates a NanoVDB volume which is used as a bounding volume for rendering volumetric data.
    ChNVDBVolume(double Xsize,          ///< x dimension
                 double Ysize,          ///< Y dimension
                 double Zsize,          ///< Z dimension
                 double density,        ///< density of the body
                 bool visualize = true  ///< create visualization asset
    );

  private:
    void SetupBody(double Xsize, double Ysize, double Zsize, double density, bool visualize);
};

}  // namespace sensor
}  // namespace chrono

#endif