
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
// A class to contain a volumetric object renderd using NVDB Voxels
//
// =============================================================================

#ifndef CHNVDBVOLUME_H
#define CHNVDBVOLUME_H

#include "chrono/core/ChApiCE.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
//#include "chrono/assets/ChBoxShape.h"

#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChBox.h"
#include "chrono_sensor/ChApiSensor.h"

//#include "chrono_sensor/cuda/cuda_utils.cuh"


namespace chrono {
namespace sensor {

class CH_SENSOR_API ChNVDBShape : public ChVisualShape {
  public:
    ChNVDBShape();
    ChNVDBShape(const geometry::ChBox& box);

    ~ChNVDBShape(){};

    /// Access the box geometry.
    geometry::ChBox& GetBoxGeometry() { return gbox; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    geometry::ChBox gbox;
};

class CH_SENSOR_API ChNVDBVolume : public ChBody {
  public:
    /// Creates a NanoVDB volume which is used as a bounding volume for rendering volumetric data. 
    /// When building Chrono::Sensor, enable USE_NVDB to link NanoVDB library to Chrono::Sensor.
    /// Currently this can only be used to render Chrono::FSI SPH simulations.
    /// Use this in conjunction with ChScene::SetFSIParticles() and ChScene::SetFSINumFSIParticles() to render Chrono::FSI SPH simulations.
    ChNVDBVolume(double Xsize,
                 double Ysize,
                 double Zsize,
                 double density,                                         ///< density of the body
                 bool visualize = true                                  ///< create visualization asset
    );

   
  private:
    void SetupBody(double Xsize,
                   double Ysize,
                   double Zsize,
                   double density,
                   bool visualize);


};

}  // namespace sensor
} // namespace chrono

#endif