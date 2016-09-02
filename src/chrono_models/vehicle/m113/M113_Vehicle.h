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
// M113 vehicle model.
//
// =============================================================================

#ifndef M113_VEHICLE_H
#define M113_VEHICLE_H

#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"
#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace m113 {

class CH_MODELS_API M113_Vehicle : public ChTrackedVehicle {
  public:
    M113_Vehicle(bool fixed,
                 TrackShoeType shoe_type,
                 ChMaterialSurfaceBase::ContactMethod contactMethod = ChMaterialSurfaceBase::DVI);

    M113_Vehicle(bool fixed, TrackShoeType shoe_type, ChSystem* system);

    ~M113_Vehicle() {}

    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return m_driverCsys; }

    virtual void Initialize(const ChCoordsys<>& chassisPos) override;

    void SetChassisVisType(VisualizationType vis);
    void SetSprocketVisType(VisualizationType vis);
    void SetIdlerVisType(VisualizationType vis);
    void SetRoadWheelVisType(VisualizationType vis);
    void SetTrackShoeVisType(VisualizationType vis);

    void ExportMeshPovray(const std::string& out_dir);

  private:
    void Create(bool fixed);

    TrackShoeType m_type;  ///< type of track assembly (SINGLE_PIN or DOUBLE_PIN)

    static const std::string m_chassisMeshName;  ///< name of chassis visualization mesh
    static const std::string m_chassisMeshFile;  ///< name of Wavefront file with chassis visualization mesh

    static const double m_chassisMass;         ///< chassis mass
    static const ChVector<> m_chassisCOM;      ///< location of chassis center of mass
    static const ChVector<> m_chassisInertia;  ///< chassis inertia moments

    static const ChCoordsys<> m_driverCsys;  ///< driver local coordinate system

    VisualizationType m_chassisVisType;  ///< chassis visualization type
};

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
