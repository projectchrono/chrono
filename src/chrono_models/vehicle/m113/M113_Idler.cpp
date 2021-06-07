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
// M113 idler subsystem.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/m113/M113_Idler.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113_Idler::m_wheel_mass = 25.76;
const ChVector<> M113_Idler::m_wheel_inertia(0.618, 1.12, 0.618);
const double M113_Idler::m_wheel_radius = 0.255;
const double M113_Idler::m_wheel_width = 0.181;
const double M113_Idler::m_wheel_gap = 0.051;

const double M113_Idler::m_carrier_mass = 10;
const ChVector<> M113_Idler::m_carrier_inertia(0.04, 0.04, 0.04);
const double M113_Idler::m_carrier_radius = 0.02;

const double M113_Idler::m_tensioner_l0 = 0.75;
const double M113_Idler::m_tensioner_f = 2e4;
const double M113_Idler::m_tensioner_k = 1e6;
const double M113_Idler::m_tensioner_c = 1.4e4;

const std::string M113_IdlerLeft::m_meshFile = "M113/Idler_L.obj";
const std::string M113_IdlerRight::m_meshFile = "M113/Idler_R.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
class M113_TensionerForce : public ChLinkTSDA::ForceFunctor {
  public:
    M113_TensionerForce(double k, double c, double f) : m_k(k), m_c(c), m_f(f) {}

    virtual double operator()(double time, double rest_length, double length, double vel, ChLinkTSDA* link) override {
        return m_f - m_k * (length - rest_length) - m_c * vel;
    }

  private:
    double m_k;
    double m_c;
    double m_f;
};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_Idler::M113_Idler(const std::string& name) : ChDoubleIdler(name) {
    m_tensionerForceCB = chrono_types::make_shared<M113_TensionerForce>(m_tensioner_k, m_tensioner_c, m_tensioner_f);
}

void M113_Idler::CreateContactMaterial(ChContactMethod contact_method) {
    MaterialInfo minfo;
    minfo.mu = 0.4f;
    minfo.cr = 0.75f;
    minfo.Y = 1e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_Idler::AddVisualizationAssets(VisualizationType vis) {
    ChDoubleIdler::AddVisualizationAssets(vis);

    if (vis == VisualizationType::MESH) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(GetMeshFile(), false, false);
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(GetMeshFile()).stem());
        trimesh_shape->SetStatic(true);
        m_wheel->AddAsset(trimesh_shape);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
const ChVector<> M113_Idler::GetLocation(PointId which) {
    ChVector<> point;

    switch (which) {
        case WHEEL:
            point = ChVector<>(0, 0, 0);
            break;
        case CARRIER:
            point = ChVector<>(0, -0.1, 0);
            break;
        case CARRIER_CHASSIS:
            point = ChVector<>(0, -0.2, 0);
            break;
        case TSDA_CARRIER:
            point = ChVector<>(0, -0.2, 0);
            break;
        case TSDA_CHASSIS:
            point = ChVector<>(0.5, -0.2, 0);
            break;
        default:
            point = ChVector<>(0, 0, 0);
            break;
    }

    if (GetVehicleSide() == RIGHT)
        point.y() *= -1;

    return point;
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
