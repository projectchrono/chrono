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
// Authors: Rainer Gericke
// =============================================================================
//
// M113 idler subsystem.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/marder/Marder_Idler.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace marder {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Marder_Idler::m_wheel_mass = 25.76;
const ChVector<> Marder_Idler::m_wheel_inertia(0.618, 1.12, 0.618);
const double Marder_Idler::m_wheel_radius = 0.334;
const double Marder_Idler::m_wheel_width = 0.181;
const double Marder_Idler::m_wheel_gap = 0.051;

const double Marder_Idler::m_carrier_mass = 10;
const ChVector<> Marder_Idler::m_carrier_inertia(0.04, 0.04, 0.04);
const double Marder_Idler::m_carrier_radius = 0.02;

const double Marder_Idler::m_tensioner_l0 = 0.75;
const double Marder_Idler::m_tensioner_f = 9.81 * 25000.0 / 10.0;  // 10% Weight Force      M113: 2e4;
const double Marder_Idler::m_tensioner_k = 2e6;
const double Marder_Idler::m_tensioner_c = Marder_Idler::m_tensioner_k * 0.05;

const std::string Marder_IdlerLeft::m_meshFile = "Marder/Idler_L.obj";
const std::string Marder_IdlerRight::m_meshFile = "Marder/Idler_R.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
class Marder_TensionerForce : public ChLinkTSDA::ForceFunctor {
  public:
    Marder_TensionerForce(double k, double c, double f) : m_k(k), m_c(c), m_f(f) {}

    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override {
        return m_f - m_k * (length - rest_length) - m_c * vel;
    }

  private:
    double m_k;
    double m_c;
    double m_f;
};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Marder_Idler::Marder_Idler(const std::string& name) : ChDoubleIdler(name) {
    m_tensionerForceCB = chrono_types::make_shared<Marder_TensionerForce>(m_tensioner_k, m_tensioner_c, m_tensioner_f);
}

void Marder_Idler::CreateContactMaterial(ChContactMethod contact_method) {
    MaterialInfo minfo;
    minfo.mu = 0.4f;
    minfo.cr = 0.75f;
    minfo.Y = 1e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Marder_Idler::AddVisualizationAssets(VisualizationType vis) {
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
const ChVector<> Marder_Idler::GetLocation(PointId which) {
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

}  // namespace marder
}  // end namespace vehicle
}  // end namespace chrono
