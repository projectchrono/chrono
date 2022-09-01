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
// Authors: Radu Serban
// =============================================================================
//
// M113 idler subsystem.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/m113/M113_IdlerWheel.h"
#include "chrono_models/vehicle/m113/M113_Idler.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113_Idler::m_carrier_mass = 10;
const ChVector<> M113_Idler::m_carrier_inertia(0.04, 0.04, 0.04);
const double M113_Idler::m_carrier_radius = 0.02;

const double M113_Idler::m_tensioner_l0 = 0.75;
const double M113_Idler::m_tensioner_f = 2e4;
const double M113_Idler::m_tensioner_k = 1e6;
const double M113_Idler::m_tensioner_c = 1.4e4;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
class M113_TensionerForce : public ChLinkTSDA::ForceFunctor {
  public:
    M113_TensionerForce(double k, double c, double f) : m_k(k), m_c(c), m_f(f) {}

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

M113_Idler::M113_Idler(const std::string& name, VehicleSide side) : ChTranslationalIdler(name), m_side(side) {
    m_tensionerForceCB = chrono_types::make_shared<M113_TensionerForce>(m_tensioner_k, m_tensioner_c, m_tensioner_f);

    // Create the associated idler wheel.
    if (side == LEFT)
        m_idler_wheel = chrono_types::make_shared<M113_IdlerWheelLeft>();
    else
        m_idler_wheel = chrono_types::make_shared<M113_IdlerWheelRight>();
}

const ChVector<> M113_Idler::GetLocation(PointId which) {
    ChVector<> point;

    switch (which) {
        case CARRIER_WHEEL:
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

    if (m_side == RIGHT)
        point.y() *= -1;

    return point;
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
