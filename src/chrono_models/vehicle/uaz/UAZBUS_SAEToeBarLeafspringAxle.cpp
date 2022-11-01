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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// UAZBUS leafspring axle with SAE three-link model and Toebar Steering.
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "chrono_models/vehicle/uaz/UAZBUS_SAEToeBarLeafspringAxle.h"

namespace chrono {
namespace vehicle {
namespace uaz {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double UAZBUS_SAEToeBarLeafspringAxle::m_leafHeight = 0.008;
const double UAZBUS_SAEToeBarLeafspringAxle::m_leafWidth = 0.08;

const double UAZBUS_SAEToeBarLeafspringAxle::m_axleTubeMass = 124.0;
const double UAZBUS_SAEToeBarLeafspringAxle::m_spindleMass = 14.705;
const double UAZBUS_SAEToeBarLeafspringAxle::m_knuckleMass = 10.0;
const double UAZBUS_SAEToeBarLeafspringAxle::m_tierodMass = 5.0;
const double UAZBUS_SAEToeBarLeafspringAxle::m_draglinkMass = 5.0;

const double UAZBUS_SAEToeBarLeafspringAxle::m_axleTubeRadius = 0.0476;
const double UAZBUS_SAEToeBarLeafspringAxle::m_spindleRadius = 0.10;
const double UAZBUS_SAEToeBarLeafspringAxle::m_spindleWidth = 0.06;
const double UAZBUS_SAEToeBarLeafspringAxle::m_knuckleRadius = 0.05;
const double UAZBUS_SAEToeBarLeafspringAxle::m_tierodRadius = 0.02;
const double UAZBUS_SAEToeBarLeafspringAxle::m_draglinkRadius = 0.02;

const ChVector<> UAZBUS_SAEToeBarLeafspringAxle::m_axleTubeInertia(22.21, 0.0775, 22.21);
const ChVector<> UAZBUS_SAEToeBarLeafspringAxle::m_spindleInertia(0.04117, 0.07352, 0.04117);
const ChVector<> UAZBUS_SAEToeBarLeafspringAxle::m_knuckleInertia(0.1, 0.1, 0.1);
const ChVector<> UAZBUS_SAEToeBarLeafspringAxle::m_tierodInertia(1.0, 0.1, 1.0);
const ChVector<> UAZBUS_SAEToeBarLeafspringAxle::m_draglinkInertia(0.1, 1.0, 0.1);

const double UAZBUS_SAEToeBarLeafspringAxle::m_auxSpringDesignLength = 0.2;
const double UAZBUS_SAEToeBarLeafspringAxle::m_auxSpringCoefficient = 0;
const double UAZBUS_SAEToeBarLeafspringAxle::m_auxSpringRestLength = m_auxSpringDesignLength + 0.0;
const double UAZBUS_SAEToeBarLeafspringAxle::m_auxSpringMinLength = m_auxSpringDesignLength - 0.08;
const double UAZBUS_SAEToeBarLeafspringAxle::m_auxSpringMaxLength = m_auxSpringDesignLength + 0.08;
const double UAZBUS_SAEToeBarLeafspringAxle::m_damperCoefficient = 15079.644737231;
const double UAZBUS_SAEToeBarLeafspringAxle::m_damperDegressivityCompression = 3.0;
const double UAZBUS_SAEToeBarLeafspringAxle::m_damperDegressivityExpansion = 1.0;
const double UAZBUS_SAEToeBarLeafspringAxle::m_axleShaftInertia = 0.4;

const double UAZBUS_SAEToeBarLeafspringAxle::m_vert_spring_trans_A = 94748.2022504578 / 2.0;
const double UAZBUS_SAEToeBarLeafspringAxle::m_vert_spring_trans_B = 94748.2022504578 / 2.0;

const double UAZBUS_SAEToeBarLeafspringAxle::m_lat_spring_trans_A =
    10.0 * UAZBUS_SAEToeBarLeafspringAxle::m_vert_spring_trans_A;
const double UAZBUS_SAEToeBarLeafspringAxle::m_lat_spring_trans_B =
    10.0 * UAZBUS_SAEToeBarLeafspringAxle::m_vert_spring_trans_B;

const double UAZBUS_SAEToeBarLeafspringAxle::m_vert_preload = 2000.0;

const double UAZBUS_SAEToeBarLeafspringAxle::m_frontleafMass = 3.5168;
const double UAZBUS_SAEToeBarLeafspringAxle::m_rearleafMass = 3.5168;
const double UAZBUS_SAEToeBarLeafspringAxle::m_clampMass = 0.70336;
const double UAZBUS_SAEToeBarLeafspringAxle::m_shackleMass = 0.70336;

const ChVector<> UAZBUS_SAEToeBarLeafspringAxle::m_frontleafInertia = {0.00191239, 0.0733034, 0.0751423};
const ChVector<> UAZBUS_SAEToeBarLeafspringAxle::m_rearleafInertia = {0.00191239, 0.0733034, 0.0751423};
const ChVector<> UAZBUS_SAEToeBarLeafspringAxle::m_clampInertia = {0.000382478, 0.000593486, 0.000961259};
const ChVector<> UAZBUS_SAEToeBarLeafspringAxle::m_shackleInertia = {0.000382478, 0.000593486, 0.000961259};

// ---------------------------------------------------------------------------------------
// UAZBUS spring functor class - implements a linear spring + bump stop + rebound stop
// ---------------------------------------------------------------------------------------
class UAZBUS_AuxSpringForceFront : public ChLinkTSDA::ForceFunctor {
  public:
    UAZBUS_AuxSpringForceFront(double spring_constant, double min_length, double max_length);

    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;

  private:
    double m_spring_constant;
    double m_min_length;
    double m_max_length;

    ChFunction_Recorder m_bump;
};

UAZBUS_AuxSpringForceFront::UAZBUS_AuxSpringForceFront(double spring_constant, double min_length, double max_length)
    : m_spring_constant(spring_constant), m_min_length(min_length), m_max_length(max_length) {
    // From ADAMS/Car
    m_bump.AddPoint(0.0, 0.0);
    m_bump.AddPoint(2.0e-3, 200.0);
    m_bump.AddPoint(4.0e-3, 400.0);
    m_bump.AddPoint(6.0e-3, 600.0);
    m_bump.AddPoint(8.0e-3, 800.0);
    m_bump.AddPoint(10.0e-3, 1000.0);
    m_bump.AddPoint(20.0e-3, 2500.0);
    m_bump.AddPoint(30.0e-3, 4500.0);
    m_bump.AddPoint(40.0e-3, 7500.0);
    m_bump.AddPoint(50.0e-3, 12500.0);
}

double UAZBUS_AuxSpringForceFront::evaluate(double time,
                                            double rest_length,
                                            double length,
                                            double vel,
                                            const ChLinkTSDA& link) {
    double force = 0;

    double defl_spring = rest_length - length;
    double defl_bump = 0.0;
    double defl_rebound = 0.0;

    if (length < m_min_length) {
        defl_bump = m_min_length - length;
    }

    if (length > m_max_length) {
        defl_rebound = length - m_max_length;
    }

    force = defl_spring * m_spring_constant + m_bump.Get_y(defl_bump) - m_bump.Get_y(defl_rebound);

    return force;
}

// -----------------------------------------------------------------------------
// UAZBUS shock functor class - implements a nonlinear damper
// -----------------------------------------------------------------------------
class UAZBUS_SAEShockForceFront : public ChLinkTSDA::ForceFunctor {
  public:
    UAZBUS_SAEShockForceFront(double compression_slope,
                              double compression_degressivity,
                              double expansion_slope,
                              double expansion_degressivity);

    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;

  private:
    double m_slope_compr;
    double m_slope_expand;
    double m_degres_compr;
    double m_degres_expand;
};

UAZBUS_SAEShockForceFront::UAZBUS_SAEShockForceFront(double compression_slope,
                                                     double compression_degressivity,
                                                     double expansion_slope,
                                                     double expansion_degressivity)
    : m_slope_compr(compression_slope),
      m_degres_compr(compression_degressivity),
      m_slope_expand(expansion_slope),
      m_degres_expand(expansion_degressivity) {}

double UAZBUS_SAEShockForceFront::evaluate(double time,
                                           double rest_length,
                                           double length,
                                           double vel,
                                           const ChLinkTSDA& link) {
    // Simple model of a degressive damping characteristic
    double force = 0;

    // Calculate Damping Force
    if (vel >= 0) {
        force = -m_slope_expand / (1.0 + m_degres_expand * std::abs(vel)) * vel;
    } else {
        force = -m_slope_compr / (1.0 + m_degres_compr * std::abs(vel)) * vel;
    }

    return force;
}

UAZBUS_SAEToeBarLeafspringAxle::UAZBUS_SAEToeBarLeafspringAxle(const std::string& name)
    : ChSAEToeBarLeafspringAxle(name) {
    ChVector<> ra = getLocation(CLAMP_A) - getLocation(FRONT_HANGER);
    ChVector<> rb = getLocation(CLAMP_B) - getLocation(SHACKLE);

    ChVector<> preload(0, 0, m_vert_preload / 2.0);
    ChVector<> Ma = preload.Cross(ra);
    ChVector<> Mb = preload.Cross(rb);

    double KrotLatA = m_lat_spring_trans_A * pow(ra.Length(), 2.0);
    double KrotLatB = m_lat_spring_trans_B * pow(rb.Length(), 2.0);

    double KrotVertA = m_vert_spring_trans_A * pow(ra.Length(), 2.0);
    double KrotVertB = m_vert_spring_trans_B * pow(rb.Length(), 2.0);

    double rest_angle_A = Ma.y() / KrotVertA;
    double rest_angle_B = Mb.y() / KrotVertB;

    double damping_factor = 0.05;

    m_latRotSpringCBA = chrono_types::make_shared<LinearSpringDamperTorque>(KrotLatA, KrotLatA * damping_factor, 0);
    m_latRotSpringCBB = chrono_types::make_shared<LinearSpringDamperTorque>(KrotLatB, KrotLatA * damping_factor, 0);

    m_vertRotSpringCBA =
        chrono_types::make_shared<LinearSpringDamperTorque>(KrotVertA, KrotVertA * damping_factor, rest_angle_A);
    m_vertRotSpringCBB =
        chrono_types::make_shared<LinearSpringDamperTorque>(KrotVertB, KrotVertB * damping_factor, rest_angle_B);

    m_auxSpringForceCB = chrono_types::make_shared<UAZBUS_AuxSpringForceFront>(
        m_auxSpringCoefficient, m_auxSpringMinLength, m_auxSpringMaxLength);

    m_shockForceCB = chrono_types::make_shared<UAZBUS_SAEShockForceFront>(
        m_damperCoefficient, m_damperDegressivityCompression, m_damperCoefficient, m_damperDegressivityExpansion);
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
UAZBUS_SAEToeBarLeafspringAxle::~UAZBUS_SAEToeBarLeafspringAxle() {}

const ChVector<> UAZBUS_SAEToeBarLeafspringAxle::getLocation(PointId which) {
    switch (which) {
        case SPRING_A:
            return ChVector<>(0.0, 0.3824, m_axleTubeRadius);
        case SPRING_C:
            return ChVector<>(0.0, 0.3824, m_axleTubeRadius + m_auxSpringDesignLength);
        case SHOCK_A:
            return ChVector<>(-0.125, 0.441, -0.0507);
        case SHOCK_C:
            return ChVector<>(-0.3648, 0.4193, 0.3298);
        case SPINDLE:
            return ChVector<>(0.0, 0.7325, 0.0);
        case KNUCKLE_CM:
            return ChVector<>(0.0, 0.7325 - 0.07, 0.0);
        case KNUCKLE_L:
            return ChVector<>(0.0, 0.7325 - 0.07 + 0.0098058067569092, -0.1);
        case KNUCKLE_U:
            return ChVector<>(0.0, 0.7325 - 0.07 - 0.0098058067569092, 0.1);
        case KNUCKLE_DRL:
            return ChVector<>(0.0, 0.7325 - 0.2, 0.2);
        case TIEROD_K:
            return ChVector<>(-0.190568826619798, 0.7325 - 0.07 - 0.060692028477827, 0.1);
        case DRAGLINK_C:
            return ChVector<>(0.6, 0.7325 - 0.2, 0.2);
        case CLAMP_A:
            return ChVector<>(0.044697881113434, 0.3824, 0.102479751287605);
            break;
        case CLAMP_B:
            return ChVector<>(-0.055165072362023, 0.3824, 0.097246155663310);
            break;
        case FRONT_HANGER:
            return ChVector<>(0.494081171752993, 0.3824, 0.1260);
            break;
        case REAR_HANGER:
            return ChVector<>(-0.445529598035440, 0.3824, 0.189525823498473);
            break;
        case SHACKLE:
            return ChVector<>(-0.504548363001581, 0.3824, 0.073694975353985);
            break;
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // end namespace uaz
}  // end namespace vehicle
}  // end namespace chrono
