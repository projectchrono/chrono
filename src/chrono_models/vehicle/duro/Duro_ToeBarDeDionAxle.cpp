
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
// Authors: Rainer Gericke
// =============================================================================
//
// Front U401 suspension subsystems (toebar pushpipe).
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "chrono_models/vehicle/duro/Duro_ToeBarDeDionAxle.h"

namespace chrono {
namespace vehicle {
namespace duro {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Duro_ToeBarDeDionAxle::m_axleTubeMass = 124.0;
const double Duro_ToeBarDeDionAxle::m_spindleMass = 14.705;
const double Duro_ToeBarDeDionAxle::m_knuckleMass = 10.0;
const double Duro_ToeBarDeDionAxle::m_tierodMass = 5.0;
const double Duro_ToeBarDeDionAxle::m_draglinkMass = 5.0;
const double Duro_ToeBarDeDionAxle::m_wattCenterMass = 5.0;
const double Duro_ToeBarDeDionAxle::m_wattSideMass = 10.0;

const double Duro_ToeBarDeDionAxle::m_axleTubeRadius = 0.0476;
const double Duro_ToeBarDeDionAxle::m_spindleRadius = 0.10;
const double Duro_ToeBarDeDionAxle::m_spindleWidth = 0.06;
const double Duro_ToeBarDeDionAxle::m_knuckleRadius = 0.05;
const double Duro_ToeBarDeDionAxle::m_tierodRadius = 0.02;
const double Duro_ToeBarDeDionAxle::m_draglinkRadius = 0.02;
const double Duro_ToeBarDeDionAxle::m_wattLinkRadius = 0.03;

const ChVector<> Duro_ToeBarDeDionAxle::m_axleTubeInertia(22.21, 0.0775, 22.21);
const ChVector<> Duro_ToeBarDeDionAxle::m_spindleInertia(0.04117, 0.07352, 0.04117);
const ChVector<> Duro_ToeBarDeDionAxle::m_knuckleInertia(0.1, 0.1, 0.1);
const ChVector<> Duro_ToeBarDeDionAxle::m_tierodInertia(1.0, 0.1, 1.0);
const ChVector<> Duro_ToeBarDeDionAxle::m_draglinkInertia(0.1, 1.0, 0.1);
const ChVector<> Duro_ToeBarDeDionAxle::m_wattCenterInertia(0.05, 0.01, 0.05);
const ChVector<> Duro_ToeBarDeDionAxle::m_wattSideInertia(0.07, 0.01, 0.07);

const double Duro_ToeBarDeDionAxle::m_springDesignLength = 0.3;
const double Duro_ToeBarDeDionAxle::m_springCoefficient = 166283.0949;
const double Duro_ToeBarDeDionAxle::m_springRestLength = m_springDesignLength + 0.0621225507207084;
const double Duro_ToeBarDeDionAxle::m_springMinLength = m_springDesignLength - 0.08;
const double Duro_ToeBarDeDionAxle::m_springMaxLength = m_springDesignLength + 0.08;
const double Duro_ToeBarDeDionAxle::m_damperCoefficient = 20792.69215;
const double Duro_ToeBarDeDionAxle::m_damperDegressivityCompression = 3.0;
const double Duro_ToeBarDeDionAxle::m_damperDegressivityExpansion = 1.0;
const double Duro_ToeBarDeDionAxle::m_axleShaftInertia = 0.4;

// ---------------------------------------------------------------------------------------
// U401 spring functor class - implements a linear spring + bump stop +
// rebound stop
// ---------------------------------------------------------------------------------------
class Duro_FPPSpringForceFront : public ChLinkTSDA::ForceFunctor {
  public:
    Duro_FPPSpringForceFront(double spring_constant, double min_length, double max_length);

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

Duro_FPPSpringForceFront::Duro_FPPSpringForceFront(double spring_constant, double min_length, double max_length)
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

double Duro_FPPSpringForceFront::evaluate(double time,
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
// U401 shock functor class - implements a nonlinear damper
// -----------------------------------------------------------------------------
class Duro_FPPShockForceFront : public ChLinkTSDA::ForceFunctor {
  public:
    Duro_FPPShockForceFront(double compression_slope,
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

Duro_FPPShockForceFront::Duro_FPPShockForceFront(double compression_slope,
                                                 double compression_degressivity,
                                                 double expansion_slope,
                                                 double expansion_degressivity)
    : m_slope_compr(compression_slope),
      m_degres_compr(compression_degressivity),
      m_slope_expand(expansion_slope),
      m_degres_expand(expansion_degressivity) {}

double Duro_FPPShockForceFront::evaluate(double time,
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

Duro_ToeBarDeDionAxle::Duro_ToeBarDeDionAxle(const std::string& name) : ChToeBarDeDionAxle(name) {
    m_springForceCB =
        chrono_types::make_shared<Duro_FPPSpringForceFront>(m_springCoefficient, m_springMinLength, m_springMaxLength);

    m_shockForceCB = chrono_types::make_shared<Duro_FPPShockForceFront>(
        m_damperCoefficient, m_damperDegressivityCompression, m_damperCoefficient, m_damperDegressivityExpansion);
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
Duro_ToeBarDeDionAxle::~Duro_ToeBarDeDionAxle() {}

const ChVector<> Duro_ToeBarDeDionAxle::getLocation(PointId which) {
    switch (which) {
        case SPRING_A:
            return ChVector<>(0.03, 0.55, 0.068);
        case SPRING_C:
            return ChVector<>(0.01, 0.52, 0.368);
        case SHOCK_A:
            return ChVector<>(-0.1, 0.55, 0.124);
        case SHOCK_C:
            return ChVector<>(-0.12, 0.52, 0.468);
        case SPINDLE:
            return ChVector<>(0.0, 0.84, 0.0);  // 0.84 - 0.635 = 0.205
        case KNUCKLE_CM:
            return ChVector<>(0.0, 0.635 - 0.07 + 0.155, 0.0);
        case KNUCKLE_L:
            return ChVector<>(0.0, 0.635 - 0.07 + 0.0098058067569092 + 0.155, -0.1);
        case KNUCKLE_U:
            return ChVector<>(0.0, 0.635 - 0.07 - 0.0098058067569092 + 0.155, 0.1);
        case KNUCKLE_DRL:
            return ChVector<>(0.22, -(0.635 - 0.2 + 0.155), 0.1);
        case TIEROD_K:
            return ChVector<>(-0.190568826619798, (0.635 - 0.07 - 0.060692028477827 + 0.155), 0.1);
        case DRAGLINK_C:
            return ChVector<>(0.22, 0.635 - 0.2 + 0.155, 0.1);
        case AXLE_C:
            return ChVector<>(-1.675, 0.0, 0.1);
        case STABI_CON:
            return ChVector<>(-1.675, 0.42, 0.1);
        case WATT_CNT_LE:
            return ChVector<>(0.15, 0, 0.25);
        case WATT_CNT_RI:
            return ChVector<>(0.15, 0.0, 0.05);
        case WATT_LE_CH:
            return ChVector<>(0.15, -0.44, 0.25);
        case WATT_RI_CH:
            return ChVector<>(0.15, 0.44, 0.05);
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // namespace duro
}  // end namespace vehicle
}  // end namespace chrono
