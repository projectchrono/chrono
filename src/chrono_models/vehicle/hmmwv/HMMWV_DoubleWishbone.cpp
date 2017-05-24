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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// Front and Rear HMMWV suspension subsystems (double A-arm).
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origins at the midpoint between the
// lower control arms' connection points to the chassis.
//
// All point locations are provided for the left half of the supspension.
//
// =============================================================================

#include "chrono_models/vehicle/hmmwv/HMMWV_DoubleWishbone.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lb2kg = 0.453592;
static const double lbf2N = 4.44822162;
static const double lbfpin2Npm = 175.12677;

const double HMMWV_DoubleWishboneFront::m_UCAMass = 5.813;
const double HMMWV_DoubleWishboneFront::m_LCAMass = 23.965;
const double HMMWV_DoubleWishboneFront::m_uprightMass = 19.450;
const double HMMWV_DoubleWishboneFront::m_spindleMass = 14.705;

const double HMMWV_DoubleWishboneFront::m_spindleRadius = 0.10;
const double HMMWV_DoubleWishboneFront::m_spindleWidth = 0.06;
const double HMMWV_DoubleWishboneFront::m_LCARadius = 0.03;
const double HMMWV_DoubleWishboneFront::m_UCARadius = 0.02;
const double HMMWV_DoubleWishboneFront::m_uprightRadius = 0.04;

// TODO: Fix these values
const ChVector<> HMMWV_DoubleWishboneFront::m_spindleInertia(0.04117, 0.07352, 0.04117);
const ChVector<> HMMWV_DoubleWishboneFront::m_UCAInertiaMoments(0.03, 0.03, 0.06276);
const ChVector<> HMMWV_DoubleWishboneFront::m_UCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> HMMWV_DoubleWishboneFront::m_LCAInertiaMoments(0.4, 0.4, 0.8938);
const ChVector<> HMMWV_DoubleWishboneFront::m_LCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> HMMWV_DoubleWishboneFront::m_uprightInertiaMoments(0.1656, 0.1934, 0.04367);
const ChVector<> HMMWV_DoubleWishboneFront::m_uprightInertiaProducts(0.0, 0.0, 0.0);

const double HMMWV_DoubleWishboneFront::m_axleInertia = 0.4;

const double HMMWV_DoubleWishboneFront::m_springCoefficient = 167062.000;
const double HMMWV_DoubleWishboneFront::m_springRestLength = 0.339;

// -----------------------------------------------------------------------------

const double HMMWV_DoubleWishboneRear::m_UCAMass = 5.813;
const double HMMWV_DoubleWishboneRear::m_LCAMass = 23.965;
const double HMMWV_DoubleWishboneRear::m_uprightMass = 19.450;
const double HMMWV_DoubleWishboneRear::m_spindleMass = 14.705;

const double HMMWV_DoubleWishboneRear::m_spindleRadius = 0.10;
const double HMMWV_DoubleWishboneRear::m_spindleWidth = 0.06;
const double HMMWV_DoubleWishboneRear::m_LCARadius = 0.03;
const double HMMWV_DoubleWishboneRear::m_UCARadius = 0.02;
const double HMMWV_DoubleWishboneRear::m_uprightRadius = 0.04;

// TODO: Fix these values
const ChVector<> HMMWV_DoubleWishboneRear::m_spindleInertia(0.04117, 0.07352, 0.04117);
const ChVector<> HMMWV_DoubleWishboneRear::m_UCAInertiaMoments(0.03, 0.03, 0.06276);
const ChVector<> HMMWV_DoubleWishboneRear::m_UCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> HMMWV_DoubleWishboneRear::m_LCAInertiaMoments(0.4, 0.4, 0.8938);
const ChVector<> HMMWV_DoubleWishboneRear::m_LCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> HMMWV_DoubleWishboneRear::m_uprightInertiaMoments(0.1656, 0.1934, 0.04367);
const ChVector<> HMMWV_DoubleWishboneRear::m_uprightInertiaProducts(0.0, 0.0, 0.0);

const double HMMWV_DoubleWishboneRear::m_axleInertia = 0.4;

const double HMMWV_DoubleWishboneRear::m_springCoefficient = 369149.000;
const double HMMWV_DoubleWishboneRear::m_springRestLength = 0.382;

// -----------------------------------------------------------------------------
// HMMWV shock functor class - implements a nonlinear damper
// -----------------------------------------------------------------------------
class HMMWV_ShockForce : public ChLinkSpringCB::ForceFunctor {
  public:
    HMMWV_ShockForce(double midstroke_compression_slope,
                     double midstroke_rebound_slope,
                     double bumpstop_compression_slope,
                     double bumpstop_rebound_slope,
                     double metalmetal_slope,
                     double min_bumpstop_compression_force,
                     double midstroke_lower_bound,
                     double midstroke_upper_bound,
                     double metalmetal_lower_bound,
                     double metalmetal_upper_bound);

    virtual double operator()(double time,
                              double rest_length,
                              double length,
                              double vel,
                              ChLinkSpringCB* link) override;

  private:
    double m_ms_compr;
    double m_ms_rebound;
    double m_bs_compr;
    double m_bs_rebound;
    double m_metal_K;
    double m_F0;
    double m_ms_min_length;
    double m_ms_max_length;
    double m_min_length;
    double m_max_length;
};

HMMWV_ShockForce::HMMWV_ShockForce(double midstroke_compression_slope,
                                   double midstroke_rebound_slope,
                                   double bumpstop_compression_slope,
                                   double bumpstop_rebound_slope,
                                   double metalmetal_slope,
                                   double min_bumpstop_compression_force,
                                   double midstroke_lower_bound,
                                   double midstroke_upper_bound,
                                   double metalmetal_lower_bound,
                                   double metalmetal_upper_bound)
    : m_ms_compr(midstroke_compression_slope),
      m_ms_rebound(midstroke_rebound_slope),
      m_bs_compr(bumpstop_compression_slope),
      m_bs_rebound(bumpstop_rebound_slope),
      m_metal_K(metalmetal_slope),
      m_F0(min_bumpstop_compression_force),
      m_ms_min_length(midstroke_lower_bound),
      m_ms_max_length(midstroke_upper_bound),
      m_min_length(metalmetal_lower_bound),
      m_max_length(metalmetal_upper_bound) {}

double HMMWV_ShockForce::operator()(double time, double rest_length, double length, double vel, ChLinkSpringCB* link) {
    /*
    // On midstroke curve
    if (length >= m_min_length && length <= m_max_length)
      return (vel >= 0) ? -m_ms_rebound * vel : -m_ms_compr * vel;

    // Hydraulic bump engaged
    return (vel >= 0) ? -m_bs_rebound * vel : -m_bs_compr * vel + m_F0;
    */

    double force = 0;

    // Calculate Damping Force
    if (vel >= 0) {
        force = (length >= m_ms_max_length) ? -m_bs_rebound * vel : -m_ms_rebound * vel;
    } else {
        force = (length <= m_ms_min_length) ? -m_bs_compr * vel : -m_ms_compr * vel;
    }

    // Add in Shock metal to metal contact force
    if (length <= m_min_length) {
        force = m_metal_K * (m_min_length - length);
    } else if (length >= m_max_length) {
        force = -m_metal_K * (length - m_max_length);
    }

    return force;
}

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
HMMWV_DoubleWishboneFront::HMMWV_DoubleWishboneFront(const std::string& name) : ChDoubleWishbone(name) {
    m_springForceCB = new LinearSpringForce(m_springCoefficient  // coefficient for linear spring
                                            );

    m_shockForceCB = new HMMWV_ShockForce(lbfpin2Npm * 71.50,   // midstroke_compression_slope
                                          lbfpin2Npm * 128.25,  // midstroke_rebound_slope
                                          lbfpin2Npm * 33.67,   // bumpstop_compression_slope
                                          lbfpin2Npm * 343.00,  // bumpstop_rebound_slope
                                          lbfpin2Npm * 150000,  // metalmetal_slope
                                          lbf2N * 3350,         // min_bumpstop_compression_force
                                          in2m * 13.76,         // midstroke_lower_bound
                                          in2m * 15.85,         // midstroke_upper_bound
                                          in2m * 12.76,         // metalmetal_lower_bound
                                          in2m * 16.48          // metalmetal_upper_boun
                                          );
}

HMMWV_DoubleWishboneRear::HMMWV_DoubleWishboneRear(const std::string& name) : ChDoubleWishbone(name) {
    m_springForceCB = new LinearSpringForce(m_springCoefficient  // coefficient for linear spring
                                            );

    m_shockForceCB = new HMMWV_ShockForce(lbfpin2Npm * 83.00,   // midstroke_compression_slope
                                          lbfpin2Npm * 200.00,  // midstroke_rebound_slope
                                          lbfpin2Npm * 48.75,   // bumpstop_compression_slope
                                          lbfpin2Npm * 365.00,  // bumpstop_rebound_slope
                                          lbfpin2Npm * 150000,  // metalmetal_slope
                                          lbf2N * 3350,         // min_bumpstop_compression_force
                                          in2m * 13.76,         // midstroke_lower_bound
                                          in2m * 15.85,         // midstroke_upper_bound
                                          in2m * 12.76,         // metalmetal_lower_bound
                                          in2m * 16.48          // metalmetal_upper_bound
                                          );
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
HMMWV_DoubleWishboneFront::~HMMWV_DoubleWishboneFront() {
    delete m_springForceCB;
    delete m_shockForceCB;
}

HMMWV_DoubleWishboneRear::~HMMWV_DoubleWishboneRear() {
    delete m_springForceCB;
    delete m_shockForceCB;
}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> HMMWV_DoubleWishboneFront::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return in2m * ChVector<>(-1.59, 35.815, -1.035);
        case UPRIGHT:
            return in2m * ChVector<>(-1.59, 29.5675, -1.0350);
        case UCA_F:
            return in2m * ChVector<>(-1.8864, 17.5575, 9.6308);
        case UCA_B:
            return in2m * ChVector<>(-10.5596, 18.8085, 7.6992);
        case UCA_U:
            return in2m * ChVector<>(-2.088, 28.17, 8.484);
        case UCA_CM:
            return in2m * ChVector<>(-4.155, 23.176, 8.575);
        case LCA_F:
            return in2m * ChVector<>(8.7900, 12.09, 0);
        case LCA_B:
            return in2m * ChVector<>(-8.7900, 12.09, 0);
        case LCA_U:
            return in2m * ChVector<>(-1.40, 30.965, -4.65);
        case LCA_CM:
            return in2m * ChVector<>(0, 21.528, -2.325);
        case SHOCK_C:
            return in2m * ChVector<>(4.095, 19.598, 12.722);
        case SHOCK_A:
            return in2m * ChVector<>(3.827, 21.385, -1.835);
        case SPRING_C:
            return in2m * ChVector<>(4.095, 20.07, 7.775);
        case SPRING_A:
            return in2m * ChVector<>(3.827, 21.385, -1.835);
        case TIEROD_C:
            return in2m * ChVector<>(-9.855, 17.655, 2.135);
        case TIEROD_U:
            return in2m * ChVector<>(-6.922, 32.327, -0.643);
        default:
            return ChVector<>(0, 0, 0);
    }
}

const ChVector<> HMMWV_DoubleWishboneRear::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return in2m * ChVector<>(1.40, 35.815, -1.035);
        case UPRIGHT:
            return in2m * ChVector<>(1.40, 29.5675, -1.035);
        case UCA_F:
            return in2m * ChVector<>(13.7445, 18.1991, 8.9604);
        case UCA_B:
            return in2m * ChVector<>(3.0355, 18.1909, 8.8096);
        case UCA_U:
            return in2m * ChVector<>(1.40, 28.17, 8.5);
        case UCA_CM:
            return in2m * ChVector<>(4.895, 23.183, 8.692);
        case LCA_F:
            return in2m * ChVector<>(8.7900, 12.09, 0);
        case LCA_B:
            return in2m * ChVector<>(-8.7900, 12.09, 0);
        case LCA_U:
            return in2m * ChVector<>(1.40, 30.965, -4.650);
        case LCA_CM:
            return in2m * ChVector<>(0, 21.527, -2.325);
        case SHOCK_C:
            return in2m * ChVector<>(-4.095, 19.598, 12.722);
        case SHOCK_A:
            return in2m * ChVector<>(-3.827, 21.415, -1.511);
        case SPRING_C:
            return in2m * ChVector<>(-4.095, 19.747, 10.098);
        case SPRING_A:
            return in2m * ChVector<>(-3.827, 21.385, -1.835);
        case TIEROD_C:
            return in2m * ChVector<>(8.790, 16.38, 2.310);
        case TIEROD_U:
            return in2m * ChVector<>(6.704, 32.327, -0.365);
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
