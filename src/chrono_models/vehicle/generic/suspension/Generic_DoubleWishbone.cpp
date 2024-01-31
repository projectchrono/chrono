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
// Authors: Radu Serban, Mike Taylor
// =============================================================================
//
// Generic concrete double wishbone suspension subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origin in the chassis midplane.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include <vector>
#include <algorithm>

#include "chrono_models/vehicle/generic/suspension/Generic_DoubleWishbone.h"
#include "chrono/core/ChCubicSpline.h"

namespace chrono {
namespace vehicle {
namespace generic {

// -----------------------------------------------------------------------------
// Static variables -- all in SI units
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lbf2N = 4.44822162;
static const double lbfpin2Npm = 175.12677;

const double Generic_DoubleWishbone::m_spindleMass = 14.705;
const double Generic_DoubleWishbone::m_uprightMass = 19.450;
const double Generic_DoubleWishbone::m_UCAMass = 5.813;
const double Generic_DoubleWishbone::m_LCAMass = 23.965;

const double Generic_DoubleWishbone::m_spindleRadius = 0.10;
const double Generic_DoubleWishbone::m_spindleWidth = 0.06;
const double Generic_DoubleWishbone::m_uprightRadius = 0.04;
const double Generic_DoubleWishbone::m_UCARadius = 0.02;
const double Generic_DoubleWishbone::m_LCARadius = 0.03;

const ChVector<> Generic_DoubleWishbone::m_spindleInertia(0.04117, 0.07352, 0.04117);
const ChVector<> Generic_DoubleWishbone::m_uprightInertiaMoments(0.1656, 0.1934, 0.04367);
const ChVector<> Generic_DoubleWishbone::m_uprightInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> Generic_DoubleWishbone::m_UCAInertiaMoments(0.03, 0.03, 0.06276);
const ChVector<> Generic_DoubleWishbone::m_UCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> Generic_DoubleWishbone::m_LCAInertiaMoments(0.4, 0.4, 0.8938);
const ChVector<> Generic_DoubleWishbone::m_LCAInertiaProducts(0.0, 0.0, 0.0);

const double Generic_DoubleWishbone::m_axleInertia = 0.4;

const double Generic_DoubleWishbone::m_springCoefficient = 369149.000;
const double Generic_DoubleWishbone::m_dampingCoefficient = 22459.000;
const double Generic_DoubleWishbone::m_springRestLength = 0.339;

// -----------------------------------------------------------------------------

const double Generic_DoubleWishboneFront::m_UCAMass = 5.813;
const double Generic_DoubleWishboneFront::m_LCAMass = 23.965;
const double Generic_DoubleWishboneFront::m_uprightMass = 19.450;
const double Generic_DoubleWishboneFront::m_spindleMass = 14.705;
const double Generic_DoubleWishboneFront::m_tierodMass = 6.0;

const double Generic_DoubleWishboneFront::m_spindleRadius = 0.10;
const double Generic_DoubleWishboneFront::m_spindleWidth = 0.06;
const double Generic_DoubleWishboneFront::m_LCARadius = 0.03;
const double Generic_DoubleWishboneFront::m_UCARadius = 0.02;
const double Generic_DoubleWishboneFront::m_uprightRadius = 0.04;
const double Generic_DoubleWishboneFront::m_tierodRadius = 0.02;

// TODO: Fix these values
const ChVector<> Generic_DoubleWishboneFront::m_spindleInertia(0.04117, 0.07352, 0.04117);
const ChVector<> Generic_DoubleWishboneFront::m_UCAInertiaMoments(0.03, 0.03, 0.06276);
const ChVector<> Generic_DoubleWishboneFront::m_UCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> Generic_DoubleWishboneFront::m_LCAInertiaMoments(0.4, 0.4, 0.8938);
const ChVector<> Generic_DoubleWishboneFront::m_LCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> Generic_DoubleWishboneFront::m_uprightInertiaMoments(0.1656, 0.1934, 0.04367);
const ChVector<> Generic_DoubleWishboneFront::m_uprightInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> Generic_DoubleWishboneFront::m_tierodInertia(0.05, 0.05, 0.5);

const double Generic_DoubleWishboneFront::m_axleInertia = 0.4;

const double Generic_DoubleWishboneFront::m_springRestLength = 0.339;

// -----------------------------------------------------------------------------

const double Generic_DoubleWishboneRear::m_UCAMass = 5.813;
const double Generic_DoubleWishboneRear::m_LCAMass = 23.965;
const double Generic_DoubleWishboneRear::m_uprightMass = 19.450;
const double Generic_DoubleWishboneRear::m_spindleMass = 14.705;
const double Generic_DoubleWishboneRear::m_tierodMass = 6.0;

const double Generic_DoubleWishboneRear::m_spindleRadius = 0.10;
const double Generic_DoubleWishboneRear::m_spindleWidth = 0.06;
const double Generic_DoubleWishboneRear::m_LCARadius = 0.03;
const double Generic_DoubleWishboneRear::m_UCARadius = 0.02;
const double Generic_DoubleWishboneRear::m_uprightRadius = 0.04;
const double Generic_DoubleWishboneRear::m_tierodRadius = 0.02;

// TODO: Fix these values
const ChVector<> Generic_DoubleWishboneRear::m_spindleInertia(0.04117, 0.07352, 0.04117);
const ChVector<> Generic_DoubleWishboneRear::m_UCAInertiaMoments(0.03, 0.03, 0.06276);
const ChVector<> Generic_DoubleWishboneRear::m_UCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> Generic_DoubleWishboneRear::m_LCAInertiaMoments(0.4, 0.4, 0.8938);
const ChVector<> Generic_DoubleWishboneRear::m_LCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> Generic_DoubleWishboneRear::m_uprightInertiaMoments(0.1656, 0.1934, 0.04367);
const ChVector<> Generic_DoubleWishboneRear::m_uprightInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> Generic_DoubleWishboneRear::m_tierodInertia(0.05, 0.05, 0.5);

const double Generic_DoubleWishboneRear::m_axleInertia = 0.4;

const double Generic_DoubleWishboneRear::m_springRestLength = 0.382;

// -----------------------------------------------------------------------------
// Generic_DoubleWishbone
// -----------------------------------------------------------------------------

Generic_DoubleWishbone::Generic_DoubleWishbone(const std::string& name) : ChDoubleWishbone(name) {
    m_springForceCB = chrono_types::make_shared<LinearSpringForce>(m_springCoefficient);
    m_shockForceCB = chrono_types::make_shared<LinearDamperForce>(m_dampingCoefficient);
}
Generic_DoubleWishbone::~Generic_DoubleWishbone() {}

const ChVector<> Generic_DoubleWishbone::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return ChVector<>(-0.040, 1.100, -0.026);  // location of spindle center of mass
        case UPRIGHT:
            return ChVector<>(-0.040, 0.880, -0.026);  // location of upright center of mass
        case UCA_F:
            return ChVector<>(0.160, 0.539, 0.243);  // UCA front connection point to chassis
        case UCA_B:
            return ChVector<>(-0.339, 0.587, 0.249);  // UCA rear (back) connection point to chassis
        case UCA_U:
            return ChVector<>(-0.088, 0.808, 0.243);  // UCA connection point to upright
        case UCA_CM:
            return ChVector<>(-0.196, 0.645, 0.245);  // location of UCA center of mass
        case LCA_F:
            return ChVector<>(0.199, 0.479, -0.206);  // LCA front connection point to chassis
        case LCA_B:
            return ChVector<>(-0.279, 0.539, -0.200);  // LCA rear (back) connection point to chassis
        case LCA_U:
            return ChVector<>(-0.040, 0.898, -0.265);  // LCA connection point to upright
        case LCA_CM:
            return ChVector<>(-0.040, 0.639, -0.224);  // location of LCA center of mass
        case SHOCK_C:
            return ChVector<>(-0.088, 0.599, 0.393);  // shock connection to chassis
        case SHOCK_A:
            return ChVector<>(-0.040, 0.718, -0.206);  // shock connection point to LCA
        case SPRING_C:
            return ChVector<>(-0.064, 0.659, 0.094);  // spring connection point to chassis
        case SPRING_A:
            return ChVector<>(-0.040, 0.718, -0.206);  // spring connection point to LCA
        case TIEROD_C:
            return ChVector<>(-0.279, 0.479, -0.026);  // tierod connection point to chassis
        case TIEROD_U:
            return ChVector<>(-0.220, 0.898, -0.026);  // tierod connection point to upright
        default:
            return ChVector<>(0, 0, 0);
    }
}

// =============================================================================================================

// Generic shock functor class - implements a nonlinear damper (with hydraulic bumps)
class Generic_ShockForce : public ChLinkTSDA::ForceFunctor {
  public:
    Generic_ShockForce(double midstroke_compression,  // midstroke compression rate
                       double midstroke_rebound,      // midstroke rebound rate
                       double bumpstop_compression,   // bumpstop compression rate
                       double bumpstop_rebound,       // bumpstop rebound rate
                       double min_length,             // bumpstop engagement min length
                       double max_length              // bumpstop engagement max length
    );

    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;

  private:
    double m_ms_compr;
    double m_ms_rebound;
    double m_bs_compr;
    double m_bs_rebound;
    double m_min_length;
    double m_max_length;
};

Generic_ShockForce::Generic_ShockForce(double midstroke_compression,
                                       double midstroke_rebound,
                                       double bumpstop_compression,
                                       double bumpstop_rebound,
                                       double min_length,
                                       double max_length)
    : m_ms_compr(midstroke_compression),
      m_ms_rebound(midstroke_rebound),
      m_bs_compr(bumpstop_compression),
      m_bs_rebound(bumpstop_rebound),
      m_min_length(min_length),
      m_max_length(max_length) {}

double Generic_ShockForce::evaluate(double time,
                                    double rest_length,
                                    double length,
                                    double vel,
                                    const ChLinkTSDA& link) {
    // On midstroke curve
    if (length >= m_min_length && length <= m_max_length)
        return (vel >= 0) ? -m_ms_rebound * vel : -m_ms_compr * vel;

    // Hydraulic bump engaged
    return (vel >= 0) ? -m_bs_rebound * vel : -m_bs_compr * vel;
}

// -----------------------------------------------------------------------------

Generic_DoubleWishboneFront::Generic_DoubleWishboneFront(const std::string& name, bool use_tierod_bodies)
    : ChDoubleWishbone(name), m_use_tierod_bodies(use_tierod_bodies) {
    // Table-based nonlinear spring
    double def[] = {-0.2, -0.18, -0.16, -0.14, -0.12, -0.1, -0.08, -0.06, -0.04, -0.02, 0,
                    0.02, 0.04,  0.06,  0.08,  0.1,   0.12, 0.14,  0.16,  0.18,  0.2};
    double frc[] = {-322095.536, -240521.166, -174535.686, -122406.996, -82402.997, -52791.592, -31840.681,
                    -17818.165,  -8991.945,   -3629.923,   0,           3629.923,   8991.945,   17818.165,
                    31840.681,   52791.592,   82402.997,   122406.996,  174535.686, 240521.166, 322095.536};
    auto springCB = chrono_types::make_shared<NonlinearSpringForce>(0);
    for (int i = 0; i < 21; i++) {
        springCB->add_pointK(def[i], frc[i]);
    }
    springCB->enable_stops(0.15, 0.3);
    m_springForceCB = springCB;

    // Functor for a piece-wise linear damper (with hydraulic bump stops)
    m_shockForceCB = chrono_types::make_shared<Generic_ShockForce>(lbfpin2Npm * 71.50,   // midstroke compression rate
                                                                   lbfpin2Npm * 128.25,  // midstroke rebound rate
                                                                   lbfpin2Npm * 33.67,   // bumpstop compression rate
                                                                   lbfpin2Npm * 343.00,  // bumpstop rebound rate
                                                                   in2m * 12.76,  // bumpstop engagement min length
                                                                   in2m * 16.48   // bumpstop engagement max length
    );
}

Generic_DoubleWishboneRear::Generic_DoubleWishboneRear(const std::string& name, bool use_tierod_bodies)
    : ChDoubleWishbone(name), m_use_tierod_bodies(use_tierod_bodies) {
    // Table-based nonlinear spring
    double def[] = {-0.2, -0.18, -0.16, -0.14, -0.12, -0.1, -0.08, -0.06, -0.04, -0.02, 0,
                    0.02, 0.04,  0.06,  0.08,  0.1,   0.12, 0.14,  0.16,  0.18,  0.2};
    double frc[] = {-711719.272, -531468.245, -385663.250, -270476.949, -182082.006, -116651.084, -70356.846,
                    -39371.956,  -19869.076,  -8020.869,   0,           8020.869,    19869.076,   39371.956,
                    70356.846,   116651.084,  182082.006,  270476.949,  385663.250,  531468.245,  711719.272};
    auto springCB = chrono_types::make_shared<NonlinearSpringForce>(0);
    for (int i = 0; i < 21; i++) {
        springCB->add_pointK(def[i], frc[i]);
    }
    springCB->enable_stops(0.15, 0.35);
    m_springForceCB = springCB;

    // Functor for a piece-wise linear damper (with hydraulic bump stops)
    m_shockForceCB = chrono_types::make_shared<Generic_ShockForce>(lbfpin2Npm * 83.00,   // midstroke compression rate
                                                                   lbfpin2Npm * 200.00,  // midstroke rebound rate
                                                                   lbfpin2Npm * 48.75,   // bumpstop compression rate
                                                                   lbfpin2Npm * 365.00,  // bumpstop rebound rate
                                                                   in2m * 12.76,  // bumpstop engagement min length
                                                                   in2m * 16.48   // bumpstop engagement max length
    );
}

Generic_DoubleWishboneFront::~Generic_DoubleWishboneFront() {}

Generic_DoubleWishboneRear::~Generic_DoubleWishboneRear() {}

// -----------------------------------------------------------------------------

const ChVector<> Generic_DoubleWishboneFront::getLocation(PointId which) {
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

const ChVector<> Generic_DoubleWishboneRear::getLocation(PointId which) {
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

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
