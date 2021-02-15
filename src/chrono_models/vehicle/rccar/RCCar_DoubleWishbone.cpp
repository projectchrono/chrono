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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz, Jayen Henry
// =============================================================================
//
// Front and Rear RCCar suspension subsystems (double A-arm).
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origins at the midpoint between the
// lower control arms' connection points to the chassis.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "chrono_models/vehicle/rccar/RCCar_DoubleWishbone.h"

namespace chrono {
namespace vehicle {
namespace rccar {

using namespace chrono;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lb2kg = 0.453592;
static const double lbf2N = 4.44822162;
static const double lbfpin2Npm = 175.12677;

const double RCCar_DoubleWishboneFront::m_UCAMass = lb2kg * 0.15;
const double RCCar_DoubleWishboneFront::m_LCAMass = lb2kg * 0.25;
const double RCCar_DoubleWishboneFront::m_uprightMass = lb2kg * 0.1;
const double RCCar_DoubleWishboneFront::m_spindleMass = lb2kg * 0.1;

const double RCCar_DoubleWishboneFront::m_spindleRadius = in2m * 0.2;
const double RCCar_DoubleWishboneFront::m_spindleWidth = in2m * 0.2;
const double RCCar_DoubleWishboneFront::m_LCARadius = in2m * 0.2;
const double RCCar_DoubleWishboneFront::m_UCARadius = in2m * 0.2;
const double RCCar_DoubleWishboneFront::m_uprightRadius = in2m * 0.2;

// TODO: Fix these values
const ChVector<> RCCar_DoubleWishboneFront::m_spindleInertia(0.0000079, 0.0000079, 0.00001172);
const ChVector<> RCCar_DoubleWishboneFront::m_UCAInertiaMoments(0.001136, 0.001136, 0.00001465);
const ChVector<> RCCar_DoubleWishboneFront::m_UCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> RCCar_DoubleWishboneFront::m_LCAInertiaMoments(0.001136, 0.001136, 0.00001465);
const ChVector<> RCCar_DoubleWishboneFront::m_LCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> RCCar_DoubleWishboneFront::m_uprightInertiaMoments(0.00009523, 0.00009523, 0.00001456);
const ChVector<> RCCar_DoubleWishboneFront::m_uprightInertiaProducts(0.0, 0.0, 0.0);

const double RCCar_DoubleWishboneFront::m_axleInertia = 0.00035;

const double RCCar_DoubleWishboneFront::m_springCoefficient = 5000;
const double RCCar_DoubleWishboneFront::m_dampingCoefficient = 450;
const double RCCar_DoubleWishboneFront::m_springRestLength = in2m * 4.95;

// -----------------------------------------------------------------------------

const double RCCar_DoubleWishboneRear::m_UCAMass = lb2kg * 0.15;
const double RCCar_DoubleWishboneRear::m_LCAMass = lb2kg * 0.25;
const double RCCar_DoubleWishboneRear::m_uprightMass = lb2kg * 0.1;
const double RCCar_DoubleWishboneRear::m_spindleMass = lb2kg * 0.1;

const double RCCar_DoubleWishboneRear::m_spindleRadius = in2m * 0.2;
const double RCCar_DoubleWishboneRear::m_spindleWidth = in2m * 0.2;
const double RCCar_DoubleWishboneRear::m_LCARadius = in2m * 0.2;
const double RCCar_DoubleWishboneRear::m_UCARadius = in2m * 0.2;
const double RCCar_DoubleWishboneRear::m_uprightRadius = in2m * 0.2;

// TODO: Fix these values
const ChVector<> RCCar_DoubleWishboneRear::m_spindleInertia(0.0000079, 0.0000079, 0.00001172);
const ChVector<> RCCar_DoubleWishboneRear::m_UCAInertiaMoments(0.001136, 0.001136, 0.00001465);
const ChVector<> RCCar_DoubleWishboneRear::m_UCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> RCCar_DoubleWishboneRear::m_LCAInertiaMoments(0.001136, 0.001136, 0.00001465);
const ChVector<> RCCar_DoubleWishboneRear::m_LCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> RCCar_DoubleWishboneRear::m_uprightInertiaMoments(0.00009523, 0.00009523, 0.00001456);
const ChVector<> RCCar_DoubleWishboneRear::m_uprightInertiaProducts(0.0, 0.0, 0.0);

const double RCCar_DoubleWishboneRear::m_axleInertia = 0.00035;

const double RCCar_DoubleWishboneRear::m_springCoefficient = 5000;
const double RCCar_DoubleWishboneRear::m_dampingCoefficient = 450;
const double RCCar_DoubleWishboneRear::m_springRestLength = in2m * 4.95;

// -----------------------------------------------------------------------------
// RCCar shock functor class - implements a nonlinear damper
// -----------------------------------------------------------------------------
class RCCar_ShockForce : public ChLinkTSDA::ForceFunctor {
  public:
    RCCar_ShockForce(double midstroke_compression_slope,
                     double midstroke_rebound_slope,
                     double bumpstop_compression_slope,
                     double bumpstop_rebound_slope,
                     double metalmetal_slope,
                     double min_bumpstop_compression_force,
                     double midstroke_lower_bound,
                     double midstroke_upper_bound,
                     double metalmetal_lower_bound,
                     double metalmetal_upper_bound);

    virtual double operator()(double time, double rest_length, double length, double vel, ChLinkTSDA* link) override;

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

RCCar_ShockForce::RCCar_ShockForce(double midstroke_compression_slope,
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

double RCCar_ShockForce::operator()(double time, double rest_length, double length, double vel, ChLinkTSDA* link) {
    /*
    // On midstroke curve
    if (length >= m_min_length && length <= m_max_length)
      return (vel >= 0) ? -m_ms_rebound * vel : -m_ms_compr * vel;

    // Hydraulic bump engaged
    return (vel >= 0) ? -m_bs_rebound * vel : -m_bs_compr * vel + m_F0;
    */

    double force = 0;
    // return 0;

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
    // std::cout << "using shock curve\n";
    return force;
}

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
RCCar_DoubleWishboneFront::RCCar_DoubleWishboneFront(const std::string& name) : ChDoubleWishbone(name) {
    m_springForceCB = chrono_types::make_shared<LinearSpringForce>(m_springCoefficient  // coefficient for linear spring
    );
    m_shockForceCB =
        chrono_types::make_shared<LinearDamperForce>(m_dampingCoefficient  // coefficient for linear damping
        );

    // float f = .1;
    // m_shockForceCB = new RCCar_ShockForce(400 * f,     // midstroke_compression_slope
    //                                       500 * f,     // midstroke_rebound_slope
    //                                       5000 * f,    // bumpstop_compression_slope
    //                                       50000 * f,   // bumpstop_rebound_slope
    //                                       150000 * f,  // metalmetal_slope
    //                                       33500 * f,   // min_bumpstop_compression_force
    //                                       in2m * 3.9,  // midstroke_lower_bound
    //                                       in2m * 4.9,  // midstroke_upper_bound
    //                                       in2m * 3.7,  // metalmetal_lower_bound
    //                                       in2m * 5.1   // metalmetal_upper_boun
    // );
}

RCCar_DoubleWishboneRear::RCCar_DoubleWishboneRear(const std::string& name) : ChDoubleWishbone(name) {
    m_springForceCB = chrono_types::make_shared<LinearSpringForce>(m_springCoefficient  // coefficient for linear spring
    );
    m_shockForceCB =
        chrono_types::make_shared<LinearDamperForce>(m_dampingCoefficient  // coefficient for linear damping
        );

    // float f = .1;
    // m_shockForceCB = new RCCar_ShockForce(400 * f,     // midstroke_compression_slope
    //                                       500 * f,     // midstroke_rebound_slope
    //                                       5000 * f,    // bumpstop_compression_slope
    //                                       50000 * f,   // bumpstop_rebound_slope
    //                                       150000 * f,  // metalmetal_slope
    //                                       33500 * f,   // min_bumpstop_compression_force
    //                                       in2m * 3.9,  // midstroke_lower_bound
    //                                       in2m * 4.9,  // midstroke_upper_bound
    //                                       in2m * 3.7,  // metalmetal_lower_bound
    //                                       in2m * 5.1   // metalmetal_upper_boun
    // );
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
RCCar_DoubleWishboneFront::~RCCar_DoubleWishboneFront() {}

RCCar_DoubleWishboneRear::~RCCar_DoubleWishboneRear() {}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> RCCar_DoubleWishboneFront::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return in2m * ChVector<>(0, 6.3, 0);
        case UPRIGHT:
            return in2m * ChVector<>(0.0, 6.0, 0.0);
        case UCA_F:
            return in2m * ChVector<>(0.5, 0.9, 0.6);
        case UCA_B:
            return in2m * ChVector<>(-0.5, 0.9, 0.6);
        case UCA_U:
            return in2m * ChVector<>(0.0, 5.5, 0.6);
        case UCA_CM:
            return in2m * ChVector<>(0.0, 3.1, -0.6);
        case LCA_F:
            return in2m * ChVector<>(0.5, 0.9, -0.6);
        case LCA_B:
            return in2m * ChVector<>(-0.5, 0.9, -0.6);
        case LCA_U:
            return in2m * ChVector<>(0.0, 5.5, -0.6);
        case LCA_CM:
            return in2m * ChVector<>(0.0, 3.1, -0.6);
        case SHOCK_C:
            return in2m * ChVector<>(-0.5, 0.8, 3.0);
        case SHOCK_A:
            return in2m * ChVector<>(-0.5, 3.4, -0.6);
        case SPRING_C:
            return in2m * ChVector<>(-0.5, 1.0, 3.0);
        case SPRING_A:
            return in2m * ChVector<>(-0.5, 3.6, -0.6);
        case TIEROD_C:
            return in2m * ChVector<>(-1, 1.6, 0);
        case TIEROD_U:
            return in2m * ChVector<>(-1, 5.5, 0);
        default:
            return ChVector<>(0, 0, 0);
    }
}

const ChVector<> RCCar_DoubleWishboneRear::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return in2m * ChVector<>(0, 6.3, 0);
        case UPRIGHT:
            return in2m * ChVector<>(0.0, 6.0, 0.0);
        case UCA_F:
            return in2m * ChVector<>(0.5, 1.0, 0.6);
        case UCA_B:
            return in2m * ChVector<>(-0.5, 1.0, 0.6);
        case UCA_U:
            return in2m * ChVector<>(0.0, 5.5, 0.6);
        case UCA_CM:
            return in2m * ChVector<>(0.0, 3.1, -0.6);
        case LCA_F:
            return in2m * ChVector<>(0.5, 1.0, -0.6);
        case LCA_B:
            return in2m * ChVector<>(-0.5, 1.0, -0.6);
        case LCA_U:
            return in2m * ChVector<>(0.0, 5.5, -0.6);
        case LCA_CM:
            return in2m * ChVector<>(0.0, 3.1, -0.6);
        case SHOCK_C:
            return in2m * ChVector<>(-0.5, 0.8, 3.0);
        case SHOCK_A:
            return in2m * ChVector<>(-0.5, 3.4, -0.6);
        case SPRING_C:
            return in2m * ChVector<>(-0.5, 0.8, 3.0);
        case SPRING_A:
            return in2m * ChVector<>(-0.5, 3.4, -0.6);
        case TIEROD_C:
            return in2m * ChVector<>(-1, 1.6, 0);
        case TIEROD_U:
            return in2m * ChVector<>(-1, 5.5, 0);
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // namespace rccar
}  // namespace vehicle
}  // namespace chrono
