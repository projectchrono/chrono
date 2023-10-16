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
// Front and Rear ARTcar suspension subsystems (double A-arm).
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origins at the midpoint between the
// lower control arms' connection points to the chassis.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "chrono_models/vehicle/artcar/ARTcar_DoubleWishbone.h"

namespace chrono {
namespace vehicle {
namespace artcar {

using namespace chrono;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lb2kg = 0.453592;

const double ARTcar_DoubleWishboneFront::m_UCAMass = 0.01583115;
const double ARTcar_DoubleWishboneFront::m_LCAMass = 0.08629433;
const double ARTcar_DoubleWishboneFront::m_uprightMass = 0.08666171;
const double ARTcar_DoubleWishboneFront::m_spindleMass = 0.0137509;

const double ARTcar_DoubleWishboneFront::m_spindleRadius = 0.005;
const double ARTcar_DoubleWishboneFront::m_spindleWidth = 0.01;
const double ARTcar_DoubleWishboneFront::m_LCARadius = 0.005;
const double ARTcar_DoubleWishboneFront::m_UCARadius = 0.005;
const double ARTcar_DoubleWishboneFront::m_uprightRadius = 0.005;

const ChVector<> ARTcar_DoubleWishboneFront::m_spindleInertia(0.00000184, 0.00000032, 0.00000184);
const ChVector<> ARTcar_DoubleWishboneFront::m_UCAInertiaMoments(0.00002208, 0.00000258, 0.00002352);
const ChVector<> ARTcar_DoubleWishboneFront::m_UCAInertiaProducts(0.00000216, 0.00000033, 0.00000324);
const ChVector<> ARTcar_DoubleWishboneFront::m_LCAInertiaMoments(0.00012785, 0.00002197, 0.00014179);
const ChVector<> ARTcar_DoubleWishboneFront::m_LCAInertiaProducts(-0.00000886, -0.00000064, 0.00001841);
const ChVector<> ARTcar_DoubleWishboneFront::m_uprightInertiaMoments(0.00001691, 0.00001433, 0.00001402);
const ChVector<> ARTcar_DoubleWishboneFront::m_uprightInertiaProducts(-0.00000039, -0.00000008, -0.00000090);

const double ARTcar_DoubleWishboneFront::m_axleInertia = 0.00000016;

const double ARTcar_DoubleWishboneFront::m_springCoefficient = 8000;
const double ARTcar_DoubleWishboneFront::m_dampingCoefficient = 450;
const double ARTcar_DoubleWishboneFront::m_springRestLength = .1363;

// -----------------------------------------------------------------------------

const double ARTcar_DoubleWishboneRear::m_UCAMass = 0.01583115;
const double ARTcar_DoubleWishboneRear::m_LCAMass = 0.08622608;
const double ARTcar_DoubleWishboneRear::m_uprightMass = 0.07137211;
const double ARTcar_DoubleWishboneRear::m_spindleMass = 0.01375090;

const double ARTcar_DoubleWishboneRear::m_spindleRadius = 0.005;
const double ARTcar_DoubleWishboneRear::m_spindleWidth = 0.01;
const double ARTcar_DoubleWishboneRear::m_LCARadius = 0.005;
const double ARTcar_DoubleWishboneRear::m_UCARadius = 0.005;
const double ARTcar_DoubleWishboneRear::m_uprightRadius = 0.005;

const ChVector<> ARTcar_DoubleWishboneRear::m_spindleInertia(0.00000184, 0.00000032, 0.00000184);
const ChVector<> ARTcar_DoubleWishboneRear::m_UCAInertiaMoments(0.00002208, 0.00000252, 0.00002358);
const ChVector<> ARTcar_DoubleWishboneRear::m_UCAInertiaProducts(-0.00000217, -0.00000031, 0.00000305);
const ChVector<> ARTcar_DoubleWishboneRear::m_LCAInertiaMoments(0.00012731, 0.00002163, 0.00014159);
const ChVector<> ARTcar_DoubleWishboneRear::m_LCAInertiaProducts(0.00000893, 0.00000057, 0.00002163);
const ChVector<> ARTcar_DoubleWishboneRear::m_uprightInertiaMoments(0.00001507, 0.00001253, 0.00001141);
const ChVector<> ARTcar_DoubleWishboneRear::m_uprightInertiaProducts(0.0, 0.0, -0.00000117);

const double ARTcar_DoubleWishboneRear::m_axleInertia = 0.00000016;

const double ARTcar_DoubleWishboneRear::m_springCoefficient = 8000;  // TODO
const double ARTcar_DoubleWishboneRear::m_dampingCoefficient = 450;  // TODO
const double ARTcar_DoubleWishboneRear::m_springRestLength = .1363;  // TODO

// -----------------------------------------------------------------------------
// ARTcar shock functor class - implements a nonlinear damper
// -----------------------------------------------------------------------------
class ARTcar_ShockForce : public ChLinkTSDA::ForceFunctor {
  public:
    ARTcar_ShockForce(double midstroke_compression_slope,
                     double midstroke_rebound_slope,
                     double bumpstop_compression_slope,
                     double bumpstop_rebound_slope,
                     double metalmetal_slope,
                     double min_bumpstop_compression_force,
                     double midstroke_lower_bound,
                     double midstroke_upper_bound,
                     double metalmetal_lower_bound,
                     double metalmetal_upper_bound);

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
    double m_metal_K;
    ////double m_F0;
    double m_ms_min_length;
    double m_ms_max_length;
    double m_min_length;
    double m_max_length;
};

ARTcar_ShockForce::ARTcar_ShockForce(double midstroke_compression_slope,
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
      ////m_F0(min_bumpstop_compression_force),
      m_ms_min_length(midstroke_lower_bound),
      m_ms_max_length(midstroke_upper_bound),
      m_min_length(metalmetal_lower_bound),
      m_max_length(metalmetal_upper_bound) {}

double ARTcar_ShockForce::evaluate(double time, double rest_length, double length, double vel, const ChLinkTSDA& link) {
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
ARTcar_DoubleWishboneFront::ARTcar_DoubleWishboneFront(const std::string& name) : ChDoubleWishbone(name) {
    m_springForceCB = chrono_types::make_shared<LinearSpringForce>(m_springCoefficient  // coefficient for linear spring
    );
    m_shockForceCB =
        chrono_types::make_shared<LinearDamperForce>(m_dampingCoefficient  // coefficient for linear damping
        );

    // float f = .1;
    // m_shockForceCB = new ARTcar_ShockForce(400 * f,     // midstroke_compression_slope
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

ARTcar_DoubleWishboneRear::ARTcar_DoubleWishboneRear(const std::string& name) : ChDoubleWishbone(name) {
    m_springForceCB = chrono_types::make_shared<LinearSpringForce>(m_springCoefficient  // coefficient for linear spring
    );
    m_shockForceCB =
        chrono_types::make_shared<LinearDamperForce>(m_dampingCoefficient  // coefficient for linear damping
        );

    // float f = .1;
    // m_shockForceCB = new ARTcar_ShockForce(400 * f,     // midstroke_compression_slope
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
ARTcar_DoubleWishboneFront::~ARTcar_DoubleWishboneFront() {}

ARTcar_DoubleWishboneRear::~ARTcar_DoubleWishboneRear() {}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> ARTcar_DoubleWishboneFront::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return ChVector<>(.1426, .16599, -.03725);
        case UPRIGHT:
            return ChVector<>(.1426, .162, -.03725);
        case UCA_F:
            return ChVector<>(.17134, .0235, 0);
        case UCA_B:
            return ChVector<>(.12334, .0235, 0);
        case UCA_U:
            return ChVector<>(.14424, .15348, -.01169);
        case UCA_CM:
            return ChVector<>(.14284, .08306, -.00536);
        case LCA_F:
            return ChVector<>(.16547, .01998, -.039);
        case LCA_B:
            return ChVector<>(.10948, .01998, -.039);
        case LCA_U:
            return ChVector<>(.14224, .15744, -.05599);
        case LCA_CM:
            return ChVector<>(.14318, .07774, -.04044);
        case SHOCK_C:
            return ChVector<>(.1421, .03108, 0.07157);
        case SHOCK_A:
            return ChVector<>(.15318, .098, -.043);
        case SPRING_C:
            return ChVector<>(.1421, .03108, 0.07157);
        case SPRING_A:
            return ChVector<>(.15318, .098, -.043);
        case TIEROD_C:
            return ChVector<>(.1155, .033, -.027);
        case TIEROD_U:
            return ChVector<>(.11716, .14677, -.041);
        default:
            return ChVector<>(0, 0, 0);
    }
}

const ChVector<> ARTcar_DoubleWishboneRear::getLocation(PointId which) {
    const double front_to_rear = -(.321 + .14424);
    switch (which) {
        case SPINDLE:
            return ChVector<>(.1426 + front_to_rear, .16599, -.03725);
        case UPRIGHT:
            return ChVector<>(.1426 + front_to_rear, .162, -.03725);
        case UCA_F:
            return ChVector<>(.17134 + front_to_rear, .0235, 0);
        case UCA_B:
            return ChVector<>(.12334 + front_to_rear, .0235, 0);
        case UCA_U:
            return ChVector<>(.14424 + front_to_rear, .15348, -.01169);
        case UCA_CM:
            return ChVector<>(.14284 + front_to_rear, .08306, -.00536);
        case LCA_F:
            return ChVector<>(.16547 + front_to_rear, .01998, -.039);
        case LCA_B:
            return ChVector<>(.10948 + front_to_rear, .01998, -.039);
        case LCA_U:
            return ChVector<>(.14224 + front_to_rear, .15744, -.05599);
        case LCA_CM:
            return ChVector<>(.14318 + front_to_rear, .07774, -.04044);
        case SHOCK_C:
            return ChVector<>(.1421 + front_to_rear, .03108, 0.07157);
        case SHOCK_A:
            return ChVector<>(.15318 + front_to_rear, .098, -.043);
        case SPRING_C:
            return ChVector<>(.1421 + front_to_rear, .03108, 0.07157);
        case SPRING_A:
            return ChVector<>(.15318 + front_to_rear, .098, -.043);
        case TIEROD_C:
            return ChVector<>(.1155 + front_to_rear, .033, -.027);
        case TIEROD_U:
            return ChVector<>(.11716 + front_to_rear, .14677, -.041);
        default:
            return ChVector<>(0, 0, 0);
    }
    // switch (which) {
    //     case SPINDLE:
    //         return ChVector<>(-.3586, .1656, -.0731);
    //     case UPRIGHT:
    //         return ChVector<>(-.3586, .1556, -.0731);
    //     case UCA_F:
    //         return ChVector<>(-0.343, .023, -.0278);
    //     case UCA_B:
    //         return ChVector<>(-0.391, .023, -.0278);
    //     case UCA_U:
    //         return ChVector<>(-.3522, .1591, -.0472); //x=-.321
    //     case UCA_CM:
    //         return ChVector<>(-.3567, .0817, -.0361);
    //     case LCA_F:
    //         return ChVector<>(-.3274, .0217, -.0668);
    //     case LCA_B:
    //         return ChVector<>(-.3785, .0217, -.0668);
    //     case LCA_U:
    //         return ChVector<>(-.3538, .1591, -.0916);
    //     case LCA_CM:
    //         return ChVector<>(-.3611, .0771, -.0739);
    //     case SHOCK_C:
    //         return ChVector<>(-.3752, .0311, .0448);
    //     case SHOCK_A:
    //         return ChVector<>(-.3752, .0858, -.069);
    //     case SPRING_C:
    //         return ChVector<>(-.3752, .0311, .0448);
    //     case SPRING_A:
    //         return ChVector<>(-.3752, .0858, -.069);
    //     case TIEROD_C:
    //         return ChVector<>(-.3924, .0217, -.0668);
    //     case TIEROD_U:
    //         return ChVector<>(-.3798, .1591, -.0916);
    //     default:
    //         return ChVector<>(0, 0, 0);
    // }
}

}  // namespace artcar
}  // namespace vehicle
}  // namespace chrono
