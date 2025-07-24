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
// Authors: Radu Serban, Mike Taylor, Asher Elmquist
// =============================================================================
//
// Sedan concrete double wishbone suspension subsystem.
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

#include "chrono_models/vehicle/sedan/Sedan_DoubleWishbone.h"
#include "chrono/core/ChCubicSpline.h"

namespace chrono {
namespace vehicle {
namespace sedan {

// -----------------------------------------------------------------------------
// Static variables -- all in SI units
// -----------------------------------------------------------------------------

const double Sedan_DoubleWishbone::m_spindleMass = 1.103;
const double Sedan_DoubleWishbone::m_uprightMass = 1.397;
const double Sedan_DoubleWishbone::m_UCAMass = 1.032;
const double Sedan_DoubleWishbone::m_LCAMass = 1.611;

const double Sedan_DoubleWishbone::m_spindleRadius = 0.1;
const double Sedan_DoubleWishbone::m_spindleWidth = 0.02;
const double Sedan_DoubleWishbone::m_uprightRadius = 0.025;
const double Sedan_DoubleWishbone::m_UCARadius = 0.02;
const double Sedan_DoubleWishbone::m_LCARadius = 0.03;

const ChVector3d Sedan_DoubleWishbone::m_spindleInertia(0.000478, 0.000496, 0.000478);
const ChVector3d Sedan_DoubleWishbone::m_uprightInertiaMoments(0.0138, 0.0146, 0.00283);
const ChVector3d Sedan_DoubleWishbone::m_uprightInertiaProducts(0.0, 0.0, 0.0);
const ChVector3d Sedan_DoubleWishbone::m_UCAInertiaMoments(0.00591, 0.00190, 0.00769);
const ChVector3d Sedan_DoubleWishbone::m_UCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector3d Sedan_DoubleWishbone::m_LCAInertiaMoments(0.0151, 0.0207, 0.0355);
const ChVector3d Sedan_DoubleWishbone::m_LCAInertiaProducts(0.0, 0.0, 0.0);

const double Sedan_DoubleWishbone::m_axleInertia = 0.4;

const double Sedan_DoubleWishbone::m_springCoefficient = 73574.10163;
const double Sedan_DoubleWishbone::m_dampingCoefficient = 15054.53731;
const double Sedan_DoubleWishbone::m_springRestLength = 0.511468474;
const double Sedan_DoubleWishbone::m_springPreload = 7492.646764;

// -----------------------------------------------------------------------------
// Sedan shock functor class - implements a nonlinear damper
// -----------------------------------------------------------------------------
class Sedan_ShockForce : public ChLinkTSDA::ForceFunctor {
  public:
    Sedan_ShockForce(std::vector<double> vel, std::vector<double> frc);

    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;

  private:
    ChCubicSpline m_ShockTable;
    double m_MaxVel;
    double m_MinVel;
};

Sedan_ShockForce::Sedan_ShockForce(std::vector<double> vel, std::vector<double> frc) : m_ShockTable(vel, frc) {
    m_MaxVel = *std::max_element(std::begin(vel), std::end(vel));
    m_MinVel = *std::min_element(std::begin(vel), std::end(vel));
}

double Sedan_ShockForce::evaluate(double time, double rest_length, double length, double vel, const ChLinkTSDA& link) {
    double force = 0;
    double dcurve = 0;
    double ddcurve = 0;

    if ((vel >= m_MinVel) && (vel <= m_MaxVel)) {
        m_ShockTable.Evaluate(vel, force, dcurve, ddcurve);
    } else if ((vel <= m_MinVel)) {
        m_ShockTable.Evaluate(m_MinVel, force, dcurve, ddcurve);
        ////std::cout << "Time: " << time << ", vel: " << vel << ", minVel: " << m_MinVel << ", frc " << force << ",
        ///modfrc " << force - dcurve*(m_MinVel - vel) << std::endl;
        force -= dcurve * (m_MinVel - vel);
    } else {
        m_ShockTable.Evaluate(m_MaxVel, force, dcurve, ddcurve);
        ////std::cout << "Time: " << time << ", vel: " << vel << ", maxVel: " << m_MaxVel << ", frc " << force << ",
        ///modfrc " << force + dcurve*(m_MaxVel - vel) << std::endl;
        force += dcurve * (m_MaxVel - vel);
    }

    return force;
}

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
Sedan_DoubleWishbone::Sedan_DoubleWishbone(const std::string& name) : ChDoubleWishbone(name) {
    m_springForceCB = chrono_types::make_shared<utils::LinearSpringForce>(m_springCoefficient, m_springPreload);
    auto sptr = std::static_pointer_cast<utils::LinearSpringForce>(m_springForceCB);
    sptr->enable_stops(m_springRestLength - 0.04, m_springRestLength + 0.04);
    sptr->set_stops(2.0 * m_springCoefficient, 2.0 * m_springCoefficient);
    m_shockForceCB = chrono_types::make_shared<utils::LinearDamperForce>(m_dampingCoefficient);
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
Sedan_DoubleWishbone::~Sedan_DoubleWishbone() {}

// -----------------------------------------------------------------------------
// Implementation of the getLocation() virtual method.
// This function returns the position of the specified suspension hardpoint,
// with respect to the suspension subsystem's reference frame (a right-hand
// frame with X pointing towards the front, Y to the left, and Z up and with
// its origin and origin in the chassis midplane. The hardpoints returned by this
// function are for the left half of the suspension only.
// -----------------------------------------------------------------------------

const ChVector3d Sedan_DoubleWishbone::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return ChVector3d(-0.4584 + 0.4584, 0.7979, -0.1199);  // location of spindle center of mass
        case UPRIGHT:
            return ChVector3d(-0.4808 + 0.4584, 0.7470, -0.1118);  // location of upright center of mass
        case UCA_F:
            return ChVector3d(-0.5584 + 0.4584, 0.4700, 0.1050);  // UCA front connection point to chassis
        case UCA_B:
            return ChVector3d(-0.7084 + 0.4584, 0.5100, 0.1100);  // UCA rear (back) connection point to chassis
        case UCA_U:
            return ChVector3d(-0.4984 + 0.4584, 0.6950, 0.1050);  // UCA connection point to upright
        case UCA_CM:
            return ChVector3d(-0.5667 + 0.4584, 0.5972, 0.1063);  // location of UCA center of mass
        case LCA_F:
            return ChVector3d(-0.2584 + 0.4584, 0.4200, -0.2700);  // LCA front connection point to chassis
        case LCA_B:
            return ChVector3d(-0.6584 + 0.4584, 0.4700, -0.2650);  // LCA rear (back) connection point to chassis
        case LCA_U:
            return ChVector3d(-0.4584 + 0.4584, 0.7700, -0.3200);  // LCA connection point to upright
        case LCA_CM:
            return ChVector3d(-0.4536 + 0.4584, 0.6112, -0.2932);  // location of LCA center of mass
        case SHOCK_C:
            return ChVector3d(-0.4984 + 0.4584, 0.5200, 0.2300);  // shock connection to chassis
        case SHOCK_A:
            return ChVector3d(-0.4584 + 0.4584, 0.6200, -0.2700);  // shock connection point to LCA
        case SPRING_C:
            return ChVector3d(-0.4984 + 0.4584, 0.5200, 0.2300);  // spring connection point to chassis
        case SPRING_A:
            return ChVector3d(-0.4584 + 0.4584, 0.6200, -0.2700);  // spring connection point to LCA
        case TIEROD_C:
            return ChVector3d(-0.6584 + 0.4584, 0.4200, -0.1200);  // tierod connection point to chassis
        case TIEROD_U:
            return ChVector3d(-0.6084 + 0.4584, 0.7700, -0.1200);  // tierod connection point to upright
        default:
            return ChVector3d(0, 0, 0);
    }
}

}  // end namespace sedan
}  // end namespace vehicle
}  // end namespace chrono
