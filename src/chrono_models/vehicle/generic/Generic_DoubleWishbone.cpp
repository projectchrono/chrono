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
// Authors: Radu Serban, Mike Taylor
// =============================================================================
//
// Generic concrete double wishbone suspension subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origin in the chassis midplane.
//
// All point locations are provided for the left half of the supspension.
//
// =============================================================================

#include <vector>
#include <algorithm>

#include "chrono_models/vehicle/generic/Generic_DoubleWishbone.h"
#include "chrono/core/ChCubicSpline.h"

namespace chrono {
namespace vehicle {
namespace generic {

// -----------------------------------------------------------------------------
// Static variables -- all in SI units
// -----------------------------------------------------------------------------

const double Generic_DoubleWishbone::m_spindleMass = 1.103;
const double Generic_DoubleWishbone::m_uprightMass = 1.397;
const double Generic_DoubleWishbone::m_UCAMass = 1.032;
const double Generic_DoubleWishbone::m_LCAMass = 1.611;

const double Generic_DoubleWishbone::m_spindleRadius = 0.15;
const double Generic_DoubleWishbone::m_spindleWidth = 0.06;
const double Generic_DoubleWishbone::m_uprightRadius = 0.025;
const double Generic_DoubleWishbone::m_UCARadius = 0.02;
const double Generic_DoubleWishbone::m_LCARadius = 0.03;

const ChVector<> Generic_DoubleWishbone::m_spindleInertia(0.000478, 0.000496, 0.000478);
const ChVector<> Generic_DoubleWishbone::m_uprightInertia(0.0138, 0.0146, 0.00283);
const ChVector<> Generic_DoubleWishbone::m_UCAInertia(0.00591, 0.00190, 0.00769);
const ChVector<> Generic_DoubleWishbone::m_LCAInertia(0.0151, 0.0207, 0.0355);

const double Generic_DoubleWishbone::m_axleInertia = 0.4;

const double Generic_DoubleWishbone::m_springCoefficient = 369149.000;
const double Generic_DoubleWishbone::m_dampingCoefficient = 22459.000;
const double Generic_DoubleWishbone::m_springRestLength = 0.306;

// -----------------------------------------------------------------------------

const double Generic_DoubleWishboneFront::m_spindleMass = 1.103;
const double Generic_DoubleWishboneFront::m_uprightMass = 1.397;
const double Generic_DoubleWishboneFront::m_UCAMass = 1.032;
const double Generic_DoubleWishboneFront::m_LCAMass = 1.611;

const double Generic_DoubleWishboneFront::m_spindleRadius = 0.15;
const double Generic_DoubleWishboneFront::m_spindleWidth = 0.06;
const double Generic_DoubleWishboneFront::m_uprightRadius = 0.015;
const double Generic_DoubleWishboneFront::m_UCARadius = 0.015;
const double Generic_DoubleWishboneFront::m_LCARadius = 0.015;

const ChVector<> Generic_DoubleWishboneFront::m_spindleInertia(0.000478, 0.000496, 0.000478);
const ChVector<> Generic_DoubleWishboneFront::m_uprightInertia(0.0138, 0.0146, 0.00283);
const ChVector<> Generic_DoubleWishboneFront::m_UCAInertia(0.00470, 0.00311, 0.00769);
const ChVector<> Generic_DoubleWishboneFront::m_LCAInertia(0.0151, 0.0211, 0.0351);

const double Generic_DoubleWishboneFront::m_axleInertia = 0.0007;

const double Generic_DoubleWishboneFront::m_springCoefficient = 125000;
const double Generic_DoubleWishboneFront::m_springRestLength = (511.4685 + (300 - 256)) / 1000;

// -----------------------------------------------------------------------------

const double Generic_DoubleWishboneRear::m_spindleMass = 1.103;
const double Generic_DoubleWishboneRear::m_uprightMass = 1.329;
const double Generic_DoubleWishboneRear::m_UCAMass = 1.032;
const double Generic_DoubleWishboneRear::m_LCAMass = 1.609;

const double Generic_DoubleWishboneRear::m_spindleRadius = 0.15;
const double Generic_DoubleWishboneRear::m_spindleWidth = 0.06;
const double Generic_DoubleWishboneRear::m_uprightRadius = 0.015;
const double Generic_DoubleWishboneRear::m_UCARadius = 0.015;
const double Generic_DoubleWishboneRear::m_LCARadius = 0.015;

const ChVector<> Generic_DoubleWishboneRear::m_spindleInertia(0.000478, 0.000496, 0.000478);
const ChVector<> Generic_DoubleWishboneRear::m_uprightInertia(0.0115, 0.0118, 0.00327);
const ChVector<> Generic_DoubleWishboneRear::m_UCAInertia(0.00470, 0.00311, 0.00769);
const ChVector<> Generic_DoubleWishboneRear::m_LCAInertia(0.0151, 0.0210, 0.0351);

const double Generic_DoubleWishboneRear::m_axleInertia = 0.0007;

const double Generic_DoubleWishboneRear::m_springCoefficient = 125000;
const double Generic_DoubleWishboneRear::m_springRestLength = (511.4685 + (300 - 240)) / 1000;

// -----------------------------------------------------------------------------
// Generic shock functor class - implements a nonlinear damper
// -----------------------------------------------------------------------------
class Genaric_ShockForce : public ChSpringForceCallback {
  public:
    Genaric_ShockForce(std::vector<double> vel, std::vector<double> frc);

    virtual double operator()(double time, double rest_length, double length, double vel);

  private:
    ChCubicSpline m_ShockTable;
    double m_MaxVel;
    double m_MinVel;
};

Genaric_ShockForce::Genaric_ShockForce(std::vector<double> vel, std::vector<double> frc) : m_ShockTable(vel, frc) {
    m_MaxVel = *std::max_element(std::begin(vel), std::end(vel));
    m_MinVel = *std::min_element(std::begin(vel), std::end(vel));
}

double Genaric_ShockForce::operator()(double time, double rest_length, double length, double vel) {
    double force = 0;
    double dcurve = 0;
    double ddcurve = 0;
    double org_vel = vel;

    if ((vel >= m_MinVel) && (vel <= m_MaxVel)) {
        m_ShockTable.Evaluate(vel, force, dcurve, ddcurve);
    }
    else if ((vel <= m_MinVel)){
        m_ShockTable.Evaluate(m_MinVel, force, dcurve, ddcurve);
        std::cout << "Time: " << time << ", vel: " << vel << ", minVel: " << m_MinVel << ", frc " << force << ", modfrc " << force - dcurve*(m_MinVel - vel) << std::endl;
        force -= dcurve*(m_MinVel - vel);
    }
    else {
        m_ShockTable.Evaluate(m_MaxVel, force, dcurve, ddcurve);
        std::cout << "Time: " << time << ", vel: " << vel << ", maxVel: " << m_MaxVel << ", frc " << force << ", modfrc " << force + dcurve*(m_MaxVel - vel) << std::endl;
        force += dcurve*(m_MaxVel - vel);
    }

    return force;
}

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
Generic_DoubleWishbone::Generic_DoubleWishbone(const std::string& name) : ChDoubleWishbone(name) {
    m_springForceCB = new LinearSpringForce(m_springCoefficient);
    m_shockForceCB = new LinearDamperForce(m_dampingCoefficient);
}

Generic_DoubleWishboneFront::Generic_DoubleWishboneFront(const std::string& name) : ChDoubleWishbone(name) {
    m_springForceCB = new LinearSpringForce(m_springCoefficient);

    std::vector<double> vel({-1.2700, -0.2540, -0.1524, -0.1270, -0.1016, -0.0762, -0.0508, -0.0254, 0.0, 0.0254,
                             0.0508, 0.0762, 0.1016, 0.1270, 0.1524, 0.2540, 1.2700});
    std::vector<double> frc({1495.5, 809.5, 654.8, 587.1, 533.8, 455.5, 370.1, 206.4, 0.0, -462.6, -695.4, -854.0,
                             -966.4, -1085.1, -1171.4, -1423.4, -3218.1});

    m_shockForceCB = new Genaric_ShockForce(vel, frc);
}

Generic_DoubleWishboneRear::Generic_DoubleWishboneRear(const std::string& name) : ChDoubleWishbone(name) {
    m_springForceCB = new LinearSpringForce(m_springCoefficient);

    std::vector<double> vel({-1.2700, -0.2540, -0.1524, -0.1270, -0.1016, -0.0762, -0.0508, -0.0254, 0.0, 0.0254,
                             0.0508, 0.0762, 0.1016, 0.1270, 0.1524, 0.2540, 1.2700});
    std::vector<double> frc({1495.5, 809.5, 654.8, 587.1, 533.8, 455.5, 370.1, 206.4, 0.0, -462.6, -695.4, -854.0,
                             -966.4, -1085.1, -1171.4, -1423.4, -3218.1});

    m_shockForceCB = new Genaric_ShockForce(vel, frc);
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
Generic_DoubleWishbone::~Generic_DoubleWishbone() {
    delete m_springForceCB;
    delete m_shockForceCB;
}

Generic_DoubleWishboneFront::~Generic_DoubleWishboneFront() {
    delete m_springForceCB;
    delete m_shockForceCB;
}

Generic_DoubleWishboneRear::~Generic_DoubleWishboneRear() {
    delete m_springForceCB;
    delete m_shockForceCB;
}

// -----------------------------------------------------------------------------
// Implementation of the getLocation() virtual method.
// This function returns the position of the specified suspension hardpoint,
// with respect to the suspension subsystem's reference frame (a right-hand
// frame with X pointing towards the front, Y to the left, and Z up and with
// its origin and origin in the chassis midplane. The hardpoints returned by this
// function are for the left half of the suspension only.
// -----------------------------------------------------------------------------

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

const ChVector<> Generic_DoubleWishboneFront::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return ChVector<>(-0.4584, 0.7750, -0.1199);  // location of spindle center of mass
        case UPRIGHT:
            return ChVector<>(-0.4808, 0.7270, -0.1118);  // location of upright center of mass
        case UCA_F:
            return ChVector<>(-0.5584, 0.4500, 0.1050);  // UCA front connection point to chassis
        case UCA_B:
            return ChVector<>(-0.7084, 0.4900, 0.1100);  // UCA rear (back) connection point to chassis
        case UCA_U:
            return ChVector<>(-0.4984, 0.6750, 0.1050);  // UCA connection point to upright
        case UCA_CM:
            return ChVector<>(-0.5667, 0.5772, 0.1063);  // location of UCA center of mass
        case LCA_F:
            return ChVector<>(-0.2584, 0.4000, -0.2700);  // LCA front connection point to chassis
        case LCA_B:
            return ChVector<>(-0.6584, 0.4500, -0.2650);  // LCA rear (back) connection point to chassis
        case LCA_U:
            return ChVector<>(-0.4584, 0.7500, -0.3200);  // LCA connection point to upright
        case LCA_CM:
            return ChVector<>(-0.4536, 0.5912, -0.2932);  // location of LCA center of mass
        case SHOCK_C:
            return ChVector<>(-0.4984, 0.5000, 0.2300);  // shock connection to chassis
        case SHOCK_A:
            return ChVector<>(-0.4584, 0.6000, -0.2700);  // shock connection point to LCA
        case SPRING_C:
            return ChVector<>(-0.4984, 0.5000, 0.2300);  // spring connection point to chassis
        case SPRING_A:
            return ChVector<>(-0.4584, 0.6000, -0.2700);  // spring connection point to LCA
        case TIEROD_C:
            return ChVector<>(-0.6584, 0.4000, -0.1200);  // tierod connection point to chassis
        case TIEROD_U:
            return ChVector<>(-0.6084, 0.7500, -0.1200);  // tierod connection point to upright
        default:
            return ChVector<>(0, 0, 0);
    }
}

const ChVector<> Generic_DoubleWishboneRear::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return ChVector<>(0.3595, 0.8120, -0.0999);  // location of spindle center of mass
        case UPRIGHT:
            return ChVector<>(0.3360, 0.7482, -0.0789);  // location of upright center of mass
        case UCA_F:
            return ChVector<>(0.2595, 0.4500, 0.1250);  // UCA front connection point to chassis
        case UCA_B:
            return ChVector<>(0.1095, 0.4900, 0.1300);  // UCA rear (back) connection point to chassis
        case UCA_U:
            return ChVector<>(0.3195, 0.6750, 0.1250);  // UCA connection point to upright
        case UCA_CM:
            return ChVector<>(0.2512, 0.5772, 0.1263);  // location of UCA center of mass
        case LCA_F:
            return ChVector<>(0.5595, 0.4000, -0.2100);  // LCA front connection point to chassis
        case LCA_B:
            return ChVector<>(0.1595, 0.4500, -0.2050);  // LCA rear (back) connection point to chassis
        case LCA_U:
            return ChVector<>(0.3595, 0.7500, -0.2600);  // LCA connection point to upright
        case LCA_CM:
            return ChVector<>(0.3643, 0.5912, -0.2352);  // location of LCA center of mass
        case SHOCK_C:
            return ChVector<>(0.3195, 0.5000, 0.2500);  // shock connection to chassis
        case SHOCK_A:
            return ChVector<>(0.3595, 0.6000, -0.2500);  // shock connection point to LCA
        case SPRING_C:
            return ChVector<>(0.3195, 0.5000, 0.2500);  // spring connection point to chassis
        case SPRING_A:
            return ChVector<>(0.3595, 0.6000, -0.2500);  // spring connection point to LCA
        case TIEROD_C:
            return ChVector<>(0.1595, 0.4000, -0.1000);  // tierod connection point to chassis
        case TIEROD_U:
            return ChVector<>(0.2095, 0.7500, -0.1000);  // tierod connection point to upright
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
