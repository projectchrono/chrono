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
// Authors: Radu Serban
// =============================================================================
//
// Base class for an idler subsystem.  An idler consists of the idler wheel and
// a carrier body.  The idler wheel is connected through a revolute joint to the
// carrier body which in turn is connected to the chassis through a prismatic
// joint.  A tensioner force element (TSDA) acts between the chassis and the
// carrier body.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChColorAsset.h"

#include "chrono_vehicle/tracked_vehicle/ChIdler.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChIdler::ChIdler(const std::string& name)
    : m_name(name), m_friction(0.6f), m_restitution(0.1f), m_young_modulus(2e5f), m_poisson_ratio(0.3f) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIdler::SetContactMaterial(float friction_coefficient,
                                 float restitution_coefficient,
                                 float young_modulus,
                                 float poisson_ratio) {
    m_friction = friction_coefficient;
    m_restitution = restitution_coefficient;
    m_young_modulus = young_modulus;
    m_poisson_ratio = poisson_ratio;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIdler::Initialize(std::shared_ptr<ChBodyAuxRef> chassis, const ChVector<>& location) {
    // Express the idler reference frame in the absolute coordinate system.
    ChFrame<> idler_to_abs(location);
    idler_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

    // Transform all points and directions to absolute frame.
    std::vector<ChVector<> > points(NUM_POINTS);

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> rel_pos = GetLocation(static_cast<PointId>(i));
        points[i] = idler_to_abs.TransformPointLocalToParent(rel_pos);
    }

    // Create and initialize the wheel body.
    m_wheel = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_wheel->SetNameString(m_name + "_wheel");
    m_wheel->SetPos(points[WHEEL]);
    m_wheel->SetRot(idler_to_abs.GetRot());
    m_wheel->SetMass(GetWheelMass());
    m_wheel->SetInertiaXX(GetWheelInertia());
    chassis->GetSystem()->AddBody(m_wheel);

    // Create and initialize the carrier body.
    m_carrier = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_carrier->SetNameString(m_name + "_carrier");
    m_carrier->SetPos(points[CARRIER]);
    m_carrier->SetRot(idler_to_abs.GetRot());
    m_carrier->SetMass(GetWheelMass());
    m_carrier->SetInertiaXX(GetWheelInertia());
    AddVisualizationCarrier(m_carrier, points[WHEEL], points[CARRIER], points[CARRIER_CHASSIS]);
    chassis->GetSystem()->AddBody(m_carrier);

    // Create and initialize the revolute joint between wheel and carrier.
    // The axis of rotation is the y axis of the idler reference frame.
    m_revolute = std::make_shared<ChLinkLockRevolute>();
    m_revolute->SetNameString(m_name + "_revolute");
    m_revolute->Initialize(m_carrier, m_wheel,
                           ChCoordsys<>(points[WHEEL], idler_to_abs.GetRot() * Q_from_AngX(CH_C_PI_2)));
    chassis->GetSystem()->AddLink(m_revolute);

    // Create and initialize the prismatic joint between carrier and chassis.
    // The axis of translation is pitched by the specified angle from the x axis
    // of the idler reference frame.
    m_prismatic = std::make_shared<ChLinkLockPrismatic>();
    m_prismatic->SetNameString(m_name + "_prismatic");
    m_prismatic->Initialize(chassis, m_carrier,
                            ChCoordsys<>(points[CARRIER_CHASSIS],
                                         idler_to_abs.GetRot() * Q_from_AngY(CH_C_PI_2 + GetPrismaticPitchAngle())));
    chassis->GetSystem()->AddLink(m_prismatic);

    // Create and initialize the tensioner force element.
    m_tensioner = std::make_shared<ChLinkSpringCB>();
    m_tensioner->SetNameString(m_name + "_tensioner");
    m_tensioner->Initialize(chassis, m_carrier, false, points[TSDA_CHASSIS], points[TSDA_CARRIER]);
    m_tensioner->Set_SpringCallback(GetTensionerForceCallback());
    chassis->GetSystem()->AddLink(m_tensioner);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChIdler::GetMass() const {
    return GetWheelMass() + GetCarrierMass();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIdler::AddVisualizationCarrier(std::shared_ptr<ChBody> carrier,
                                      const ChVector<>& pt_W,
                                      const ChVector<>& pt_C,
                                      const ChVector<>& pt_T) {
    static const double threshold2 = 1e-6;
    double radius = GetCarrierVisRadius();

    // Express hardpoint locations in body frame.
    ChVector<> p_W = carrier->TransformPointParentToLocal(pt_W);
    ChVector<> p_C = carrier->TransformPointParentToLocal(pt_C);
    ChVector<> p_T = carrier->TransformPointParentToLocal(pt_T);

    if ((p_W - p_C).Length2() > threshold2) {
        auto cyl = std::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = p_W;
        cyl->GetCylinderGeometry().p2 = p_C;
        cyl->GetCylinderGeometry().rad = radius;
        carrier->AddAsset(cyl);
    }

    if ((p_C - p_T).Length2() > threshold2) {
        auto cyl = std::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = p_C;
        cyl->GetCylinderGeometry().p2 = p_T;
        cyl->GetCylinderGeometry().rad = radius;
        carrier->AddAsset(cyl);
    }

    auto box = std::make_shared<ChBoxShape>();
    box->GetBoxGeometry().Size = ChVector<>(3 * radius, radius, radius);
    box->GetBoxGeometry().Pos = p_T;
    box->GetBoxGeometry().Rot = ChMatrix33<>(GetPrismaticPitchAngle(), ChVector<>(0, 1, 0));
    carrier->AddAsset(box);

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.2f, 0.2f, 0.6f));
    carrier->AddAsset(col);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIdler::LogConstraintViolations() {
    {
        ChMatrix<>* C = m_revolute->GetC();
        GetLog() << "  Idler-carrier revolute\n";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "  ";
        GetLog() << "  " << C->GetElement(3, 0) << "  ";
        GetLog() << "  " << C->GetElement(4, 0) << "\n";
    }
    {
        ChMatrix<>* C = m_prismatic->GetC();
        GetLog() << "  Carrier-chassis prismatic\n";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "  ";
        GetLog() << "  " << C->GetElement(3, 0) << "  ";
        GetLog() << "  " << C->GetElement(4, 0) << "\n";
    }
}

}  // end namespace vehicle
}  // end namespace chrono
