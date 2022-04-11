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
#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono/assets/ChColorAsset.h"

#include "chrono_vehicle/tracked_vehicle/ChIdler.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
ChIdler::ChIdler(const std::string& name) : ChPart(name), m_track(nullptr) {}

ChIdler::~ChIdler() {
    auto sys = m_wheel->GetSystem();
    if (sys) {
        sys->Remove(m_wheel);
        sys->Remove(m_carrier);
        sys->Remove(m_revolute);
        sys->Remove(m_prismatic);
        sys->Remove(m_tensioner);
    }
}

// -----------------------------------------------------------------------------
void ChIdler::Initialize(std::shared_ptr<ChChassis> chassis, const ChVector<>& location, ChTrackAssembly* track) {
    m_parent = chassis;
    m_rel_loc = location;
    m_track = track;

    // Express the idler reference frame in the absolute coordinate system.
    ChFrame<> idler_to_abs(location);
    idler_to_abs.ConcatenatePreTransformation(chassis->GetBody()->GetFrame_REF_to_abs());

    // Transform all points and directions to absolute frame.
    std::vector<ChVector<> > points(NUM_POINTS);

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> rel_pos = GetLocation(static_cast<PointId>(i));
        points[i] = idler_to_abs.TransformPointLocalToParent(rel_pos);
    }

    // Create and initialize the wheel body.
    m_wheel = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_wheel->SetNameString(m_name + "_wheel");
    m_wheel->SetIdentifier(BodyID::IDLER_BODY);
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
    chassis->GetSystem()->AddBody(m_carrier);

    // Cache points for carrier visualization (expressed in the carrier frame)
    m_pW = m_carrier->TransformPointParentToLocal(points[WHEEL]);
    m_pC = m_carrier->TransformPointParentToLocal(points[CARRIER]);
    m_pT = m_carrier->TransformPointParentToLocal(points[CARRIER_CHASSIS]);

    // Create and initialize the revolute joint between wheel and carrier.
    // The axis of rotation is the y axis of the idler reference frame.
    m_revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute->SetNameString(m_name + "_revolute");
    m_revolute->Initialize(m_carrier, m_wheel,
                           ChCoordsys<>(points[WHEEL], idler_to_abs.GetRot() * Q_from_AngX(CH_C_PI_2)));
    chassis->GetSystem()->AddLink(m_revolute);

    // Create and initialize the prismatic joint between carrier and chassis.
    // The axis of translation is pitched by the specified angle from the x axis
    // of the idler reference frame.
    m_prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    m_prismatic->SetNameString(m_name + "_prismatic");
    m_prismatic->Initialize(chassis->GetBody(), m_carrier,
                            ChCoordsys<>(points[CARRIER_CHASSIS],
                                         idler_to_abs.GetRot() * Q_from_AngY(CH_C_PI_2 + GetPrismaticPitchAngle())));
    chassis->GetSystem()->AddLink(m_prismatic);

    // Create and initialize the tensioner force element.
    m_tensioner = chrono_types::make_shared<ChLinkTSDA>();
    m_tensioner->SetNameString(m_name + "_tensioner");
    m_tensioner->Initialize(chassis->GetBody(), m_carrier, false, points[TSDA_CHASSIS], points[TSDA_CARRIER]);
    m_tensioner->RegisterForceFunctor(GetTensionerForceCallback());
    m_tensioner->SetRestLength(GetTensionerFreeLength());
    chassis->GetSystem()->AddLink(m_tensioner);
}

void ChIdler::InitializeInertiaProperties() {
    m_mass = GetWheelMass() + GetCarrierMass();
}

void ChIdler::UpdateInertiaProperties() {
    m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT), m_xform);

    // Calculate COM and inertia expressed in global frame
    utils::CompositeInertia composite;
    composite.AddComponent(m_wheel->GetFrame_COG_to_abs(), m_wheel->GetMass(),
                           m_wheel->GetInertia());
    composite.AddComponent(m_carrier->GetFrame_COG_to_abs(), m_carrier->GetMass(),
                           m_carrier->GetInertia());

    // Express COM and inertia in subsystem reference frame
    m_com.coord.pos = m_xform.TransformPointParentToLocal(composite.GetCOM());
    m_com.coord.rot = QUNIT;

    m_inertia = m_xform.GetA().transpose() * composite.GetInertia() * m_xform.GetA();
}

// -----------------------------------------------------------------------------
void ChIdler::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    static const double threshold2 = 1e-6;
    double radius = GetCarrierVisRadius();

    if ((m_pW - m_pC).Length2() > threshold2) {
        auto cyl = chrono_types::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = m_pW;
        cyl->GetCylinderGeometry().p2 = m_pC;
        cyl->GetCylinderGeometry().rad = radius;
        m_carrier->AddAsset(cyl);
    }

    if ((m_pC - m_pT).Length2() > threshold2) {
        auto cyl = chrono_types::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = m_pC;
        cyl->GetCylinderGeometry().p2 = m_pT;
        cyl->GetCylinderGeometry().rad = radius;
        m_carrier->AddAsset(cyl);
    }

    auto box = chrono_types::make_shared<ChBoxShape>();
    box->GetBoxGeometry().Size = ChVector<>(3 * radius, radius, radius);
    box->Pos = m_pT;
    box->Rot = ChMatrix33<>(GetPrismaticPitchAngle(), ChVector<>(0, 1, 0));
    m_carrier->AddAsset(box);

    auto col = chrono_types::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.2f, 0.2f, 0.6f));
    m_carrier->AddAsset(col);

    // Visualization of the tensioner spring (with default color)
    m_tensioner->AddAsset(chrono_types::make_shared<ChPointPointSpring>(0.06, 150, 15));
}

void ChIdler::RemoveVisualizationAssets() {
    m_carrier->GetAssets().clear();
}

// -----------------------------------------------------------------------------
void ChIdler::LogConstraintViolations() {
    {
        ChVectorDynamic<> C = m_revolute->GetConstraintViolation();
        GetLog() << "  Idler-carrier revolute\n";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "  ";
        GetLog() << "  " << C(4) << "\n";
    }
    {
        ChVectorDynamic<> C = m_prismatic->GetConstraintViolation();
        GetLog() << "  Carrier-chassis prismatic\n";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "  ";
        GetLog() << "  " << C(4) << "\n";
    }
}

// -----------------------------------------------------------------------------
void ChIdler::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_wheel);
    bodies.push_back(m_carrier);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute);
    joints.push_back(m_prismatic);
    ChPart::ExportJointList(jsonDocument, joints);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_tensioner);
    ChPart::ExportLinSpringList(jsonDocument, springs);
}

void ChIdler::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_wheel);
    bodies.push_back(m_carrier);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute);
    joints.push_back(m_prismatic);
    database.WriteJoints(joints);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_tensioner);
    database.WriteLinSprings(springs);
}

}  // end namespace vehicle
}  // end namespace chrono
