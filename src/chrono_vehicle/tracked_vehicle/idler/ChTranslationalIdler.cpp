// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
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
// Base class for an idler subsystem with a tensioner mechanism using a
// translational joint and TSDA.  An idler consists of the idler wheel and
// a connecting body.  The idler wheel is connected through a revolute joint to
// the connecting body which in turn is connected to the chassis through a
// translational joint. A linear actuator acts as a tensioner.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChPointPointShape.h"

#include "chrono_vehicle/tracked_vehicle/idler/ChTranslationalIdler.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
ChTranslationalIdler::ChTranslationalIdler(const std::string& name) : ChIdler(name) {}

ChTranslationalIdler::~ChTranslationalIdler() {
    auto sys = m_carrier->GetSystem();
    if (sys) {
        sys->Remove(m_carrier);
        sys->Remove(m_prismatic);
        sys->Remove(m_tensioner);
    }
}

// -----------------------------------------------------------------------------
void ChTranslationalIdler::Initialize(std::shared_ptr<ChChassis> chassis,
                                      const ChVector<>& location,
                                      ChTrackAssembly* track) {
    // Express the idler reference frame in the absolute coordinate system.
    ChFrame<> idler_to_abs(location);
    idler_to_abs.ConcatenatePreTransformation(chassis->GetBody()->GetFrame_REF_to_abs());

    // Transform all points and directions to absolute frame.
    std::vector<ChVector<> > points(NUM_POINTS);

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> rel_pos = GetLocation(static_cast<PointId>(i));
        points[i] = idler_to_abs.TransformPointLocalToParent(rel_pos);
    }

    // Create and initialize the carrier body.
    m_carrier = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_carrier->SetNameString(m_name + "_carrier");
    m_carrier->SetPos(points[CARRIER]);
    m_carrier->SetRot(idler_to_abs.GetRot());
    m_carrier->SetMass(GetCarrierMass());
    m_carrier->SetInertiaXX(GetCarrierInertia());
    chassis->GetSystem()->AddBody(m_carrier);

    // Cache points for carrier visualization (expressed in the carrier frame)
    m_pC = m_carrier->TransformPointParentToLocal(points[CARRIER]);
    m_pW = m_carrier->TransformPointParentToLocal(points[CARRIER_WHEEL]);
    m_pT = m_carrier->TransformPointParentToLocal(points[CARRIER_CHASSIS]);

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

    // Invoke the base class implementation. This initializes the associated idler wheel.
    // Note: we must call this here, after the m_carrier body is created.
    ChIdler::Initialize(chassis, location, track);
}

void ChTranslationalIdler::InitializeInertiaProperties() {
    m_mass = GetCarrierMass() + m_idler_wheel->GetMass();
}

void ChTranslationalIdler::UpdateInertiaProperties() {
    m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT), m_xform);

    // Calculate COM and inertia expressed in global frame
    utils::CompositeInertia composite;
    composite.AddComponent(m_carrier->GetFrame_COG_to_abs(), m_carrier->GetMass(), m_carrier->GetInertia());
    composite.AddComponent(m_idler_wheel->GetBody()->GetFrame_COG_to_abs(), m_idler_wheel->GetBody()->GetMass(),
                           m_idler_wheel->GetBody()->GetInertia());

    // Express COM and inertia in subsystem reference frame
    m_com.coord.pos = m_xform.TransformPointParentToLocal(composite.GetCOM());
    m_com.coord.rot = QUNIT;

    m_inertia = m_xform.GetA().transpose() * composite.GetInertia() * m_xform.GetA();
}

// -----------------------------------------------------------------------------
void ChTranslationalIdler::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    static const double threshold2 = 1e-6;
    double radius = GetCarrierVisRadius();

    if ((m_pW - m_pC).Length2() > threshold2) {
        auto cyl = chrono_types::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = m_pW;
        cyl->GetCylinderGeometry().p2 = m_pC;
        cyl->GetCylinderGeometry().rad = radius;
        m_carrier->AddVisualShape(cyl);
    }

    if ((m_pC - m_pT).Length2() > threshold2) {
        auto cyl = chrono_types::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = m_pC;
        cyl->GetCylinderGeometry().p2 = m_pT;
        cyl->GetCylinderGeometry().rad = radius;
        m_carrier->AddVisualShape(cyl);
    }

    auto box = chrono_types::make_shared<ChBoxShape>();
    box->GetBoxGeometry().Size = ChVector<>(3 * radius, radius, radius);
    m_carrier->AddVisualShape(box, ChFrame<>(m_pT, ChMatrix33<>(GetPrismaticPitchAngle(), ChVector<>(0, 1, 0))));

    // Visualization of the tensioner spring (with default color)
    m_tensioner->AddVisualShape(chrono_types::make_shared<ChSpringShape>(0.06, 150, 15));
}

void ChTranslationalIdler::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_carrier);
    ChPart::RemoveVisualizationAssets(m_tensioner);
}

// -----------------------------------------------------------------------------
void ChTranslationalIdler::LogConstraintViolations() {
    ChVectorDynamic<> C = m_prismatic->GetConstraintViolation();
    GetLog() << "  Carrier-chassis prismatic\n";
    GetLog() << "  " << C(0) << "  ";
    GetLog() << "  " << C(1) << "  ";
    GetLog() << "  " << C(2) << "  ";
    GetLog() << "  " << C(3) << "  ";
    GetLog() << "  " << C(4) << "\n";

    m_idler_wheel->LogConstraintViolations();
}

// -----------------------------------------------------------------------------
void ChTranslationalIdler::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_carrier);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_prismatic);
    ChPart::ExportJointList(jsonDocument, joints);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_tensioner);
    ChPart::ExportLinSpringList(jsonDocument, springs);
}

void ChTranslationalIdler::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_carrier);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_prismatic);
    database.WriteJoints(joints);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_tensioner);
    database.WriteLinSprings(springs);
}

}  // end namespace vehicle
}  // end namespace chrono
