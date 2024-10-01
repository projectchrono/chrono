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
// Base class for an idler subsystem with a fixed distance tensioner.
// An idler consists of the idler wheel and a carrier body. The carrier body is
// connected to the chassis and the idler wheel to the carrier. A linear
// actuator connects the carrier body and a link body (the chassis or a
// supsension arm).
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapePointPoint.h"

#include "chrono_vehicle/tracked_vehicle/idler/ChDistanceIdler.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

class DistanceIdlerFunction : public ChFunction {
  public:
    DistanceIdlerFunction(double time, double init_val, double final_val)
        : m_time(time), m_init_val(init_val), m_final_val(final_val) {}
    DistanceIdlerFunction(const DistanceIdlerFunction& other)
        : m_time(other.m_time), m_init_val(other.m_init_val), m_final_val(other.m_final_val) {}

    virtual DistanceIdlerFunction* Clone() const override { return new DistanceIdlerFunction(*this); }

    virtual double GetVal(double x) const override {
        if (x < m_time)
            return m_init_val + (m_final_val - m_init_val) * (x / m_time);
        return m_final_val;
    }

  private:
    double m_time;
    double m_init_val;
    double m_final_val;
};

// -----------------------------------------------------------------------------
ChDistanceIdler::ChDistanceIdler(const std::string& name) : ChIdler(name) {}

ChDistanceIdler::~ChDistanceIdler() {
    auto sys = m_carrier->GetSystem();
    if (sys) {
        sys->Remove(m_carrier);
        sys->Remove(m_revolute);
        sys->Remove(m_tensioner);
    }
}

void ChDistanceIdler::Initialize(std::shared_ptr<ChChassis> chassis,
                                 const ChVector3d& location,
                                 ChTrackAssembly* track) {
    // Express the idler reference frame in the absolute coordinate system
    ChFrame<> idler_to_abs(location);
    idler_to_abs.ConcatenatePreTransformation(chassis->GetBody()->GetFrameRefToAbs());

    // Transform all points and directions to absolute frame
    m_points.resize(NUM_POINTS);

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector3d rel_pos = GetLocation(static_cast<PointId>(i));
        m_points[i] = idler_to_abs.TransformPointLocalToParent(rel_pos);
    }

    // Create and initialize the carrier body
    m_carrier = chrono_types::make_shared<ChBody>();
    m_carrier->SetName(m_name + "_carrier");
    m_carrier->SetPos(m_points[CARRIER]);
    m_carrier->SetRot(idler_to_abs.GetRot());
    m_carrier->SetMass(GetCarrierMass());
    m_carrier->SetInertiaXX(GetCarrierInertia());
    chassis->GetSystem()->AddBody(m_carrier);

    // Create and initialize the revolute joint between carrier and chassis
    m_revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute->SetName(m_name + "_carrier_pin");
    m_revolute->Initialize(chassis->GetBody(), m_carrier,
                           ChFrame<>(m_points[CARRIER_CHASSIS], idler_to_abs.GetRot() * QuatFromAngleX(CH_PI_2)));
    chassis->GetSystem()->AddLink(m_revolute);

    // Linear actuator function
    double init_dist = (m_points[MOTOR_ARM] - m_points[MOTOR_CARRIER]).Length();
    assert(init_dist < GetTensionerDistance());
    auto motfun = chrono_types::make_shared<DistanceIdlerFunction>(GetTensionerExtensionTime(), init_dist,
                                                                   GetTensionerDistance());

    // Create and initialize the tensioner motor element.
    // Connect the idler wheel carrier to the arm of the last suspension subsystem.
    // Attach a ramp function to extend the tensioner to desired distance.
    auto arm = track->GetTrackSuspensions().back()->GetCarrierBody();
    m_tensioner = chrono_types::make_shared<ChLinkLockLinActuator>();
    m_tensioner->SetName(m_name + "_tensioner");
    m_tensioner->SetActuatorFunction(motfun);
    m_tensioner->Initialize(arm, m_carrier, false, ChFrame<>(m_points[MOTOR_ARM]), ChFrame<>(m_points[MOTOR_CARRIER]));
    chassis->GetSystem()->AddLink(m_tensioner);

    // Invoke the base class implementation. This initializes the associated idler wheel.
    // Note: we must call this here, after the m_carrier body is created.
    ChIdler::Initialize(chassis, location, track);
}

void ChDistanceIdler::InitializeInertiaProperties() {
    m_mass = GetCarrierMass() + m_idler_wheel->GetMass();
}

void ChDistanceIdler::UpdateInertiaProperties() {
    m_xform = m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT));

    // Calculate COM and inertia expressed in global frame
    utils::CompositeInertia composite;
    composite.AddComponent(m_carrier->GetFrameCOMToAbs(), m_carrier->GetMass(), m_carrier->GetInertia());
    composite.AddComponent(m_idler_wheel->GetBody()->GetFrameCOMToAbs(), m_idler_wheel->GetBody()->GetMass(),
                           m_idler_wheel->GetBody()->GetInertia());

    // Express COM and inertia in subsystem reference frame
    m_com.SetPos(m_xform.TransformPointParentToLocal(composite.GetCOM()));
    m_com.SetRot(QUNIT);

    m_inertia = m_xform.GetRotMat().transpose() * composite.GetInertia() * m_xform.GetRotMat();
}

// -----------------------------------------------------------------------------
void ChDistanceIdler::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    // Express hardpoints in carrier local frame
    auto pC = m_carrier->TransformPointParentToLocal(m_points[CARRIER]);
    auto pW = m_carrier->TransformPointParentToLocal(m_points[CARRIER_WHEEL]);
    auto pR = m_carrier->TransformPointParentToLocal(m_points[CARRIER_CHASSIS]);
    auto pM = m_carrier->TransformPointParentToLocal(m_points[MOTOR_CARRIER]);

    double radius = GetCarrierVisRadius();

    ChColor carrier_col(0.6f, 0.2f, 0.6f);

    // Carrier-chassis revolute joint
    {
        auto cyl = utils::ChBodyGeometry::AddVisualizationCylinder(m_carrier,                                    //
                                                                   ChVector3d(pR.x(), pC.y() - radius, pR.z()),  //
                                                                   ChVector3d(pR.x(), pC.y() + radius, pR.z()),  //
                                                                   3 * radius);
        cyl->SetColor(carrier_col);
    }

    // Carrier-wheel revolute joint
    {
        auto cyl = utils::ChBodyGeometry::AddVisualizationCylinder(m_carrier,                                    //
                                                                   ChVector3d(pW.x(), pC.y() - radius, pW.z()),  //
                                                                   ChVector3d(pW.x(), pC.y() + radius, pW.z()),  //
                                                                   2 * radius);
        cyl->SetColor(carrier_col);
    }

    {
        auto cyl = utils::ChBodyGeometry::AddVisualizationCylinder(m_carrier,                           //
                                                                   ChVector3d(pR.x(), pC.y(), pR.z()),  //
                                                                   ChVector3d(pW.x(), pC.y(), pW.z()),  //
                                                                   radius);
        cyl->SetColor(carrier_col);
    }

    {
        auto cyl = utils::ChBodyGeometry::AddVisualizationCylinder(m_carrier,                           //
                                                                   ChVector3d(pW.x(), pC.y(), pW.z()),  //
                                                                   ChVector3d(pM.x(), pC.y(), pM.z()),  //
                                                                   radius);
        cyl->SetColor(carrier_col);
    }

    // Visualization of the tensioner spring
    auto seg = chrono_types::make_shared<ChVisualShapeSegment>();
    seg->SetColor(carrier_col);
    m_tensioner->AddVisualShape(seg);
}

void ChDistanceIdler::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_carrier);
    ChPart::RemoveVisualizationAssets(m_tensioner);
}

// -----------------------------------------------------------------------------
void ChDistanceIdler::LogConstraintViolations() {
    ChVectorDynamic<> C = m_revolute->GetConstraintViolation();
    std::cout << "  Carrier-chassis revolute\n";
    std::cout << "  " << C(0) << "  ";
    std::cout << "  " << C(1) << "  ";
    std::cout << "  " << C(2) << "  ";
    std::cout << "  " << C(3) << "  ";
    std::cout << "  " << C(4) << "\n";

    m_idler_wheel->LogConstraintViolations();
}

// -----------------------------------------------------------------------------
void ChDistanceIdler::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_carrier);
    ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute);
    joints.push_back(m_tensioner);
    ExportJointList(jsonDocument, joints);
}

void ChDistanceIdler::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_carrier);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute);
    joints.push_back(m_tensioner);
    database.WriteJoints(joints);
}

}  // end namespace vehicle
}  // end namespace chrono
