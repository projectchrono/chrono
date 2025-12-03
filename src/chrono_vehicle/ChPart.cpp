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
// Base class for all vehicle subsystems.
//
// =============================================================================

#include <stdexcept>

#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualShapeModelFile.h"

#include "chrono_vehicle/ChPart.h"

#include "chrono_thirdparty/rapidjson/stringbuffer.h"

namespace chrono {
namespace vehicle {

ChPart::ChPart(const std::string& name)
    : m_name(name), m_initialized(false), m_output(false), m_parent(nullptr), m_mass(0), m_inertia(0), m_obj_tag(-1) {}

// -----------------------------------------------------------------------------

uint16_t ChPart::GetVehicleTag() const {
    if (m_parent)
        return m_parent->GetVehicleTag();

    // If no parent and an override is not provided, assume single "vehicle"
    return 0;
}

// -----------------------------------------------------------------------------

void ChPart::Create(const rapidjson::Document& d) {
    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    assert(d.HasMember("Name"));

    // Set subsystem name
    SetName(d["Name"].GetString());

    // If present, enable output (default is no output)
    if (d.HasMember("Output")) {
        assert(d["Output"].IsBool());
        SetOutput(d["Output"].GetBool());
    }
}

void ChPart::Initialize() {
    PopulateComponentList();
    m_initialized = true;
}

// -----------------------------------------------------------------------------
void ChPart::SetVisualizationType(VisualizationType vis) {
    RemoveVisualizationAssets();
    AddVisualizationAssets(vis);
}

// -----------------------------------------------------------------------------
void ChPart::AddMass(double& mass) {
    InitializeInertiaProperties();
    mass += m_mass;
}

void ChPart::AddInertiaProperties(ChVector3d& com, ChMatrix33<>& inertia) {
    //// RADU TODO: change ChFrame::TransformLocalToParent to return the transformed frame!!!!
    UpdateInertiaProperties();

    // Express the COM frame in global frame
    ChFrame<> com_abs = m_xform.TransformLocalToParent(m_com);

    // Increment total COM position
    com += GetMass() * com_abs.GetPos();

    // Shift inertia away from COM (parallel axis theorem)
    // Express inertia relative to global frame.
    inertia += com_abs.GetRotMat() * m_inertia * com_abs.GetRotMat().transpose() +
               GetMass() * CompositeInertia::InertiaShiftMatrix(com_abs.GetPos());
}

// -----------------------------------------------------------------------------
// Utility function for transforming inertia tensors between centroidal frames.
// It converts an inertia matrix specified in a centroidal frame aligned with the
// vehicle reference frame to an inertia matrix expressed in a centroidal body
// reference frame.
// -----------------------------------------------------------------------------
ChMatrix33<> ChPart::TransformInertiaMatrix(
    const ChVector3d& moments,        // moments of inertia in vehicle-aligned centroidal frame
    const ChVector3d& products,       // products of inertia in vehicle-aligned centroidal frame
    const ChMatrix33<>& vehicle_rot,  // vehicle absolute orientation matrix
    const ChMatrix33<>& body_rot      // body absolute orientation matrix
) {
    // Calculate rotation matrix body-to-vehicle
    ChMatrix33<> R = vehicle_rot.transpose() * body_rot;

    // Assemble the inertia matrix in vehicle-aligned centroidal frame
    ChMatrix33<> J_vehicle(moments, products);

    // Calculate transformed inertia matrix:  (R' * J_vehicle * R)
    return R.transpose() * J_vehicle * R;
}

// -----------------------------------------------------------------------------
// Implementation of the function ExportComponentList.
// -----------------------------------------------------------------------------

rapidjson::Value Vec2Val(const ChVector3d& vec, rapidjson::Document::AllocatorType& allocator) {
    rapidjson::Value array(rapidjson::kArrayType);
    array.PushBack(vec.x(), allocator);
    array.PushBack(vec.y(), allocator);
    array.PushBack(vec.z(), allocator);
    return array;
}

rapidjson::Value Quat2Val(const ChQuaternion<>& q, rapidjson::Document::AllocatorType& allocator) {
    rapidjson::Value array(rapidjson::kArrayType);
    array.PushBack(q.e0(), allocator);
    array.PushBack(q.e1(), allocator);
    array.PushBack(q.e2(), allocator);
    array.PushBack(q.e3(), allocator);
    return array;
}

rapidjson::Value Frame2Val(const ChFrame<>& frame, rapidjson::Document::AllocatorType& allocator) {
    rapidjson::Value obj(rapidjson::kObjectType);
    obj.AddMember("pos", Vec2Val(frame.GetPos(), allocator), allocator);
    obj.AddMember("rot quat", Quat2Val(frame.GetRot(), allocator), allocator);
    obj.AddMember("rot u", Vec2Val(frame.GetRotMat().GetAxisX(), allocator), allocator);
    obj.AddMember("rot v", Vec2Val(frame.GetRotMat().GetAxisY(), allocator), allocator);
    obj.AddMember("rot w", Vec2Val(frame.GetRotMat().GetAxisZ(), allocator), allocator);
    return obj;
}

rapidjson::Value Csys2Val(const ChCoordsys<>& csys, rapidjson::Document::AllocatorType& allocator) {
    return Frame2Val(ChFrame<>(csys), allocator);
}

rapidjson::Value Mat2Val(const ChMatrixNM<double, 6, 6>& mat, rapidjson::Document::AllocatorType& allocator) {
    rapidjson::Value rows(rapidjson::kArrayType);
    for (int i = 0; i < 6; i++) {
        rapidjson::Value column(rapidjson::kArrayType);
        column.PushBack(mat(i, 0), allocator);
        column.PushBack(mat(i, 1), allocator);
        column.PushBack(mat(i, 2), allocator);
        column.PushBack(mat(i, 3), allocator);
        column.PushBack(mat(i, 4), allocator);
        column.PushBack(mat(i, 5), allocator);
        rows.PushBack(column, allocator);
    }
    return rows;
}

void ChPart::ExportComponentList(rapidjson::Document& jsonDocument) const {
    rapidjson::Document::AllocatorType& allocator = jsonDocument.GetAllocator();
    std::string template_name = GetTemplateName();
    ChFrame<> frame = GetTransform();
    jsonDocument.AddMember("name", rapidjson::StringRef(m_name.c_str()), allocator);
    jsonDocument.AddMember("template", rapidjson::Value(template_name.c_str(), allocator).Move(), allocator);
    jsonDocument.AddMember("output", m_output, allocator);
    jsonDocument.AddMember("position", Vec2Val(frame.GetPos(), allocator), allocator);
    jsonDocument.AddMember("rotation", Quat2Val(frame.GetRot(), allocator), allocator);

    ExportBodyList(jsonDocument, m_bodies);
    ExportMarkerList(jsonDocument, m_markers);
    ExportShaftList(jsonDocument, m_shafts);
    ExportJointList(jsonDocument, m_joints);
    ExportCouplesList(jsonDocument, m_couples);
    ExportLinSpringList(jsonDocument, m_tsdas);
    ExportRotSpringList(jsonDocument, m_rsdas);
    ExportBodyLoadList(jsonDocument, m_body_loads);
    ExportLinMotorList(jsonDocument, m_lin_motors);
    ExportRotMotorList(jsonDocument, m_rot_motors);
}

rapidjson::Value VisualModel2Val(std::shared_ptr<ChVisualModel> model, rapidjson::Document::AllocatorType& allocator) {
    rapidjson::Value jsonArray(rapidjson::kArrayType);

    for (const auto& item : model->GetShapeInstances()) {
        const auto& shape = item.shape;  // visual shape
        const auto& frame = item.frame;  // shape position in model

        rapidjson::Value obj(rapidjson::kObjectType);
        if (auto trimesh = std::dynamic_pointer_cast<ChVisualShapeTriangleMesh>(shape)) {
            const auto& name = trimesh->GetName();
            obj.AddMember("type", "TRIMESH", allocator);
            obj.AddMember("name", rapidjson::StringRef(name.c_str()), allocator);
            obj.AddMember("center", Vec2Val(frame.GetPos(), allocator), allocator);
            obj.AddMember("orientation", Quat2Val(frame.GetRot(), allocator), allocator);
        } else if (auto mfile = std::dynamic_pointer_cast<ChVisualShapeModelFile>(shape)) {
            const auto& file = mfile->GetFilename();
            obj.AddMember("type", "MODELFILE", allocator);
            obj.AddMember("filename", rapidjson::StringRef(file.c_str()), allocator);
            obj.AddMember("center", Vec2Val(frame.GetPos(), allocator), allocator);
            obj.AddMember("orientation", Quat2Val(frame.GetRot(), allocator), allocator);
        } else if (auto sph = std::dynamic_pointer_cast<ChVisualShapeSphere>(shape)) {
            auto rad = sph->GetRadius();
            obj.AddMember("type", "SPHERE", allocator);
            obj.AddMember("center", Vec2Val(frame.GetPos(), allocator), allocator);
            obj.AddMember("radius", rad, allocator);
        } else if (auto cyl = std::dynamic_pointer_cast<ChVisualShapeCylinder>(shape)) {
            auto rad = cyl->GetRadius();
            auto height = cyl->GetHeight();
            obj.AddMember("type", "CYLINDER", allocator);
            obj.AddMember("center", Vec2Val(frame.GetPos(), allocator), allocator);
            obj.AddMember("orientation", Quat2Val(frame.GetRot(), allocator), allocator);
            obj.AddMember("radius", rad, allocator);
            obj.AddMember("height", height, allocator);
        } else if (auto box = std::dynamic_pointer_cast<ChVisualShapeBox>(shape)) {
            const auto& len = box->GetLengths();
            obj.AddMember("type", "BOX", allocator);
            obj.AddMember("center", Vec2Val(frame.GetPos(), allocator), allocator);
            obj.AddMember("orientation", Quat2Val(frame.GetRot(), allocator), allocator);
            obj.AddMember("lengths", Vec2Val(len, allocator), allocator);
        }
        jsonArray.PushBack(obj, allocator);
    }
    return jsonArray;
}

void ChPart::ExportBodyList(rapidjson::Document& jsonDocument, std::vector<std::shared_ptr<ChBody>> bodies) const {
    rapidjson::Document::AllocatorType& allocator = jsonDocument.GetAllocator();

    auto P_X_A = GetTransform().GetInverse();  // transform from absolute to parent

    rapidjson::Value jsonArray(rapidjson::kArrayType);
    for (auto body : bodies) {
        ChFrame<> A_X_B = *body;          // transform from body to absolute
        ChFrame<> P_X_B = P_X_A * A_X_B;  // transform from body to parent
        rapidjson::Value obj(rapidjson::kObjectType);
        obj.SetObject();
        obj.AddMember("name", rapidjson::StringRef(body->GetName().c_str()), allocator);
        obj.AddMember("id", body->GetIdentifier(), allocator);
        obj.AddMember("mass", body->GetMass(), allocator);
        obj.AddMember("coordinates w.r.t. subsystem", Frame2Val(P_X_B, allocator), allocator);
        obj.AddMember("moments of inertia", Vec2Val(body->GetInertiaXX(), allocator), allocator);
        obj.AddMember("products of inertia", Vec2Val(body->GetInertiaXY(), allocator), allocator);
        if (body->GetVisualModel())
            obj.AddMember("visualization", VisualModel2Val(body->GetVisualModel(), allocator), allocator);
        jsonArray.PushBack(obj, allocator);
    }
    jsonDocument.AddMember("bodies", jsonArray, allocator);
}

void ChPart::ExportShaftList(rapidjson::Document& jsonDocument, std::vector<std::shared_ptr<ChShaft>> shafts) const {
    rapidjson::Document::AllocatorType& allocator = jsonDocument.GetAllocator();

    rapidjson::Value jsonArray(rapidjson::kArrayType);
    for (auto shaft : shafts) {
        rapidjson::Value obj(rapidjson::kObjectType);
        obj.SetObject();
        obj.AddMember("name", rapidjson::StringRef(shaft->GetName().c_str()), allocator);
        obj.AddMember("id", shaft->GetIdentifier(), allocator);
        obj.AddMember("inertia", shaft->GetInertia(), allocator);
        jsonArray.PushBack(obj, allocator);
    }
    jsonDocument.AddMember("shafts", jsonArray, allocator);
}

void ChPart::ExportJointList(rapidjson::Document& jsonDocument, std::vector<std::shared_ptr<ChLink>> joints) const {
    rapidjson::Document::AllocatorType& allocator = jsonDocument.GetAllocator();

    auto P_X_A = GetTransform().GetInverse();  // transform from absolute to parent

    rapidjson::Value jsonArray(rapidjson::kArrayType);
    for (auto joint : joints) {
        ChFrame<> A_X_J = ChFrame<>(joint->GetFrame2Abs());  // transform from joint to absolute
        ChFrame<> P_X_J = P_X_A * A_X_J;                     // transform from joint to parent
        rapidjson::Value obj(rapidjson::kObjectType);
        obj.SetObject();
        obj.AddMember("name", rapidjson::StringRef(joint->GetName().c_str()), allocator);
        obj.AddMember("id", joint->GetIdentifier(), allocator);
        //// TODO: Change ChLink class hierarchy so we don't have to do these hacks!!!!
        auto body1 = static_cast<const ChBody*>(joint->GetBody1());
        auto body2 = static_cast<const ChBody*>(joint->GetBody2());
        obj.AddMember("body1 name", rapidjson::StringRef(body1->GetName().c_str()), allocator);
        obj.AddMember("body2 name", rapidjson::StringRef(body2->GetName().c_str()), allocator);
        obj.AddMember("body1 id", body1->GetIdentifier(), allocator);
        obj.AddMember("body2 id", body2->GetIdentifier(), allocator);
        obj.AddMember("coordinates w.r.t. subsystem", Frame2Val(P_X_J, allocator), allocator);
        obj.AddMember("relative link coordinates", Csys2Val(joint->GetFrame2Rel().GetCoordsys(), allocator), allocator);
        jsonArray.PushBack(obj, allocator);
    }
    jsonDocument.AddMember("joints", jsonArray, allocator);
}

void ChPart::ExportCouplesList(rapidjson::Document& jsonDocument,
                               std::vector<std::shared_ptr<ChShaftsCouple>> couples) const {
    rapidjson::Document::AllocatorType& allocator = jsonDocument.GetAllocator();

    rapidjson::Value jsonArray(rapidjson::kArrayType);
    for (auto couple : couples) {
        rapidjson::Value obj(rapidjson::kObjectType);
        obj.SetObject();
        obj.AddMember("name", rapidjson::StringRef(couple->GetName().c_str()), allocator);
        obj.AddMember("id", couple->GetIdentifier(), allocator);
        obj.AddMember("shaft1 name", rapidjson::StringRef(couple->GetShaft1()->GetName().c_str()), allocator);
        obj.AddMember("shaft2 name", rapidjson::StringRef(couple->GetShaft2()->GetName().c_str()), allocator);
        obj.AddMember("shaft1 id", couple->GetShaft1()->GetIdentifier(), allocator);
        obj.AddMember("shaft2 id", couple->GetShaft2()->GetIdentifier(), allocator);
        jsonArray.PushBack(obj, allocator);
    }
    jsonDocument.AddMember("couples", jsonArray, allocator);
}

void ChPart::ExportMarkerList(rapidjson::Document& jsonDocument, std::vector<std::shared_ptr<ChMarker>> markers) const {
    rapidjson::Document::AllocatorType& allocator = jsonDocument.GetAllocator();

    rapidjson::Value jsonArray(rapidjson::kArrayType);
    for (auto marker : markers) {
        rapidjson::Value obj(rapidjson::kObjectType);
        obj.SetObject();
        obj.AddMember("name", rapidjson::StringRef(marker->GetName().c_str()), allocator);
        obj.AddMember("id", marker->GetIdentifier(), allocator);
        obj.AddMember("body name", rapidjson::StringRef(marker->GetBody()->GetName().c_str()), allocator);
        obj.AddMember("body id", marker->GetBody()->GetIdentifier(), allocator);
        obj.AddMember("relative coordinates", Csys2Val(marker->GetCoordsys(), allocator), allocator);
        jsonArray.PushBack(obj, allocator);
    }
    jsonDocument.AddMember("markers", jsonArray, allocator);
}

void ChPart::ExportLinSpringList(rapidjson::Document& jsonDocument,
                                 std::vector<std::shared_ptr<ChLinkTSDA>> springs) const {
    rapidjson::Document::AllocatorType& allocator = jsonDocument.GetAllocator();

    const auto& A_X_P = GetTransform();  // transform from parent to absolute

    rapidjson::Value jsonArray(rapidjson::kArrayType);
    for (auto spring : springs) {
        rapidjson::Value obj(rapidjson::kObjectType);
        obj.SetObject();
        obj.AddMember("name", rapidjson::StringRef(spring->GetName().c_str()), allocator);
        obj.AddMember("id", spring->GetIdentifier(), allocator);
        //// TODO: Change ChLink class hierarchy so we don't have to do these hacks!!!!
        auto body1 = static_cast<const ChBody*>(spring->GetBody1());
        auto body2 = static_cast<const ChBody*>(spring->GetBody2());
        obj.AddMember("body1 name", rapidjson::StringRef(body1->GetName().c_str()), allocator);
        obj.AddMember("body2 name", rapidjson::StringRef(body2->GetName().c_str()), allocator);
        obj.AddMember("body1 id", body1->GetIdentifier(), allocator);
        obj.AddMember("body2 id", body2->GetIdentifier(), allocator);
        obj.AddMember("point1", Vec2Val(A_X_P.TransformPointParentToLocal(spring->GetPoint1Abs()), allocator),
                      allocator);
        obj.AddMember("point2", Vec2Val(A_X_P.TransformPointParentToLocal(spring->GetPoint2Abs()), allocator),
                      allocator);
        obj.AddMember("has functor", spring->GetForceFunctor() != nullptr, allocator);
        if (!spring->GetForceFunctor()) {
            obj.AddMember("spring coefficient", spring->GetSpringCoefficient(), allocator);
            obj.AddMember("damping coefficient", spring->GetDampingCoefficient(), allocator);
            obj.AddMember("pre-load", spring->GetActuatorForce(), allocator);
        } else {
            obj.AddMember("force functor", spring->GetForceFunctor()->exportJSON(allocator), allocator);
        }
        obj.AddMember("rest length", spring->GetRestLength(), allocator);
        jsonArray.PushBack(obj, allocator);
    }
    jsonDocument.AddMember("linear spring-dampers", jsonArray, allocator);
}

void ChPart::ExportRotSpringList(rapidjson::Document& jsonDocument,
                                 std::vector<std::shared_ptr<ChLinkRSDA>> springs) const {
    rapidjson::Document::AllocatorType& allocator = jsonDocument.GetAllocator();

    const auto& A_X_P = GetTransform();  // transform from parent to absolute

    rapidjson::Value jsonArray(rapidjson::kArrayType);
    for (auto spring : springs) {
        auto pos = spring->GetVisualModelFrame().GetPos();                 // position in absolute frame
        auto axis = spring->GetVisualModelFrame().GetRotMat().GetAxisZ();  // axis in absolute frame
        rapidjson::Value obj(rapidjson::kObjectType);
        obj.SetObject();
        obj.AddMember("name", rapidjson::StringRef(spring->GetName().c_str()), allocator);
        obj.AddMember("id", spring->GetIdentifier(), allocator);
        //// TODO: Change ChLink class hierarchy so we don't have to do these hacks!!!!
        auto body1 = static_cast<const ChBody*>(spring->GetBody1());
        auto body2 = static_cast<const ChBody*>(spring->GetBody2());
        obj.AddMember("body1 name", rapidjson::StringRef(body1->GetName().c_str()), allocator);
        obj.AddMember("body2 name", rapidjson::StringRef(body2->GetName().c_str()), allocator);
        obj.AddMember("body1 id", body1->GetIdentifier(), allocator);
        obj.AddMember("body2 id", body2->GetIdentifier(), allocator);
        obj.AddMember("pos", Vec2Val(A_X_P.TransformPointParentToLocal(pos), allocator), allocator);
        obj.AddMember("axis", Vec2Val(A_X_P.TransformDirectionParentToLocal(axis), allocator), allocator);
        obj.AddMember("has functor", spring->GetTorqueFunctor() != nullptr, allocator);
        if (!spring->GetTorqueFunctor()) {
            obj.AddMember("spring coefficient", spring->GetSpringCoefficient(), allocator);
            obj.AddMember("damping coefficient", spring->GetDampingCoefficient(), allocator);
            obj.AddMember("pre-load", spring->GetActuatorTorque(), allocator);
        } else {
            obj.AddMember("torque functor", spring->GetTorqueFunctor()->exportJSON(allocator), allocator);
        }
        obj.AddMember("rest angle", spring->GetRestAngle(), allocator);
        jsonArray.PushBack(obj, allocator);
    }
    jsonDocument.AddMember("rotational spring-dampers", jsonArray, allocator);
}

void ChPart::ExportBodyLoadList(rapidjson::Document& jsonDocument,
                                std::vector<std::shared_ptr<ChLoadBodyBody>> loads) const {
    rapidjson::Document::AllocatorType& allocator = jsonDocument.GetAllocator();

    rapidjson::Value jsonArray(rapidjson::kArrayType);
    for (auto load : loads) {
        rapidjson::Value obj(rapidjson::kObjectType);
        obj.SetObject();
        obj.AddMember("name", rapidjson::StringRef(load->GetName().c_str()), allocator);
        obj.AddMember("id", load->GetIdentifier(), allocator);
        obj.AddMember("body1 name", rapidjson::StringRef(load->GetBodyA()->GetName().c_str()), allocator);
        obj.AddMember("body2 name", rapidjson::StringRef(load->GetBodyB()->GetName().c_str()), allocator);
        obj.AddMember("body1 id", load->GetBodyA()->GetIdentifier(), allocator);
        obj.AddMember("body2 id", load->GetBodyB()->GetIdentifier(), allocator);
        if (auto bushing = std::dynamic_pointer_cast<ChLoadBodyBodyBushingGeneric>(load)) {
            obj.AddMember("stiffness matrix", Mat2Val(bushing->GetStiffnessMatrix(), allocator), allocator);
            obj.AddMember("damping matrix", Mat2Val(bushing->GetDampingMatrix(), allocator), allocator);
        }
        jsonArray.PushBack(obj, allocator);
    }
    jsonDocument.AddMember("bushings", jsonArray, allocator);
}

void ChPart::ExportLinMotorList(rapidjson::Document& jsonDocument,
                                std::vector<std::shared_ptr<ChLinkMotorLinear>> loads) const {
    //// TODO
}

void ChPart::ExportRotMotorList(rapidjson::Document& jsonDocument,
                                std::vector<std::shared_ptr<ChLinkMotorRotation>> loads) const {
    //// TODO
}

void ChPart::Output(ChOutput& database) const {
    if (!m_output)
        return;

    database.WriteBodies(m_bodies);
    database.WriteMarkers(m_markers);
    database.WriteShafts(m_shafts);
    database.WriteJoints(m_joints);
    database.WriteCouples(m_couples);
    database.WriteLinSprings(m_tsdas);
    database.WriteRotSprings(m_rsdas);
    database.WriteBodyBodyLoads(m_body_loads);
    database.WriteLinMotors(m_lin_motors);
    database.WriteRotMotors(m_rot_motors);
}

void ChPart::WriteCheckpoint(ChCheckpoint& database) const {
    database.WriteBodies(m_bodies);
    database.WriteShafts(m_shafts);
    database.WriteJoints(m_joints);
    database.WriteCouples(m_couples);
    database.WriteLinSprings(m_tsdas);
    database.WriteRotSprings(m_rsdas);
    database.WriteBodyBodyLoads(m_body_loads);
    database.WriteLinMotors(m_lin_motors);
    database.WriteRotMotors(m_rot_motors);
}

void ChPart::ReadCheckpoint(ChCheckpoint& database) {
    database.ReadBodies(m_bodies);
    database.ReadShafts(m_shafts);
    database.ReadJoints(m_joints);
    database.ReadCouples(m_couples);
    database.ReadLinSprings(m_tsdas);
    database.ReadRotSprings(m_rsdas);
    database.ReadBodyBodyLoads(m_body_loads);
    database.ReadLinMotors(m_lin_motors);
    database.ReadRotMotors(m_rot_motors);
}

void ChPart::RemoveVisualizationAssets(std::shared_ptr<ChPhysicsItem> item) {
    if (item->GetVisualModel())
        item->GetVisualModel()->Clear();
}

void ChPart::RemoveVisualizationAsset(std::shared_ptr<ChPhysicsItem> item, std::shared_ptr<ChVisualShape> shape) {
    if (item->GetVisualModel())
        item->GetVisualModel()->Erase(shape);
}

}  // end namespace vehicle
}  // end namespace chrono
