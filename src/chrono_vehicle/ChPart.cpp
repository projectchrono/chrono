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

#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualShapeModelFile.h"

#include "chrono_vehicle/ChPart.h"

#include "chrono_thirdparty/rapidjson/stringbuffer.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
ChPart::ChPart(const std::string& name)
    : m_name(name), m_initialized(false), m_output(false), m_parent(nullptr), m_mass(0), m_inertia(0) {}

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

void ChPart::AddInertiaProperties(ChVector<>& com, ChMatrix33<>& inertia) {
    //// RADU TODO: change ChFrame::TransformLocalToParent to return the transformed frame!!!!
    UpdateInertiaProperties();

    // Express the COM frame in global frame
    ChFrame<> com_abs;
    m_xform.TransformLocalToParent(m_com, com_abs);

    // Increment total COM position
    com += GetMass() * com_abs.GetPos();

    // Shift inertia away from COM (parallel axis theorem)
    // Express inertia relative to global frame.
    inertia += com_abs.GetA() * m_inertia * com_abs.GetA().transpose() +
               GetMass() * utils::CompositeInertia::InertiaShiftMatrix(com_abs.GetPos());
}

// -----------------------------------------------------------------------------
// Utility function for transforming inertia tensors between centroidal frames.
// It converts an inertia matrix specified in a centroidal frame aligned with the
// vehicle reference frame to an inertia matrix expressed in a centroidal body
// reference frame.
// -----------------------------------------------------------------------------
ChMatrix33<> ChPart::TransformInertiaMatrix(
    const ChVector<>& moments,        // moments of inertia in vehicle-aligned centroidal frame
    const ChVector<>& products,       // products of inertia in vehicle-aligned centroidal frame
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
// Default implementation of the function ExportComponentList.
// Derived classes should override this function and first invoke this base
// class implementation, followed by calls to the various static Export***List
// functions, as appropriate.
// -----------------------------------------------------------------------------

rapidjson::Value Vec2Val(const ChVector<>& vec, rapidjson::Document::AllocatorType& allocator) {
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
    obj.AddMember("rot u", Vec2Val(frame.GetA().Get_A_Xaxis(), allocator), allocator);
    obj.AddMember("rot v", Vec2Val(frame.GetA().Get_A_Yaxis(), allocator), allocator);
    obj.AddMember("rot w", Vec2Val(frame.GetA().Get_A_Zaxis(), allocator), allocator);
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
}

rapidjson::Value VisualModel2Val(std::shared_ptr<ChVisualModel> model, rapidjson::Document::AllocatorType& allocator) {
    rapidjson::Value jsonArray(rapidjson::kArrayType);

    for (const auto& item : model->GetShapes()) {
        const auto& shape = item.first;   // visual shape
        const auto& frame = item.second;  // shape position in model

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
        obj.AddMember("name", rapidjson::StringRef(body->GetName()), allocator);
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
        obj.AddMember("name", rapidjson::StringRef(shaft->GetName()), allocator);
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
        ChFrame<> A_X_J = ChFrame<>(joint->GetLinkAbsoluteCoords());  // transform from joint to absolute
        ChFrame<> P_X_J = P_X_A * A_X_J;                              // transform from joint to parent
        rapidjson::Value obj(rapidjson::kObjectType);
        obj.SetObject();
        obj.AddMember("name", rapidjson::StringRef(joint->GetName()), allocator);
        obj.AddMember("id", joint->GetIdentifier(), allocator);
        //// TODO: Change ChLink class hierarchy so we don't have to do these hacks!!!!
        auto body1 = static_cast<ChBody*>(joint->GetBody1());
        auto body2 = static_cast<ChBody*>(joint->GetBody2());
        obj.AddMember("body1 name", rapidjson::StringRef(body1->GetName()), allocator);
        obj.AddMember("body2 name", rapidjson::StringRef(body2->GetName()), allocator);
        obj.AddMember("body1 id", body1->GetIdentifier(), allocator);
        obj.AddMember("body2 id", body2->GetIdentifier(), allocator);
        obj.AddMember("coordinates w.r.t. subsystem", Frame2Val(P_X_J, allocator), allocator);
        obj.AddMember("relative link coordinates", Csys2Val(joint->GetLinkRelativeCoords(), allocator), allocator);
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
        obj.AddMember("name", rapidjson::StringRef(couple->GetName()), allocator);
        obj.AddMember("id", couple->GetIdentifier(), allocator);
        obj.AddMember("shaft1 name", rapidjson::StringRef(couple->GetShaft1()->GetName()), allocator);
        obj.AddMember("shaft2 name", rapidjson::StringRef(couple->GetShaft2()->GetName()), allocator);
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
        obj.AddMember("name", rapidjson::StringRef(marker->GetName()), allocator);
        obj.AddMember("id", marker->GetIdentifier(), allocator);
        obj.AddMember("body name", rapidjson::StringRef(marker->GetBody()->GetName()), allocator);
        obj.AddMember("body id", marker->GetBody()->GetIdentifier(), allocator);
        obj.AddMember("relative coordinates", Csys2Val(marker->GetCoord(), allocator), allocator);
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
        obj.AddMember("name", rapidjson::StringRef(spring->GetName()), allocator);
        obj.AddMember("id", spring->GetIdentifier(), allocator);
        //// TODO: Change ChLink class hierarchy so we don't have to do these hacks!!!!
        auto body1 = static_cast<ChBody*>(spring->GetBody1());
        auto body2 = static_cast<ChBody*>(spring->GetBody2());
        obj.AddMember("body1 name", rapidjson::StringRef(body1->GetName()), allocator);
        obj.AddMember("body2 name", rapidjson::StringRef(body2->GetName()), allocator);
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
        auto pos = spring->GetVisualModelFrame().GetPos();               // position in absolute frame
        auto axis = spring->GetVisualModelFrame().GetA().Get_A_Zaxis();  // axis in absolute frame
        rapidjson::Value obj(rapidjson::kObjectType);
        obj.SetObject();
        obj.AddMember("name", rapidjson::StringRef(spring->GetName()), allocator);
        obj.AddMember("id", spring->GetIdentifier(), allocator);
        //// TODO: Change ChLink class hierarchy so we don't have to do these hacks!!!!
        auto body1 = static_cast<ChBody*>(spring->GetBody1());
        auto body2 = static_cast<ChBody*>(spring->GetBody2());
        obj.AddMember("body1 name", rapidjson::StringRef(body1->GetName()), allocator);
        obj.AddMember("body2 name", rapidjson::StringRef(body2->GetName()), allocator);
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
        obj.AddMember("name", rapidjson::StringRef(load->GetName()), allocator);
        obj.AddMember("id", load->GetIdentifier(), allocator);
        obj.AddMember("body1 name", rapidjson::StringRef(load->GetBodyA()->GetName()), allocator);
        obj.AddMember("body2 name", rapidjson::StringRef(load->GetBodyB()->GetName()), allocator);
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
