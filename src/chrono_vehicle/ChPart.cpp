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

#include "chrono_vehicle/ChPart.h"

#include "chrono_thirdparty/rapidjson/stringbuffer.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
ChPart::ChPart(const std::string& name) : m_name(name), m_output(false), m_parent(nullptr), m_mass(0), m_inertia(0) {}

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
void ChPart::ExportComponentList(rapidjson::Document& jsonDocument) const {
    std::string template_name = GetTemplateName();
    jsonDocument.AddMember("name", rapidjson::StringRef(m_name.c_str()), jsonDocument.GetAllocator());
    jsonDocument.AddMember("template", rapidjson::Value(template_name.c_str(), jsonDocument.GetAllocator()).Move(),
                           jsonDocument.GetAllocator());
    jsonDocument.AddMember("output", m_output, jsonDocument.GetAllocator());
}

void ChPart::ExportBodyList(rapidjson::Document& jsonDocument, std::vector<std::shared_ptr<ChBody>> bodies) {
    rapidjson::Document::AllocatorType& allocator = jsonDocument.GetAllocator();

    rapidjson::Value jsonArray(rapidjson::kArrayType);
    for (auto body : bodies) {
        rapidjson::Value obj(rapidjson::kObjectType);
        obj.SetObject();
        obj.AddMember("name", rapidjson::StringRef(body->GetName()), allocator);
        obj.AddMember("id", body->GetIdentifier(), allocator);
        jsonArray.PushBack(obj, allocator);
    }
    jsonDocument.AddMember("bodies", jsonArray, allocator);
}

void ChPart::ExportShaftList(rapidjson::Document& jsonDocument, std::vector<std::shared_ptr<ChShaft>> shafts) {
    rapidjson::Document::AllocatorType& allocator = jsonDocument.GetAllocator();

    rapidjson::Value jsonArray(rapidjson::kArrayType);
    for (auto shaft : shafts) {
        rapidjson::Value obj(rapidjson::kObjectType);
        obj.SetObject();
        obj.AddMember("name", rapidjson::StringRef(shaft->GetName()), allocator);
        obj.AddMember("id", shaft->GetIdentifier(), allocator);
        jsonArray.PushBack(obj, allocator);
    }
    jsonDocument.AddMember("shafts", jsonArray, allocator);
}

void ChPart::ExportJointList(rapidjson::Document& jsonDocument, std::vector<std::shared_ptr<ChLink>> joints) {
    rapidjson::Document::AllocatorType& allocator = jsonDocument.GetAllocator();

    rapidjson::Value jsonArray(rapidjson::kArrayType);
    for (auto joint : joints) {
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
        jsonArray.PushBack(obj, allocator);
    }
    jsonDocument.AddMember("joints", jsonArray, allocator);
}

void ChPart::ExportCouplesList(rapidjson::Document& jsonDocument,
                               std::vector<std::shared_ptr<ChShaftsCouple>> couples) {
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

void ChPart::ExportMarkerList(rapidjson::Document& jsonDocument, std::vector<std::shared_ptr<ChMarker>> markers) {
    rapidjson::Document::AllocatorType& allocator = jsonDocument.GetAllocator();

    rapidjson::Value jsonArray(rapidjson::kArrayType);
    for (auto marker : markers) {
        rapidjson::Value obj(rapidjson::kObjectType);
        obj.SetObject();
        obj.AddMember("name", rapidjson::StringRef(marker->GetName()), allocator);
        obj.AddMember("id", marker->GetIdentifier(), allocator);
        obj.AddMember("body name", rapidjson::StringRef(marker->GetBody()->GetName()), allocator);
        obj.AddMember("body id", marker->GetBody()->GetIdentifier(), allocator);
        jsonArray.PushBack(obj, allocator);
    }
    jsonDocument.AddMember("markers", jsonArray, allocator);
}

void ChPart::ExportLinSpringList(rapidjson::Document& jsonDocument, std::vector<std::shared_ptr<ChLinkTSDA>> springs) {
    rapidjson::Document::AllocatorType& allocator = jsonDocument.GetAllocator();

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
        jsonArray.PushBack(obj, allocator);
    }
    jsonDocument.AddMember("linear spring-dampers", jsonArray, allocator);
}

void ChPart::ExportRotSpringList(rapidjson::Document& jsonDocument, std::vector<std::shared_ptr<ChLinkRSDA>> springs) {
    rapidjson::Document::AllocatorType& allocator = jsonDocument.GetAllocator();

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
        jsonArray.PushBack(obj, allocator);
    }
    jsonDocument.AddMember("rotational spring-dampers", jsonArray, allocator);
}

void ChPart::ExportBodyLoadList(rapidjson::Document& jsonDocument, std::vector<std::shared_ptr<ChLoadBodyBody>> loads) {
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
        jsonArray.PushBack(obj, allocator);
    }
    jsonDocument.AddMember("body-body loads", jsonArray, allocator);
}

void ChPart::RemoveVisualizationAssets(std::shared_ptr<ChPhysicsItem> item) {
    if (item->GetVisualModel())
        item->GetVisualModel()->Clear();
}

void ChPart::RemoveVisualizationAsset(std::shared_ptr<ChPhysicsItem> item, std::shared_ptr<ChVisualShape> shape) {
    if (item->GetVisualModel())
        item->GetVisualModel()->Erase(shape);
}

// =============================================================================

MaterialInfo::MaterialInfo() : mu(0.8f), cr(0.01f), Y(2e7f), nu(0.3f), kn(2e5f), gn(40.0f), kt(2e5f), gt(20.0f) {}

MaterialInfo::MaterialInfo(float mu_, float cr_, float Y_, float nu_, float kn_, float gn_, float kt_, float gt_)
    : mu(mu_), cr(cr_), Y(Y_), nu(nu_), kn(kn_), gn(gn_), kt(kt_), gt(gt_) {}

std::shared_ptr<ChMaterialSurface> MaterialInfo::CreateMaterial(ChContactMethod contact_method) {
    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChMaterialSurface>();
    }
}

}  // end namespace vehicle
}  // end namespace chrono
