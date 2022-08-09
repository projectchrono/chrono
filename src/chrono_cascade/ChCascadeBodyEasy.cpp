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

#include "chrono_cascade/ChCascadeBodyEasy.h"

namespace chrono {
namespace cascade {

ChCascadeBodyEasy::ChCascadeBodyEasy(TopoDS_Shape& shape,
                                     double density,
                                     std::shared_ptr<ChCascadeTriangulate> vis_params,
                                     bool collide,
                                     std::shared_ptr<ChMaterialSurface> mat) {
    Init(shape, density, vis_params, collide, mat);
}

ChCascadeBodyEasy::ChCascadeBodyEasy(TopoDS_Shape& shape,
                                     double density,
                                     bool visualize,
                                     bool collide,
                                     std::shared_ptr<ChMaterialSurface> mat) {
    if (visualize)
        Init(shape, density, chrono_types::make_shared<ChCascadeTriangulate>(), collide, mat);
    else
        Init(shape, density, nullptr, collide, mat);
}

void ChCascadeBodyEasy::Init(TopoDS_Shape& shape,
                             double density,
                             std::shared_ptr<ChCascadeTriangulate> vis_params,
                             bool collide,
                             std::shared_ptr<ChMaterialSurface> mat) {
    chrono::ChFrame<>* user_ref_to_abs = 0;  // as parameter?
    chrono::ChFrame<> frame_ref_to_abs;

    if (!user_ref_to_abs) {
        TopLoc_Location loc_shape_to_abs = shape.Location();
        chrono::cascade::ChCascadeDoc::FromCascadeToChrono(loc_shape_to_abs, frame_ref_to_abs);
    } else {
        frame_ref_to_abs = *user_ref_to_abs;
    }

    // Reset shape location to local ref csys (identity).
    this->topods_shape = shape;
    this->topods_shape.Location(TopLoc_Location());

    // compute mass properties and COG reference
    chrono::ChVector<> cog;
    chrono::ChVector<> inertiaXX;
    chrono::ChVector<> inertiaXY;
    double vol;
    double mass;
    chrono::cascade::ChCascadeDoc::GetVolumeProperties(topods_shape, density, cog, inertiaXX, inertiaXY, vol, mass);

    // Set mass and COG and REF references
    this->SetDensity((float)density);
    this->SetMass(mass);
    this->SetInertiaXX(inertiaXX);
    this->SetInertiaXY(inertiaXY);
    this->SetFrame_REF_to_abs(frame_ref_to_abs);

    chrono::ChFrame<> frame_cog_to_ref;
    frame_cog_to_ref.SetPos(cog);
    frame_cog_to_ref.SetRot(chrono::QUNIT);
    this->SetFrame_COG_to_REF(frame_cog_to_ref);

    // Add a visualization asset if needed
    if (vis_params) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        ChCascadeMeshTools::fillTriangleMeshFromCascade(*trimesh, topods_shape, *vis_params);

        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        this->AddVisualShape(trimesh_shape);

        // Add a collision shape if needed
        if (collide) {
            assert(mat);
            GetCollisionModel()->ClearModel();
            GetCollisionModel()->AddTriangleMesh(mat, trimesh, false, false);
            GetCollisionModel()->BuildModel();
            SetCollide(true);
        }
    }
}

ChCascadeBodyEasyProfile::ChCascadeBodyEasyProfile(std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>> wires,
                                                   std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>> holes,
                                                   double thickness,
                                                   double density,
                                                   std::shared_ptr<ChCascadeTriangulate> vis_params,
                                                   bool collide,
                                                   std::shared_ptr<ChMaterialSurface> mat) {
    AddProfile(wires, holes, thickness, density, vis_params, collide, mat);
}

void ChCascadeBodyEasyProfile::AddProfile(std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>> wires,
                                          std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>> holes,
                                          double thickness,
                                          double density,
                                          std::shared_ptr<ChCascadeTriangulate> vis_params,
                                          bool collide,
                                          std::shared_ptr<ChMaterialSurface> mat) {
    ChCascadeExtrusionFace face;
    face.wires = wires;
    face.holes = holes;
    face.thickness = thickness;
    face.density = density;
    face.visualization = vis_params;
    face.collide = collide;
    face.material = mat;
    faces.push_back(face);
    UpdateCollisionAndVisualizationShapes();
}

void ChCascadeBodyEasyProfile::ClearProfiles() {
    faces.clear();
    UpdateCollisionAndVisualizationShapes();
}

void ChCascadeBodyEasyProfile::UpdateCollisionAndVisualizationShapes() {
    chrono::utils::CompositeInertia inertia_composer;

    TopoDS_Compound mcompound;
    TopoDS_Builder builder;
    builder.MakeCompound(mcompound);

    for (auto& face : faces) {
        BRepBuilderAPI_MakeFace facebuilder(FromChronoPathToCascadeWire(face.wires[0]));

        bool first = true;
        for (auto profile : face.wires) {
            if (first) {
                first = false;
                continue;  // first already added in BRepBuilderAPI_MakeFace constructor
            }
            auto cascade_wire = FromChronoPathToCascadeWire(profile);
            facebuilder.Add(cascade_wire);
        }
        for (auto profilehole : face.holes) {
            auto cascade_wire_hole = FromChronoPathToCascadeWire(profilehole);
            cascade_wire_hole.Reverse();
            facebuilder.Add(cascade_wire_hole);
        }

        gp_Pnt starting_point(0., 0., 0.);
        gp_Pnt end_point(0., 0., face.thickness);
        gp_Vec vec(starting_point, end_point);

        auto prism = BRepPrimAPI_MakePrism(facebuilder.Face(), vec).Shape();

        // Add a visualization asset if needed
        if (face.visualization) {
            auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
            ChCascadeMeshTools::fillTriangleMeshFromCascade(*trimesh, prism, *face.visualization);

            auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
            trimesh_shape->SetMesh(trimesh);
            this->AddVisualShape(trimesh_shape);
        }

        prism.Location(TopLoc_Location());  // needed?

        chrono::ChVector<> cog;
        chrono::ChVector<> inertiaXX;
        chrono::ChVector<> inertiaXY;
        double vol;
        double mass;
        chrono::cascade::ChCascadeDoc::GetVolumeProperties(prism, face.density, cog, inertiaXX, inertiaXY, vol, mass);

        ChMatrix33<> inertia;
        inertia(0, 0) = inertiaXX.x();
        inertia(1, 1) = inertiaXX.y();
        inertia(2, 2) = inertiaXX.z();

        inertia(0, 1) = inertiaXY.x();
        inertia(0, 2) = inertiaXY.y();
        inertia(1, 2) = inertiaXY.z();
        inertia(1, 0) = inertiaXY.x();
        inertia(2, 0) = inertiaXY.y();
        inertia(2, 1) = inertiaXY.z();
        inertia_composer.AddComponent(ChFrame<>(cog), mass, inertia);

        builder.Add(mcompound, prism);
    }

    chrono::ChFrame<>* user_ref_to_abs = 0;  // as parameter?
    chrono::ChFrame<> frame_ref_to_abs;

    if (!user_ref_to_abs) {
        TopLoc_Location loc_shape_to_abs = mcompound.Location();
        chrono::cascade::ChCascadeDoc::FromCascadeToChrono(loc_shape_to_abs, frame_ref_to_abs);
    } else {
        frame_ref_to_abs = *user_ref_to_abs;
    }

    // Reset shape location to local ref csys (identity).
    this->topods_shape = mcompound;
    this->topods_shape.Location(TopLoc_Location());

    if (faces.size())
        this->SetDensity(
            (float)this->faces[0].density);  // anyway has no sound meaning...  densities could be different if
                                             // cluster of  different profiles with different densities..

    // Set the total mass and total inertia

    this->SetMass(inertia_composer.GetMass());
    ChVector<> m_inertiaXX(inertia_composer.GetInertia()(0, 0), inertia_composer.GetInertia()(1, 1),
                           inertia_composer.GetInertia()(2, 2));
    ChVector<> m_inertiaXY(inertia_composer.GetInertia()(0, 1), inertia_composer.GetInertia()(0, 2),
                           inertia_composer.GetInertia()(1, 2));
    this->SetInertiaXX(m_inertiaXX);
    this->SetInertiaXY(m_inertiaXY);

    // this->SetFrame_REF_to_abs(frame_ref_to_abs); //not needed

    chrono::ChFrame<> frame_cog_to_ref;
    frame_cog_to_ref.SetPos(inertia_composer.GetCOM());
    frame_cog_to_ref.SetRot(chrono::QUNIT);
    this->SetFrame_COG_to_REF(frame_cog_to_ref);

    // Add a collision shape if needed

    GetCollisionModel()->ClearModel();
    bool somefacecollide = false;

    for (auto& chface : this->faces) {
        if (chface.collide) {
            assert(chface.material);
            for (auto mpath : chface.wires) {
                ChVector<> pathposz;
                mpath->Evaluate(pathposz, 0.0);  // for offset along Z
                GetCollisionModel()->Add2Dpath(chface.material, mpath,
                                               ChVector<>(0, 0, pathposz.z() + chface.thickness * 0.5), ChMatrix33<>(1),
                                               chface.thickness * 0.99);
            }
            for (auto mhole : chface.holes) {
                ChVector<> pathposz;
                mhole->Evaluate(pathposz, 0.0);  // for offset along Z
                GetCollisionModel()->Add2Dpath(chface.material, mhole,
                                               ChVector<>(0, 0, pathposz.z() + chface.thickness * 0.5), ChMatrix33<>(1),
                                               chface.thickness * 0.99);
            }
            somefacecollide = true;
        }
    }
    GetCollisionModel()->BuildModel();
    if (somefacecollide)
        SetCollide(true);
    else
        SetCollide(false);
}

const TopoDS_Wire ChCascadeBodyEasyProfile::FromChronoPathToCascadeWire(
    std::shared_ptr<::chrono::geometry::ChLinePath> profile) {
    BRepBuilderAPI_MakeWire mwirebuilder;
    for (size_t i = 0; i < profile->GetSubLinesCount(); ++i) {
        if (auto msegment = std::dynamic_pointer_cast<::chrono::geometry::ChLineSegment>(profile->GetSubLineN(i))) {
            if (msegment->pA.z() != msegment->pB.z())
                throw ChException(
                    "Error! ChCascadeBodyEasyProfile: sub segment of ChLinePath not parallel to XY plane!");

            gp_Pnt aPntA(msegment->pA.x(), msegment->pA.y(), msegment->pA.z());
            gp_Pnt aPntB(msegment->pB.x(), msegment->pB.y(), msegment->pB.z());
            Handle(Geom_TrimmedCurve) aSegment1 = GC_MakeSegment(aPntA, aPntB);

            TopoDS_Edge aEdge1 = BRepBuilderAPI_MakeEdge(aSegment1);

            mwirebuilder.Add(aEdge1);

        } else if (auto marc = std::dynamic_pointer_cast<::chrono::geometry::ChLineArc>(profile->GetSubLineN(i))) {
            if ((marc->origin.rot.e1() != 0) || (marc->origin.rot.e2() != 0))
                throw ChException("Error! ChCascadeBodyEasyProfile: a sub arc of ChLinePath not parallel to XY plane!");

            gp_Circ aCirc(
                gp_Ax2(gp_Pnt(marc->origin.pos.x(), marc->origin.pos.y(), marc->origin.pos.z()), gp_Dir(0., 0., 1.)),
                marc->radius);

            // note the reversal of angle2 angle1 respect to chrono ChLineArc: in OCC proceeds as gpCirc Y axis in
            // twist direction. In OCC sense=true means starts from 1st angle parameter,
            Handle(Geom_TrimmedCurve) aArcOfCircle;
            if (!marc->counterclockwise) {
                aArcOfCircle = GC_MakeArcOfCircle(aCirc, marc->angle2, marc->angle1, false);
            } else {
                aArcOfCircle = GC_MakeArcOfCircle(aCirc, marc->angle1, marc->angle2, true);
            }

            TopoDS_Edge aEdge = BRepBuilderAPI_MakeEdge(aArcOfCircle);

            mwirebuilder.Add(aEdge);

        } else {
            throw ChException(
                "Error! ChCascadeBodyEasyProfile: ChLinePath must contain only ChLineArc and/or ChLineSegment.");
        }
    }

    if (!mwirebuilder.IsDone()) {
        throw ChException("Error! ChCascadeBodyEasyProfile: profile is not closed.");
    }

    return mwirebuilder.Wire();
}

}  // end namespace cascade
}  // end namespace chrono
