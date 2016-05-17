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
// Authors: Justin Madsen
// =============================================================================
//
// Generates a track chain aropund the track system
//
// =============================================================================

#include <cstdio>

#include "TrackChain.h"

#include "physics/ChBody.h"

#include "assets/ChCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
#include "assets/ChAssetLevel.h"

#include "geometry/ChTriangleMeshSoup.h"

#include "utils/ChUtilsInputOutput.h"

#include "subsys/ChVehicleModelData.h"
#include "subsys/collision/CollisionReporters.h"

namespace chrono {

// static variables
const ChVector<> TrackChain::m_COM = ChVector<>(0., 0., 0.);  // location of COM, relative to REF (e.g, geomtric center)
const ChVector<> TrackChain::m_shoe_box(0.152, 0.0663, 0.38);  // length, height, width   (0.205, 0.0663, 0.38)
const double TrackChain::m_pin_width = 0.531;                  // total width of cylinder pinseach
const double TrackChain::m_pin_dist = 0.15162;  // .205; // linear distance between a shoe chain spacing. exact = 0.205
const double TrackChain::m_pin_radius = 0.02317;
const double TrackChain::m_pin_COG_offset = -0.07581;           // local c-sys x-dir offset from body center
const ChVector<> TrackChain::m_tooth_box(0.06, 0.07967, 0.06);  // length, height, width
const double TrackChain::m_tooth_COG_offset = -0.07313;         // , -0.0731.  local c-sys y-dir offset from body center
// distance between body center and the vertical offset to the inner-surface of the collision geometry
//  used for initializing shoes as a chain
const double TrackChain::m_shoe_chain_Yoffset = 0.04;  // .03315 exact

TrackChain::TrackChain(const std::string& name,
                       VisualizationType::Enum vis,
                       CollisionType::Enum collide,
                       size_t chainSys_idx,
                       double shoe_mass,
                       const ChVector<>& shoeIxx,
                       double mu)
    : m_vis(vis),
      m_collide(collide),
      m_chainSys_idx(chainSys_idx),
      m_mass(shoe_mass),
      m_inertia(shoeIxx),
      m_mu(mu),
      m_collisionFile(vehicle::GetDataFile("M113/shoe_collision.obj")),
      m_meshFile(vehicle::GetDataFile("M113/shoe_view.obj")),
      m_meshName("M113 shoe"),
      m_numShoes(0),
      m_use_custom_damper(false) {
    // clear vector holding list of body handles
    m_shoes.clear();
    // add first track shoe body
    m_shoes.push_back(ChSharedPtr<ChBody>(new ChBody));
    // m_shoes.push_back(ChSharedPtr<ChBodyAuxRef>(new ChBodyAuxRef));
    // m_shoes[0]->SetFrame_COG_to_REF(ChFrame<>(m_COM,QUNIT));

    m_numShoes++;

    std::stringstream shoe_s;
    shoe_s << "shoe" << m_numShoes;
    m_shoes.front()->SetNameString(shoe_s.str());
    m_shoes.front()->SetMass(m_mass);
    m_shoes.front()->SetInertiaXX(m_inertia);

    // Attach visualization to the base track shoe
    AddVisualization(0, true, "cubetexture_pinkwhite.png");

    // Init. some helper debugging, reporter variables
    m_SG_info.resize(2, ChVector<>());
    m_SG_is_persistentContact_set.resize(2, false);
    m_SG_PosRel.resize(2, ChVector<>());
    m_SG_PosAbs.resize(2, ChVector<>());
    m_SG_Fn.resize(2, ChVector<>());
}

// figure out the control points, 2 per rolling element to wrap the chain around.
void TrackChain::Initialize(ChSharedPtr<ChBody> chassis,
                            const ChFrame<>& chassis_REF,
                            const std::vector<ChVector<> >& rolling_element_loc,
                            const std::vector<double>& clearance,
                            const std::vector<ChVector<> >& spin_axis,
                            const ChVector<>& start_loc) {
    assert(rolling_element_loc.size() == clearance.size());
    // get the following in abs coords: 1) start_loc, 2) rolling_elem, 3) control_points
    m_start_loc_bar = start_loc;  // keep track of start_loc in chassis ref frame
    ChFrame<> start_to_abs(start_loc);
    start_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

    // find control points, which lie on the envelope of each rolling element, in abs coords
    std::vector<ChFrame<> > control_to_abs;
    std::vector<ChFrame<> > rolling_to_abs;

    size_t num_elem = rolling_element_loc.size();  // number of control points

    // define the envelope to wrap the chain around using 1) body locs and clearances, 2) control_points.
    // each rolling element has 2 control points, a start and end point for a given envenlope line segment.
    ChVector<> start_point;  // current start_loc for this segment
    ChVector<> end_point;    // end point of the current segment
    ChVector<> rad_dir;      // center to segment start/end point on rolling elments
    ChVector<> r_21;         // vector between two pulley centerpoints
    // ChVector<> lateral_dir;  // norm = r_12 cross r_32

    // iterate over the line segments, first segment is between start_loc and rolling_elem 0
    // last segment  is between rolling_elem[last] and rolling_elem[last-1]
    for (size_t i = 0; i < num_elem; i++) {
        // convert the center of the rolling body to abs coords
        rolling_to_abs.push_back(ChFrame<>(rolling_element_loc[i]));
        rolling_to_abs[i].ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

        // start and end points of line segment are found (in chassis ref system)
        if (i == 0) {
            // first point, know start_point off the bat.
            start_point = start_loc;

            // this is where the assumption that start_loc is on the top of the chain matters,
            // no first pulley to use to find r_21
            ChVector<> top_center = rolling_element_loc[i];
            top_center.y += clearance[i];
            r_21 = top_center - start_loc;  // first guess at start and end points
            ChVector<> tan_dir = r_21.GetNormalized();

            // If start_loc is precisely located, this rad_dir will define the end point as it is now.
            rad_dir = top_center - rolling_element_loc[i];
            rad_dir.Normalize();
            // If start_loc isn't exactly placed, r_21 won't be tangent to the envelope at point top_center.
            // Enforce that the vector between the start and end points is tangent.
            if (abs(Vdot(tan_dir, rad_dir)) > 1e-3) {
                // move top_center so r_21 is orthogonal to rad_dir
                // i.e., rad_dir dot r_21 = 0
                // easier to figure out the relative angle rad_dir needs to be rotated about the norm axis.
                ChVector<> start_cen = start_loc - rolling_element_loc[i];
                double theta = std::acos(Vdot(start_cen.GetNormalized(), rad_dir));  // angle between direction vectors
                double seglen = std::sqrt(start_cen.Length2() - pow(clearance[i], 2));
                double h = start_cen.Length();
                double phi = std::asin(seglen / h);  // what theta should be
                double rot_ang = theta - phi;

                // find the radial direction from the center of adjacent rolling elements
                // for seg i, norm = r12 x r32
                // lateral_dir = Vcross(rolling_element_loc.back() - rolling_element_loc[0], rolling_element_loc[1] -
                // rolling_element_loc[0]);
                // no real reason why this wouldn't be the z-axis of the rolling element body???

                // rotate rad_dir about norm axis
                ChQuaternion<> rot_q = Q_from_AngAxis(rot_ang, spin_axis[i].GetNormalized());
                ChFrame<> rad_dir_frame(rad_dir, QUNIT);
                rad_dir = (rot_q * rad_dir_frame).GetPos();
            }
            // rad_dir is now tangent to r_21, so define the new endpoint of this segment
            end_point = rolling_element_loc[i] + rad_dir * clearance[i];

        } else {
            // intermediate points, find start and end from roller elem locations
            // first guess at start/end points: center distance vector
            r_21 = rolling_element_loc[i] - rolling_element_loc[i - 1];

            // if the two rolling elements have the same radius, no angle of wrap.
            rad_dir = Vcross(spin_axis[i].GetNormalized(), r_21.GetNormalized());
            rad_dir.Normalize();
            // when not the same size, find the angle of pulley wrap.
            // Probably overkill, but check that the radial vector is ortho. to r_21
            if (abs(clearance[i] - clearance[i - 1]) > 1.0e-3 || abs(Vdot(r_21.GetNormalized(), rad_dir)) > 1.0e-3) {
                // pulley wrap angle, a = (r2-r1)/center_len
                double alpha = asin((clearance[i] - clearance[i - 1]) / r_21.Length());
                ChQuaternion<> alpha_rot = Q_from_AngAxis(alpha, spin_axis[i].GetNormalized());
                ChFrame<> rot_frame(ChVector<>(), alpha_rot);
                // rotate rad_dir the angle of wrap about the normal axis
                rad_dir = rad_dir >> rot_frame;
            }

            // with a radial direction vector, start/end points are easy
            start_point = rolling_element_loc[i - 1] + rad_dir * clearance[i - 1];
            end_point = rolling_element_loc[i] + rad_dir * clearance[i];
        }

        // with starting and end points in local frame, push to abs frame
        if (i == 0) {
            // first segment, only use the end_point
            control_to_abs.push_back(ChFrame<>(end_point));
            control_to_abs[0].ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());
        } else {
            // intermediate segments, use both start and end points
            control_to_abs.push_back(ChFrame<>(start_point));
            control_to_abs.back().ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());
            control_to_abs.push_back(ChFrame<>(end_point));
            control_to_abs.back().ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());
        }
    }

    // there is 1 more line segment between the last roller and the start_loc
    // know end_point off the bat
    end_point = start_point;

    // no second pulley to use to find r_21
    ChVector<> top_center = rolling_element_loc.back();
    top_center.y += clearance.back();  // top center of the last rolling element
    r_21 = start_loc - top_center;     // first guess at start and end points
    ChVector<> tan_dir = r_21.GetNormalized();

    // If start_loc is precisely located, this rad_dir will define the end point as it is now.
    rad_dir = top_center - rolling_element_loc.back();
    rad_dir.Normalize();
    // If start_loc isn't exactly placed, r_21 won't be tangent to the envelope at point top_center.
    // Enforce that the vector between the start and end points is tangent.
    if (abs(Vdot(tan_dir, rad_dir)) > 1e-3) {
        // move top_center so r_21 is orthogonal to rad_dir
        // i.e., rad_dir dot r_21 = 0
        // easier to figure out the relative angle rad_dir needs to be rotated about the norm axis.
        ChVector<> start_cen = start_loc - rolling_element_loc.back();
        double theta = std::acos(Vdot(start_cen.GetNormalized(), rad_dir));  // angle between direction vectors
        double seglen = std::sqrt(start_cen.Length2() - pow(clearance.back(), 2));
        double h = start_cen.Length();
        double phi = std::asin(seglen / h);  // what theta should be
        double rot_ang = phi - theta;  // since we're looking at it backwards compared to the first rolling element

        // find the radial direction from the center of adjacent rolling elements
        // for seg i, norm = r12 x r32
        // norm_dir = Vcross(rolling_element_loc.back() - rolling_element_loc[0], rolling_element_loc[num_elem-2] -
        // rolling_element_loc.back() );
        // norm_dir.Normalize();
        // assert(norm_dir.Length() == 1);

        // rotate rad_dir about norm axis
        ChFrame<> rot_frame(ChVector<>(), Q_from_AngAxis(rot_ang, spin_axis.back().GetNormalized()));
        rad_dir = rad_dir >> rot_frame;
    }
    // rad_dir is now tangent to r_21, so define the new endpoint of this segment
    start_point = rolling_element_loc.back() + rad_dir * clearance.back();

    // last segment, only use the start_point
    control_to_abs.push_back(ChFrame<>(start_point));
    (control_to_abs.back()).ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

    // "wrap" the track chain around the trackSystem rolling elements, e.g., drive-gear,
    // idler, road-wheels. First and last shoes are allowed to be in any orientation,
    // as long as the final pin joint connects correctly.
    CreateChain(chassis, chassis->GetFrame_REF_to_abs(), control_to_abs, rolling_to_abs, clearance,
                start_to_abs.GetPos());
}

void TrackChain::AddVisualization() {
    assert(m_numShoes > 0);
    AddVisualization(m_numShoes - 1);
}

// use a custom texture, if desired.
void TrackChain::AddVisualization(size_t track_idx, bool custom_texture, const std::string& tex_name) {
    assert(track_idx < m_numShoes);

    if (m_shoes[track_idx]->GetAssets().size() > 0)
        m_shoes[track_idx]->GetAssets().clear();

    // Attach visualization asset
    switch (m_vis) {
        case VisualizationType::Primitives: {
            // color the boxes and cylinders differently
            ChSharedPtr<ChAssetLevel> boxLevel(new ChAssetLevel);
            ChSharedPtr<ChAssetLevel> pinLevel(new ChAssetLevel);

            // shoes will be added to the same collision family so self-collision can be toggled
            ChSharedPtr<ChBoxShape> box(new ChBoxShape);
            box->GetBoxGeometry().SetLengths(ChVector<>(m_shoe_box.x, m_shoe_box.y, m_shoe_box.z));
            box->GetBoxGeometry().Pos = ChVector<>(0, 0, 0);
            boxLevel->AddAsset(box);

            // add the tooth box
            ChSharedPtr<ChBoxShape> tooth_box(new ChBoxShape);
            tooth_box->GetBoxGeometry().SetLengths(ChVector<>(m_tooth_box.x, m_tooth_box.y, m_tooth_box.z));
            tooth_box->GetBoxGeometry().Pos = ChVector<>(0, m_tooth_COG_offset, 0);
            boxLevel->AddAsset(tooth_box);

            // add a color to the shoes
            ChSharedPtr<ChTexture> box_tex(new ChTexture);
            if (custom_texture)
                box_tex->SetTextureFilename(GetChronoDataFile(tex_name));
            else
                box_tex->SetTextureFilename(GetChronoDataFile("blu.png"));
            boxLevel->AddAsset(box_tex);

            // finally, add the box asset level to the shoe
            m_shoes[track_idx]->AddAsset(boxLevel);

            // add the pin as a single cylinder
            ChSharedPtr<ChCylinderShape> pin(new ChCylinderShape);
            pin->GetCylinderGeometry().p1 = ChVector<>(m_pin_COG_offset, 0, m_pin_width / 2.0);
            pin->GetCylinderGeometry().p2 = ChVector<>(m_pin_COG_offset, 0, -m_pin_width / 2.0);
            pin->GetCylinderGeometry().rad = m_pin_radius;
            pinLevel->AddAsset(pin);

            // add a color to the pin
            ChSharedPtr<ChColorAsset> pinCol(new ChColorAsset);
            // blue, or grey
            if (track_idx % 2 == 0)
                pinCol->SetColor(ChColor(0.3f, 0.3f, 0.7f));
            else
                pinCol->SetColor(ChColor(0.2f, 0.2f, 0.2f));

            pinLevel->AddAsset(pinCol);

            // finally, add the pin asset level to the shoe
            m_shoes[track_idx]->AddAsset(pinLevel);

            break;
        }
        case VisualizationType::CompoundPrimitives: {
            // use same set of primitives as was used for the corresponding collsion shape
            // shoe geometry provided can be exactly represented by 5 smaller boxes, 2 cylinders.
            // tooth is also a box
            double subBox_width = m_shoe_box.z / 5.0;
            double stagger_offset = 0.03;

            // color the boxes and cylinders differently
            ChSharedPtr<ChAssetLevel> boxLevel(new ChAssetLevel);
            ChSharedPtr<ChAssetLevel> pinLevel(new ChAssetLevel);

            // 5 smaller boxes make up the base of the shoe
            ChSharedPtr<ChBoxShape> box1(new ChBoxShape);
            box1->GetBoxGeometry().SetLengths(ChVector<>(m_shoe_box.x, m_shoe_box.y, subBox_width));
            boxLevel->AddAsset(box1);

            ChSharedPtr<ChBoxShape> box2(new ChBoxShape(*box1.get_ptr()));
            box2->GetBoxGeometry().Pos = ChVector<>(-stagger_offset, 0, subBox_width);
            boxLevel->AddAsset(box2);

            ChSharedPtr<ChBoxShape> box3(new ChBoxShape(*box1.get_ptr()));
            box3->GetBoxGeometry().Pos = ChVector<>(-stagger_offset, 0, -subBox_width);
            boxLevel->AddAsset(box3);

            ChSharedPtr<ChBoxShape> box4(new ChBoxShape(*box1.get_ptr()));
            box4->GetBoxGeometry().Pos = ChVector<>(0, 0, 2.0 * subBox_width);
            boxLevel->AddAsset(box4);

            ChSharedPtr<ChBoxShape> box5(new ChBoxShape(*box1.get_ptr()));
            box5->GetBoxGeometry().Pos = ChVector<>(0, 0, -2.0 * subBox_width);
            boxLevel->AddAsset(box5);

            // add the tooth box
            ChSharedPtr<ChBoxShape> tooth_box(new ChBoxShape);
            tooth_box->GetBoxGeometry().SetLengths(ChVector<>(m_tooth_box.x, m_tooth_box.y, m_tooth_box.z));
            tooth_box->GetBoxGeometry().Pos = ChVector<>(0, m_tooth_COG_offset, 0);
            boxLevel->AddAsset(tooth_box);

            // add a color to the shoes
            ChSharedPtr<ChTexture> box_tex(new ChTexture);
            if (custom_texture)
                box_tex->SetTextureFilename(GetChronoDataFile(tex_name));
            else
                box_tex->SetTextureFilename(GetChronoDataFile("blu.png"));
            boxLevel->AddAsset(box_tex);

            // finally, add the box asset level to the shoe
            m_shoes[track_idx]->AddAsset(boxLevel);

            // add the pin as a single cylinder
            ChSharedPtr<ChCylinderShape> pin(new ChCylinderShape);
            pin->GetCylinderGeometry().p1 = ChVector<>(m_pin_COG_offset, 0, m_pin_width / 2.0);
            pin->GetCylinderGeometry().p2 = ChVector<>(m_pin_COG_offset, 0, -m_pin_width / 2.0);
            pin->GetCylinderGeometry().rad = m_pin_radius;
            pinLevel->AddAsset(pin);

            // add a color to the pin
            ChSharedPtr<ChColorAsset> pinCol(new ChColorAsset);
            // blue, or grey
            if (track_idx % 2 == 0)
                pinCol->SetColor(ChColor(0.3f, 0.3f, 0.7f));
            else
                pinCol->SetColor(ChColor(0.2f, 0.2f, 0.2f));

            pinLevel->AddAsset(pinCol);

            // finally, add the pin asset level to the shoe
            m_shoes[track_idx]->AddAsset(pinLevel);

            break;
        }
        case VisualizationType::Mesh: {
            // mesh for visualization only.
            geometry::ChTriangleMeshConnected trimesh;
            trimesh.LoadWavefrontMesh(GetMeshFile(), false, false);

            ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
            trimesh_shape->SetMesh(trimesh);
            trimesh_shape->SetName(GetMeshName());
            m_shoes[track_idx]->AddAsset(trimesh_shape);

            // add a color to the shoes
            if (custom_texture) {
                ChSharedPtr<ChTexture> box_tex(new ChTexture);
                box_tex->SetTextureFilename(GetChronoDataFile(tex_name));
                m_shoes[track_idx]->AddAsset(box_tex);
            } else {
                ChSharedPtr<ChColorAsset> mcolor(new ChColorAsset(0.3f, 0.3f, 0.3f));
                m_shoes[track_idx]->AddAsset(mcolor);
            }

            break;
        }
        default: { GetLog() << "Didn't recognize VisualizationType for TrackChain \n"; }
    }  // end switch
}

void TrackChain::AddCollisionGeometry(VehicleSide side, double mu, double mu_sliding, double mu_roll, double mu_spin) {
    assert(m_numShoes > 0);
    AddCollisionGeometry(side, m_numShoes - 1, mu, mu_sliding, mu_roll, mu_spin);
}

void TrackChain::AddCollisionGeometry(VehicleSide side,
                                      size_t track_idx,
                                      double mu,
                                      double mu_sliding,
                                      double mu_roll,
                                      double mu_spin) {
    assert(track_idx < m_numShoes);
    // add collision geometrey to the chassis, if enabled. Warn if disabled
    if (m_collide == CollisionType::None) {
        GetLog() << " !!! track shoe # " << track_idx << " collision deactivated !!! \n\n";
        m_shoes[track_idx]->SetCollide(false);
        return;
    }

    m_shoes[track_idx]->SetCollide(true);
    m_shoes[track_idx]->GetCollisionModel()->ClearModel();

    m_shoes[track_idx]->GetCollisionModel()->SetSafeMargin(0.002);  // inward safe margin
    m_shoes[track_idx]->GetCollisionModel()->SetEnvelope(0.004);    // distance of the outward "collision envelope"

    // set the collision material
    m_shoes[track_idx]->GetMaterialSurface()->SetSfriction(mu);
    m_shoes[track_idx]->GetMaterialSurface()->SetKfriction(mu_sliding);
    m_shoes[track_idx]->GetMaterialSurface()->SetRollingFriction(mu_roll);
    m_shoes[track_idx]->GetMaterialSurface()->SetSpinningFriction(mu_spin);

    switch (m_collide) {
        case CollisionType::Primitives: {
            // use a simple box for the shoe
            m_shoes[track_idx]->GetCollisionModel()->AddBox(0.5 * m_shoe_box.x, 0.5 * m_shoe_box.y, 0.5 * m_shoe_box.z);

            // add the tooth
            m_shoes[track_idx]->GetCollisionModel()->AddBox(0.5 * m_tooth_box.x, 0.5 * m_tooth_box.y,
                                                            0.5 * m_tooth_box.z, ChVector<>(0, m_tooth_COG_offset, 0));

            // pin is a single cylinder
            m_shoes[track_idx]->GetCollisionModel()->AddCylinder(m_pin_radius, m_pin_radius, m_pin_width / 2.0,
                                                                 ChVector<>(m_pin_COG_offset, 0, 0),
                                                                 ChMatrix33<>(Q_from_AngAxis(CH_C_PI_2, VECT_X)));

            break;
        }
        case CollisionType::CompoundPrimitives: {
            // shoe geometry provided can be exactly represented by 6 smaller boxes, 2 cylinders.
            double subBox_width = 0.5 * 0.082;
            double stagger_offset = 0.03;

            // 5 smaller boxes make up the base of the shoe. Use half-lengths w/ collision shape Adds.
            // middle 2 boxes are shifted back slightly
            m_shoes[track_idx]->GetCollisionModel()->AddBox(0.5 * m_shoe_box.x, 0.5 * m_shoe_box.y, subBox_width);
            m_shoes[track_idx]->GetCollisionModel()->AddBox(0.5 * m_shoe_box.x, 0.5 * m_shoe_box.y, subBox_width,
                                                            ChVector<>(-stagger_offset, 0, subBox_width));
            m_shoes[track_idx]->GetCollisionModel()->AddBox(0.5 * m_shoe_box.x, 0.5 * m_shoe_box.y, subBox_width,
                                                            ChVector<>(-stagger_offset, 0, -subBox_width));
            m_shoes[track_idx]->GetCollisionModel()->AddBox(0.5 * m_shoe_box.x, 0.5 * m_shoe_box.y, subBox_width,
                                                            ChVector<>(0, 0, 2.0 * subBox_width));
            m_shoes[track_idx]->GetCollisionModel()->AddBox(0.5 * m_shoe_box.x, 0.5 * m_shoe_box.y, subBox_width,
                                                            ChVector<>(0, 0, -2.0 * subBox_width));

            // add the tooth box
            m_shoes[track_idx]->GetCollisionModel()->AddBox(0.5 * m_tooth_box.x, 0.5 * m_tooth_box.y,
                                                            0.5 * m_tooth_box.z, ChVector<>(0, m_tooth_COG_offset, 0));

            // add the pin as a single cylinder
            m_shoes[track_idx]->GetCollisionModel()->AddCylinder(m_pin_radius, m_pin_radius, m_pin_width / 2.0,
                                                                 ChVector<>(m_pin_COG_offset, 0, 0),
                                                                 ChMatrix33<>(Q_from_AngAxis(CH_C_PI_2, VECT_X)));

            break;
        }
        case CollisionType::Mesh: {
            // use a triangle mesh

            geometry::ChTriangleMeshSoup temp_trianglemesh;

            // TODO: fill the triangleMesh here with some track shoe geometry

            // is there an offset??
            double shoelength = 0.2;
            ChVector<> mesh_displacement(shoelength * 0.5, 0, 0);  // since mesh origin is not in body center of mass
            m_shoes[track_idx]->GetCollisionModel()->AddTriangleMesh(temp_trianglemesh, false, false,
                                                                     mesh_displacement);

            break;
        }
        case CollisionType::ConvexHull: {
            // use convex hulls, loaded from file
            ChStreamInAsciiFile chull_file(GetChronoDataFile("track_data/M113/shoe_collision.chulls").c_str());
            // transform the collision geometry as needed
            double mangle = 45.0;  // guess
            ChQuaternion<> rot;
            rot.Q_from_AngAxis(mangle * (CH_C_PI / 180.), VECT_X);
            ChMatrix33<> rot_offset(rot);
            ChVector<> disp_offset(0, 0, 0);  // no displacement offset
            m_shoes[track_idx]->GetCollisionModel()->AddConvexHullsFromFile(chull_file, disp_offset, rot_offset);
            break;
        }
        default:
            // no collision shape
            GetLog() << "not recognized CollisionType: " << (int)m_collide << " for shoe # " << track_idx << "\n";
            m_shoes[track_idx]->SetCollide(false);
            return;
    }  // end switch

    // set collision family, according to which side the shoe is on
    if (side == RIGHTSIDE) {
        m_shoes[track_idx]->GetCollisionModel()->SetFamily((int)CollisionFam::ShoeRight);
    } else {
        m_shoes[track_idx]->GetCollisionModel()->SetFamily((int)CollisionFam::ShoeLeft);
    }

    // don't collide with other shoes, but with everything else
    m_shoes[track_idx]->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::ShoeRight);
    m_shoes[track_idx]->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::ShoeLeft);
    m_shoes[track_idx]->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::Hull);

    m_shoes[track_idx]->GetCollisionModel()->BuildModel();
}

// two control points per rolling body.
// each two control points correspond to a single rolling element & clearance value, in the same order.
void TrackChain::CreateChain(ChSharedPtr<ChBody> chassis,
                             const ChFrame<>& chassis_REF,
                             const std::vector<ChFrame<> >& control_points_abs,
                             const std::vector<ChFrame<> >& rolling_element_abs,
                             const std::vector<double>& clearance,
                             const ChVector<>& start_pos_abs) {
    // Each segment has 2 parts, the initial linear portion, then wrapping around
    //   the rolling element the start point of the next linear line segment.
    size_t num_elems = clearance.size();
    // Keep track of the current position along the line segment as new shoe bodies are created.
    // position along line defined by projecting the shoe body normal at the shoe COM down to the linear line segment,
    // or toward the center of the rolling element.
    ChVector<> curr_pos = start_pos_abs;
    for (int idx = 0; idx < num_elems; idx++) {
        ChVector<> start_seg;  // start and end point of the line segment
        if (idx == 0)
            start_seg = start_pos_abs;
        else
            start_seg = control_points_abs[2 * (idx - 1) + 1].GetPos();
        ChVector<> end_seg = control_points_abs[2 * idx].GetPos();        // end of line seg.
        ChVector<> end_curve = control_points_abs[2 * idx + 1].GetPos();  // end of curved section
        // build the bodies for this line segment and rolling element curved section
        CreateShoes(chassis, chassis->GetFrame_REF_to_abs(), start_seg, end_seg, end_curve,
                    rolling_element_abs[idx].GetPos(), clearance[idx]);
    }
    // this will get us around the last rolling element curved segment. Add the final length of shoes
    CreateShoes_closeChain(chassis, chassis->GetFrame_REF_to_abs(), control_points_abs.back().GetPos(), start_pos_abs);
}

// all locations are in the abs ref. frame.
// Some assumptions:
// Body orientation and axis of pin rotation always in the lateral_dir
void TrackChain::CreateShoes(ChSharedPtr<ChBody> chassis,
                             const ChFrame<>& chassis_REF,
                             const ChVector<>& start_seg,
                             const ChVector<>& end_seg,
                             const ChVector<>& end_curve_seg,
                             const ChVector<>& rolling_elem_center,
                             double clearance) {
    // get coordinate directions of the linear segment, in the global c-sys.
    // lateral in terms of the vehicle chassis
    ChVector<> lateral_dir;
    if (m_numShoes == 1)
        lateral_dir = Vcross(start_seg - end_seg, end_curve_seg - end_seg).GetNormalized();
    else
        lateral_dir = m_shoes.back()->GetRot().GetZaxis();

    ChVector<> tan_dir = (end_seg - start_seg).GetNormalized();
    ChVector<> norm_dir = Vcross(lateral_dir, tan_dir).GetNormalized();  // normal to the envelope surface
    ChMatrix33<> rot_seg_A;                                              // orientation of line segment
    rot_seg_A.Set_A_axis(tan_dir, norm_dir, lateral_dir);

    ChVector<> pos_on_seg;  // current position on the boundary, e.g. the line segment.
    ChVector<> shoe_pos;    // current shoe position, abs coords.

    // special case: very first shoe
    // A single ChBody shoe is created upon construction, so it be can copied by the other shoes.
    // add 1) collision geometry, 2) set init. pos/rot, 3) add to system.
    if (m_numShoes == 1) {
        pos_on_seg = start_seg;  // special case
        shoe_pos = pos_on_seg + norm_dir * m_shoe_chain_Yoffset;
        m_shoes.front()->SetPos(shoe_pos);
        m_shoes.front()->SetRot(rot_seg_A);  // can assume its the same as the line segment

        // add collision geometry
        (m_start_loc_bar.z < 0) ? AddCollisionGeometry(LEFTSIDE, m_mu, 0.9*m_mu) : AddCollisionGeometry(RIGHTSIDE, m_mu, 0.9*m_mu);
        chassis->GetSystem()->AddBody(m_shoes.front());
        // First shoe on first line segment should always be aligned.
        m_aligned_with_seg = true;
    } else {
        // normal height above envelope surface of last place shoe
        double h = Vdot(norm_dir, m_shoes.back()->GetPos() - start_seg);
        // point projected on the surface of the envelope
        pos_on_seg = m_shoes.back()->GetPos() - norm_dir * h;
    }

    // First, create the shoes along the linear segment. Connect by pins via revolute constraints.
    double dist_to_end = Vdot(end_seg - pos_on_seg, tan_dir);

    ChFrame<> COG_frame;
    ChFrame<> pin_frame;  // between this shoe and the last one
    // pin is just forward of the COG, and vice-versa
    ChVector<> COG_to_pin_rel(m_pin_dist / 2.0, 0, 0);

    // keep going until last created shoe COG pos_on_seg passes the end point.
    // Want to overshoot upon reaching curved segment, to not interpenetrate surface boundary.
    while (dist_to_end > 0) {
        // create a new body by copying the first, add to handle vector.
        // Copy doesn't set the collision shape.
        // Don't reset visualization assets, copied in ChPhysicsItem::Copy()
        m_shoes.push_back(ChSharedPtr<ChBody>(new ChBody));
        m_numShoes++;
        m_shoes.back()->Copy(m_shoes.front().get_ptr());
        // create the collision geometry for the new shoe
        (m_start_loc_bar.z < 0) ? AddCollisionGeometry(LEFTSIDE, m_mu, 0.9*m_mu) : AddCollisionGeometry(RIGHTSIDE, m_mu, 0.9*m_mu);

        // even shoes can be a different texture
        (m_numShoes % 2 == 0) ? AddVisualization(m_numShoes - 1, true, "spheretexture.png") : AddVisualization();

        std::stringstream shoe_s;
        shoe_s << "shoe" << m_numShoes;
        m_shoes.back()->SetNameString(shoe_s.str());

        // Find where the pin to the previous shoe should be positioned.
        // From there, calculate what the body pos/rot should be.
        // Rse the COG pos, rot of the previous shoe to get us there.

        pin_frame =
            ChFrame<>((m_shoes.end()[-2])->GetFrame_COG_to_abs() * COG_to_pin_rel, (m_shoes.end()[-2])->GetRot());
        // ChVector<> pin_pos = (m_shoes.end()[-2])->GetFrame_COG_to_abs() * COG_to_pin_rel;
        // pin_frame = ChFrame<>(pin_pos, (m_shoes.end()[-2])->GetRot() );

        // Creating shoes along the line segment, one of two situations possible:
        // 1) shoe is exactly on the envelope boundary, and exactly parallel (e.g., first segment this is guaranteed).
        // 2) shoe is off the boundary, and will have a rot slightly different than previous shoe to get it closer,
        //      e.g., after a straight segment follows a curved segment.

        // At first,COG frame assume this shoe has same orientation as the previous.
        COG_frame = ChFrame<>((m_shoes.end()[-2])->GetFrame_COG_to_abs());
        // set the body pos. from the pin_pos w/ the previous shoe;
        COG_frame.SetPos(pin_frame * COG_to_pin_rel);

        // hold on to the global direction vector between the original 1-2 pin locations.
        ChVector<> pin_dir_original = (COG_frame.GetPos() - pin_frame.GetPos()).GetNormalized();

        // Assumption 1) is true if:
        //  A) We set the last shoe pin in such a way so this shoe can be exactly aligned with the line segment, or
        //  B) the last shoe on the linear segment was already aligned
        if (m_aligned_with_seg) {
            // previous shoe has been positioned so pin location is exactly
            //  m_shoe_chain_Yoffset above the envelope surface (norm. dir).
            // Use the rotation frame for the line segment for the shoe to keep the x-axis parallel to segment.
            COG_frame.SetRot(rot_seg_A);
            // if the previous shoe x-axis is not parallel to segment, using COG_frame
            // rot from that shoe will be incorrect. Instead, use the updated pin rot.
            pin_frame.SetRot(rot_seg_A);
            // now the position can be found relative to the newly oriented pin frame.
            // doesn't change if last shoe has the same orientation.
            COG_frame.SetPos(pin_frame * COG_to_pin_rel);
        } else {
            // verify that this configuration:
            //  1) does not cross the line segment boundary, and
            //  2) stays as clsoe as possible to the line segment (e.g., by rotating the body slightly at the pin_pos)
            //  3) does not rotate the body too aggressively, by setting the following limit:
            double lim_rot_angle = CH_C_PI_4;  // don't rotate the shoe more than 45 deg. towards the line segment

            // Consider the next pin location, relative to the current pin
            ChVector<> nextPin_pos_rel(m_pin_dist, 0, 0);
            ChVector<> nextPin_pos_abs = pin_frame * nextPin_pos_rel;

            // distance the nextpin is above the line segment surface
            double nextPin_clearance = Vdot(norm_dir, nextPin_pos_abs - start_seg);

            // distance the pin position is above the line segment surface
            double pin_clearance = Vdot(norm_dir, pin_frame.GetPos() - start_seg);

            // going to need the next pin to be ::m_shoe_chain_Yoffset above the line segment.
            // normal distance between pins relative to the line segment.
            double pin_len_norm_seg = pin_clearance - m_shoe_chain_Yoffset;

            double psi = 0;  // find rotation angle of shoe relative to the last one
            /*
            if( pin_len_norm_seg > m_pin_dist * std::sin(lim_rot_angle) )
            {
              // don't rotate too aggressively
              psi = lim_rot_angle;
            }
            else
            {
            */
            // distance of pins, tangent to the line segment.
            double len_on_seg = std::sqrt(m_pin_dist * m_pin_dist - pow(pin_len_norm_seg, 2));
            // know the next pin location
            nextPin_pos_abs = pin_frame.GetPos() - norm_dir * pin_len_norm_seg + tan_dir * len_on_seg;
            // direction vector between pins now, normalized.
            ChVector<> pin_dir_modified = (nextPin_pos_abs - pin_frame.GetPos()).GetNormalized();

            // find rotation angles based on cos(psi) = pin_dir_original dot pin_dir_modified
            psi = std::acos(Vdot(pin_dir_original, pin_dir_modified));
            if (psi > lim_rot_angle)
                GetLog() << " incorrect rotation angle for shoe #: " << int(m_numShoes) << ", psi = " << psi << "\n";

            // make sure psi rotates us in the correct direction
            ChVector<> mcross = pin_dir_modified % pin_dir_original;
            if ((pin_dir_modified % pin_dir_original).z * lateral_dir.z < 0)
                psi = -psi;
            // }

            // rotate the pin frame about z-axis of the pin, at the pin
            ChQuaternion<> rot_frame = Q_from_AngAxis(-psi, pin_frame.GetRot().GetZaxis());
            pin_frame.SetRot(rot_frame * pin_frame.GetRot());

            // can find the shoe COG pos/rot now, from pin orientation, and COG offset from pin pos
            COG_frame.SetRot(pin_frame.GetRot());
            COG_frame.SetPos(pin_frame * COG_to_pin_rel);

            // let's check to see if the criteria for setting m_aligned_with_seg = true.
            // Note: the function also sets the bool
            bool check_alignment = check_shoe_aligned(COG_frame * COG_to_pin_rel, start_seg, end_seg, norm_dir);
            if (0)
                GetLog() << " aligned shoe # : " << int(m_numShoes) << " ?? " << check_alignment << "\n";
        }

        // COG_frame is set correctly, add the body to the system
        m_shoes.back()->SetPos(COG_frame.GetPos());
        m_shoes.back()->SetRot(COG_frame.GetRot());
        chassis->GetSystem()->AddBody(m_shoes.back());

        // create and init. the pin between the last shoe and this one, add to system.
        m_pins.push_back(ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute));
        std::stringstream pin_s;
        pin_s << "pin" << m_pins.size();
        m_pins.back()->SetNameString(pin_s.str());
        // pin frame picked up from COG_frame, where z-axis should be in the lateral direction.
        m_pins.back()->Initialize(m_shoes.back(), m_shoes.end()[-2],
                                  ChCoordsys<>(pin_frame.GetPos(), pin_frame.GetRot()));
        chassis->GetSystem()->AddLink(m_pins.back());

        // See if the end of the linear segment has been reached.
        //  Could use either A) or B), but B) is simpler.
        // A) step along the line in the tangent direction, for the added half-pin lengths on
        //  the last two shoes.
        ChVector<> COG1_pin = pin_frame.GetPos() - (m_shoes.end()[-2])->GetPos();
        ChVector<> pin_COG2 = COG_frame.GetPos() - pin_frame.GetPos();
        ChVector<> pos_on_seg_A = pos_on_seg + tan_dir * (Vdot(COG1_pin, tan_dir) + Vdot(pin_COG2, tan_dir));

        // B) when the COG of the last shoe passes the endpoint.
        // project the COG normal to segment surface.
        double proj_dist = Vdot(norm_dir, COG_frame.GetPos() - start_seg);
        ChVector<> pos_on_seg_B = COG_frame.GetPos() - norm_dir * proj_dist;

        // should be equal; check the largest error term. Say something if it's exceeded.
        if ((pos_on_seg_A - pos_on_seg_B).LengthInf() > 1e-5)
            GetLog() << " comparing pos_on_seg error: shoe # " << int(m_numShoes) << pos_on_seg_A - pos_on_seg_B
                     << "\n";

        // update distance using method B
        pos_on_seg = pos_on_seg_B;
        // will become negative when the last created shoe COG center is past the endpoint.
        dist_to_end = Vdot(end_seg - pos_on_seg, tan_dir);
    }

    // At this point, wrap the shoes around the curved segment (if there is one).
    //  E.g., roadwheels 1-2, 2-3, 3-4, 4-5 have no curved segment, so skip.
    if ((end_curve_seg - end_seg).LengthInf() < 1e-3) {
        // GetLog() << " no curved line segment, at location: " << end_curve_seg << " ... \n";
        return;
    }

    // It is assumed that there is a previous shoe with a pin location that is compatible.
    // tangent direction at the end of the curved segment
    ChVector<> tan_dir_end_curved_seg = Vcross(end_curve_seg - rolling_elem_center, lateral_dir).GetNormalized();

    // project the pin at the front of the shoe towards the center of the rolling element.
    ChVector<> next_pin_pos = COG_frame * COG_to_pin_rel;
    ChVector<> rad_dir = (next_pin_pos - rolling_elem_center).GetNormalized();
    // pin is not exactly a known height above the surface,
    //  project outrwards from center of rolling element instead.
    pos_on_seg = rolling_elem_center + rad_dir * clearance;
    dist_to_end =
        Vdot(end_curve_seg - pos_on_seg,
             tan_dir_end_curved_seg);  // if there is a curved section w/ non-zero length, need at least 1 shoe.

    // radius of the next pin position, relative to rolling element center
    double len_r2 = std::sqrt(pow(clearance + m_shoe_chain_Yoffset, 2) + pow(m_pin_dist / 2.0, 2));

    // stop when furthest pin location on last created shoe body passes the end point.
    // Note: some rolling elements with short curved segments may end up skipping this, depending
    //      on initial placement of first shoe.
    while (dist_to_end > 0) {
        // create a new body by copying the first, add to handle vector.
        // Copy doesn't set the collision shape.
        // Don't reset visualization assets, copied in ChPhysicsItem::Copy()
        m_shoes.push_back(ChSharedPtr<ChBody>(new ChBody));
        m_numShoes++;
        m_shoes.back()->Copy(m_shoes.front().get_ptr());
        // create the collision geometry for the new shoe
        (m_start_loc_bar.z < 0) ? AddCollisionGeometry(LEFTSIDE, m_mu, 0.9*m_mu) : AddCollisionGeometry(RIGHTSIDE, m_mu, 0.9*m_mu);

        // even shoes can be a different texture
        (m_numShoes % 2 == 0) ? AddVisualization(m_numShoes - 1, true, "spheretexture.png") : AddVisualization();

        std::stringstream shoe_s;
        shoe_s << "shoe" << m_numShoes;
        m_shoes.back()->SetNameString(shoe_s.str());

        // pin between shoes position is know, orient the same as the previous shoe.
        pin_frame = ChFrame<>(COG_frame * COG_to_pin_rel, COG_frame.GetRot());
        ChVector<> r1 = pin_frame.GetPos() - rolling_elem_center;  // center to first pin

        double len_r1 = r1.Length();
        // COG orientation will need to be modified, according to clearance of the rolling element
        double phi = std::acos(Vdot(-r1 / len_r1, pin_frame.GetRot().GetXaxis()));
        double h1 = std::sqrt(pow(m_pin_dist, 2) - pow((len_r1 * len_r1 - len_r2 * len_r2) / (2.0 * len_r1), 2));
        double theta = std::asin(h1 / m_pin_dist);
        double psi = phi - theta;
        // rotate the pin frame
        ChQuaternion<> rot_frame = Q_from_AngAxis(-psi, pin_frame.GetRot().GetZaxis());
        pin_frame.SetRot(rot_frame * pin_frame.GetRot());

        if (0) {
            GetLog() << " x, y-axis, pre-xform: " << pin_frame.GetRot().GetXaxis() << "\n"
                     << pin_frame.GetRot().GetYaxis() << "\n";
            ChFrame<> pin_frameCHECK = rot_frame * pin_frame;
            GetLog() << " x, y-axis, POST xform: " << pin_frameCHECK.GetRot().GetXaxis() << "\n"
                     << pin_frameCHECK.GetRot().GetYaxis() << "\n";
        }

        // COG rotation is the same as the rotated pin
        COG_frame.SetRot(pin_frame.GetRot());

        // Find COG pos w/ the newly oriented pin,
        COG_frame.SetPos(pin_frame * COG_to_pin_rel);
        //  set the body info, add to system
        m_shoes.back()->SetPos(COG_frame.GetPos());
        m_shoes.back()->SetRot(COG_frame.GetRot());
        chassis->GetSystem()->AddBody(m_shoes.back());

        // create and init. the pin between the last shoe and this one, add to system.
        m_pins.push_back(ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute));
        std::stringstream pin_s;
        pin_s << "pin " << m_pins.size();
        m_pins.back()->SetNameString(pin_s.str());
        // pin frame picked up from COG_frame, where z-axis should be in the lateral direction.
        m_pins.back()->Initialize(m_shoes.back(), m_shoes.end()[-2],
                                  ChCoordsys<>(pin_frame.GetPos(), pin_frame.GetRot()));
        chassis->GetSystem()->AddLink(m_pins.back());

        // figure out how far pin 2 is from the end of the circular segment.
        next_pin_pos = COG_frame * COG_to_pin_rel;
        // project the pin towards the center of the rolling element.
        ChVector<> rad_dir = (next_pin_pos - rolling_elem_center).GetNormalized();
        // pin is not exactly a known height above the surface,
        //  project outrwards from center of rolling element instead.
        pos_on_seg = rolling_elem_center + rad_dir * clearance;
        dist_to_end = Vdot(end_curve_seg - pos_on_seg, tan_dir_end_curved_seg);
    }

    // assume we aren't aligned after creating shoes around the curved segment
    m_aligned_with_seg = false;
}

/// close the trackChain loop by connecting the end of the chain to the start.
/// Absolute coordinates.
void TrackChain::CreateShoes_closeChain(ChSharedPtr<ChBody> chassis,
                                        const ChFrame<>& chassis_REF,
                                        const ChVector<>& start_seg,
                                        const ChVector<>& end_seg) {
    // coming off the end of a curved segment, then following a straight line until near the end.
    // Once forward-most pin location crosses that of the first shoe, stop, and modify the first and last bodies.
    // Then, create the final pin joint.

    // get coordinate directions of the envelope surface
    // lateral in terms of the vehicle chassis
    ChVector<> lateral_dir = (m_shoes.front()->GetRot()).GetZaxis();
    ChVector<> tan_dir = (end_seg - start_seg).GetNormalized();
    ChVector<> norm_dir = Vcross(lateral_dir, tan_dir).GetNormalized();  // normal to the envelope surface
    ChMatrix33<> rot_seg_A;                                              // orientation of line segment
    rot_seg_A.Set_A_axis(tan_dir, norm_dir, lateral_dir);

    ChVector<> pos_on_seg;  // current position on the boundary, e.g. the line segment.
    ChVector<> shoe_pos;    // current shoe position, abs coords.

    // normal height above envelope surface of last place shoe
    double h = Vdot(norm_dir, m_shoes.back()->GetPos() - start_seg);
    // point projected on the surface of the envelope
    pos_on_seg = m_shoes.back()->GetPos() - norm_dir * h;

    // First, create the shoes along the linear segment. Connect by pins via revolute constraints.
    double dist_to_end = Vdot(end_seg - pos_on_seg, tan_dir);

    ChFrame<> COG_frame;
    ChFrame<> pin_frame;  // between this shoe and the last one
    // pin is just forward of the COG, and vice-versa
    ChVector<> COG_to_pin_rel(m_pin_dist / 2.0, 0, 0);

    // keep going until last created shoe COG pos_on_seg passes the end point.
    // Want to overshoot upon reaching curved segment, to not interpenetrate surface boundary.
    while (dist_to_end > 0) {
        // create a new body by copying the first, add to handle vector.
        // Copy doesn't set the collision shape.
        // Don't reset visualization assets, copied in ChPhysicsItem::Copy()
        m_shoes.push_back(ChSharedPtr<ChBody>(new ChBody));
        m_numShoes++;
        m_shoes.back()->Copy(m_shoes.front().get_ptr());
        // create the collision geometry for the new shoe
        (m_start_loc_bar.z < 0) ? AddCollisionGeometry(LEFTSIDE, m_mu, 0.9*m_mu) : AddCollisionGeometry(RIGHTSIDE, m_mu, 0.9*m_mu);

        // even shoes can be a different texture
        (m_numShoes % 2 == 0) ? AddVisualization(m_numShoes - 1, true, "spheretexture.png") : AddVisualization();

        std::stringstream shoe_s;
        shoe_s << "shoe" << m_numShoes;
        m_shoes.back()->SetNameString(shoe_s.str());

        // Find where the pin to the previous shoe should be positioned.
        // From there, calculate what the body pos/rot should be.
        ChVector<> pin_pos = (m_shoes.end()[-2])->GetFrame_COG_to_abs() * COG_to_pin_rel;
        pin_frame = ChFrame<>(pin_pos, (m_shoes.end()[-2])->GetRot());

        // Creating shoes along the line segment, one of two situations possible:
        // 1) shoe is exactly on the envelope boundary, and exactly parallel (e.g., first segment this is guaranteed).
        // 2) shoe is off the boundary, and will have a rot slightly different than previous shoe to get it closer,
        //      e.g., after a straight segment follows a curved segment.

        // At first,COG frame assume this shoe has same orientation as the previous.
        COG_frame = ChFrame<>((m_shoes.end()[-2])->GetFrame_COG_to_abs());
        // set the body pos. from the pin_pos w/ the previous shoe;
        COG_frame.SetPos(pin_frame * COG_to_pin_rel);

        // hold on to the global direction vector between the original 2 pin locations.
        ChVector<> pin_dir_original = (COG_frame.GetPos() - pin_frame.GetPos()).GetNormalized();

        // Assumption 1) is true if:
        //  A) We set the last shoe pin in such a way so this shoe can be exactly aligned with the line segment, or
        //  B) the last shoe on the linear segment was already aligned
        if (m_aligned_with_seg) {
            // previous shoe has been positioned so pin location is exactly
            //  m_shoe_chain_Yoffset above the envelope surface (norm. dir).
            // Use the rotation frame for the line segment for the shoe to keep the x-axis parallel to segment.
            COG_frame.SetRot(rot_seg_A);
            // if the previous shoe x-axis is not parallel to segment, using COG_frame
            // rot from that shoe will be incorrect. Instead, use the updated pin rot.
            pin_frame.SetRot(rot_seg_A);
            // now the position can be found relative to the newly oriented pin frame.
            // doesn't change if last shoe has the same orientation.
            COG_frame.SetPos(pin_frame * COG_to_pin_rel);
        } else {
            // verify that this configuration:
            //  1) does not cross the line segment boundary, and
            //  2) stays as clsoe as possible to the line segment (e.g., by rotating the body slightly at the pin_pos)
            //  3) does not rotate the body too aggressively, by setting the following limit:
            double lim_rot_angle = CH_C_PI_4;  // don't rotate the shoe more than 45 deg. towards the line segment

            // Consider the next pin location, relative to the current pin
            ChVector<> nextPin_pos_rel(m_pin_dist, 0, 0);
            ChVector<> nextPin_pos_abs = pin_frame * nextPin_pos_rel;

            // distance the nextpin is above the line segment surface
            double nextPin_clearance = Vdot(norm_dir, nextPin_pos_abs - start_seg);

            // distance the pin position is above the line segment surface
            double pin_clearance = Vdot(norm_dir, pin_frame.GetPos() - start_seg);

            // going to need the next pin to be ::m_shoe_chain_Yoffset above the line segment.
            // normal distance between pins relative to the line segment.
            double pin_len_norm_seg = pin_clearance - m_shoe_chain_Yoffset;

            double psi = 0;  // find rotation angle of shoe relative to the last one

            /*
            if( pin_len_norm_seg > m_pin_dist * std::sin(lim_rot_angle) )
            {
              // don't rotate too aggressively
              psi = lim_rot_angle;
            }
            else
            {
            */
            // distance of pins, tangent to the line segment.
            double len_on_seg = std::sqrt(m_pin_dist * m_pin_dist - pow(pin_len_norm_seg, 2));
            // know the next pin location
            nextPin_pos_abs = pin_frame.GetPos() - norm_dir * pin_len_norm_seg + tan_dir * len_on_seg;
            // direction vector between pins 1-2, normalized.
            ChVector<> pin_dir_modified = (nextPin_pos_abs - pin_frame.GetPos()).GetNormalized();

            // find rotation angles based on cos(psi) = pin_dir_original dot pin_dir_modified
            psi = std::acos(Vdot(pin_dir_original, pin_dir_modified));
            ChVector<> mcross = pin_dir_modified % pin_dir_original;
            if ((pin_dir_modified % pin_dir_original).z * lateral_dir.z > 0)
                psi = -psi;

            // }

            // rotate the pin frame about z-axis of the pin, at the pin
            ChQuaternion<> rot_frame = Q_from_AngAxis(psi, pin_frame.GetRot().GetZaxis());

            pin_frame.SetRot(rot_frame * pin_frame.GetRot());

            // can find the shoe COG pos/rot now, from pin orientation, and COG offset from pin pos
            COG_frame.SetRot(pin_frame.GetRot());
            COG_frame.SetPos(pin_frame * COG_to_pin_rel);

            // let's check to see if the criteria for setting m_aligned_with_seg = true.
            // Note: the function also sets the bool
            bool check_alignment = check_shoe_aligned(COG_frame * COG_to_pin_rel, start_seg, end_seg, norm_dir);
            if (0)
                GetLog() << " aligned ?? shoe # : " << m_numShoes << " ?? " << check_alignment << "\n";
        }

        // COG_frame is set correctly, add the body to the system
        m_shoes.back()->SetPos(COG_frame.GetPos());
        m_shoes.back()->SetRot(COG_frame.GetRot());
        chassis->GetSystem()->AddBody(m_shoes.back());

        // create and init. the pin between the last shoe and this one, add to system.
        m_pins.push_back(ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute));
        std::stringstream pin_s;
        pin_s << "pin" << m_pins.size();
        m_pins.back()->SetNameString(pin_s.str());
        // pin frame picked up from COG_frame, where z-axis should be in the lateral direction.
        m_pins.back()->Initialize(m_shoes.back(), m_shoes.end()[-2],
                                  ChCoordsys<>(pin_frame.GetPos(), pin_frame.GetRot()));
        chassis->GetSystem()->AddLink(m_pins.back());

        // See if the end of the linear segment has been reached.
        ChVector<> next_pin_loc = COG_frame * COG_to_pin_rel;
        // project this point down to the surface of the line segment.
        double proj_dist = Vdot(norm_dir, next_pin_loc - start_seg);
        ChVector<> pos_on_seg = next_pin_loc - norm_dir * proj_dist;

        // stop when the next pin pos on seg. is within half a shoe width of the beginning pos on seg.
        dist_to_end = Vdot(end_seg - pos_on_seg, tan_dir) - m_pin_dist / 2.0;
    }

    // If assumptions were followed throughout, first shoe is somewhere near the middle of the gear and idler.
    // The first and last shoes ought to be aligned at first, e.g. same Rot().
    // If the stars align, the pin locations will line up
    ChVector<> last_pin_pos = m_shoes.back()->GetFrame_COG_to_abs() * -COG_to_pin_rel;
    ChVector<> first_pin_pos = m_shoes.front()->GetFrame_COG_to_abs() * COG_to_pin_rel;

    // no way does this occur accidentally, but check anyway
    if ((first_pin_pos - last_pin_pos).LengthInf() < 1e-6) {
        GetLog() << "You are super lucky ! \n\n";

    } else {
        // pin 1 - pin last
        ChVector<> r_first_last = last_pin_pos - first_pin_pos;
        // assuming both shoes are aligned (same Rot), rotation is symmetric and angle is:
        double rotAng = std::acos(r_first_last.Length() / (2.0 * m_pin_dist));

        // rotate the first body about it's 2nd pin by -rotAng
        // get the the first pin frame from the first shoe Frame
        COG_frame = ChFrame<>(m_shoes.front()->GetFrame_COG_to_abs() * COG_to_pin_rel, m_shoes.front()->GetRot());
        // rotate about that pin
        ChQuaternion<> rot_q = Q_from_AngAxis(-rotAng, lateral_dir);
        COG_frame.SetRot(rot_q * COG_frame.GetRot());
        // now, reposition the COG backwards from the first pin
        COG_frame.SetPos(COG_frame * -COG_to_pin_rel);
        // know the COG frame for the first shoe, set the body info
        m_shoes.front()->SetPos(COG_frame.GetPos());
        m_shoes.front()->SetRot(COG_frame.GetRot());

        // rotate the last body about the last pin
        // get the pin location by stepping back from the shoe COG frame
        COG_frame = ChFrame<>(m_shoes.back()->GetFrame_COG_to_abs() * -COG_to_pin_rel, m_shoes.back()->GetRot());
        // rotate about that pin
        rot_q = Q_from_AngAxis(rotAng, lateral_dir);
        // reposition COG forward from this pin
        COG_frame.SetRot(rot_q * COG_frame.GetRot());

        // reposition COG of last body forward from the last pin
        COG_frame.SetPos(COG_frame * COG_to_pin_rel);
        // set the pos, rot of the last shoe body
        m_shoes.back()->SetPos(COG_frame.GetPos());
        m_shoes.back()->SetRot(COG_frame.GetRot());
    }

    // re-positioned the first and last shoe to share final pin.
    ChVector<> pin_pos_abs = m_shoes.back()->GetFrame_COG_to_abs() * COG_to_pin_rel;
    if (0) {
        ChVector<> pin_pos_abs_check = m_shoes.front()->GetFrame_COG_to_abs() * -COG_to_pin_rel;
        GetLog() << " final pin pos., Inf norm(rear - front) = " << (pin_pos_abs - pin_pos_abs_check).LengthInf()
                 << "\n";
    }
    // create and init. the pin between the last shoe and first, add to system.
    m_pins.push_back(ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute));
    std::stringstream pin_s;
    pin_s << "pin" << m_pins.size();
    m_pins.back()->SetNameString(pin_s.str());
    // use the orientation of the last shoe added
    m_pins.back()->Initialize(m_shoes.front(), m_shoes.back(), ChCoordsys<>(pin_pos_abs, m_shoes.back()->GetRot()));
    chassis->GetSystem()->AddLink(m_pins.back());
}

// all inputs in absolute coords.
// Sets m_aligned_with_seg, and returns it also
bool TrackChain::check_shoe_aligned(const ChVector<>& pin2_pos_abs,
                                    const ChVector<>& start_seg_pos,
                                    const ChVector<>& end_seg_pos,
                                    const ChVector<>& seg_norm_axis) {
    bool out = false;  // guilty, until proven innocent.
    ChVector<> tan_hat = (end_seg_pos - start_seg_pos).GetNormalized();
    // criteria 1) pin2 should be less than m_shoe_chain_Yoffset from the line segment.
    //  If it's too much less, the shoe might penetrate the envelope, so cap it in either direction.
    double dist = Vdot(seg_norm_axis, pin2_pos_abs - start_seg_pos);
    if (abs(dist - m_shoe_chain_Yoffset) < 1e-6)
        out = true;

    // set the outcome, and return it
    m_aligned_with_seg = out;
    return out;
}

ChSharedPtr<ChBody> TrackChain::GetShoeBody(size_t track_idx) const
// ChSharedPtr<ChBodyAuxRef> TrackChain::GetShoeBody(size_t track_idx)
{
    assert(track_idx < m_numShoes);
    return (track_idx > m_numShoes - 1) ? m_shoes[track_idx] : m_shoes[0];
}

const std::vector<ChSharedPtr<ChBody> >& TrackChain::GetShoeBody() const {
    assert(m_shoes.size() > 0);
    return m_shoes;
}

const ChVector<> TrackChain::GetPinReactForce(size_t pin_idx) {
    assert(pin_idx < m_pins.size());
    return m_pins[pin_idx]->Get_react_force();
}

const ChVector<> TrackChain::GetPinReactTorque(size_t pin_idx) {
    assert(pin_idx < m_pins.size());
    return m_pins[pin_idx]->Get_react_torque();
}

void TrackChain::Set_pin_friction(double damping_C, bool use_custom_damper, double damping_C_nonlin) {
    // make sure track shoes and connecting pins have already been created
    assert(m_pins.size() > 0);

    // if we already called this function, there will be existing ChLinkForce in the m_pin_friction array,
    if (!m_pin_friction.empty()) {
        // objects are deleted, clear the pointer array
        m_pin_friction.clear();
        // shared ptrs, can just clear the array and memory takes care of itself.
        m_custom_dampers.clear();
    }

    // iterate thru the pin revolute joints, adding a friction force on the DOF for each
    for (std::vector<ChSharedPtr<ChLinkLockRevolute> >::iterator itr = m_pins.begin(); itr != m_pins.end(); itr++) {
        // just use a shared ptr to the force, keep it in an array
        ChLinkForce* pin_friction = new ChLinkForce;
        pin_friction->Set_active(1);
        pin_friction->Set_K(0);
        pin_friction->Set_R(damping_C);

        // if we're using a custom spring...
        if (m_use_custom_damper) {
            ChSharedPtr<ChFunction_CustomDamper> custom_damper(
                new ChFunction_CustomDamper(damping_C, damping_C_nonlin));
            pin_friction->Set_R(1);
            // add the custom damper modulus function to the pin friction damping val
            pin_friction->Set_modul_R(custom_damper.get_ptr());

            // add handle to shared ptr to an array
            m_custom_dampers.push_back(custom_damper);
        }

        // link force should be completely set at this point
        // use the pointer to set the spring function in the pin revolute constraint
        (*itr)->SetForce_Rz(pin_friction);

        // add the pointer to the array
        m_pin_friction.push_back(pin_friction);
    }

    // now that custom dampers have been created, we can use them.
    m_use_custom_damper = use_custom_damper;
}

// Get some data about the gear and its normal and friction contact forces with the specified shoe each timestep.
// Returns: total number of contacts between the specified shoe body and gear body
// Sets non-const inputs with contact persistence info, # of contacts, Fn, Ft statistics.
// SG_info = (Num_contacts, t_persist, t_persist_max)
// Force_mag_info = (Fn, Ft, 0)
// PosRel, VRel = relative pos, vel of contact point, relative to gear c-sys
// NormDirRel = tracked contact normal dir., w.r.t. gear c-sys
int TrackChain::reportShoeGearContact(ChSystem* system,
                                      const std::string& shoe_name,
                                      std::vector<ChVector<> >& SG_info,
                                      std::vector<ChVector<> >& Force_mag_info,
                                      std::vector<ChVector<> >& PosRel_contact,
                                      std::vector<ChVector<> >& VRel_contact,
                                      std::vector<ChVector<> >& NormDirRel_contact) {
    // setup the reporter, init any variables that are not history dependent
    _shoeGear_report_contact reporter(shoe_name);

    // ----------------------------------
    // history dependent variables.
    // SG_info = (Num_contacts, t_persist, t_persist_max), carry over timers.
    reporter.m_t_persist.resize(2, 0);
    reporter.m_t_persist_max.resize(2, 0);
    for (int i = 0; i < 2; i++) {
        reporter.m_t_persist[i] = m_SG_info[i].y;
        reporter.m_t_persist_max[i] = m_SG_info[i].z;
    }
    // track a single shoe pin/gear contact point relative pos,vel., if it has been set, for each side of the gear.
    reporter.m_is_persistentContact_set.resize(2, 0);
    for (int pc = 0; pc < 2; pc++) {
        reporter.m_is_persistentContact_set[pc] = m_SG_is_persistentContact_set[pc];
        if (reporter.m_is_persistentContact_set[pc])
            reporter.m_PosRel[pc] = m_SG_PosRel[pc];
        else {
            // when not set (between contact events), keep the z-coord (so the contact doesn't swap to the other side)
            reporter.m_PosRel[pc] = ChVector<>(0, 0, m_SG_PosRel[pc].z);
            // reporter.m_PosRel = ChVector<>();
        }
    }
    // to be complete
    reporter.m_ContactPos_all.clear();
    reporter.m_ContactFn_all.clear();

    // pass the reporter callback to the system
    system->GetContactContainer()->ReportAllContacts(&reporter);

    // fill the output data vectors. MaKe sure they are empty first
    SG_info.clear();
    Force_mag_info.clear();
    PosRel_contact.clear();
    VRel_contact.clear();
    NormDirRel_contact.clear();
    // fill both persistent contacts
    for (int rc = 0; rc < 2; rc++) {
        // process any history dependent values after reporter is called
        // reset persistent time counter when no contacts
        if (reporter.m_Fn[rc] == 0) {
            reporter.m_t_persist[rc] = 0;
            // allow the relative position of the shoe-gear contact to track to reset
            // reporter.m_PosRel = ChVector<>();
            reporter.m_is_persistentContact_set[rc] = false;
        }

        // set any data here from the reporter,
        SG_info.push_back(
            ChVector<>(reporter.m_num_contacts_side[rc], reporter.m_t_persist[rc], reporter.m_t_persist_max[rc]));

        Force_mag_info.push_back(ChVector<>(reporter.m_Fn[rc], reporter.m_Ft[rc], 0));

        // set loc, vel, norm. force dir. of contact point, relative to gear csys
        PosRel_contact.push_back(reporter.m_PosRel[rc]);
        VRel_contact.push_back(reporter.m_VelRel[rc]);
        NormDirRel_contact.push_back(reporter.m_NormDirRel[rc]);

        // finally, set any data that should persist to the next time step to the ChainSystem.
        m_SG_Fn.push_back(reporter.m_NormDirAbs[rc] * reporter.m_Fn[rc]);
    }

    // finally, set any data that should persist to the next time step to the ChainSystem.
    m_SG_numContacts = reporter.m_num_contacts;
    m_SG_info = SG_info;
    m_SG_is_persistentContact_set = reporter.m_is_persistentContact_set;
    m_SG_PosRel = reporter.m_PosRel;

    // set any aother desired info, e.g.  for plotting the contact point and normal dir.
    m_SG_PosAbs = reporter.m_PosAbs;

    m_SG_ContactPos_all = reporter.m_ContactPos_all;
    m_SG_ContactFn_all = reporter.m_ContactFn_all;

    //  # of contacts between specified shoe and gear body
    return reporter.m_num_contacts;
}

/// write the header for the chain system
void TrackChain::Write_header(const std::string& filename, DebugType type) {
    if (type & DBG_BODY) {
        m_filename_DBG_BODY = filename;
        ChStreamOutAsciiFile ofileDBG_FIRSTSHOE(m_filename_DBG_BODY.c_str());
        std::stringstream ss_fs_head;
        ss_fs_head << "time,x,y,z,xRel,yRel,zRel,Vx,Vy,Vz,Ax,Ay,Az,Wx,Wy,Wz,Fx,Fy,Fz,Tx,Ty,Tz\n";
        ofileDBG_FIRSTSHOE << ss_fs_head.str().c_str();
    }
    if (type & DBG_CONSTRAINTS) {
        // to-do, constraint violation of two revolute joints of adjacent shoes
    }
    if (type & DBG_CONTACTS) {
        m_filename_DBG_CONTACTS = filename;
        ChStreamOutAsciiFile ofileDBG_shoeGear(m_filename_DBG_CONTACTS.c_str());
        std::stringstream ss_sg_head;
        // suffix "P" is the z-positive side of the gear, "N" is the z-negative side
        ss_sg_head
            << "time,Ncontacts,NcontactsP,t_persistP,t_persist_maxP,FnMagP,FtMagP,xRelP,yRelP,zRelP,VxRelP,VyRelP,"
               "VzRelP,normDirRelxP,normDirRelyP,normDirRelzP"
            << ",NcontactsN,t_persistN,t_persist_maxN,FnMagN,FtMagN,xRelN,yRelN,zRelN,VxRelN,VyRelN,VzRelN,"
               "normDirRelxN,normDirRelyN,normDirRelzN\n";
        ofileDBG_shoeGear << ss_sg_head.str().c_str();
    }
}

/// write the data at time t
void TrackChain::Write_data(const double t, const ChSharedPtr<ChBody> chassis, DebugType type) {
    if (type & DBG_BODY) {
        std::stringstream ss;
        // time,x,y,z,xBar,yBar,zBar,vx,vy,vz,ax,ay,az,wx,wy,wz,fx,fy,fz
        ChSharedPtr<ChBody> shoe = GetShoeBody(0);
        ChFrame<> parent_rot(ChVector<>(), chassis->GetA());
        ss << t << "," << shoe->GetPos() << ","
           << parent_rot.TransformParentToLocal(shoe->GetPos() - chassis->GetPos()) << ","
           << shoe->GetPos_dt() << "," << shoe->GetPos_dtdt() << "," << shoe->GetWvel_loc() << ","
           << GetPinReactForce(0) << "," << GetPinReactTorque(0) << "\n";
        // open the file for appending, write the data.
        ChStreamOutAsciiFile ofile(m_filename_DBG_BODY.c_str(), std::ios::app);
        ofile << ss.str().c_str();
    }
    if (type & DBG_CONSTRAINTS) {
        // todo
    }
    if (type & DBG_CONTACTS) {
        // second file, to specify some collision info with the gear
        double num_contacts = 0;
        std::vector<ChVector<> > sg_info;             // output data set
        std::vector<ChVector<> > Force_mag_info;      // per step contact force magnitude, (Fn, Ft, 0)
        std::vector<ChVector<> > PosRel_contact;      // location of a contact point relative to the gear c-sys
        std::vector<ChVector<> > VRel_contact;        // follow the vel. of a contact point relative to the gear c-sys
        std::vector<ChVector<> > NormDirRel_contact;  // tracked contact normal dir., w.r.t. gear c-sys
        // sg_info = (Num_contacts, t_persist, t_persist_max)
        num_contacts = reportShoeGearContact(chassis->GetSystem(), GetShoeBody(0)->GetNameString(), sg_info,
                                             Force_mag_info, PosRel_contact, VRel_contact, NormDirRel_contact);

        // suffix "P" is the z-positive side of the gear, "N" is the z-negative side
        // "time,Ncontacts,t_persistP,t_persist_maxP,FnMagP,FtMagP,xRelP,yRelP,zRelP,VxRelP,VyRelP,VzRelP,normDirRelxP,normDirRelyP,normDirRelzP,t_persistN,t_persist_maxN,FnMagN,FtMagN,xRelN,yRelN,zRelN,VxRelN,VyRelN,VzRelN,normDirRelxN,normDirRelyN,normDirRelzN";
        std::stringstream ss_sg;
        ss_sg << t << "," << num_contacts << "," << sg_info[0] << "," << Force_mag_info[0].x << ","
              << Force_mag_info[0].y << "," << PosRel_contact[0] << "," << VRel_contact[0] << ","
              << NormDirRel_contact[0] << "," << sg_info[1] << "," << Force_mag_info[1].x << "," << Force_mag_info[1].y
              << "," << PosRel_contact[1] << "," << VRel_contact[1] << "," << NormDirRel_contact[1] << "\n";
        ChStreamOutAsciiFile ofile_shoeGear(m_filename_DBG_CONTACTS.c_str(), std::ios::app);
        ofile_shoeGear << ss_sg.str().c_str();
    }
}

}  // end namespace chrono
