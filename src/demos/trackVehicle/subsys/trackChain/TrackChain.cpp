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
// collision mesh
#include "geometry/ChCTriangleMeshSoup.h"

#include "utils/ChUtilsData.h"
#include "utils/ChUtilsInputOutput.h"


namespace chrono {


// static variables
const std::string TrackChain::m_collisionFile = utils::GetModelDataFile("track_data/M113/shoe_collision.obj");
const std::string TrackChain::m_meshName = "M113 shoe"; 
const std::string TrackChain::m_meshFile = utils::GetModelDataFile("track_data/M113/shoe_view.obj");

const double TrackChain::m_mass = 18.02;
const ChVector<> TrackChain::m_inertia(0.22, 0.25,  0.04); // TODO: what is this w/ new single pin configuration???
const ChVector<> TrackChain::m_COM = ChVector<>(0., 0., 0.);  // location of COM, relative to REF (e.g, geomtric center)
const ChVector<> TrackChain::m_shoe_box(0.205 *0.5, 0.0663 *0.5, 0.38 *0.5); // length, height, width HALF DIMS!
const double TrackChain::m_pin_width = 0.531; // total width of cylinder pins
const double TrackChain::m_pin_dist = 0.15162;		// linear distance between a shoe chain spacing. exact = 0.205
const double TrackChain::m_pin_radius = 0.02317;
const ChVector<> TrackChain::m_tooth_box(0.08 *0.5, 0.075 *0.5, 0.08 *0.5);  // length, height, width HALF DIMS
 // distance between body center and the vertical offset to the inner-surface of the collision geometry
//  used for initializing shoes as a chain
const double TrackChain::m_shoe_chain_Yoffset = 0.035; // .03315 exact

TrackChain::TrackChain(const std::string& name, 
                       VisualizationType vis, 
                       CollisionType collide)
                       : m_vis(vis), m_collide(collide), m_numShoes(0)
{
  // clear vector holding list of body handles
  m_shoes.clear();
  // add first track shoe body
  m_shoes.push_back(ChSharedPtr<ChBody>(new ChBody));
  // m_shoes.push_back(ChSharedPtr<ChBodyAuxRef>(new ChBodyAuxRef));
  // m_shoes[0]->SetFrame_COG_to_REF(ChFrame<>(m_COM,QUNIT));
  m_numShoes++;

  m_shoes[0]->SetNameString("shoe 1, "+name);
  m_shoes[0]->SetMass(m_mass);
  m_shoes[0]->SetInertiaXX(m_inertia);


  // Attach visualization to the base track shoe
  AddVisualization();
}

// figure out the control points, 2 per rolling element to wrap the chain around.
void TrackChain::Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
                            const std::vector<ChVector<>>& rolling_element_loc,
                            const std::vector<double>& clearance,
                            const ChVector<>& start_loc)
{
  assert(rolling_element_loc.size() == clearance.size() );
  // get the following in abs coords: 1) start_loc, 2) rolling_elem, 3) control_points
  ChFrame<> start_to_abs(start_loc);
  start_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

  // find control points, which lie on the envelope of each rolling element, in abs coords
  std::vector<ChFrame<>> control_to_abs;
  std::vector<ChFrame<>> rolling_to_abs;

  size_t num_elem = rolling_element_loc.size();  // number of control points

  // define the envelope to wrap the chain around using 1) body locs and clearances, 2) control_points.
  // each rolling element has 2 control points, a start and end point for a given envenlope line segment.
  ChVector<> start_point;  // current start_loc for this segment
  ChVector<> end_point; // end point of the current segment
  ChVector<> rad_dir;   // center to segment start/end point on rolling elments
  ChVector<> r_21;      // vector between two pulley centerpoints
  ChVector<> norm_dir;  // norm = r_12 cross r_32
  // iterate over the line segments, first segment is between start_loc and rolling_elem 0
  // last segment  is between rolling_elem[last] and rolling_elem[last-1]
  for(size_t i = 0; i < num_elem; i++)
  { 
    // convert the center of the rolling body to abs coords
    rolling_to_abs.push_back(ChFrame<>(rolling_element_loc[i]));
    rolling_to_abs[i].ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

    // start and end points of line segment are found (in chassis ref system)
    if(i == 0)
    { 
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
      if( abs( Vdot( tan_dir, rad_dir) ) > 1e-3 )
      {
        // move top_center so r_21 is orthogonal to rad_dir
        // i.e., rad_dir dot r_21 = 0
        // easier to figure out the relative angle rad_dir needs to be rotated about the norm axis.
        ChVector<> start_cen = start_loc - rolling_element_loc[i];
        double theta = std::acos( Vdot(start_cen.GetNormalized(), rad_dir));  // angle between direction vectors
        double seglen = std::sqrt(start_cen.Length2() - pow(clearance[i],2));
        double h = start_cen.Length();
        double phi = std::asin( seglen / h ); // what theta should be
        double rot_ang = theta - phi;
        // find the radial direction from the center of adjacent rolling elements
        // for seg i, norm = r12 x r32
        norm_dir = Vcross(rolling_element_loc.back() - rolling_element_loc[0],
          rolling_element_loc[1] - rolling_element_loc[0]);
        norm_dir.Normalize();
        // rotate rad_dir about norm axis
        ChQuaternion<> rot_q = Q_from_AngAxis(rot_ang, norm_dir);
        ChFrame<> rad_dir_frame(rad_dir, QUNIT);
        rad_dir =  (rot_q * rad_dir_frame).GetPos();
      }
      // rad_dir is now tangent to r_21, so define the new endpoint of this segment
      end_point = rolling_element_loc[i] + rad_dir * clearance[i];

    } else 
    {
      // intermediate points, find start and end from roller elem locations
      // first guess at start/end points: center distance vector
      r_21 = rolling_element_loc[i] - rolling_element_loc[i-1];

      // find the radial direction from the center of adjacent rolling elements
      // for seg i, norm = [r(i-2)-r(i-1)] x [r(i)-r(i-1)
      if(i == 1)
      {
        norm_dir = Vcross( rolling_element_loc.back() - rolling_element_loc[i-1],
          rolling_element_loc[i] - rolling_element_loc[i-1]);
      } else if( abs(clearance[i] - clearance[i-1]) < 1e-3 && abs(clearance[i-2] - clearance[i-1]) < 1e-3) 
      {
        // this works, as long as i, i-1 and i-2 don't have the same clearance (then it's just a line).
        norm_dir = Vcross( rolling_element_loc[0] - rolling_element_loc[i-1],
          rolling_element_loc[i] - rolling_element_loc[i-1]);
      } else {
        // this works, as long as i, i-1 and i-2 don't have the same clearance (then it's just a line).
        norm_dir = Vcross( rolling_element_loc[i-2] - rolling_element_loc[i-1],
          rolling_element_loc[i] - rolling_element_loc[i-1]);
      }
      norm_dir.Normalize();
      // if the two rolling elements have the same radius, no angle of wrap.
      rad_dir = Vcross(norm_dir, r_21.GetNormalized());
      rad_dir.Normalize();
      // when not the same size, find the angle of pulley wrap.
      // Probably overkill, but check that the radial vector is ortho. to r_21
      if(  abs(clearance[i] - clearance[i-1]) > 1.0e-3 || abs(Vdot(r_21.GetNormalized(), rad_dir)) > 1.0e-3)
      {
        // pulley wrap angle, a = (r2-r1)/center_len
        double alpha = asin( (clearance[i] - clearance[i-1]) / r_21.Length() );
        ChQuaternion<> alpha_rot = Q_from_AngAxis(alpha, norm_dir);
        ChFrame<> rot_frame(ChVector<>(),alpha_rot);
        // rotate rad_dir the angle of wrap about the normal axis
        rad_dir =  rad_dir >> rot_frame;
      }

      // with a radial direction vector, start/end points are easy
      start_point = rolling_element_loc[i-1] + rad_dir * clearance[i-1];
      end_point = rolling_element_loc[i] + rad_dir * clearance[i];
    }

    // with starting and end points in local frame, push to abs frame
    if(i == 0)
    {
      // first segment, only use the end_point
      control_to_abs.push_back(ChFrame<>(end_point));
      control_to_abs[0].ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs() );
    } else
    {
      // intermediate segments, use both start and end points
      control_to_abs.push_back(ChFrame<>(start_point));
      control_to_abs.back().ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs() );
      control_to_abs.push_back(ChFrame<>(end_point));
      control_to_abs.back().ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs() );
    }

  }

  // there is 1 more line segment between the last roller and the start_loc
  // know end_point off the bat
  end_point = start_point;

  // no second pulley to use to find r_21
  ChVector<> top_center = rolling_element_loc.back();
  top_center.y += clearance.back(); // top center of the last rolling element
  r_21 = start_loc - top_center;  // first guess at start and end points
  ChVector<> tan_dir = r_21.GetNormalized();

  // If start_loc is precisely located, this rad_dir will define the end point as it is now.
  rad_dir = top_center - rolling_element_loc.back();
  rad_dir.Normalize();
  // If start_loc isn't exactly placed, r_21 won't be tangent to the envelope at point top_center.
  // Enforce that the vector between the start and end points is tangent.
  if( abs(Vdot(tan_dir,rad_dir)) > 1e-3 )
  {
    // move top_center so r_21 is orthogonal to rad_dir
    // i.e., rad_dir dot r_21 = 0
    // easier to figure out the relative angle rad_dir needs to be rotated about the norm axis.
    ChVector<> start_cen = start_loc - rolling_element_loc.back();
    double theta = std::acos( Vdot(start_cen.GetNormalized(), rad_dir));  // angle between direction vectors
    double seglen = std::sqrt(start_cen.Length2() - pow(clearance.back(),2));
    double h = start_cen.Length();
    double phi = std::asin( seglen / h ); // what theta should be
    double rot_ang = phi - theta; // since we're looking at it backwards compared to the first rolling element
    // find the radial direction from the center of adjacent rolling elements
    // for seg i, norm = r12 x r32
    norm_dir = Vcross(rolling_element_loc.back() - rolling_element_loc[0],
      rolling_element_loc[num_elem-2] - rolling_element_loc.back() );
    norm_dir.Normalize();
    // rotate rad_dir about norm axis
    ChFrame<> rot_frame(ChVector<>(), Q_from_AngAxis(rot_ang, norm_dir));
    rad_dir =  rad_dir >> rot_frame;
  }
  // rad_dir is now tangent to r_21, so define the new endpoint of this segment
  start_point = rolling_element_loc.back() + rad_dir * clearance.back();

  // last segment, only use the start_point
  control_to_abs.push_back(ChFrame<>(start_point));
  (control_to_abs.back()).ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs() );

  // "wrap" the track chain around the trackSystem rolling elements, e.g., drive-gear,
  // idler, road-wheels. First and last shoes are allowed to be in any orientation,
  // as long as the final pin joint connects correctly.
  CreateChain(chassis, control_to_abs, rolling_to_abs, clearance, start_to_abs.GetPos() );
}

void TrackChain::AddVisualization()
{
  assert(m_numShoes > 0);
  AddVisualization(m_numShoes -1);
}

void TrackChain::AddVisualization(size_t track_idx)
{
  assert(track_idx < m_numShoes);
  // Attach visualization asset
  switch (m_vis) {
  case VisualizationType::PRIMITIVES:
  {
    // shoes will be added to the same collision family so self-collision can be toggled
    ChSharedPtr<ChBoxShape> box(new ChBoxShape);
    box->GetBoxGeometry().SetLengths(2.0 * m_shoe_box);  // use full distances w/ assets
    m_shoes[track_idx]->AddAsset(box);

    ChSharedPtr<ChTexture> tex(new ChTexture);
    tex->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
    m_shoes[track_idx]->AddAsset(tex);

    break;
  }
  case VisualizationType::MESH:
  {
    // mesh for visualization only.
    geometry::ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(getMeshFile(), false, false);

    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(getMeshName());
    m_shoes[track_idx]->AddAsset(trimesh_shape);

    ChSharedPtr<ChColorAsset> mcolor(new ChColorAsset(0.3f, 0.3f, 0.3f));
    m_shoes[track_idx]->AddAsset(mcolor);

    break;
  }
  }
}


void TrackChain::AddCollisionGeometry()
{
  assert(m_numShoes > 0);
  AddCollisionGeometry(m_numShoes-1);
}

void TrackChain::AddCollisionGeometry(size_t track_idx)
{
  assert(track_idx < m_numShoes);
   // add collision geometrey to the chassis, if enabled
  m_shoes[track_idx]->SetCollide(true);
  m_shoes[track_idx]->GetCollisionModel()->ClearModel();

  switch (m_collide) {
  case CollisionType::PRIMITIVES:
  {
    // use a simple box, single pin
    m_shoes[track_idx]->GetCollisionModel()->AddBox(m_shoe_box.x, m_shoe_box.y, m_shoe_box.z);
    // pin is a single cylinder
    double pin_offset = -0.07581;
    m_shoes[track_idx]->GetCollisionModel()->AddCylinder(m_pin_radius, m_pin_radius, m_pin_width/2.0, ChVector<>(pin_offset, 0, 0) );
    
    break;
  }
  case CollisionType::COMPOUNDPRIMITIVES:
  {
    // shoe geometry provided can be exactly represented by 6 smaller boxes, 2 cylinders.
    double subBox_width = 0.082;
    double stagger_offset = 0.03;
    // 5 smaller boxes make up the base of the shoe
    m_shoes[track_idx]->GetCollisionModel()->AddBox(m_shoe_box.x, m_shoe_box.y, subBox_width);
    m_shoes[track_idx]->GetCollisionModel()->AddBox(m_shoe_box.x, m_shoe_box.y, subBox_width, ChVector<>(-stagger_offset, 0, subBox_width));
    m_shoes[track_idx]->GetCollisionModel()->AddBox(m_shoe_box.x, m_shoe_box.y, subBox_width, ChVector<>(-stagger_offset, 0, -subBox_width));
    m_shoes[track_idx]->GetCollisionModel()->AddBox(m_shoe_box.x, m_shoe_box.y, subBox_width, ChVector<>(0, 0, 2.0*subBox_width));
    m_shoes[track_idx]->GetCollisionModel()->AddBox(m_shoe_box.x, m_shoe_box.y, subBox_width, ChVector<>(0, 0, 2.0*subBox_width));
    // add the tooth box
    double tooth_offset = -0.07315; // vertical offset
    m_shoes[track_idx]->GetCollisionModel()->AddBox(m_tooth_box.x, m_tooth_box.y, m_tooth_box.z, ChVector<>(0, tooth_offset, 0));
    // add the pin as a single cylinder
    double pin_offset = -0.07581;
    m_shoes[track_idx]->GetCollisionModel()->AddCylinder(m_pin_radius, m_pin_radius, m_pin_width, ChVector<>(pin_offset, 0, 0) );
    
    break;
  }
  case CollisionType::MESH:
  {
    // use a triangle mesh
   
		geometry::ChTriangleMeshSoup temp_trianglemesh; 
		
    // TODO: fill the triangleMesh here with some track shoe geometry

		m_shoes[track_idx]->GetCollisionModel()->SetSafeMargin(0.004);	// inward safe margin
		m_shoes[track_idx]->GetCollisionModel()->SetEnvelope(0.010);		// distance of the outward "collision envelope"
		m_shoes[track_idx]->GetCollisionModel()->ClearModel();

    // is there an offset??
    double shoelength = 0.2;
    ChVector<> mesh_displacement(shoelength*0.5,0,0);  // since mesh origin is not in body center of mass
    m_shoes[track_idx]->GetCollisionModel()->AddTriangleMesh(temp_trianglemesh, false, false, mesh_displacement);

    break;
  }
  case CollisionType::CONVEXHULL:
  {
    // use convex hulls, loaded from file
    ChStreamInAsciiFile chull_file(GetChronoDataFile("track_data/M113/shoe_collision.chulls").c_str());
    // transform the collision geometry as needed
    double mangle = 45.0; // guess
    ChQuaternion<>rot;
    rot.Q_from_AngAxis(mangle*(CH_C_PI/180.),VECT_X);
    ChMatrix33<> rot_offset(rot);
    ChVector<> disp_offset(0,0,0);  // no displacement offset
    m_shoes[track_idx]->GetCollisionModel()->AddConvexHullsFromFile(chull_file, disp_offset, rot_offset);
    break;
  }
  } // end switch

  // set collision family
  m_shoes[track_idx]->GetCollisionModel()->SetFamily( (int)CollisionFam::SHOES);
  // don't collide with other shoes, but with everything else
  m_shoes[track_idx]->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily( (int)CollisionFam::SHOES );

  m_shoes[track_idx]->GetCollisionModel()->BuildModel();

}

// two control points per rolling body.
// each two control points correspond to a single rolling element & clearance value, in the same order.
void TrackChain::CreateChain(ChSharedPtr<ChBodyAuxRef> chassis,
                             const std::vector<ChFrame<>>& control_points_abs,
                             const std::vector<ChFrame<>>& rolling_element_abs,
                             const std::vector<double>& clearance,
                             const ChVector<>& start_pos_abs)
{
  // Each segment has 2 parts, the initial linear portion, then wrapping around 
  //   the rolling element the start point of the next linear line segment.
  size_t num_elems = clearance.size();
  // Keep track of the current position along the line segment as new shoe bodies are created.
  // position along line defined by projecting the shoe body normal at the shoe COM down to the linear line segment,
  // or toward the center of the rolling element.
  ChVector<> curr_pos = start_pos_abs;  
  for(int idx = 0; idx < num_elems; idx++)
  {
    ChVector<> start_seg;  // start and end point of the line segment
    if(idx == 0)
      start_seg = start_pos_abs;
    else
      start_seg = control_points_abs[2*(idx-1)+1].GetPos();
    ChVector<> end_seg = control_points_abs[2*idx].GetPos();  // end of line seg.
    ChVector<> end_curve = control_points_abs[2*idx+1].GetPos();  // end of curved section
    // build the bodies for this line segment and rolling element curved section
    CreateShoes(chassis, start_seg, end_seg, end_curve, rolling_element_abs[idx].GetPos(), clearance[idx]);
  }
  // this will get us around the last rolling element curved segment. Add the final length of shoes
  CreateShoes_closeChain(chassis, control_points_abs.back().GetPos(), start_pos_abs);
}

// all locations are in the abs ref. frame.
// Some assumptions:
// Body orientation and axis of pin rotation always in the lateral_dir
void TrackChain::CreateShoes(ChSharedPtr<ChBodyAuxRef> chassis,
    const ChVector<>& start_seg,
    const ChVector<>& end_seg,
    const ChVector<>& end_curve_seg,
    const ChVector<>& rolling_elem_center,
    double clearance)
{
  // get coordinate directions of the envelope surface
  // lateral in terms of the vehicle chassis
  ChVector<> lateral_dir = Vcross(start_seg-end_seg, end_curve_seg-end_seg).GetNormalized();
  ChVector<> tan_dir = (end_seg - start_seg).GetNormalized();
  ChVector<> norm_dir = Vcross(lateral_dir, tan_dir).GetNormalized();   // normal to the envelope surface
  ChMatrix33<> rot_seg_A;  // orientation of line segment
  rot_seg_A.Set_A_axis(tan_dir, norm_dir, lateral_dir);

  ChVector<> pos_on_seg;  // current position on the boundary, e.g. the line segment.
  ChVector<> shoe_pos;    // current shoe position, abs coords.

  // special case: very first shoe
  // A single ChBody shoe is created upon construction, so it be can copied by the other shoes.
  // add 1) collision geometry, 2) set init. pos/rot, 3) add to system.
  if(m_numShoes == 1)
  {
    pos_on_seg = start_seg; // special case
    AddCollisionGeometry(); // add collision geometry
    shoe_pos = pos_on_seg + norm_dir * m_shoe_chain_Yoffset;
    m_shoes.front()->SetPos(shoe_pos);
    m_shoes.front()->SetRot(rot_seg_A); // can assume its the same as the line segment
    chassis->GetSystem()->Add(m_shoes.front());

    // First shoe on first line segment should always be aligned.
    m_aligned_with_seg = true; 
  }
  else
  {
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
  ChVector<> COG_to_pin_rel(m_pin_dist/2.0, 0, 0); 

  // keep going until last created shoe COG pos_on_seg passes the end point.
  // Want to overshoot upon reaching curved segment, to not interpenetrate surface boundary.
  while(dist_to_end > 0 )  
  {
    // create a new body by copying the first, add to handle vector.
    // Copy doesn't set the collision shape.
    // Don't reset visualization assets, copied in ChPhysicsItem::Copy()
    m_shoes.push_back(ChSharedPtr<ChBody>(new ChBody));
    m_numShoes++;
    m_shoes.back()->Copy( m_shoes.front().get_ptr() );
    AddCollisionGeometry();
    m_shoes.back()->SetNameString( "shoe " + std::to_string(m_numShoes) );

    // Find where the pin to the previous shoe should be positioned.
    // From there, calculate what the body pos/rot should be.
    // Rse the COG pos, rot of the previous shoe to get us there.
    ChVector<> pin_pos = (m_shoes.end()[-2])->GetFrame_COG_to_abs() * COG_to_pin_rel;
    pin_frame = ChFrame<>(pin_pos, (m_shoes.end()[-2])->GetRot() );

    // Creating shoes along the line segment, one of two situations possible:
    // 1) shoe is exactly on the envelope boundary, and exactly parallel (e.g., first segment this is guaranteed).
    // 2) shoe is off the boundary, and will have a rot slightly different than previous shoe to get it closer,
    //      e.g., after a straight segment follows a curved segment.

    // At first,COG frame assume this shoe has same orientation as the previous.
    COG_frame = ChFrame<>((m_shoes.end()[-2])->GetFrame_COG_to_abs() );
    // set the body pos. from the pin_pos w/ the previous shoe;
    COG_frame.SetPos(pin_frame * COG_to_pin_rel);

    // Assumption 1) is true if: 
    //  A) We set the last shoe pin in such a way so this shoe can be exactly aligned with the line segment, or
    //  B) the last shoe on the linear segment was already aligned
    if( m_aligned_with_seg )
    {
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
    }
    else 
    {
      // verify that this configuration:
      //  1) does not cross the line segment boundary, and
      //  2) stays as clsoe as possible to the line segment (e.g., by rotating the body slightly at the pin_pos)
      //  3) does not rotate the body too aggressively

      // For 1), consider a point at the very front edge of a bounding box, relative to the pin.
      ChVector<> corner_pos_rel( m_pin_dist + (m_shoe_box.x/2.0), m_shoe_chain_Yoffset, 0);
      double corner_pos_len = corner_pos_rel.Length();
      ChVector<> corner_pos_abs = pin_frame * corner_pos_rel;

      // distance the corner point is above the line segment surface
      double corner_clearance = Vdot(norm_dir, corner_pos_abs - start_seg);

      // distance the pin position is above the line segment surface
      double pin_clearance = Vdot(norm_dir, pin_pos - start_seg);
      double lim_angle = CH_C_PI_4; // don't rotate the shoe more than 45 deg. 
      double psi = 0;

      // rotate the max amount in this case
      if( corner_clearance > corner_pos_len*std::cos(lim_angle) )
      {
        psi = lim_angle;
      }
      else
      {
        // place the shoe relative to the pin location, so the next pin location is set such 
        //  that the next shoe body can be easily aligned to the line segment.
        ChVector<> r0 = corner_pos_abs - pin_pos;
        // project pin center down to the line segment, normal to segment. Distance to r1.
        double len_on_seg = std::sqrt( pow(corner_pos_len,2) - pow(pin_clearance,2) );
        ChVector<> r1 = -norm_dir * pin_clearance + tan_dir * len_on_seg;
        // rotate shoe about the z-axis of the pin, at the pin
        psi = std::acos( Vdot(r0.GetNormalized(), r1.GetNormalized()) );
        ChQuaternion<> rot_frame =  Q_from_AngAxis(-psi, pin_frame.GetRot().GetZaxis() );
        pin_frame = rot_frame * pin_frame;

        // can find the shoe COG pos/rot now, from pin orientation, and COG offset from pin pos
        COG_frame.SetRot(pin_frame.GetRot() );
        COG_frame.SetPos(pin_frame * COG_to_pin_rel); 

        // let's check to see if the criteria for setting m_aligned_with_seg = true.
        // Note: the function also sets the bool
        bool check_alignment = check_shoe_aligned(COG_frame*COG_to_pin_rel, start_seg, end_seg, norm_dir);
        if( 1 ) 
          GetLog() << " aligned ?? shoe # : " << m_numShoes << " ?? " << check_alignment << "\n";
      }
    }

    // COG_frame is set correctly, add the body to the system
    m_shoes.back()->SetPos(COG_frame.GetPos() );
    m_shoes.back()->SetRot(COG_frame.GetRot() );
    chassis->GetSystem()->Add(m_shoes.back());

    // create and init. the pin between the last shoe and this one, add to system.
    m_pins.push_back(ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute));
    m_pins.back()->SetNameString(" pin " + std::to_string(m_pins.size()) );
    // pin frame picked up from COG_frame, where z-axis should be in the lateral direction.
    m_pins.back()->Initialize(m_shoes.back(), m_shoes.end()[-2], ChCoordsys<>(pin_frame.GetPos(), pin_frame.GetRot() ) );
    chassis->GetSystem()->AddLink(m_pins.back());
    
    // See if the end of the linear segment has been reached.
    //  Could use either A) or B), but B) is simpler.
    // A) step along the line in the tangent direction, for the added half-pin lengths on 
    //  the last two shoes.
    ChVector<> COG1_pin = pin_frame.GetPos() - (m_shoes.end()[-2])->GetPos();
    ChVector<> pin_COG2 = COG_frame.GetPos() - pin_frame.GetPos();
    ChVector<> pos_on_seg_A = pos_on_seg + tan_dir*(Vdot(COG1_pin, tan_dir) + Vdot(pin_COG2, tan_dir));

    // B) when the COG of the last shoe passes the endpoint.
    // project the COG normal to segment surface.
    double proj_dist = Vdot(norm_dir, COG_frame.GetPos() - start_seg);
    ChVector<> pos_on_seg_B = COG_frame.GetPos() - norm_dir*proj_dist;
    

    // check the largest error term. Say something if it's exceeded.
    if( (pos_on_seg_A - pos_on_seg_B).LengthInf() > 1e-6 )
      GetLog() << " comparing pos_on_seg error: " << pos_on_seg_A - pos_on_seg_B << "\n";

    // update distance using method B
    pos_on_seg = pos_on_seg_B;
    // will become negative when the last created shoe COG center is past the endpoint.
    dist_to_end = Vdot(end_seg - pos_on_seg, tan_dir);
  }

  // At this point, wrap the shoes around the curved segment (if there is one).
  //  E.g., roadwheels 1-2, 2-3, 3-4, 4-5 have no curved segment, so skip.
  if( (end_curve_seg - end_seg).LengthInf() < 1e-3)
  {
    GetLog() << " no curved line segment, at location: " << end_curve_seg << " ... \n";
    return;
  }

  // It is assumed that there is a previous shoe with a pin location that is compatible.
  // Guess the next pin location assuming the same orientation as the last created body.
  ChVector<> next_pin_pos;
  dist_to_end = 1;
  // radius of the next pin position, relative to rolling element center
  double len_r2 = std::sqrt( pow(clearance + m_shoe_chain_Yoffset,2) + pow(m_pin_dist/2.0,2) );
  // tangent direction at the end of the curved segment
  ChVector<> tan_dir_end_curved_seg = Vcross(end_curve_seg - rolling_elem_center, lateral_dir).GetNormalized();

  // stop when furthest pin location on last created shoe body passes the end point
  while(dist_to_end > 0) 
  {
    // create a new body by copying the first, add to handle vector.
    // Copy doesn't set the collision shape.
    // Don't reset visualization assets, copied in ChPhysicsItem::Copy()
    m_shoes.push_back(ChSharedPtr<ChBody>(new ChBody));
    m_numShoes++;
    m_shoes.back()->Copy( m_shoes.front().get_ptr() );
    AddCollisionGeometry();
    m_shoes.back()->SetNameString( "shoe " + std::to_string(m_numShoes) );

    // pin between shoes position is know, orient the same as the previous shoe.
    pin_frame = ChFrame<>(COG_frame*COG_to_pin_rel, COG_frame.GetRot() );
    ChVector<> r1 = pin_frame.GetPos() - rolling_elem_center; // center to first pin

    double len_r1 = r1.Length();
    // COG orientation will need to be modified, according to clearance of the rolling element
    double phi = std::acos( Vdot( -r1 / len_r1, pin_frame.GetRot().GetXaxis() ));
    double h1 = std::sqrt( pow(m_pin_dist,2) - pow( (len_r1*len_r1 - len_r2*len_r2) / (2.0*len_r1), 2) );
    double theta = std::asin( h1 / m_pin_dist );
    double psi = phi - theta;
    // rotate the pin frame
    ChQuaternion<> rot_frame =  Q_from_AngAxis(-psi, pin_frame.GetRot().GetZaxis() );
    pin_frame.SetRot(rot_frame * pin_frame.GetRot());

    if(0)
    {
      GetLog() << " x, y-axis, pre-xform: " << pin_frame.GetRot().GetXaxis() << "\n" << pin_frame.GetRot().GetYaxis() << "\n";
      ChFrame<> pin_frameCHECK = rot_frame * pin_frame;
      GetLog() << " x, y-axis, POST xform: " << pin_frameCHECK.GetRot().GetXaxis() << "\n" << pin_frameCHECK.GetRot().GetYaxis() <<"\n";
    }

    // COG rotation is the same as the rotated pin
    COG_frame.SetRot(pin_frame.GetRot());

    // Find COG pos w/ the newly oriented pin,
    COG_frame.SetPos( pin_frame * COG_to_pin_rel );
    //  set the body info, add to system
    m_shoes.back()->SetPos(COG_frame.GetPos() );
    m_shoes.back()->SetRot(COG_frame.GetRot() );
    chassis->GetSystem()->Add(m_shoes.back());

    // create and init. the pin between the last shoe and this one, add to system.
    m_pins.push_back(ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute));
    m_pins.back()->SetNameString(" pin " + std::to_string(m_pins.size()) );
    // pin frame picked up from COG_frame, where z-axis should be in the lateral direction.
    m_pins.back()->Initialize(m_shoes.back(), m_shoes.end()[-2], ChCoordsys<>(pin_frame.GetPos(), pin_frame.GetRot() ) );
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
void TrackChain::CreateShoes_closeChain(ChSharedPtr<ChBodyAuxRef> chassis,
  const ChVector<>& start_seg,
  const ChVector<>& end_seg)
{
  // coming off the end of a curved segment, then following a straight line until near the end.
  // Once forward-most pin location crosses that of the first shoe, stop, and modify the first and last bodies.
  // Then, create the final pin joint.


  // get coordinate directions of the envelope surface
  // lateral in terms of the vehicle chassis
  ChVector<> lateral_dir = (m_shoes.front()->GetRot()).GetZaxis();
  ChVector<> tan_dir = (end_seg - start_seg).GetNormalized();
  ChVector<> norm_dir = Vcross(lateral_dir, tan_dir).GetNormalized();   // normal to the envelope surface
  ChMatrix33<> rot_seg_A;  // orientation of line segment
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
  ChVector<> COG_to_pin_rel(m_pin_dist/2.0, 0, 0); 

  // keep going until last created shoe COG pos_on_seg passes the end point.
  // Want to overshoot upon reaching curved segment, to not interpenetrate surface boundary.
  while( dist_to_end > m_pin_dist/2.0 )  
  {
    // create a new body by copying the first, add to handle vector.
    // Copy doesn't set the collision shape.
    // Don't reset visualization assets, copied in ChPhysicsItem::Copy()
    m_shoes.push_back(ChSharedPtr<ChBody>(new ChBody));
    m_numShoes++;
    m_shoes.back()->Copy( m_shoes.front().get_ptr() );
    AddCollisionGeometry();
    m_shoes.back()->SetNameString( "shoe " + std::to_string(m_numShoes) );

    // Find where the pin to the previous shoe should be positioned.
    // From there, calculate what the body pos/rot should be.
    ChVector<> pin_pos = (m_shoes.end()[-2])->GetFrame_COG_to_abs() * COG_to_pin_rel;
    pin_frame = ChFrame<>(pin_pos, (m_shoes.end()[-2])->GetRot() );

    // Creating shoes along the line segment, one of two situations possible:
    // 1) shoe is exactly on the envelope boundary, and exactly parallel (e.g., first segment this is guaranteed).
    // 2) shoe is off the boundary, and will have a rot slightly different than previous shoe to get it closer,
    //      e.g., after a straight segment follows a curved segment.

    // At first,COG frame assume this shoe has same orientation as the previous.
    COG_frame = ChFrame<>((m_shoes.end()[-2])->GetFrame_COG_to_abs() );
    // set the body pos. from the pin_pos w/ the previous shoe;
    COG_frame.SetPos(pin_frame * COG_to_pin_rel);

    // Assumption 1) is true if: 
    //  A) We set the last shoe pin in such a way so this shoe can be exactly aligned with the line segment, or
    //  B) the last shoe on the linear segment was already aligned
    if( m_aligned_with_seg )
    {
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
    }
    else 
    {
      // verify that this configuration:
      //  1) does not cross the line segment boundary, and
      //  2) stays as clsoe as possible to the line segment (e.g., by rotating the body slightly at the pin_pos)
      //  3) does not rotate the body too aggressively

      // For 1), consider a point at the very front edge of a bounding box
      ChVector<> corner_pos_rel( (m_pin_dist + m_shoe_box.x)/2.0, m_shoe_chain_Yoffset, 0);
      double corner_pos_len = corner_pos_rel.Length();
      ChVector<> corner_pos_abs = pin_frame * corner_pos_rel;

      // distance the corner point is above the line segment surface
      double corner_clearance = Vdot(norm_dir, corner_pos_abs - start_seg);

      // distance the pin position is above the line segment surface
      double pin_clearance = Vdot(norm_dir, pin_pos - start_seg);
      double lim_angle = CH_C_PI_4; // don't rotate the shoe more than 45 deg. 
      double psi = 0;

      // rotate the max amount in this case
      if( corner_clearance > corner_pos_len*std::cos(lim_angle) )
      {
        psi = lim_angle;
      }
      else
      {
        // place the shoe relative to the pin location, so the next pin location is set such 
        //  that the next shoe body can be easily aligned to the line segment.
        ChVector<> r0 = corner_pos_abs - pin_pos;
        // project pin center down to the line segment, normal to segment. Distance to r1.
        double len_on_seg = std::sqrt( pow(corner_pos_len,2) - pow(pin_clearance,2) );
        ChVector<> r1 = -norm_dir * pin_clearance + tan_dir * len_on_seg;
        // rotate shoe about the z-axis of the pin, at the pin
        psi = std::acos( Vdot(r0.GetNormalized(), r1.GetNormalized()) );
        ChQuaternion<> rot_frame =  Q_from_AngAxis(-psi, pin_frame.GetRot().GetZaxis() );
        pin_frame = rot_frame * pin_frame;

        // can find the shoe COG pos/rot now, from pin orientation, and COG offset from pin pos
        COG_frame.SetRot(pin_frame.GetRot() );
        COG_frame.SetPos(pin_frame * COG_to_pin_rel); 

        // let's check to see if the criteria for setting m_aligned_with_seg = true.
        // Note: the function also sets the bool
        bool check_alignment = check_shoe_aligned(COG_frame*COG_to_pin_rel, start_seg, end_seg, norm_dir);
        if( 1 ) 
          GetLog() << " aligned ?? shoe # : " << m_numShoes << " ?? " << check_alignment << "\n";
      }
    }

    // COG_frame is set correctly, add the body to the system
    m_shoes.back()->SetPos(COG_frame.GetPos() );
    m_shoes.back()->SetRot(COG_frame.GetRot() );
    chassis->GetSystem()->Add(m_shoes.back());

    // create and init. the pin between the last shoe and this one, add to system.
    m_pins.push_back(ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute));
    m_pins.back()->SetNameString(" pin " + std::to_string(m_pins.size()) );
    // pin frame picked up from COG_frame, where z-axis should be in the lateral direction.
    m_pins.back()->Initialize(m_shoes.back(), m_shoes.end()[-2], ChCoordsys<>(pin_frame.GetPos(), pin_frame.GetRot() ) );
    chassis->GetSystem()->AddLink(m_pins.back());
    
    // See if the end of the linear segment has been reached.
    ChVector<> next_pin_loc = COG_frame * COG_to_pin_rel;
    // project this point down to the surface of the line segment.
    double proj_dist = Vdot(norm_dir, next_pin_loc - start_seg);
    ChVector<> pos_on_seg = next_pin_loc - norm_dir*proj_dist;
    
    // stop when the next pin pos on seg. is within half a shoe width of the beginning pos on seg.
    dist_to_end = Vdot(end_seg - pos_on_seg, tan_dir);
  }

  // If assumptions were followed throughout, first shoe is somewhere near the middle of the gear and idler.
  // The first and last shoes ought to be aligned at first, e.g. same Rot().
  // If the stars align, the pin locations will line up
  ChVector<> last_pin_pos = m_shoes.back()->GetFrame_COG_to_abs() * COG_to_pin_rel;
  ChVector<> first_pin_pos = m_shoes.front()->GetFrame_COG_to_abs() * COG_to_pin_rel;

  // no way does this occur accidentally, but check anyway
  if( (first_pin_pos - last_pin_pos).LengthInf() < 1e-6 )
  {
    GetLog() << "You are super lucky ! \n\n";

  }
  else
  {
    // pin overlap distance
    ChVector<> r_last_first = last_pin_pos - first_pin_pos;
    // assuming both shoes are aligned (same Rot), rotation is symmetric and angle is:
    double rotAng = std::acos( (m_pin_dist - r_last_first.Length()/2.0) / m_pin_dist);
    
    // rotate the first body about it's 2nd pin by -rotAng
    COG_frame = ChFrame<>(m_pins.front()->GetMarker2()->GetPos(), m_shoes.front()->GetRot() );
    ChQuaternion<> rot_q = Q_from_AngAxis( -rotAng, lateral_dir);
    COG_frame = rot_q * COG_frame;

    // now, reposition the COG backwards from the first pin location
    COG_frame.SetPos( COG_frame * -COG_to_pin_rel);
    // know the COG frame for the first shoe, set the body info
    m_shoes.front()->SetPos(COG_frame.GetPos() );
    m_shoes.front()->SetRot(COG_frame.GetRot() );

    // rotate the last body about it's 1st pin by rotAng
    COG_frame = ChFrame<>(m_pins.back()->GetMarker1()->GetPos(), m_shoes.back()->GetRot() );
    rot_q = Q_from_AngAxis( rotAng, lateral_dir);
    COG_frame = rot_q * COG_frame;

    // reposition COG of last body forward from the last pin
    COG_frame.SetPos( COG_frame * COG_to_pin_rel);
    // set the pos, rot of the last shoe body
    m_shoes.back()->SetPos(COG_frame.GetPos() );
    m_shoes.back()->SetRot(COG_frame.GetRot() );

    // TODO: did I just rotate the markers on the first and last ChLinkLockRevolute objects,
    //       when using SetRot() on the shoe body, since they're connected?
    // I may have to rotate Marker2 on the first pin, Marker1 on the last pin, align them to the COG_frame.
    // Might just mess up reported measured values for initial conditions.

  }

  // re-positioned the first and last shoe to share final pin.
  ChVector<> pin_pos_abs = m_shoes.back()->GetFrame_COG_to_abs() * COG_to_pin_rel;

  // create and init. the pin between the last shoe and first, add to system.
  m_pins.push_back(ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute));
  m_pins.back()->SetNameString(" pin " + std::to_string(m_pins.size()) );
  // use the orientation of the last shoe added
  m_pins.back()->Initialize(m_shoes.front(), m_shoes.back(), ChCoordsys<>(pin_pos_abs, m_shoes.back()->GetRot() ) );
  chassis->GetSystem()->AddLink(m_pins.back());

}

// all inputs in absolute coords.
// Sets m_aligned_with_seg, and returns it also
bool TrackChain::check_shoe_aligned(const ChVector<>& pin2_pos_abs,
    const ChVector<>& start_seg_pos,
    const ChVector<>& end_seg_pos,
    const ChVector<>& seg_norm_axis)
{
  bool out = false;
  ChVector<> tan_hat = (end_seg_pos - start_seg_pos).GetNormalized();
  // criteria 1) pin2 should be less than m_shoe_chain_Yoffset from the line segment.
  //  If it's too much less, the shoe might penetrate the envelope, so cap it in either direction.
  double dist = Vdot(seg_norm_axis, pin2_pos_abs - start_seg_pos);
  if( abs(dist - m_shoe_chain_Yoffset) < 1e-3 )
    out = true;

  // set the outcome, and return it
  m_aligned_with_seg = out;
  return out;
}

ChSharedPtr<ChBody> TrackChain::GetShoeBody(size_t track_idx)
// ChSharedPtr<ChBodyAuxRef> TrackChain::GetShoeBody(size_t track_idx)
{
  assert( track_idx < m_numShoes);
  return (track_idx > m_numShoes-1) ? m_shoes[track_idx] : m_shoes[0] ;
}

} // end namespace chrono
