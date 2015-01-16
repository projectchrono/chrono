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
  AddVisualization(0);
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
        ChFrame<> rot_frame(ChVector<>(), Q_from_AngAxis(rot_ang, norm_dir));
        rad_dir =  rad_dir >> rot_frame;
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
    curr_pos = CreateShoes(chassis, curr_pos, start_seg, end_seg, end_curve, rolling_element_abs[idx].GetPos(), clearance[idx]);
  }
}

// all locations are in the abs ref. frame.
// Some assumptions:
// Body orientation and axis of pin rotation always in the lateral_dir
// 
ChVector<> TrackChain::CreateShoes(ChSharedPtr<ChBodyAuxRef> chassis,
    const ChVector<>& curr_pos,
    const ChVector<>& start_seg,
    const ChVector<>& end_seg,
    const ChVector<>& end_curve,
    const ChVector<>& rolling_elem_center,
    double clearance)
{
  // used to place next shoe center
  ChVector<> pos_on_seg = curr_pos;
  // lateral in terms of the vehicle chassis
  ChVector<> lateral_dir = Vcross(pos_on_seg-end_seg, end_curve-end_seg);
  lateral_dir.Normalize();
  ChVector<> tan_dir = (end_seg - start_seg).GetNormalized();
  // normal to the envelope surface
  ChVector<> norm_dir = Vcross(lateral_dir, tan_dir);

  // create the shoes along the segment
  // (may not be creating shoes exactly parallel to envelope surface)
  double dist_to_end = Vdot(end_seg - pos_on_seg, tan_dir);
  ChVector<> shoe_pos;
  ChMatrix33<> seg_rot_A; // line segment orientation, x points from start to end.
  // special case: very first shoe
  // A single ChBody shoe is created upon construction, so it be can copied by the other shoes.
  if(m_numShoes == 1) 
  {    
    AddCollisionGeometry(0);
    // initialize pos, rot of this shoe.
    shoe_pos = pos_on_seg + norm_dir * m_shoe_chain_Yoffset;
    seg_rot_A.Set_A_axis(tan_dir, norm_dir, lateral_dir);
    m_shoes.front()->SetPos(shoe_pos);
    m_shoes.front()->SetRot(seg_rot_A);
    // pos, rot set, add to system
    chassis->GetSystem()->Add(m_shoes.front());

  } 

  // when the track chain is not exactly aligned to the envelope, rotate the shoe about the pin.
  // First shoe is where it should be.
  m_aligned_with_seg = true; 
  // keep going until last created shoe COG passes the end point
  while(dist_to_end > 0 )  
  {
    // create a new body by copying the first. Should pick up collision shape, etc.
    // just rename it
    m_shoes.push_back(ChSharedPtr<ChBody>(new ChBody( *(m_shoes[0].get_ptr()) )) );
    // m_shoes.push_back(ChSharedPtr<ChBodyAuxRef>(new ChBodyAuxRef( *(m_shoes[0].get_ptr()) )) );
    m_numShoes++;
    m_shoes.back()->SetNameString( "shoe " + std::to_string(m_numShoes) );

    // Find where the pin to the previous shoe should be positioned.
    // From there, calculate what the body pos/rot should be.
    // Add both the body and the pin.
    ChVector<> COG_to_pin_rel(m_pin_dist/2.0, 0, 0);  // pin is just forward of the COG.
    // use the COG pos, rot of the previous shoe
    ChVector<> pin_pos = (m_shoes.end()[-2])->GetFrame_COG_to_abs() * COG_to_pin_rel;
    
    // creating shoes along the line segment, one of two situations:
    // 1) shoe is exactly on the envelope boundary, and exactly parallel (e.g., first segment this is guaranteed).
    // 2) shoe is off the boundary, and will have a rot slightly different than previous shoe to get it closer
    //    e.g., after a straight segment follows a curved segment.
    ChFrame<> COG_frame( (m_shoes.end()[-2])->GetFrame_COG_to_abs() );
    ChFrame<> pin_frame(pin_pos, COG_frame.GetRot() );
    // set the body pos. from the pin_pos;
    COG_frame.SetPos(pin_frame * COG_to_pin_rel);
    // the last shoe pin is set so this shoe can be exactly aligned with the line segment
    if( m_aligned_with_seg )
    {
      // previous shoe has been positioned so pin location is exactly 
      //  m_shoe_chain_Yoffset above the envelope surface (norm. dir).
      // Use the rotation frame for the line segment for the shoe to keep the x-axis parallel to segment.
      COG_frame.SetRot(seg_rot_A);
      // if the previous shoe x-axis is not parallel to segment, using COG_frame 
      // rot from that shoe will be incorrect. Instead, use the updated pin rot.
      pin_frame.SetRot(seg_rot_A);
      // now the position can be found relative to the newly oriented pin frame.
      COG_frame.SetPos(pin_frame * COG_to_pin_rel);
    }
    else 
    {
      // verify that this configuration:
      //  1) does not cross the line segment boundary, and
      //  2) stays as clsoe as possible to the line segment (e.g., by rotating the body slightly at the pin_pos)
      // For 1), consider a point at the very edge of a bounding box
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
        ChVector<> r0 = corner_pos_abs - pin_pos;
        // project pin center down to the line segment, normal to segment. Distance to r1.
        double len_on_seg = std::sqrt( pow(corner_pos_len,2) - pow(pin_clearance,2) );
        ChVector<> r1 = -norm_dir * pin_clearance + tan_dir * len_on_seg;
        // rotate shoe about the z-axis of the pin, at the pin
        psi = std::acos( Vdot(r0,r1) );
        ChFrame<> rot_frame(ChVector<>(), Q_from_AngAxis(-psi, pin_frame.GetRot().GetZaxis() ));
        pin_frame = pin_frame >> rot_frame;
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

    // COG_frame is set correctly, add the body and pin to the system
    m_shoes.back()->SetPos(COG_frame.GetPos() );
    m_shoes.back()->SetRot(COG_frame.GetRot() );
    chassis->GetSystem()->Add(m_shoes.back());

    m_pins.push_back(ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute));
    m_pins.back()->SetNameString(" pin " + std::to_string(m_numShoes) );
    // pin frame picked up from COG_frame, where z-axis should be in the lateral direction.
    m_pins.back()->Initialize(m_shoes.back(), m_shoes.end()[-2], ChCoordsys<>(pin_frame.GetPos(), pin_frame.GetRot() ) );
    chassis->GetSystem()->AddLink(m_pins.back());
    
    // move along the line segment, in the tangent dir
    pos_on_seg += tan_dir * Vdot(m_pin_dist*COG_frame.GetRot().GetXaxis(), tan_dir);
    // update distance, so we can get out of this loop eventually
    dist_to_end = Vdot(end_seg - pos_on_seg, tan_dir);
  }

  // At this point, wrap the shoes around the curved segment.
  dist_to_end = (end_curve - shoe_pos).Length();
  while(dist_to_end > 0) // keep going until within half spacing dist
  {


  }


  // assume we aren't aligned after creating shoes around the curved segment
  m_aligned_with_seg = false;
  return shoe_pos;
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
