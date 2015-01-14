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
const ChVector<> TrackChain::m_inertia(0.04, 0.22, 0.25);
const double TrackChain::m_shoe_width_box = 0.38; // total width of box
const double TrackChain::m_shoe_width_cyl = 0.531; // total width of cylinder pins
const double TrackChain::m_shoe_height = 0.0663; 
const double TrackChain::m_pin_dist = 0.21;		// linear distance between a shoe chain spacing. exact = 0.205
const double TrackChain::m_pin_radius = 0.02317;
 // distance between body center and the vertical offset to the inner-surface of the collision geometry
//  used for initializing shoes as a chain
const double TrackChain::m_shoe_chain_offset = 0.035; // .03315 exact

TrackChain::TrackChain(const std::string& name, 
                       VisualizationType vis, 
                       CollisionType collide)
                       : m_vis(vis), m_collide(collide), m_numShoes(0)
{
  // clear vector holding list of body handles
  m_shoes.clear();
  // add first track shoe body
  m_shoes.push_back(ChSharedPtr<ChBody>(new ChBody));
  m_shoes[0]->SetNameString("shoe 1, "+name);
  m_shoes[0]->SetMass(m_mass);
  m_shoes[0]->SetInertiaXX(m_inertia);
  m_numShoes++;

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
      ChVector<> tan_dir = r_21.Normalize();

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
        start_cen.Normalize();
        double theta = std::acos( Vdot(start_cen, rad_dir));  // angle between direction vectors
        double phi = std::acos(clearance[i] / start_cen.Length() );
        double rot_ang = theta - phi;
        // find the radial direction from the center of adjacent rolling elements
        // for seg i, norm = r12 x r32
        norm_dir = Vcross(rolling_element_loc[num_elem-1] - rolling_element_loc[0],
          rolling_element_loc[1] - rolling_element_loc[0]);
        norm_dir.Normalize();
        // rotate rad_dir about norm axis
        ChQuaternion<> alpha_rot(Q_from_AngAxis(rot_ang, norm_dir) );
        ChFrame<> rot_frame(ChVector<>(),alpha_rot);
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
        norm_dir = Vcross( rolling_element_loc[num_elem-1] - rolling_element_loc[i-1],
          rolling_element_loc[i] - rolling_element_loc[i-1]);
      } else
      {
        norm_dir = Vcross( rolling_element_loc[i-2] - rolling_element_loc[i-1],
          rolling_element_loc[i] - rolling_element_loc[i-1]);
      }
      norm_dir.Normalize();
      // if the two rolling elements have the same radius, no angle of wrap.
      rad_dir = Vcross(norm_dir, r_21);
      rad_dir.Normalize();
      // when not the same size, find the angle of pulley wrap.
      // Probably overkill, but check that the radial vector is ortho. to r_21
      if( abs(clearance[i] - clearance[i-1]) > 1.0e-3 || abs(Vdot(r_21,rad_dir)) > 1.0e-3)
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
  ChVector<> top_center = rolling_element_loc[num_elem-1];
  top_center.y += clearance[num_elem-1];
  r_21 = start_loc - top_center;  // first guess at start and end points
  ChVector<> tan_dir = r_21.Normalize();

  // If start_loc is precisely located, this rad_dir will define the end point as it is now.
  rad_dir = top_center - rolling_element_loc[num_elem-1];
  rad_dir.Normalize();
  // If start_loc isn't exactly placed, r_21 won't be tangent to the envelope at point top_center.
  // Enforce that the vector between the start and end points is tangent.
  if( abs(Vdot(tan_dir,rad_dir)) > 1e-3 )
  {
    // move top_center so r_21 is orthogonal to rad_dir
    // i.e., rad_dir dot r_21 = 0
    // easier to figure out the relative angle rad_dir needs to be rotated about the norm axis.
    ChVector<> start_cen = start_loc - rolling_element_loc[num_elem-1];
    start_cen.Normalize();
    double theta = std::acos( Vdot(start_cen, rad_dir));  // angle between vector directions
    double phi = std::acos(clearance[num_elem-1] / start_cen.Length() );
    double rot_ang = theta - phi;
    // find the radial direction from the center of adjacent rolling elements
    //  norm = r12 x r32
    norm_dir = Vcross( rolling_element_loc[num_elem-2] - rolling_element_loc[num_elem-1],
        rolling_element_loc[0] - rolling_element_loc[num_elem-1]);
    norm_dir.Normalize();
    // rotate rad_dir about norm axis
    ChQuaternion<> alpha_rot(Q_from_AngAxis(rot_ang, norm_dir) );
    ChFrame<> rot_frame(ChVector<>(),alpha_rot);
    rad_dir =  rad_dir >> rot_frame;
  }
  // rad_dir is now tangent to r_21, so define the new endpoint of this segment
  start_point = rolling_element_loc[num_elem-1] + rad_dir * clearance[num_elem];

  // last segment, only use the start_point
  control_to_abs.push_back(ChFrame<>(start_point));
  (*control_to_abs.end()).ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs() );

  // "wrap" the track chain around the trackSystem rolling elements, e.g., drive-gear,
  // idler, road-wheels. First and last shoes are allowed to be in any orientation,
  // as long as the final pin joint connects correctly.
  CreateChain(control_to_abs, rolling_to_abs, clearance, start_to_abs.GetPos() );
}

void TrackChain::AddVisualization(size_t track_idx)
{
  assert(track_idx < m_numShoes);
  // Attach visualization asset
  switch (m_vis) {
  case VisualizationType::PRIMITIVES:
  {
    // primitive box, also used for collision.
    // shoes will be added to the same collision family so self-collision can be toggled
    ChSharedPtr<ChBoxShape> box(new ChBoxShape);
    box->GetBoxGeometry().SetLengths( 0.5 * ChVector<>( m_pin_dist-2*m_pin_radius, m_shoe_height, m_shoe_width_box) );
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
    // use a simple box
    m_shoes[track_idx]->GetCollisionModel()->AddBox(m_pin_dist-2*m_pin_radius, m_shoe_height, m_shoe_width_box);

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
void TrackChain::CreateChain(const std::vector<ChFrame<>>& control_points_abs,
                             const std::vector<ChFrame<>>& rolling_element_abs,
                             const std::vector<double>& clearance,
                             const ChVector<>& start_pos_abs)
{
  // Each segment has 2 parts, the initial linear portion, then wrapping around 
  //   the rolling element the start point of the next linear line segment.
  size_t num_elems = clearance.size();
  // keep track of the current position along the line segment
  // defined by projecting the shoe body normal at the shoe COM down to the linear line segment,
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
    curr_pos = CreateShoes(curr_pos, start_seg, end_seg, end_curve, rolling_element_abs[idx].GetPos(), clearance[idx]);
  }
}

// all locations are in the abs ref. frame.
// Some assumptions:
// Body orientation and axis of pin rotation always in the lateral_dir
// 
ChVector<> TrackChain::CreateShoes(const ChVector<>& curr_pos,
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
  ChVector<> tan_dir = end_seg - pos_on_seg;
  tan_dir.Normalize();
  // normal to the envelope surface
  ChVector<> norm_dir = Vcross( lateral_dir, tan_dir);

  // create the shoes along the segment!
  double dist_to_end = Vdot(end_seg - pos_on_seg, tan_dir);
  ChVector<> shoe_pos;
  ChQuaternion<> shoe_rot;
  ChMatrix33<> shoe_rot_A;
  while(dist_to_end > 0 )  // keep going until within half pin dist.
  {
    // build the shoe here, unless it's the first pass thru
    if(m_shoes.size() == 1) 
    {    
      // add collision geometry
      AddCollisionGeometry(0);
    } else 
    {
      // create a new body by copying the first. Should pick up collision shape, etc.
      // just rename it
      m_shoes.push_back(ChSharedPtr<ChBody>(new ChBody( *(m_shoes[0].get_ptr()) )) );
      m_shoes[m_numShoes]->SetNameString( "shoe " + std::to_string(m_numShoes+1) );
    }
    
    // initialize pos, rot of this shoe.
    shoe_pos = pos_on_seg + norm_dir * m_shoe_chain_offset;
    shoe_rot_A.Set_A_axis(tan_dir, -norm_dir, lateral_dir);
    m_shoes[m_numShoes]->SetPos(shoe_pos);
    m_shoes[m_numShoes]->SetRot(shoe_rot_A);

    // done adding this shoe
    m_numShoes++;
    // move along the line segment, in the tangent dir
    pos_on_seg += tan_dir*m_pin_dist;
    // update distance, so we can get out of this loop eventually
    dist_to_end = Vdot(end_seg - pos_on_seg, tan_dir);
  }

  // At this point, wrap the shoes around the curved segment.
  dist_to_end = (end_curve - shoe_pos).Length();
  while(dist_to_end > 0) // keep going until within half spacing dist
  {


  }


  return shoe_pos;
}


ChSharedPtr<ChBody> TrackChain::GetShoeBody(size_t track_idx)
{
  assert( track_idx < m_numShoes);
  return (track_idx > m_numShoes-1) ? m_shoes[track_idx] : m_shoes[0] ;
}

} // end namespace chrono
