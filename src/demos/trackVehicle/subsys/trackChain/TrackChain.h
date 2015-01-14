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
// Generates a track chain around the track system
//
// =============================================================================

#ifndef TRACKCHAIN_H
#define TRACKCHAIN_H

#include "subsys/ChApiSubsys.h"
#include "assets/ChTriangleMeshShape.h"
#include "physics/ChSystem.h"
#include "physics/ChBody.h"
#include "physics/ChBodyAuxRef.h"

#include "ModelDefs.h"

namespace chrono {

/// Generates and manages the track chain system, which is typically generated
/// with a single track shoe/pin geometry set.
class CH_SUBSYS_API TrackChain : public ChShared
{
public:

  TrackChain(const std::string& name, 
    VisualizationType vis = VisualizationType::PRIMITIVES,
    CollisionType collide = CollisionType::PRIMITIVES );
  
  ~TrackChain() {}

  /// Pass in a vector of center of rolling element bodies w.r.t. chassis c-sys.
  /// Use the clearance to define a spherical envelope for each rolling element.
  /// An envelope is where the surface of the track chain will not penetrate.
  /// Start_loc should be somewhere between the idler and driveGear, e.g. the top of the chain.
  /// Start_loc is expressed w.r.t chassis c-sys.
  /// NOTE: control_points begin and end with the idler and driveGear
  void Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
    const std::vector<ChVector<>>& rolling_element_loc,
    const std::vector<double>& clearance,
    const ChVector<>& start_loc);
  
  /// handle to the shoe body
  ChSharedPtr<ChBody> GetShoeBody(size_t track_idx); 

private:

  // private functions
  /// add visualization assets to a track shoe
  void AddVisualization(size_t track_idx);

  /// add collision geometrey to a track shoe
  void AddCollisionGeometry(size_t track_idx);
   
  /// initialize shoe bodies by wrapping the track chain around the rolling elements.
  /// Define a string along which the chain is wrapper, using the input values.
  /// Note: start_loc_abs should be between the idler and sprockets
  void CreateChain(const std::vector<ChFrame<>>& control_points_abs,
    const std::vector<ChFrame<>>& rolling_element_abs,
    const std::vector<double>& clearance,
    const ChVector<>& start_pos_abs );

  /// create the shoes along a line segment and associated curved section that define
  /// the collision envelope.
  /// Returns the current point along the line where we last created a shoe body.
  ChVector<> CreateShoes(const ChVector<>& curr_pos,
    const ChVector<>& start_seg,
    const ChVector<>& end_seg,
    const ChVector<>& end_curve,
    const ChVector<>& rolling_elem_center,
    double clearance);

    // private functions
  const std::string& getMeshName() const { return m_meshName; }
  const std::string& getMeshFile() const { return m_meshFile; }
  const std::string& getCollisionFilename() const { return m_collisionFile; }

  // private variables
  std::vector<ChSharedPtr<ChBody>> m_shoes;  ///< handle to track shoes
  std::vector<ChSharedPtr<ChLinkLockRevolute>> m_pins; ///< handles to pin joints
  size_t m_numShoes;      ///< number of track shoes and pins


  VisualizationType m_vis;    // visual asset geometry type
  CollisionType m_collide;    // collision geometry type

//  ChSharedPtr<ChTriangleMeshShape> m_geom_visual;
  
  //static values 
  static const double m_mass;         // mass per shoe
  static const ChVector<> m_inertia;  // inertia of a shoe

  static const double m_shoe_width_box;
  static const double m_shoe_width_cyl;
  static const double m_shoe_height;
  static const double m_shoe_chain_offset;  // figure this from your collision mesh

  static const double m_pin_dist;		  // linear distance between a shoe's two pin joint center
  static const double m_pin_radius;
  

  static const std::string m_collisionFile;	// collision geometry filename
  static const std::string m_meshFile;  // visual geometry mesh filename
  static const std::string m_meshName;  // name of the mesh

};


} // end namespace chrono


#endif
