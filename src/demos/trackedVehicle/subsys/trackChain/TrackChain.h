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

  /// pass in a vector of bodies to wrap the chain around.
  /// Use the clearnance as a spherical offset from the body CM points.
  /// Start the first shoe and location of first/last pin joint at the start_loc
  void Initialize(std::vector<ChBody>& control_bodies,
					std::vector<double> clearance,
					const ChVector<>& start_loc);
  
  /// handle to the shoe body
  ChSharedPtr<ChBody> GetShoeBody(size_t track_idx); 

private:

  // private functions
  /// add visualization assets to a track shoe
  void AddVisualization(size_t track_idx);

  /// add collision geometrey to a track shoe
  void AddCollisionGeometry(size_t track_idx);
   
  /// "wrap" the track chain around the other elements in this TrackSystem
  void CreateChain( );

    // private functions
  const std::string& getMeshName() const { return m_meshName; }
  const std::string& getMeshFile() const { return m_meshFile; }
  const std::string& getCollisionFilename() const { return m_collisionFile; }

  // private variables
  std::vector<ChSharedPtr<ChBody>> m_shoes;
  size_t m_numShoes;

  VisualizationType m_vis;    // visual asset geometry type
  CollisionType m_collide;    // collision geometry type

//  ChSharedPtr<ChTriangleMeshShape> m_geom_visual;
  
  //static values 
  static const double m_mass;         // mass per shoe
  static const ChVector<> m_inertia;  // inertia of a shoe

  static const double m_shoe_width;
  static const double m_shoe_height;

  static const double m_pin_dist;		  // linear distance between a shoe's two pin joint center
  static const double m_pin_radius;

  static const std::string m_collisionFile;	// collision geometry filename
  static const std::string m_meshFile;  // visual geometry mesh filename
  static const std::string m_meshName;  // name of the mesh

};


} // end namespace chrono


#endif
