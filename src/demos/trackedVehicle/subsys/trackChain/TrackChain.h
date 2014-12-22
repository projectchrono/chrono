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

namespace chrono {

/// Generates and manages the track chain system, which is typically generated
/// with a single track shoe/pin geometry set.
class CH_SUBSYS_API TrackChain : public ChShared
{
public:

  TrackChain(const std::string& filename);
  
  ~TrackChain() {}

  /// pass in a vector of bodies to wrap the chain around.
  /// Use the clearnance as a spherical offset from the body CM points.
  /// Start the first shoe and location of first/last pin joint at the start_loc
  void Initialize(std::vector<ChBody>& control_bodies,
					std::vector<double> clearance,
					const ChVector<>& start_loc);
  
  
  ChSharedPtr<ChBody> GetShoeBody(int track_idx) { return (track_idx > m_numShoes-1) ? m_shoes[track_idx] : m_shoes[0] ; }

private:
  int Create();
  
  // private functions
  void AddVisualizationIdler();
  
  // private variables
  std::vector<ChSharedPtr<ChBody>> m_shoes;
  int m_numShoes;
  
  double m_PinDist;		// linear distance between a shoe's two pin joint center
  std::string m_collision_filename;	// wavefront mesh for collision geometry
  std::string m_visual_filename;	// wavefront mesh for visuals
  ChSharedPtr<ChTriangleMeshShape> m_geom_visual;
  

};


} // end namespace chrono


#endif
