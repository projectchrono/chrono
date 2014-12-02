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
// Tracked vehicle model built from subsystems specified w/ JSON input data file
//
// =============================================================================

#ifndef TRACKVEHICLE_H
#define TRACKVEHICLE_H

#include "core/ChCoordsys.h"
#include "physics/ChSystem.h"

namespace chrono {

class CH_SUBSYS_API TrackTrackVehicle
{
public:

  TrackVehicle(const std::string& filename, bool chassis_fixed = false);

  ~TrackVehicle() {}

  int GetNum_TrackSystems() const { return m_num_tracks; }

  virtual ChCoordsys<> GetLocalDriverCoordsys() const { return m_driverCsys; }

  void Initialize(const ChCoordsys<>& chassisPos);
  
  void Update(double	time,
              double	left_drive_input,
			  double	right_drive_input);


private:

  void Load_TrackSystem(const std::string& filename, int track);

private:

  int                      m_num_tracks;       // number of axles for this vehicle

  std::vector<ChVector<> > m_TrackSystem_locs;   // locations of the track system c-sys relative to chassis
  std::vector<ChSharedPtr<TrackSystem> > m_TrackSystems;	// list of track systems

  double     m_Mass;                   // chassis mass
  ChVector<> m_COM;                    // location of the chassis COM in the local ref frame
  ChVector<> m_Inertia;                // symmetric moments of inertia of the chassis

  ChCoordsys<> m_driverCsys;                  // driver position and orientation relative to chassis
};


} // end namespace chrono


#endif
