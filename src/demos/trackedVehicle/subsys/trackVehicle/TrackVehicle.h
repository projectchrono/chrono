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
#include "subsys/trackSystem/TrackSystem.h"
#include "subsys/powertrain/TrackPowertrain.h"

namespace chrono {

class CH_SUBSYS_API TrackVehicle : public ChSystem
{
public:

  TrackVehicle(bool fixed = false, bool chassisVis = false);

  ~TrackVehicle() {}

  int GetNum_TrackSystems() const { return m_num_tracks; }

  virtual ChCoordsys<> GetLocalDriverCoordsys() const { return m_driverCsys; }

  void Initialize(const ChCoordsys<>& chassisPos);
  
  void Update(double	time,
              double	left_drive_input,
			        double	right_drive_input);

  /// Set the integration step size for the vehicle system.
  void SetStepsize(double val) { m_stepsize = val; }

  /// Get the current value of the integration step size for the vehicle system.
  double GetStepsize() const { return m_stepsize; }

private:

  void Load_TrackSystem(const std::string& filename, int track);


  ChSharedPtr<ChBodyAuxRef> m_chassis;  ///< hull body
  int m_num_tracks;       // number of tracks for this vehicle

  std::vector<ChVector<> > m_TrackSystem_locs;   // locations of the track system c-sys relative to chassis
  std::vector<ChSharedPtr<TrackSystem> > m_TrackSystems;	// list of track systems

  ChSharedPtr<TrackPowertrain> m_ptrain;  ///< powertrain system

  static const double     m_Mass;                   // chassis mass
  static const ChVector<> m_COM;                    // location of the chassis COM in the local ref frame
  static const ChVector<> m_Inertia;                // symmetric moments of inertia of the chassis

  static const ChCoordsys<> m_driverCsys;  // driver position and orientation relative to chassis
  static const std::string m_MeshFile;

  double m_stepsize;          ///< integration time step for tracked vehicle system

  // friend class irrDriver
};


} // end namespace chrono


#endif
