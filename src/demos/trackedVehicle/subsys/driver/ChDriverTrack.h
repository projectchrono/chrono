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
// Authors: Justin Madsen, Radu Serban
// =============================================================================
//
// Base class for a tracked vehicle driver. A driver system must be able to report the
// current values of the inputs (throttle, braking) for each track system
//
// =============================================================================

#ifndef CH_DRIVER_TRACK_H
#define CH_DRIVER_TRACK_H

#include <string>

#include "core/ChShared.h"
#include "physics/ChSystem.h"

#include "subsys/ChApiSubsys.h"

namespace chrono {

/// Base class for a tracked vehicle driver system.
/// A driver system must be able to report the current values of the inputs,
/// which are the inputs to each drive gear system
class CH_SUBSYS_API ChDriverTrack : public ChShared
{
public:

  ChDriverTrack(int num_tracks);
  virtual ~ChDriverTrack() {}

  /// Get the driver throttle input (in the range [0,1])
  double GetThrottle(int track_idx) const { return m_throttle[track_idx]; }

  /// Get the driver braking input (in the range [0,1])
  double GetBraking(int track_idx) const  { return m_braking[track_idx]; }

  /// Update the state of this driver system at the current time.
  virtual void Update(double time) {}

  /// Advance the state of this driver system by the specified time step.
  virtual void Advance(double step) {}

  /// Initialize output file for recording driver inputs.
  bool LogInit(const std::string& filename);

  /// Record the current driver inputs to the log file.
  bool Log(double time);

protected:
  /// clamp to interval
  double clamp(double val, double min_val, double max_val);

  /// Set the value for the driver throttle input.
  void SetThrottle(double val, int track_idx, double min_val = 0, double max_val = 1);

  /// Set the value for the driver braking input.
  void SetBraking(double val, int track_idx, double min_val = 0, double max_val = 1);

  /// no need for a track index, modify both at once in ChIrrGuiTrack
  virtual void SetSteering(double val, double min_val = 0, double max_val = 1);

  std::vector<double> m_throttle;   ///< current value of throttle input for each track system
  std::vector<double> m_braking;    ///< current value of braking input for each track system
  double m_steering;                ///< current value of steer input, modifies throttle only, for now

private:
  std::string  m_log_filename;  // name of output file for recording driver inputs

};


} // end namespace chrono


#endif