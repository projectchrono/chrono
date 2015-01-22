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
// Base class for a Tracked vehicle driver. A driver object must be able to report the
// current values of the inputs (throttle, braking). To set these
// values, a concrete driver class can implement the virtual method Update()
// which will be invoked at each time step.
//
// =============================================================================

#include <iostream>
#include <sstream>
#include <fstream>

#include "subsys/driver/ChDriverTrack.h"


namespace chrono {

ChDriverTrack::ChDriverTrack(int num_tracks)
: m_log_filename("")
{
  m_throttle.resize(num_tracks, 0);
  m_braking.resize(num_tracks, 0);
}

bool ChDriverTrack::LogInit(const std::string& filename)
{
  m_log_filename = filename;

  std::ofstream ofile(filename.c_str(), std::ios::out);
  if (!ofile)
    return false;

  ofile << "Time\tTrackIdx\tThrottle\tBraking\n";
  ofile.close();
  return true;
}


// Record the current driver inputs to the log file.
bool ChDriverTrack::Log(double time)
{
  if (m_log_filename.empty())
    return false;

  std::ofstream ofile(m_log_filename.c_str(), std::ios::app);
  if (!ofile)
    return false;

  for(int i = 0; i < m_throttle.size(); i++) 
  {
    ofile << time << "\t" << i << "\t" << m_throttle[i] <<  "\t" << m_braking[i] << std::endl;
  }
  ofile.close();
  return true;
}


// -----------------------------------------------------------------------------
// Clamp a specified input value to appropriate interval.
double ChDriverTrack::clamp(double val, double min_val, double max_val)
{
  if (val <= min_val)
    return min_val;
  if (val >= max_val)
    return max_val;
  return val;
}

// set a single throttle value
void ChDriverTrack::SetThrottle(double val, int track_idx, double min_val, double max_val)
{
  m_throttle[track_idx] = clamp(val, min_val, max_val);
}

/// increment and set all the throttles at once
void ChDriverTrack::SetThrottle(double delta_throttle, double min_val, double max_val)
{
  for(int i = 0; i < m_throttle.size(); i++)
  {
    SetThrottle(m_throttle[i] + delta_throttle, i);
    // if throttle if ever positive, be sure brakes are turned off
    if (m_throttle[i] > 0)
        SetBraking(m_braking[i] - delta_throttle, i);
    else
        SetBraking(m_braking[i] + delta_throttle, i);
  }
}

// set a single braking value
void ChDriverTrack::SetBraking(double val, int track_idx, double min_val, double max_val)
{
  m_braking[track_idx] = clamp(val, min_val, max_val);
}

// set a single steering value. 0 = full left, 1 = right
void ChDriverTrack::SetSteering(double delta_steering, double min_val, double max_val)
{
  m_steering = clamp(m_steering + delta_steering, min_val, max_val);
  if( delta_steering < 0) 
  {
    // left down, right up = left turn
    SetThrottle( m_throttle[0] - delta_steering, 0);
    SetThrottle( m_throttle[1] + delta_steering, 1);
  }
  else
  {
    // left up, right down = right turn
    SetThrottle( m_throttle[0] + delta_steering, 0);
    SetThrottle( m_throttle[1] - delta_steering, 1);
  }
}

}  // end namespace chrono