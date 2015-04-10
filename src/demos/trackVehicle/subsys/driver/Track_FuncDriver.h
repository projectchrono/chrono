// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Justin Madsen 2015
//
//  Implements a ChTrackDriven base class, interacts with a track vehicle each timestep.
//  Templated based on the function you are passing in.
// =============================================================================

#ifndef TRACK_FUNCDRIVER_H
#define TRACK_FUNCDRIVER_H

#include "motion_functions/ChFunction_Base.h"
#include "subsys/base/ChDriverTrack.h"
// #include "core/ChLog.h"

namespace chrono {

template <class Function_T = ChFunction_Sine>
class Track_FuncDriver : public ChDriverTrack
{
private:

  // data members
  ChSharedPtr<ChFunction> m_throttle_func;
  double m_t_begin;
  double m_throttle_min;  // minimum throttle value
  double m_throttle_max;  // max throttle value

public:

  /// constructor generates a function to apply to the throttle (0 to 1)
  /// after some time to allow for settling.
  explicit Track_FuncDriver(int num_tracks,
    ChSharedPtr<Function_T> throttle_func,
    double time_start,
    double throttle_min = -1.0,
    double throttle_max = 1.0
): ChDriverTrack(num_tracks),
   m_t_begin(time_start)
  {
    // make sure the func and the templated type are the same
    ChSharedPtr<ChFunction> fun =  throttle_func.DynamicCastTo<ChFunction>();
    assert(fun);
    m_throttle_func = fun;

    // set min/max throttle values
    assert(throttle_min < throttle_max);
    m_throttle_max = throttle_max;
    m_throttle_min = throttle_min;
  }
  
  ~Track_FuncDriver() {}

  virtual void Update(double time)
  {
    if(time > m_t_begin)
    {
      for(size_t track_id = 0; track_id < m_throttle.size(); track_id++)
      {
        SetThrottle(track_id, m_throttle_func->Get_y(time - m_t_begin), m_throttle_min, m_throttle_max );
      }
    }

  }

  /// get a handle to the throttle function
  ChSharedPtr<Function_T> GetThrottleFunc() { return m_throttle_func.DynamicCastTo<Function_T>(); }

};


} // end namespace chrono


#endif