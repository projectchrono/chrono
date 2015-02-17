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
// Authors: Justin Madsen
// =============================================================================

#ifndef TRACK_FUNCDRIVER_H
#define TRACK_FUNCDRIVER_H

#include "subsys/base/ChDriverTrack.h"
#include "motion_functions/ChFunction_Sine.h"

namespace chrono {

class CH_SUBSYS_API Track_FuncDriver : public ChDriverTrack
{
public:

  /// constructor generates a sine wave for the throttle (0 to 1) and applies it after a settling time.
  Track_FuncDriver(int num_tracks,
    double func_freq = 0.2,
    double func_amp = 1.0,
    double time_start = 1.0
  ): ChDriverTrack(num_tracks),
  m_throttle_func( ChSharedPtr<ChFunction_Sine>(new ChFunction_Sine(0, func_freq,func_amp)) ),
  m_t_begin(time_start)
  { assert(func_amp <= 1.0); }
  
  ~Track_FuncDriver() {}

  virtual void Update(double time)
  {
    if(time > m_t_begin)
    {
      m_throttle[0] = m_throttle_func->Get_y(time - m_t_begin);

    }

  }

private:

  ChSharedPtr<ChFunction_Sine> m_throttle_func;
  double m_t_begin;

};


} // end namespace chrono


#endif