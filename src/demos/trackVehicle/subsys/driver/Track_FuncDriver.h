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

namespace chrono {

class Track_FuncDriver : public ChDriverTrack
{
public:

  Track_FuncDriver(int num_tracks):
  ChDriverTrack(num_tracks) {}
  ~Track_FuncDriver() {}

  virtual void Update(double time);

private:

};


} // end namespace chrono


#endif