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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// =============================================================================

#ifndef MODEL_DEFS_H
#define MODEL_DEFS_H

#include "ChronoT_config.h"

enum VisualizationType {
  NONE,
  PRIMITIVES,
  MESH
};

enum TireModelType {
  RIGID,
  PACEJKA,
  LUGRE
};

enum PowertrainModelType {
  SHAFTS,
  SIMPLE
};

enum DrivelineType {
  FWD,
  RWD,
  AWD
};

enum SuspensionType {
  DOUBLE_WISHBONE,
  SOLID_AXLE,
  MULTI_LINK
};

enum DebugInformation {
  DBG_SPRINGS     = 1 << 0,
  DBG_SHOCKS      = 1 << 1,
  DBG_CONSTRAINTS = 1 << 2,
  DBG_SUSPENSIONTEST = 1 << 3
};


#endif
