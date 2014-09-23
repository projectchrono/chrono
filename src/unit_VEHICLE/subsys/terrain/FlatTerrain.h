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
// Rigid, flat terrain, no obstacles


#ifndef FLATTERRAIN_H
#define FLATTERRAIN_H

#include "subsys/ChTerrain.h"
#include "subsys/ChApiSubsys.h"

namespace hmmwv {

class CH_SUBSYS_API FlatTerrain : public chrono::ChTerrain {
public:

	FlatTerrain(const int height);

  ~FlatTerrain() {}

  virtual double GetHeight(double x, double y) const { return m_height; }

private:
	double m_height;

};


} // end namespace hmmwv


#endif