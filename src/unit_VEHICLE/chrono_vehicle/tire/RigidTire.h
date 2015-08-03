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
// Authors: Radu Serban
// =============================================================================
//
// Rigid tire constructed with data from file (JSON format).
//
// =============================================================================

#ifndef RIGID_TIRE_H
#define RIGID_TIRE_H

#include "subsys/ChApiSubsys.h"
#include "subsys/tire/ChRigidTire.h"

#include "rapidjson/document.h"

namespace chrono {


class CH_SUBSYS_API RigidTire : public ChRigidTire
{
public:

  RigidTire(const std::string&       filename,
            const chrono::ChTerrain& terrain);
  RigidTire(const rapidjson::Document& d,
            const chrono::ChTerrain&   terrain);
  ~RigidTire() {}

  virtual float getFrictionCoefficient() const { return m_mu; }
  virtual double getRadius() const             { return m_radius; }
  virtual double getWidth() const              { return m_width; }

private:

  void Create(const rapidjson::Document& d);

  float   m_mu;
  double  m_radius;
  double  m_width;
};


} // end namespace chrono


#endif
