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
// HMMWV wheel subsystem
//
// =============================================================================

#ifndef HMMWV9_WHEEL_H
#define HMMWV9_WHEEL_H

#include "subsys/ChWheel.h"

#include "HMMWV9.h"

namespace hmmwv9 {

class HMMWV9_Wheel : public chrono::ChWheel {
public:

  HMMWV9_Wheel(bool               enableContact,
               double             mu,
               VisualizationType  visType);

  ~HMMWV9_Wheel() {}

  virtual double getMass() const { return m_mass; }
  virtual const chrono::ChVector<>& getInertia() { return m_inertia; }

  static const std::string& MeshName() { return m_meshName; }
  static const std::string& MeshFile() { return m_meshFile; }

private:
  virtual void OnInitialize(chrono::ChSharedBodyPtr body);

  bool               m_contact;
  double             m_mu;
  VisualizationType  m_visType;

  static const std::string  m_meshName;
  static const std::string  m_meshFile;

  static const double  m_radius;
  static const double  m_width;
  static const double  m_mass;
  static const chrono::ChVector<>  m_inertia;

};


} // end namespace hmmwv9


#endif
