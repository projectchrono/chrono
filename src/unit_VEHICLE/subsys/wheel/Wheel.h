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
// Vehicle wheel constructed with data from file (JSON format).
//
// =============================================================================

#ifndef WHEEL_H
#define WHEEL_H

#include "subsys/ChApiSubsys.h"
#include "subsys/ChWheel.h"

namespace chrono {


class CH_SUBSYS_API Wheel : public ChWheel {
public:
  Wheel(const std::string& filename);
  ~Wheel() {}

  virtual double getMass() const         { return m_mass; }
  virtual const ChVector<>& getInertia() { return m_inertia; }

  virtual void Initialize(ChSharedBodyPtr spindle);

  void ExportMeshPovray(const std::string& out_dir);

private:
  enum VisMode {
    NONE,
    PRIMITIVES,
    MESH
  };

  double      m_mass;
  ChVector<>  m_inertia;

  VisMode     m_vis;
  double      m_radius;
  double      m_width;
  std::string m_meshName;
  std::string m_meshFile;
};


} // end namespace chrono


#endif
