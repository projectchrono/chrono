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
// Vehicle wheel constructed with data from file (JSON format).
//
// =============================================================================

#ifndef WHEEL_H
#define WHEEL_H

#include "subsys/ChApiSubsys.h"
#include "subsys/ChWheel.h"

#include "rapidjson/document.h"

namespace chrono {


class CH_SUBSYS_API Wheel : public ChWheel
{
public:

  Wheel(const std::string& filename);
  Wheel(const rapidjson::Document& d);
  ~Wheel() {}

  virtual void Initialize(ChSharedBodyPtr spindle);

  /// Accessors
  virtual double GetMass() const { return m_mass; }
  virtual ChVector<> GetInertia() const { return m_inertia; }
  virtual double GetRadius() const { return m_radius; }
  virtual double GetWidth() const { return m_width; }

  /// set variables from ChTire
  void SetRadius(double rad) { m_radius = rad; }
  void SetWidth(double width) { m_width = width; }


  void ExportMeshPovray(const std::string& out_dir);

private:

  enum VisMode {
    NONE,
    PRIMITIVES,
    MESH
  };

  void Create(const rapidjson::Document& d);

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
