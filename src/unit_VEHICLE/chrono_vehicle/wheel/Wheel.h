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

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChWheel.h"

#include "thirdparty/rapidjson/document.h"

namespace chrono {


class CH_VEHICLE_API Wheel : public ChWheel
{
public:

  Wheel(const std::string& filename);
  Wheel(const rapidjson::Document& d);
  ~Wheel() {}

  virtual void Initialize(ChSharedBodyPtr spindle);

  virtual double GetMass() const { return m_mass; }
  virtual ChVector<> GetInertia() const { return m_inertia; }
  virtual double GetRadius() const { return m_radius; }
  virtual double GetWidth() const { return m_width; }

  void SetRadius(double rad) { m_radius = rad; }
  void SetWidth(double width) { m_width = width; }

  bool UseVisualizationMesh() const          { return m_vis == MESH; }
  bool UseVisualizationPrimitives() const    { return m_vis == PRIMITIVES; }
  const std::string& GetMeshFilename() const { return m_meshFile; }
  const std::string& GetMeshName() const     { return m_meshName; }

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
