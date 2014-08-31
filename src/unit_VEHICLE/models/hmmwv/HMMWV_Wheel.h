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

#ifndef HMMWV_WHEEL_H
#define HMMWV_WHEEL_H

#include "subsys/ChWheel.h"

#include "HMMWV.h"

namespace hmmwv {

class HMMWV_Wheel : public chrono::ChWheel {
public:

  HMMWV_Wheel(VisualizationType  visType);
  ~HMMWV_Wheel() {}

  virtual double getMass() const { return m_mass; }
  virtual const chrono::ChVector<>& getInertia() { return m_inertia; }

  virtual void Initialize(chrono::ChSharedBodyPtr spindle);

  virtual const std::string& getMeshName() const = 0;
  virtual const std::string& getMeshFile() const = 0;

private:

  VisualizationType  m_visType;

  static const double  m_radius;
  static const double  m_width;
  static const double  m_mass;
  static const chrono::ChVector<>  m_inertia;
};


class HMMWV_WheelLeft : public HMMWV_Wheel {
public:
  HMMWV_WheelLeft(VisualizationType  visType);
  ~HMMWV_WheelLeft() {}

  virtual const std::string& getMeshName() const { return m_meshName; }
  virtual const std::string& getMeshFile() const { return m_meshFile; }

  static void ExportMeshPovray(const std::string& out_dir);

private:
  static const std::string  m_meshName;
  static const std::string  m_meshFile;
};


class HMMWV_WheelRight : public HMMWV_Wheel {
public:
  HMMWV_WheelRight(VisualizationType  visType);
  ~HMMWV_WheelRight() {}

  virtual const std::string& getMeshName() const { return m_meshName; }
  virtual const std::string& getMeshFile() const { return m_meshFile; }

  static void ExportMeshPovray(const std::string& out_dir);

private:
  static const std::string  m_meshName;
  static const std::string  m_meshFile;
};


} // end namespace hmmwv


#endif
