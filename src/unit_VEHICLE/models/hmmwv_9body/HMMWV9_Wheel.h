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
               float              mu,
               VisualizationType  visType);

  virtual ~HMMWV9_Wheel() {}

  virtual double getMass() const { return m_mass; }
  virtual const chrono::ChVector<>& getInertia() { return m_inertia; }

  virtual const std::string& getMeshName() const = 0;
  virtual const std::string& getMeshFile() const = 0;

protected:
  virtual void OnInitialize(chrono::ChSharedBodyPtr body);

  bool               m_contact;
  float              m_mu;
  VisualizationType  m_visType;

  static const double  m_radius;
  static const double  m_width;
  static const double  m_mass;
  static const chrono::ChVector<>  m_inertia;
};


class HMMWV9_WheelLeft : public HMMWV9_Wheel {
public:
  HMMWV9_WheelLeft(bool               enableContact,
                   float              mu,
                   VisualizationType  visType);
  ~HMMWV9_WheelLeft() {}

  virtual const std::string& getMeshName() const { return m_meshName; }
  virtual const std::string& getMeshFile() const { return m_meshFile; }

  static void ExportMeshPovray(const std::string& out_dir);

private:
  static const std::string  m_meshName;
  static const std::string  m_meshFile;
};


class HMMWV9_WheelRight : public HMMWV9_Wheel {
public:
  HMMWV9_WheelRight(bool               enableContact,
                    float              mu,
                    VisualizationType  visType);
  ~HMMWV9_WheelRight() {}

  virtual const std::string& getMeshName() const { return m_meshName; }
  virtual const std::string& getMeshFile() const { return m_meshFile; }

  static void ExportMeshPovray(const std::string& out_dir);

private:
  static const std::string  m_meshName;
  static const std::string  m_meshFile;
};


} // end namespace hmmwv9


#endif
