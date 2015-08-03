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
// RSD antirollbar model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef ANTIROLLBAR_RSD_H
#define ANTIROLLBAR_RSD_H

#include "chrono_vehicle/ChApiSubsys.h"
#include "chrono_vehicle/antirollbar/ChAntirollBarRSD.h"

#include "thirdparty/rapidjson/document.h"

namespace chrono {


class CH_SUBSYS_API AntirollBarRSD : public ChAntirollBarRSD
{
public:

  AntirollBarRSD(const std::string& filename);
  AntirollBarRSD(const rapidjson::Document& d);
  ~AntirollBarRSD() {}

  virtual double getArmMass() const { return m_arm_mass; }
  virtual ChVector<> getArmInertia() { return m_arm_inertia; }

  virtual double getArmLength() const { return m_arm_length; }
  virtual double getArmWidth() const { return m_arm_width; }
  virtual double getDroplinkHeight() const { return m_link_height; }
  virtual double getArmRadius() const { return m_arm_radius; }

  virtual double getSpringCoefficient() const { return m_spring_coef; }
  virtual double getDampingCoefficient() const { return m_damping_coef; }

private:

  void Create(const rapidjson::Document& d);

  double m_arm_mass;
  ChVector<> m_arm_inertia;

  double m_arm_length;
  double m_arm_width;
  double m_link_height;
  double m_arm_radius;

  double m_spring_coef;
  double m_damping_coef;
};


} // end namespace chrono


#endif
