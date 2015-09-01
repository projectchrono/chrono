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
// HMMWV LuGre tire subsystem
//
// =============================================================================

#ifndef HMMWV_LUGRE_TIRE_H
#define HMMWV_LUGRE_TIRE_H

#include "chrono_vehicle/tire/ChLugreTire.h"

namespace hmmwv {

class HMMWV_LugreTire : public chrono::ChLugreTire {
public:
  HMMWV_LugreTire(const std::string&       name,
                  const chrono::ChTerrain& terrain);
  ~HMMWV_LugreTire() {}

  virtual int getNumDiscs() const                { return m_numDiscs; }
  virtual double getRadius() const               { return m_radius; }
  virtual const double* getDiscLocations() const { return m_discLocs; }

  virtual double getNormalStiffness() const      { return m_normalStiffness; }
  virtual double getNormalDamping() const        { return m_normalDamping; }

  virtual void SetLugreParams();

private:
  static const double  m_radius;
  static const int     m_numDiscs = 3;
  static const double  m_discLocs[m_numDiscs];

  static const double  m_normalStiffness;
  static const double  m_normalDamping;
};


} // end namespace hmmwv


#endif
