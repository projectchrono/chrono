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
// LuGre tire constructed with data from file (JSON format).
//
// =============================================================================

#ifndef LUGRE_TIRE_H
#define LUGRE_TIRE_H

#include "subsys/ChApiSubsys.h"
#include "subsys/tire/ChLugreTire.h"

#include "rapidjson/document.h"

namespace chrono {


class CH_SUBSYS_API LugreTire : public ChLugreTire
{
public:

  LugreTire(const std::string&       filename,
            const chrono::ChTerrain& terrain);
  LugreTire(const rapidjson::Document& d,
            const chrono::ChTerrain&   terrain);
  ~LugreTire();

  virtual int getNumDiscs() const                { return m_numDiscs; }
  virtual double getRadius() const               { return m_radius; }
  virtual const double* getDiscLocations() const { return m_discLocs; }

  virtual double getNormalStiffness() const      { return m_normalStiffness; }
  virtual double getNormalDamping() const        { return m_normalDamping; }

  virtual void SetLugreParams() {}

private:

  void Create(const rapidjson::Document& d);

  double   m_radius;
  int      m_numDiscs;
  double*  m_discLocs;

  double   m_normalStiffness;
  double   m_normalDamping;
};


} // end namespace chrono


#endif
