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
// HMMWV LuGre subsystem
//
// =============================================================================

#include "assets/ChCylinderShape.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"

#include "models/hmmwv/tire/HMMWV_LugreTire.h"

using namespace chrono;

namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;

const double HMMWV_LugreTire::m_radius = 18.15 * in2m;
const double HMMWV_LugreTire::m_discLocs[] = { 0 };/// { -5 * in2m, 0 * in2m, 5 * in2m };

const double HMMWV_LugreTire::m_normalStiffness = 2e6;
const double HMMWV_LugreTire::m_normalDamping = 1e3;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_LugreTire::HMMWV_LugreTire(const ChTerrain& terrain)
: ChLugreTire(terrain)
{
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_LugreTire::Initialize(ChSharedBodyPtr wheel)
{
  // Invoke the base class Initialize function.
  ChLugreTire::Initialize();

  // Add visualization assets.
  double discWidth = 0.04;

  for (int id = 0; id < m_numDiscs; id++) {
    ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
    cyl->GetCylinderGeometry().rad = m_radius;
    cyl->GetCylinderGeometry().p1 = ChVector<>(0, m_discLocs[id] + discWidth / 2, 0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(0, m_discLocs[id] - discWidth / 2, 0);
    wheel->AddAsset(cyl);
  }

  ChSharedPtr<ChTexture> tex(new ChTexture);
  tex->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
  wheel->AddAsset(tex);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_LugreTire::SetLugreParams()
{
  // Longitudinal direction
  m_sigma0[0] = 4000;
  m_sigma1[0] = 1;
  m_sigma2[0] = 0.02;

  m_Fc[0] = 0.9;
  m_Fs[0] = 0.55;

  m_vs[0] = 0.05;

  // Lateral direction
  m_sigma0[1] = 4000;
  m_sigma1[1] = 1;
  m_sigma2[1] = 0.02;

  m_Fc[1] = 0.9;
  m_Fs[1] = 0.55;

  m_vs[1] = 0.05;
}

} // end namespace hmmwv
