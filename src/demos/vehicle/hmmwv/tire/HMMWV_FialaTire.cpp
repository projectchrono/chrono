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
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// HMMWV Fiala subsystem
//
// =============================================================================

#include "hmmwv/tire/HMMWV_FialaTire.h"
#include <cmath>

using namespace chrono;

namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double HMMWV_FialaTire::m_normalDamping = 17513;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_FialaTire::HMMWV_FialaTire(const std::string& name) : ChFialaTire(name) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_FialaTire::SetFialaParams() {
    //Parametes were fit at roughly 1700lbs and 24psi
    //Note, the width is not based on the width of the physical tire.
    // It was soley based on providing a good fit to the aligning torque curve.
    m_unloaded_radius = 0.461264;
    m_width = 0.8235;
    m_rolling_resistance = 0.001; //Assume very small since no other data exists
    m_c_slip = 52300.24;
    m_c_alpha = 6669.41;
    m_u_min = 0.5568;
    m_u_max = 0.9835;
    m_relax_length_x = 0.1244;
    m_relax_length_y = 0.0317;
}

double HMMWV_FialaTire::getNormalStiffnessForce(double depth) const {
  
  //corresponding depths = 0 : 0.00254 : 0.08128
  double normalforcetabel[33] = { 0,
    296.562935405400,
    611.630472750000,
    889.644324000000,
    1278.86371575000,
    1890.49418850000,
    2390.91912075000,
    2946.94682325000,
    3558.57729600000,
    4225.81053900000,
    4893.04378200000,
    5615.87979525000,
    6338.71580850000,
    7005.94905150000,
    7673.18229450000,
    8451.62107800000,
    9230.05986150000,
    10008.4986450000,
    10786.9374285000,
    11593.1775971250,
    12399.4177657500,
    13150.0551641250,
    13900.6925625000,
    14818.1382716250,
    15735.5839807500,
    16458.4199940000,
    17181.2560072500,
    18015.2975610000,
    18849.3391147500,
    19655.5792833750,
    20461.8194520000,
    21295.8610057500,
    22129.9025595000 };

  depth = depth*(depth > 0); //Ensure that depth is positive;

  double position = (33. * (depth / 0.08128));

  //Linear extrapolation if the depth is at or greater than the maximum depth in the table (0.08128)
  if (position >= 32) {
    return(22129.9025595000 + (depth - 0.08128)*3.283628164370066e+05);
  }
  //Linearly interpolate between the table entries
  else {
    double scale = std::ceil(position) - position;
    return(normalforcetabel[int(std::floor(position))] * (1 - scale) + normalforcetabel[int(std::floor(position) + 1)] * scale);
  }
}

}  // end namespace hmmwv
