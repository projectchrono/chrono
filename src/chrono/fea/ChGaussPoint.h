// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Andrea Favali
// =============================================================================

#ifndef CHGAUSSPOINT
#define CHGAUSSPOINT

#include "chrono/core/ChFrame.h"
#include "chrono/core/ChTensors.h"
#include "chrono/solver/ChSystemDescriptor.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_math
/// @{

/// Class for a Gauss point, that has a position (1D-3D) and a weight.
/// It also contains the strain and the stress tensor.
class ChGaussPoint {
  public:
    ChMatrixDynamic<>* MatrB;  ///< Matrix of partial derivatives: to obtain strain & stress
    ChStrainTensor<> Strain;   ///< Strain tensor
    ChStressTensor<> Stress;   ///< Stress tensor

    /// Create a Gauss point with given number, coordinates and weight.
    ChGaussPoint(int number, ChVector3d* coord, double weight);

    ~ChGaussPoint();

    /// Return local coordinates
    ChVector3d GetLocalCoordinates() const { return m_local_coordinates; }
    void SetLocalCoordinates(const ChVector3d& c) { m_local_coordinates = c; }

    /// Return absolute coordinates
    ChVector3d GetCoordinates() const;

    /// Set absolute coordinates
    void SetCoordinates(const ChVector3d& c);

    /// Return integration weight of receiver
    double GetWeight() const { return m_weight; }
    void SetWeight(double w) { m_weight = w; }

    /// Return number of the point
    int GetNumber() const { return m_number; }

  private:
    int m_number;                    ///< number of this point
    ChVector3d m_local_coordinates;  ///< local (natural) coordinates of the point
    ChVector3d* m_coordinates;       ///< absolute point coordinates
    double m_weight;                 ///< integration weight
};

/// @} fea_math

}  //  end namespace fea
}  //  end namespace chrono

#endif
