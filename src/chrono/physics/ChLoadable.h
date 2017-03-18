//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLOADABLE_H
#define CHLOADABLE_H

#include "chrono/core/ChVectorDynamic.h"
#include "chrono/solver/ChVariables.h"

namespace chrono {

// Forward decl.
class ChState;
class ChStateDelta;

/// Interface for objects that can be subject to loads (forces)
/// Forces can be distributed on UV surfaces, or lines, etc.,so
/// look also the more detailed children classes.

class ChApi ChLoadable {
  public:
    virtual ~ChLoadable() {}

    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() = 0;

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() = 0;

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) = 0;

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) = 0;

    /// Increment all DOFs using a delta. Default is sum, but may override if 
    /// ndof_x is diffenrent than ndof_w, for example with rotation quaternions and angular w vel.
    /// This could be invoked, for example, by the BDF differentiation that computes the jacobians.
    virtual void LoadableStateIncrement(const unsigned int off_x,
                                   ChState& x_new,
                                   const ChState& x,
                                   const unsigned int off_v,
                                   const ChStateDelta& Dv) = 0;


    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual int Get_field_ncoords() = 0;

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() = 0;

    /// Get the offset of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockOffset(int nblock) = 0;

    /// Get the size of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockSize(int nblock) = 0;

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) = 0;
};

/// Interface for objects that can be subject to volume loads,
/// distributed along UVW coordinates of the object.
/// For instance finite elements like 3D bricks, ex.for gravitational loads.

class ChApi ChLoadableUVW : public ChLoadable {
  public:
    virtual ~ChLoadableUVW() {}

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U,V,W coordinates of the volume, each ranging in -1..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in volume
                           const double V,              ///< parametric coordinate in volume
                           const double W,              ///< parametric coordinate in volume
                           ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here, maybe with offset block_offset
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is = n.field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) = 0;

    /// This can be useful for loadable objects that has some density property, so it can be
    /// accessed by ChLoaderVolumeGravity. Return 0 if the element/nodes does not support xyz gravity.
    virtual double GetDensity() = 0;

    /// If true, use quadrature over u,v,w in [0..1] range as tetrahedron volumetric coords, with z=1-u-v-w
    /// otherwise use quadrature over u,v,w in [-1..+1] as box isoparametric coords.
    virtual bool IsTetrahedronIntegrationNeeded() { return false; }
};

/// Interface for objects that can be subject to area loads,
/// distributed along UV coordinates of the object.
/// For instance finite elements like shells, ex.for applied pressure.

class ChApi ChLoadableUV : public ChLoadable {
  public:
    virtual ~ChLoadableUV() {}

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U,V coordinates of the surface, each ranging in -1..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in surface
                           const double V,              ///< parametric coordinate in surface
                           ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is =n. field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) = 0;

    /// Gets the normal to the surface at the parametric coordinate u,v.
    /// Normal must be considered pointing outside in case the surface is a boundary to a volume.
    virtual ChVector<> ComputeNormal(const double U, const double V) = 0;

    /// If true, use quadrature over u,v in [0..1] range as triangle area coords, with w=1-u-v
    /// otherwise use quadrature over u,v in [-1..+1] as rectangular isoparametric coords.
    virtual bool IsTriangleIntegrationNeeded() { return false; }
};

/// Interface for objects that can be subject to line loads,
/// distributed along U coordinate of the object.
/// For instance finite elements like beams.

class ChApi ChLoadableU : public ChLoadable {
  public:
    virtual ~ChLoadableU() {}

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U coordinate of the line, ranging in -1..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in line
                           ChVectorDynamic<>& Qi,       ///< Return result of Q = N'*F  here
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is =n. field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) = 0;
};

}  // end namespace chrono

#endif
