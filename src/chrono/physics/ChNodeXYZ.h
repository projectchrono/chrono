//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File author: A.Tasora

#ifndef CHNODEXYZ_H
#define CHNODEXYZ_H

#include "physics/ChNodeBase.h"
#include "physics/ChLoadable.h"
#include "lcp/ChLcpVariablesBodyOwnMass.h"
#include "lcp/ChLcpVariablesNode.h"

namespace chrono {

/// Class for a single 'point' node, that has 3 DOF degrees of
/// freedom and a mass.

class ChApi ChNodeXYZ : public virtual ChNodeBase,
                        public ChLoadableUVW
{

    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChNodeXYZ, ChNodeBase);

  public:
    ChNodeXYZ();
    virtual ~ChNodeXYZ();

    ChNodeXYZ(const ChNodeXYZ& other);             // Copy constructor
    ChNodeXYZ& operator=(const ChNodeXYZ& other);  // Assignment operator

    //
    // FUNCTIONS
    //

    // Access the xyz 'LCP variables' of the node
    virtual ChLcpVariablesNode& Variables() =0;

    // Position of the node - in absolute csys.
    const ChVector<>& GetPos() const { return pos; }
    // Position of the node - in absolute csys.
    void SetPos(const ChVector<>& mpos) {pos = mpos;}

    // Velocity of the node - in absolute csys.
    const ChVector<>& GetPos_dt() const { return pos_dt; }
    // Velocity of the node - in absolute csys.
    void SetPos_dt(const ChVector<>& mposdt) { pos_dt = mposdt; }

    // Acceleration of the node - in absolute csys.
    const ChVector<>& GetPos_dtdt() const { return pos_dtdt; }
    // Acceleration of the node - in absolute csys.
    void SetPos_dtdt(const ChVector<>& mposdtdt) { pos_dtdt = mposdtdt; }

    // Get mass of the node. To be implemented in children classes
    virtual double GetMass() const = 0;
    // Set mass of the node. To be implemented in children classes
    virtual void SetMass(double mm) = 0;

    /// Get the number of degrees of freedom
    virtual int Get_ndof_x() { return 3; }



    //
    // INTERFACE to ChLoadable 
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() { return 3; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() { return 3; }

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlock_x(int block_offset, ChVectorDynamic<>& mD) {
        mD.PasteVector(this->pos, block_offset, 0);
    }

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlock_w(int block_offset, ChVectorDynamic<>& mD) {
        mD.PasteVector(this->pos_dt, block_offset, 0);
    }

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, etc. Here is 6: xyz displ + xyz rots
    virtual int Get_field_ncoords() { return 3; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() { return 1; }

    /// Get the offset of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockOffset(int nblock) { return this->NodeGetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockSize(int nblock) { return 3; }

    /// Get the pointers to the contained ChLcpVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChLcpVariables*>& mvars) { 
        mvars.push_back(&this->Variables());
    };

    /// Evaluate Q=N'*F , for Q generalized lagrangian load, where N is some type of matrix
    /// evaluated at point P(U,V,W) assumed in absolute coordinates, and 
    /// F is a load assumed in absolute coordinates.
    /// The det[J] is unused.
    virtual void ComputeNF(const double U,              ///< x coordinate of application point in absolute space
                           const double V,              ///< y coordinate of application point in absolute space
                           const double W,              ///< z coordinate of application point in absolute space
                           ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here, maybe with offset block_offset
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is 3, it is Force x,y,z in absolute coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) {
        //ChVector<> abs_pos(U,V,W); not needed, nodes has no torque. Assuming load is applied to node center
        ChVector<> absF=F.ClipVector(0,0);
        Qi.PasteVector(absF,0,0);
        detJ=1; // not needed because not used in quadrature.
    }

    /// This is not needed because not used in quadrature.
    virtual double GetDensity() { return 1; }


    // SERIALIZATION

    virtual void ArchiveOUT(ChArchiveOut& marchive);
    virtual void ArchiveIN(ChArchiveIn& marchive);


    //
    // DATA
    //
    ChVector<> pos;
    ChVector<> pos_dt;
    ChVector<> pos_dtdt;
};

}  // END_OF_NAMESPACE____

#endif
