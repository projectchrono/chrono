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

#ifndef CHLOAD_H
#define CHLOAD_H



#include "physics/ChLoader.h"
#include "lcp/ChLcpSystemDescriptor.h"


namespace chrono {



/// Base class for loads. 
/// This class can be inherited to implement applied loads to bodies, 
/// finite elements, etc.
/// A load is an object that might wrap one or more ChLoader objects, whose
/// value is dependent.
/// It implements functionalities to perform automatic differentiation of
/// the load so it optionally can compute the jacobian (the tangent stiffness
/// matrix of the load) that can be used in implicit integrators, statics, etc.

class ChLoadBase : public ChShared {
public:
        /// Gets the number of DOFs affected by this load (position part)
    virtual int LoadGet_ndof_x() = 0;
        
        /// Gets the number of DOFs affected by this load (speed part)
    virtual int LoadGet_ndof_w() = 0;

        /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadGetStateBlock_x(ChMatrixDynamic<>& mD) = 0;

        /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadGetStateBlock_w(ChMatrixDynamic<>& mD) = 0;

        /// Number of coordinates in the interpolated field, ex=3 for a 
        /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual int LoadGet_field_ncoords() = 0;

        /// Compute Q, the generalized load(s). Each Q is stored in each wrapped ChLoader.
        /// Called automatically at each Update().
    virtual void ComputeQ() = 0;

        /// Compute the K=-dQ/dx, R=-dQ/dv , M=-dQ/da jacobians, 
        /// multiply them for given factors, sum and store in H.
        /// Called automatically at each Update().
    virtual void ComputeJacobian() {}; //***TODO*** ChMatrix<>& H, double Kfactor, double Rfactor = 0, double Mfactor = 0) = 0;

        /// Update: this is called at least at each time step. 
        /// It recomputes the generalized load Q vector(s) and the jacobian(s) K,R,M.
    virtual void Update(){
        this->ComputeQ();
        this->ComputeJacobian();
    };

    //
    // Functions for interfacing to the state bookkeeping and LCP solver
    //

        /// Adds the internal loads (pasted at global nodes offsets) into
        /// a global vector R, multiplied by a scaling factor c, as
        ///   R += forces * c
    virtual void LoadIntLoadResidual_F(ChVectorDynamic<>& R, const double c) =0;

        /// Tell to a system descriptor that there are item(s) of type
        /// ChLcpKblock in this object (for further passing it to a LCP solver)
        /// Basically does nothing, but inherited classes must specialize this.
    virtual void InjectKRMmatrices(ChLcpSystemDescriptor& mdescriptor)  {} //***TODO***;

        /// Adds the current stiffness K and damping R and mass M matrices in encapsulated
        /// ChLcpKblock item(s), if any. The K, R, M matrices are added with scaling
        /// values Kfactor, Rfactor, Mfactor.
    virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) {} //***TODO***;
};


/// Loads acting on a single ChLoadable item.
/// Create them as ChLoadOne< ChLoaderPressure > my_load(...); for example.

template <class Tloader>
class ChLoad : public ChLoadBase  {
public:
    Tloader loader;

    ChLoad(ChSharedPtr<typename Tloader::type_loadable> mloadable) :
        loader(mloadable)
    {}

    virtual int LoadGet_ndof_x() { return this->loader.GetLoadable()->LoadableGet_ndof_x();}
    virtual int LoadGet_ndof_w() { return this->loader.GetLoadable()->LoadableGet_ndof_w();}
    virtual void LoadGetStateBlock_x(ChMatrixDynamic<>& mD) { this->loader.GetLoadable()->LoadableGetStateBlock_x(0, mD);}
    virtual void LoadGetStateBlock_w(ChMatrixDynamic<>& mD) { this->loader.GetLoadable()->LoadableGetStateBlock_w(0, mD);}
    virtual int LoadGet_field_ncoords() { return this->loader.GetLoadable()->Get_field_ncoords();}

    virtual void ComputeQ() {this->loader.ComputeQ(0,0);};

    virtual void LoadIntLoadResidual_F(ChVectorDynamic<>& R, const double c) { 
        for (int i =0; i< this->loader.GetLoadable()->GetSubBlocks(); ++i) {
            unsigned int moffset = this->loader.GetLoadable()->GetSubBlockOffset(i);
            for (unsigned int row =0; row< this->loader.GetLoadable()->GetSubBlockSize(i); ++row) {
                R(row + moffset) += this->loader.Q(row) * c;
            }
        }
    };

};


}  // END_OF_NAMESPACE____

#endif  
