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

#ifndef CHLOADCONTAINER_H
#define CHLOADCONTAINER_H


#include "physics/ChLoad.h"
#include "physics/ChPhysicsItem.h"


namespace chrono {


/// A container of ChLoad objects. This container can be added to a ChSystem.
/// One usually create one or more ChLoad objects acting on a ChLoadable items (ex. FEA elements),
/// add them to this container, then  the container is added to a ChSystem.


class  ChApi ChLoadContainer : public ChPhysicsItem   {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChLoadContainer, ChPhysicsItem);

private: 
    
    // 
    // DATA
    //
    std::vector< std::shared_ptr<ChLoadBase> > loadlist;

public:
    ChLoadContainer () {}
    virtual ~ChLoadContainer() {}

        /// Add a load to the container list of loads
    void Add(std::shared_ptr<ChLoadBase> newload) {
        //// Radu: I don't think find can be used on a container of shared pointers which does not support the == operator.
        //// Radu: check if this is still true, now that we switched to std::shared_ptr
        ////assert(std::find<std::vector<std::shared_ptr<ChLoadBase> >::iterator>(loadlist.begin(), loadlist.end(), newload) == loadlist.end());
        loadlist.push_back(newload);
    }
        /// Direct access to the load vector, for iterating etc.
    std::vector< std::shared_ptr<ChLoadBase> >& GetLoadList() {return loadlist;}

    virtual void Setup(){
    }

    virtual void Update(double mytime, bool update_assets = true) {
        for (size_t i=0; i<loadlist.size(); ++i)  {
            loadlist[i]->Update();
        }
        // Overloading of base class:
        ChPhysicsItem::Update(mytime, update_assets);
    }


    virtual void IntLoadResidual_F(const unsigned int off,  ///< offset in R residual
                                   ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
                                   const double c           ///< a scaling factor
                                   ){
        for (size_t i=0; i<loadlist.size(); ++i)  { 
            loadlist[i]->LoadIntLoadResidual_F(R,c);
        }
    };

        /// Tell to a system descriptor that there are items of type
    /// ChLcpKblock in this object (for further passing it to a LCP solver)
    /// Basically does nothing, but maybe that inherited classes may specialize this.
    virtual void InjectKRMmatrices(ChLcpSystemDescriptor& mdescriptor){

        for (size_t i=0; i<loadlist.size(); ++i) {
            loadlist[i]->InjectKRMmatrices(mdescriptor);
        }
    };


    /// Adds the current stiffness K and damping R and mass M matrices in encapsulated
    /// ChLcpKblock item(s), if any. The K, R, M matrices are added with scaling
    /// values Kfactor, Rfactor, Mfactor.
    /// NOTE: signs are flipped respect to the ChTimestepper dF/dx terms:  K = -dF/dq, R = -dF/dv
    virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor){

         for (size_t i=0; i<loadlist.size(); ++i) {
             loadlist[i]->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
         }
    };


    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) {
        //***TODO***
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) {
        //***TODO***
    }

};


}  // END_OF_NAMESPACE____

#endif  
