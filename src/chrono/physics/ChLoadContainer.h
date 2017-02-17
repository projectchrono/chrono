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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHLOADCONTAINER_H
#define CHLOADCONTAINER_H

#include "chrono/physics/ChLoad.h"
#include "chrono/physics/ChPhysicsItem.h"

namespace chrono {

/// A container of ChLoad objects. This container can be added to a ChSystem.
/// One usually create one or more ChLoad objects acting on a ChLoadable items (ex. FEA elements),
/// add them to this container, then  the container is added to a ChSystem.

class ChApi ChLoadContainer : public ChPhysicsItem {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLoadContainer)

  private:
    std::vector<std::shared_ptr<ChLoadBase> > loadlist;

  public:
    ChLoadContainer() {}
    ChLoadContainer(const ChLoadContainer& other);
    ~ChLoadContainer() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadContainer* Clone() const override { return new ChLoadContainer(*this); }

    /// Add a load to the container list of loads
    void Add(std::shared_ptr<ChLoadBase> newload);

    /// Direct access to the load vector, for iterating etc.
    std::vector<std::shared_ptr<ChLoadBase> >& GetLoadList() { return loadlist; }

    virtual void Setup() override {}

    virtual void Update(double mytime, bool update_assets = true) override;

    virtual void IntLoadResidual_F(const unsigned int off,  ///< offset in R residual
                                   ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
                                   const double c           ///< a scaling factor
                                   ) override;

    /// Tell to a system descriptor that there are items of type
    /// ChKblock in this object (for further passing it to a solver)
    /// Basically does nothing, but maybe that inherited classes may specialize this.
    virtual void InjectKRMmatrices(ChSystemDescriptor& mdescriptor) override;

    /// Adds the current stiffness K and damping R and mass M matrices in encapsulated
    /// ChKblock item(s), if any. The K, R, M matrices are added with scaling
    /// values Kfactor, Rfactor, Mfactor.
    /// NOTE: signs are flipped respect to the ChTimestepper dF/dx terms:  K = -dF/dq, R = -dF/dv
    virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) override;

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChLoadContainer,0)

}  // end namespace chrono

#endif
