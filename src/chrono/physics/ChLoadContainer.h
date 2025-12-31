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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHLOADCONTAINER_H
#define CHLOADCONTAINER_H

#include "chrono/physics/ChLoad.h"
#include "chrono/physics/ChPhysicsItem.h"

namespace chrono {

/// A container of ChLoad objects. This container can be added to a ChSystem.
/// One usually create one or more ChLoad objects acting on a ChLoadable items (e.g. FEA elements), add them to this
/// container, then  the container is added to a ChSystem.
class ChApi ChLoadContainer : public ChPhysicsItem {
  public:
    ChLoadContainer() {}
    ChLoadContainer(const ChLoadContainer& other);
    ~ChLoadContainer() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadContainer* Clone() const override { return new ChLoadContainer(*this); }

    /// Add a load to the container list of loads
    void Add(std::shared_ptr<ChLoadBase> newload);

    /// Direct access to the load vector.
    std::vector<std::shared_ptr<ChLoadBase> >& GetLoadList() { return loadlist; }

    /// Return the number of loads in this container.
    size_t GetNumLoads() const { return loadlist.size(); }

    virtual void Setup() override {}

    virtual void Update(double time, bool update_assets) override;

    virtual void IntLoadResidual_F(const unsigned int off,  ///< offset in R residual
                                   ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
                                   const double c           ///< a scaling factor
                                   ) override;

    virtual void IntLoadResidual_Mv(const unsigned int off,      ///< offset in R residual
                                    ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                    const ChVectorDynamic<>& w,  ///< the w vector
                                    const double c               ///< a scaling factor
                                    ) override;
    virtual void IntLoadLumpedMass_Md(const unsigned int off,  ///< offset in Md vector
                                      ChVectorDynamic<>& Md,  ///< result: Md vector, diagonal of the lumped mass matrix
                                      double& err,    ///< result: not touched if lumping does not introduce errors
                                      const double c  ///< a scaling factor
                                      ) override;

    /// Register with the given system descriptor any ChKRMBlock objects associated with this item.
    virtual void InjectKRMMatrices(ChSystemDescriptor& descriptor) override;

    /// Compute and load current stiffnes (K), damping (R), and mass (M) matrices in encapsulated ChKRMBlock objects.
    /// The resulting KRM blocks represent linear combinations of the K, R, and M matrices, with the specified
    /// coefficients Kfactor, Rfactor,and Mfactor, respectively.
    /// Note: signs are flipped from the term dF/dx in the integrator: K = -dF/dq and R = -dF/dv.
    virtual void LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    std::vector<std::shared_ptr<ChLoadBase> > loadlist;
};

CH_CLASS_VERSION(ChLoadContainer, 0)

}  // end namespace chrono

#endif
