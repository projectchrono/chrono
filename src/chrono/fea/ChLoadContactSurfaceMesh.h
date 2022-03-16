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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHLOADCONTACTSURFACEMESH_H
#define CHLOADCONTACTSURFACEMESH_H

#include "chrono/physics/ChLoadsXYZnode.h"
#include "chrono/fea/ChContactSurfaceMesh.h"

namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{

/// Class for applying loads to a contact mesh as a cluster of forces
/// operating on the nodes of the underlying finite elements.
/// It is useful for doing cosimulation: one can pass this object's vertex & faces
/// to an external software (ex. CFD) that in turn will perform collision detection
/// with its entities, compute forces, send forces back to Chrono via this object.
/// Note, this is based on a cluster of  std::vector< std::shared_ptr<ChLoadXYZnode> >, but
/// the class itself could bypass all methods of ChLoadXYZnode and directly implement
/// a more efficient LoadIntLoadResidual_F, however this is left in this way for didactical reasons.

class ChApi ChLoadContactSurfaceMesh : public ChLoadBase {
  public:
    ChLoadContactSurfaceMesh(std::shared_ptr<ChContactSurfaceMesh> cmesh);

    virtual ~ChLoadContactSurfaceMesh() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadContactSurfaceMesh* Clone() const override { return new ChLoadContactSurfaceMesh(*this); }

    //
    // FUNCTIONS
    //

    // Functions that can be used
    // for cosimulation  A <----> B

    /// Get the collision mesh in a pointer-less way, where vertices are given in a vector of xyz points, and triangles
    /// are given as indexes to the three vertexes in that vector (similar to Wavefront OBJ meshes) Note, indexes are
    /// 0-based. These vectors can be later sent to another computing node that computes, say, CFD forces on the mesh.
    void OutputSimpleMesh(
        std::vector<ChVector<>>& vert_pos,     ///< array of vertexes (absolute xyz positions)
        std::vector<ChVector<>>& vert_vel,     ///< array of vertexes (absolute xyz velocities, might be useful)
        std::vector<ChVector<int>>& triangles  ///< array of triangles (indexes to vertexes, ccw)
    );

    /// Set the forces to the nodes in a pointer-less way, where forces are given as a vector of xyz vectors and indexes
    /// to the referenced vertex, as obtained by OutputSimpleMesh.
    /// NOTE: do not insert/remove nodes from the collision mesh between the OutputSimpleMesh-InputSimpleForces pair!
    void InputSimpleForces(
        const std::vector<ChVector<>> vert_forces,  ///< array of forces (absolute xyz forces in [N])
        const std::vector<int> vert_ind             ///< array of indexes to vertexes to which forces are applied
    );

    /// Set the contact mesh (also resets the applied nodes)
    void SetContactMesh(std::shared_ptr<ChContactSurfaceMesh> mmesh);

    /// Get the contact mesh
    std::shared_ptr<ChContactSurfaceMesh> GetContactMesh() const { return contactmesh; }

    /// Access the list of applied forces, so you can add new ones by using push_back(),
    /// remove them, count them, etc.
    /// Note that if you add nodes, these should belong to the referenced mesh.
    std::vector<std::shared_ptr<ChLoadXYZnode>>& GetForceList() { return forces; }

    //
    // ChLoadBase interface
    //

    virtual int LoadGet_ndof_x() override;
    virtual int LoadGet_ndof_w() override;
    virtual void LoadGetStateBlock_x(ChState& mD) override;
    virtual void LoadGetStateBlock_w(ChStateDelta& mD) override;
    virtual void LoadStateIncrement(const ChState& x, const ChStateDelta& dw, ChState& x_new) override;

    // simple.. field is x y z, hardcoded return val:
    virtual int LoadGet_field_ncoords() override { return 3; }

    /// Compute Q, the generalized load.
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;

    /// Compute jacobians.
    /// Not needed when forces are constant, btw.
    virtual void ComputeJacobian(ChState* state_x,       ///< state position to evaluate jacobians
                                 ChStateDelta* state_w,  ///< state speed to evaluate jacobians
                                 ChMatrixRef mK,         ///< result dQ/dx
                                 ChMatrixRef mR,         ///< result dQ/dv
                                 ChMatrixRef mM          ///< result dQ/da
                                 ) override;

    virtual bool IsStiff() override { return false; }

    virtual void CreateJacobianMatrices() override;
    virtual void LoadIntLoadResidual_F(ChVectorDynamic<>& R, const double c) override;
    virtual void LoadIntLoadResidual_Mv(ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c) override {}
    virtual void InjectKRMmatrices(ChSystemDescriptor& mdescriptor) override;
    virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) override;

  private:
    std::shared_ptr<ChContactSurfaceMesh> contactmesh;
    std::vector<std::shared_ptr<ChLoadXYZnode>> forces;
};

/// @} chrono_fea

}  // end namespace fea
}  // end namespace chrono

#endif
