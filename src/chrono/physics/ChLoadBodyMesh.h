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
// File authors: Alessandro Tasora

#ifndef CHLOADBODYMESH_H
#define CHLOADBODYMESH_H

#include "chrono/physics/ChLoadsBody.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

namespace chrono {


/// Class for applyings loads to a triangle mesh belonging to a ChBody, as a cluster of forces
/// operating on the underlying rigid body.
/// It is useful for doing cosimulation: one can pass this object's vertex & faces
/// to an external software (ex. CFD) that in turn will perform collision detection 
/// with its entities, compute forces, send forces back to Chrono via this object.
/// Note, this is based on a cluster of  std::vector< std::shared_ptr<ChLoadBodyForce> >, but
/// the class itself could bypass all methods of ChLoadBodyForce and directly implement
/// a more efficient LoadIntLoadResidual_F, however this is left in this way for didactical reasons.

class  ChLoadBodyMesh : public ChLoadBase {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLoadBodyMesh)


  public:
    ChLoadBodyMesh(std::shared_ptr<ChBody> cbody, geometry::ChTriangleMeshConnected& cmesh) {
        contactbody = cbody;
        contactmesh = cmesh;
    };

    virtual ~ChLoadBodyMesh(){};


    // 
    // FUNCTIONS
    //

        // Functions that can be used
        // for cosimulation  A <----> B

        /// A --> B
        /// Get the collision mesh where vertex are given in a vector of xyz points, 
        /// expressed in absolute coordinates, and triangles are given as indexes to the three 
        /// vertexes in that vector. Similarly to Wavefront .OBJ meshes. Note, indexes are 0-based.
        /// These vectors can be later sent to another computing node that computes, say, CFD forces on the mesh.
    void OutputSimpleMesh(std::vector< ChVector<> >& vert_pos,    ///< array of vertexes (absolute xyz positions)
                          std::vector< ChVector<> >& vert_vel,    ///< array of vertexes (absolute xyz velocities, might be useful)
                          std::vector< ChVector<int> >& triangles ///< array of triangles (indexes to vertexes, ccw)
                            ) { 
        vert_pos.resize (contactmesh.m_vertices.size());
        vert_vel.resize (contactmesh.m_vertices.size());
        triangles = contactmesh.m_face_v_indices;
        // Transform the body-relative collsion mesh into the output vectors with positions and speeds in absolute coords
        for(size_t i=0; i< contactmesh.m_vertices.size(); ++i) {
            vert_pos[i] = contactbody->TransformPointLocalToParent( contactmesh.m_vertices[i] );
            vert_vel[i] = contactbody->PointSpeedLocalToParent( contactmesh.m_vertices[i] );
        }
    }

        /// A <-- B
        /// Set the forces to the body, where forces are given as a vector of xyz vectors
        /// (expressed in absolute coordinates) and indexes to the referenced vertex, as
        /// obtained by OutputSimpleMesh.
        /// NOTE! do not insert/remove nodes from the collision mesh 
        ///       between the OutputSimpleMesh-InputSimpleForces pair!
    void InputSimpleForces(const std::vector< ChVector<> > vert_forces,  ///< array of forces (absolute xyz forces in [N])
                           const std::vector< int > vert_ind          ///< array of indexes to vertexes to whom you apply forces
                            ) {
        // check the vert_forces and vert_ind arrays must have same size:
        assert (vert_forces.size() == vert_ind.size());
        // reset the previously applied forces if any:
        this->forces.clear();

        // Populate the array of applied loads to nodes
        for (size_t i = 0; i< vert_forces.size(); ++i) {
            ChVector<> rel_application = contactmesh.m_vertices[vert_ind[i]];

            std::shared_ptr< ChLoadBodyForce > mforce(new ChLoadBodyForce(contactbody, vert_forces[i], false, rel_application, true));
            this->forces.push_back(mforce);
        }
        
    }

        /// Set the contact mesh (also resets the applied nodes)
    void SetContactMesh(geometry::ChTriangleMeshConnected& mmesh) {
        this->contactmesh = mmesh;
        this->forces.clear();
    }
        /// Get the contact mesh
    geometry::ChTriangleMeshConnected& GetContactMesh() { return this->contactmesh;}

        /// Access the list of applied forces, so you can add new ones by using push_back(), 
        /// remove them, count them, etc.
        /// Note that if you add nodes, these should belong to the referenced mesh. 
    std::vector< std::shared_ptr<ChLoadBodyForce> >& GetForceList() { return forces; }


    //
    // ChLoadBase interface
    //

    virtual int LoadGet_ndof_x() { 
        int ndoftot = 0;
        for (int i= 0; i<forces.size(); ++i)
            ndoftot += forces[i]->LoadGet_ndof_x();
        return ndoftot;
    }
    virtual int LoadGet_ndof_w() { 
        int ndoftot = 0;
        for (int i= 0; i<forces.size(); ++i)
            ndoftot += forces[i]->LoadGet_ndof_w();
        return ndoftot;
    }
    virtual void LoadGetStateBlock_x(ChState& mD) { 
        int ndoftot = 0;
        for (int i= 0; i<forces.size(); ++i) {
            ChState mDi(forces[i]->LoadGet_ndof_x(), nullptr);
            forces[i]->LoadGetStateBlock_x(mDi);
            mD.PasteMatrix(mDi,ndoftot,0);
            ndoftot += forces[i]->LoadGet_ndof_x();
        }
    }
    virtual void LoadGetStateBlock_w(ChStateDelta& mD) { 
        int ndoftot = 0;
        for (int i= 0; i<forces.size(); ++i) {
            ChStateDelta mDi(forces[i]->LoadGet_ndof_w(), nullptr);
            forces[i]->LoadGetStateBlock_w(mDi);
            mD.PasteMatrix(mDi,ndoftot,0);
            ndoftot += forces[i]->LoadGet_ndof_w();
        }
    }
    virtual void LoadStateIncrement(const ChState& x, const ChStateDelta& dw, ChState& x_new) override {
        int ndoftotx = 0;
        int ndoftotw = 0;
        for (int i = 0; i < forces.size(); ++i) {
            ChState      mx_inc (forces[i]->LoadGet_ndof_x(), nullptr);
            ChState      mx     (forces[i]->LoadGet_ndof_x(), nullptr);
            ChStateDelta mDi    (forces[i]->LoadGet_ndof_w(), nullptr);
            mx.PasteClippedMatrix (x,  ndoftotx,0, forces[i]->LoadGet_ndof_x(),1,  0,0);
            mDi.PasteClippedMatrix(dw, ndoftotw,0, forces[i]->LoadGet_ndof_w(),1,  0,0);
            forces[i]->LoadStateIncrement(mx, mDi, mx_inc);
            x_new.PasteMatrix(mx_inc,ndoftotx,0);
            ndoftotx += forces[i]->LoadGet_ndof_x();
            ndoftotw += forces[i]->LoadGet_ndof_w();
        }
    }

         // simple.. field is x y z, hardcoded return val:
    virtual int LoadGet_field_ncoords() { return 3;}

        /// Compute Q, the generalized load. 
    virtual void ComputeQ(ChState*      state_x, ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) {
        for (int i= 0; i<forces.size(); ++i) {
            forces[i]->ComputeQ(state_x, state_w);
        }
    };

        /// Compute jacobians. 
        /// Not needed when forces are constant, btw.
    virtual void ComputeJacobian(ChState*      state_x, ///< state position to evaluate jacobians
                                 ChStateDelta* state_w, ///< state speed to evaluate jacobians
                                 ChMatrix<>& mK, ///< result dQ/dx
                                 ChMatrix<>& mR, ///< result dQ/dv
                                 ChMatrix<>& mM) ///< result dQ/da  
     {
        for (int i= 0; i<forces.size(); ++i) {
            forces[i]->ComputeJacobian(state_x, state_w, mK, mR, mM);
        }
     }; 

    virtual bool IsStiff() {return false;}

    virtual void CreateJacobianMatrices() {
        for (int i= 0; i<forces.size(); ++i) {
            forces[i]->CreateJacobianMatrices();
        }
    };

    virtual void LoadIntLoadResidual_F(ChVectorDynamic<>& R, const double c) {
        for (int i= 0; i<forces.size(); ++i) {
            forces[i]->LoadIntLoadResidual_F(R, c);
        }
    };

    virtual void InjectKRMmatrices(ChSystemDescriptor& mdescriptor)  {
        for (int i= 0; i<forces.size(); ++i) {
            forces[i]->InjectKRMmatrices(mdescriptor);
        }
    } 

    virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) {
        for (int i= 0; i<forces.size(); ++i) {
            forces[i]->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
        }
    }

 private:

    // 
    // DATA
    //
    std::shared_ptr<ChBody> contactbody;
    geometry::ChTriangleMeshConnected contactmesh;

    std::vector<std::shared_ptr<ChLoadBodyForce> > forces;
};

}  // end namespace chrono

#endif
