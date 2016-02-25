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

#ifndef CHMATTERMESHLESS_H
#define CHMATTERMESHLESS_H

//////////////////////////////////////////////////
//
//   ChMatterMeshless.h
//
//   Class for clusters of nodes that can 
//   simulate a visco-elasto-plastic deformable solid 
//   using the approach in Mueller ("Point based.." paper)
//   that is with a 'meshless' FEA approach.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <math.h>

#include "chrono_fea/ChApiFEA.h"
#include "chrono/physics/ChIndexedNodes.h"
#include "chrono/physics/ChNodeXYZ.h"
#include "chrono/collision/ChCCollisionModel.h"
#include "chrono/lcp/ChLcpVariablesNode.h"
#include "chrono/physics/ChContinuumMaterial.h"

namespace chrono {

// Forward references (for parent hierarchy pointer)
class ChSystem;

namespace fea {

// Forward
class ChMatterMeshless;

/// Class for a single node in the meshless FEA  cluster
class ChApiFea ChNodeMeshless : public ChNodeXYZ, public ChContactable_1vars<3> {
  public:
    ChNodeMeshless();
    ~ChNodeMeshless();

    ChNodeMeshless(const ChNodeMeshless& other);  // Copy constructor
    ChNodeMeshless& operator=(const ChNodeMeshless& other);  // Assignment operator

    //
    // FUNCTIONS
    //

    // Reference (initial) position of the node - in absolute csys.
    // Note that the simulation algorithm might reset this once in a while, exp. for highly plastic objects.
    ChVector<> GetPosReference() { return pos_ref; }
    // Reference (initial) position of the node - in absolute csys.
    void SetPosReference(const ChVector<>& mpos) { pos_ref = mpos; }

    // Get the kernel radius (max. radius while checking surrounding particles)
    double GetKernelRadius() { return h_rad; }
    void SetKernelRadius(double mr);

    // Set collision radius (for colliding with bodies, boundaries, etc.)
	double GetCollisionRadius() {return coll_rad;}
	void SetCollisionRadius(double mr);

    // Set the mass of the node
    void SetMass(double mmass) { this->variables.SetNodeMass(mmass); }
    // Get the mass of the node
    double GetMass() const { return variables.GetNodeMass(); }

    // Access the 'LCP variables' of the node
    ChLcpVariablesNode& Variables() { return variables; }

    // Get the SPH container
    ChMatterMeshless* GetMatterContainer() const { return container; }
    // Set the SPH container
    void SetMatterContainer(ChMatterMeshless* mc) { container = mc; }

    //
    // INTERFACE TO ChContactable
    //

    /// Access variables
    virtual ChLcpVariables* GetVariables1() override { return &Variables(); }

    /// Tell if the object must be considered in collision detection
    virtual bool IsContactActive() override { return true; }

    /// Get the number of DOFs affected by this object (position part).
    virtual int ContactableGet_ndof_x() override { return 3; }

    /// Get the number of DOFs affected by this object (speed part).
    virtual int ContactableGet_ndof_w() override { return 3; }

    /// Get all the DOFs packed in a single vector (position part)
    virtual void ContactableGetStateBlock_x(ChState& x) override { x.PasteVector(this->pos, 0, 0); }

    /// Get all the DOFs packed in a single vector (speed part)
    virtual void ContactableGetStateBlock_w(ChStateDelta& w) override { w.PasteVector(this->pos_dt, 0, 0); }

    /// Increment the provided state of this object by the given state-delta increment.
    /// Compute: x_new = x + dw.
    virtual void ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) override {
        NodeIntStateIncrement(0, x_new, x, 0, dw);
    }

    /// Express the local point in absolute frame, for the given state position.
    virtual ChVector<> GetContactPoint(const ChVector<>& loc_point, const ChState& state_x) override {
        return state_x.ClipVector(0, 0);
    }

    /// Get the absolute speed of a local point attached to the contactable.
    /// The given point is assumed to be expressed in the local frame of this object.
    /// This function must use the provided states.
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& loc_point,
                                            const ChState& state_x,
                                            const ChStateDelta& state_w) override {
        return state_w.ClipVector(0, 0);
    }

    /// Get the absolute speed of point abs_point if attached to the surface.
    /// Easy in this case because there are no roations..
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& abs_point) override { return this->pos_dt; }

    /// Return the coordinate system for the associated collision model.
    /// ChCollisionModel might call this to get the position of the
    /// contact model (when rigid) and sync it.
    virtual ChCoordsys<> GetCsysForCollisionModel() override { return ChCoordsys<>(this->pos, QNULL); }

    /// Apply the force, expressed in absolute reference, applied in pos, to the
    /// coordinates of the variables. Force for example could come from a penalty model.
    virtual void ContactForceLoadResidual_F(const ChVector<>& F,
                                            const ChVector<>& abs_point,
                                            ChVectorDynamic<>& R) override;

    /// Apply the given force at the given point and load the generalized force array.
    /// The force and its application point are specified in the gloabl frame.
    /// Each object must set the entries in Q corresponding to its variables, starting at the specified offset.
    /// If needed, the object states must be extracted from the provided state position.
    virtual void ContactForceLoadQ(const ChVector<>& F,
                                   const ChVector<>& point,
                                   const ChState& state_x,
                                   ChVectorDynamic<>& Q,
                                   int offset) override {
        Q.PasteVector(F, offset, 0);
    }

    /// Compute the jacobian(s) part(s) for this contactable item. For example,
    /// if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector<>& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               type_constraint_tuple& jacobian_tuple_N,
                                               type_constraint_tuple& jacobian_tuple_U,
                                               type_constraint_tuple& jacobian_tuple_V,
                                               bool second) override;

    /// Used by some DEM code
    virtual double GetContactableMass() override { return this->GetMass(); }

    /// Return the pointer to the surface material.
    virtual std::shared_ptr<ChMaterialSurfaceBase>& GetMaterialSurfaceBase() override;

    /// This is only for backward compatibility
    virtual ChPhysicsItem* GetPhysicsItem() override;

    //
    // DATA
    //

    ChMatterMeshless* container;

	ChVector<> pos_ref; 
	
	ChMatrix33<> Amoment;
	ChMatrix33<> J;
	ChMatrix33<> FA;

	ChStrainTensor<> t_strain; //
	ChStrainTensor<> p_strain; // plastic strain
	ChStrainTensor<> e_strain; // elastic strain
	ChStressTensor<> e_stress; // stress

	ChLcpVariablesNode	variables;
	collision::ChCollisionModel*	collision_model;

	ChVector<> UserForce;		

	double volume; 
	double density;
	double h_rad;
	double coll_rad;
	double hardening;
};

/// Class for clusters of nodes that can
/// simulate a visco-elasto-plastic deformable solid
/// using the approach in Mueller ("Point based.." 2004 paper)
/// that is with a 'meshless' FEA approach.
class ChApiFea ChMatterMeshless : public ChIndexedNodes {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChMatterMeshless, ChIndexedNodes);

  private:
    //
    // DATA
    //

    std::vector<std::shared_ptr<ChNodeMeshless> > nodes;  ///< nodes

    // ChContinuumPlasticVonMises material;
    std::shared_ptr<ChContinuumElastoplastic> material; //* ChContinuumDruckerPrager material; //***TEST***

	double viscosity;

    bool do_collide;

    // data for surface contact and impact (can be shared):
    std::shared_ptr<ChMaterialSurfaceBase> matsurface;

  public:
    //
    // CONSTRUCTORS
    //

    /// Build a cluster of nodes for Meshless and meshless FEA.
    /// By default the cluster will contain 0 particles.
    ChMatterMeshless();

    /// Destructor
    ~ChMatterMeshless();

    /// Copy from another ChMatterMeshless.
    void Copy(ChMatterMeshless* source);

    //
    // FLAGS
    //

    /// Enable/disable the collision for this cluster of particles.
    /// After setting ON, remember RecomputeCollisionModel()
    /// before anim starts (it is not automatically
    /// recomputed here because of performance issues.)
    void SetCollide(bool mcoll);
    bool GetCollide() { return do_collide; }

    // STATISTICS  - override these in child classes if needed
    //

    /// Get the number of scalar coordinates (variables), if any, in this item
    virtual int GetDOF() { return 3 * this->GetNnodes(); }

    //
    // FUNCTIONS
    //

    /// Get the number of nodes
    unsigned int GetNnodes() { return (unsigned int)nodes.size(); }

    /// Access the N-th node
    std::shared_ptr<ChNodeBase> GetNode(unsigned int n) {
        assert(n < nodes.size());
        return nodes[n];
    }

    /// Resize the node cluster. Also clear the state of
    /// previously created particles, if any.
    void ResizeNnodes(int newsize);

    /// Add a new node to the particle cluster, passing a
    /// vector as initial position.
    void AddNode(ChVector<double> initial_state);

    /// Set the material surface for 'boundary contact'
    void SetMaterialSurface(const std::shared_ptr<ChMaterialSurfaceBase>& mnewsurf) { matsurface = mnewsurf; }

    /// Set the material surface for 'boundary contact'
    virtual std::shared_ptr<ChMaterialSurfaceBase>& GetMaterialSurfaceBase() { return matsurface; }

    //
    // STATE FUNCTIONS
    //

    // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
    virtual void IntStateGather(const unsigned int off_x,
                                ChState& x,
                                const unsigned int off_v,
                                ChStateDelta& v,
                                double& T);
    virtual void IntStateScatter(const unsigned int off_x,
                                 const ChState& x,
                                 const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const double T);
    virtual void IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a);
    virtual void IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a);
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c);
    virtual void IntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& w,
                                    const double c);
    virtual void IntToLCP(const unsigned int off_v,
                          const ChStateDelta& v,
                          const ChVectorDynamic<>& R,
                          const unsigned int off_L,
                          const ChVectorDynamic<>& L,
                          const ChVectorDynamic<>& Qc);
    virtual void IntFromLCP(const unsigned int off_v, ChStateDelta& v, const unsigned int off_L, ChVectorDynamic<>& L);

    //
    // LCP INTERFACE
    //

    // Override/implement LCP system functions of ChPhysicsItem
    // (to assembly/manage data for LCP system solver)

    void VariablesFbReset();

    void VariablesFbLoadForces(double factor = 1.);

    void VariablesQbLoadSpeed();

    void VariablesFbIncrementMq();

    void VariablesQbSetSpeed(double step = 0.);

    void VariablesQbIncrementPosition(double step);

    virtual void InjectVariables(ChLcpSystemDescriptor& mdescriptor);

    // Other functions

    /// Set no speed and no accelerations (but does not change the position)
    void SetNoSpeedNoAcceleration();

    /// Synchronize coll.models coordinates and bounding boxes to the positions of the particles.
    virtual void SyncCollisionModels();
    virtual void AddCollisionModelsToSystem();
    virtual void RemoveCollisionModelsFromSystem();

    void UpdateParticleCollisionModels();

    /// Access the material
    std::shared_ptr<ChContinuumElastoplastic>& GetMaterial() { return material; }
    /// Change the default material (by default it is a ChContinuumPlasticVonMises )
    /// with a new one, that you create and handle with smart pointer, so you do not have to worry about deletion.
    void ReplaceMaterial(std::shared_ptr<ChContinuumElastoplastic> newmaterial);

    /// Set the Newtonian viscosity of the material
    void SetViscosity(double mvisc) { viscosity = mvisc; }
    /// Get the Newtonian viscosity of the material
    double GetViscosity() { return viscosity; }

    /// Initialize the material as a prismatic region filled with nodes,
    /// initially well ordered as a lattice. This is a helper function
    /// so that you avoid to create all nodes one by one with many calls
    /// to AddNode() .
    void FillBox(
        const ChVector<> size,                ///< x,y,z sizes of the box to fill (better if integer multiples of spacing)
        const double spacing,                 ///< the spacing between two near nodes
        const double initial_density,         ///< density of the material inside the box, for initialization of node's masses
        const ChCoordsys<> cords = CSYSNORM,  ///< position and rotation of the box
        const bool do_centeredcube = false,   ///< if false, array is simply cubic, if true is centered cubes (highest regularity)
        const double kernel_sfactor = 2.2,    ///< the radius of kernel of the particle is 'spacing' multiplied this value
        const double randomness = 0.0         ///< randomness of the initial distribution lattice, 0...1
        );

    //
    // UPDATE FUNCTIONS
    //

    /// Update all auxiliary data of the particles
    virtual void Update(double mytime, bool update_assets = true);
    /// Update all auxiliary data of the particles
    virtual void Update(bool update_assets = true);

    //
    // STREAMING
    //

    /// Method to allow deserializing a persistent binary archive (ex: a file)
    /// into transient data.
    void StreamIN(ChStreamInBinary& mstream);

    /// Method to allow serializing transient data into a persistent
    /// binary archive (ex: a file).
    void StreamOUT(ChStreamOutBinary& mstream);
};

} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif
