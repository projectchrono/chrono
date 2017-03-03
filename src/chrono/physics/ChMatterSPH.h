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

#ifndef CHMATTERSPH_H
#define CHMATTERSPH_H

#include <cmath>

#include "chrono/collision/ChCCollisionModel.h"
#include "chrono/physics/ChContinuumMaterial.h"
#include "chrono/physics/ChIndexedNodes.h"
#include "chrono/physics/ChNodeXYZ.h"
#include "chrono/solver/ChVariablesNode.h"

namespace chrono {

// Forward references (for parent hierarchy pointer)
class ChSystem;
class ChMatterSPH;

/// Class for a single node in the SPH cluster.
/// Does not define mass, inertia and shape because those are shared among them.

class ChApi ChNodeSPH : public ChNodeXYZ, public ChContactable_1vars<3> {
  public:
    ChNodeSPH();
    ChNodeSPH(const ChNodeSPH& other);
    ~ChNodeSPH();

    ChNodeSPH& operator=(const ChNodeSPH& other);

    //
    // FUNCTIONS
    //

    // Get the kernel radius (max. radius while checking surrounding particles)
    double GetKernelRadius() const { return h_rad; }
    void SetKernelRadius(double mr);

    // Set collision radius (for colliding with bodies, boundaries, etc.)
    double GetCollisionRadius() const { return coll_rad; }
    void SetCollisionRadius(double mr);

    // Set the mass of the node
    void SetMass(double mmass) override { variables.SetNodeMass(mmass); }
    // Get the mass of the node
    double GetMass() const override { return variables.GetNodeMass(); }

    // Access the variables of the node
    virtual ChVariablesNode& Variables() override { return variables; }

    // Get the SPH container
    ChMatterSPH* GetContainer() const { return container; }
    // Set the SPH container
    void SetContainer(ChMatterSPH* mc) { container = mc; }

    //
    // INTERFACE TO ChContactable
    //

    /// Access variables.
    virtual ChVariables* GetVariables1() override { return &Variables(); }

    /// Tell if the object must be considered in collision detection.
    virtual bool IsContactActive() override { return true; }

    /// Get the number of DOFs affected by this object (position part).
    virtual int ContactableGet_ndof_x() override { return 3; }

    /// Get the number of DOFs affected by this object (speed part).
    virtual int ContactableGet_ndof_w() override { return 3; }

    /// Get all the DOFs packed in a single vector (position part)
    virtual void ContactableGetStateBlock_x(ChState& x) override { x.PasteVector(pos, 0, 0); }

    /// Get all the DOFs packed in a single vector (speed part)
    virtual void ContactableGetStateBlock_w(ChStateDelta& w) override { w.PasteVector(pos_dt, 0, 0); }

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
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& abs_point) override { return pos_dt; }

    /// Return the coordinate system for the associated collision model.
    /// ChCollisionModel might call this to get the position of the
    /// contact model (when rigid) and sync it.
    virtual ChCoordsys<> GetCsysForCollisionModel() override { return ChCoordsys<>(pos, QUNIT); }

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

    virtual double GetContactableMass() override { return GetMass(); }

    /// Return the pointer to the surface material.
    virtual std::shared_ptr<ChMaterialSurfaceBase>& GetMaterialSurfaceBase() override;

    /// This is only for backward compatibility
    virtual ChPhysicsItem* GetPhysicsItem() override;

    // SERIALIZATION

    virtual void ArchiveOUT(ChArchiveOut& marchive) override;
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

    //
    // DATA
    //

    ChMatterSPH* container;

    ChVariablesNode variables;

    collision::ChCollisionModel* collision_model;

    ChVector<> UserForce;

    double volume;
    double density;
    double h_rad;
    double coll_rad;
    double pressure;
};

/// Class for SPH fluid material, with basic property of uncompressible fluid.

class ChApi ChContinuumSPH : public fea::ChContinuumMaterial {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChContinuumSPH)

  private:
    double viscosity;
    double surface_tension;
    double pressure_stiffness;

  public:
    /// Create a continuum isothropic elastoplastic material,
    /// where you can define also plastic and elastic max. stress (yeld limits
    /// for transition elastic->blastic and plastic->fracture).
    ChContinuumSPH(double m_refdensity = 1000, double mviscosity = 0.1, double mtension = 0)
        : viscosity(mviscosity),
          surface_tension(mtension),
          pressure_stiffness(100),
          ChContinuumMaterial(m_refdensity) {}
    ChContinuumSPH(const ChContinuumSPH& other);
    virtual ~ChContinuumSPH() {}

    /// Set the viscosity, in [Pa s] units.
    void Set_viscosity(double mvisc) { viscosity = mvisc; }
    /// Get the viscosity.
    double Get_viscosity() const { return viscosity; }

    /// Set the surface tension coefficient.
    void Set_surface_tension(double mten) { surface_tension = mten; }
    /// Get the surface tension coefficient.
    double Get_surface_tension() const { return surface_tension; }

    /// Set the pressure stiffness (should be infinite for water
    /// or other almost-incompressible fluids, but too large
    /// values can cause numerical troubles).
    void Set_pressure_stiffness(double mst) { pressure_stiffness = mst; }
    /// Set the pressure stiffness.
    double Get_pressure_stiffness() const { return pressure_stiffness; }

    // SERIALIZATION

    virtual void ArchiveOUT(ChArchiveOut& marchive) override;
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

/// Class for clusters of point nodes that can simulate a fluid or an elastic/plastic
/// solid with the Smooth Particle Hydrodynamics (SPH) approach, that is with a
/// 'meshless' FEA approach.

class ChApi ChMatterSPH : public ChIndexedNodes {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChMatterSPH)

  private:
    std::vector<std::shared_ptr<ChNodeSPH> > nodes;     ///< the nodes (markers)
    ChContinuumSPH material;                            ///< continuum material properties
    std::shared_ptr<ChMaterialSurfaceBase> matsurface;  ///< data for surface contact and impact
    bool do_collide;                                    ///< flag indicating whether or not nodes collide

  public:
    /// Build a cluster of nodes for SPH and meshless FEM.
    /// By default the cluster will contain 0 particles.
    ChMatterSPH();
    ChMatterSPH(const ChMatterSPH& other);
    ~ChMatterSPH();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChMatterSPH* Clone() const override { return new ChMatterSPH(*this); }

    /// Enable/disable the collision for this cluster of particles.
    /// After setting ON, remember RecomputeCollisionModel()
    /// before anim starts (it is not automatically
    /// recomputed here because of performance issues.)
    void SetCollide(bool mcoll);
    virtual bool GetCollide() const override { return do_collide; }

    /// Get the number of scalar coordinates (variables), if any, in this item
    virtual int GetDOF() override { return 3 * GetNnodes(); }

    //
    // FUNCTIONS
    //

    /// Get the number of nodes
    virtual unsigned int GetNnodes() const override { return (unsigned int)nodes.size(); }

    /// Access the N-th node
    virtual std::shared_ptr<ChNodeBase> GetNode(unsigned int n) override {
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
                                double& T) override;
    virtual void IntStateScatter(const unsigned int off_x,
                                 const ChState& x,
                                 const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const double T) override;
    virtual void IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override;
    virtual void IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override;
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void IntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& w,
                                    const double c) override;
    virtual void IntToDescriptor(const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) override;
    virtual void IntFromDescriptor(const unsigned int off_v,
                                   ChStateDelta& v,
                                   const unsigned int off_L,
                                   ChVectorDynamic<>& L) override;

    //
    // SOLVER INTERFACE
    //

    // Override/implement system functions of ChPhysicsItem
    // (to assemble/manage data for system solver)

    /// Sets the 'fb' part of the encapsulated ChVariablesBody to zero.
    virtual void VariablesFbReset() override;

    /// Adds the current forces applied to body (including gyroscopic torque) in
    /// encapsulated ChVariablesBody, in the 'fb' part: qf+=forces*factor
    virtual void VariablesFbLoadForces(double factor = 1) override;

    /// Initialize the 'qb' part of the ChVariablesBody with the
    /// current value of body speeds. Note: since 'qb' is the unknown, this
    /// function seems unuseful, unless used before VariablesFbIncrementMq()
    virtual void VariablesQbLoadSpeed() override;

    /// Adds M*q (masses multiplied current 'qb') to Fb, ex. if qb is initialized
    /// with v_old using VariablesQbLoadSpeed, this method can be used in
    /// timestepping schemes that do: M*v_new = M*v_old + forces*dt
    virtual void VariablesFbIncrementMq() override;

    /// Fetches the body speed (both linear and angular) from the
    /// 'qb' part of the ChVariablesBody (does not updates the full body&markers state)
    /// and sets it as the current body speed.
    /// If 'step' is not 0, also computes the approximate acceleration of
    /// the body using backward differences, that is  accel=(new_speed-old_speed)/step.
    /// Mostly used after the solver provided the solution in ChVariablesBody .
    virtual void VariablesQbSetSpeed(double step = 0) override;

    /// Increment body position by the 'qb' part of the ChVariablesBody,
    /// multiplied by a 'step' factor.
    ///     pos+=qb*step
    /// If qb is a speed, this behaves like a single step of 1-st order
    /// numerical integration (Eulero integration).
    /// Does not automatically update markers & forces.
    virtual void VariablesQbIncrementPosition(double step) override;

    /// Tell to a system descriptor that there are variables of type
    /// ChVariables in this object (for further passing it to a solver)
    /// Basically does nothing, but maybe that inherited classes may specialize this.
    virtual void InjectVariables(ChSystemDescriptor& mdescriptor) override;

    // Other functions

    /// Set no speed and no accelerations (but does not change the position)
    void SetNoSpeedNoAcceleration() override;

    /// Synchronize coll.models coordinates and bounding boxes to the positions of the particles.
    virtual void SyncCollisionModels() override;
    virtual void AddCollisionModelsToSystem() override;
    virtual void RemoveCollisionModelsFromSystem() override;

    void UpdateParticleCollisionModels();

    /// Access the material
    ChContinuumSPH& GetMaterial() { return material; }

    /// Initialize the fluid as a prismatic region filled with nodes,
    /// initially well ordered as a lattice. This is a helper function
    /// so that you avoid to create all nodes one by one with many calls
    /// to AddNode() .
    void FillBox(
        const ChVector<> size,                ///< x,y,z sizes of the box to fill (better if integer multiples of spacing)
        const double spacing,                 ///< the spacing between two near nodes
        const double initial_density,         ///< density of the material inside the box, for initialization of node's masses
        const ChCoordsys<> cords = CSYSNORM,  ///< position and rotation of the box
        const bool do_centeredcube = true,    ///< if false, array is simply cubic, if true is centered cubes (highest regularity)
        const double kernel_sfactor = 2.2,    ///< the radius of kernel of the particle is 'spacing' multiplied this value
        const double randomness = 0.0         ///< randomness of the initial distribution lattice, 0...1
        );

    //
    // UPDATE FUNCTIONS
    //

    /// Update all auxiliary data of the particles
    virtual void Update(double mytime, bool update_assets = true) override;
    /// Update all auxiliary data of the particles
    virtual void Update(bool update_assets = true) override;

    // SERIALIZATION

    virtual void ArchiveOUT(ChArchiveOut& marchive) override;
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

}  // end namespace chrono

#endif
