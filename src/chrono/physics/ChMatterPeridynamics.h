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

#ifndef CHMATTERPERIDYNAMICS_H
#define CHMATTERPERIDYNAMICS_H

#include <cmath>

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/physics/ChIndexedNodes.h"
#include "chrono/physics/ChNodeXYZ.h"
#include "chrono/fea/ChContinuumMaterial.h"
#include "chrono/solver/ChVariablesNode.h"

namespace chrono {

// Forward references (for parent hierarchy pointer)
class ChSystem;

namespace fea {

/// @addtogroup chrono_fea
/// @{

// Forward
class ChMatterPeridynamics;

/// Class for a single node in the Peridynamics   cluster
class ChApi ChNodePeridynamics : public ChNodeXYZ, public ChContactable_1vars<3> {
  public:
    ChNodePeridynamics();
    ChNodePeridynamics(const ChNodePeridynamics& other);
    ~ChNodePeridynamics();

    ChNodePeridynamics& operator=(const ChNodePeridynamics& other);

    //
    // FUNCTIONS
    //

    /// Reference (initial) position of the node - in absolute csys.
    /// Note that the simulation algorithm might reset this once in a while, exp. for highly plastic objects.
    ChVector<> GetPosReference() { return pos_ref; }
    /// Reference (initial) position of the node - in absolute csys.
    void SetPosReference(const ChVector<>& mpos) { pos_ref = mpos; }


    /// Get the horizon radius (max. radius while checking surrounding particles).
    double GetHorizonRadius() { return h_rad; }
    
    /// Set the horizon radius (max. radius while checking surrounding particles).
    void SetHorizonRadius(double mr);


    /// Get collision radius (for colliding with bodies, boundaries, etc.).
    double GetCollisionRadius() { return coll_rad; }
    /// Set collision radius (for colliding with bodies, boundaries, etc.)
    void SetCollisionRadius(double mr);

    /// Set the mass of the node.
    void SetMass(double mmass) override { this->variables.SetNodeMass(mmass); }
    /// Get the mass of the node.
    double GetMass() const override { return variables.GetNodeMass(); }

    /// Access the variables of the node.
    virtual ChVariablesNode& Variables() override { return variables; }

    // Get the SPH container
    ChMatterPeridynamics* GetMatterContainer() const { return container; }
    // Set the SPH container
    void SetMatterContainer(ChMatterPeridynamics* mc) { container = mc; }

    //
    // INTERFACE TO ChContactable
    //

	virtual ChContactable::eChContactableType GetContactableType() const override { return CONTACTABLE_3; }

    /// Access variables.
    virtual ChVariables* GetVariables1() override { return &Variables(); }

    /// Tell if the object must be considered in collision detection.
    virtual bool IsContactActive() override { return true; }

    /// Get the number of DOFs affected by this object (position part).
    virtual int ContactableGet_ndof_x() override { return 3; }

    /// Get the number of DOFs affected by this object (speed part).
    virtual int ContactableGet_ndof_w() override { return 3; }

    /// Get all the DOFs packed in a single vector (position part).
    virtual void ContactableGetStateBlock_x(ChState& x) override { x.segment(0,3) = this->pos.eigen(); }

    /// Get all the DOFs packed in a single vector (speed part).
    virtual void ContactableGetStateBlock_w(ChStateDelta& w) override { w.segment(0,3) = this->pos_dt.eigen(); }

    /// Increment the provided state of this object by the given state-delta increment.
    /// Compute: x_new = x + dw.
    virtual void ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) override {
        NodeIntStateIncrement(0, x_new, x, 0, dw);
    }

    /// Express the local point in absolute frame, for the given state position.
    virtual ChVector<> GetContactPoint(const ChVector<>& loc_point, const ChState& state_x) override {
        return state_x.segment(0, 3);
    }

    /// Get the absolute speed of a local point attached to the contactable.
    /// The given point is assumed to be expressed in the local frame of this object.
    /// This function must use the provided states.
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& loc_point,
                                            const ChState& state_x,
                                            const ChStateDelta& state_w) override {
        return state_w.segment(0, 3);
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
    /// The force F and its application point are specified in the absolute reference frame.
    virtual void ContactForceLoadResidual_F(const ChVector<>& F, ///< force 
                                            const ChVector<>& T, ///< torque
                                            const ChVector<>& abs_point,
                                            ChVectorDynamic<>& R) override;

    /// Apply the given force at the given point and load the generalized force array.
    /// The force and its application point are specified in the gloabl frame.
    /// Each object must set the entries in Q corresponding to its variables, starting at the specified offset.
    /// If needed, the object states must be extracted from the provided state position.
    virtual void ContactComputeQ(const ChVector<>& F, ///< force
                                   const ChVector<>& T, ///< torque
                                   const ChVector<>& point,
                                   const ChState& state_x,
                                   ChVectorDynamic<>& Q,
                                   int offset) override {
        Q.segment(offset, 3) = F.eigen();
        Q.segment(offset+3, 3) = VNULL.eigen();
    }

    /// Compute the jacobian(s) part(s) for this contactable item. For example,
    /// if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector<>& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               type_constraint_tuple& jacobian_tuple_N,
                                               type_constraint_tuple& jacobian_tuple_U,
                                               type_constraint_tuple& jacobian_tuple_V,
                                               bool second) override;

    /// Used by some SMC code.
    virtual double GetContactableMass() override { return this->GetMass(); }

    /// This is only for backward compatibility.
    virtual ChPhysicsItem* GetPhysicsItem() override;

    //
    // DATA
    //

    bool is_boundary = false;
    bool is_elastic = false;

    ChMatterPeridynamics* container;

    ChVector<> pos_ref;

    ChMatrix33<> Amoment;
    ChMatrix33<> J;
    ChMatrix33<> FA;

    ChStrainTensor<> t_strain;
    ChStrainTensor<> p_strain;  ///< plastic strain
    ChStrainTensor<> e_strain;  ///< elastic strain
    ChStressTensor<> e_stress;  ///< stress

    ChVariablesNode variables;

    ChVector<> UserForce;

    double volume;
    double density;
    double h_rad;
    double coll_rad;
    double hardening;
};



/// Class for clusters of nodes that can simulate a deformable
/// solid using peridynamics approach.
class ChApi ChMatterPeridynamics : public ChIndexedNodes {

  private:
    std::vector<std::shared_ptr<ChNodePeridynamics> > nodes;  ///< nodes
    std::shared_ptr<ChContinuumElastoplastic> material;   ///< continuum material
    double viscosity;                                     ///< viscosity
    bool do_collide;                                      ///< flag indicating whether or not nodes collide
    std::shared_ptr<ChMaterialSurface> matsurface;        ///< data for surface contact and impact

  public:
    /// Build a cluster of nodes for peridynamics.
    /// By default the cluster will contain 0 particles.
    ChMatterPeridynamics();
    ChMatterPeridynamics(const ChMatterPeridynamics& other);
    ~ChMatterPeridynamics();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChMatterPeridynamics* Clone() const override { return new ChMatterPeridynamics(*this); }

    /// Enable/disable the collision for this cluster of particles.
    /// before anim starts (it is not automatically
    /// recomputed here because of performance issues.)
    void SetCollide(bool mcoll);
    virtual bool GetCollide() const override { return do_collide; }

    /// Get the number of scalar coordinates (variables), if any, in this item.
    virtual int GetDOF() override { return 3 * GetNnodes(); }

    /// Get the number of nodes.
    virtual unsigned int GetNnodes() const override { return (unsigned int)nodes.size(); }

    /// Access the N-th node.
    virtual std::shared_ptr<ChNodeBase> GetNode(unsigned int n) override {
        assert(n < nodes.size());
        return nodes[n];
    }

    /// Resize the node cluster. Also clear the state of
    /// previously created particles, if any.
    void ResizeNnodes(int newsize, double horizon);

    /// Add a new node to the particle cluster, passing a
    /// vector as initial position.
    std::shared_ptr<ChNodePeridynamics> AddNode(ChVector<double> initial_state, double horizon);

    /// Set the material surface for 'boundary contact'.
    void SetMaterialSurface(const std::shared_ptr<ChMaterialSurface>& mnewsurf) { matsurface = mnewsurf; }

    /// Set the material surface for 'boundary contact'.
    std::shared_ptr<ChMaterialSurface>& GetMaterialSurface() { return matsurface; }

    //
    // STATE FUNCTIONS
    //

    // Override/implement interfaces for global state vectors, see ChPhysicsItem for comments.

    virtual void IntStateGather(const unsigned int off_x,
                                ChState& x,
                                const unsigned int off_v,
                                ChStateDelta& v,
                                double& T) override;
    virtual void IntStateScatter(const unsigned int off_x,
                                 const ChState& x,
                                 const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const double T,
                                 bool full_update) override;
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

    // Override/implement system functions of ChPhysicsItem (to assemble/manage data for system solver))

    virtual void VariablesFbReset() override;
    virtual void VariablesFbLoadForces(double factor = 1) override;
    virtual void VariablesQbLoadSpeed() override;
    virtual void VariablesFbIncrementMq() override;
    virtual void VariablesQbSetSpeed(double step = 0) override;
    virtual void VariablesQbIncrementPosition(double step) override;
    virtual void InjectVariables(ChSystemDescriptor& mdescriptor) override;

    // Other functions

    /// Set no speed and no accelerations (but does not change the position).
    void SetNoSpeedNoAcceleration() override;

    /// Synchronize coll.models coordinates and bounding boxes to the positions of the particles.
    virtual void SyncCollisionModels() override;

    /// Add node collision models (if any) to the provided collision system.
    virtual void AddCollisionModelsToSystem(ChCollisionSystem* coll_sys) const override;

    /// Remove the nodes collision models (if any) from the provided collision system.
    virtual void RemoveCollisionModelsFromSystem(ChCollisionSystem* coll_sys) const override;


    //void UpdateParticleCollisionModels();

    /// Access the material.
    std::shared_ptr<ChContinuumElastoplastic>& GetMaterial() { return material; }

    /// Change the default material (by default it is a ChContinuumPlasticVonMises )
    /// with a new one, that you create and handle with smart pointer, so you do not have to worry about deletion.
    void ReplaceMaterial(std::shared_ptr<ChContinuumElastoplastic> newmaterial);

    /// Set the Newtonian viscosity of the material.
    void SetViscosity(double mvisc) { viscosity = mvisc; }
    /// Get the Newtonian viscosity of the material.
    double GetViscosity() const { return viscosity; }

    /// Initialize the material as a prismatic region filled with nodes,
    /// initially well ordered as a lattice. This is a helper function
    /// so that you avoid to create all nodes one by one with many calls
    /// to AddNode() .
    void FillBox(
        const ChVector<> size,         ///< x,y,z sizes of the box to fill (better if integer multiples of spacing)
        const double spacing,          ///< the spacing between two near nodes
        const double initial_density,  ///< density of the material inside the box, for initialization of node's masses
        const ChCoordsys<> cords = CSYSNORM,  ///< position and rotation of the box
        const bool do_centeredcube = false,  ///< if false, array is simply cubic, if true is centered cubes (highest regularity)
        const double horizon_sfactor = 2.2,  ///< the radius of horizon of the particle is 'spacing' multiplied this value
        const double randomness = 0.0       ///< randomness of the initial distribution lattice, 0...1
        );

    /// Setup initial lattice of bonds, running a single shot of collision detection.
    void SetupInitialBonds();

    //
    // UPDATE FUNCTIONS
    //

    /// Update all auxiliary data of the particles.
    virtual void Update(double mytime, bool update_assets = true) override;
    /// Update all auxiliary data of the particles.
    virtual void Update(bool update_assets = true) override;

    //
    // STREAMING
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};


//----------------------------------------------------------------------------------------------


/// Class for a single node in the Peridynamics  cluster
class ChApi ChNodePeri : public ChNodeXYZ, public ChContactable_1vars<3> {
  public:
    ChNodePeri();
    ChNodePeri(const ChNodePeri& other);
    ~ChNodePeri();

    //ChNodePeri& operator=(const ChNodePeri& other);

    //
    // FUNCTIONS
    //

    /// Reference (initial) position of the node - in absolute csys.
    /// Note that the simulation algorithm might reset this once in a while, exp. for highly plastic objects.
    ChVector<> GetPosReference() { return pos_ref; }
    /// Reference (initial) position of the node - in absolute csys.
    void SetPosReference(const ChVector<>& mpos) { pos_ref = mpos; }


    /// Get the horizon radius (max. radius while checking surrounding particles).
    double GetHorizonRadius() { return h_rad; }
    
    /// Set the horizon radius (max. radius while checking surrounding particles).
    void SetHorizonRadius(double mr);


    /// Get collision radius (for colliding with bodies, boundaries, etc.).
    double GetCollisionRadius() { return coll_rad; }
    /// Set collision radius (for colliding with bodies, boundaries, etc.)
    void SetCollisionRadius(double mr);

    /// Set the mass of the node.
    void SetMass(double mmass) override { this->variables.SetNodeMass(mmass); }
    /// Get the mass of the node.
    double GetMass() const override { return variables.GetNodeMass(); }

    /// Access the variables of the node.
    virtual ChVariablesNode& Variables() override { return variables; }


    //
    // INTERFACE TO ChContactable
    //

	virtual ChContactable::eChContactableType GetContactableType() const override { return CONTACTABLE_3; }

    /// Access variables.
    virtual ChVariables* GetVariables1() override { return &Variables(); }

    /// Tell if the object must be considered in collision detection.
    virtual bool IsContactActive() override { return true; }

    /// Get the number of DOFs affected by this object (position part).
    virtual int ContactableGet_ndof_x() override { return 3; }

    /// Get the number of DOFs affected by this object (speed part).
    virtual int ContactableGet_ndof_w() override { return 3; }

    /// Get all the DOFs packed in a single vector (position part).
    virtual void ContactableGetStateBlock_x(ChState& x) override { x.segment(0,3) = this->pos.eigen(); }

    /// Get all the DOFs packed in a single vector (speed part).
    virtual void ContactableGetStateBlock_w(ChStateDelta& w) override { w.segment(0,3) = this->pos_dt.eigen(); }

    /// Increment the provided state of this object by the given state-delta increment.
    /// Compute: x_new = x + dw.
    virtual void ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) override {
        NodeIntStateIncrement(0, x_new, x, 0, dw);
    }

    /// Express the local point in absolute frame, for the given state position.
    virtual ChVector<> GetContactPoint(const ChVector<>& loc_point, const ChState& state_x) override {
        return state_x.segment(0, 3);
    }

    /// Get the absolute speed of a local point attached to the contactable.
    /// The given point is assumed to be expressed in the local frame of this object.
    /// This function must use the provided states.
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& loc_point,
                                            const ChState& state_x,
                                            const ChStateDelta& state_w) override {
        return state_w.segment(0, 3);
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
    /// The force F and its application point are specified in the absolute reference frame.
    virtual void ContactForceLoadResidual_F(const ChVector<>& F, ///< force 
                                            const ChVector<>& T, ///< torque
                                            const ChVector<>& abs_point,
                                            ChVectorDynamic<>& R) override;

    /// Apply the given force at the given point and load the generalized force array.
    /// The force and its application point are specified in the gloabl frame.
    /// Each object must set the entries in Q corresponding to its variables, starting at the specified offset.
    /// If needed, the object states must be extracted from the provided state position.
    virtual void ContactComputeQ(const ChVector<>& F, ///< force
                                   const ChVector<>& T, ///< torque
                                   const ChVector<>& point,
                                   const ChState& state_x,
                                   ChVectorDynamic<>& Q,
                                   int offset) override {
        Q.segment(offset, 3) = F.eigen();
        Q.segment(offset+3, 3) = VNULL.eigen();
    }

    /// Compute the jacobian(s) part(s) for this contactable item. For example,
    /// if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector<>& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               type_constraint_tuple& jacobian_tuple_N,
                                               type_constraint_tuple& jacobian_tuple_U,
                                               type_constraint_tuple& jacobian_tuple_V,
                                               bool second) override;

    /// Used by some SMC code.
    virtual double GetContactableMass() override { return this->GetMass(); }

    /// This is only for backward compatibility. OBSOLETE
    virtual ChPhysicsItem* GetPhysicsItem() override { return nullptr;};

    //
    // DATA
    //
    bool is_requiring_bonds = true;  // requires collision detection to initialize bounds even if elastic
    bool is_boundary = false;        // always requires collsion detection
    bool is_elastic = false;         // if true, once computed, bounds do not require updating via collision detection proximities

    bool is_colliding = false;       // has a collision model that is already inserted in ChSystem collision engine

    bool IsRequiringCollision() {
        return (is_boundary || !is_elastic || is_requiring_bonds);
    }

    ChVector<> pos_ref;

    ChVariablesNode variables;

    ChVector<> UserForce;

    ChVector<> F; // placeholder for storing peridynamic forces, automatically computed

    double volume;
    double h_rad;
    double coll_rad;
};


// Base properties per each peridynamic node. Can be inherited if a material requires more data.
class  ChApi ChMatterDataPerNode {
public:
    std::shared_ptr<ChNodePeri> node;
};

// Base properties per each peridynamic bound. Can be inherited if a material requires more data.
class  ChApi ChMatterDataPerBound { 
public: 
    ChNodePeri* nodeA = nullptr;
    ChNodePeri* nodeB = nullptr;

    void Initialize(ChNodePeri* mA, ChNodePeri* mB) {
        nodeA = mA;
        nodeB = mB;
    };
};

// Custom hash for std::pair 
template<typename T>
void hash_combine(std::size_t &seed, T const &key) {
  std::hash<T> hasher;
  seed ^= hasher(key) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}
template <typename T1, typename T2>
struct std::hash<::std::pair<T1,T2>>
{
    std::size_t operator()(const std::pair<T1,T2>& s) const noexcept
    {
        std::size_t seed(0);
        hash_combine(seed, s.first);
        hash_combine(seed, s.second);
        return seed;
    }
};

/// Base class for assigning material properties (elasticity, viscosity etc) to a cluster
/// of peridynamics nodes.
class ChApi ChMatterPeriBase : public ChPhysicsItem {
public:
    virtual ~ChMatterPeriBase() {
    }
    virtual bool AddProximity(
        ChNodePeri* nodeA,   
        ChNodePeri* nodeB) = 0;  

    /// CONSTITUTIVE MODEL - INTERFACE TO IMPLEMENT: (optionally) 
    /// Base behaviour: resets the ChNodePeri::F vector of each ChNodePeri to zero. 
    virtual void ComputeForcesReset() = 0;

    /// CONSTITUTIVE MODEL - INTERFACE TO IMPLEMENT: 
    /// Add the forces caused by this material to the ChNodePeri::F vector of each ChNodePeri.
    /// Child class can use the containers  this->nodes and this->bounds to compute F, assuming
    /// bounds have been updated with latest collision detection.
    virtual void ComputeForces() = 0;

    /// CONSTITUTIVE MODEL - INTERFACE TO IMPLEMENT: (optionally)  
    /// Changes the state of nodes, from elastic to non elastic, etc., and/or the state of bonds
    /// ex. from not broken into broken, etc. depending on the evolution of the system.
    /// This also adds/removes collision models 
    /// The base behavior is: do nothing. 
    virtual void ComputeStates() = 0;
};

/// Sub-base templated class for assigning material properties (elasticity, viscosity etc) to a cluster
/// of peridynamics nodes.
/// It must be inherited by classes that define specific materials, and those may optionally
/// use per-edge and per-node data structures by inheriting ChMatterDataPerNode and ChMatterDataPerBound
/// and passing them as template parameters.

template <class T_per_node = ChMatterDataPerNode, class T_per_bound = ChMatterDataPerBound>
class  ChMatterPeri : public ChMatterPeriBase {

  protected:
    std::unordered_map<ChNodePeri*, T_per_node > nodes;      ///< nodes
    std::unordered_map<std::pair<ChNodePeri*,ChNodePeri*>, T_per_bound> bounds;    ///< bounds

    std::shared_ptr<ChMaterialSurface> matsurface;        ///< data for surface contact and impact

  public:

    /// Build a cluster of nodes for peridynamics.
    /// By default the cluster will contain 0 particles.
    ChMatterPeri() {
        matsurface = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    }

    ChMatterPeri(const ChMatterPeri& other) {
        matsurface = other.matsurface;
        nodes = other.nodes;
        bounds = other.bounds;
    }

    virtual ~ChMatterPeri() {
    }


    /// Get the number of scalar coordinates (variables), if any, in this item.
    virtual int GetDOF() override { return 3 * GetNnodes(); }

    /// Get the number of nodes.
    unsigned int GetNnodes() const { return (unsigned int)nodes.size(); }
    
    /// Get the number of bounds.
    unsigned int GetNbounds() const { return (unsigned int)bounds.size(); }

    /// Access the node container
    const std::unordered_map<ChNodePeri*, T_per_node>& GetMapOfNodes() {
        return nodes;
    }
    /// Access the bounds container
    std::unordered_map<std::pair<ChNodePeri*,ChNodePeri*>, T_per_bound>& GetMapOfBounds() {
        return bounds;
    }

    /// Add a  node to the particle cluster
    std::shared_ptr<ChNodePeri> AddNode(std::shared_ptr<ChNodePeri> mnode, double horizon) {
        T_per_node node_data;
        node_data.node = mnode;

        mnode->variables.SetUserData((void*)this);
        mnode->h_rad = horizon;

        this->nodes[mnode.get()] = node_data;  // add to container using ptr of node as unique key

        return (mnode);
    }
    /// Create and add a new node to the particle cluster, passing a
    /// vector as initial position.
    std::shared_ptr<ChNodePeri> AddNode(ChVector<double> initial_state, double horizon) {
        // create a node
        auto mnode = chrono_types::make_shared<ChNodePeri>();

        mnode->SetPos(initial_state);
        mnode->SetPosReference(initial_state);
       
        return AddNode(mnode, horizon);
    }

    /// Remove a node from the particle cluster
    bool RemoveNode(std::shared_ptr<ChNodePeri> mnode) {
        this->nodes.erase(mnode.get());
        return true;
    }

    /// Add a proximity: this will be called thousand of times by the broadphase collision detection.
    /// If the two nodes are of proper type, and if bond not yet present, add a T_per_node structure 
    /// to this->bounds and return true, otherwise return false.
    virtual bool AddProximity(
        ChNodePeri* nodeA,   
        ChNodePeri* nodeB    
    ) override {
        ChNodePeri* anodeA = nodeA;
        ChNodePeri* anodeB = nodeB;
        
        // do not add if both nodes are not in this material
        auto foundA = this->nodes.find(anodeA); 
        if (foundA == this->nodes.end())
            return false;
        auto foundB = this->nodes.find(anodeB);
        if (foundB == this->nodes.end())
            return false;
        
        // force A and B to be in increasing order, otherwise two hash keys for same pair
        if (anodeA > anodeB)
            std::swap(anodeA, anodeB);

        auto foundBound = this->bounds.find(std::pair<ChNodePeri*, ChNodePeri*>(anodeA, anodeB));
        if (foundBound != this->bounds.end())
            return false;

        // avoid adding bounds more distant than horizon (use only one of the two horizons of the two particles)
        if ((anodeA->GetPos() - anodeB->GetPos()).Length() > (anodeA->GetHorizonRadius()))
            return false;

        // add bound to container using ptr of two nodes as unique key
        this->bounds[std::pair<ChNodePeri*, ChNodePeri*>(anodeA, anodeB)].Initialize(anodeA, anodeB);
        
        return true;
    }

    
    /// CONSTITUTIVE MODEL - INTERFACE TO IMPLEMENT: (optionally) 
    /// Base behaviour: 
    ///  - resets the ChNodePeri::F vector of each ChNodePeri to zero.
    ///  - adds or remove collision model from the collision engine 
    ///  - resets is_elastic optimization flag (is is_elastic, bounds are persistent and collision engine not needed)
    virtual void ComputeForcesReset() override {
        
        for (auto& nodedata : nodes) {

            // resets force accumulator to zero
            auto& mnode = nodedata.second.node;
            mnode->F = VNULL;

            // add collision model to system if not yet added
            if (!mnode->is_colliding && mnode->IsRequiringCollision()) {
                // create model
                if (!mnode->GetCollisionModel()) {
                    double aabb_rad = mnode->GetHorizonRadius() / 2;  // to avoid too many pairs: bounding boxes hemisizes will sum..  __.__--*--
                    double coll_rad = mnode->GetCollisionRadius();
                    auto cshape = chrono_types::make_shared<ChCollisionShapeSphere>(matsurface, coll_rad);  
                    mnode->AddCollisionShape(cshape);
                    mnode->GetCollisionModel()->SetSafeMargin(coll_rad);
                    mnode->GetCollisionModel()->SetEnvelope(ChMax(0.0, aabb_rad - coll_rad));
                }
                // add to system
                GetSystem()->GetCollisionSystem()->Add(mnode->GetCollisionModel());
                mnode->is_colliding = true; // prevents other material adding twice
            }
            // remove collision model from system if not yet removed
            if (mnode->is_colliding && !mnode->IsRequiringCollision()) {
                GetSystem()->GetCollisionSystem()->Remove(mnode->GetCollisionModel());
                mnode->is_colliding = false; // prevents other material adding twice
            }

            // Initialize flag to mark if the bounds are persistent. Default: not.
            // Materials will flag is_elastic to true in ComputeStates(), if needed.
            nodedata.second.node->is_elastic = false;

            // Deactivate this flag, that is used only to force the generation of bounds once. 
            // If collison model for proximity collision was needed, than it was alredy setup few lines above 
            nodedata.second.node->is_requiring_bonds = false;
        }
    }

    /// CONSTITUTIVE MODEL - INTERFACE TO IMPLEMENT: (optionally)  
    /// Changes the state of nodes, from elastic to non elastic, etc., and/or the state of bonds
    /// ex. from not broken into broken, etc. depending on the evolution of the system.
    /// The base behavior is: set as elastic (bounds are computed at beginning and no need to update them). 
    virtual void ComputeStates() override { 
        for (auto& nodedata : nodes) {
            nodedata.second.node->is_elastic = true;
        }
    };
    

    /// Set the material surface for 'boundary contact'.
    void SetMaterialSurface(const std::shared_ptr<ChMaterialSurface>& mnewsurf) { matsurface = mnewsurf; }

    /// Set the material surface for 'boundary contact'.
    std::shared_ptr<ChMaterialSurface>& GetMaterialSurface() { return matsurface; }

    //
    // STATE FUNCTIONS
    //

    // Override/implement interfaces for global state vectors, see ChPhysicsItem for comments.

    virtual void IntStateGather(const unsigned int off_x,
                                ChState& x,
                                const unsigned int off_v,
                                ChStateDelta& v,
                                double& T) {
        unsigned int j = 0;
        for (auto& nodedata : nodes) {
            x.segment(off_x + 3 * j, 3) = nodedata.second.node->pos.eigen();
            v.segment(off_v + 3 * j, 3) = nodedata.second.node->pos_dt.eigen();
            ++j;
        }
        T = GetChTime();
    }
    virtual void IntStateScatter(const unsigned int off_x,
                                 const ChState& x,
                                 const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const double T,
                                 bool full_update) {
        unsigned int j = 0;
        for (auto& nodedata : nodes) {
            nodedata.second.node->pos = x.segment(off_x + 3 * j, 3);
            nodedata.second.node->pos_dt = v.segment(off_v + 3 * j, 3);
            ++j;
        }
        SetChTime(T);
        Update(T, full_update);
    }
    virtual void IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
        unsigned int j = 0;
        for (auto& nodedata : nodes) {
            a.segment(off_a + 3 * j, 3) = nodedata.second.node->pos_dtdt.eigen();
            ++j;
        }
    }
    virtual void IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
        unsigned int j = 0;
        for (auto& nodedata : nodes) {
            nodedata.second.node->SetPos_dtdt(a.segment(off_a + 3 * j, 3));
            ++j;
        }
    }
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c)  {
        
        // assume nodedata.second.node->F already computed via 
        // ComputeForcesReset() and ComputeForces(), called by ChProximityPeri

        unsigned int j = 0;
        for (auto& nodedata : nodes) {
            // add gravity
            ChVector<> Gforce = GetSystem()->Get_G_acc() * nodedata.second.node->GetMass();
            ChVector<> TotForce = nodedata.second.node->F + nodedata.second.node->UserForce + Gforce;

            R.segment(off + 3 * j, 3) += c * TotForce.eigen();
            ++j;
        }
    }
    virtual void IntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& w,
                                    const double c) {
        unsigned int j = 0;
        for (auto& nodedata : nodes) {
            R(off + 3 * j) += c * nodedata.second.node->GetMass() * w(off + 3 * j);
            R(off + 3 * j + 1) += c * nodedata.second.node->GetMass() * w(off + 3 * j + 1);
            R(off + 3 * j + 2) += c * nodedata.second.node->GetMass() * w(off + 3 * j + 2);
            ++j;
        }
    }
    virtual void IntToDescriptor(const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) {
        unsigned int j = 0;
        for (auto& nodedata : nodes) {
            nodedata.second.node->variables.Get_qb() = v.segment(off_v + 3 * j, 3);
            nodedata.second.node->variables.Get_fb() = R.segment(off_v + 3 * j, 3);
            ++j;
        }
    }
    virtual void IntFromDescriptor(const unsigned int off_v,
                                   ChStateDelta& v,
                                   const unsigned int off_L,
                                   ChVectorDynamic<>& L) {
        unsigned int j = 0;
        for (auto& nodedata : nodes) {
            v.segment(off_v + 3 * j, 3) = nodedata.second.node->variables.Get_qb();
            ++j;
        }
    }

    //
    // SOLVER INTERFACE
    //

    // Override/implement system functions of ChPhysicsItem (to assemble/manage data for system solver))

    virtual void VariablesFbReset() {
        // OBSOLETE
        for (auto& nodedata : nodes) {
            nodedata.second.node->variables.Get_fb().setZero();
        }
    }
    virtual void VariablesFbLoadForces(double factor = 1) {
        
        // assume nodedata.second.node->F already computed via 
        // ComputeForcesReset() and ComputeForces(), called by ChProximityPeri

        for (auto& nodedata : nodes) {
            // add gravity
            ChVector<> Gforce = GetSystem()->Get_G_acc() * nodedata.second.node->GetMass();
            ChVector<> TotForce = nodedata.second.node->F + nodedata.second.node->UserForce + Gforce;

            nodedata.second.node->variables.Get_fb() += factor * TotForce.eigen();
        }
    }
    virtual void VariablesQbLoadSpeed() {
        // OBSOLETE
        for (auto& nodedata : nodes) {
            // set current speed in 'qb', it can be used by the solver when working in incremental mode
            nodedata.second.node->variables.Get_qb() = nodedata.second.node->GetPos_dt().eigen();
        }
    }
    virtual void VariablesFbIncrementMq() {
        // OBSOLETE
        for (auto& nodedata : nodes) {
            nodedata.second.node->variables.Compute_inc_Mb_v(nodedata.second.node->variables.Get_fb(), nodedata.second.node->variables.Get_qb());
        }
    }
    virtual void VariablesQbSetSpeed(double step = 0) {
        // OBSOLETE
        for (auto& nodedata : nodes) {
            ChVector<> old_pos_dt = nodedata.second.node->GetPos_dt();

            // from 'qb' vector, sets body speed, and updates auxiliary data
            nodedata.second.node->SetPos_dt(nodedata.second.node->variables.Get_qb());

            // Compute accel. by BDF (approximate by differentiation);
            if (step) {
                nodedata.second.node->SetPos_dtdt((nodedata.second.node->GetPos_dt() - old_pos_dt) / step);
            }
        }
    }
    virtual void VariablesQbIncrementPosition(double step) {
        // OBSOLETE
        // if (!IsActive())
        //	return;

        // TODO PLASTIC FLOW

        for (auto& nodedata : nodes) {
            // Updates position with incremental action of speed contained in the
            // 'qb' vector:  pos' = pos + dt * speed   , like in an Eulero step.

            ChVector<> newspeed(nodedata.second.node->variables.Get_qb());

            // ADVANCE POSITION: pos' = pos + dt * vel
            nodedata.second.node->SetPos(nodedata.second.node->GetPos() + newspeed * step);
        }
    }
    virtual void InjectVariables(ChSystemDescriptor& mdescriptor) {
        // variables.SetDisabled(!IsActive());
        for (auto& nodedata : nodes) {
            mdescriptor.InsertVariables(&(nodedata.second.node->variables));
        }
    }

    // Other functions

    /// Set no speed and no accelerations (but does not change the position).
    void SetNoSpeedNoAcceleration()  {
        for (auto& nodedata : nodes) {
            nodedata.second.node->SetPos_dt(VNULL);
            nodedata.second.node->SetPos_dtdt(VNULL);
        }
    }

    /// Synchronize coll.models coordinates and bounding boxes to the positions of the particles.
    virtual void SyncCollisionModels() {
        for (auto& nodedata : nodes) {
            if (nodedata.second.node->GetCollisionModel())
                nodedata.second.node->GetCollisionModel()->SyncPosition();
        }
    }

    /// Add node collision models (if any) to the provided collision system.
    virtual void AddCollisionModelsToSystem(ChCollisionSystem* coll_sys) const {
        /*
        for (auto& nodedata : nodes) {
            if (nodedata.second.node->GetCollisionModel())
                coll_sys->Add(nodedata.second.node->GetCollisionModel());
        }
        */
    }

    /// Remove the nodes collision models (if any) from the provided collision system.
    virtual void RemoveCollisionModelsFromSystem(ChCollisionSystem* coll_sys) const {
        /*
        for (auto& nodedata : nodes) {
            if (nodedata.second.node->GetCollisionModel())
                coll_sys->Remove(nodedata.second.node->GetCollisionModel());
        }
        */
    }



    /// Initialize the material as a prismatic region filled with nodes,
    /// initially well ordered as a lattice. This is a helper function
    /// so that you avoid to create all nodes one by one with many calls
    /// to AddNode() .
    void FillBox(
        const ChVector<> size,         ///< x,y,z sizes of the box to fill (better if integer multiples of spacing)
        const double spacing,          ///< the spacing between two near nodes
        const double initial_density,  ///< density of the material inside the box, for initialization of node's masses
        const ChCoordsys<> boxcoords = CSYSNORM,  ///< position and rotation of the box
        const bool do_centeredcube = false,  ///< if false, array is simply cubic, if true is centered cubes (highest regularity)
        const double horizon_sfactor = 2.2,  ///< the radius of horizon of the particle is 'spacing' multiplied this value
        const double randomness = 0.0       ///< randomness of the initial distribution lattice, 0...1
        )  
        {
            int samples_x = (int)(size.x() / spacing);
            int samples_y = (int)(size.y() / spacing);
            int samples_z = (int)(size.z() / spacing);
            int totsamples = 0;

            double mrandomness = randomness;
            if (do_centeredcube)
                mrandomness = randomness * 0.5;

            double horizon = horizon_sfactor * spacing;

            for (int ix = 0; ix < samples_x; ix++)
                for (int iy = 0; iy < samples_y; iy++)
                    for (int iz = 0; iz < samples_z; iz++) {
                        ChVector<> pos(ix * spacing - 0.5 * size.x(), iy * spacing - 0.5 * size.y(), iz * spacing - 0.5 * size.z());
                        pos += ChVector<>(mrandomness * ChRandom() * spacing, mrandomness * ChRandom() * spacing,
                                          mrandomness * ChRandom() * spacing);
                        auto newp = AddNode(boxcoords.TransformLocalToParent(pos), horizon);
                        newp->is_elastic = true;
                        newp->is_requiring_bonds = true;
                        if ((ix == 0) || (ix == samples_x - 1) || (iy == 0) || (iy == samples_y - 1) || (iz == 0) || (iz == samples_z - 1)) {
                            newp->is_boundary = true;
                        }

                        totsamples++;

                        if (do_centeredcube && !((ix == samples_x - 1) || (iy == samples_y - 1) || (iz == samples_z - 1))) {
                            ChVector<> pos2 = pos + 0.5 * ChVector<>(spacing, spacing, spacing);
                            pos2 += ChVector<>(mrandomness * ChRandom() * spacing, mrandomness * ChRandom() * spacing,
                                               mrandomness * ChRandom() * spacing);
                            newp = AddNode(boxcoords.TransformLocalToParent(pos2), horizon);
                            newp->is_elastic = true;
                            newp->is_requiring_bonds = true;
                            totsamples++;
                        }
                    }

            double mtotvol = size.x() * size.y() * size.z();
            double mtotmass = mtotvol * initial_density;
            double nodemass = mtotmass / (double)totsamples;

            for (auto& mnodedata : this->nodes) {
                // downcasting
                std::shared_ptr<ChNodePeri> mnode(mnodedata.second.node);
                assert(mnode);

                // mnode->SetCollisionRadius(spacing * 0.1);
                mnode->SetMass(nodemass);
            }
        }


    //
    // UPDATE FUNCTIONS
    //

    /// Update all auxiliary data.
    virtual void Update(double mytime, bool update_assets = true)  {
        // Inherit time changes of parent class
        ChPhysicsItem::Update(mytime, update_assets);

        // Forces not updated here. The function ComputeForces() will be called 
        // by the Update(...) function of the ChProximityContainerPeri, for all materials.
    }
    /// Update all auxiliary data.
    virtual void Update(bool update_assets = true) {
        ChMatterPeri::Update(GetChTime(), update_assets);
    }

    //
    // STREAMING
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) {
        // version number
        marchive.VersionWrite<ChMatterPeri>();

        // serialize the parent class data too
        ChPhysicsItem::ArchiveOut(marchive);

        // serialize all member data:
        //***TODO
    }

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) {
        // version number
        /*int version =*/ marchive.VersionRead<ChMatterPeri>();

        // deserialize the parent class data too
        ChPhysicsItem::ArchiveIn(marchive);

        // deserialize all member data:
        //***TODO
    }
};


/// The simplest peridynamic material: a bond-based material based on a 
/// network of springs, each with the same stiffness k regardless of length, etc. 
/// Just for didactical purposes - do not use it for serious applications.

class ChApi ChMatterPeriSprings : public ChMatterPeri<> {
public:
    double k = 100;

    ChMatterPeriSprings() {};

    // Implement the function that adds the peridynamic force to each node, as a 
    // summation of all the effects of neighbouring nodes.
    virtual void ComputeForces() {
        // loop on bounds
        for (auto& bound : this->bounds) {
            ChMatterDataPerBound& mbound = bound.second;
            ChVector<> old_dist = mbound.nodeB->GetPosReference() - mbound.nodeA->GetPosReference();
            ChVector<>     dist = mbound.nodeB->GetPos() - mbound.nodeA->GetPos();
            ChVector force_val = (dist.Length() - old_dist.Length()) * this->k;
            mbound.nodeB->F += -dist.GetNormalized() * force_val;
            mbound.nodeA->F += dist.GetNormalized() * force_val;
        }
    };
};




/// @} chrono_fea

}  // end namespace fea
}  // end namespace chrono

#endif
