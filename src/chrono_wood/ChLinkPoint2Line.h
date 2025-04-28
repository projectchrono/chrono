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

#ifndef CHLINKPOINT2LINE_H
#define CHLINKPOINT2LINE_H

#include <array>

#include "chrono_wood/ChWoodApi.h"
#include "chrono/physics/ChLinkBase.h"
#include "chrono/solver/ChConstraintTwoTuplesContactN.h"

#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

#include "chrono/core/ChQuaternion.h"

using namespace chrono::fea;

namespace chrono {

class ChIndexedNodes;  // forward ref

namespace wood {

/// @addtogroup fea_constraints
/// @{

/// Utility class for using the ChLinkPoint2Line constraint
class ChWoodApi ChLineOfXYZnodes : public ChVariableTupleCarrier_2vars<3, 3> {
  public:
    std::shared_ptr<fea::ChNodeFEAxyz> mnodeB1;
    std::shared_ptr<fea::ChNodeFEAxyz> mnodeB2; 
    //std::shared_ptr<fea::ChNodeFEAxyz> mnodeB3;   

    virtual ChVariables* GetVariables1() override { return &mnodeB1->Variables(); }
    virtual ChVariables* GetVariables2() override { return &mnodeB2->Variables(); }   
    //virtual ChVariables* GetVariables3() override { return &mnodeB3->Variables(); } 
};

/// Class for creating a constraint between a xyz FEA node (point)
/// and a triangular face given by three xyz FEA nodes, with linear
/// shape function (ex. the face of a tetrahedron or a triangular shell)
/// The node can be offset respect to the face.

class ChWoodApi ChLinkPoint2Line : public ChLinkBase {
  private:
    ChVector3d react;

    // used as an interface to the solver.
    ChConstraintTwoTuples<ChNodeFEAxyz, ChLineOfXYZnodes> constraint1;
    ChConstraintTwoTuples<ChNodeFEAxyz, ChLineOfXYZnodes> constraint2;
    ChConstraintTwoTuples<ChNodeFEAxyz, ChLineOfXYZnodes> constraint3;

    std::shared_ptr<ChNodeFEAxyz> mnodeA;
    ChLineOfXYZnodes mLine;

    double s1, s2;
    double d;

  public:
    ChLinkPoint2Line();
    ChLinkPoint2Line(const ChLinkPoint2Line& other);
    ~ChLinkPoint2Line() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkPoint2Line* Clone() const override { return new ChLinkPoint2Line(*this); }
    /// Get the number of scalar variables affected by constraints in this link.
    virtual unsigned int GetNumAffectedCoords() override { return 3 + 3 + 3 + 3; }

    /// Number of scalar constraints.
    virtual unsigned int GetNumConstraintsBilateral() override { return 3; }

    /// Return the link frame, expressed in absolute coordinates.
    ChFrame<> GetFrameNodeAbs() const { return ChFrame<>(mnodeA->GetPos(), QUNIT); }
    
   /// Get the link frame 1, on the connected node, expressed in the absolute frame.
    virtual ChFramed GetFrame1Abs() const override { return GetFrameNodeAbs(); }

    /// Get the link frame 2, on the beam, expressed in the absolute frame.
    virtual ChFramed GetFrame2Abs() const override { return ChFramed(); }  //// TODO


    /// Get reaction force and torque on node, expressed on link frame 1.
    virtual ChWrenchd GetReaction1() const override {
        return {GetFrame1Abs().TransformDirectionParentToLocal(GetReactionOnNode()), VNULL};
    }

    /// Get reaction force and torque on beam, expressed on link frame 2.
    virtual ChWrenchd GetReaction2() const override {
        return {GetFrame2Abs().TransformDirectionParentToLocal(GetReactionOnNode()), VNULL};
        //// TODO: check torque
    }

    //
    // STATE FUNCTIONS
    //

    // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;
    virtual void IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c) override;
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     bool do_clamp,
                                     double recovery_clamp) override;
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

    // Override/implement system functions of ChPhysicsItem
    // (to assemble/manage data for system solver)

    virtual void InjectConstraints(ChSystemDescriptor& mdescriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void LoadConstraintJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

    // Other functions

    
    /// Use this function after object creation, to initialize it, given
    /// the node and the Line to join.
    /// The attachment position is the actual position of the node.
    /// Note, nodes must belong to the same ChSystem.
    virtual int Initialize(std::shared_ptr<ChNodeFEAxyz> anodeA,   ///< xyz node (point) to join
                           std::shared_ptr<ChNodeFEAxyz> anodeB1,  ///< Line: corner n.1
                           std::shared_ptr<ChNodeFEAxyz> anodeB2  ///< Line: corner n.2                           
    );

    /// Set the area coordinates to specify where the point A is connected on Line.
    /// These are 0..1 values, one respect to point B2, the other respect to B3
    /// (the third area coord follows automatically as 1-s2-s3).
    /// The Initialize() function initialize this automatically.
    virtual void SetAreaCoords(const double ms1, const double ms2) {
        s1 = ms1;
        s2 = ms2;
    }

    /// Get the area coordinates, as set respect to B2 and B3
    ///(the third area coord follows automatically as 1-s2-s3).
    virtual void GetAreaCoords(double& ms1, double& ms2) const {
        ms1 = s1;
        ms2 = s2;
    }

    /// Set an optional offset of point A respect to Line, along Line normal.
    /// Note that it is better to avoid large offsets. Better if offset is zero.
    /// The Initialize() function initialize this automatically.
    virtual void SetOffset(const double md) { d = md; }

    /// Get the imposed offset of point A respect to Line
    virtual double GetOffset(double md) const { return d; }

    /// Get the connected xyz node (point)
    std::shared_ptr<fea::ChNodeFEAxyz> GetConstrainedNodeA() const { return this->mnodeA; }

    /// Get the connected Line
    std::array<std::shared_ptr<fea::ChNodeFEAxyz>, 2> GetConstrainedLine() const {
        return std::array<std::shared_ptr<fea::ChNodeFEAxyz>, 2>{
            {this->mLine.mnodeB1, this->mLine.mnodeB2}};
    }

    /// Get the reaction force considered as applied to node A, in abs coords.
    ChVector3d GetReactionOnNode() const { return -react; }

    //
    // UPDATE FUNCTIONS
    //

    /// Update all auxiliary data of the gear transmission at given time
    virtual void Update(double mytime, bool update_assets = true) override;

    //
    // STREAMING
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};

////////////////////////////////////////////////////////////////////////////////////

// The following classes might be removed if ChNodeFEAxyzrot were inherited from ChNodeFEAxys.
// Planned for future

/// Utility class for using the ChLinkPoint2Line constraint
class ChWoodApi ChLineOfXYZROTnodes : public ChVariableTupleCarrier_2vars<6, 6> {
  public:
    std::shared_ptr<fea::ChNodeFEAxyzrot> mnodeB1;
    std::shared_ptr<fea::ChNodeFEAxyzrot> mnodeB2; 
    //std::shared_ptr<fea::ChNodeFEAxyzrot> mnodeB3;   

    virtual ChVariables* GetVariables1() { return &mnodeB1->Variables(); };
    virtual ChVariables* GetVariables2() { return &mnodeB2->Variables(); }; 
    //virtual ChVariables* GetVariables3() { return &mnodeB3->Variables(); };   
};

/// Class for creating a constraint between a xyz FEA node (point)
/// and a triangular face given by three xyzrot FEA nodes, with linear
/// shape function (ex. the face of a tetrahedron or a triangular shell)
/// The node can be offset respect to the face.
class ChWoodApi ChLinkPoint2LineRot : public ChLinkBase {
  public:
    ChLinkPoint2LineRot();
    ChLinkPoint2LineRot(const ChLinkPoint2LineRot& other);
    ~ChLinkPoint2LineRot() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkPoint2LineRot* Clone() const override { return new ChLinkPoint2LineRot(*this); }
    
    /// Get the number of scalar variables affected by constraints in this link.
    virtual unsigned int GetNumAffectedCoords() override { return 6 + 6 + 6; }

    /// Number of scalar constraints.
    virtual unsigned int GetNumConstraintsBilateral() override { return 6; }

    /// Return the link frame, expressed in absolute coordinates.
    ChFrame<> GetFrameNodeAbs() const { return ChFrame<>(mnodeA->GetPos(), QUNIT); }

        /// Use this function after object creation, to initialize it, given
    /// the node and the Line to join.
    /// The attachment position is the actual position of the node.
    /// Note, nodes must belong to the same ChSystem.
    virtual int Initialize(std::shared_ptr<ChNodeFEAxyzrot> anodeA,      ///< xyz node (point) to join
                           std::shared_ptr<ChNodeFEAxyzrot> anodeB1,  ///< Line: corner n.1
                           std::shared_ptr<ChNodeFEAxyzrot> anodeB2  ///< Line: corner n.2                           
    );

    /// Set the area coordinates to specify where the point A is connected on Line.
    /// These are 0..1 values, one respect to point B2, the other respect to B3
    /// (the third area coord follows automatically as 1-s2-s3).
    /// The Initialize() function initialize this automatically.
    virtual void SetAreaCoords(const double ms1, const double ms2) {
        s1 = ms1;
        s2 = ms2;
    }

    /// Get the area coordinates, as set respect to B2 and B3
    ///(the third area coord follows automatically as 1-s2-s3).
    virtual void GetAreaCoords(double& ms1, double& ms2) const {
        ms1 = s1;
        ms2 = s2;
    }

    /// Set an optional offset of point A respect to Line, along Line normal.
    /// Note that it is better to avoid large offsets. Better if offset is zero.
    /// The Initialize() function initialize this automatically.
    virtual void SetOffset(const double md) { d = md; }

    /// Get the imposed offset of point A respect to Line
    virtual double GetOffset(double md) const { return d; }

    /// Get the connected xyz node (point)
    std::shared_ptr<fea::ChNodeFEAxyzrot> GetConstrainedNodeA() const { return this->mnodeA; }

    /// Get the connected Line
    std::array<std::shared_ptr<fea::ChNodeFEAxyzrot>, 2> GetConstrainedLine() const {
        return std::array<std::shared_ptr<fea::ChNodeFEAxyzrot>, 2>{
            {this->mLine.mnodeB1, this->mLine.mnodeB2}};
    }

    /// Get the reaction force considered as applied to node A, in abs coords.
    //ChVector3d GetReactionOnNode() const { return -react; }
    //ChVector3d GetTorqueOnNode() const { return -torque; }  

    /// Get the reaction force considered as applied to the node, in abs coords.
    ChVector3d GetReactionOnNode() const { return -react; }

    /// Get the reaction force considered as applied to the triangle, in abs coords.
    ChVector3d GetReactionOnBeam() const { return -react; }

    /// Update all auxiliary data of the gear transmission at given time.
    virtual void Update(double mytime, bool update_assets = true) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

    // STATE FUNCTIONS
    //

    // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;
    virtual void IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c) override;
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     bool do_clamp,
                                     double recovery_clamp) override;
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

    // Override/implement system functions of ChPhysicsItem
    // (to assemble/manage data for system solver)

    virtual void InjectConstraints(ChSystemDescriptor& descriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void LoadConstraintJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;


    // Other functions

    //virtual ChCoordsys<> GetLinkAbsoluteCoords() override { return ChCoordsys<>(mnodeA->GetPos()); }

    
  private:
    ChVector3d react;
    ChVector3d torque;

    // used as an interface to the solver.
    ChConstraintTwoTuples<ChNodeFEAxyzrot, ChLineOfXYZROTnodes> constraint1;
    ChConstraintTwoTuples<ChNodeFEAxyzrot, ChLineOfXYZROTnodes> constraint2;
    ChConstraintTwoTuples<ChNodeFEAxyzrot, ChLineOfXYZROTnodes> constraint3;
    
    ChConstraintTwoTuples<ChNodeFEAxyzrot, ChLineOfXYZROTnodes> constraint4;
    ChConstraintTwoTuples<ChNodeFEAxyzrot, ChLineOfXYZROTnodes> constraint5;
    ChConstraintTwoTuples<ChNodeFEAxyzrot, ChLineOfXYZROTnodes> constraint6;

    std::shared_ptr<ChNodeFEAxyzrot> mnodeA;
    ChLineOfXYZROTnodes mLine;

    double s1, s2;
    double d;
    
 
    /// Get the link frame 1, on the connected node, expressed in the absolute frame.
    virtual ChFramed GetFrame1Abs() const override { return GetFrameNodeAbs(); }

    /// Get the link frame 2, on the face, expressed in the absolute frame.
    virtual ChFramed GetFrame2Abs() const override { return ChFramed(); }  //// TODO

    /// Get reaction force and torque on node, expressed on link frame 1.
    virtual ChWrenchd GetReaction1() const override {
        return {GetFrame1Abs().TransformDirectionParentToLocal(GetReactionOnNode()), VNULL};
    }

    /// Get reaction force and torque on triangle, expressed on link frame 2.
    virtual ChWrenchd GetReaction2() const override {
        return {GetFrame2Abs().TransformDirectionParentToLocal(GetReactionOnBeam()), VNULL};
        //// TODO: check torque
    }


};

/// @} fea_constraints

}  // end namespace fea
}  // end namespace chrono

#endif
