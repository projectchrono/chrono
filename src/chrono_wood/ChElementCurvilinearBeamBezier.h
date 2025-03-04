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
// Authors: Erol Lale
// =============================================================================

#ifndef CHELEMENT_CURVILINEAR_BEAM_Bezier_H
#define CHELEMENT_CURVILINEAR_BEAM_Bezier_H

//#define BEAM_VERBOSE
#include "chrono_wood/ChWoodApi.h"
#include "chrono_wood/ChBasisToolsBeziers.h"
#include "chrono_wood/ChBeamSectionCurvedIGA.h"
#include "chrono/fea/ChElementBeam.h"
//#include "chrono/fea/ChElementBeamIGA.h"
#include "chrono/fea/ChBeamSectionCosserat.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"
#include "chrono/geometry/ChBasisToolsBSpline.h"
#include "chrono/geometry/ChBasisToolsNurbs.h"
#include "chrono/core/ChQuadrature.h"

using namespace chrono::fea;
using namespace chrono;

namespace chrono {
namespace wood {
	
	
template <class RealA>
void XdirToDxDyDz(const ChVector3<RealA>& Vxdir,
                  const ChVector3<RealA>& Vsingular,
                  ChVector3<RealA>& Vx,
                  ChVector3<RealA>& Vy,
                  ChVector3<RealA>& Vz) {
    ChVector3<RealA> mVnull(0, 0, 0);
    double zlen;

    if (Vequal(Vxdir, mVnull))
        Vx = ChVector3<RealA>(1, 0, 0);
    else
        Vx = Vnorm(Vxdir);

    Vz = Vcross(Vsingular, Vx);
    zlen = Vlength(Vz);

    // If close to singularity, change reference vector
    if (zlen < 1E-10) {
        ChVector3<> mVsingular;
        if (std::abs(Vsingular.z()) < 0.9)
            mVsingular = ChVector3<RealA>(0, 0, 1);
        if (std::abs(Vsingular.y()) < 0.9)
            mVsingular = ChVector3<RealA>(0, 1, 0);
        if (std::abs(Vsingular.x()) < 0.9)
            mVsingular = ChVector3<RealA>(1, 0, 0);
        Vz = Vcross(mVsingular, Vx);
        zlen = Vlength(Vz);  // now should be nonzero length.
    }

    // normalize Vz
    Vz = Vmul(Vz, 1.0 / zlen);
    // compute Vy
    Vy = Vcross(Vx, Vz);
}

/// @addtogroup wood_elements
/// @{

/// Isogeometric formulation (IGA) of a Cosserat rod, with large displacements, based on the Geometrically Exact Beam
/// Theory. User-defined order n (ex: 1=linear 2=quadratic, 3=cubic), where each element is a span of a b-spline, so
/// each element uses n+1 control points, ie. nodes of chrono::fea::ChNodeFEAxyzrot type. As a thick beam, shear effects
/// are possible, v. Timoshenko theory. Reduced integration to correct shear locking (*note, use order 1 for the moment,
/// this must be improved) Initial curved configuration is supported. The section is defined in a modular way, via a
/// chrono::fea::ChBeamSectionCosserat object that is composed via an elastic model, an inertial model, a damping
/// (optional) model, a plastic (optional) model. Some of the ready-to-use implementation of those models allow a very
/// generic beam where the center of mass, center of shear etc. are arbitrarily offset from the beam centerline, thus
/// allowing the simulation of advanced cases like helicopter blades etc.

class ChWoodApi ChElementCurvilinearBeamBezier : public ChElementBeam, public ChLoadableU, public ChLoadableUVW {
  public:
    ChElementCurvilinearBeamBezier();

    ~ChElementCurvilinearBeamBezier() {}

    // To avoid issues with unique_ptr of ChBeamMaterialInternalData and declspec(dllexport), one must
    // prevent automatic creation of copy constructors - but maybe in future provide custom ones
    ChElementCurvilinearBeamBezier(const ChElementCurvilinearBeamBezier&) = delete;
    ChElementCurvilinearBeamBezier& operator=(const ChElementCurvilinearBeamBezier&) = delete;

    virtual unsigned int GetNumNodes() override { return (unsigned int)nodes.size(); }
    virtual unsigned int GetNumCoordsPosLevel() override { return GetNumNodes() * 6; }
    virtual unsigned int GetNodeNumCoordsPosLevel(unsigned int n) override { return 6; }


    virtual std::shared_ptr<ChNodeFEAbase> GetNode(unsigned int n) override { return nodes[n]; }
    virtual std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& GetNodes() { return nodes; }
	
	// For Bspline based element
    virtual void SetNodesCubic(std::shared_ptr<ChNodeFEAxyzrot> nodeA,
                               std::shared_ptr<ChNodeFEAxyzrot> nodeB,
                               std::shared_ptr<ChNodeFEAxyzrot> nodeC,
                               std::shared_ptr<ChNodeFEAxyzrot> nodeD,
                               double knotA1,
                               double knotA2,
                               double knotB1,
                               double knotB2,
                               double knotB3,
                               double knotB4,
                               double knotB5,
                               double knotB6);
	
	
	// For Bspline based element
    virtual void SetNodesGenericOrder(std::vector<std::shared_ptr<ChNodeFEAxyzrot>> mynodes,
                                      std::vector<double> myknots,
                                      int myorder);
		
    ChMatrix33<> FrenetSerret(std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& nodes, ChVector3d& jacob, 
				double invJacob, ChVectorDynamic<>& RR, ChVectorDynamic<>& dRdxi, ChVectorDynamic<>& ddRddxi, 
				ChVectorDynamic<>& dddRdddxi, double& ba_curvature, double& ba_torsion);
    
    ChMatrix33<> GetLocalSystemOfReference(int ig) { return LocalSysOfReference[ig];}   
    void SetLocalSystemOfReference(int ig, ChMatrix33<> mtnb) { LocalSysOfReference[ig]=mtnb;}
    
    //
    ChMatrixNM<double, 6, 6> GetCurvedBeamSecDmat(int igaus, std::shared_ptr<ChElasticityCosseratSimple>& melasticity);
    
    /// Set the integration points, for shear components and for bending components:
    void SetIntegrationPoints(int npoints_s, int npoints_b);

    //
    // FEM functions
    //

    /// Set the section & material of beam element .
    /// It is a shared property, so it can be shared between other beams.
    void SetSection(std::shared_ptr<CurvedBeamSection> my_material) { section = my_material; }
    /// Get the section & material of the element
    std::shared_ptr<CurvedBeamSection> GetSection() { return section; }

    /// Access the local knot sequence of this element (ex.for diagnostics)
    ChVectorDynamic<>& GetKnotSequence() { return this->knots; }
	
    /// Get the parametric coordinate at the beginning of the span
    double GetU1() { return knots(order); }
    /// Get the parametric coordinate at the end of the span
    double GetU2() { return knots(knots.size() - order - 1); }

    virtual void Update() override ;

     /// Get the plastic data, in a vector with as many elements as Gauss points.
    std::vector<std::unique_ptr<ChBeamMaterialInternalData>>& GetPlasticData() { return plastic_data; }

    /// Get the stress, as cut-force [N], in a vector with as many elements as Gauss points.
    std::vector<ChVector3d>& GetStressN() { return this->stress_n; }

    /// Get the stress, as cut-torque [Nm], in a vector with as many elements as Gauss points.
    std::vector<ChVector3d>& GetStressM() { return this->stress_m; }

    /// Get the strain (total=elastic+plastic), as deformation (x is axial strain), in a vector with as many elements as
    /// Gauss points.
    std::vector<ChVector3d>& GetStrainE() { return this->strain_e; }

    /// Get the strain (total=elastic+plastic), as curvature (x is torsion), in a vector with as many elements as Gauss
    /// points.
    std::vector<ChVector3d>& GetStrainK() { return this->strain_k; }


    /// Fills the D vector with the current field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    virtual void GetStateBlock(ChVectorDynamic<>& mD) override {
        mD.resize((int)this->nodes.size() * 7);

        for (int i = 0; i < nodes.size(); ++i) {
            mD.segment(i * 7 + 0, 3) = nodes[i]->GetPos().eigen();
            mD.segment(i * 7 + 3, 4) = nodes[i]->GetRot().eigen();
        }
    }
	
	void GetStateBlock_NEW(ChVectorDynamic<>& mD);
	///
	/// Compute B-matrix
	///
	ChMatrixDynamic<> ComputeBMatrix(ChVectorDynamic<>& R, ChVector3d& dRdx, ChMatrix33<>& tnb);
	///
	/// Compute stiffness matrix
	///
	void ComputeStiffnessMatrix();
	//
	ChMatrixDynamic<> GetStiffnessMatrix(){ return this->stiffness; }
	///
	///
	///
	virtual void UpdateRotation();
	///
	///
	///
	//
    	virtual void ComputeMmatrixGlobal(ChMatrixRef M) override;
	///
	///
	///
    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    virtual void ComputeKRMmatricesGlobal(ChMatrixRef H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

    /// Computes the internal forces (ex. the actual position of nodes is not in relaxed reference position) and set
    /// values in the Fi vector.
    virtual void ComputeInternalForces(ChVectorDynamic<>& Fi) override;

    void ComputeInternalForces_impl(ChVectorDynamic<>& Fi,                 ///< output vector of internal forces
                                    ChState& state_x,                      ///< state position to evaluate Fi
                                    ChStateDelta& state_w,                 ///< state speed to evaluate Fi
                                    bool used_for_differentiation = false  ///< used during FD Jacobian evaluation?
    );

    /// Compute gravity forces, grouped in the Fg vector, one node after the other
    virtual void ComputeGravityForces(ChVectorDynamic<>& Fg, const ChVector3d& G_acc) override;

    //
    // Beam-specific functions
    //

    /// Gets the xyz displacement of a point on the beam line,
    /// and the rotation RxRyRz of section plane, at abscyssa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Note, 'displ' is the displ.state of 2 nodes, ex. get it as GetStateBlock()
    /// Results are not corotated.
    virtual void EvaluateSectionDisplacement(const double eta, ChVector3d& u_displ, ChVector3d& u_rotaz) override {
        ChVectorDynamic<> N((int)nodes.size());

        /* To be completed: Created to be consistent with base class implementation*/
    }

    /// Gets the absolute xyz position of a point on the beam line, at abscissa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    virtual void EvaluateSectionPoint(const double eta, ChVector3d& point) {
        
        ChVectorDynamic<> N((int)nodes.size());
	
        ChBasisToolsBeziers::BasisEvaluate(this->order, eta, N);  ///< here return  in N
		
        point = VNULL;
        for (int i = 0; i < nodes.size(); ++i) {
            point += N(i) * nodes[i]->GetPos();
        }
    }

    /// Gets the absolute xyz position of a point on the beam line,
    /// and the absolute rotation of section plane, at abscissa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    virtual void EvaluateSectionFrame(const double eta, ChVector3d& point, ChQuaternion<>& rot) override {
        // compute parameter in knot space from eta-1..+1
	
        //double u1 = knots(order);  // extreme of span
        //double u2 = knots(knots.size() - order - 1);
        //double u = u1 + ((eta + 1) / 2.0) * (u2 - u1);
        int nspan = order;

        ChVectorDynamic<> N((int)nodes.size());
	
        ChBasisToolsBeziers::BasisEvaluate(this->order, eta, N);  ///< here return  in N

        point = VNULL;
        for (int i = 0; i < nodes.size(); ++i) {
            point += N(i) * nodes[i]->GetPos();
        }
        rot = QNULL;
        for (int i = 0; i < nodes.size(); ++i) {
            ChQuaternion<> myrot = nodes[i]->GetRot();
            rot.e0() += N(i) * myrot.e0();
            rot.e1() += N(i) * myrot.e1();
            rot.e2() += N(i) * myrot.e2();
            rot.e3() += N(i) * myrot.e3();
        }
        rot.Normalize();
    }
    

    /// Gets the force (traction x, shear y, shear z) and the
    /// torque (torsion on x, bending on y, on bending on z) at a section along
    /// the beam line, at abscissa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    virtual void EvaluateSectionForceTorque(const double eta, ChVector3d& Fforce, ChVector3d& Mtorque) override {
        /* To be completed: Created to be consistent with base class implementation*/
    }

    virtual void EvaluateSectionStrain(const double eta, ChVector3d& StrainV) override {
        /* To be completed: Created to be consistent with base class implementation*/
    }

    //
    // Functions for interfacing to the solver
    //            (***most not needed, thank to bookkeeping in parent class ChElementGeneric)

    virtual void EleDoIntegration() override {
        for (size_t i = 0; i < this->plastic_data.size(); ++i) {
            this->plastic_data_old[i]->Copy(*this->plastic_data[i]);
        }
    }

    //
    // Functions for ChLoadable interface
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual unsigned int GetLoadableNumCoordsPosLevel() override { return (int)nodes.size() * 7; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual unsigned int GetLoadableNumCoordsVelLevel() override { return (int)nodes.size() * 6; }

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlockPosLevel(int block_offset, ChState& mD) override {
        for (int i = 0; i < nodes.size(); ++i) {
            mD.segment(block_offset + i * 7 + 0, 3) = this->nodes[i]->GetPos().eigen();
            mD.segment(block_offset + i * 7 + 3, 4) = this->nodes[i]->GetRot().eigen();
        }
    }

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& mD) override {
        for (int i = 0; i < nodes.size(); ++i) {
            mD.segment(block_offset + i * 6 + 0, 3) = this->nodes[i]->GetPosDt().eigen();
            mD.segment(block_offset + i * 6 + 3, 3) = this->nodes[i]->GetAngVelLocal().eigen();
        }
    }

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x,
                                        ChState& x_new,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& Dv) override {
        for (int i = 0; i < nodes.size(); ++i) {
            nodes[i]->NodeIntStateIncrement(off_x + i * 7, x_new, x, off_v + i * 6, Dv);
        }
    }

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual unsigned int GetNumFieldCoords() override { return 6; }

    /// Get the number of DOFs sub-blocks.
    virtual unsigned int GetNumSubBlocks() override { return (unsigned int)nodes.size(); }

    /// Get the offset of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(unsigned int nblock) override {
        return nodes[nblock]->NodeGetOffsetVelLevel();
    }

    /// Get the size of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(unsigned int nblock) override { return 6; }

    /// Check if the specified sub-block of DOFs is active.
    virtual bool IsSubBlockActive(unsigned int nblock) const override { return !nodes[nblock]->IsFixed(); }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override {
        for (int i = 0; i < nodes.size(); ++i) {
            mvars.push_back(&this->nodes[i]->Variables());
        }
    };


    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U coordinates of the line, ranging in -1..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< eta parametric coordinate in line -1..+1
                           ChVectorDynamic<>& Qi,       ///< Return result of Q = N'*F  here
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is =n. field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) override;

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U,V,W coordinates of the volume, each ranging in -1..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in volume
                           const double V,              ///< parametric coordinate in volume
                           const double W,              ///< parametric coordinate in volume
                           ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here, maybe with offset block_offset
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is = n.field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) override;

    /// This is needed so that it can be accessed by ChLoaderVolumeGravity
    virtual double GetDensity() override {
        return this->section->GetInertia()->GetMassPerUnitLength();
    }  // this->section->GetArea()* this->section->GetDensity();
    
    
    virtual void SetLargeDeflection(bool mtrue) { LargeDeflection=mtrue; }
    virtual bool GetLargeDeflection() { return LargeDeflection; }

    ///  For testing purposes:
    enum class QuadratureType { FULL_OVER, FULL_EXACT, REDUCED, SELECTIVE, CUSTOM1, URI2 };

    ///  For testing purposes:
    static QuadratureType quadrature_type;

    ///  For testing purposes:
    static double Delta;

    /// Set if the element mass matrix is computed in lumped or consistent way
    static bool LumpedMass;

    /// Set if the element forces will include the gyroscopic and centrifugal terms (slower performance, but might be
    /// needed esp. when center of mass is offset)
    static bool add_gyroscopic_terms;
    
    static bool LargeDeflection;
    
    virtual std::vector<double> ComputeSectionModifiers(const double kappa, const int order);
	
	
	void BasisEvaluateDeriv(
                const int p,                    ///< order
                const double u,                 ///< parameter
                const ChVectorDynamic<>& Weights,///< weights
                const ChVectorDynamic<>& Knots, ///< knots
                ChVectorDynamic<>& R,           ///< here return basis functions R evaluated at u, that is: R(u)
                ChVectorDynamic<>& dRdu,        ///< here return basis functions derivatives dR/du evaluated at u
                ChVectorDynamic<>& ddRddu,       ///< here return basis functions derivatives ddR/ddu evaluated at u
				ChVectorDynamic<>& dddRdddu       ///< here return basis functions derivatives dddR/dddu evaluated at u
                );


    
	ChMatrixNM<double, 1, 9> ComputeMacroStressContribution();
	
	void ComputeProjectionMatrix();
	
	ChMatrixNM<double,3,9> GetProjectionMatrix(int ig) const { return mprojection_matrix[ig];; }
    void SetProjectionMatrix( int ig, ChMatrixNM<double,3,9> projection_matrix) { mprojection_matrix[ig]=projection_matrix; }
	
	void ComputeEigenStrain(int ig, std::shared_ptr<ChMatrixNM<double,1,9>> macro_strain);
	
	ChVector3d Get_nonMechanicStrain(int ig) const { return m_nonMechanicStrain[ig]; };
    void Set_nonMechanicStrain(int ig, ChVector3d  nonMechanicStrain) { m_nonMechanicStrain[ig]=nonMechanicStrain; } 
	
  private:
    /// Initial setup. Precompute mass and matrices that do not change during the simulation. In particular, compute the
    /// arc-length parametrization.
    virtual void SetupInitial(ChSystem* system) override;

    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodes;  // also "control points"
    ChVectorDynamic<> knots;
    //ChVectorDynamic<> weights;
	public:
    int order;

    int int_order_s;
    int int_order_b;
    
    ChMatrix33<> tnb; //"local reference system of beam"
	
    ChMatrixDynamic<> stiffness;
	
    std::vector<double> Jacobian_s;
    std::vector<double> Jacobian_b;

    std::vector<ChVector3d> strain_e_0;
    std::vector<ChVector3d> strain_k_0;

    std::vector<ChVector3d> stress_n;
    std::vector<ChVector3d> stress_m;
    std::vector<ChVector3d> strain_e;
    std::vector<ChVector3d> strain_k;

    std::vector<std::unique_ptr<ChBeamMaterialInternalData>> plastic_data_old;
    std::vector<std::unique_ptr<ChBeamMaterialInternalData>> plastic_data;

    std::shared_ptr<CurvedBeamSection> section;    
    
   public:
    std::vector<std::vector<double>> sectionModifiers;  
    std::vector<ChMatrix33<>> LocalSysOfReference;
    std::vector<ChQuaternion<>> Qpoint_abs_rot;
    std::vector<ChQuaternion<>> Qpoint_ref_rot; 
    ChVectorDynamic<> DUn_1;
    ChVectorDynamic<> Un_1;
	std::vector<ChVector3d>  m_nonMechanicStrain;	
	
	std::shared_ptr<ChMatrixNM<double,1,9>> macro_strain;
	std::vector<ChMatrixNM<double,3,9>> mprojection_matrix;

    friend class ChExtruderBeamIGA;
};



/// @} wood_elements

}  // end namespace wood
}  // end namespace chrono

#endif
