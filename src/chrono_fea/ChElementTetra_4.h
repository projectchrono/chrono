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
// Authors: Andrea Favali, Alessandro Tasora
// =============================================================================

#ifndef CHELEMENTTETRA4_H
#define CHELEMENTTETRA4_H

#include <cmath>

#include "chrono/physics/ChTensors.h"
#include "chrono_fea/ChElementTetrahedron.h"
#include "chrono_fea/ChNodeFEAxyz.h"
#include "chrono_fea/ChNodeFEAxyzP.h"
#include "chrono_fea/ChContinuumPoisson3D.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Tetahedron FEA element with 4 nodes.
/// This is a classical element with linear displacement, hence with constant stress
/// and constant strain. It can be easily used for 3D FEA problems.
class ChApiFea ChElementTetra_4 : public ChElementTetrahedron, public ChLoadableUVW {
  protected:
    std::vector<std::shared_ptr<ChNodeFEAxyz> > nodes;
    std::shared_ptr<ChContinuumElastic> Material;
    ChMatrixDynamic<> MatrB;            // matrix of shape function's partial derivatives
    ChMatrixDynamic<> StiffnessMatrix;  // undeformed local stiffness matrix

    ChMatrixNM<double, 4, 4> mM;  // for speeding up corotational approach

  public:
    ChElementTetra_4();
    virtual ~ChElementTetra_4();

    virtual int GetNnodes() override { return 4; }
    virtual int GetNdofs() override { return 4 * 3; }
    virtual int GetNodeNdofs(int n) override { return 3; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return nodes[n]; }

    virtual void SetNodes(std::shared_ptr<ChNodeFEAxyz> nodeA,
                          std::shared_ptr<ChNodeFEAxyz> nodeB,
                          std::shared_ptr<ChNodeFEAxyz> nodeC,
                          std::shared_ptr<ChNodeFEAxyz> nodeD) {
        nodes[0] = nodeA;
        nodes[1] = nodeB;
        nodes[2] = nodeC;
        nodes[3] = nodeD;
        std::vector<ChVariables*> mvars;
        mvars.push_back(&nodes[0]->Variables());
        mvars.push_back(&nodes[1]->Variables());
        mvars.push_back(&nodes[2]->Variables());
        mvars.push_back(&nodes[3]->Variables());
        Kmatr.SetVariables(mvars);
    }

    //
    // FEM functions
    //

    /// Fills the N shape function matrix with the
    /// values of shape functions at r,s,t 'volume' coordinates, where
    /// r=1 at 2nd vertex, s=1 at 3rd, t = 1 at 4th. All ranging in [0...1].
    /// The last (u, u=1 at 1st vertex) is computed form the first 3 as 1.0-r-s-t.
    /// NOTE! actually N should be a 3row, 12 column sparse matrix,
    /// as  N = [n1*eye(3) n2*eye(3) n3*eye(3) n4*eye(3)]; ,
    /// but to avoid wasting zero and repeated elements, here
    /// it stores only the n1 n2 n3 n4 values in a 1 row, 4 columns matrix.
    virtual void ShapeFunctions(ChMatrix<>& N, double r, double s, double t) {
        N(0) = 1.0 - r - s - t;
        N(1) = r;
        N(2) = s;
        N(3) = t;
    };

    /// Fills the D vector (displacement) column matrix with the current
    /// field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    /// For corotational elements, field is assumed in local reference!
    virtual void GetStateBlock(ChMatrixDynamic<>& mD) override {
        mD.Reset(this->GetNdofs(), 1);
        mD.PasteVector(A.MatrT_x_Vect(nodes[0]->pos) - nodes[0]->GetX0(), 0, 0);
        mD.PasteVector(A.MatrT_x_Vect(nodes[1]->pos) - nodes[1]->GetX0(), 3, 0);
        mD.PasteVector(A.MatrT_x_Vect(nodes[2]->pos) - nodes[2]->GetX0(), 6, 0);
        mD.PasteVector(A.MatrT_x_Vect(nodes[3]->pos) - nodes[3]->GetX0(), 9, 0);
    }

    double ComputeVolume() {
        ChVector<> B1, C1, D1;
        B1.Sub(nodes[1]->pos, nodes[0]->pos);
        C1.Sub(nodes[2]->pos, nodes[0]->pos);
        D1.Sub(nodes[3]->pos, nodes[0]->pos);
        ChMatrixDynamic<> M(3, 3);
        M.PasteVector(B1, 0, 0);
        M.PasteVector(C1, 0, 1);
        M.PasteVector(D1, 0, 2);
        M.MatrTranspose();
        Volume = std::abs(M.Det() / 6);
        return Volume;
    }

    /// Computes the local STIFFNESS MATRIX of the element:
    /// K = Volume * [B]' * [D] * [B]
    virtual void ComputeStiffnessMatrix() {
        // M = [ X0_0 X0_1 X0_2 X0_3 ] ^-1
        //     [ 1    1    1    1    ]
        mM.PasteVector(nodes[0]->GetX0(), 0, 0);
        mM.PasteVector(nodes[1]->GetX0(), 0, 1);
        mM.PasteVector(nodes[2]->GetX0(), 0, 2);
        mM.PasteVector(nodes[3]->GetX0(), 0, 3);
        mM(3, 0) = 1.0;
        mM(3, 1) = 1.0;
        mM(3, 2) = 1.0;
        mM(3, 3) = 1.0;
        mM.MatrInverse();

        MatrB.Reset(6, 12);
        MatrB(0) = mM(0);
        MatrB(3) = mM(4);
        MatrB(6) = mM(8);
        MatrB(9) = mM(12);
        MatrB(13) = mM(1);
        MatrB(16) = mM(5);
        MatrB(19) = mM(9);
        MatrB(22) = mM(13);
        MatrB(26) = mM(2);
        MatrB(29) = mM(6);
        MatrB(32) = mM(10);
        MatrB(35) = mM(14);
        MatrB(36) = mM(1);
        MatrB(37) = mM(0);
        MatrB(39) = mM(5);
        MatrB(40) = mM(4);
        MatrB(42) = mM(9);
        MatrB(43) = mM(8);
        MatrB(45) = mM(13);
        MatrB(46) = mM(12);
        MatrB(49) = mM(2);
        MatrB(50) = mM(1);
        MatrB(52) = mM(6);
        MatrB(53) = mM(5);
        MatrB(55) = mM(10);
        MatrB(56) = mM(9);
        MatrB(58) = mM(14);
        MatrB(59) = mM(13);
        MatrB(60) = mM(2);
        MatrB(62) = mM(0);
        MatrB(63) = mM(6);
        MatrB(65) = mM(4);
        MatrB(66) = mM(10);
        MatrB(68) = mM(8);
        MatrB(69) = mM(14);
        MatrB(71) = mM(12);

        ChMatrixNM<double, 6, 12> EB;
        EB.MatrMultiply(this->Material->Get_StressStrainMatrix(), MatrB);

        StiffnessMatrix.MatrTMultiply(MatrB, EB);

        StiffnessMatrix.MatrScale(Volume);

        // ***TEST*** SYMMETRIZE TO AVOID ROUNDOFF ASYMMETRY
        for (int row = 0; row < StiffnessMatrix.GetRows() - 1; ++row)
            for (int col = row + 1; col < StiffnessMatrix.GetColumns(); ++col)
                StiffnessMatrix(row, col) = StiffnessMatrix(col, row);

        double max_err = 0;
        int err_r = -1;
        int err_c = -1;
        for (int row = 0; row < StiffnessMatrix.GetRows(); ++row)
            for (int col = 0; col < StiffnessMatrix.GetColumns(); ++col) {
                double diff = fabs(StiffnessMatrix.GetElement(row, col) - StiffnessMatrix.GetElement(col, row));
                if (diff > max_err) {
                    max_err = diff;
                    err_r = row;
                    err_c = col;
                }
            }
        if (max_err > 1e-10)
            GetLog() << "NONSYMMETRIC local stiffness matrix! err " << max_err << " at " << err_r << "," << err_c
                     << "\n";
    }

    /// set up the element's parameters and matrices
    virtual void SetupInitial(ChSystem* system) override {
        ComputeVolume();
        ComputeStiffnessMatrix();
    }

    /// compute large rotation of element for corotational approach
    virtual void UpdateRotation() override {
        // P = [ p_0  p_1  p_2  p_3 ]
        //     [ 1    1    1    1   ]
        ChMatrixNM<double, 4, 4> P;
        P.PasteVector(nodes[0]->pos, 0, 0);
        P.PasteVector(nodes[1]->pos, 0, 1);
        P.PasteVector(nodes[2]->pos, 0, 2);
        P.PasteVector(nodes[3]->pos, 0, 3);
        P(3, 0) = 1.0;
        P(3, 1) = 1.0;
        P(3, 2) = 1.0;
        P(3, 3) = 1.0;

        ChMatrix33<double> F;
        // F=P*mM (only upper-left 3x3 block!)
        double sum;
        for (int colres = 0; colres < 3; ++colres)
            for (int row = 0; row < 3; ++row) {
                sum = 0;
                for (int col = 0; col < 4; ++col)
                    sum += (P(row, col)) * (mM(col, colres));
                F(row, colres) = sum;
            }
        ChMatrix33<> S;
        double det = ChPolarDecomposition<>::Compute(F, this->A, S, 1E-6);
        if (det < 0)
            this->A.MatrScale(-1.0);

        // GetLog() << "FEM rotation: \n" << A << "\n" ;
    }

    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    virtual void ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor = 0, double Mfactor = 0) override {
        assert((H.GetRows() == 12) && (H.GetColumns() == 12));

        // warp the local stiffness matrix K in order to obtain global
        // tangent stiffness CKCt:
        ChMatrixDynamic<> CK(12, 12);
        ChMatrixDynamic<> CKCt(12, 12);  // the global, corotated, K matrix
        ChMatrixCorotation<>::ComputeCK(StiffnessMatrix, this->A, 4, CK);
        ChMatrixCorotation<>::ComputeKCt(CK, this->A, 4, CKCt);
        /*
        // ***TEST***
        ChMatrixDynamic<> testCKCt(12,12);
        ChMatrixDynamic<> mC(12,12);
        mC.PasteMatrix(this->A,0,0);
        mC.PasteMatrix(this->A,3,3);
        mC.PasteMatrix(this->A,6,6);
        mC.PasteMatrix(this->A,9,9);
        CK.MatrMultiply(mC,StiffnessMatrix);
        testCKCt.MatrMultiplyT(CK,mC);
        ChMatrixDynamic<> mdiff = testCKCt - CKCt;
        double maxerr=0;
        for (int row = 0; row < mdiff.GetRows()-1; ++row)
            for (int col = 0; col < mdiff.GetColumns(); ++col)
                if (fabs(mdiff(col,row)) > maxerr ) maxerr =  fabs(mdiff(col,row));
        if (maxerr > 0)
            GetLog() << " !!!corotation symmetry error!!!! "  << maxerr << "\n";
        */

        // ***TEST*** SYMMETRIZE TO AVOID ROUNDOFF ASYMMETRY
        for (int row = 0; row < CKCt.GetRows() - 1; ++row)
            for (int col = row + 1; col < CKCt.GetColumns(); ++col)
                CKCt(row, col) = CKCt(col, row);

        //***DEBUG***
        double max_err = 0;
        int err_r = -1;
        int err_c = -1;
        for (int row = 0; row < StiffnessMatrix.GetRows(); ++row)
            for (int col = 0; col < StiffnessMatrix.GetColumns(); ++col) {
                double diff = fabs(StiffnessMatrix.GetElement(row, col) - StiffnessMatrix.GetElement(col, row));
                if (diff > max_err) {
                    max_err = diff;
                    err_r = row;
                    err_c = col;
                }
            }
        if (max_err > 1e-10)
            GetLog() << "NONSYMMETRIC local stiffness matrix! err " << max_err << " at " << err_r << "," << err_c
                     << "\n";
        max_err = 0;
        err_r = -1;
        err_c = -1;
        double maxval = 0;
        for (int row = 0; row < CKCt.GetRows(); ++row)
            for (int col = 0; col < CKCt.GetColumns(); ++col) {
                double diff = fabs(CKCt.GetElement(row, col) - CKCt.GetElement(col, row));
                if (diff > max_err) {
                    max_err = diff;
                    err_r = row;
                    err_c = col;
                }
                if (CKCt.GetElement(row, col) > maxval)
                    maxval = CKCt.GetElement(row, col);
            }
        if (max_err > 1e-10)
            GetLog() << "NONSYMMETRIC corotated matrix! err " << max_err << " at " << err_r << "," << err_c
                     << ",   maxval=" << maxval << "\n";

        // DEBUG
        /*
        ChMatrixDynamic<> Ctest(12,12);
        Ctest.PasteMatrix(A,0,0);
        Ctest.PasteMatrix(A,3,3);
        Ctest.PasteMatrix(A,6,6);
        Ctest.PasteMatrix(A,9,9);
        ChMatrixDynamic<> CKtest(12,12);
        CKtest.MatrMultiply(Ctest,StiffnessMatrix);
        ChMatrixDynamic<> CKCttest(12,12);
        CKCttest.MatrMultiplyT(CKtest,Ctest);
        GetLog() << "CKCt difference \n" << CKCt-CKCttest << "\n";
        */

        // For K stiffness matrix and R damping matrix:

        double mkfactor = Kfactor + Rfactor * this->GetMaterial()->Get_RayleighDampingK();

        CKCt.MatrScale(mkfactor);

        H.PasteMatrix(CKCt, 0, 0);

        // For M mass matrix:
        if (Mfactor) {
            double lumped_node_mass = (this->GetVolume() * this->Material->Get_density()) / 4.0;
            for (int id = 0; id < 12; id++) {
                double amfactor = Mfactor + Rfactor * this->GetMaterial()->Get_RayleighDampingM();
                H(id, id) += amfactor * lumped_node_mass;
            }
        }
        //***TO DO*** better per-node lumping, or 12x12 consistent mass matrix.
    }

    /// Computes the internal forces (ex. the actual position of
    /// nodes is not in relaxed reference position) and set values
    /// in the Fi vector.
    virtual void ComputeInternalForces(ChMatrixDynamic<>& Fi) override {
        assert((Fi.GetRows() == 12) && (Fi.GetColumns() == 1));

        // set up vector of nodal displacements (in local element system) u_l = R*p - p0
        ChMatrixDynamic<> displ(12, 1);
        this->GetStateBlock(displ);  // nodal displacements, local

        // [local Internal Forces] = [Klocal] * displ + [Rlocal] * displ_dt
        ChMatrixDynamic<> FiK_local(12, 1);
        FiK_local.MatrMultiply(StiffnessMatrix, displ);

        displ.PasteVector(A.MatrT_x_Vect(nodes[0]->pos_dt), 0, 0);  // nodal speeds, local
        displ.PasteVector(A.MatrT_x_Vect(nodes[1]->pos_dt), 3, 0);
        displ.PasteVector(A.MatrT_x_Vect(nodes[2]->pos_dt), 6, 0);
        displ.PasteVector(A.MatrT_x_Vect(nodes[3]->pos_dt), 9, 0);
        ChMatrixDynamic<> FiR_local(12, 1);
        FiR_local.MatrMultiply(StiffnessMatrix, displ);
        FiR_local.MatrScale(this->Material->Get_RayleighDampingK());

        double lumped_node_mass = (this->GetVolume() * this->Material->Get_density()) / 4.0;
        displ.MatrScale(lumped_node_mass * this->Material->Get_RayleighDampingM());  // reuse 'displ' for performance
        FiR_local.MatrInc(displ);
        //***TO DO*** better per-node lumping, or 12x12 consistent mass matrix.

        FiK_local.MatrInc(FiR_local);

        FiK_local.MatrScale(-1.0);

        // Fi = C * Fi_local  with C block-diagonal rotations A
        ChMatrixCorotation<>::ComputeCK(FiK_local, this->A, 4, Fi);
    }

    //
    // Custom properties functions
    //

    /// Set the material of the element
    void SetMaterial(std::shared_ptr<ChContinuumElastic> my_material) { Material = my_material; }
    std::shared_ptr<ChContinuumElastic> GetMaterial() { return Material; }

    /// Get the partial derivatives matrix MatrB and the StiffnessMatrix
    ChMatrix<>& GetMatrB() { return MatrB; }
    ChMatrix<>& GetStiffnessMatrix() { return StiffnessMatrix; }

    /// Returns the strain tensor (note that the tetahedron 4 nodes is a linear
    /// element, thus the strain is constant in the entire volume).
    /// The tensor is in the original undeformed unrotated reference.
    ChStrainTensor<> GetStrain() {
        // set up vector of nodal displacements (in local element system) u_l = R*p - p0
        ChMatrixDynamic<> displ(12, 1);
        this->GetStateBlock(displ);  // nodal displacements, local

        ChStrainTensor<> mstrain;
        mstrain.MatrMultiply(MatrB, displ);
        return mstrain;
    }
    /// Returns the stress tensor (note that the tetahedron 4 nodes is a linear
    /// element, thus the stress is constant in the entire volume).
    /// The tensor is in the original undeformed unrotated reference.
    ChStressTensor<> GetStress() {
        ChStressTensor<> mstress;
        mstress.MatrMultiply(this->Material->Get_StressStrainMatrix(), this->GetStrain());
        return mstress;
    }
    /// This class computes and adds corresponding masses to ElementBase member m_TotalMass
    void ComputeNodalMass() override {
        nodes[0]->m_TotalMass += this->GetVolume() * this->Material->Get_density() / 4.0;
        nodes[1]->m_TotalMass += this->GetVolume() * this->Material->Get_density() / 4.0;
        nodes[2]->m_TotalMass += this->GetVolume() * this->Material->Get_density() / 4.0;
        nodes[3]->m_TotalMass += this->GetVolume() * this->Material->Get_density() / 4.0;
    }
    //
    // Functions for interfacing to the solver
    //            (***not needed, thank to bookkeeping in parent class ChElementGeneric)

    //
    // Functions for ChLoadable interface
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() override { return 4 * 3; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() override { return 4 * 3; }

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override {
        mD.PasteVector(this->nodes[0]->GetPos(), block_offset, 0);
        mD.PasteVector(this->nodes[1]->GetPos(), block_offset + 3, 0);
        mD.PasteVector(this->nodes[2]->GetPos(), block_offset + 6, 0);
        mD.PasteVector(this->nodes[3]->GetPos(), block_offset + 9, 0);
    }

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override {
        mD.PasteVector(this->nodes[0]->GetPos_dt(), block_offset, 0);
        mD.PasteVector(this->nodes[1]->GetPos_dt(), block_offset + 3, 0);
        mD.PasteVector(this->nodes[2]->GetPos_dt(), block_offset + 6, 0);
        mD.PasteVector(this->nodes[3]->GetPos_dt(), block_offset + 9, 0);
    }

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv) override {
        for (int i=0; i<4; ++i) {
            nodes[i]->NodeIntStateIncrement(off_x + i*3  , x_new, x, off_v  + i*3  , Dv);
        }
    }

    /// Number of coordinates in the interpolated field: here the {x,y,z} displacement
    virtual int Get_field_ncoords() override { return 3; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() override { return 4; }

    /// Get the offset of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockOffset(int nblock) override { return nodes[nblock]->NodeGetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockSize(int nblock) override { return 3; }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override {
        for (int i = 0; i < nodes.size(); ++i)
            mvars.push_back(&this->nodes[i]->Variables());
    };

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U,V,W coordinates of the volume, each ranging in 0..+1
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
                           ) override {
        // evaluate shape functions (in compressed vector), btw. not dependant on state
        ChMatrixNM<double, 1, 4> N;
        this->ShapeFunctions(
            N, U, V, W);  // note: U,V,W in 0..1 range, thanks to IsTetrahedronIntegrationNeeded() {return true;}

        detJ = 6 * this->GetVolume();

        Qi(0) = N(0) * F(0);
        Qi(1) = N(0) * F(1);
        Qi(2) = N(0) * F(2);
        Qi(3) = N(1) * F(0);
        Qi(4) = N(1) * F(1);
        Qi(5) = N(1) * F(2);
        Qi(6) = N(2) * F(0);
        Qi(7) = N(2) * F(1);
        Qi(8) = N(2) * F(2);
        Qi(9) = N(3) * F(0);
        Qi(10) = N(3) * F(1);
        Qi(11) = N(3) * F(2);
    }

    /// This is needed so that it can be accessed by ChLoaderVolumeGravity
    virtual double GetDensity() override { return this->Material->Get_density(); }

    /// If true, use quadrature over u,v,w in [0..1] range as tetrahedron volumetric coords, with z=1-u-v-w
    /// otherwise use quadrature over u,v,w in [-1..+1] as box isoparametric coords.
    virtual bool IsTetrahedronIntegrationNeeded() override { return true; }
};

/// Tetahedron FEM element with 4 nodes for scalar fields (for Poisson-like problems).
/// This is a classical element with linear displacement.
/// ***EXPERIMENTAL***
class ChApiFea ChElementTetra_4_P : public ChElementTetrahedron, public ChLoadableUVW {
  protected:
    std::vector<std::shared_ptr<ChNodeFEAxyzP> > nodes;
    std::shared_ptr<ChContinuumPoisson3D> Material;
    ChMatrixDynamic<> MatrB;            // matrix of shape function's partial derivatives
    ChMatrixDynamic<> StiffnessMatrix;  // local stiffness matrix

    ChMatrixNM<double, 4, 4> mM;  // for speeding up corotational approach

  public:
    ChElementTetra_4_P() {
        nodes.resize(4);
        this->MatrB.Resize(3, 12);
        this->StiffnessMatrix.Resize(4, 4);
    }

    virtual ~ChElementTetra_4_P(){};

    virtual int GetNnodes() override { return 4; }
    virtual int GetNdofs() override { return 4 * 1; }
    virtual int GetNodeNdofs(int n) override { return 1; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return nodes[n]; }

    virtual void SetNodes(std::shared_ptr<ChNodeFEAxyzP> nodeA,
                          std::shared_ptr<ChNodeFEAxyzP> nodeB,
                          std::shared_ptr<ChNodeFEAxyzP> nodeC,
                          std::shared_ptr<ChNodeFEAxyzP> nodeD) {
        nodes[0] = nodeA;
        nodes[1] = nodeB;
        nodes[2] = nodeC;
        nodes[3] = nodeD;
        std::vector<ChVariables*> mvars;
        mvars.push_back(&nodes[0]->Variables());
        mvars.push_back(&nodes[1]->Variables());
        mvars.push_back(&nodes[2]->Variables());
        mvars.push_back(&nodes[3]->Variables());
        Kmatr.SetVariables(mvars);
    }

    //
    // FEM functions
    //

    /// Fills the N shape function matrix with the
    /// values of shape functions at zi parametric coordinates, where
    /// z0=1 at 1st vertex, z1=1 at second, z2 = 1 at third (volumetric shape functions).
    /// The 4th is computed form the first 3.  All ranging in [0...1].
    /// NOTE! actually N should be a 3row, 12 column sparse matrix,
    /// as  N = [n1*eye(3) n2*eye(3) n3*eye(3) n4*eye(3)]; ,
    /// but to avoid wasting zero and repeated elements, here
    /// it stores only the n1 n2 n3 n4 values in a 1 row, 4 columns matrix!
    virtual void ShapeFunctions(ChMatrix<>& N, double z0, double z1, double z2) {
        N(0) = z0;
        N(1) = z1;
        N(2) = z2;
        N(3) = 1.0 - z0 - z1 - z2;
    };

    /// Fills the D vector column matrix with the current
    /// field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    /// For corotational elements, field is assumed in local reference!
    virtual void GetStateBlock(ChMatrixDynamic<>& mD) override {
        mD.Reset(this->GetNdofs(), 1);
        mD(0) = nodes[0]->GetP();
        mD(1) = nodes[1]->GetP();
        mD(2) = nodes[2]->GetP();
        mD(3) = nodes[3]->GetP();
    }

    double ComputeVolume() {
        ChVector<> B1, C1, D1;
        B1.Sub(nodes[1]->GetPos(), nodes[0]->GetPos());
        C1.Sub(nodes[2]->GetPos(), nodes[0]->GetPos());
        D1.Sub(nodes[3]->GetPos(), nodes[0]->GetPos());
        ChMatrixDynamic<> M(3, 3);
        M.PasteVector(B1, 0, 0);
        M.PasteVector(C1, 0, 1);
        M.PasteVector(D1, 0, 2);
        M.MatrTranspose();
        Volume = std::abs(M.Det() / 6);
        return Volume;
    }

    /// Computes the local STIFFNESS MATRIX of the element:
    /// K = Volume * [B]' * [D] * [B]
    virtual void ComputeStiffnessMatrix() {
        // M = [ X0_0 X0_1 X0_2 X0_3 ] ^-1
        //     [ 1    1    1    1    ]
        mM.PasteVector(nodes[0]->GetPos(), 0, 0);
        mM.PasteVector(nodes[1]->GetPos(), 0, 1);
        mM.PasteVector(nodes[2]->GetPos(), 0, 2);
        mM.PasteVector(nodes[3]->GetPos(), 0, 3);
        mM(3, 0) = 1.0;
        mM(3, 1) = 1.0;
        mM(3, 2) = 1.0;
        mM(3, 3) = 1.0;
        mM.MatrInverse();

        MatrB.Reset(3, 4);
        MatrB(0, 0) = mM(0);
        MatrB(0, 1) = mM(4);
        MatrB(0, 2) = mM(8);
        MatrB(0, 3) = mM(12);
        MatrB(1, 0) = mM(1);
        MatrB(1, 1) = mM(5);
        MatrB(1, 2) = mM(9);
        MatrB(1, 3) = mM(13);
        MatrB(2, 0) = mM(2);
        MatrB(2, 1) = mM(6);
        MatrB(2, 2) = mM(10);
        MatrB(2, 3) = mM(14);

        ChMatrixNM<double, 3, 4> EB;
        EB.MatrMultiply(this->Material->Get_ConstitutiveMatrix(), MatrB);

        StiffnessMatrix.MatrTMultiply(MatrB, EB);

        StiffnessMatrix.MatrScale(Volume);
    }

    /// set up the element's parameters and matrices
    virtual void SetupInitial(ChSystem* system) override {
        ComputeVolume();
        ComputeStiffnessMatrix();
    }

    // compute large rotation of element for corotational approach
    // Btw: NOT really needed for Poisson problems
    virtual void UpdateRotation() override {
        // P = [ p_0  p_1  p_2  p_3 ]
        //     [ 1    1    1    1   ]
        ChMatrixNM<double, 4, 4> P;
        P.PasteVector(nodes[0]->GetPos(), 0, 0);
        P.PasteVector(nodes[1]->GetPos(), 0, 1);
        P.PasteVector(nodes[2]->GetPos(), 0, 2);
        P.PasteVector(nodes[3]->GetPos(), 0, 3);
        P(3, 0) = 1.0;
        P(3, 1) = 1.0;
        P(3, 2) = 1.0;
        P(3, 3) = 1.0;

        ChMatrix33<double> F;
        // F=P*mM (only upper-left 3x3 block!)
        double sum;
        for (int colres = 0; colres < 3; ++colres)
            for (int row = 0; row < 3; ++row) {
                sum = 0;
                for (int col = 0; col < 4; ++col)
                    sum += (P(row, col)) * (mM(col, colres));
                F(row, colres) = sum;
            }
        ChMatrix33<> S;
        double det = ChPolarDecomposition<>::Compute(F, this->A, S, 1E-6);
        if (det < 0)
            this->A.MatrScale(-1.0);
    }

    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    virtual void ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor = 0, double Mfactor = 0) override {
        assert((H.GetRows() == 4) && (H.GetColumns() == 4));

        // For K  matrix (jacobian d/dT of  c dT/dt + div [C] grad T = f )

        ChMatrixDynamic<> mK(this->StiffnessMatrix);  // local copy of stiffness
        mK.MatrScale(Kfactor);

        H.PasteMatrix(mK, 0, 0);

        // For R  matrix: (jacobian d/d\dot(T) of  c dT/dt + div [C] grad T = f )
        if (Rfactor)
            if (this->GetMaterial()->Get_DtMultiplier()) {
                // lumped approx. integration of c
                double lumped_node_c = (this->GetVolume() * this->GetMaterial()->Get_DtMultiplier()) / 4.0;
                for (int id = 0; id < 4; id++) {
                    H(id, id) += Rfactor * lumped_node_c;
                }
            }
        //***TO DO*** better per-node lumping, or 4x4 consistent c integration as per mass matrices.

        // For M mass matrix: NONE in Poisson equation c dT/dt + div [C] grad T = f
    }

    /// Computes the internal 'pseudo-forces' and set values
    /// in the Fi vector. The iterative solver uses this to know if the residual went to zero.
    virtual void ComputeInternalForces(ChMatrixDynamic<>& Fi) override {
        assert((Fi.GetRows() == 4) && (Fi.GetColumns() == 1));

        // set up vector of nodal fields
        ChMatrixDynamic<> displ(4, 1);
        this->GetStateBlock(displ);

        // [local Internal Forces] = [Klocal] * P
        ChMatrixDynamic<> FiK_local(4, 1);
        FiK_local.MatrMultiply(StiffnessMatrix, displ);

        //***TO DO*** derivative terms? + [Rlocal] * P_dt ???? ***NO because Poisson  rho dP/dt + div [C] grad P = 0

        FiK_local.MatrScale(-1.0);

        // ChMatrixCorotation<>::ComputeCK(FiK_local, this->A, 4, Fi);  ***corotation NOT NEEDED

        Fi = FiK_local;
    }

    //
    // Custom properties functions
    //

    /// Set the material of the element
    void SetMaterial(std::shared_ptr<ChContinuumPoisson3D> my_material) { Material = my_material; }
    std::shared_ptr<ChContinuumPoisson3D> GetMaterial() { return Material; }

    /// Get the partial derivatives matrix MatrB and the StiffnessMatrix
    ChMatrix<>& GetMatrB() { return MatrB; }
    ChMatrix<>& GetStiffnessMatrix() { return StiffnessMatrix; }

    /// Returns the gradient of P (note that the tetahedron 4 nodes is a linear
    /// element, thus the gradient is constant in the entire volume).
    /// It is in the original undeformed unrotated reference.
    ChMatrixNM<double, 3, 1> GetPgradient() {
        // set up vector of nodal displacements (in local element system) u_l = R*p - p0
        ChMatrixDynamic<> displ(4, 1);
        this->GetStateBlock(displ);

        ChMatrixNM<double, 3, 1> mPgrad;
        mPgrad.MatrMultiply(MatrB, displ);
        return mPgrad;
    }

    //
    // Functions for interfacing to the solver
    //            (***not needed, thank to bookkeeping in parent class ChElementGeneric)

    //
    // Functions for ChLoadable interface
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() override { return 4 * 3; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() override { return 4 * 3; }

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override {
        mD(block_offset) = this->nodes[0]->GetP();
        mD(block_offset + 1) = this->nodes[1]->GetP();
        mD(block_offset + 2) = this->nodes[2]->GetP();
        mD(block_offset + 3) = this->nodes[3]->GetP();
    }

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override {
        mD(block_offset) = this->nodes[0]->GetP_dt();
        mD(block_offset + 1) = this->nodes[1]->GetP_dt();
        mD(block_offset + 2) = this->nodes[2]->GetP_dt();
        mD(block_offset + 3) = this->nodes[3]->GetP_dt();
    }

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv) override {
        for (int i=0; i<4; ++i) {
            nodes[i]->NodeIntStateIncrement(off_x + i*1  , x_new, x, off_v  + i*1  , Dv);
        }
    }

    /// Number of coordinates in the interpolated field: here the {t} temperature
    virtual int Get_field_ncoords() override { return 1; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() override { return 4; }

    /// Get the offset of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockOffset(int nblock) override { return nodes[nblock]->NodeGetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockSize(int nblock) override { return 1; }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override {
        for (int i = 0; i < nodes.size(); ++i)
            mvars.push_back(&this->nodes[i]->Variables());
    };

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U,V,W coordinates of the volume, each ranging in 0..+1
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
                           ) override {
        // evaluate shape functions (in compressed vector), btw. not dependant on state
        ChMatrixNM<double, 1, 4> N;
        this->ShapeFunctions(
            N, U, V, W);  // note: U,V,W in 0..1 range, thanks to IsTetrahedronIntegrationNeeded() {return true;}

        detJ = 6 * this->GetVolume();

        Qi(0) = N(0) * F(0);
        Qi(1) = N(1) * F(0);
        Qi(2) = N(2) * F(0);
        Qi(3) = N(3) * F(0);
    }

    /// Return 0 if not supprotable by ChLoaderVolumeGravity
    virtual double GetDensity() override { return 0; }

    /// If true, use quadrature over u,v,w in [0..1] range as tetrahedron volumetric coords, with z=1-u-v-w
    /// otherwise use quadrature over u,v,w in [-1..+1] as box isoparametric coords.
    virtual bool IsTetrahedronIntegrationNeeded() override { return true; }
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
