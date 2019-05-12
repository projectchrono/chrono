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
// Authors: Kassem Mohamad, Alessandro Tasora
// =============================================================================

#ifndef CHELEMENTBEAMIGA_H
#define CHELEMENTBEAMIGA_H

//#define BEAM_VERBOSE

#include "chrono/fea/ChElementBeam.h"
#include "chrono/fea/ChBeamSectionCosserat.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"
#include "chrono/geometry/ChBasisToolsBspline.h"
#include "chrono/core/ChQuadrature.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Element of IGA type, with Timoshenko shear etc.
/// et.. etc.  (intro to write)
/// Note: each IGA element represents one "knot span" of the spline!
class  ChElementBeamIGA :   public ChElementBeam,
                            public ChLoadableU,
							public ChLoadableUVW
{
  protected:

    std::vector< std::shared_ptr<ChNodeFEAxyzrot> > nodes; // also "control points" 
    ChVectorDynamic<> knots;

    int order;

    int int_order_s;
    int int_order_b;

    std::vector< double > Jacobian_s;
    std::vector< double > Jacobian_b;

	std::vector< ChVector<> > strain_e_0;
	std::vector< ChVector<> > strain_k_0;

	std::vector< ChVector<> > stress_n;
	std::vector< ChVector<> > stress_m;
	std::vector< ChVector<> > strain_e;
	std::vector< ChVector<> > strain_k;

	std::vector< std::unique_ptr<ChBeamMaterialInternalData> > plastic_data_old;
	std::vector< std::unique_ptr<ChBeamMaterialInternalData> > plastic_data;

	std::shared_ptr<ChBeamSectionCosserat> section;

  public:
    ChElementBeamIGA() {
        order = 3;
        nodes.resize(4); // controllare se ordine = -> 2 nodi, 2 control points, o di più
        knots.Resize(8);
        int_order_s = 1;
        int_order_b = 1;
    }

    virtual ~ChElementBeamIGA() {}

    virtual int GetNnodes() override { return (int)nodes.size(); }
    virtual int GetNdofs() override { return GetNnodes() * 6; }
    virtual int GetNodeNdofs(int n) override { return 6; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return nodes[n]; }
	virtual std::vector< std::shared_ptr<ChNodeFEAxyzrot> >& GetNodes() { return  nodes; }

	virtual void SetNodesCubic(std::shared_ptr<ChNodeFEAxyzrot> nodeA, std::shared_ptr<ChNodeFEAxyzrot> nodeB, std::shared_ptr<ChNodeFEAxyzrot> nodeC, std::shared_ptr<ChNodeFEAxyzrot> nodeD, double knotA1, double knotA2, double knotB1, double knotB2, double knotB3, double knotB4, double knotB5, double knotB6) {
        nodes.resize(4);
        nodes[0] = nodeA;
        nodes[1] = nodeB;
		nodes[2] = nodeC;
		nodes[3] = nodeD;
        knots.Resize(8);
        knots(0) = knotA1;
        knots(1) = knotA2;
        knots(2) = knotB1;
        knots(3) = knotB2;
		knots(4) = knotB3;
		knots(5) = knotB4;
		knots(6) = knotB5;
		knots(7) = knotB6;
        std::vector<ChVariables*> mvars;
        mvars.push_back(&nodes[0]->Variables());
        mvars.push_back(&nodes[1]->Variables());
		mvars.push_back(&nodes[2]->Variables());
		mvars.push_back(&nodes[3]->Variables());
        Kmatr.SetVariables(mvars);
        
        int_order_s = 1;
        int_order_b = 1;
    }

    virtual void SetNodesGenericOrder(std::vector<std::shared_ptr<ChNodeFEAxyzrot>> mynodes, std::vector<double> myknots, int myorder) {
		this->order = myorder;

        nodes.resize(myorder+1);
        for (int i= 0; i< mynodes.size(); ++i) {
            nodes[i] = mynodes[i];
        }
        knots.Resize((int)nodes.size()+myorder+1);
        for (int i= 0; i< myknots.size(); ++i) {
            knots(i) = myknots[i];
        }

        std::vector<ChVariables*> mvars;
        for (int i= 0; i< mynodes.size(); ++i) {
            mvars.push_back(&nodes[i]->Variables());
        }
        Kmatr.SetVariables(mvars);

        // integration for in-between elements:
        int_order_s = 1;

		 // FULL OVER INTEGRATION:
		//int_order_b = myorder+1;
		 // FULL EXACT INTEGRATION:
		int_order_b = (int)std::ceil((this->order + 1.0) / 2.0);
		 // REDUCED INTEGRATION:
        //int_order_b = myorder; 
		 // SELECTIVE INTEGRATION:
		//int_order_b = 1;

		// ensure integration order is odd, to have a centered gauss point
		//if (int_order_b % 2 == 0) {
		//	int_order_b += 1;
		//}

        // Full integration for end elements:
        int multiplicity_a = 1;
        int multiplicity_b = 1;
        for (int im = order-1; im>=0; --im) {
            if (knots(im) == knots(order)) // extreme of span
                ++multiplicity_a;
        }
        for (int im = knots.GetRows() - order ; im<knots.GetRows(); ++im) {
            if (knots(im) == knots(knots.GetRows() - order - 1)) // extreme of span
                ++multiplicity_b;
        }
        if (multiplicity_a > 1 || multiplicity_b > 1){ 
            int_order_s = (int)std::ceil((this->order+1.0)/2.0);
            int_order_b = (int)std::ceil((this->order+1.0)/2.0);
        }
    }

    /// Set the integration points, for shear components and for bending components:
    void SetIntegrationPoints(int npoints_s, int npoints_b) {
        int_order_s = npoints_s;
        int_order_b = npoints_b;
    }

    //
    // FEM functions
    //


	/// Set the section & material of beam element .
	/// It is a shared property, so it can be shared between other beams.
	void SetSection(std::shared_ptr<ChBeamSectionCosserat> my_material) { section = my_material; }
	/// Get the section & material of the element
	std::shared_ptr<ChBeamSectionCosserat> GetSection() { return section; }

    /// Access the local knot sequence of this element (ex.for diagnostics)
    ChVectorDynamic<>& GetKnotSequence() {return this->knots;}

	/// Get the parametric coordinate at the beginning of the span
	double GetU1() { return knots(order); }
	/// Get the parametric coordinate at the end of the span
	double GetU2() { return knots(knots.GetRows() - order - 1); }

    virtual void Update() override {
        // parent class update:
        ChElementGeneric::Update();

    };

	/// Get the plastic data, in a vector with as many elements as Gauss points.
	std::vector< std::unique_ptr<ChBeamMaterialInternalData> >& GetPlasticData() { return plastic_data;}

	/// Get the stress, as cut-force [N], in a vector with as many elements as Gauss points.
	std::vector< ChVector<>>& GetStressN() { return this->stress_n; }

	/// Get the stress, as cut-torque [Nm], in a vector with as many elements as Gauss points.
	std::vector< ChVector<>>& GetStressM() { return this->stress_m; }
	
	/// Get the strain (total=elastic+plastic), as deformation (x is axial strain), in a vector with as many elements as Gauss points.
	std::vector< ChVector<>>& GetStrainE() { return this->strain_e; }
	
	/// Get the strain (total=elastic+plastic), as curvature (x is torsion), in a vector with as many elements as Gauss points.
	std::vector< ChVector<>>& GetStrainK() { return this->strain_k; }


    /// Setup. Precompute mass and matrices that do not change during the
    /// simulation.
    /// In particular, compute the arc-length parametrization.

    virtual void SetupInitial(ChSystem* system) override {
        assert(section);

		if (this->section->GetPlasticity()) {
			this->section->GetPlasticity()->CreatePlasticityData(int_order_b, this->plastic_data_old);
			this->section->GetPlasticity()->CreatePlasticityData(int_order_b, this->plastic_data);
		}

        this->length=0;

        // get two values of absyssa at extreme of span
        double u1 = knots(order); 
		double u2 = knots(knots.GetRows() - order - 1);

        double c1 = (u2 - u1) / 2;
        double c2 = (u2 + u1) / 2;

        // Gauss points for "s" shear components:

        this->Jacobian_s.resize(int_order_s);

        for (int ig = 0; ig < int_order_s; ++ig) {

            // absyssa in typical -1,+1 range:
            double eta = ChQuadrature::GetStaticTables()->Lroots[int_order_s-1][ig];
            // absyssa in span range:
            double u = (c1 * eta + c2); 
            // scaling = gauss weight * change of range:
            double w = ChQuadrature::GetStaticTables()->Weight[int_order_s-1][ig];

            // compute the basis functions N(u) at given u:
            int nspan = order;

            ChMatrixDynamic<> N(2,(int)nodes.size()); // row n.0 contains N, row n.1 contains dN/du

            geometry::ChBasisToolsBspline::BasisEvaluateDeriv(
                   this->order,  
                   nspan,
                   u,  
                   knots, 
                   N);           ///< here return N and dN/du 

            // compute reference spline gradient \dot{dr_0} = dr0/du
            ChVector<> dr0; 
            for (int i = 0 ; i< nodes.size(); ++i) {
                dr0 += nodes[i]->GetX0ref().coord.pos * N(1,i);
            }
            this->Jacobian_s[ig] = dr0.Length(); // J = |dr0/du|
        } 

        // Gauss points for "b" bend components:

        this->Jacobian_b.resize(int_order_b);
		this->strain_e_0.resize(int_order_b);
		this->strain_k_0.resize(int_order_b);
		this->stress_m.resize(int_order_b);
		this->stress_n.resize(int_order_b);
		this->strain_e.resize(int_order_b);
		this->strain_k.resize(int_order_b);

        for (int ig = 0; ig < int_order_b; ++ig) {

            // absyssa in typical -1,+1 range:
            double eta = ChQuadrature::GetStaticTables()->Lroots[int_order_b-1][ig];
            // absyssa in span range:
            double u = (c1 * eta + c2); 
            // scaling = gauss weight * change of range:
            double w = ChQuadrature::GetStaticTables()->Weight[int_order_b-1][ig];

            // compute the basis functions N(u) at given u:
            int nspan = order;

            ChMatrixDynamic<> N(2,(int)nodes.size()); // row n.0 contains N, row n.1 contains dN/du

            geometry::ChBasisToolsBspline::BasisEvaluateDeriv(
                   this->order,  
                   nspan,
                   u,  
                   knots, 
                   N);           ///< here return N and dN/du 

            // compute reference spline gradient \dot{dr_0} = dr0/du
            ChVector<> dr0; 
            for (int i = 0 ; i< nodes.size(); ++i) {
                dr0 += nodes[i]->GetX0ref().coord.pos * N(1,i);
            }
            this->Jacobian_b[ig] = dr0.Length(); // J = |dr0/du|

			// From now on, compute initial strains as in ComputeInternalForces 

			// Jacobian Jsu = ds/du
			double Jsu = this->Jacobian_b[ig];
			// Jacobian Jue = du/deta
			double Jue = c1;

			// interpolate rotation of section at given u, to compute R.
			// Note: this is approximate.
			// A more precise method: use quaternion splines, as in Kim,Kim and Shin, 1995 paper.
			ChQuaternion<> q_delta;
			ChVector<> da = VNULL;
			ChVector<> delta_rot_dir;
			double delta_rot_angle;
			for (int i = 0; i< nodes.size(); ++i) {
				ChQuaternion<> q_i = nodes[i]->GetX0ref().GetRot(); // state_x.ClipQuaternion(i * 7 + 3, 0);
				q_delta = nodes[0]->GetX0ref().GetRot().GetConjugate() * q_i;
				q_delta.Q_to_AngAxis(delta_rot_angle, delta_rot_dir); // a_i = dir_i*angle_i (in spline local reference, -PI..+PI)
				da += delta_rot_dir * delta_rot_angle * N(0, i);  // a = N_i*a_i
			}
			ChQuaternion<> qda; qda.Q_from_Rotv(da);
			ChQuaternion<> qR = nodes[0]->GetX0ref().GetRot() * qda;

			// compute the 3x3 rotation matrix R equivalent to quaternion above
			ChMatrix33<> R(qR);

			// compute abs. spline gradient r'  = dr/ds
			ChVector<> dr;
			for (int i = 0; i< nodes.size(); ++i) {
				ChVector<> r_i = nodes[i]->GetX0().GetPos();
				dr += r_i * N(1, i);  // dr/du = N_i'*r_i
			}
			// (note r'= dr/ds = dr/du du/ds = dr/du * 1/Jsu   where Jsu computed in SetupInitial)
			dr *= 1.0 / Jsu;

			// compute abs spline rotation gradient q' = dq/ds 
			// But.. easier to compute local gradient of spin vector a' = da/ds
			da = VNULL;
			for (int i = 0; i< nodes.size(); ++i) {
				ChQuaternion<> q_i = nodes[i]->GetX0ref().GetRot();
				q_delta = qR.GetConjugate() * q_i;
				q_delta.Q_to_AngAxis(delta_rot_angle, delta_rot_dir); // a_i = dir_i*angle_i (in spline local reference, -PI..+PI)
				da += delta_rot_dir * delta_rot_angle * N(1, i);  // da/du = N_i'*a_i
			}
			// (note a= da/ds = da/du du/ds = da/du * 1/Jsu   where Jsu computed in SetupInitial)
			da *= 1.0 / Jsu;

			// compute local epsilon strain:  strain_e= R^t * r' - {1, 0, 0}
			this->strain_e_0[ig] = R.MatrT_x_Vect(dr) - VECT_X;

			// compute local curvature strain:  strain_k= 2*[F(q*)(+)] * q' = 2*[F(q*)(+)] * N_i'*q_i = R^t * a' = a'_local
			this->strain_k_0[ig] = da;

			// as a byproduct, also compute length 
			this->length += w * Jsu * Jue;
        }

		// as a byproduct, also compute total mass
		this->mass = this->length * this->section->GetArea() * this->section->GetDensity();
    }


    /// Fills the D vector (column matrix) with the current
    /// field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    virtual void GetStateBlock(ChMatrixDynamic<>& mD) override {
        mD.Reset((int)this->nodes.size()*7, 1);

        for (int i= 0; i< nodes.size(); ++i) {
            mD.PasteVector( nodes[i]->coord.pos, i*7, 0);
            mD.PasteQuaternion( nodes[i]->coord.rot, i*7+3, 0);
        }
       
    }


    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    virtual void ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor = 0, double Mfactor = 0) override {
        assert((H.GetRows() == 6 * (int)nodes.size()) && (H.GetColumns() == 6 * (int)nodes.size()));
        assert(section);

        // BRUTE FORCE METHOD: USE NUMERICAL DIFFERENTIATION!

        //
        // The K stiffness matrix of this element span:  
        //

        ChState      state_x(this->LoadableGet_ndof_x(), nullptr);
        ChStateDelta state_w(this->LoadableGet_ndof_w(), nullptr);
        this->LoadableGetStateBlock_x(0,state_x);
        this->LoadableGetStateBlock_w(0,state_w);

        double Delta = 1e-10;
        
        int mrows_w = this->LoadableGet_ndof_w();
        int mrows_x = this->LoadableGet_ndof_x();

        ChMatrixDynamic<> K(mrows_w, mrows_w); 

        // compute Q at current speed & position, x_0, v_0
        ChMatrixDynamic<> Q0(mrows_w,1);
        this->ComputeInternalForces_impl(Q0, state_x, state_w, true);     // Q0 = Q(x, v)

        ChMatrixDynamic<> Q1(mrows_w,1);
        ChVectorDynamic<> Jcolumn(mrows_w);
        ChState       state_x_inc(mrows_x, nullptr);
        ChStateDelta  state_delta(mrows_w, nullptr);

        // Compute K=-dQ(x,v)/dx by backward differentiation
        state_delta.Reset(mrows_w, nullptr);

        for (int i=0; i<mrows_w; ++i) {
            state_delta(i)+= Delta;
            this->LoadableStateIncrement(0, state_x_inc, state_x, 0, state_delta);  // exponential, usually state_x_inc(i) = state_x(i) + Delta;

            Q1.Reset(mrows_w,1);
            this->ComputeInternalForces_impl(Q1, state_x_inc, state_w, true);   // Q1 = Q(x+Dx, v)
            state_delta(i)-= Delta;
            
            Jcolumn = (Q1 - Q0)*(-1.0/Delta);   // - sign because K=-dQ/dx
            K.PasteMatrix(Jcolumn,0,i);
        }

		// finally, store K into H:

		K.MatrScale(Kfactor);

		H.PasteMatrix(K, 0, 0);

        // Compute R=-dQ(x,v)/dv by backward differentiation
		if (this->section->GetDamping()) {
			ChStateDelta  state_w_inc(mrows_w, nullptr);
			state_w_inc = state_w;
			ChMatrixDynamic<> R(mrows_w, mrows_w);

			for (int i = 0; i < mrows_w; ++i) {
				Q1.Reset(mrows_w, 1);

				state_w_inc(i) += Delta;
				this->ComputeInternalForces_impl(Q1, state_x, state_w_inc, true); // Q1 = Q(x, v+Dv)
				state_w_inc(i) -= Delta;

				Jcolumn = (Q1 - Q0)*(-1.0 / Delta);   // - sign because R=-dQ/dv
				R.PasteMatrix(Jcolumn, 0, i);
			}
			
			R.MatrScale(Rfactor);

			H.PasteSumMatrix(R, 0, 0);
		}

		
        



        //
        // The M mass matrix of this element span: (lumped version)
        //

        ChMatrixDynamic<> Mloc(6 * (int)nodes.size(), 6 * (int)nodes.size());
        Mloc.Reset();

        double nmass = mass /(double)nodes.size();
         //Iyy and Izz: (orthogonal to spline) approx as 1/50 lumped mass at half dist:
        double lineryz = (1. / 50.) * nmass * pow(length, 2);  // note: 1/50 can be even less (this is 0 in many texts, but 0 means no explicit integrator could be used) 
         //Ixx: (tangent to spline) approx as node cuboid
        double linerx =  (1. / 12.) * nmass * (pow(section->GetDrawThicknessY(),2) + pow(section->GetDrawThicknessZ(),2));

        for (int i = 0; i< nodes.size(); ++i) {
            int stride = i*6;
			double nodelineryz = lineryz;
			//if (i == 0 || i == (nodes.size() - 1)) {
				// node overlapped in neighbouring element
			//	nodelineryz = lineryz * 0.5;
			//}
            Mloc(stride+0, stride+0) += Mfactor * nmass;  // node A x,y,z
            Mloc(stride+1, stride+1) += Mfactor * nmass;
            Mloc(stride+2, stride+2) += Mfactor * nmass;
            Mloc(stride+3, stride+3) += Mfactor * linerx;  // node A Ixx,Iyy,Izz 
            Mloc(stride+4, stride+4) += Mfactor * lineryz;
            Mloc(stride+5, stride+5) += Mfactor * lineryz;
        }

        H.PasteSumMatrix(Mloc, 0, 0);

    }

    /// Computes the internal forces (ex. the actual position of
    /// nodes is not in relaxed reference position) and set values
    /// in the Fi vector.
    virtual void ComputeInternalForces(ChMatrixDynamic<>& Fi) override {
        ChState      mstate_x(this->LoadableGet_ndof_x(), nullptr);
        ChStateDelta mstate_w(this->LoadableGet_ndof_w(), nullptr);
        this->LoadableGetStateBlock_x(0,mstate_x);
        this->LoadableGetStateBlock_w(0,mstate_w);
        ComputeInternalForces_impl(Fi, mstate_x, mstate_w);
    }

	virtual void ComputeInternalForces_impl(ChMatrixDynamic<>& Fi,
									ChState&      state_x, ///< state position to evaluate Fi
									ChStateDelta& state_w, ///< state speed to evaluate Fi
									bool used_for_differentiation = false)
                                 {

        // get two values of absyssa at extreme of span
        double u1 = knots(order); 
		double u2 = knots(knots.GetRows() - order - 1);

        double c1 = (u2 - u1) / 2;
        double c2 = (u2 + u1) / 2;

        // zeroes the Fi accumulator
        Fi.Reset();

        // Do quadrature over the "s" shear Gauss points 
        // (only if int_order_b != int_order_s, otherwise do a single loop later over "b" bend points also for shear)

		//***TODO*** maybe not needed: separate bending and shear integration points. 



        // Do quadrature over the "b" bend Gauss points

		for (int ig = 0; ig < int_order_b; ++ig) {

			// absyssa in typical -1,+1 range:
			double eta = ChQuadrature::GetStaticTables()->Lroots[int_order_b - 1][ig];
			// absyssa in span range:
			double u = (c1 * eta + c2);
			// scaling = gauss weight
			double w = ChQuadrature::GetStaticTables()->Weight[int_order_b - 1][ig];

			// Jacobian Jsu = ds/du
			double Jsu = this->Jacobian_b[ig];
			// Jacobian Jue = du/deta
			double Jue = c1;

			// compute the basis functions N(u) at given u:
			int nspan = order;

			ChMatrixDynamic<> N(2, (int)nodes.size()); // row n.0 contains N, row n.1 contains dN/du

			geometry::ChBasisToolsBspline::BasisEvaluateDeriv(
				this->order,
				nspan,
				u,
				knots,
				N);           ///< here return N and dN/du 

			// interpolate rotation of section at given u, to compute R.
			// Note: this is approximate.
			// A more precise method: use quaternion splines, as in Kim,Kim and Shin, 1995 paper.
			ChQuaternion<> q_delta;
			ChVector<> da = VNULL;
			ChVector<> delta_rot_dir;
			double delta_rot_angle;
			for (int i = 0; i < nodes.size(); ++i) {
				ChQuaternion<> q_i = state_x.ClipQuaternion(i * 7 + 3, 0);
				q_delta = nodes[0]->coord.rot.GetConjugate() * q_i;
				q_delta.Q_to_AngAxis(delta_rot_angle, delta_rot_dir); // a_i = dir_i*angle_i (in spline local reference, -PI..+PI)
				da += delta_rot_dir * delta_rot_angle * N(0, i);  // a = N_i*a_i
			}
			ChQuaternion<> qda; qda.Q_from_Rotv(da);
			ChQuaternion<> qR = nodes[0]->coord.rot * qda;


			// compute the 3x3 rotation matrix R equivalent to quaternion above
			ChMatrix33<> R(qR);

			// compute abs. spline gradient r'  = dr/ds
			ChVector<> dr;
			for (int i = 0; i < nodes.size(); ++i) {
				ChVector<> r_i = state_x.ClipVector(i * 7, 0);
				dr += r_i * N(1, i);  // dr/du = N_i'*r_i
			}
			// (note r'= dr/ds = dr/du du/ds = dr/du * 1/Jsu   where Jsu computed in SetupInitial)
			dr *= 1.0 / Jsu;

			// compute abs. time rate of spline gradient  dr'/dt
			ChVector<> drdt;
			for (int i = 0; i < nodes.size(); ++i) {
				ChVector<> drdt_i = state_w.ClipVector(i * 6, 0);
				drdt += drdt_i * N(1, i);
			}
			drdt *= 1.0 / Jsu;

			// compute abs spline rotation gradient q' = dq/ds 
			// But.. easier to compute local gradient of spin vector a' = da/ds
			da = VNULL;
			for (int i = 0; i < nodes.size(); ++i) {
				ChQuaternion<> q_i = state_x.ClipQuaternion(i * 7 + 3, 0);
				q_delta = qR.GetConjugate() * q_i;
				q_delta.Q_to_AngAxis(delta_rot_angle, delta_rot_dir); // a_i = dir_i*angle_i (in spline local reference, -PI..+PI)
				da += delta_rot_dir * delta_rot_angle * N(1, i);  // da/du = N_i'*a_i
			}
			// (note a= da/ds = da/du du/ds = da/du * 1/Jsu   where Jsu computed in SetupInitial)
			da *= 1.0 / Jsu;

			// compute abs rate of spline rotation gradient da'/dt
			ChVector<> dadt;
			for (int i = 0; i < nodes.size(); ++i) {
				ChQuaternion<> q_i = state_x.ClipQuaternion(i * 7 + 3, 0);
				ChVector<> wl_i = state_w.ClipVector(i * 6 + 3, 0); //  w in node csys
				ChVector<> w_i = q_i.Rotate(wl_i); // w in absolute csys
				dadt += w_i * N(1, i);
			}
			dadt *= 1.0 / Jsu;

			// compute local epsilon strain:  strain_e= R^t * r' - {1, 0, 0}
			ChVector<> astrain_e = R.MatrT_x_Vect(dr) - VECT_X - this->strain_e_0[ig];

			// compute local time rate of strain:  strain_e_dt = R^t * dr'/dt
			ChVector<> astrain_e_dt = R.MatrT_x_Vect(drdt);

			// compute local curvature strain:  strain_k= 2*[F(q*)(+)] * q' = 2*[F(q*)(+)] * N_i'*q_i = R^t * a' = a'_local
			ChVector<> astrain_k = da - this->strain_k_0[ig];

			// compute local time rate of curvature strain:
			ChVector<> astrain_k_dt = R.MatrT_x_Vect(dadt);


			// compute stress n  (local cut forces) 
			// compute stress_m  (local cut torque) 
			ChVector<> astress_n;
			ChVector<> astress_m;
			ChBeamMaterialInternalData* aplastic_data_old = nullptr;
			ChBeamMaterialInternalData* aplastic_data = nullptr;
			std::vector< std::unique_ptr<ChBeamMaterialInternalData> > foo_plastic_data;
			if (this->section->GetPlasticity()) {
				aplastic_data_old = this->plastic_data_old[ig].get();
				aplastic_data = this->plastic_data[ig].get();
				if (used_for_differentiation) {
					// Avoid updating plastic_data_new if ComputeInternalForces_impl() called for computing 
					// stiffness matrix by backward differentiation. Otherwise pollutes the plastic_data integration.
					this->section->GetPlasticity()->CreatePlasticityData(1, foo_plastic_data);
					aplastic_data = foo_plastic_data[0].get();
				}
			}
			this->section->ComputeStress(
				astress_n,
				astress_m,
				astrain_e,
				astrain_k,
				aplastic_data,
				aplastic_data_old);

			if (!used_for_differentiation) {
				this->stress_n[ig] = astress_n;
				this->stress_m[ig] = astress_m;
				this->strain_e[ig] = astrain_e;
				this->strain_k[ig] = astrain_k;
			}

            // add viscous damping 
			if (this->section->GetDamping()) {
				ChVector<> n_sp;
				ChVector<> m_sp;
				this->section->GetDamping()->ComputeStress(
					n_sp,
					m_sp,
					astrain_e_dt,
					astrain_k_dt);
				astress_n += n_sp;
				astress_m += m_sp;
			}

            // compute internal force, in generalized coordinates:

            ChVector<> stress_n_abs = R * astress_n;
            ChVector<> stress_m_abs = R * astress_m;

            for (int i = 0; i< nodes.size(); ++i) {
                
                // -Force_i = w * Jue * Jsu * Jsu^-1 * N' * R * C * (strain_e - strain_e0)
                //          = w * Jue * N'            * stress_n_abs
                ChVector<> Force_i = stress_n_abs* N(1,i) * (-w*Jue);
			    Fi.PasteSumVector(Force_i, i *6, 0);

                // -Torque_i =   w * Jue * Jsu * Jsu^-1 * R_i^t * N'               * R * D * (strain_k - strain_k0) + 
                //             + w * Jue * Jsu *        R_i^t * N  * skew(r')^t  * R * C * (strain_e - strain_e0)
                //           =   w * Jue * R_i^t * N'                          * stress_m_abs + 
                //             + w * Jue * Jsu * R_i^t * N * skew(r')^t          * stress_n_abs 
                ChQuaternion<> q_i = state_x.ClipQuaternion(i*7+3, 0);
                ChVector<> Torque_i = q_i.RotateBack(
                    stress_m_abs * N(1,i) * (-w*Jue) 
                    - Vcross(dr, stress_n_abs) *  N(0,i) * (-w*Jue*Jsu)
                    );
                Fi.PasteSumVector(Torque_i, 3+ i *6, 0);
            }

            //GetLog() << "     gp n." << ig <<   "  J=" << this->Jacobian[ig] << "   strain_e= " << strain_e << "\n";
            //GetLog() << "                    stress_n= " << stress_n << "\n";
        } 

    }

    //
    // Beam-specific functions
    //

    /// Gets the xyz displacement of a point on the beam line,
    /// and the rotation RxRyRz of section plane, at abscyssa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Note, 'displ' is the displ.state of 2 nodes, ex. get it as GetStateBlock()
    /// Results are not corotated.
    virtual void EvaluateSectionDisplacement(const double eta,
                                             ChVector<>& u_displ,
                                             ChVector<>& u_rotaz) override {
        ChMatrixDynamic<> N(1, (int)nodes.size());
        
        /* To be completed: Created to be consistent with base class implementation*/
        
    }

	/// Gets the absolute xyz position of a point on the beam line, at abscissa 'eta'.
	/// Note, eta=-1 at node1, eta=+1 at node2.
	virtual void EvaluateSectionPoint(const double eta,
		ChVector<>& point)   {

		// compute parameter in knot space from eta-1..+1
		double u1 = knots(order); // extreme of span
		double u2 = knots(knots.GetRows() - order - 1);
		double u = u1 + ((eta + 1) / 2.0)*(u2 - u1);
		int nspan = order;

		ChVectorDynamic<> N((int)nodes.size());

		geometry::ChBasisToolsBspline::BasisEvaluate(
			this->order,
			nspan,
			u,
			knots,
			N);           ///< here return  in N

		point = VNULL;
		for (int i = 0; i< nodes.size(); ++i) {
			point += N(i) * nodes[i]->coord.pos;
		}
	}

    /// Gets the absolute xyz position of a point on the beam line,
    /// and the absolute rotation of section plane, at abscissa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    virtual void EvaluateSectionFrame(const double eta,
                                      ChVector<>& point,
                                      ChQuaternion<>& rot) override {
        // compute parameter in knot space from eta-1..+1
		
		double u1 = knots(order); // extreme of span
		double u2 = knots(knots.GetRows() - order - 1);
		double u = u1 + ((eta + 1) / 2.0)*(u2 - u1);
		int nspan = order;

        ChVectorDynamic<> N((int)nodes.size());
        
        geometry::ChBasisToolsBspline::BasisEvaluate(
               this->order,  
               nspan,
               u,  
               knots, 
               N);           ///< here return  in N

        point = VNULL;
        for (int i = 0 ; i< nodes.size(); ++i) {
            point += N(i) * nodes[i]->coord.pos;
        }
        rot  = QNULL;
        for (int i = 0 ; i< nodes.size(); ++i) {
            ChQuaternion<> myrot = nodes[i]->coord.rot;
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
    virtual void EvaluateSectionForceTorque(const double eta,
                                            ChVector<>& Fforce,
                                            ChVector<>& Mtorque) override {

        /* To be completed: Created to be consistent with base class implementation*/
        
    }

    virtual void EvaluateSectionStrain(
        const double eta,
        ChVector<>& StrainV) override { 

        /* To be completed: Created to be consistent with base class implementation*/
    }
 


    //
    // Functions for interfacing to the solver
    //            (***not needed, thank to bookkeeping in parent class ChElementGeneric)

	virtual void EleDoIntegration() override {
		for (size_t i = 0; i < this->plastic_data.size(); ++i) {
			this->plastic_data_old[i]->Copy(*this->plastic_data[i]);
		}
	}

    //
    // Functions for ChLoadable interface
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() override { return (int)nodes.size() * 7; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() override { return (int)nodes.size() * 6; }

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override {
        for (int i = 0 ; i< nodes.size(); ++i) {
            mD.PasteVector(this->nodes[i]->GetPos(), block_offset + i*7, 0);
            mD.PasteQuaternion(this->nodes[i]->GetRot(), block_offset + i*7 + 3, 0);
        }
    }

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override {
        for (int i = 0 ; i< nodes.size(); ++i) {
            mD.PasteVector(this->nodes[i]->GetPos_dt(), block_offset + i*6, 0);
            mD.PasteVector(this->nodes[i]->GetWvel_loc(), block_offset + i*6 + 3, 0);
        }
    }

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv) override {
        for (int i = 0 ; i< nodes.size(); ++i) {
            nodes[i]->NodeIntStateIncrement(off_x+ i*7 , x_new, x, off_v + i*6 , Dv);
        }
    }

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual int Get_field_ncoords() override { return 6; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() override { return (int)nodes.size(); }

    /// Get the offset of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockOffset(int nblock) override { return nodes[nblock]->NodeGetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockSize(int nblock) override { return 6; }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override {
        for (int i = 0 ; i< nodes.size(); ++i) {
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
                           ) override {
         // get two values of absyssa at extreme of span
        double u1 = knots(order); 
		double u2 = knots(knots.GetRows() - order - 1);

        double c1 = (u2 - u1) / 2;
        double c2 = (u2 + u1) / 2;

        // absyssa in span range:
        double u = (c1 * U + c2);
        
        // compute the basis functions N(u) at given u:
        int nspan = order;

        ChMatrixDynamic<> N(2,(int)nodes.size()); // row n.0 contains N, row n.1 contains dN/du

        geometry::ChBasisToolsBspline::BasisEvaluateDeriv(
                this->order,  
                nspan,
                u,  
                knots, 
                N);           ///< h
		
        ChVector<> dr0; 
        for (int i = 0 ; i< nodes.size(); ++i) {
            dr0 += nodes[i]->GetX0ref().coord.pos * N(1,i);
        }
        detJ = dr0.Length() * c1;

        for (int i = 0; i < nodes.size(); ++i) {
            int stride = i*6;
            Qi(stride+0) = N(i) * F(0);
            Qi(stride+1) = N(i) * F(1);
            Qi(stride+2) = N(i) * F(2);
            Qi(stride+3) = N(i) * F(3);
            Qi(stride+4) = N(i) * F(4);
            Qi(stride+5) = N(i) * F(5); 
        }
    }

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
	) override {
		this->ComputeNF(U, Qi, detJ, F, state_x, state_w);
		detJ /= 4.0;  // because volume
	}

	/// This is needed so that it can be accessed by ChLoaderVolumeGravity
	virtual double GetDensity() override { return this->section->GetArea() * this->section->GetDensity(); }
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
