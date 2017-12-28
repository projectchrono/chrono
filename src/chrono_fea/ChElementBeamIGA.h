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

#include "chrono_fea/ChElementBeam.h"
#include "chrono_fea/ChBeamSection.h"
#include "chrono_fea/ChNodeFEAxyzrot.h"
#include "chrono/geometry/ChBasisToolsBspline.h"
#include "chrono/core/ChQuadrature.h"


namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Element of IGA type, with Timoshenko shear etc.
/// et.. etc.  (intro to write)
/// Note: each IGA element represents one "knot span" of the spline!


class  ChElementBeamIGA :   public ChElementBeam
                                    //public ChLoadableU,
                                    //public ChLoadableUVW,
                                    //public ChElementCorotational 
{
  protected:

    std::vector< std::shared_ptr<ChNodeFEAxyzrot> > nodes; // also "control points" 
    //std::vector< double > knots;
    ChVectorDynamic<> knots;

    int order;

    std::vector< double > Jacobian;  


    std::shared_ptr<ChBeamSectionAdvanced> section;

  public:
    ChElementBeamIGA() {
        order = 3;
        nodes.resize(4); // controllare se ordine = -> 2 nodi, 2 control points, o di più
        knots.Resize(8);
    }

    virtual ~ChElementBeamIGA() {}

    virtual int GetNnodes() override { return (int)nodes.size(); }
    virtual int GetNdofs() override { return GetNnodes() * 6; }
    virtual int GetNodeNdofs(int n) override { return 6; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return nodes[n]; }

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
    }

    virtual void SetNodesGenericOrder(std::vector<std::shared_ptr<ChNodeFEAxyzrot>> mynodes, std::vector<double> myknots, int myorder) {
		this->order = myorder;

        nodes.resize(myorder+1);
        for (int i= 0; i< mynodes.size(); ++i) {
            nodes[i] = mynodes[i];
        }
        knots.Resize(nodes.size()+myorder+1);
        for (int i= 0; i< myknots.size(); ++i) {
            knots(i) = myknots[i];
        }

        std::vector<ChVariables*> mvars;
        for (int i= 0; i< mynodes.size(); ++i) {
            mvars.push_back(&nodes[i]->Variables());
        }
        Kmatr.SetVariables(mvars);
    }



    //
    // FEM functions
    //

    /// Set the section & material of beam element .
    /// It is a shared property, so it can be shared between other beams.
    void SetSection(std::shared_ptr<ChBeamSectionAdvanced> my_material) { section = my_material; }
    /// Get the section & material of the element
    std::shared_ptr<ChBeamSectionAdvanced> GetSection() { return section; }


    virtual void Update() override {
        // parent class update:
        ChElementGeneric::Update();

    };



    /// Setup. Precompute mass and matrices that do not change during the
    /// simulation.
    /// In particular, compute the arc-length parametrization.

    virtual void SetupInitial(ChSystem* system) override {
        assert(section);

        this->length=0;

        // get two values of absyssa at extreme of span
        double u1 = knots(order); 
		double u2 = knots(knots.GetRows() - order - 1);

        double c1 = (u2 - u1) / 2;
        double c2 = (u2 + u1) / 2;

        //int int_order = (int)std::ceil((this->order+1.0)/2.0);
        int int_order = 2;

        this->Jacobian.resize(int_order);

        for (int ig = 0; ig < int_order; ++ig) {

            // absyssa in typical -1,+1 range:
            double eta = ChQuadrature::GetStaticTables()->Lroots[int_order-1][ig];
            // absyssa in span range:
            double u = (c1 * eta + c2); 
            // scaling = gauss weight * change of range:
            double w = ChQuadrature::GetStaticTables()->Weight[int_order-1][ig] * c1;

            // compute the basis functions N(u) at given u:
            int nspan = order;
            /*
		    ChVectorDynamic<> knotU((int)knots.size());
		    for (int i = 0; i< knots.size(); ++i) {
			    this->knotU(i) = knots[i];
            }
            */
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
            this->Jacobian[ig] = dr0.Length(); // J = |dr0/du|

            this->length += w*this->Jacobian[ig];
        } 

        this->mass = this->length * this->section->density;
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

        //
        // The K stiffness matrix of this element span:
        //

        // ChMatrixDynamic<> K(12, 12); // only if linear...
        ChMatrixDynamic<> K(6 * (int)nodes.size(), 6 * (int)nodes.size()); 

        // ***KASSEM*** calcola la matrice K (per il singolo knot span) 
        // valutandola per gli n punti di collocation (che stanno in questo knot span)
        // 
        // 
        //K(0,0) = 1222121;
        // meglio: credo serva un ciclo for tipo: (e forse questo all'interno di un ciclo sui punti di collocazione?)
        //
        // 
        //
        for (int i = 0; i< nodes.size(); ++i) {
            // K
            //...
        }

        // ...
        //. ...
        // ...
        // ...








        // finally, store K (and R) into H:

        double mkrfactor = Kfactor + Rfactor * this->section->rdamping;

        K.MatrScale(mkrfactor);

        H.PasteMatrix(K, 0, 0);  // because [R] = r*[K] , so kf*[K]+rf*[R] = (kf+rf*r)*[K]




        //
        // The M mass matrix of this element span:
        //

        ChMatrixDynamic<> Mloc(6 * (int)nodes.size(), 6 * (int)nodes.size());

        double lmass = mass /(double)nodes.size();
        double lineryz = (1. / 50.) * mass * pow(length, 2);  // note: 1/50 can be even less (this is 0 in many texts)
        double linerx = (1. / 2.) * length * section->GetDensity() * (section->GetIyy() + section->GetIzz());  // boh..

        for (int i = 0; i< nodes.size(); ++i) {
            int stride = i*6;
            Mloc(stride+0, stride+0) += Mfactor * lmass;  // node A x,y,z
            Mloc(stride+1, stride+1) += Mfactor * lmass;
            Mloc(stride+2, stride+2) += Mfactor * lmass;
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

        // shortcuts:
        double Area = section->Area;
        double E = section->E;
        double Izz = section->Izz;
        double Iyy = section->Iyy;
        double G = section->G;
        double Jpolar = section->J;
        double Ky = section->Ks_y;
        double Kz = section->Ks_z;

        // get two values of absyssa at extreme of span
        double u1 = knots(order); 
		double u2 = knots(knots.GetRows() - order - 1);

        double c1 = (u2 - u1) / 2;
        double c2 = (u2 + u1) / 2;

        // zeroes the Fi accumulator
        Fi.Reset();

        // Do quadrature over the Gauss points, exact for polynomial of order p= 2n-1; ie n= (p+1)/2 minimum

        //int int_order = (int)std::ceil((this->order+1.0)/2.0);
        int int_order = 2;

        for (int ig = 0; ig < int_order; ++ig) {

            // absyssa in typical -1,+1 range:
            double eta = ChQuadrature::GetStaticTables()->Lroots[int_order-1][ig];
            // absyssa in span range:
            double u = (c1 * eta + c2);
            // scaling = gauss weight * change of range:
            double w = ChQuadrature::GetStaticTables()->Weight[int_order-1][ig] * c1;

            // compute the basis functions N(u) at given u:
            int nspan = order;
            /*
		    ChVectorDynamic<> knotU((int)knots.size());
		    for (int i = 0; i< knots.size(); ++i) {
			    knotU(i) = knots[i];
            }
            */
            ChMatrixDynamic<> N(2,(int)nodes.size()); // row n.0 contains N, row n.1 contains dN/du

            geometry::ChBasisToolsBspline::BasisEvaluateDeriv(
                   this->order,  
                   nspan,
                   u,  
                   knots, 
                   N);           ///< here return N and dN/du 
            
            // interpolate rotation of section at given u, to compute R.
            // Note: this is approximate, in fact quaternion must be normalized at the end.
            // A more precise method: use quaternion splines, as in Kim,Kim and Shin, 1995 paper.
            ChQuaternion<> qR = QNULL;
            for (int i = 0 ; i< nodes.size(); ++i) {
                qR += nodes[i]->coord.rot * N(0,i);
            }
            qR.Normalize();
            
            // compute the 3x3 rotation matrix R equivalent to quaternion above
            ChMatrix33<> R(qR);

            // compute abs. spline gradient r'  = dr/ds
            ChVector<> dr; 
            for (int i = 0 ; i< nodes.size(); ++i) {
                dr += nodes[i]->coord.pos * N(1,i);  // dr/dt = N_i'*r_i
            }
            // (note r'= dr/ds = dr/du * 1/J   where J computed in SetupInitial)
            dr *=  1.0/this->Jacobian[ig];

            // compute abs spline rotation gradient q' = dq/ds 
            // But.. easier to compute local gradient of spin vector a' = da/ds
            //ChQuaternion<> dq;
            ChQuaternion<> q_delta;
            ChVector<> da; 
            ChVector<> delta_rot_dir;
            double delta_rot_angle;
            for (int i = 0 ; i< nodes.size(); ++i) {
                //dq += nodes[i]->coord.rot * N(1,i);  // dq/dt = N_i'*q_i  (in abs reference)
                q_delta = qR.GetConjugate() % nodes[i]->coord.rot;
                q_delta.Q_to_AngAxis(delta_rot_angle, delta_rot_dir);  // a_i = dir_i*angle_i (in spline local reference)
                if (delta_rot_angle > CH_C_PI)
                    delta_rot_angle -= CH_C_2PI;  // no 0..360 range, use -180..+180
                da += delta_rot_dir * delta_rot_angle * N(1,i);  // da/dt = N_i'*a_i
            }
            // (note a'= da/ds = da/dt * 1/J   where J computed in SetupInitial)
            da *=  1.0/this->Jacobian[ig];

            // compute local epsilon strain:  strain_e= R^t * r' - {1, 0, 0}
            ChVector<> strain_e = R.MatrT_x_Vect(dr) - VECT_X;

            // compute local curvature strain:  strain_k= 2*[F(q*)(+)] * q' = 2*[F(q*)(+)] * N_i'*q_i = R^t * a' = a'_local
            ChVector<> strain_k = da;

            // compute stress n  (local cut forces) = C * (strain_e - strain_e0)
            ChVector<> stress_n;
            stress_n.x() = strain_e.x() * E * Area;
            stress_n.y() = strain_e.y() * Ky * G * Area;
            stress_n.z() = strain_e.z() * Kz * G * Area;

            // compute stress_m  (local cut torque) = D * (strain_k - strain_k0)
            ChVector<> stress_m;
            stress_m.x() = strain_k.x() * G * Jpolar;
            stress_m.y() = strain_k.y() * E * Iyy;
            stress_m.z() = strain_k.z() * E * Izz;

            // compute internal force, in generalized coordinates:

            ChVector<> stress_n_abs = R * stress_n;
            ChVector<> stress_m_abs = R * stress_m;

            for (int i = 0; i< nodes.size(); ++i) {

                // -Force_i = w * N' * R * C * (strain_e - strain_e0)
                ChVector<> Force_i = stress_n_abs* N(1,i) * -w;
			    Fi.PasteSumVector(Force_i, i *6, 0);

                // -Torque_i = w * R_i^t * N'^t * R * D * (strain_k - strain_k0) + 
                //           + w * R_i^t * N^t  * skew(r')^t *  R * C * (strain_e - strain_e0)
                ChVector<> Torque_i = nodes[i]->coord.rot.RotateBack(
                    stress_m_abs * N(1,i) * -w 
                    - Vcross(dr, stress_n_abs) *  N(0,i) * -w
                    );
                Fi.PasteSumVector(Torque_i, 3+ i *6, 0);
            }

            //GetLog() << "     gp n." << ig <<   "  J=" << this->Jacobian[ig] << "   strain_e= " << strain_e << "\n";
            //GetLog() << "                    stress_n= " << stress_n << "\n";
        } 
        
        
        //***KASSEM** se vuoi fare del test/debug, è comodo aggiungere dei GetLog() per stampare su schermo dei dati che vuoi controllare, es: 
        /*
        GetLog() << "\nInternal forces, F and M for each node: \n";
        for (int i = 0; i < nodes.size(); i++) {
            GetLog() << "\n  at node " << i << " is: ";
            for (int c = 0; c < 6; c++) {
                GetLog() << Fi(i*6 + c) << "  ";
            }
        }
        GetLog() << "\n";
        */
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

    /// Gets the absolute xyz position of a point on the beam line,
    /// and the absolute rotation of section plane, at abscissa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Note, 'displ' is the displ.state of 2 nodes, ex. get it as GetStateBlock()
    /// Results are corotated (expressed in world reference)
    virtual void EvaluateSectionFrame(const double eta,
                                      ChVector<>& point,
                                      ChQuaternion<>& rot) override {
        // compute parameter in knot space from eta-1..+1
		
		double u1 = knots(order); // extreme of span
		double u2 = knots(knots.GetRows() - order - 1);
		double u = u1 + ((eta + 1) / 2.0)*(u2 - u1);
		int nspan = order;
        /*
		ChVectorDynamic<> knotU((int)knots.size());
		for (int i = 0; i< knots.size(); ++i) {
			knotU(i) = knots[i];
        }
        */
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
    /// Note, 'displ' is the displ.state of 2 nodes, ex. get it as GetStateBlock().
    /// Results are not corotated, and are expressed in the reference system of beam.
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
 
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
