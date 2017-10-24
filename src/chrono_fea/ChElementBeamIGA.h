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
    std::vector< double > knots;
    int order;

    std::shared_ptr<ChBeamSectionAdvanced> section;

    double cose; // esempio..

  public:
    ChElementBeamIGA() {
        order = 1;
        nodes.resize(2); // controllare se ordine = -> 2 nodi, 2 control points, o di più
        knots.resize(4);

    }

    virtual ~ChElementBeamIGA() {}

    virtual int GetNnodes() override { return (int)nodes.size(); }
    virtual int GetNdofs() override { return GetNnodes() * 6; }
    virtual int GetNodeNdofs(int n) override { return 6; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return nodes[n]; }

    virtual void SetNodesLinear(std::shared_ptr<ChNodeFEAxyzrot> nodeA, std::shared_ptr<ChNodeFEAxyzrot> nodeB, double knotA1, double knotA2, double knotB1, double knotB2) {
        nodes.resize(2);
        nodes[0] = nodeA;
        nodes[1] = nodeB;
        knots.resize(4);
        knots[0] = knotA1;
        knots[1] = knotA2;
        knots[2] = knotB1;
        knots[3] = knotB2;
        std::vector<ChVariables*> mvars;
        mvars.push_back(&nodes[0]->Variables());
        mvars.push_back(&nodes[1]->Variables());
        Kmatr.SetVariables(mvars);
    }

    virtual void SetNodesGenericOrder(std::vector<std::shared_ptr<ChNodeFEAxyzrot>> mynodes, std::vector<double> myknots, int myorder) {
        nodes.resize(myorder+1);
        for (int i= 0; i< mynodes.size(); ++i) {
            nodes[i] = mynodes[i];
        }
        knots.resize(nodes.size()+myorder+1);
        for (int i= 0; i< myknots.size(); ++i) {
            knots[i] = myknots[i];
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
    /// simulation, such as the local tangent stiffness Kl of each element, if needed, etc.

    virtual void SetupInitial(ChSystem* system) override {
        assert(section);

        // Kassem:*** metti cose da calcolare una volta per tutte all'inizio
        // this->length = ... 
        // this->mass = ....
        this->length = 1; //***TEST*** ***TODO***
        this->mass = this->length * this->section->density;
        this->cose = 0;
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
        assert((H.GetRows() == 12) && (H.GetColumns() == 12));
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

        //***KASSEM**** calcolare i valori in Fi: !!!!!!!!!!!!!!!!!!!!!!!
        
        // Fi(0) = ....;
        // Fi(1) = ....;
        // Fi(2) = ....;
        //
        // meglio: credo serva un ciclo for tipo: (e forse questo all'interno di un ciclo sui punti di collocazione?)
        // 
        //
        for (int i = 0; i< nodes.size(); ++i) {
            // Fi(i*6 + 0) =
            // Fi(i*6 + 1) =
            //...
        }
        
        //***KASSEM** se vuoi fare del test/debug, è comodo aggiungere dei GetLog() per stampare su schermo dei dati che vuoi controllare, es: 
        GetLog() << "\nInternal forces, F and M for each node: \n";
        for (int i = 0; i < nodes.size(); i++) {
            GetLog() << "\n  at node " << i << " is: ";
            for (int c = 0; c < 6; c++)
                GetLog() << Fi(i*6 + c) << "  ";
        }
        GetLog() << "\n";
        
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
        double tau1 = knots[order]; // extreme of span
        double tau2 = knots[knots.size()-order-1];
        double u = tau1 + ((eta+1)/2.0)*(tau2-tau1);
        int nspan = order;
        ChVectorDynamic<> knotU((int)knots.size());
        for (int i = 0; i< knots.size(); ++i) {
            knotU(i) = knots[i];
        }
        ChVectorDynamic<> N((int)nodes.size());

        geometry::ChBasisToolsBspline::BasisEvaluate(
               this->order,  
               nspan,
               u,  
               knotU, 
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
