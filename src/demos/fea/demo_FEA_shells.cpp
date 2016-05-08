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

//
//   Demo code about
//
//     - FEA for 3D beams of 'cable' type (ANCF gradient-deficient beams)

// Include some headers used by this tutorial...

#include "chrono/lcp/ChLcpIterativeMINRES.h"
#include "chrono/lcp/ChLcpIterativePMINRES.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/timestepper/ChTimestepper.h"

#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChElementShellEANS4.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_mkl/ChLcpMklSolver.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_postprocess/ChGnuPlot.h"
#include <vector>

// Remember to use the namespace 'chrono' because all classes 
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace chrono::postprocess;
using namespace irr;

int main(int argc, char* argv[]) {
    // Create a Chrono::Engine physical system
    ChSystem my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Shells FEA", core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0.f, 6.0f, -10.f));

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    //my_system.Set_G_acc(VNULL); or 
    my_mesh->SetAutomaticGravity(false);


   
    std::shared_ptr<ChNodeFEAxyzrot> nodePlotA;
    std::shared_ptr<ChNodeFEAxyzrot> nodePlotB;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodesLoad;

    //
    // Add an EANS SHELL:
    //

    if (false)
    {
        double shell_thickness = 0.02;
        double shell_L = 1;
        double shell_W = 1;

        // Create a material 
        double rho = 0.0;
        double E = 1e9;
        double nu = 0.4; 
        auto mat = std::make_shared<ChMaterialShellEANS>(shell_thickness,
                                                         rho, 
                                                         E, 
                                                         nu,
                                                         1.00,
                                                         0.01);

        // Create the nodes (each with position & normal to shell)
        
        auto hnodeeans1 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(0, 0, 0)));
        auto hnodeeans2 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(shell_L, 0, 0)));
        auto hnodeeans3 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(shell_L, shell_W, 0)));
        auto hnodeeans4 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(0, shell_W, 0 )));
        
        nodePlotA = hnodeeans3;

        double mn = 10.5;
        hnodeeans1->GetInertia().FillDiag(1./12.*pow((shell_L/2.),3)*mn);
        hnodeeans2->GetInertia().FillDiag(1./12.*pow((shell_L/2.),3)*mn);
        hnodeeans3->GetInertia().FillDiag(1./12.*pow((shell_L/2.),3)*mn);
        hnodeeans4->GetInertia().FillDiag(1./12.*pow((shell_L/2.),3)*mn);
        hnodeeans1->SetMass(mn);
        hnodeeans1->SetMass(mn);
        hnodeeans1->SetMass(mn);
        hnodeeans1->SetMass(mn);
        my_mesh->AddNode(hnodeeans1);
        my_mesh->AddNode(hnodeeans2);
        my_mesh->AddNode(hnodeeans3);
        my_mesh->AddNode(hnodeeans4);

        hnodeeans1->SetFixed(true);
        hnodeeans2->SetFixed(true);

        // Create the element

        auto elementeans = std::make_shared<ChElementShellEANS4>();
        my_mesh->AddElement(elementeans);

        // Set its nodes
        elementeans->SetNodes(hnodeeans1, hnodeeans2, hnodeeans3, hnodeeans4);

        // Set element dimensions
        //elementeans->SetDimensions(shell_L, shell_W); // not needed, already set at initialization from initial pos of nodes

        // Add a single layers with a fiber angle of 0 degrees.
        elementeans->AddLayer(shell_thickness, 0 * CH_C_DEG_TO_RAD, mat);

        // Set other element properties
        elementeans->SetAlphaDamp(0.02);    // Structural damping for this element

        // Apply a lumped force to a node:
       // hnodeeans3->SetPos(hnodeeans3->GetPos()+ChVector<>(0, 0, 0.01));
       // hnodeeans4->SetPos(hnodeeans4->GetPos()+ChVector<>(0, 0, 0.01));
       // hnodeeans3->SetForce(ChVector<>(0, 3000, 0));
       // hnodeeans4->SetForce(ChVector<>(0, 3000, 0));
       //hnodeeans3->SetForce(ChVector<>(0, 0, 100));
       //hnodeeans4->SetForce(ChVector<>(0, 0, 100));
       // hnodeeans3->SetForce(ChVector<>(0, 50, 0));
       // hnodeeans4->SetForce(ChVector<>(0, 50, 0));
       //hnodeeans3->SetTorque(ChVector<>(0.2, 0, 0));
       //hnodeeans4->SetTorque(ChVector<>(0.2, 0, 0));
       // hnodeeans4->SetMass(2000);
 
        
    }

    double l0;
    ChFunction_Recorder ref_X;
    ChFunction_Recorder ref_Y;

    //
    // Add an EANS SHELL cantilever:
    //

    if (false)
    {
        double rect_thickness = 0.10;
        double rect_L = 12.0;
        double rect_W = 1.0;

        // Create a material 
        double rho = 0.0;
        double E = 1.2e6;
        double nu = 0.0; 
        auto mat = std::make_shared<ChMaterialShellEANS>(rect_thickness,
                                                         rho, 
                                                         E, 
                                                         nu,
                                                         1.0,
                                                         0.01);

        // Create the nodes (each with position & normal to shell)
        double node_density=0.0001;
        
        int nels_L = 15;
        int nels_W = 1;
        std::vector<std::shared_ptr<ChElementShellEANS4>> elarray(nels_L*nels_W);
        std::vector<std::shared_ptr<ChNodeFEAxyzrot>>     nodearray((nels_L+1)*(nels_W+1));
        std::vector<std::shared_ptr<ChNodeFEAxyzrot>>     nodes_start(nels_W+1);
        std::vector<std::shared_ptr<ChNodeFEAxyzrot>>     nodes_end(nels_W+1);

        for (int il = 0; il<= nels_L; ++il) {
            for (int iw = 0; iw<= nels_W; ++iw) {
                // Make nodes
                ChVector<> nodepos(rect_L*((double)il/(double)nels_L),   rect_W*((double)iw/(double)nels_W),  0);
                //ChQuaternion<> noderot(ChQuaternion<>(ChRandom(), ChRandom(), ChRandom(), ChRandom()).GetNormalized());
                ChQuaternion<> noderot(QUNIT);
                ChFrame<> nodeframe(nodepos,noderot);

                auto mnode = std::make_shared<ChNodeFEAxyzrot>(nodeframe);
                my_mesh->AddNode(mnode);

                double mn = node_density*(rect_L*rect_W*rect_thickness)/(nels_L*nels_W); // approx
                mnode->GetInertia().FillDiag(1./12.*pow(((rect_L/nels_L)/2.),3)*mn); // approx
                mnode->SetMass(mn);

                nodearray[il*(nels_W+1) + iw] = mnode;

                if (il==0)
                    nodes_start[iw] = mnode;
                if (il==nels_L)
                    nodes_end[iw] = mnode;

                // Make elements
                if (il>0 && iw>0) {
                    auto melement = std::make_shared<ChElementShellEANS4>();
                    my_mesh->AddElement(melement);
                    melement->SetNodes(
                        nodearray[(il-1)*(nels_W+1) + (iw-1)], 
                        nodearray[(il  )*(nels_W+1) + (iw-1)],
                        nodearray[(il  )*(nels_W+1) + (iw  )],
                        nodearray[(il-1)*(nels_W+1) + (iw  )]
                        );
                    melement->AddLayer(rect_thickness, 0 * CH_C_DEG_TO_RAD, mat);
                    melement->SetAlphaDamp(0.001);   
                    elarray[(il-1)*(nels_W) + (iw-1)] = melement;
                }
            }
        }
        nodesLoad = nodes_end;
        nodePlotA = nodes_end.front();
        nodePlotB = nodes_end.back();

        for (auto mstartnode : nodes_start) {
            mstartnode->SetFixed(true);
        }
        /*
        // applied shear
        l0 = 4;
        ref_Y.AddPoint(0.10,1.309); ref_X.AddPoint(0.40,0.103);
        ref_Y.AddPoint(0.20,2.493); ref_X.AddPoint(0.80,0.381);
        ref_Y.AddPoint(0.30,3.488); ref_X.AddPoint(1.20,0.763);
        ref_Y.AddPoint(0.40,4.292); ref_X.AddPoint(1.60,1.184);
        ref_Y.AddPoint(0.50,4.933); ref_X.AddPoint(2.00,1.604);
        ref_Y.AddPoint(0.60,5.444); ref_X.AddPoint(2.40,2.002);
        ref_Y.AddPoint(0.70,5.855); ref_X.AddPoint(2.80,2.370);
        ref_Y.AddPoint(0.80,6.190); ref_X.AddPoint(3.20,2.705);
        ref_Y.AddPoint(0.90,6.467); ref_X.AddPoint(3.60,3.010);
        ref_Y.AddPoint(1.00,6.698); ref_X.AddPoint(4.00,3.286);
        */
        
        // applied torque
        l0 = 50*CH_C_PI/3;
        for (double t= 0.05; t<=1; t+=0.05) {
            ref_X.AddPoint(t, -12* ( (1./(CH_C_2PI*t))*(sin(CH_C_2PI*t)) -1) ); 
            ref_Y.AddPoint(t, 12* (1./(CH_C_2PI*t))*(1.-cos(CH_C_2PI*t)) );
        }
        
    }

    //
    // Add a SLIT ANNULAR PLATE:
    //

    if (true)
    {
        double plate_thickness = 0.03;
        double plate_Ri = 6;
        double plate_Ro = 10;

        // Create a material 
        double rho = 0.0;
        double E = 21e6;
        double nu = 0.0; 
        auto mat = std::make_shared<ChMaterialShellEANS>(plate_thickness,
                                                         rho, 
                                                         E, 
                                                         nu,
                                                         1.0,
                                                         0.001);

        // Create the nodes (each with position & normal to shell)
        double node_density=0.0001;
        
        int nels_U = 30;
        int nels_W = 6;
        double arc = CH_C_2PI;// *0.1;
        std::vector<std::shared_ptr<ChElementShellEANS4>> elarray(nels_U*nels_W);
        std::vector<std::shared_ptr<ChNodeFEAxyzrot>>     nodearray((nels_U+1)*(nels_W+1));
        std::vector<std::shared_ptr<ChNodeFEAxyzrot>>     nodes_start(nels_W+1);
        std::vector<std::shared_ptr<ChNodeFEAxyzrot>>     nodes_end(nels_W+1);

        for (int iu= 0; iu<= nels_U; ++iu) {
            for (int iw = 0; iw<= nels_W; ++iw) {
                // Make nodes
                double u = ((double)iu/(double)nels_U);
                double w = ((double)iw/(double)nels_W);
                ChVector<> nodepos(
                    (plate_Ri+(plate_Ro-plate_Ri)*w) * cos(u*arc), 
                    0,  
                    (plate_Ri+(plate_Ro-plate_Ri)*w) * sin(u*arc));
                //ChMatrix33<> nr;
                //nr.Set_A_Xdir (ChVector<>(cos(u*CH_C_2PI), 0, sin(u*CH_C_2PI)),VECT_Y);
                //ChQuaternion<> noderot(nr.Get_A_quaternion());
                //ChQuaternion<> noderot(ChQuaternion<>(ChRandom(), ChRandom(), ChRandom(), ChRandom()).GetNormalized());//QUNIT);
                ChQuaternion<> noderot(QUNIT);
                ChFrame<> nodeframe(nodepos, noderot);

                auto mnode = std::make_shared<ChNodeFEAxyzrot>(nodeframe);
                my_mesh->AddNode(mnode);

                double mn = node_density*(plate_Ro*CH_C_2PI*(plate_Ro-plate_Ri)*plate_thickness)/(nels_U*nels_W); // approx
                mnode->GetInertia().FillDiag(0); //mnode->GetInertia().FillDiag(1./12.*pow((((plate_Ro-plate_Ri)/nels_W)/2.),3)*mn); // approx
                mnode->SetMass(0.000); //mn);

                nodearray[iu*(nels_W+1) + iw] = mnode;

                if (iu==0)
                    nodes_start[iw] = mnode;
                if (iu==nels_U)
                    nodes_end[iw] = mnode;

                // Make elements
                if (iu>0 && iw>0) {
                    auto melement = std::make_shared<ChElementShellEANS4>();
                    my_mesh->AddElement(melement);
                    
                    melement->SetNodes(
                        nodearray[(iu  )*(nels_W+1) + (iw  )],
                        nodearray[(iu-1)*(nels_W+1) + (iw  )],
                        nodearray[(iu-1)*(nels_W+1) + (iw-1)], 
                        nodearray[(iu  )*(nels_W+1) + (iw-1)]
                        );
                      
                    /*    
                    melement->SetNodes( // not working well..
                        nodearray[(iu  )*(nels_W+1) + (iw-1)],
                        nodearray[(iu  )*(nels_W+1) + (iw  )],
                        nodearray[(iu-1)*(nels_W+1) + (iw  )],
                        nodearray[(iu-1)*(nels_W+1) + (iw-1)] 
                        );
                    */  
                    melement->AddLayer(plate_thickness, 0 * CH_C_DEG_TO_RAD, mat);
                    melement->SetAlphaDamp(0.0);   
                    elarray[(iu-1)*(nels_W) + (iw-1)] = melement;
                }
            }
        }

        nodesLoad = nodes_end;
        nodePlotA = nodes_end.front();
        nodePlotB = nodes_end.back();

        for (auto mstartnode : nodes_start) {
            mstartnode->SetFixed(true);
        }
        /*
        auto mtruss = std::make_shared<ChBody>();
        mtruss->SetBodyFixed(true);
        my_system.Add(mtruss);
        for (auto mendnode : nodes_end) {
            auto mlink = std::make_shared<ChLinkMateGeneric>(false,false,true, false,false,false);
            mlink->Initialize(mendnode, mtruss, false, mtruss->GetFrame_REF_to_abs(), mtruss->GetFrame_COG_to_abs());
            my_system.Add(mlink);
        }
        */

        l0= 0.8*4;
        ref_X.AddPoint(0.025,1.305);  ref_Y.AddPoint(0.025,1.789);
        ref_X.AddPoint(0.10,4.277);   ref_Y.AddPoint(0.10,5.876);
        ref_X.AddPoint(0.20,6.725);   ref_Y.AddPoint(0.20,9.160);
        ref_X.AddPoint(0.30,8.340);   ref_Y.AddPoint(0.30,11.213);
        ref_X.AddPoint(0.40,9.529);   ref_Y.AddPoint(0.40,12.661);
        ref_X.AddPoint(0.50,10.468);  ref_Y.AddPoint(0.50,13.768);
        ref_X.AddPoint(0.60,11.257);  ref_Y.AddPoint(0.60,14.674);
        ref_X.AddPoint(0.70,11.970);  ref_Y.AddPoint(0.70,15.469);
        ref_X.AddPoint(0.80,12.642);  ref_Y.AddPoint(0.80,16.202);
        ref_X.AddPoint(0.9,13.282);  ref_Y.AddPoint(0.90,16.886);
        ref_X.AddPoint(1.00,13.891);  ref_Y.AddPoint(1.00,17.528);
    }
    


    // ==Asset== attach a visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChTriangleMeshShape
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colours as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a coloured ChTriangleMeshShape).
    // Do not forget AddAsset() at the end!

    auto mvisualizeshellA = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizeshellA->SetSmoothFaces(true);
	mvisualizeshellA->SetWireframe(true);
	my_mesh->AddAsset(mvisualizeshellA);

    auto mvisualizeshellB = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizeshellB->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizeshellB->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizeshellB->SetSymbolsThickness(0.01);
    my_mesh->AddAsset(mvisualizeshellB);
/*
    auto mvisualizeshellC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizeshellC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizeshellC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    mvisualizeshellC->SetSymbolsThickness(0.05);
    mvisualizeshellC->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizeshellC);
*/

    // ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    // in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
    // If you need a finer control on which item really needs a visualization proxy in
    // Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

    application.AssetBindAll();

    // ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    // that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

    application.AssetUpdateAll();

    // Mark completion of system construction
    my_system.SetupInitial();

    //
    // THE SOFT-REAL-TIME CYCLE
    //
    // Change solver to MKL
    ChLcpMklSolver* mkl_solver_stab = new ChLcpMklSolver;
    ChLcpMklSolver* mkl_solver_speed = new ChLcpMklSolver;
    my_system.ChangeLcpSolverStab(mkl_solver_stab);
    my_system.ChangeLcpSolverSpeed(mkl_solver_speed);
	mkl_solver_stab->SetSparsityPatternLock(true);
	mkl_solver_speed->SetSparsityPatternLock(true);
    /*
    my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_MINRES); // <- NEEDED THIS or Matlab or MKL solver
	my_system.SetIterLCPwarmStarting(true); // this helps a lot to speedup convergence in this class of problems
	my_system.SetIterLCPmaxItersSpeed(200);
	my_system.SetIterLCPmaxItersStab(200);
	my_system.SetTolForce(1e-13);
	chrono::ChLcpIterativeMINRES* msolver = (chrono::ChLcpIterativeMINRES*)my_system.GetLcpSolverSpeed();
	msolver->SetVerbose(false);
	msolver->SetDiagonalPreconditioning(true);
    */

    // Change type of integrator:
    my_system.SetIntegrationType(chrono::ChSystem::INT_EULER_IMPLICIT);  
    //my_system.SetIntegrationType(chrono::ChSystem::INT_EULER_IMPLICIT_LINEARIZED);  // fast, less precise
 
    application.SetTimestep(0.01);
    application.SetPaused(true);
    my_system.Setup();
    my_system.Update();
    
    ChFunction_Recorder rec_X;
    ChFunction_Recorder rec_Y;
    

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        // .. draw also a grid
        ChIrrTools::drawGrid(application.GetVideoDriver(), 1, 1);

        // ...update load at end nodes, as simple lumped nodal forces
        double load_scale = my_system.GetChTime()*0.1;
        double loadF = l0 * load_scale;
        for (auto mendnode : nodesLoad) {
            mendnode->SetForce(ChVector<>(0, loadF, 0) * (1./ (double)nodesLoad.size()) );
            //mendnode->SetTorque(ChVector<>(0, loadF, 0) * (1./ (double)nodesLoad.size()) );
        }

        application.DoStep();

        if(!application.GetPaused() ) {
            rec_Y.AddPoint( load_scale, nodePlotA->GetPos().y);
            rec_X.AddPoint( load_scale, nodePlotB->GetPos().y);
        }

        application.EndScene();

        if (load_scale > 1)
            application.GetDevice()->closeDevice();
        
    }

    ChGnuPlot mplot("__cantilever.gpl");
    mplot.SetGrid(false, 1, ChColor(0.8,0.8,0.8));
    mplot.SetLabelX("Torque T/T0");
    mplot.SetLabelY("Tip displacement [m]");
    mplot << "set key left top";
    mplot.Plot(rec_Y, "W", " with lines lt -1 lc rgb'#00AAEE' ");
    mplot.Plot(rec_X, "-U", " with lines lt -1 lc rgb'#AA00EE' ");
    mplot.Plot(ref_Y, "W ref.", "pt 4 ps 1.4");
    mplot.Plot(ref_X, "-U ref.", "pt 4 ps 1.4");

    return 0;
}


