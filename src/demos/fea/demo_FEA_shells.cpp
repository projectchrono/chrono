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
//     - FEA for shells of Reissner 6-field type


#include <vector>

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/solver/ChSolverPMINRES.h"
#include "chrono/timestepper/ChTimestepper.h"

#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChElementShellReissner4.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_fea/ChRotUtils.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_mkl/ChSolverMKL.h"
#include "chrono_postprocess/ChGnuPlot.h"

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


    ChFunction_Recorder ref_X;
    ChFunction_Recorder ref_Y;

    ChVector<> load_torque;
    ChVector<> load_force;


    //
    // BENCHMARK n.1
    //
    // Add an EANS SHELL cantilever:
    //

    if (false)  // set as 'true' to execute this 
    {
        double rect_thickness = 0.10;
        double rect_L = 10.0;
        double rect_W = 1;

        // Create a material 
        double rho = 0.0;
        double E = 1.2e6;
        double nu = 0.0; 

        auto mat = std::make_shared<ChMaterialShellReissnerIsothropic>(
                                                         rho, 
                                                         E, 
                                                         nu,
                                                         1.0,
                                                         0.01);
        

        // Create the nodes 
        
        int nels_L = 12;
        int nels_W = 1;
        std::vector<std::shared_ptr<ChElementShellReissner4>> elarray(nels_L*nels_W);
        std::vector<std::shared_ptr<ChNodeFEAxyzrot>>     nodearray((nels_L+1)*(nels_W+1));
        std::vector<std::shared_ptr<ChNodeFEAxyzrot>>     nodes_start(nels_W+1);
        std::vector<std::shared_ptr<ChNodeFEAxyzrot>>     nodes_end(nels_W+1);

        for (int il = 0; il<= nels_L; ++il) {
            for (int iw = 0; iw<= nels_W; ++iw) {
                // Make nodes
                ChVector<> nodepos(rect_L*((double)il/(double)nels_L), 0,  rect_W*((double)iw/(double)nels_W));
                ChQuaternion<> noderot(QUNIT);

                ChFrame<> nodeframe(nodepos,noderot);

                auto mnode = std::make_shared<ChNodeFEAxyzrot>(nodeframe);
                my_mesh->AddNode(mnode);

                mnode->GetInertia().FillDiag(0); // approx
                mnode->SetMass(0);

                nodearray[il*(nels_W+1) + iw] = mnode;

                if (il==0)
                    nodes_start[iw] = mnode;
                if (il==nels_L)
                    nodes_end[iw] = mnode;

                // Make elements
                if (il>0 && iw>0) {
                    auto melement = std::make_shared<ChElementShellReissner4>();
                    my_mesh->AddElement(melement);
                    
                    melement->SetNodes( 
                        nodearray[(il-1)*(nels_W+1) + (iw-1)], 
                        nodearray[(il  )*(nels_W+1) + (iw-1)],
                        nodearray[(il  )*(nels_W+1) + (iw  )],
                        nodearray[(il-1)*(nels_W+1) + (iw  )]
                        );

                    melement->AddLayer(rect_thickness, 0 * CH_C_DEG_TO_RAD, mat);
                    melement->SetAlphaDamp(0.0);   
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
        
        // applied load
        //load_force = ChVector<>(200000,0, 20000);
        load_force = ChVector<>(0, 4, 0);
        //load_torque = ChVector<>(0, 0, 50*CH_C_PI/3.0);

        // reference solution for (0, 4, 0) shear to plot 
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
        
    }


    // 
    // BENCHMARK n.2
    //
    // Add a SLIT ANNULAR PLATE:
    //

    if (true)  // set as 'true' to execute this
    {
        double plate_thickness = 0.03;
        double plate_Ri = 6;
        double plate_Ro = 10;

        // Create a material 
        double rho = 0.0;
        double E = 21e6;
        double nu = 0.0; 

        auto mat = std::make_shared<ChMaterialShellReissnerIsothropic>(
                                                         rho, 
                                                         E, 
                                                         nu,
                                                         1.0,
                                                         0.01);

        // Create the nodes  
        
        int nels_U = 60;
        int nels_W = 10;
        double arc = CH_C_2PI *1;
        std::vector<std::shared_ptr<ChElementShellReissner4>> elarray(nels_U*nels_W);
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
                ChQuaternion<> noderot(QUNIT);
                ChFrame<> nodeframe(nodepos, noderot);

                auto mnode = std::make_shared<ChNodeFEAxyzrot>(nodeframe);
                my_mesh->AddNode(mnode);

                mnode->GetInertia().FillDiag(0.0); 
                mnode->SetMass(0); 
                
                nodearray[iu*(nels_W+1) + iw] = mnode;

                if (iu==0)
                    nodes_start[iw] = mnode;
                if (iu==nels_U)
                    nodes_end[iw] = mnode;

                // Make elements
                if (iu>0 && iw>0) {
                    auto melement = std::make_shared<ChElementShellReissner4>();
                    my_mesh->AddElement(melement);
                    
                    melement->SetNodes(
                        nodearray[(iu  )*(nels_W+1) + (iw  )],
                        nodearray[(iu-1)*(nels_W+1) + (iw  )],
                        nodearray[(iu-1)*(nels_W+1) + (iw-1)], 
                        nodearray[(iu  )*(nels_W+1) + (iw-1)]
                        );

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

        load_force  = ChVector<>(0, 0.8*4, 0);
        load_torque = VNULL;

        // reference solution to plot
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
    
    // 
    // BENCHMARK n.3
    //
    // Add a CLAMPED HALF CYLINDER :
    //

    if (false)
    {
        double plate_thickness = 0.03;
        double plate_R = 1.016;
        double plate_L = 3.048;

        // Create a material 
        double rho = 0.0;
        double E = 2.0685e7;
        double nu = 0.3; 

        auto mat = std::make_shared<ChMaterialShellReissnerIsothropic>(
                                                         rho, 
                                                         E, 
                                                         nu,
                                                         1.0,
                                                         0.01);

        // In case you want to test laminated shells, use this:
        auto mat_ortho = std::make_shared<ChMaterialShellReissnerOrthotropic>(
                                                         rho, 
                                                         2.0685e7,
                                                         0.517e7,
                                                         0.3,
                                                         0.795e7,
                                                         0.795e7,
                                                         0.795e7,
                                                         1.0,
                                                         0.01);

        // Create the nodes  
        
        int nels_U = 32;
        int nels_W = 32;
        double arc = CH_C_PI;
        std::vector<std::shared_ptr<ChElementShellReissner4>> elarray(nels_U*nels_W);
        std::vector<std::shared_ptr<ChNodeFEAxyzrot>>     nodearray((nels_U+1)*(nels_W+1));
        std::vector<std::shared_ptr<ChNodeFEAxyzrot>>     nodes_start(nels_W+1);
        std::vector<std::shared_ptr<ChNodeFEAxyzrot>>     nodes_end(nels_W+1);
        std::vector<std::shared_ptr<ChNodeFEAxyzrot>>     nodes_left(nels_U+1);
        std::vector<std::shared_ptr<ChNodeFEAxyzrot>>     nodes_right(nels_U+1);

        for (int iu= 0; iu<= nels_U; ++iu) {
            for (int iw = 0; iw<= nels_W; ++iw) {
                // Make nodes
                double u = ((double)iu/(double)nels_U);
                double w = ((double)iw/(double)nels_W);
                ChVector<> nodepos(
                    (plate_R) * cos(w*arc),
                    (plate_R) * sin(w*arc),
                    u*plate_L);
                ChQuaternion<> noderot(QUNIT);
                ChFrame<> nodeframe(nodepos, noderot);

                auto mnode = std::make_shared<ChNodeFEAxyzrot>(nodeframe);
                my_mesh->AddNode(mnode);

                mnode->GetInertia().FillDiag(0.0); 
                mnode->SetMass(0.0); 

                nodearray[iu*(nels_W+1) + iw] = mnode;

                if (iu==0)
                    nodes_start[iw] = mnode;
                if (iu==nels_U)
                    nodes_end[iw] = mnode;
                if (iw==0)
                    nodes_left[iu] = mnode;
                if (iw==nels_W)
                    nodes_right[iu] = mnode;

                // Make elements
                if (iu>0 && iw>0) {
                    auto melement = std::make_shared<ChElementShellReissner4>();
                    my_mesh->AddElement(melement);
                    
                    melement->SetNodes(
                        nodearray[(iu  )*(nels_W+1) + (iw  )],
                        nodearray[(iu-1)*(nels_W+1) + (iw  )],
                        nodearray[(iu-1)*(nels_W+1) + (iw-1)], 
                        nodearray[(iu  )*(nels_W+1) + (iw-1)]
                        );

                    melement->AddLayer(plate_thickness, 0 * CH_C_DEG_TO_RAD, mat);
                    // In case you want to test laminated shells, do instead:
                    //  melement->AddLayer(plate_thickness/3, 0 * CH_C_DEG_TO_RAD, mat_ortho);
                    //  melement->AddLayer(plate_thickness/3, 90 * CH_C_DEG_TO_RAD, mat_ortho);
                    //  melement->AddLayer(plate_thickness/3, 0 * CH_C_DEG_TO_RAD, mat_ortho);
                    melement->SetAlphaDamp(0.0);   
                    elarray[(iu-1)*(nels_W) + (iw-1)] = melement;                    
                }
            }
        }

        nodesLoad.push_back( nodes_end[nodes_end.size()/2] );
        nodePlotA = nodes_end[nodes_end.size()/2];
        nodePlotB = nodes_end[nodes_end.size()/2];

        for (auto mstartnode : nodes_start) {
            mstartnode->SetFixed(true);
        }
        
        auto mtruss = std::make_shared<ChBody>();
        mtruss->SetBodyFixed(true);
        my_system.Add(mtruss);
        for (auto mendnode : nodes_left) {
            auto mlink = std::make_shared<ChLinkMateGeneric>(false,true,false, true,false,true);
            mlink->Initialize(mendnode, mtruss, false, mendnode->Frame(), mendnode->Frame());
            my_system.Add(mlink);
        }
        for (auto mendnode : nodes_right) {
            auto mlink = std::make_shared<ChLinkMateGeneric>(false,true,false, true,false,true);
            mlink->Initialize(mendnode, mtruss, false, mendnode->Frame(), mendnode->Frame());
            my_system.Add(mlink);
        }
        

        load_force  = ChVector<>(0, -2000, 0);
        load_torque = VNULL;

        // reference solution to plot
        ref_X.AddPoint(0.10, 1-0.16);   
        ref_X.AddPoint(0.20, 1-0.37);   
        ref_X.AddPoint(0.30, 1-0.66);  
        ref_X.AddPoint(0.40, 1-1.13);   
        ref_X.AddPoint(0.50, 1-1.32); 
        ref_X.AddPoint(0.60, 1-1.44);  
        ref_X.AddPoint(0.70, 1-1.52);  
        ref_X.AddPoint(0.80, 1-1.60);  
        ref_X.AddPoint(0.90, 1-1.66);  
        ref_X.AddPoint(1.00, 1-1.71);  
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
    /*
    auto mvisualizeshellB = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizeshellB->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizeshellB->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizeshellB->SetSymbolsThickness(0.01);
    my_mesh->AddAsset(mvisualizeshellB);
    */

    auto mvisualizeshellC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizeshellC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    //mvisualizeshellC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    mvisualizeshellC->SetSymbolsThickness(0.05);
    mvisualizeshellC->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizeshellC);


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
    auto mkl_solver = std::make_shared<ChSolverMKL<>>();
    mkl_solver->SetSparsityPatternLock(true);
    my_system.SetSolver(mkl_solver);

    /*
    my_system.SetSolverType(ChSolver::Type::MINRES); // <- NEEDED THIS or Matlab or MKL solver
	my_system.SetSolverWarmStarting(true); // this helps a lot to speedup convergence in this class of problems
	my_system.SetMaxItersSolverSpeed(200);
	my_system.SetMaxItersSolverStab(200);
	my_system.SetTolForce(1e-13);
	auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
	msolver->SetVerbose(false);
	msolver->SetDiagonalPreconditioning(true);
    */

    // Change type of integrator:
    my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT); 
    //my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    //my_system.SetTimestepperType(ChTimestepper::NEWMARK);

    if (auto mint =  std::dynamic_pointer_cast<ChImplicitIterativeTimestepper>(my_system.GetTimestepper())) {
        mint->SetMaxiters(5);
        mint->SetAbsTolerances(1e-12, 1e-12);
    }

    double timestep = 0.1;
    application.SetTimestep(timestep);
    my_system.Setup();
    my_system.Update();
    
    ChFunction_Recorder rec_X;
    ChFunction_Recorder rec_Y;
    
    double mtime = 0;

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        // .. draw also a grid
        ChIrrTools::drawGrid(application.GetVideoDriver(), 1, 1);

        // ...update load at end nodes, as simple lumped nodal forces
        
        double load_scale = mtime*0.1;
        for (auto mendnode : nodesLoad) {
            mendnode->SetForce (load_force  * load_scale * (1./ (double)nodesLoad.size()) );
            mendnode->SetTorque(load_torque * load_scale * (1./ (double)nodesLoad.size()) );
        }

        //application.DoStep();
        //mtime = my_system.GetChTime();
        application.GetSystem()->DoStaticNonlinear(3);
        //application.GetSystem()->DoStaticLinear();
        mtime += timestep;

        if(!application.GetPaused() && nodePlotA && nodePlotB) {
            rec_Y.AddPoint( load_scale, nodePlotA->GetPos().y());
            rec_X.AddPoint( load_scale, nodePlotB->GetPos().y());
        }

        application.EndScene();

        if (load_scale > 1)
            application.GetDevice()->closeDevice();
        
    }

    // Outputs results in a GNUPLOT plot:

    ChGnuPlot mplot("__shell_benchmark.gpl");
    mplot.SetGrid(false, 1, ChColor(0.8f, 0.8f, 0.8f));
    mplot.SetLabelX("Torque T/T0");
    mplot.SetLabelY("Tip displacement [m]");
    mplot << "set key left top";
    mplot.Plot(rec_Y, "W", " with lines lt -1 lc rgb'#00AAEE' ");
    mplot.Plot(rec_X, "-U", " with lines lt -1 lc rgb'#AA00EE' ");
    mplot.Plot(ref_Y, "W ref.", "pt 4 ps 1.4");
    mplot.Plot(ref_X, "-U ref.", "pt 4 ps 1.4");

    return 0;
}


