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
#include <iomanip>
#include <filesystem>
#include <set>

#include "chrono_wood/ChBasisToolsBeziers.h"
#include "chrono_wood/ChLineBezierCBL.h"
#include "chrono_wood/ChElementCBLCON.h"
#include "chrono_wood/ChBuilderCBLCON.h"
#include "chrono_wood/ChWoodMaterialVECT.h"
#include "chrono_wood/ChBeamSectionCBLCON.h"
#include "chrono_wood/ChElementCurvilinearBeamBezier.h"
#include "chrono_wood/ChBuilderCurvilinearBeam.h"
#include "chrono_wood/ChVisualShapeWOOD.h"

#include "chrono/core/ChBezierCurve.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotor.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChInertiaUtils.h"

#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono/geometry/ChLineNurbs.h"

#include "chrono/utils/ChConvexHull.h"

#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::wood;

void WriteFrame(std::shared_ptr<ChMesh> mesh,
                const std::string &vtk_filename)
{
    std::ofstream out_stream;
    out_stream.open(vtk_filename, std::ios::trunc);

    out_stream << "# vtk DataFile Version 2.0" << std::endl;
    out_stream << "Unstructured Grid Example" << std::endl;
    out_stream << "ASCII" << std::endl;
    out_stream << "DATASET UNSTRUCTURED_GRID" << std::endl;

    out_stream << "POINTS " << mesh->GetNumNodes() << " double\n";

    for (unsigned int i = 0; i < mesh->GetNumNodes(); i++)
    {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(i));
        out_stream << node->GetPos().x() << " " << node->GetPos().y() << " " << node->GetPos().z() << "\n";
    }

    std::vector<std::vector<int>> CableElemNodes;
    std::vector<std::vector<int>> ShellElemNodes;
    std::vector<std::vector<int>> BrickElemNodes;
    std::vector<std::vector<int>> BeamElemNodes;
    std::vector<std::vector<int>> TetElemNodes;

    std::vector<std::shared_ptr<ChNodeFEAbase>> myNodevector;
    myNodevector.resize(mesh->GetNumNodes());

    for (unsigned int i = 0; i < mesh->GetNumNodes(); i++)
    {
        myNodevector[i] = std::dynamic_pointer_cast<ChNodeFEAbase>(mesh->GetNode(i));
    }

    int numCables = 0;
    int numShells = 0;
    int numBricks = 0;
    int numBeams = 0;
    int QnumBeams = 0;
    int numTets = 0;

    for (unsigned int iele = 0; iele < mesh->GetNumElements(); iele++)
    {
        if (std::dynamic_pointer_cast<ChElementBeamEuler>(mesh->GetElement(iele)))
            numBeams++;
        if (std::dynamic_pointer_cast<ChElementCBLCON>(mesh->GetElement(iele)))
            numBeams++;
        if (std::dynamic_pointer_cast<ChElementCurvilinearBeamBezier>(mesh->GetElement(iele)))
            QnumBeams++;
    }
    out_stream << "\nCELLS " << mesh->GetNumElements() << " "
               << (unsigned int)(numCables * 3 + numBeams * 3 + QnumBeams * 4 + numTets * 5 + numShells * 5 + numBricks * 9) << "\n";

    for (unsigned int iele = 0; iele < mesh->GetNumElements(); iele++)
    {
        std::vector<int> mynodes;

        if (auto elementBm = std::dynamic_pointer_cast<ChElementBeamEuler>(mesh->GetElement(iele)))
        {
            mynodes.resize(2);
            out_stream << elementBm->GetNumNodes() << " ";
            int nodeOrder[] = {0, 1};
            mynodes[0] = elementBm->GetNode(nodeOrder[0])->GetIndex();
            mynodes[1] = elementBm->GetNode(nodeOrder[1])->GetIndex();
            BeamElemNodes.push_back(mynodes);
            for (int myNodeN = 0; myNodeN < mynodes.size(); myNodeN++)
            {
                auto nodeA = (elementBm->GetNode(nodeOrder[myNodeN]));
                std::vector<std::shared_ptr<ChNodeFEAbase>>::iterator it;
                it = find(myNodevector.begin(), myNodevector.end(), nodeA);
                if (it == myNodevector.end())
                {
                    // name not in vector
                }
                else
                {
                    auto index = std::distance(myNodevector.begin(), it);
                    out_stream << (unsigned int)index << " ";
                }
            }
            out_stream << "\n";
        }
        else if (auto elementBm = std::dynamic_pointer_cast<ChElementCBLCON>(mesh->GetElement(iele)))
        {
            mynodes.resize(elementBm->GetNumNodes());
            out_stream << elementBm->GetNumNodes() << " ";
            int nodeOrder[] = {0, 1};
            mynodes[0] = elementBm->GetNode(nodeOrder[0])->GetIndex();
            mynodes[1] = elementBm->GetNode(nodeOrder[1])->GetIndex();
            BeamElemNodes.push_back(mynodes);
            for (int myNodeN = 0; myNodeN < mynodes.size(); myNodeN++)
            {
                auto nodeA = (elementBm->GetNode(nodeOrder[myNodeN]));
                std::vector<std::shared_ptr<ChNodeFEAbase>>::iterator it;
                it = find(myNodevector.begin(), myNodevector.end(), nodeA);
                if (it == myNodevector.end())
                {
                    // name not in vector
                }
                else
                {
                    auto index = std::distance(myNodevector.begin(), it);
                    out_stream << (unsigned int)index << " ";
                }
            }
            out_stream << "\n";
        }
        else if (auto elementBm = std::dynamic_pointer_cast<ChElementCurvilinearBeamBezier>(mesh->GetElement(iele)))
        {

            mynodes.resize(elementBm->GetNumNodes());
            out_stream << elementBm->GetNumNodes() << " ";
            // mynodes[0] = elementBm->GetNode(nodeOrder[0])->GetIndex();
            // mynodes[1] = elementBm->GetNode(nodeOrder[1])->GetIndex();
            // BeamElemNodes.push_back(mynodes);
            for (int myNodeN = 0; myNodeN < mynodes.size(); myNodeN++)
            {
                auto nodeA = (elementBm->GetNode(myNodeN));
                std::vector<std::shared_ptr<ChNodeFEAbase>>::iterator it;
                it = find(myNodevector.begin(), myNodevector.end(), nodeA);
                if (it == myNodevector.end())
                {
                    // name not in vector
                }
                else
                {
                    auto index = std::distance(myNodevector.begin(), it);
                    out_stream << (unsigned int)index << " ";
                }
            }
            out_stream << "\n";
        }
    }

    out_stream << "\nCELL_TYPES " << mesh->GetNumElements() << "\n";

    for (unsigned int iele = 0; iele < mesh->GetNumElements(); iele++)
    {
        if (std::dynamic_pointer_cast<ChElementCurvilinearBeamBezier>(mesh->GetElement(iele)))
            out_stream << "4\n";
        else if (std::dynamic_pointer_cast<ChElementBeamEuler>(mesh->GetElement(iele)))
            out_stream << "3\n";
        else if (std::dynamic_pointer_cast<ChElementCBLCON>(mesh->GetElement(iele)))
            out_stream << "3\n";
    }

    int numCell = 0;
    for (unsigned int iele = 0; iele < mesh->GetNumElements(); iele++)
    {
        if (std::dynamic_pointer_cast<ChElementCableANCF>(mesh->GetElement(iele)))
            numCell++;
        else if (std::dynamic_pointer_cast<ChElementBeamEuler>(mesh->GetElement(iele)))
            numCell++;
        else if (std::dynamic_pointer_cast<ChElementCBLCON>(mesh->GetElement(iele)))
            numCell++;
        else if (std::dynamic_pointer_cast<ChElementCurvilinearBeamBezier>(mesh->GetElement(iele)))
            numCell++;
    }

    out_stream << "\nCELL_DATA " << numCell << "\n";
    out_stream << "SCALARS Force double\n";
    out_stream << "LOOKUP_TABLE default\n";

    out_stream << "\nPOINT_DATA " << mesh->GetNumNodes() << "\n";
    out_stream << "VECTORS Displacement double\n";
    // out_stream << "LOOKUP_TABLE default\n";
    for (unsigned int i = 0; i < mesh->GetNumNodes(); i++)
    {
        ChVector3d disp = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(i))->GetPos();
        ChVector3d disp0 = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(i))->GetX0().GetPos();
        disp -= disp0; // ChVector3d(1e-40);
        out_stream << (double)disp.x() << " " << (double)disp.y() << " " << (double)disp.z() << "\n";
    }

    out_stream << "\nPOINT_DATA " << mesh->GetNumNodes() << "\n";
    out_stream << "VECTORS Velocity double\n";
    // out_stream << "LOOKUP_TABLE default\n";
    for (unsigned int i = 0; i < mesh->GetNumNodes(); i++)
    {
        ChVector3d vel = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(i))->GetPosDt();
        vel += ChVector3d(1e-40);
        out_stream << (double)vel.x() << " " << (double)vel.y() << " " << (double)vel.z() << "\n";
    }

    out_stream << "\nPOINT_DATA " << mesh->GetNumNodes() << "\n";
    out_stream << "VECTORS Acceleration double\n";
    // out_stream << "LOOKUP_TABLE default\n";

    for (unsigned int i = 0; i < mesh->GetNumNodes(); i++)
    {
        ChVector3d acc = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(i))->GetPosDt2();
        acc += ChVector3d(1e-40);
        out_stream << (double)acc.x() << " " << (double)acc.y() << " " << (double)acc.z() << "\n";
    }

    out_stream.close();
}

void plainstrain(ChSystem &sys, std::string &current_dir)
{
    ///
    /// Define Directory and files
    ///
    std::string CBL_data_path = current_dir + "spruce1_0mm_cube3/";
    std::string CBL_GeoName = "spruce1_0mm_cube3";
    //
    //
    const std::string out_dir = current_dir + "/spruce1_0mm_cube3_out/";
    //
    // Create oputput directories
    if (!filesystem::create_directory(filesystem::path(out_dir)))
    {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        exit(1);
    }

    std::string history_filename = "hist.dat";
    std::ofstream histfile;
    histfile.open(out_dir + history_filename, std::ios::out);

    histfile << "test";

    //
    // Create ground
    //
    auto mtruss = chrono_types::make_shared<ChBody>();
    mtruss->SetFixed(true);
    mtruss->EnableCollision(false);
    mtruss->SetPos(ChVector3d(9., 0, 0));
    sys.Add(mtruss);
    ///
    /// Define loading plates
    ///
    auto mysurfmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    mysurfmaterial->SetYoungModulus(2e5);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.0f);
    mysurfmaterial->SetAdhesion(0);
    ///
    ///
    auto bottom_plate = chrono_types::make_shared<ChBodyEasyBox>(10, 10, 0.2, 7.8E-14, true, true, mysurfmaterial);
    bottom_plate->SetPos(ChVector3d(5.75, 0, -0.1));
    bottom_plate->GetVisualShape(0)->SetTexture("/chrono-concrete/data/textures/blue.png");
    bottom_plate->GetVisualShape(0)->SetOpacity(0.5);
    sys.Add(bottom_plate);
    ///
    ///
    auto top_plate = chrono_types::make_shared<ChBodyEasyBox>(10, 10, 0.2, 7.8E-14, true, true, mysurfmaterial);
    top_plate->SetPos(ChVector3d(5.75, 0, 1.1));
    top_plate->GetVisualShape(0)->SetTexture("/chrono-concrete/data/textures/blue.png");
    top_plate->GetVisualShape(0)->SetOpacity(0.5);
    sys.Add(top_plate);
    ///
    auto back_plate = chrono_types::make_shared<ChBodyEasyBox>(10, 0.2, 10, 7.8E-9, true, true, mysurfmaterial);
    back_plate->SetPos(ChVector3d(5.15, 0, 0.5));
    back_plate->GetVisualShape(0)->SetTexture("/chrono-concrete/data/textures/blue.png");
    back_plate->GetVisualShape(0)->SetOpacity(0.5);
    sys.Add(back_plate);
    ///
    ///
    auto front_plate = chrono_types::make_shared<ChBodyEasyBox>(10, 0.2, 10, 7.8E-9, true, true, mysurfmaterial);
    front_plate->SetPos(ChVector3d(6.25, 0., 0.5));
    front_plate->GetVisualShape(0)->SetTexture("/chrono-concrete/data/textures/blue.png");
    front_plate->GetVisualShape(0)->SetOpacity(0.5);
    sys.Add(front_plate);
    ///
    auto left_plate = chrono_types::make_shared<ChBodyEasyBox>(0.2, 10, 10, 7.8E-9, true, true, mysurfmaterial);
    left_plate->SetPos(ChVector3d(5.75, -.6, 0.5));
    left_plate->GetVisualShape(0)->SetTexture("/chrono-concrete/data/textures/blue.png");
    left_plate->GetVisualShape(0)->SetOpacity(0.5);
    sys.Add(left_plate);
    ///
    ///
    auto right_plate = chrono_types::make_shared<ChBodyEasyBox>(0.2, 10, 10, 7.8E-9, true, true, mysurfmaterial);
    right_plate->SetPos(ChVector3d(5.75, .6, 0.5));
    right_plate->GetVisualShape(0)->SetTexture("/chrono-concrete/data/textures/blue.png");
    right_plate->GetVisualShape(0)->SetOpacity(0.5);
    sys.Add(right_plate);
    
    ///
    /// Assign boundary conditions to loading plates (plain strain)
    ///
    auto constr_bottom_plate = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, true); // fully fixed
    constr_bottom_plate->Initialize(bottom_plate, mtruss, bottom_plate->GetFrameCOMToAbs());
    sys.Add(constr_bottom_plate);
    auto constr_back_plate = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, true);
    constr_back_plate->Initialize(back_plate, mtruss, back_plate->GetFrameCOMToAbs());
    sys.Add(constr_back_plate);
    auto constr_front_plate = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, true);
    constr_front_plate->Initialize(front_plate, mtruss, front_plate->GetFrameCOMToAbs());
    sys.Add(constr_front_plate);
    auto constr_left_plate = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, true);
    constr_left_plate->Initialize(left_plate, mtruss, left_plate->GetFrameCOMToAbs());
    sys.Add(constr_left_plate);
    auto constr_right_plate = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, true);
    constr_right_plate->Initialize(right_plate, mtruss, right_plate->GetFrameCOMToAbs());
    sys.Add(constr_right_plate);
    ///
    /// Create mesh objects to store FEM and CBL meshes and elements
    ///
    auto my_mesh = chrono_types::make_shared<ChMesh>();
    my_mesh->SetAutomaticGravity(false);
    //
    // Read Bezier elements info
    //
    ChElementCurvilinearBeamBezier::LumpedMass = true;
    ChElementCurvilinearBeamBezier::quadrature_type = ChElementCurvilinearBeamBezier::QuadratureType::FULL_OVER;
    ChElementCurvilinearBeamBezier::LargeDeflection = false;
    double rho = 5.00E-8;
    double E_mod = 3.5e+4;
    double nu_rat = 0.3;

    //
    ChBuilderCurvilinearBeamBezier builder;
    builder.read_CBL_info_ElBased(my_mesh, CBL_data_path, CBL_GeoName, E_mod, E_mod / (1.0 + nu_rat) / 2., rho / 2.);

    // exit(9);
    //
    //
    ///////////////////////////////////////////////////////////////////////////////
    //
    // Read connector elements info
    //
    ///////////////////////////////////////////////////////////////////////////////
    //
    //
    //
    // Create  vectorial material for CBLCON
    //
    auto my_mat = chrono_types::make_shared<ChWoodMaterialVECT>();
    my_mat->Set_density(rho / 2.);
    my_mat->Set_E0(8000);
    my_mat->Set_alpha(0.2373);
    my_mat->Set_sigmat(75.);
    my_mat->Set_sigmas(30.);
    my_mat->Set_nt(0.2);
    my_mat->Set_lt(5.0);
    my_mat->Set_Ed(3000);
    my_mat->Set_sigmac0(120);
    my_mat->Set_Hc0(9900);
    my_mat->Set_Hc1(3000);
    my_mat->Set_beta(0);
    my_mat->Set_kc0(3);
    my_mat->Set_kc1(0.5);
    my_mat->Set_kc2(5);
    my_mat->Set_kc3(0.1);
    my_mat->Set_mu0(0.2);
    my_mat->Set_muinf(0.2);
    my_mat->Set_sigmaN0(600);
    my_mat->SetCoupleMultiplier(1.0);
    my_mat->SetElasticAnalysisFlag(true);
    //
    auto my_mat_long = chrono_types::make_shared<ChWoodMaterialVECT>();
    my_mat_long->Set_density(rho);
    my_mat_long->Set_E0(8000);
    my_mat_long->Set_alpha(0.2373);
    my_mat_long->Set_sigmat(75.);
    my_mat_long->Set_sigmas(30.);
    my_mat_long->Set_nt(0.2);
    my_mat_long->Set_lt(5.0);
    my_mat_long->Set_Ed(3000);
    my_mat_long->Set_sigmac0(120);
    my_mat_long->Set_Hc0(9900);
    my_mat_long->Set_Hc1(3000);
    my_mat_long->Set_beta(0);
    my_mat_long->Set_kc0(3);
    my_mat_long->Set_kc1(0.5);
    my_mat_long->Set_kc2(5);
    my_mat_long->Set_kc3(0.1);
    my_mat_long->Set_mu0(0.2);
    my_mat_long->Set_muinf(0.2);
    my_mat_long->Set_sigmaN0(600);
    my_mat_long->SetCoupleMultiplier(1.0);
    my_mat_long->SetElasticAnalysisFlag(true); //
    //
    std::vector<std::shared_ptr<ChWoodMaterialVECT>> connector_mat;
    connector_mat.push_back(my_mat);
    connector_mat.push_back(my_mat_long);
    //
    // exit(8);
    ChElementCBLCON::LumpedMass = true;
    ChElementCBLCON::EnableCoupleForces = true;
    ChElementCBLCON::LargeDeflection = false;
    //
    ChBuilderCBLCON conbuilder;
    conbuilder.read_CBLCON_info(my_mesh, connector_mat, CBL_data_path, CBL_GeoName);
    //
    // Add mesh into system object
    //
    sys.Add(my_mesh);
    /////////////////////////////////////////////////////
    /// Select nodes on top surface of the wood specimen
    ///
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> top_nodes;
    for (unsigned int i = 0; i < my_mesh->GetNumNodes(); i++)
    {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(i));
        double cz = node->GetPos().z();
        if (cz >= 0.999)
        {
            top_nodes.push_back(node);
        }
    }
    std::cout << "top_nodes: " << top_nodes.size() << std::endl;
    ///
    /// Select nodes on bottom surface of the wood specimen
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> bottom_nodes;
    for (unsigned int i = 0; i < my_mesh->GetNumNodes(); i++)
    {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(i));
        double cz = node->GetPos().z();
        if (cz <= 0.0001)
        {
            bottom_nodes.push_back(node);
        }
    }
    std::cout << "bottom_nodes: " << bottom_nodes.size() << std::endl;
    ///
    /// Select nodes on front surface
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> front_nodes;
    for (unsigned int i = 0; i < my_mesh->GetNumNodes(); i++)
    {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(i));
        double cx = node->GetPos().x();
        if (cx >= 6.2499)
        {
            front_nodes.push_back(node);
        }
    }
    std::cout << "front_nodes: " << front_nodes.size() << std::endl;
    ///
    /// Select nodes on back surface
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> back_nodes;
    for (unsigned int i = 0; i < my_mesh->GetNumNodes(); i++)
    {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(i));
        double cx = node->GetPos().x();
        if (cx <= 5.25001)
        {
            back_nodes.push_back(node);
        }
    }
    std::cout << "back_nodes: " << back_nodes.size() << std::endl;
    ///
    /// Select nodes on right surface
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> right_nodes;
    for (unsigned int i = 0; i < my_mesh->GetNumNodes(); i++)
    {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(i));
        double cy = node->GetPos().y();
        if (cy >= 0.4999)
        {
            right_nodes.push_back(node);
        }
    }
    std::cout << "right_nodes: " << right_nodes.size() << std::endl;
    ///
    /// Select nodes on left surface
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> left_nodes;
    for (unsigned int i = 0; i < my_mesh->GetNumNodes(); i++)
    {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(i));
        double cy = node->GetPos().y();
        if (cy <= -0.4999)
        {
            left_nodes.push_back(node);
        }
    }
    std::cout << "left_nodes: " << left_nodes.size() << std::endl;
    ///
    ///
    /// Tie bottom nodes to bottom plate
    std::vector<std::shared_ptr<ChLinkMateGeneric>> const_bot;
    for (auto node : bottom_nodes)
    {
        auto constr_tie = chrono_types::make_shared<ChLinkMateGeneric>(false, false, true, true, true, true);
        constr_tie->Initialize(node, bottom_plate, false, node->Frame(), node->Frame());
        const_bot.push_back(constr_tie);
        sys.Add(constr_tie);
    }
    ///
    /// Tie top nodes to top plate
    std::vector<std::shared_ptr<ChLinkMateGeneric>> const_top;
    for (auto node : top_nodes)
    {
        auto constr_tie = chrono_types::make_shared<ChLinkMateGeneric>(false, false, true, false, false, false);
        constr_tie->Initialize(node, top_plate, false, node->Frame(), node->Frame());
        const_top.push_back(constr_tie);
        sys.Add(constr_tie);
    }
    ///
    /// Tie front nodes to front plate
    std::vector<std::shared_ptr<ChLinkMateGeneric>> const_front;
    for (auto node : front_nodes)
    {
        auto constr_tie = chrono_types::make_shared<ChLinkMateGeneric>(true, true, false, true, true, true);
        constr_tie->Initialize(node, front_plate, false, node->Frame(), node->Frame());
        const_front.push_back(constr_tie);
        sys.Add(constr_tie);
    }
    ///
    /// Tie back nodes to back plate
    std::vector<std::shared_ptr<ChLinkMateGeneric>> const_back;
    for (auto node : back_nodes)
    {
        auto constr_tie = chrono_types::make_shared<ChLinkMateGeneric>(true, true, false, true, true, true);
        constr_tie->Initialize(node, back_plate, false, node->Frame(), node->Frame());
        const_back.push_back(constr_tie);
        sys.Add(constr_tie);
    }
    ///
    /// Tie right nodes to right plate
    std::vector<std::shared_ptr<ChLinkMateGeneric>> const_right;
    for (auto node : right_nodes)
    {
        auto constr_tie = chrono_types::make_shared<ChLinkMateGeneric>(true, true, false, true, true, true);
        constr_tie->Initialize(node, right_plate, false, node->Frame(), node->Frame());
        const_right.push_back(constr_tie);
        sys.Add(constr_tie);
    }
    ///
    /// Tie left nodes to left plate
    std::vector<std::shared_ptr<ChLinkMateGeneric>> const_left;
    for (auto node : left_nodes)
    {
        auto constr_tie = chrono_types::make_shared<ChLinkMateGeneric>(true, true, false, true, true, true);
        constr_tie->Initialize(node, left_plate, false, node->Frame(), node->Frame());
        const_left.push_back(constr_tie);
        sys.Add(constr_tie);
    }
    ///
    ///
    /// Create the linear motor
    ///
    auto motor1 = chrono_types::make_shared<ChLinkMotorLinearPosition>();

    // Connect the guide and the slider and add the motor to the system:
    motor1->Initialize(top_plate,                            // body A (slave)
                       mtruss,                               // body B (master)
                       ChFrame<>(top_plate->GetPos(), QUNIT) // motor frame, in abs. coords
                                                             // ChFrame<>(front_plate->GetPos(),GetFromAngleAxis(-CH_PI_2*0, VECT_Y))
    );

    auto my_motion_function = chrono_types::make_shared<ChFunctionRamp>();
    my_motion_function->SetAngularCoeff(0.03);

    motor1->SetMotionFunction(my_motion_function);
    sys.Add(motor1);
    ///
    /// Set solver and time stepper
    ///
    ChStaticNonLinearAnalysis static_analysis;
    static_analysis.SetCorrectionTolerance(1e-4, 1e-8);
    static_analysis.SetIncrementalSteps(20);
    static_analysis.SetMaxIterations(50);
    static_analysis.SetVerbose(true);
    int stepnum = 0;
    double currentTime = 0;
    while (sys.GetChTime() < 1)
    {
        // my_motion_function->SetConstant(currentTime*0.005);
        std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n"
                  << std::endl;
        std::cout << "Time: " << currentTime << std::endl;
        sys.DoStepDynamics(0.02);

        if (stepnum == -1)
        {
            std::cout << " GetNcoords " << sys.GetNumCoordsPosLevel() << std::endl;
            std::cout << " GetDof() " << sys.GetNumCoordsPosLevel() << std::endl;
            std::cout << " GetDoc() " << sys.GetNumConstraints() << std::endl;
            std::cout << " GetNbodies() " << sys.GetNumBodiesActive() << std::endl;
            std::cout << " GetNmeshes() " << sys.GetNumMeshes() << std::endl;
            sys.WriteSystemMatrices(true, true, true, true, (current_dir + "debug").c_str());
        }

        if (stepnum % 1 == 0)
        {

            std::cout << " t=\t" << sys.GetChTime() << "\ttop_plate disp_x=\t" << motor1->GetMotorPos()
                      << "\tforce=\t" << motor1->GetReaction1().force
                      // << "\tSupport=\t" << calculate_Force( constr_bot ) //constr_bot->Get_react_force()
                      << "\t\n";

            histfile << " t=\t" << sys.GetChTime() << "\ttop_plate disp_x=\t" << motor1->GetMotorPos()
                     << "\tforce=\t" << motor1->GetReaction1().force
                     // << "\tSupport=\t" << calculate_Force( constr_bot ) //constr_bot->Get_react_force()
                     << "\t\n";
        }

        stepnum++;
    }

    histfile.close();
}

int main(int argc, char *argv[])
{

    SetChronoDataPath(CHRONO_DATA_DIR);
    std::cout << std::setprecision(10);

    // Initialize output //
    std::filesystem::path path_to_this_file(__FILE__);
    std::string current_dir = path_to_this_file.remove_filename().string();
    std::cout << current_dir;

    // Create a Chrono::Engine physical system
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));
    // sys.SetNumThreads(1,1,1);

    // auto solver = chrono_types::make_shared<ChSolverSparseLU>();
    auto solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);

    auto mystepper = chrono_types::make_shared<ChTimestepperHHT>(&sys);
    mystepper->SetAlpha(-0.2); // alpha=-0.2 default value
    mystepper->SetMaxIters(250);
    mystepper->SetAbsTolerances(1e-08);

    mystepper->SetVerbose(true);
    mystepper->SetModifiedNewton(chrono::ChTimestepperHHT::JacobianUpdate::NEVER);
    mystepper->SetStepControl(false);

    sys.SetTimestepper(mystepper);

    plainstrain(sys, current_dir);

    return 0;
}
