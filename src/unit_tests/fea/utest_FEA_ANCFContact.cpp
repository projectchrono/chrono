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
// Authors: Milad Rakhsha
// =============================================================================
//
// This unit test creats 2 ANCF shell elements and place them in different positions
// in order to validate the collision detection in Chrono.
// =============================================================================

#include <cmath>

#include <algorithm>
#include <functional>

#include "chrono/collision/ChCCollisionModel.h"
#include "chrono/physics/ChContactContainerBase.h"
#include "chrono/physics/ChContactContainerBase.h"
#include "chrono/physics/ChContactContainerDEM.h"
#include "chrono/physics/ChContactDEM.h"
#include "chrono/physics/ChContactTuple.h"
#include "chrono/physics/ChContactable.h"
#include "chrono/physics/ChMaterialSurfaceDEM.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChSystemDEM.h"
#include "chrono/physics/ChSystemDEM.h"
#include "chrono/solver/ChSolverMINRES.h"

#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChContactSurfaceMesh.h"
#include "chrono_fea/ChLoadContactSurfaceMesh.h"
#include "chrono_fea/ChContactSurfaceNodeCloud.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"

using namespace chrono;
using namespace chrono::fea;

bool addGravity = false;
double time_step = 0.001;
// Forward declaration
bool EvaluateContact(std::shared_ptr<ChMaterialShellANCF> material,
                     std::shared_ptr<ChMaterialSurfaceDEM> mysurfmaterial,
                     double sphere_swept_thickness,
                     double scaleFactor,
                     double elementThickness,
                     ChVector<> trans_elem2,
                     ChMatrix33<> rot_elem2,
                     bool AlsoPrint);

// =============================================================================
// ========================= main Function
// =============================================================================
int main(int argc, char* argv[]) {
    // You can perform this unit test with different material prop.
    double rho = 1000;  ///< material density
    double E = 5e8;     ///< Young's modulus
    double nu = 0.3;    ///< Poisson ratio
    auto my_material = std::make_shared<ChMaterialShellANCF>(rho, E, nu);
    // You can also change the contact surface propeties for further investigation.
    auto mysurfmaterial = std::make_shared<ChMaterialSurfaceDEM>();
    mysurfmaterial->SetKn(1e0);
    mysurfmaterial->SetKt(0);
    mysurfmaterial->SetGn(1e0);
    mysurfmaterial->SetGt(0);
    // =============================================================================
    ChMatrix33<> rot_transform(1);
    ChVector<> translate;
    double sphere_swept_thickness;
    double scaleFactor;
    double elementThickness;
    bool printContacts = true;
    // This unit test creates 2 ANCF Shell Elements and place them in different
    // positions in each test and evaluates if there is contact between them or not.
    // The first element is a square of 2m*2m which is placed at the origin of the
    // coordinate system. The second element is a square of 1m*1m which is also
    // placed at the same place, but it is translated and rotated using translate
    // vector and rot_transform matrix in order to obtain different config. Note that
    // the plate is positioned in xz plane and y is the normal direction.

    // =======================TEST 1============================================
    printf("--------------------------------------------------\n");
    translate = ChVector<>(0, 1, 0);
    sphere_swept_thickness = 0.505;
    scaleFactor = 1;
    elementThickness = 0.01;
    bool test1Passed = false;

    bool ThereIsContact1 = EvaluateContact(my_material, mysurfmaterial, sphere_swept_thickness, scaleFactor,
                                           elementThickness, translate, rot_transform, printContacts);
    if (ThereIsContact1) {
        printf("There are contacts in test 1. Test passed.\n");
        test1Passed = true;
    }  // This case has contacts
    else
        printf("There are no contacts in test 2. Test failed.\n");
    printf("--------------------------------------------------\n");
    // =======================TEST 2============================================
    // same test with smaller sphere_swept_thickness
    translate = ChVector<>(0, 1, 0);
    sphere_swept_thickness = 0.499;
    scaleFactor = 1;
    elementThickness = 0.01;
    bool test2Passed = false;

    bool ThereIsContact2 = EvaluateContact(my_material, mysurfmaterial, sphere_swept_thickness, scaleFactor,
                                           elementThickness, translate, rot_transform, printContacts);
    if (ThereIsContact2)
        printf("There are contacts in test 2. Test failed.\n");
    else {
        {
            printf("There are no contacts in test 2. Test passed.\n");
            test2Passed = true;
        }
    }  // This case does not have contacts
    printf("--------------------------------------------------\n");
    // =======================TEST 3============================================
    // test for to elements positioned on the same plane.
    translate = ChVector<>(2.5, 0, 0);
    sphere_swept_thickness = 0.249;
    scaleFactor = 1;
    elementThickness = 0.01;
    bool test3Passed = false;

    bool ThereIsContact3 = EvaluateContact(my_material, mysurfmaterial, sphere_swept_thickness, scaleFactor,
                                           elementThickness, translate, rot_transform, printContacts);
    if (ThereIsContact3)
        printf("There are contacts in test 3. Test failed.\n");
    else {
        printf("There are no contacts in test 3. Test passed.\n");
        test3Passed = true;
    }  // This case does not have contacts
    printf("--------------------------------------------------\n");

    // =======================TEST 4============================================

    translate = ChVector<>(2.1, 0, 0);
    sphere_swept_thickness = 0.4;
    scaleFactor = 1;
    elementThickness = 0.01;
    bool test4Passed = false;
    bool ThereIsContact4 = EvaluateContact(my_material, mysurfmaterial, sphere_swept_thickness, scaleFactor,
                                           elementThickness, translate, rot_transform, printContacts);
    if (ThereIsContact4)
        printf("There are contacts in test 4. Test failed.\n");
    // This case should not pass if the sphere_swept_thickness is expanded
    // only in normal direction not normal to the element's edge.
    else {
        printf("There are no contacts in test 4. Test passed.\n");
        test4Passed = true;
    }
    printf("--------------------------------------------------\n");
    // =============================================================================

    if (test1Passed && test2Passed && test3Passed && test4Passed)
        printf("Unit test passed successfully.\n");
    else
        printf("Unit test passed failed.\n");

    return 0;
}

// Custom contact container -- get access to the contact lists in the base class.
class MyContactContainer : public ChContactContainerDEM {
  public:
    MyContactContainer() {}
    // Traverse the list contactlist_6_6
    bool isThereContacts(std::shared_ptr<ChElementBase> myShellANCF, bool print) {
        auto iter = contactlist_333_333.begin();
        int num_contact = 0;
        while (iter != contactlist_333_333.end()) {
            ChContactable* objA = (*iter)->GetObjA();
            ChContactable* objB = (*iter)->GetObjB();
            ChVector<> p1 = (*iter)->GetContactP1();
            ChVector<> p2 = (*iter)->GetContactP2();
            double CD = (*iter)->GetContactDistance();

            if (print) {
                printf("P1=[%f %f %f]\n", p1.x(), p1.y(), p1.z());
                printf("P2=[%f %f %f]\n", p2.x(), p2.y(), p2.z());
                printf("Contact Distance=%f\n\n", CD);
            }
            num_contact++;
            ++iter;
        }
        return num_contact > 0;
    }
};

/////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
bool EvaluateContact(std::shared_ptr<ChMaterialShellANCF> material,
                     std::shared_ptr<ChMaterialSurfaceDEM> mysurfmaterial,
                     double sphere_swept_thickness,
                     double scaleFactor,
                     double elementThickness,
                     ChVector<> trans_elem2,
                     ChMatrix33<> rot_elem2,
                     bool AlsoPrint) {
    ChSystemDEM my_system(false, 16000, 500);

    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.001);
    my_system.SetContactForceModel(ChSystemDEM::Hooke);

    double L_x = 1.0;
    double L_y = elementThickness;
    double L_z = 1.0;  // small thickness

    std::vector<ChVector<>> N1(4);  // To add nodes of the first element
    std::vector<ChVector<>> N2(4);  // To add nodes of the second element

    N1[0] = ChVector<>(-L_x, 0, -L_z) * scaleFactor;
    N1[1] = ChVector<>(+L_x, 0, -L_z) * scaleFactor;
    N1[2] = ChVector<>(+L_x, 0, +L_z) * scaleFactor;
    N1[3] = ChVector<>(-L_x, 0, +L_z) * scaleFactor;

    N2[0] = ChVector<>(-L_x, 0, -L_z) * scaleFactor + trans_elem2;
    N2[1] = ChVector<>(-L_x, 0, +L_z) * scaleFactor + trans_elem2;
    N2[2] = ChVector<>(+L_x, 0, +L_z ) * scaleFactor + trans_elem2;
    N2[3] = ChVector<>(+L_x, 0, -L_z ) * scaleFactor + trans_elem2;

    ChVector<> direction1 (0, 1, 0);
    ChVector<> direction2(0, -1, 0);
    auto my_meshes_1 = std::make_shared<ChMesh>();
    auto my_meshes_2 = std::make_shared<ChMesh>();

    // Note that two elements are added in two different meshes
    for (int i = 0; i < 4; i++) {
        auto node = std::make_shared<ChNodeFEAxyzD>(N1[i], direction1);
        node->SetMass(0);
        my_meshes_1->AddNode(node);
    }
    for (int i = 0; i < 4; i++) {
        auto node = std::make_shared<ChNodeFEAxyzD>(N2[i], direction2);
        node->SetMass(0);
        my_meshes_2->AddNode(node);
    }

    // Create the element 1 and 2 and add them to their relevant mesh.
    auto Element1 = std::make_shared<ChElementShellANCF>();  // To add nodes of the first element
    Element1->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_meshes_1->GetNode(0)),
                       std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_meshes_1->GetNode(1)),
                       std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_meshes_1->GetNode(2)),
                       std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_meshes_1->GetNode(3)));

    Element1->SetDimensions(L_x, L_z);
    Element1->AddLayer(L_y, 0 * CH_C_DEG_TO_RAD, material);
    Element1->SetAlphaDamp(0.02);   // Structural damping for this element
    Element1->SetGravityOn(false);  // turn internal gravitational force calculation off
    my_meshes_1->AddElement(Element1);

    auto Element2 = std::make_shared<ChElementShellANCF>();  // To add nodes of the first element

    Element2->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_meshes_2->GetNode(0)),
                       std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_meshes_2->GetNode(1)),
                       std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_meshes_2->GetNode(2)),
                       std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_meshes_2->GetNode(3)));
    Element2->SetDimensions(L_x, L_z);
    Element2->AddLayer(L_y, 0 * CH_C_DEG_TO_RAD, material);
    Element2->SetAlphaDamp(0.02);   // Structural damping for this element
    Element2->SetGravityOn(false);  // turn internal gravitational force calculation off
    my_meshes_2->AddElement(Element2);
    std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_meshes_1->GetNode(0))->SetFixed(true);

    auto mcontactsurf_1 = std::make_shared<ChContactSurfaceMesh>();
    auto mcontactsurf_2 = std::make_shared<ChContactSurfaceMesh>();
    my_meshes_1->AddContactSurface(mcontactsurf_1);
    my_meshes_2->AddContactSurface(mcontactsurf_2);
    mcontactsurf_1->AddFacesFromBoundary(sphere_swept_thickness);  // do this after my_mesh->AddContactSurface
    mcontactsurf_1->SetMaterialSurface(mysurfmaterial);            // use the DEM penalty contacts
    mcontactsurf_2->AddFacesFromBoundary(sphere_swept_thickness);  // do this after my_mesh->AddContactSurface
    mcontactsurf_2->SetMaterialSurface(mysurfmaterial);

    // use the DEM penalty contacts
    my_meshes_1->SetAutomaticGravity(addGravity);
    my_meshes_2->SetAutomaticGravity(addGravity);

    if (addGravity) {
        my_system.Set_G_acc(ChVector<>(0, -1, 0));
    } else {
        my_system.Set_G_acc(ChVector<>(0, 0, 0));
    }
    my_system.Add(my_meshes_1);
    my_system.Add(my_meshes_2);

    my_system.SetupInitial();
    // ---------------

    my_system.SetSolverType(ChSolver::Type::MINRES);
    auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
    msolver->SetDiagonalPreconditioning(true);
    my_system.SetSolverWarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
    my_system.SetMaxItersSolverSpeed(100000);
    my_system.SetMaxItersSolverStab(100);
    my_system.SetTolForce(1e-6);

    my_system.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxiters(40);
    mystepper->SetAbsTolerances(1e-05, 1e-1);        // For Pos
    mystepper->SetMode(ChTimestepperHHT::POSITION);  // POSITION //ACCELERATION
    mystepper->SetScaling(true);
    mystepper->SetVerbose(false);
    auto container = std::make_shared<MyContactContainer>();
    //    auto contacts = std::make_shared<MyContacts>();

    my_system.SetContactContainer(container);
    bool thereIsContact;
    bool printContactPoints = true;
    auto myANCF = std::dynamic_pointer_cast<ChElementBase>(my_meshes_2->GetElement(0));
    my_system.DoStepDynamics(time_step);
    thereIsContact = container->isThereContacts(myANCF, AlsoPrint);
    return thereIsContact;
};
