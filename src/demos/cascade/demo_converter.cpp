//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   A small interactive editor to test the
//   convex decomposition settings and the STEP
//   conversion features of OpenCASCADE library
//
//	 CHRONO
//   ------
//   Multibody dinamics engine
//
// ------------------------------------------------
//             http://www.projectchrono.org
// ------------------------------------------------
///////////////////////////////////////////////////

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/collision/ChCConvexDecomposition.h"

#include "chrono_cascade/ChCascadeDoc.h"
#include "chrono_cascade/ChCascadeMeshTools.h"
#include "chrono_cascade/ChIrrCascadeMeshTools.h"

#include <TopoDS_Shape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_HShape.hxx>
#include <Handle_TopoDS_HShape.hxx>
#include <Handle_TopoDS_TShape.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_StepModelType.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <BRepMesh.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopExp_Explorer.hxx>
#include <Bnd_Box.hxx>
#include <gp_Pnt.hxx>
#include <Prs3d_ShapeTool.hxx>
#include <BRepAdaptor_HSurface.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepBndLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <TColgp_HArray1OfVec.hxx>
#include <TColStd_HArray1OfInteger.hxx>
#include <Poly_Connect.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <TColgp_Array1OfDir.hxx>

#include <CSLib_DerivativeStatus.hxx>
#include <CSLib_NormalStatus.hxx>
#include <CSLib.hxx>
#include <TColgp_Array1OfPnt2d.hxx>

#include <TDocStd_Document.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <TDataStd_Name.hxx>
#include <Interface_Static.hxx>
#include <TDF_Label.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_Tool.hxx>
#include <TObj_TObject.hxx>
#include <TObj_TReference.hxx>
#include <TNaming_NamedShape.hxx>

#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_irrlicht/ChIrrMeshTools.h"

// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespace of Irrlicht
// (and prefer not to use 'using ...' also for others(irr::video, irr::gui, etc.), to avoid name pollution)
using namespace irr;

// Use the namespace with OpenCascade stuff
using namespace cascade;

// to make things easier (it's a test..) introduce global variables.

scene::IAnimatedMesh* modelMesh;
scene::IAnimatedMeshSceneNode* modelNode;
scene::ISceneNode* decompositionNode;

collision::ChConvexDecompositionJR mydecompositionJR;
collision::ChConvexDecompositionHACDv2 mydecompositionHACDv2;

collision::ChConvexDecomposition* used_decomposition;

int algo_type = 0;

int hacd_maxhullcount;
int hacd_maxhullmerge;
int hacd_maxhullvertexes;
double hacd_concavity;
double hacd_smallclusterthreshold;
double hacd_fusetolerance;

int decompdepth;
int maxhullvert;
float concavity;
float merge;
float volumep;
bool useinitialislands;

// LOAD A TRIANGLE MESH USING IRRLICHT IMPORTERS
//
void LoadModel(ChIrrAppInterface* application, const char* filename) {
    if (modelNode)
        modelNode->remove();
    modelNode = 0;

    if (decompositionNode)
        decompositionNode->remove();
    decompositionNode = 0;

    // Load a mesh using the Irrlicht I/O conversion
    // from mesh file formats.
    modelMesh = application->GetSceneManager()->getMesh(filename);
    modelNode = application->GetSceneManager()->addAnimatedMeshSceneNode(modelMesh);
    modelNode->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
}

// LOAD THE STEP 3D MODEL USING OPEN CASCADE
//
void LoadStepModel(ChIrrAppInterface* application, const char* filename) {
    if (modelNode)
        modelNode->remove();
    modelNode = 0;

    if (decompositionNode)
        decompositionNode->remove();
    decompositionNode = 0;

    GetLog() << "\n\n 0-LOADING THE STEP MODEL..   \n\n\n";

    ChCascadeDoc mydoc;
    bool aRes = mydoc.Load_STEP(filename);

    if (aRes) {
        // ---Print hierarchy on screen
        mydoc.Dump(GetLog());

        // ---Find all shapes and get as a single compound
        TopoDS_Shape mshape;
        mydoc.GetRootShape(mshape);

        if (!mshape.IsNull()) {
            // ---Perform Irrlicht triangulation

            scene::SMesh* mmesh = new scene::SMesh();
            video::SColor clr(255, 100, 120, 125);

            irr::scene::ChIrrCascadeMeshTools::fillIrrlichtMeshFromCascade(mmesh, mshape, 0.5);
            // ..also show in Irrlicht view
            scene::SAnimatedMesh* Amesh = new scene::SAnimatedMesh();
            Amesh->addMesh(mmesh);
            mmesh->drop();
            modelNode = application->GetSceneManager()->addAnimatedMeshSceneNode(Amesh, decompositionNode);

            // ---Convert to OBJ Wavefront file

            ChStreamOutAsciiFile mobjfile("triangulated_step_model_root.obj");
            // afinder.res_shape.Location(TopLoc_Location()); // to reset CAD reference as center of obj.
            ChCascadeMeshTools::fillObjFileFromCascade(mobjfile, mshape, 0.5);
            GetLog() << " .. done! \n";
        }

    } else
        GetLog() << "\n.. Error with reading STEP!   \n\n\n";
}

void DecomposeModel(ChIrrAppInterface* application) {
    if (decompositionNode)
        decompositionNode->remove();
    decompositionNode = 0;

    if (!modelNode)
        return;

    decompositionNode = application->GetSceneManager()->addEmptySceneNode();

    // Convert the Irrlicht mesh into a Chrono::Engine mesh.
    geometry::ChTriangleMeshSoup chmesh;
    // modelNode->getMesh();
    fillChTrimeshFromIrlichtMesh(&chmesh, modelNode->getMesh());  // modelMesh->getMesh(0));

    // Perform the convex decomposition
    // using the desired parameters.

    if (algo_type == 0) {
        mydecompositionJR.Reset();
        mydecompositionJR.AddTriangleMesh(chmesh);
        mydecompositionJR.SetParameters(0,                  // skin width
                                        decompdepth,        // decomp.depth
                                        maxhullvert,        // max hull vertexes
                                        concavity,          // concavity threshold percent
                                        merge,              // merge threshold percent
                                        volumep,            // volume split percent
                                        useinitialislands,  // initial islands
                                        false);
        mydecompositionJR.ComputeConvexDecomposition();
        used_decomposition = &mydecompositionJR;
    }

    if (algo_type == 1) {
        mydecompositionHACDv2.Reset();
        mydecompositionHACDv2.AddTriangleMesh(chmesh);

        mydecompositionHACDv2.SetParameters(hacd_maxhullcount, hacd_maxhullmerge, hacd_maxhullvertexes, hacd_concavity,
                                            hacd_smallclusterthreshold, hacd_fusetolerance);
        mydecompositionHACDv2.ComputeConvexDecomposition();
        used_decomposition = &mydecompositionHACDv2;
    }

    // Visualize the resulting convex decomposition by creating many
    // colored meshes, each per convex hull.
    for (unsigned int j = 0; j < used_decomposition->GetHullCount(); j++) {
        scene::SMesh* mmesh = new scene::SMesh();

        // Get the j-th convex hull as a ChTriangleMesh.
        geometry::ChTriangleMeshSoup chmesh_hull;
        used_decomposition->GetConvexHullResult(j, chmesh_hull);

        video::SColor clr(255, 20 + (int)(140. * ChRandom()), 20 + (int)(140. * ChRandom()),
                          20 + (int)(140. * ChRandom()));

        // Convert the j-th convex hull from a ChTriangleMesh to an Irrlicht mesh.
        fillIrlichtMeshFromChTrimesh(mmesh, &chmesh_hull, clr);

        // Add Irrlicht mesh to the scene, as a scene node.
        scene::SAnimatedMesh* Amesh = new scene::SAnimatedMesh();
        Amesh->addMesh(mmesh);
        mmesh->drop();

        scene::IAnimatedMeshSceneNode* piece_node =
            application->GetSceneManager()->addAnimatedMeshSceneNode(Amesh, decompositionNode);
        piece_node->getMaterial(0).EmissiveColor.set(255, 40, 40, 50);  // 255, 50, 50, 50);
        // piece_node->getMaterial(0).AmbientColor.set(255,30,30,30);//100, 0,0,0);
        piece_node->getMaterial(0)
            .DiffuseColor.set(255, clr.getRed(), clr.getGreen(), clr.getBlue());  // 255, 50, 50, 50);
        // piece_node->getMaterial(0).Lighting = true;
        piece_node->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
        scene::IAnimatedMeshSceneNode* piece_node2 =
            application->GetSceneManager()->addAnimatedMeshSceneNode(Amesh, decompositionNode);
        piece_node2->getMaterial(0).Lighting = true;
        piece_node2->getMaterial(0).Wireframe = true;
        piece_node2->getMaterial(0).Thickness = 2;
    }

    modelNode->setVisible(false);
}

void SaveHullsWavefront(ChIrrAppInterface* application, const char* filename) {
    // Save the convex decomposition to a
    // file using the .obj fileformat.

    try {
        ChStreamOutAsciiFile decomposed_objfile(filename);
        mydecompositionJR.WriteConvexHullsAsWavefrontObj(decomposed_objfile);
    } catch (ChException myex) {
        application->GetIGUIEnvironment()->addMessageBox(L"Save file error", L"Impossible to write into file.");
    }
}

void SaveHullsChulls(ChIrrAppInterface* application, const char* filename) {
    // Save the convex decomposition to a
    // file using the .obj fileformat.

    try {
        ChStreamOutAsciiFile decomposed_objfile(filename);
        mydecompositionJR.WriteConvexHullsAsChullsFile(decomposed_objfile);
    } catch (ChException myex) {
        application->GetIGUIEnvironment()->addMessageBox(L"Save file error", L"Impossible to write into file.");
    }
}

// Define a MyEventReceiver class which will be used to manage input
// from the GUI graphical user interface (the interface will
// be created with the basic -yet flexible- platform
// independent toolset of Irrlicht).

class MyEventReceiver : public IEventReceiver {
  public:
    MyEventReceiver(ChIrrAppInterface* mapp) {
        // store pointer to physical system & other stuff so we can tweak them by user keyboard
        app = mapp;

        app->GetDevice()->setEventReceiver(this);

        // create menu
        gui::IGUIContextMenu* menu = app->GetIGUIEnvironment()->addMenu();
        menu->addItem(L"File", -1, true, true);
        menu->addItem(L"View", -1, true, true);

        gui::IGUIContextMenu* submenu;
        submenu = menu->getSubMenu(0);
        submenu->addItem(L"Load OBJ mesh...", 90);
        submenu->addItem(L"Load STEP model...", 95);
        submenu->addItem(L"Save convex hulls (.obj)", 91);
        submenu->addItem(L"Save convex hulls (.chulls)", 96);
        submenu->addSeparator();
        submenu->addItem(L"Quit", 92);
        submenu = menu->getSubMenu(1);
        submenu->addItem(L"View model", 93);
        submenu->addItem(L"View decomposition", 94);

        gad_algo_type = app->GetIGUIEnvironment()->addComboBox(core::rect<s32>(510, 35, 650, 50), 0, 201);
        gad_algo_type->addItem(L"JR algorithm");
        gad_algo_type->addItem(L"HACDv2 algorithm");
        gad_algo_type->setSelected(algo_type);

        // ..add a GUI
        edit_decompdepth = app->GetIGUIEnvironment()->addEditBox(irr::core::stringw((int)decompdepth).c_str(),
                                                                 core::rect<s32>(510, 60, 650, 75), true, 0, 101);
        text_decompdepth =
            app->GetIGUIEnvironment()->addStaticText(L"Decomp.depth ", core::rect<s32>(650, 60, 750, 75), false);

        // ..add a GUI
        edit_maxhullvert = app->GetIGUIEnvironment()->addEditBox(irr::core::stringw((int)maxhullvert).c_str(),
                                                                 core::rect<s32>(510, 85, 650, 100), true, 0, 102);
        text_maxhullvert =
            app->GetIGUIEnvironment()->addStaticText(L"Max hull vertexes", core::rect<s32>(650, 85, 750, 100), false);

        // ..add a GUI
        edit_concavity = app->GetIGUIEnvironment()->addEditBox(irr::core::stringw(concavity).c_str(),
                                                               core::rect<s32>(510, 110, 650, 125), true, 0, 103);
        text_concavity = app->GetIGUIEnvironment()->addStaticText(L"Concavity threshold",
                                                                  core::rect<s32>(650, 110, 750, 125), false);

        // ..add a GUI
        edit_merge = app->GetIGUIEnvironment()->addEditBox(irr::core::stringw(merge).c_str(),
                                                           core::rect<s32>(510, 135, 650, 150), true, 0, 104);
        text_merge =
            app->GetIGUIEnvironment()->addStaticText(L"Merge threshold", core::rect<s32>(650, 135, 750, 150), false);

        // ..add a GUI
        edit_volume = app->GetIGUIEnvironment()->addEditBox(irr::core::stringw(volumep).c_str(),
                                                            core::rect<s32>(510, 160, 650, 175), true, 0, 105);
        text_volume =
            app->GetIGUIEnvironment()->addStaticText(L"Volume threshold", core::rect<s32>(650, 160, 750, 175), false);

        // ..add a GUI
        checkbox_islands =
            app->GetIGUIEnvironment()->addCheckBox(useinitialislands, core::rect<s32>(620, 185, 650, 200), 0, 110);
        text_islands = app->GetIGUIEnvironment()->addStaticText(L"Use initial mesh islands",
                                                                core::rect<s32>(650, 185, 850, 200), false);

        // ..add a GUI
        edit_hacd_maxhullcount = app->GetIGUIEnvironment()->addEditBox(
            irr::core::stringw((int)hacd_maxhullcount).c_str(), core::rect<s32>(510, 60, 650, 75), true, 0, 121);
        text_hacd_maxhullcount =
            app->GetIGUIEnvironment()->addStaticText(L"Max. hull count ", core::rect<s32>(650, 60, 750, 75), false);

        // ..add a GUI
        edit_hacd_maxhullmerge = app->GetIGUIEnvironment()->addEditBox(
            irr::core::stringw((int)hacd_maxhullmerge).c_str(), core::rect<s32>(510, 85, 650, 100), true, 0, 122);
        text_hacd_maxhullmerge =
            app->GetIGUIEnvironment()->addStaticText(L"Max. hull merge ", core::rect<s32>(650, 85, 750, 100), false);

        // ..add a GUI
        edit_hacd_maxhullvertexes = app->GetIGUIEnvironment()->addEditBox(
            irr::core::stringw((int)hacd_maxhullvertexes).c_str(), core::rect<s32>(510, 110, 650, 125), true, 0, 123);
        text_hacd_maxhullvertexes = app->GetIGUIEnvironment()->addStaticText(
            L"Max. vertexes per hull", core::rect<s32>(650, 110, 750, 125), false);

        // ..add a GUI
        edit_hacd_concavity = app->GetIGUIEnvironment()->addEditBox(irr::core::stringw(hacd_concavity).c_str(),
                                                                    core::rect<s32>(510, 135, 650, 150), true, 0, 124);
        text_hacd_concavity = app->GetIGUIEnvironment()->addStaticText(L"Max. concavity (0..1)",
                                                                       core::rect<s32>(650, 135, 750, 150), false);

        // ..add a GUI
        edit_hacd_smallclusterthreshold = app->GetIGUIEnvironment()->addEditBox(
            irr::core::stringw(hacd_smallclusterthreshold).c_str(), core::rect<s32>(510, 160, 650, 175), true, 0, 125);
        text_hacd_smallclusterthreshold = app->GetIGUIEnvironment()->addStaticText(
            L"Small cluster threshold", core::rect<s32>(650, 160, 750, 175), false);

        // ..add a GUI
        edit_hacd_fusetolerance = app->GetIGUIEnvironment()->addEditBox(
            irr::core::stringw(hacd_fusetolerance).c_str(), core::rect<s32>(510, 185, 650, 200), true, 0, 126);
        text_hacd_fusetolerance = app->GetIGUIEnvironment()->addStaticText(L"Vertex fuse tolerance",
                                                                           core::rect<s32>(650, 185, 750, 200), false);

        // .. add buttons..
        button_decompose = app->GetIGUIEnvironment()->addButton(core::rect<s32>(510, 210, 650, 225), 0, 106,
                                                                L"Decompose", L"Perform convex decomposition");

        edit_decompdepth->setVisible(algo_type == 0);
        text_decompdepth->setVisible(algo_type == 0);
        edit_maxhullvert->setVisible(algo_type == 0);
        text_maxhullvert->setVisible(algo_type == 0);
        edit_concavity->setVisible(algo_type == 0);
        text_concavity->setVisible(algo_type == 0);
        edit_merge->setVisible(algo_type == 0);
        text_merge->setVisible(algo_type == 0);
        edit_volume->setVisible(algo_type == 0);
        text_volume->setVisible(algo_type == 0);
        checkbox_islands->setVisible(algo_type == 0);
        text_islands->setVisible(algo_type == 0);
        text_hacd_maxhullcount->setVisible(algo_type == 1);
        edit_hacd_maxhullcount->setVisible(algo_type == 1);
        text_hacd_maxhullmerge->setVisible(algo_type == 1);
        edit_hacd_maxhullmerge->setVisible(algo_type == 1);
        text_hacd_maxhullvertexes->setVisible(algo_type == 1);
        edit_hacd_maxhullvertexes->setVisible(algo_type == 1);
        text_hacd_concavity->setVisible(algo_type == 1);
        edit_hacd_concavity->setVisible(algo_type == 1);
        text_hacd_smallclusterthreshold->setVisible(algo_type == 1);
        edit_hacd_smallclusterthreshold->setVisible(algo_type == 1);
        text_hacd_fusetolerance->setVisible(algo_type == 1);
        edit_hacd_fusetolerance->setVisible(algo_type == 1);
    }

    bool OnEvent(const SEvent& event) {
        // check if user moved the sliders with mouse..
        if (event.EventType == EET_GUI_EVENT) {
            s32 id = event.GUIEvent.Caller->getID();
            gui::IGUIEnvironment* env = app->GetIGUIEnvironment();

            switch (event.GUIEvent.EventType) {
                case gui::EGET_MENU_ITEM_SELECTED: {
                    // a menu item was clicked

                    gui::IGUIContextMenu* menu = (gui::IGUIContextMenu*)event.GUIEvent.Caller;
                    s32 id = menu->getItemCommandId(menu->getSelectedItem());

                    switch (id) {
                        case 90:
                            env->addFileOpenDialog(L"Load a mesh file, in .OBJ/.X/.MAX format", true, 0, 80);
                            break;
                        case 95:
                            env->addFileOpenDialog(L"Load a 3D CAD model, in STEP format", true, 0, 85);
                            break;
                        case 91:
                            env->addFileOpenDialog(L"Save decomposed convex hulls in .obj 3D format", true, 0, 81);
                            break;
                        case 96:
                            env->addFileOpenDialog(L"Save decomposed convex hulls in .chulls 3D format", true, 0, 86);
                            break;
                        case 92:  // File -> Quit
                            app->GetDevice()->closeDevice();
                            break;
                        case 93:
                            if (modelNode)
                                modelNode->setVisible(true);
                            if (decompositionNode)
                                decompositionNode->setVisible(false);
                            break;
                        case 94:
                            if (modelNode)
                                modelNode->setVisible(false);
                            if (decompositionNode)
                                decompositionNode->setVisible(true);
                            break;
                    }
                    break;
                }

                case gui::EGET_FILE_SELECTED: {
                    // load the model file, selected in the file open dialog
                    gui::IGUIFileOpenDialog* dialog = (gui::IGUIFileOpenDialog*)event.GUIEvent.Caller;

                    switch (id) {
                        case 80:
                            LoadModel(app, core::stringc(dialog->getFileName()).c_str());
                            break;
                        case 85:
                            LoadStepModel(app, core::stringc(dialog->getFileName()).c_str());
                            break;
                        case 81:
                            SaveHullsWavefront(app, core::stringc(dialog->getFileName()).c_str());
                            break;
                        case 86:
                            SaveHullsChulls(app, core::stringc(dialog->getFileName()).c_str());
                            break;
                    }
                } break;

                case gui::EGET_CHECKBOX_CHANGED: {
                    switch (id) {
                        case 110:
                            useinitialislands = checkbox_islands->isChecked();
                            GetLog() << checkbox_islands->isChecked() << "\n";
                            return true;
                        default:
                            return false;
                    }
                }

                case gui::EGET_EDITBOX_ENTER: {
                    // load the model file, selected in the file open dialog
                    gui::IGUIEditBox* medit = (gui::IGUIEditBox*)event.GUIEvent.Caller;

                    switch (id) {
                        case 101:
                            decompdepth = (int)atof(irr::core::stringc(medit->getText()).c_str());
                            medit->setText(irr::core::stringw((int)decompdepth).c_str());
                            break;
                        case 102:
                            maxhullvert = (int)atof(irr::core::stringc(medit->getText()).c_str());
                            medit->setText(irr::core::stringw((int)maxhullvert).c_str());
                            break;
                        case 103:
                            concavity = (float)atof(irr::core::stringc(medit->getText()).c_str());
                            medit->setText(irr::core::stringw(concavity).c_str());
                            break;
                        case 104:
                            merge = (float)atof(irr::core::stringc(medit->getText()).c_str());
                            medit->setText(irr::core::stringw(merge).c_str());
                            break;
                        case 105:
                            volumep = (float)atof(irr::core::stringc(medit->getText()).c_str());
                            medit->setText(irr::core::stringw(volumep).c_str());
                            break;
                        case 121:
                            hacd_maxhullcount = atof(irr::core::stringc(medit->getText()).c_str());
                            medit->setText(irr::core::stringw(hacd_maxhullcount).c_str());
                            break;
                        case 122:
                            hacd_maxhullmerge = atof(irr::core::stringc(medit->getText()).c_str());
                            medit->setText(irr::core::stringw(hacd_maxhullmerge).c_str());
                            break;
                        case 123:
                            hacd_maxhullvertexes = atof(irr::core::stringc(medit->getText()).c_str());
                            medit->setText(irr::core::stringw(hacd_maxhullvertexes).c_str());
                            break;
                        case 124:
                            hacd_concavity = atof(irr::core::stringc(medit->getText()).c_str());
                            medit->setText(irr::core::stringw(hacd_concavity).c_str());
                            break;
                        case 125:
                            hacd_smallclusterthreshold = atof(irr::core::stringc(medit->getText()).c_str());
                            medit->setText(irr::core::stringw(hacd_smallclusterthreshold).c_str());
                            break;
                        case 126:
                            hacd_fusetolerance = atof(irr::core::stringc(medit->getText()).c_str());
                            medit->setText(irr::core::stringw(hacd_fusetolerance).c_str());
                            break;
                    }
                } break;

                case gui::EGET_COMBO_BOX_CHANGED:
                    if (id == 201) {
                        int sel = ((gui::IGUIComboBox*)event.GUIEvent.Caller)->getSelected();
                        switch (sel) {
                            case 0:
                                algo_type = 0;
                                break;
                            case 1:
                                algo_type = 1;
                                break;
                        }
                        edit_decompdepth->setVisible(algo_type == 0);
                        text_decompdepth->setVisible(algo_type == 0);
                        edit_maxhullvert->setVisible(algo_type == 0);
                        text_maxhullvert->setVisible(algo_type == 0);
                        edit_concavity->setVisible(algo_type == 0);
                        text_concavity->setVisible(algo_type == 0);
                        edit_merge->setVisible(algo_type == 0);
                        text_merge->setVisible(algo_type == 0);
                        edit_volume->setVisible(algo_type == 0);
                        text_volume->setVisible(algo_type == 0);
                        checkbox_islands->setVisible(algo_type == 0);
                        text_islands->setVisible(algo_type == 0);
                        text_hacd_maxhullcount->setVisible(algo_type == 1);
                        edit_hacd_maxhullcount->setVisible(algo_type == 1);
                        text_hacd_maxhullmerge->setVisible(algo_type == 1);
                        edit_hacd_maxhullmerge->setVisible(algo_type == 1);
                        text_hacd_maxhullvertexes->setVisible(algo_type == 1);
                        edit_hacd_maxhullvertexes->setVisible(algo_type == 1);
                        text_hacd_concavity->setVisible(algo_type == 1);
                        edit_hacd_concavity->setVisible(algo_type == 1);
                        text_hacd_smallclusterthreshold->setVisible(algo_type == 1);
                        edit_hacd_smallclusterthreshold->setVisible(algo_type == 1);
                        text_hacd_fusetolerance->setVisible(algo_type == 1);
                        edit_hacd_fusetolerance->setVisible(algo_type == 1);
                        break;
                    }

                case gui::EGET_BUTTON_CLICKED: {
                    switch (id) {
                        case 106:
                            DecomposeModel(app);
                            return true;
                        default:
                            return false;
                    }
                    break;
                }

                case gui::EGET_SCROLL_BAR_CHANGED:
                    if (id == 101) {
                        s32 pos = ((gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        decompdepth = (int)(double(pos) * (64.0 / 100.0));
                        return true;
                    }
                    break;
            }
        }

        return false;
    }

  private:
    ChIrrAppInterface* app;

    gui::IGUIContextMenu* menu;
    gui::IGUIStaticText* text_decompdepth;
    gui::IGUIEditBox* edit_decompdepth;
    gui::IGUIStaticText* text_maxhullvert;
    gui::IGUIEditBox* edit_maxhullvert;
    gui::IGUIStaticText* text_concavity;
    gui::IGUIEditBox* edit_concavity;
    gui::IGUIStaticText* text_merge;
    gui::IGUIEditBox* edit_merge;
    gui::IGUIStaticText* text_volume;
    gui::IGUIEditBox* edit_volume;
    gui::IGUICheckBox* checkbox_islands;
    gui::IGUIStaticText* text_islands;
    gui::IGUIButton* button_decompose;
    gui::IGUIStaticText* text_hacd_maxhullcount;
    gui::IGUIEditBox* edit_hacd_maxhullcount;
    gui::IGUIStaticText* text_hacd_maxhullmerge;
    gui::IGUIEditBox* edit_hacd_maxhullmerge;
    gui::IGUIStaticText* text_hacd_maxhullvertexes;
    gui::IGUIEditBox* edit_hacd_maxhullvertexes;
    gui::IGUIStaticText* text_hacd_concavity;
    gui::IGUIEditBox* edit_hacd_concavity;
    gui::IGUIStaticText* text_hacd_smallclusterthreshold;
    gui::IGUIEditBox* edit_hacd_smallclusterthreshold;
    gui::IGUIStaticText* text_hacd_fusetolerance;
    gui::IGUIEditBox* edit_hacd_fusetolerance;
    gui::IGUIComboBox* gad_algo_type;
};

//
// This is the program which is executed
//

int main(int argc, char* argv[]) {
    // 1- Create a ChronoENGINE physical system: all bodies and constraints
    //    will be handled by this ChSystem object.
    ChSystem my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Convex decomposition of a mesh", core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add logo, camera, lights and sky in Irrlicht scene:
    // ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice(), core::vector3df(30, 100, 30), core::vector3df(30, -80, -30), 200, 130);
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 1.5, -2));

    // Initial settings
    modelMesh = 0;
    modelNode = 0;
    decompositionNode = 0;

    hacd_maxhullcount = 512;
    hacd_maxhullmerge = 256;
    hacd_maxhullvertexes = 64;
    hacd_concavity = 0.2;
    hacd_smallclusterthreshold = 0.0;
    hacd_fusetolerance = 1e-9;

    decompdepth = 8;
    maxhullvert = 640;
    concavity = 0.1f;
    merge = 30.f;
    volumep = 0.1f;
    useinitialislands = true;

    used_decomposition = &mydecompositionHACDv2;

    //
    // USER INTERFACE
    //

    // Create some graphical-user-interface (GUI) items to show on the screen.
    // This requires an event receiver object.
    MyEventReceiver receiver(&application);

    // note how to add a custom event receiver to the default interface:
    application.SetUserEventReceiver(&receiver);

    //
    // THE SOFT-REAL-TIME CYCLE, SHOWING THE SIMULATION
    //

    while (application.GetDevice()->run()) {
        // Irrlicht must prepare frame to draw
        application.GetVideoDriver()->beginScene(true, true, video::SColor(255, 140, 161, 192));

        // .. draw solid 3D items (boxes, cylinders, shapes) belonging to Irrlicht scene, if any
        application.DrawAll();

        // .. draw also a grid (rotated so that it's horizontal)
        // ChIrrTools::drawGrid(application.GetVideoDriver(), 2, 2, 30,30,
        //	ChCoordsys<>(ChVector<>(0,0.01,0), Q_from_AngX(CH_C_PI_2) ),
        //	video::SColor(40, 90,130,140), true);

        application.GetVideoDriver()->endScene();
    }

    return 0;
}
