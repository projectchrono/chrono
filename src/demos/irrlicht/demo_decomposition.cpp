///////////////////////////////////////////////////
//
//   A small interactive editor to test the 
//   convex decomposition settings.
//
//	 CHRONO    
//   ------
//   Multibody dinamics engine
//  
// ------------------------------------------------ 
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------ 
///////////////////////////////////////////////////
 
  
 
#include "physics/ChApidll.h" 
#include "irrlicht_interface/ChIrrAppInterface.h"
#include "irrlicht_interface/ChIrrMeshTools.h"
#include "core/ChRealtimeStep.h"
#include "collision/ChCConvexDecomposition.h"

#include <irrlicht.h>


// Use the namespace of Chrono

using namespace chrono;

// Use the main namespace of Irrlicht 
// (and prefer not to use 'using ...' also for others(irr::video, irr::gui, etc.), to avoid name pollution)
using namespace irr; 



// to make things easier (it's a test..) introduce global variables.

scene::IAnimatedMesh*	modelMesh;
scene::IAnimatedMeshSceneNode* modelNode; 
scene::ISceneNode* decompositionNode; 

ChConvexDecomposition mydecomposition;

int decompdepth;
int maxhullvert;
float concavity;
float merge;
float volumep;
bool useinitialislands;




// LOAD A TRIANGLE MESH USING IRRLICHT IMPORTERS
//
void LoadModel(ChIrrAppInterface* application, const char* filename)
{
	if (modelNode)
		modelNode->remove();
	modelNode = 0;

	if (decompositionNode)
		decompositionNode->remove();
	decompositionNode = 0;

	// Load a mesh using the Irrlicht I/O conversion
	// from mesh file formats. 
	modelMesh = application->GetSceneManager()->getMesh(filename);
	modelNode = application->GetSceneManager()->addAnimatedMeshSceneNode (modelMesh);
	modelNode->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
}



void DecomposeModel(ChIrrAppInterface* application)
{
	if (decompositionNode)
		decompositionNode->remove();
	decompositionNode = 0;

	if (!modelNode)
		return;

	decompositionNode = application->GetSceneManager()->addEmptySceneNode();

	// Convert the Irrlicht mesh into a Chrono::Engine mesh.
	ChTriangleMesh chmesh;
	//modelNode->getMesh();
	fillChTrimeshFromIrlichtMesh(&chmesh,modelNode->getMesh());// modelMesh->getMesh(0));

	// Perform the convex decomposition
	// using the desired parameters.

	mydecomposition.Reset();
	mydecomposition.AddTriangleMesh(chmesh);
	
	mydecomposition.ComputeConvexDecomposition(
					0,	 // skin width 
					decompdepth, // decomp.depth 
					maxhullvert, // max hull vertexes
					concavity,   // concavity threshold percent 
					merge,		 // merge threshold percent
					volumep,     // volume split percent
					true, // initial islands
					false);
	
	// Visualize the resulting convex decomposition by creating many
	// colored meshes, each per convex hull.
	for (unsigned int j = 0; j< mydecomposition.GetHullCount(); j++)
	{
		scene::SMesh* mmesh = new scene::SMesh();	

		// Get the j-th convex hull as a ChTriangleMesh.
		ChTriangleMesh chmesh_hull;
		mydecomposition.GetConvexHullResult(j, chmesh_hull);

		video::SColor clr(255, 20+(int)(140.*ChRandom()), 20+(int)(140.*ChRandom()),  20+(int)(140.*ChRandom()));

		// Convert the j-th convex hull from a ChTriangleMesh to an Irrlicht mesh.
		fillIrlichtMeshFromChTrimesh(mmesh, &chmesh_hull, clr);
	
		// Add Irrlicht mesh to the scene, as a scene node.
		scene::SAnimatedMesh* Amesh = new scene::SAnimatedMesh();
		Amesh->addMesh(mmesh);
		mmesh->drop();

		scene::IAnimatedMeshSceneNode*	piece_node = application->GetSceneManager()->addAnimatedMeshSceneNode(Amesh, decompositionNode);
		piece_node->getMaterial(0).EmissiveColor.set(255,40,40,50);//255, 50, 50, 50);
		//piece_node->getMaterial(0).AmbientColor.set(255,30,30,30);//100, 0,0,0);
		piece_node->getMaterial(0).DiffuseColor.set(255,clr.getRed(), clr.getGreen(),clr.getBlue());//255, 50, 50, 50);
		//piece_node->getMaterial(0).Lighting = true;
		piece_node->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
		scene::IAnimatedMeshSceneNode*	piece_node2 = application->GetSceneManager()->addAnimatedMeshSceneNode(Amesh, decompositionNode);
		piece_node2->getMaterial(0).Lighting = true;
		piece_node2->getMaterial(0).Wireframe = true;
		piece_node2->getMaterial(0).Thickness = 2;	
	}

	modelNode->setVisible(false);
}


void SaveHullsWavefront(ChIrrAppInterface* application, const char* filename)
{
	// Save the convex decomposition to a 
	// file using the .obj fileformat.

	try
	{
		ChStreamOutAsciiFile decomposed_objfile(filename);
		mydecomposition.WriteConvexHullsAsWavefrontObj(decomposed_objfile);
	}
	catch (ChException myex)
	{
		application->GetIGUIEnvironment()->addMessageBox(L"Save file error", L"Impossible to write into file.");
	}
}

void SaveHullsChulls(ChIrrAppInterface* application, const char* filename)
{
	// Save the convex decomposition to a 
	// file using the .obj fileformat.

	try
	{
		ChStreamOutAsciiFile decomposed_objfile(filename);
		mydecomposition.WriteConvexHullsAsChullsFile(decomposed_objfile);
	}
	catch (ChException myex)
	{
		application->GetIGUIEnvironment()->addMessageBox(L"Save file error", L"Impossible to write into file.");
	}
}



// Define a MyEventReceiver class which will be used to manage input
// from the GUI graphical user interface (the interface will
// be created with the basic -yet flexible- platform
// independent toolset of Irrlicht).

class MyEventReceiver : public IEventReceiver
{
public:

	MyEventReceiver(ChIrrAppInterface* mapp)
			{
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
				submenu->addItem(L"Load STEP model...", 95, false);
				submenu->addItem(L"Save convex hulls (.obj)", 91);
				submenu->addItem(L"Save convex hulls (.chulls)", 96);
				submenu->addSeparator();
				submenu->addItem(L"Quit", 92);
				submenu = menu->getSubMenu(1);
				submenu->addItem(L"View model", 93);
				submenu->addItem(L"View decomposition", 94);

				// ..add a GUI
				edit_decompdepth = app->GetIGUIEnvironment()->addEditBox(irr::core::stringw((int)decompdepth).c_str(),  
					core::rect<s32>(510, 60, 650, 75), true, 0, 101);
				text_decompdepth = app->GetIGUIEnvironment()->addStaticText(
					L"Decomp.depth ", core::rect<s32>(650,60,750,75), false);

				// ..add a GUI 
				edit_maxhullvert = app->GetIGUIEnvironment()->addEditBox(irr::core::stringw((int)maxhullvert).c_str(),  
					core::rect<s32>(510, 85, 650, 100), true, 0, 102);
				text_maxhullvert = app->GetIGUIEnvironment()->addStaticText(
					L"Max hull vertexes", core::rect<s32>(650,85,750,100), false);

				// ..add a GUI  
				edit_concavity = app->GetIGUIEnvironment()->addEditBox(irr::core::stringw(concavity).c_str(),  
					core::rect<s32>(510, 110, 650, 125), true, 0, 103);
				text_concavity = app->GetIGUIEnvironment()->addStaticText(
					L"Concavity threshold", core::rect<s32>(650,110,750,125), false);

				// ..add a GUI  
				edit_merge = app->GetIGUIEnvironment()->addEditBox(irr::core::stringw(merge).c_str(),  
					core::rect<s32>(510, 135, 650, 150), true, 0, 104);
				text_merge = app->GetIGUIEnvironment()->addStaticText(
					L"Merge threshold", core::rect<s32>(650,135,750,150), false);

				// ..add a GUI  
				edit_volume = app->GetIGUIEnvironment()->addEditBox(irr::core::stringw(volumep).c_str(),  
					core::rect<s32>(510, 160, 650, 175), true, 0, 105);
				text_volume = app->GetIGUIEnvironment()->addStaticText(
					L"Volume threshold", core::rect<s32>(650,160,750,175), false);

				// ..add a GUI  
				checkbox_islands = app->GetIGUIEnvironment()->addCheckBox(
					useinitialislands, core::rect<s32>(620, 185, 650, 200), 0, 110);
				gui::IGUIStaticText* text_islands = app->GetIGUIEnvironment()->addStaticText(
					L"Use initial mesh islands", core::rect<s32>(650,185,850,200), false);

				// .. add buttons..
				button_decompose =  app->GetIGUIEnvironment()->addButton(core::rect<s32>(510, 210, 650, 225), 0, 106,
									L"Decompose", L"Perform convex decomposition");


			}

	bool OnEvent(const SEvent& event)
			{

				// check if user moved the sliders with mouse..
				if (event.EventType == EET_GUI_EVENT)
				{
					s32 id = event.GUIEvent.Caller->getID();
					gui::IGUIEnvironment* env = app->GetIGUIEnvironment();

					switch(event.GUIEvent.EventType)
					{

					case gui::EGET_MENU_ITEM_SELECTED:
						{
							// a menu item was clicked

							gui::IGUIContextMenu* menu = (gui::IGUIContextMenu*)event.GUIEvent.Caller;
							s32 id = menu->getItemCommandId(menu->getSelectedItem());

							switch(id)
							{
							case 90: 
								env->addFileOpenDialog(L"Load a mesh file, in .OBJ/.X/.MAX format",true, 0, 80);
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
							case 92: // File -> Quit
								app->GetDevice()->closeDevice();
								break;
							case 93:
								if (modelNode) modelNode->setVisible(true);
								if (decompositionNode) decompositionNode->setVisible(false);
								break;
							case 94: 
								if (modelNode) modelNode->setVisible(false);
								if (decompositionNode) decompositionNode->setVisible(true);
								break;
							}
						break;
						}

					case gui::EGET_FILE_SELECTED:
						{
							// load the model file, selected in the file open dialog
							gui::IGUIFileOpenDialog* dialog = (gui::IGUIFileOpenDialog*)event.GUIEvent.Caller;
						
							switch(id)
							{
							case 80: 
								LoadModel(app, core::stringc(dialog->getFileName()).c_str() );
								break;
							case 85: 
								//LoadStepModel(app, core::stringc(dialog->getFileName()).c_str() );
								break;
							case 81: 
								SaveHullsWavefront(app, core::stringc(dialog->getFileName()).c_str() );
								break;
							case 86: 
								SaveHullsChulls(app, core::stringc(dialog->getFileName()).c_str() );
								break;
							}
						}
						break;

					case gui::EGET_CHECKBOX_CHANGED:
						{
							switch(id)
							{
							case 110:
								useinitialislands =  checkbox_islands->isChecked();
								GetLog() << checkbox_islands->isChecked() << "\n";
								return true;
							default:
								return false;
							}
						}

					case gui::EGET_EDITBOX_ENTER:
						{
							// load the model file, selected in the file open dialog
							gui::IGUIEditBox* medit = (gui::IGUIEditBox*)event.GUIEvent.Caller;
						
							switch(id)
							{
							case 101: 
								decompdepth = (int) atof( irr::core::stringc(medit->getText()).c_str() ) ;
								medit->setText(irr::core::stringw((int)decompdepth).c_str());
								break;
							case 102: 
								maxhullvert = (int) atof( irr::core::stringc(medit->getText()).c_str() ) ;
								medit->setText(irr::core::stringw((int)maxhullvert).c_str());
								break;
							case 103: 
								concavity = atof( irr::core::stringc(medit->getText()).c_str() ) ;
								medit->setText(irr::core::stringw(concavity).c_str());
								break;
							case 104: 
								merge =  atof( irr::core::stringc(medit->getText()).c_str() ) ;
								medit->setText(irr::core::stringw(merge).c_str());
								break;
							case 105: 
								volumep =  atof( irr::core::stringc(medit->getText()).c_str() ) ;
								medit->setText(irr::core::stringw(volumep).c_str());
								break;
							}
						}
						break;

					case gui::EGET_BUTTON_CLICKED:
						{
							switch(id)
							{
							case 106:
								DecomposeModel(app);
								return true;
							default:
								return false;
							}
							break;
						}

					case gui::EGET_SCROLL_BAR_CHANGED:
							if (id == 101) 
							{
								s32 pos = ((gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
								decompdepth =  (int) (double(pos)*(64.0/100.0)) ;
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
	gui::IGUIEditBox*  edit_decompdepth;
	gui::IGUIStaticText* text_maxhullvert;
	gui::IGUIEditBox*    edit_maxhullvert;
	gui::IGUIStaticText* text_concavity;
	gui::IGUIEditBox*    edit_concavity;
	gui::IGUIStaticText* text_merge;
	gui::IGUIEditBox*    edit_merge;
	gui::IGUIStaticText* text_volume;
	gui::IGUIEditBox*    edit_volume;
	gui::IGUICheckBox*	 checkbox_islands;
	gui::IGUIButton*     button_decompose; 
};



//
// This is the program which is executed
//

int main(int argc, char* argv[])
{

	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed.
	DLL_CreateGlobals();

	// 1- Create a ChronoENGINE physical system: all bodies and constraints
	//    will be handled by this ChSystem object.
	ChSystem my_system;

	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrAppInterface application(&my_system, L"Convex decomposition of a mesh",core::dimension2d<u32>(800,600),false, true, video::EDT_OPENGL); 

	// Easy shortcuts to add logo, camera, lights and sky in Irrlicht scene:
	//ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice(), core::vector3df(30,100,30), core::vector3df(30,-80,-30),200,130);
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0,1.5,-2));


	// Initial settings
	modelMesh = 0;
	modelNode = 0;
	decompositionNode =0;

	decompdepth = 8;
	maxhullvert = 640;
	concavity = 0.1f;
	merge = 30.f;
	volumep = 0.1f;
	useinitialislands = true;


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

	
	while(application.GetDevice()->run())
	{ 
		// Irrlicht must prepare frame to draw
		application.GetVideoDriver()->beginScene(true, true, video::SColor(255,140,161,192));

		// .. draw solid 3D items (boxes, cylinders, shapes) belonging to Irrlicht scene, if any
		application.DrawAll();

		// .. draw also a grid (rotated so that it's horizontal)
		//ChIrrTools::drawGrid(application.GetVideoDriver(), 2, 2, 30,30, 
		//	ChCoordsys<>(ChVector<>(0,0.01,0), Q_from_AngX(CH_C_PI_2) ),
		//	video::SColor(40, 90,130,140), true);

		application.GetVideoDriver()->endScene(); 
	}


	
	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}


