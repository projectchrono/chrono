// =====================================================================================
//  
// ChModuleIrrlicht.i
// Create the Python and C# wrappers for the Chrono::Irrlicht module.
//
// ATTENTION: 
// Must be included from another SWIG interface file which defines the module.
//
// =====================================================================================

// Turn on the documentation of members, for more intuitive IDE typing
%feature("autodoc", "1");
%feature("flatnested", "1");


// Turn on the exception handling to intercept C++ exceptions
%include "exception.i"

%exception {
  try {
    $action
  } catch (const std::exception& e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}


// For optional casting of polimorphic objects:
%include "../chrono_cast.i" 

// For supporting shared pointers:
%include <std_shared_ptr.i>



// Include C++ headers this way...

%{
#include "chrono/solver/ChSolver.h"                      //// RADU: REMOVE?

#include "chrono/assets/ChVisualShapes.h"

#include <irrlicht.h>
#include "chrono_irrlicht/ChIrrTools.h"
#include "chrono_irrlicht/ChIrrEffects.h"
#include "chrono_irrlicht/ChIrrTools.h"
#include "chrono_irrlicht/ChIrrNodeShape.h"
#include "chrono_irrlicht/ChIrrNodeModel.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

%}


// Undefine ChApi and other macros that otherwise SWIG gives a syntax error
#define ChApiIrr 
#define ChApi 
#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#define IRRLICHT_API
#define _IRR_DEPRECATED_
#define CH_DEPRECATED(msg)

// Include other .i configuration files for SWIG. 
// These are divided in many .i files, each per a
// different c++ class, when possible.

%include "std_string.i"
%include "std_wstring.i"
%include "std_vector.i"
%include "typemaps.i"
%include "wchar.i"
#ifdef SWIGPYTHON   // --------------------------------------------------------------------- PYTHON
%include "python/cwstring.i"
#endif              // --------------------------------------------------------------------- PYTHON
%include "cstring.i"
%include "cpointer.i"


//
// For each class, keep updated the  A, B, C sections: 
// 


//
// A- ENABLE SHARED POINTERS
//
// Note that this must be done for almost all objects (not only those that are
// handled by shered pointers in C++, but all their chidren and parent classes. It
// is enough that a single class in an inheritance tree uses %shared_ptr, and all other in the 
// tree must be promoted to %shared_ptr too).

//%ignore irr::scene::ISceneNode;

%shared_ptr(chrono::irrlicht::ChIrrNodeShape)
%shared_ptr(chrono::irrlicht::ChIrrNodeModel)
%shared_ptr(chrono::irrlicht::ChVisualSystemIrrlicht)

//
// B- INCLUDE HEADERS
//
//
// 1) 
//    When including with %include all the .i files, make sure that 
// the .i of a derived class is included AFTER the .i of
// a base class, otherwise SWIG is not able to build the type
// infos. 
//
// 2)
//    Then, this said, if one member function in Foo_B.i returns
// an object of Foo_A.i (or uses it as a parameter) and yet you must %include
// A before B, ex.because of rule 1), a 'forward reference' to A must be done in
// B by. Seems that it is enough to write 
//  mynamespace { class myclass; }
// in the .i file, before the %include of the .h, even if already forwarded in .h


// WARNING: the drawChFunction is not working properly since it cannot recognize that ChFunction_XXX is derived from ChFunction

%import(module="pychrono.core") "chrono_swig/interface/core/ChClassFactory.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChVector3.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChMatrix.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChCoordsys.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChFrame.i"
// %import(module="pychrono.core") "../../../chrono/functions/ChFunction.h"
// %import(module="pychrono.core") "chrono_swig/interface/core/ChFunction.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChPhysicsItem.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChVisualMaterial.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChVisualShape.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChVisualModel.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChColor.i"
%import(module="pychrono.core") "chrono_swig/interface/core/ChSystem.i"

#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

%csmethodmodifiers irr::scene::ICameraSceneNode::OnEvent "public"

#endif             // --------------------------------------------------------------------- CSHARP

%include "IReferenceCounted.h"                         //// RADU: REMOVE?
%include "IImage.h"                                    //// RADU: REMOVE?
%include "IImageWriter.h"                              //// RADU: REMOVE?
%ignore irr::io::createWriteFile;                      //// RADU: REMOVE?
%include "IWriteFile.h"                                //// RADU: REMOVE?
%include "irrTypes.h"
%include "vector2d.h"
%template(vector2df) irr::core::vector2d<irr::f32>;
%template(vector2di) irr::core::vector2d<irr::s32>;
%include "dimension2d.h"
%template(dimension2du) irr::core::dimension2d<irr::u32>;
%include "vector3d.h"
%template(vector3df) irr::core::vector3d<irr::f32>;
%template(vector3di) irr::core::vector3d<irr::s32>;
%include "rect.h"
%template(rectf) irr::core::rect<irr::f32>;
%template(recti) irr::core::rect<irr::s32>;
%include "SColor.h"
%include "SMaterial.h"
%include "SMaterialLayer.h"
%include "IReferenceCounted.h"
%include "IImage.h"
%include "IImageWriter.h"
%ignore irr::io::createWriteFile;
%include "IWriteFile.h"
%include "IVideoDriver.h"
%include "IEventReceiver.h"
%include "ISceneNode.h"
%include "ICameraSceneNode.h"
%include "IrrlichtDevice.h"
%include "IMeshSceneNode.h"
%include "ISceneManager.h"
%include "IGUIEnvironment.h"



%ignore chrono::irrlicht::ScreenQuadCB;
%include "../../../chrono_irrlicht/ChIrrEffects.h"
%include "../../../chrono_irrlicht/ChIrrTools.h"
%include "../../../chrono_irrlicht/ChIrrNodeShape.h"    
%include "../../../chrono_irrlicht/ChIrrNodeModel.h"    

%include "ChVisualSystemIrrlicht.i"

//
// C- CASTING OF SHARED POINTERS
// 
// This is not automatic in Python + SWIG, except if one uses the 
// %downcast_output_sharedptr(...) macro, as above, but this causes
// a lot of code bloat. 
// Alternatively, in the following we create a set of Python-side
// functions to perform casting by hand, thank to the macro 
// %DefSharedPtrDynamicCast(base,derived). 
// Do not specify the "chrono::" namespace before base or derived!
// Later, in python, you can do the following:
//  myvis = chrono.CastToChVisualizationShared(myasset)
//  print ('Could be cast to visualization object?', !myvis.IsNull())



#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON


//
// ADDITIONAL C++ FUNCTIONS / CLASSES THAT ARE USED ONLY FOR PYTHON WRAPPER
//

/*
%inline %{


%}
*/

// Add function to support bytes exporting
%extend  irr::video::IImage{
		%cstring_output_allocate_size(char **buffer, unsigned int *size, free(*$1) ); 
		void get_img_bytes(char **buffer, unsigned int *size) 
					{
						*size = self->getImageDataSizeInBytes();
						*buffer = (char*)malloc(*size); 
						//char* dest = (char*)self->lock();
						strcpy(*buffer,  (char*)self->lock());
						self->unlock();
						
					}
		};
//
// ADD PYTHON CODE
//

/*
%pythoncode %{

%}
*/

#endif // --------------------------------------------------------------------- PYTHON


