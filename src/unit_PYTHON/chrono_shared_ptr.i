//
// Use as 
//   %ChSharedPtr(shared_class_name, class_to_share)
//
// for example
//   %ChSharedPtr(ChBodyShared, ChBody)


%include "../core/ChSmartpointers.h"


// #### THE MACRO ####

%define %DefChSharedPtr(__ChTYPE__Shared, __ChTYPE__)

// Tell SWIG to add a chrono::ChSharedPtr class 
%template(__ChTYPE__Shared) chrono::ChSharedPtr<__ChTYPE__>;

// Trick to avoid confusion about memory ownership: redefine the
// original chrono::ChSharedPtr constructor (that SWIG made according
// to the cpp equivalent) and allow only one type of construction with 
// no arguments, that also instances one object.

%pythoncode %{
def __ChTYPE__Shared_custominit(self):
	newsharedobj = __ChTYPE__()
	newsharedobj.thisown =0
	#print 'Debug: init __ChTYPE__Shared '
	__ChTYPE__Shared.__cppinit__(self, newsharedobj)

setattr(__ChTYPE__Shared, "__cppinit__", __ChTYPE__Shared.__init__)
setattr(__ChTYPE__Shared, "__init__", __ChTYPE__Shared_custominit)

%}

%enddef


// ### MACRO FOR SETTING UP INHERITANCE of ChSharedPtr ###

%define %DefChSharedPtrCast(__ChTYPE__, __ChTYPE_BASE__)

%types(chrono::ChSharedPtr<__ChTYPE__> = chrono::ChSharedPtr<__ChTYPE_BASE__>);


%enddef


//
// Utility macro for downcasting function outputs with ChSharedPtr pointers. 
//
// Parameters: base class, inherited, inherited, ... (only the classes wrapped by the chared ptr)
// Example:
//   %downcast_output_sharedptr(chrono::ChAsset, chrono::ChVisualization, chrono::ChObjShapeFile)

//  (works, but makes some code bloat...)

%define %_shpointers_dispatch(Type) 
	if ( typeid(*((result).get_ptr()))==typeid(Type) )
		return(SWIG_NewPointerObj((new chrono::ChSharedPtr<Type>(static_cast< const chrono::ChSharedPtr<Type>& >(result))), $descriptor(chrono::ChSharedPtr<Type> *), SWIG_POINTER_OWN |  0 ));
%enddef

%define %_shpointers_dispatchR(Type) 
	if ( typeid(*((result)->get_ptr()))==typeid(Type) )
		return(SWIG_NewPointerObj((new chrono::ChSharedPtr<Type>(static_cast< const chrono::ChSharedPtr<Type>& >(*result))), $descriptor(chrono::ChSharedPtr<Type> *), SWIG_POINTER_OWN |  0 ));
%enddef

%define %_shpointers_dispatchP(Type) 
	if ( typeid(*((result).get_ptr()))==typeid(Type) )
		return(SWIG_NewPointerObj((new chrono::ChSharedPtr<Type>(static_cast< const chrono::ChSharedPtr<Type>& >(*result))), $descriptor(chrono::ChSharedPtr<Type> *), SWIG_POINTER_OWN |  0 ));
%enddef


//***THE MACRO***
%define %downcast_output_sharedptr(OutType,Types...)
%typemap(out) chrono::ChSharedPtr<OutType> {
  %formacro(%_shpointers_dispatch, Types)
    return(SWIG_NewPointerObj((new chrono::ChSharedPtr<OutType>(static_cast< const chrono::ChSharedPtr<OutType>& >(result))), $descriptor(chrono::ChSharedPtr<OutType> *), SWIG_POINTER_OWN |  0 ));
}
%typemap(out) chrono::ChSharedPtr<OutType>& {
  %formacro(%_shpointers_dispatchR, Types)
    return(SWIG_NewPointerObj((new chrono::ChSharedPtr<OutType>(static_cast< const chrono::ChSharedPtr<OutType>& >(*result))), $descriptor(chrono::ChSharedPtr<OutType> *), SWIG_POINTER_OWN |  0 ));
}
%typemap(out) chrono::ChSharedPtr<OutType>* {
	if ($owner) delete $1;
  %formacro(%_shpointers_dispatchR, Types)
    return(SWIG_NewPointerObj((new chrono::ChSharedPtr<OutType>(static_cast< const chrono::ChSharedPtr<OutType>& >(*result))), $descriptor(chrono::ChSharedPtr<OutType> *), SWIG_POINTER_OWN |  0 ));
}
%enddef




