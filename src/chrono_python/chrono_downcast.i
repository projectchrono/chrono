// #######################################################
// MACRO FOR AUTOMATIC DOWNCASTING of raw pointers
//
// A basic mechanism to provide downcasting of base pointers
// to the specialized types, when polimorphic objects return
// from a function.
// Parameters: base class, inherited, inherited, inherited, ...
//  Example:
// %downcast_output (chrono::ChFunction, chrono::ChFunction_Sine, chrono::ChFunction_Const)




// This works, but makes some code bloat...

%define %_classes_dispatch(Type) 
	if ( typeid(*$1)==typeid(Type) )
		return (SWIG_NewPointerObj(%as_voidptr($1),$descriptor(Type *), $owner | %newpointer_flags));  
%enddef

%define %downcast_output(OutType,Types...)
%typemap(out) OutType*, OutType& {
  %formacro(%_classes_dispatch, Types)
    return(SWIG_NewPointerObj(%as_voidptr($1),$descriptor(OutType *), $owner | %newpointer_flags));
}
%enddef




// #######################################################
// MACRO FOR DOWNCASTING and generic casting of std::shared_ptr
// 
// This enables the manual DOWNCASTING (usually from base to derived,
// but could be also viceversa) by calling a python function.
// For example:  myvis = CastToChVisualizationShared(myasset)

%define %DefChSharedPtrDynamicDowncast(__CHTYPE_BASE__, __CHTYPE__)
%inline %{
  std::shared_ptr<__CHTYPE__> CastTo ## __CHTYPE__ ## Shared (std::shared_ptr<__CHTYPE_BASE__> in_obj) 
  {
	  return (std::dynamic_pointer_cast<__CHTYPE__>(in_obj));
  }
%}
%enddef






// #######################################################
// MACRO FOR AUTOMATIC DOWNCASTING of std::shared_ptr
// 
// Utility macro for enabling AUTOMATIC downcasting function 
// outputs with std::shared_ptr pointers. 
// Since this creates lot of code bloat, it is suggested to
// use it sparingly; prefer using the DefChSharedPtrDynamicDowncast
// trick when possible.
//
// Parameters: base class, inherited, inherited, ... (only the 
//    classes wrapped by the chared ptr)
//
// Example:
//   %downcast_output_sharedptr(chrono::ChAsset, chrono::ChVisualization, chrono::ChObjShapeFile)



%define %_shpointers_dispatch(Type) 
	if ( std::dynamic_pointer_cast<Type>(result) )
		return(SWIG_NewPointerObj((new std::shared_ptr<Type>(std::dynamic_pointer_cast<Type>(result) )), $descriptor(std::shared_ptr<Type> *), SWIG_POINTER_OWN |  0 ));
%enddef

%define %_shpointers_dispatchR(Type) 
	if ( std::dynamic_pointer_cast<Type>(*result) )
		return(SWIG_NewPointerObj((new std::shared_ptr<Type>(std::dynamic_pointer_cast<Type>(*result) )), $descriptor(std::shared_ptr<Type> *), SWIG_POINTER_OWN |  0 ));
%enddef

%define %_shpointers_dispatchP(Type) 
	if ( std::dynamic_pointer_cast<Type>(*result) )
		return(SWIG_NewPointerObj((new std::shared_ptr<Type>(std::dynamic_pointer_cast<Type>(*result) )), $descriptor(std::shared_ptr<Type> *), SWIG_POINTER_OWN |  0 ));
%enddef


//***THE MACRO***
%define %downcast_output_sharedptr(OutType,Types...)
%typemap(out) std::shared_ptr<OutType> {
  %formacro(%_shpointers_dispatch, Types)
    return(SWIG_NewPointerObj((new std::shared_ptr<OutType>(std::static_pointer_cast<OutType>(result) )), $descriptor(std::shared_ptr<OutType> *), SWIG_POINTER_OWN |  0 ));
}
%typemap(out) std::shared_ptr<OutType>& {
  %formacro(%_shpointers_dispatchR, Types)
    return(SWIG_NewPointerObj ((new std::shared_ptr<OutType>(std::static_pointer_cast<OutType>(*result) )), $descriptor(std::shared_ptr<OutType> *), SWIG_POINTER_OWN |  0 ));
}
%typemap(out) std::shared_ptr<OutType>* {
	if ($owner) delete $1;
  %formacro(%_shpointers_dispatchR, Types)
    return(SWIG_NewPointerObj  ((new std::shared_ptr<OutType>(std::static_pointer_cast<OutType>(*result) )), $descriptor(std::shared_ptr<OutType> *), SWIG_POINTER_OWN |  0 ));
}
%enddef