
%include "../core/ChSmartpointers.h"


// #######################################################
// MACRO FOR ENABLING SHARED POINTERS
//
// Shared pointers of type chrono::ChSharedPtr<myclass>
// are supported in this way. You must use this macro after
// the definition of the class, so that in Python you can
// use shared objects as other python objects. 
//
// Use as 
//   %DefChSharedPtr(shared_class_namespace, shared_class)
//
// for example
//   %DefChSharedPtr(chrono::fem::,ChMesh)
//
// Note 1: upcasting of shared pointers in python is not 
//  automatic as in c++, but you can make it automatic if 
//  you use the macro %DefChSharedPtrCast , see below.
// Note 2: downcasting of shared pointers is not automatic,
//  unless you use %downcast_output_sharedptr (not suggested
//  because of code bloat) or you enforce casting manually
//  using the %DefChSharedPtrDynamicDowncast macro, see below.

%define %DefChSharedPtr(__CHNAMESPACE__, __CHTYPE__)

// Tell SWIG to add a chrono::ChSharedPtr class 
%template(__CHTYPE__ ## Shared) chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__>;

// Trick to avoid confusion about memory ownership: redefine the
// original chrono::ChSharedPtr constructor (that SWIG made according
// to the cpp equivalent) and allow only one type of construction with 
// no arguments, that also instances one object.

%pythoncode %{
def __CHTYPE__ ## Shared_custominit(self,*args):
	newsharedobj = __CHTYPE__(*args)
	newsharedobj.thisown = 0
	__CHTYPE__ ## Shared.__cppinit__(self, newsharedobj)

setattr(__CHTYPE__ ## Shared, "__cppinit__", __CHTYPE__ ## Shared.__init__)
setattr(__CHTYPE__ ## Shared, "__init__", __CHTYPE__ ## Shared_custominit)

%}


// Typemaps. 
// Needed because the pointer casting returns new object (a temporary
// chrono::ChSharedPtr<> that must be used to carry 'type' infos, otherwise
// the default reinterpret_cast does not work straigt on shared ptr, since
// the embedded ptr is cast roughly, somethng that fails with classes with 
// multiple inheritance!)

// The conversion typemap for shared pointers, passed by value
%typemap(in) chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__> (void *argp = 0, int res = 0) {
  int newmem = 0;
  res = SWIG_ConvertPtrAndOwn($input, &argp, $descriptor(chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__ > *), %convertptr_flags, &newmem);
  if (!SWIG_IsOK(res)) {
    %argument_fail(res, "$type", $symname, $argnum); 
  }
  $1 = *(%reinterpret_cast(argp, chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__ > *) );
  if (SWIG_IsNewObj(res)) delete %reinterpret_cast(argp, chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__ > *);
}

// The conversion typemap for shared pointers, passed by value
%typemap(varin) chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__> {
  void *argp = 0; 
  int res = 0;
  int newmem = 0;
  res = SWIG_ConvertPtrAndOwn($input, &argp, $descriptor(chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__ > *), %convertptr_flags, &newmem);
  if (!SWIG_IsOK(res)) {
    %argument_fail(res, "$type", $symname, $argnum); 
  }
  $1 = *(%reinterpret_cast(argp, chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__ > *) );
  if (SWIG_IsNewObj(res)) delete %reinterpret_cast(argp, chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__ > *);
}


// The conversion typemap for shared pointers, passed by reference  (simple, defaults to Python refcount for the sh.pointer, no C::E refcount update)
%typemap(in) chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__> & (void *argp = 0, int res = 0, $*1_ltype tempshared) {
  int newmem = 0;
  res = SWIG_ConvertPtrAndOwn($input, &argp, $descriptor(chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__ > *), %convertptr_flags, &newmem);
  if (!SWIG_IsOK(res)) {
    %argument_fail(res, "$type", $symname, $argnum); 
  }
  if (newmem & SWIG_CAST_NEW_MEMORY) {
    if (argp) tempshared = *%reinterpret_cast(argp, $ltype);
    delete %reinterpret_cast(argp, $ltype);
    $1 = &tempshared;
  } else {
    $1 = (argp) ? %reinterpret_cast(argp, $ltype) : &tempshared;
  }
}

%typemap(varin) chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__> & %{
#error "varin typemap not implemented for reference to ChSharedPtr"
%}


// The conversion typemap for shared pointers, passed by pointer (simple, defaults to Python refcount for the sh.pointer, no C::E refcount update)
%typemap(in) chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__> * (void *argp = 0, int res = 0, $*1_ltype tempshared) {
  int newmem = 0;
  res = SWIG_ConvertPtrAndOwn($input, &argp, $descriptor(chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__ > *), %convertptr_flags, &newmem);
  if (!SWIG_IsOK(res)) {
    %argument_fail(res, "$type", $symname, $argnum); 
  }
  if (newmem & SWIG_CAST_NEW_MEMORY) {
    if (argp) tempshared = *%reinterpret_cast(argp, $ltype);
    delete %reinterpret_cast(argp, $ltype);
    $1 = &tempshared;
  } else {
    $1 = (argp) ? %reinterpret_cast(argp, $ltype) : &tempshared;
  }
}

%typemap(varin) chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__> * %{
#error "varin typemap not implemented for pointer to ChSharedPtr"
%}


// The conversion typemap for shared pointers, passed by reference to pointer
%typemap(in) chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__> *& (void *argp, int res = 0, chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__> tempshared, $*1_ltype temp = 0) {
  int newmem = 0;
  res = SWIG_ConvertPtrAndOwn($input, &argp, $descriptor(chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__> *), %convertptr_flags, &newmem);
  if (!SWIG_IsOK(res)) {
    %argument_fail(res, "$type", $symname, $argnum); 
  }
  if (argp) tempshared = *%reinterpret_cast(argp, $*ltype);
  if (newmem & SWIG_CAST_NEW_MEMORY) delete %reinterpret_cast(argp, $*ltype);
  temp = &tempshared;
  $1 = &temp;
}

%typemap(varin) chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__> *& %{
#error "varin typemap not implemented"
%}


// Typecheck typemaps
// Note: SWIG_ConvertPtr with void ** parameter set to 0 instead of using SWIG_ConvertPtrAndOwn, so that the casting 
// function is not called thereby avoiding a possible smart pointer copy constructor call when casting up the inheritance chain.
%typemap(typecheck,precedence=SWIG_TYPECHECK_POINTER,noblock=1) 
                      chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__>,
                      chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__> &,
                      chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__> *,
                      chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__> *& {
  int res = SWIG_ConvertPtr($input, 0, $descriptor(chrono::ChSharedPtr< __CHNAMESPACE__ ## __CHTYPE__> *), 0);
  $1 = SWIG_CheckState(res);
}


// To do? also typemaps for outputs? (see boost_shared_ptr.i)


%enddef



// #######################################################
// MACRO FOR UPCASTING of ChSharedPtr
//
// This enables the UPCASTING (from derived to base)
// that will happen automatically in Python. Ex. if you pass a 
// subclass to a function that requires base class.

%define %DefChSharedPtrCast(__CHTYPE__, __CHTYPE_BASE__)

%types(chrono::ChSharedPtr<__CHTYPE__> = chrono::ChSharedPtr<__CHTYPE_BASE__>)
%{
  *newmemory = SWIG_CAST_NEW_MEMORY;
  return (void *) new chrono::ChSharedPtr< __CHTYPE_BASE__ >(*(chrono::ChSharedPtr< __CHTYPE__ > *)$from);
%}

// (note the creation of a temp. shared ptr for casting without
// problems even with classes that have multiple inheritance)

%enddef



// #######################################################
// MACRO FOR DOWNCASTING and generic casting of ChSharedPtr
// 
// This enables the manual DOWNCASTING (usually from base to derived,
// but could be also viceversa) by calling a python function.
// For example:  myvis = CastToChVisualizationShared(myasset)

%define %DefChSharedPtrDynamicDowncast(__CHTYPE_BASE__, __CHTYPE__)
%inline %{
  chrono::ChSharedPtr<__CHTYPE__> CastTo ## __CHTYPE__ ## Shared (chrono::ChSharedPtr<__CHTYPE_BASE__> in_obj) 
  {
	if (in_obj.IsType<__CHTYPE__>())
		return chrono::ChSharedPtr<__CHTYPE__>(in_obj);
	else
		return chrono::ChSharedPtr<__CHTYPE__>(0);
  }
%}
%enddef




// #######################################################
// MACRO FOR AUTOMATIC DOWNCASTING of ChSharedPtr
// 
// Utility macro for enabling AUTOMATIC downcasting function 
// outputs with ChSharedPtr pointers. 
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




