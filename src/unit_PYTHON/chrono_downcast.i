//
// A basic mechanism to provide downcasting of base pointers
// to the specialized types, when polimorphic objects return
// from a function.
// Parameters: base class, inherited, inherited, inherited, ...
//  Example:
// %downcast_output (chrono::ChFunction, chrono::ChFunction_Sine, chrono::ChFunction_Const)


// The following would move common code in Downcast_function() to
// avoid code bloat, but it does not work because $descriptor  does not
// work outside %typemap, and %descriptor gives a slightly different result..

/*
%define %_populate_(Type) 
	if ( typeid(*cppobject)==typeid(Type) )
		return (SWIG_NewPointerObj(%as_voidptr(cppobject),%descriptor(Type *), 0 | 0));  
%enddef

%define %downcast_output(OutType,Types...)

%{
	SWIGRUNTIME PyObject* Downcast_function( OutType * cppobject)
	{
		%formacro(%_populate_, Types)
		 // default: return base class
		return (SWIG_NewPointerObj(%as_voidptr(cppobject), %descriptor(OutType *), 0 | 0));
	};
%}

%typemap(out) OutType*, OutType& {
	$result=TEST_ALE($1);
}

%enddef
*/


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



