//
// Use as 
//   %ChSharedPtr(shared_class_name, class_to_share)
//
// for example
//   %ChSharedPtr(ChBodyShared, ChBody)


%include "../core/ChSmartpointers.h"


// #### THE MACRO ####

%define %DefChSharedPtr(__CHTYPE__Shared, __CHTYPE__)

// Tell SWIG to add a chrono::ChSharedPtr class 
%template(__CHTYPE__Shared) chrono::ChSharedPtr<__CHTYPE__>;

// Trick to avoid confusion about memory ownership: redefine the
// original chrono::ChSharedPtr constructor (that SWIG made according
// to the cpp equivalent) and allow only one type of construction with 
// no arguments, that also instances one object.

%pythoncode %{
def __CHTYPE__Shared_custominit(self):
	newsharedobj = __CHTYPE__()
	newsharedobj.thisown =0
	#print 'Debug: init __CHTYPE__Shared '
	__CHTYPE__Shared.__cppinit__(self, newsharedobj)

setattr(__CHTYPE__Shared, "__cppinit__", __CHTYPE__Shared.__init__)
setattr(__CHTYPE__Shared, "__init__", __CHTYPE__Shared_custominit)

%}

%enddef



%define %DefChSharedPtrCast(__CHTYPE__, __CHTYPE_BASE__)
%types(chrono::ChSharedPtr<__CHTYPE__> = chrono::ChSharedPtr<__CHTYPE_BASE__>);
%enddef