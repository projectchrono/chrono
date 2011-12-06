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



%define %DefChSharedPtrCast(__ChTYPE__, __ChTYPE_BASE__)
%types(chrono::ChSharedPtr<__ChTYPE__> = chrono::ChSharedPtr<__ChTYPE_BASE__>);
%enddef