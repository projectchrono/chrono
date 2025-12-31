// A basic interface file to generate the C# vsg module
// include any other .i files as needed
// mimics the python approach

#pragma SWIG nowarn=314
#pragma SWIG nowarn=401
#pragma SWIG nowarn=451
#pragma SWIG nowarn=503
#pragma SWIG nowarn=516
#pragma SWIG nowarn=520
#pragma SWIG nowarn=833

%module(directors="1") chrono_vsg
%include "chrono_swig/interface/vsg/ChModuleVsg.i"
