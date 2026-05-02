#pragma SWIG nowarn=319
#pragma SWIG nowarn=401
#pragma SWIG nowarn=402

%module(directors="1", threads="1") sensor
// We don't want to enable threads for all Python -> C++ calls, which harm performance
// We do want to check for GIL for all C++ -> Python calls
// They are not frequent and we don't know if they are calling from foreign thread.
%feature("nothreadallow");

%include "chrono_swig/interface/sensor/ChModuleSensor.i"
