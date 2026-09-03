// #######################################################
// OPERATORS THAT ARE DELIBERATELY NOT WRAPPED
//
// SWIG already declines to wrap the operators below, and says so once per module
// that parses the declaring header - which is every module, since these are core
// value types. Stating the intent here turns that repeated report into a
// deliberate choice, and costs nothing: SWIG's behavior is unchanged, because it
// was skipping these already.
//
// Included by every module's master interface file, next to chrono_cast.i, so it
// is in effect before any header is parsed.

// Assignment. There is nothing to map this onto: in Python and in C#, assignment
// rebinds a name rather than invoking a member of the assigned-to object. Affects
// ChVector2, ChVector3, ChQuaternion, ChFrame, ChMatrix33, ChColor, ChState,
// ChAssembly and ChVisualShapeFEA. (SWIG warning 362.)
%ignore *::operator=;

// Subscripting. The C++ operator cannot be wrapped, so ChVector2.i, ChVector3.i
// and ChQuaternion.i supply __getitem__, __setitem__ and __len__ via %extend -
// which is what SWIG's "consider using %extend" advice asks for. This ignore is
// still needed alongside them: it is what stops SWIG reporting the unwrappable
// operator itself, which it does whether or not a replacement exists.
// (SWIG warning 389.)
%ignore *::operator[];

// Insertion. SWIG cannot wrap it under this name in any target language. The
// only one it currently parses is ChGnuPlot::operator<<, which the header itself
// documents as equivalent to SetCommand() - already wrapped, so nothing is lost.
// (SWIG warning 503.)
%ignore *::operator<<;
