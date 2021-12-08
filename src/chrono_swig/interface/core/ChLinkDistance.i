#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

// Extend ChLinkDistance with Initialize functions that take two ChBody (not ChBodyFrame)
%extend chrono::ChLinkDistance
{
  void Initialize(std::shared_ptr<ChBody> body1,
                  std::shared_ptr<ChBody> body2,
                  bool pos_are_relative,
                  ChVector<double> pos1,
                  ChVector<double> pos2) {
     $self->Initialize(std::dynamic_pointer_cast<ChBodyFrame>(body1),
                       std::dynamic_pointer_cast<ChBodyFrame>(body2),
                       pos_are_relative, pos1, pos2,
                       true, 0);
  }
}

#endif             // --------------------------------------------------------------------- CSHARP

%{
#include "chrono/physics/ChLinkDistance.h"
%}
 
// Tell SWIG about parent class in Python
%import "ChLinkLock.i"

// Parse the header file to generate wrappers
%include "../../../chrono/physics/ChLinkDistance.h"  
