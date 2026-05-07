%extend chrono::ChConvexDecompositionHACD
{
  // Extend writing of convex decomposition result by passing string instead of std::ofstream
  void WriteConvexHullsAsWavefrontObj(const std::string& path) {
    std::ofstream fileo(path.c_str());
    if (!fileo.is_open())
      return;
    $self->WriteConvexHullsAsWavefrontObj(fileo);
  }

  void WriteConvexHullsAsChullsFile(const std::string& path) {
    std::ofstream fileo(path.c_str());
    if (!fileo.is_open())
      return;
    $self->WriteConvexHullsAsChullsFile(fileo);
  }
}

%extend chrono::ChConvexDecompositionHACDv2 
{
  // Extend writing of convex decomposition result by passing string instead of std::ofstream
  void WriteConvexHullsAsWavefrontObj(const std::string& path) {
    std::ofstream fileo(path.c_str());
    if (!fileo.is_open())
      return;
    $self->WriteConvexHullsAsWavefrontObj(fileo);
  }

  void WriteConvexHullsAsChullsFile(const std::string& path) {
    std::ofstream fileo(path.c_str());
    if (!fileo.is_open())
      return;
    $self->WriteConvexHullsAsChullsFile(fileo);
  }
}


%{

/* Includes the header in the wrapper code */
#include "chrono/collision/ChConvexDecomposition.h"

%}


/* Parse the header file to generate wrappers */
%include "../../../chrono/collision/ChConvexDecomposition.h"