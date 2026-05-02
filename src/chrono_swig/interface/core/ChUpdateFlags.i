// Without explicit byte casts, SWIG emits uint literals
// Tell SWIG to emit the right byte cast for each value:
%feature("cs:constvalue", "(byte)0") chrono::UpdateFlags::NONE;
%feature("cs:constvalue", "(byte)(1u << 0)") chrono::UpdateFlags::DYNAMICS;
%feature("cs:constvalue", "(byte)(1u << 1)") chrono::UpdateFlags::JACOBIANS;
%feature("cs:constvalue", "(byte)(1u << 2)") chrono::UpdateFlags::VISUAL_ASSETS;
%feature("cs:constvalue", "(byte)((1u << 0) | (1u << 1) | (1u << 2))") chrono::UpdateFlags::UPDATE_ALL;
%feature("cs:constvalue", "(byte)((1u << 0) | (1u << 1))") chrono::UpdateFlags::UPDATE_ALL_NO_VISUAL;
// Include UpdateFlags so SWIG can resolve it
%include "../../../chrono/physics/ChUpdateFlags.h"

// Make UpdateFlags visible in the fea namespace
%inline %{
namespace chrono {
namespace fea {
typedef chrono::UpdateFlags UpdateFlags;
}
}
%}
