%{

/* Includes the header in the wrapper code */
#include "chrono_fsi/sph/ChFsiDefinitionsSPH.h"

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

%}

%rename(PeriodicSide_NONE) chrono::fsi::sph::PeriodicSide::NONE;
%rename(PeriodicSide_X) chrono::fsi::sph::PeriodicSide::X;
%rename(PeriodicSide_Y) chrono::fsi::sph::PeriodicSide::Y;
%rename(PeriodicSide_Z) chrono::fsi::sph::PeriodicSide::Z;
%rename(PeriodicSide_ALL) chrono::fsi::sph::PeriodicSide::ALL;

%rename(BoxSide_NONE) chrono::fsi::sph::BoxSide::NONE;
%rename(BoxSide_X_POS) chrono::fsi::sph::BoxSide::X_POS;
%rename(BoxSide_X_NEG) chrono::fsi::sph::BoxSide::X_NEG;
%rename(BoxSide_Y_POS) chrono::fsi::sph::BoxSide::Y_POS;
%rename(BoxSide_Y_NEG) chrono::fsi::sph::BoxSide::Y_NEG;
%rename(BoxSide_Z_POS) chrono::fsi::sph::BoxSide::Z_POS;
%rename(BoxSide_Z_NEG) chrono::fsi::sph::BoxSide::Z_NEG;
%rename(BoxSide_ALL) chrono::fsi::sph::BoxSide::ALL;

%rename(CylSide_NONE) chrono::fsi::sph::CylSide::NONE;
%rename(CylSide_SIDE_INT) chrono::fsi::sph::CylSide::SIDE_INT;
%rename(CylSide_SIDE_EXT) chrono::fsi::sph::CylSide::SIDE_EXT;
%rename(CylSide_Z_NEG) chrono::fsi::sph::CylSide::Z_NEG;
%rename(CylSide_Z_POS) chrono::fsi::sph::CylSide::Z_POS;
%rename(CylSide_ALL) chrono::fsi::sph::CylSide::ALL;

/* Parse the header file to generate wrappers */
%include "../../../chrono_fsi/sph/ChFsiDefinitionsSPH.h"    
