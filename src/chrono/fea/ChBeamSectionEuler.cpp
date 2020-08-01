// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================


#include "chrono/fea/ChBeamSectionEuler.h"


namespace chrono {
namespace fea {



	ChBeamSectionEulerEasyRectangular::ChBeamSectionEulerEasyRectangular(double width_y, double width_z, double myE, double mydensity)
	{
		this->SetYoungModulus(myE);
		this->SetDensity(mydensity);
		this->SetAsRectangularSection(width_y, width_z);
	}

	ChBeamSectionEulerEasyCircular::ChBeamSectionEulerEasyCircular(double diameter, double myE, double mydensity)
	{
		this->SetYoungModulus(myE);
		this->SetDensity(mydensity);
		this->SetAsCircularSection(diameter);
	}

}  // end namespace fea
}  // end namespace chrono

