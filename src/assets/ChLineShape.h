//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// Author: A.Tasora

#ifndef CHLINESHAPE_H
#define CHLINESHAPE_H


#include "assets/ChVisualization.h"
#include "geometry/ChCLine.h"
#include "geometry/ChCLineSegment.h"

namespace chrono {

/// Class for referencing a ChLine that can be 
/// visualized in some way.

class ChApi ChLineShape: public ChVisualization {

	protected:
		//
		// DATA
		//
		ChSharedPtr<geometry::ChLine> gline;

	public:
		//
		// CONSTRUCTORS
		//

		ChLineShape() {
			// default path
			gline = ChSharedPtr< geometry::ChLine > (new geometry::ChLineSegment );
		}
		;
		ChLineShape(ChSharedPtr<geometry::ChLine>& mline) :
				gline(mline) {
		}
		;

		virtual ~ChLineShape() {
		}
		;

		//
		// FUNCTIONS
		//

		// Access the line geometry
		ChSharedPtr<geometry::ChLine> GetLineGeometry() {
			return gline;
		}

		// Set the line geometry
		void SetLineGeometry(ChSharedPtr<geometry::ChLine> mline) {
			gline = mline;
		}

};

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

}// END_OF_NAMESPACE____

#endif
