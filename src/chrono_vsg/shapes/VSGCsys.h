#ifndef VSG_CSYS_H
#define VSG_CSYS_H

#include <iostream>
#include <string>
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/assets/ChAsset.h"
#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChTexture.h"
#include "chrono_vsg/core/ChApiVSG.h"

#include <vsg/all.h>

namespace chrono {
    namespace vsg3d {
        class CH_VSG_API VSGCsys {
        public:
                VSGCsys();
                void genSubgraph(vsg::ref_ptr<vsg::Switch> parentgraph);
        };
    }
}

#endif