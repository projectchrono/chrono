#ifndef VSG_ASYS_H
#define VSG_ASYS_H

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

// Line based shape to symbolize a body's reference frame. It appears once per body.
namespace chrono {
    namespace vsg3d {
        class CH_VSG_API VSGAsys {
        public:
            VSGAsys(std::shared_ptr<ChBody> body, std::shared_ptr<ChAsset> asset) {
                m_body = body;
                m_asset = asset;
            }

            void genSubgraph(vsg::ref_ptr<vsg::Switch> parentgraph, vsg::ref_ptr<vsg::MatrixTransform> tf);

        private:
            std::shared_ptr<ChBody> m_body;
            std::shared_ptr<ChAsset> m_asset;
        };
    }
}

#endif