#ifndef CREATE_SKYBOX_H
#define CREATE_SKYBOX_H

#include <vsg/all.h>
#include <vsgXchange/all.h>

namespace chrono {
    namespace vsg3d {
        vsg::ref_ptr<vsg::Node> createSkybox(const vsg::Path& filename, vsg::ref_ptr<vsg::Options> options);
    }
}

#endif
