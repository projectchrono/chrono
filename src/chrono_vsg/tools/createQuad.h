#ifndef CREATE_QUAD_H
#define CREATE_QUAD_H

#include <iostream>
#include <vsg/all.h>

#include <vsgXchange/all.h>

namespace chrono {
    namespace vsg3d {
        vsg::ref_ptr<vsg::Node> createQuad(const vsg::vec3 &origin, const vsg::vec3 &horizontal, const vsg::vec3 &vertical, vsg::ref_ptr<vsg::Data> sourceData = {});
    }
}
#endif

