#include "ChOgreLightHandle.h"

namespace ChOgre {

ChOgreLightHandle::ChOgreLightHandle() {}

ChOgreLightHandle::ChOgreLightHandle(const ChOgreLightHandle& other) {
    m_pLight = other.m_pLight;
}

ChOgreLightHandle::ChOgreLightHandle(ChOgreLightHandle&& other) {
    m_pLight = std::move(other.m_pLight);
}

ChOgreLightHandle::ChOgreLightHandle(ChOgreLight& Light) {
    m_pLight = ChOgreLightSharedPtr(&Light);
}

ChOgreLightHandle::ChOgreLightHandle(ChOgreLightSharedPtr& LightPtr) {
    m_pLight = LightPtr;
}

ChOgreLightHandle::~ChOgreLightHandle() {}

ChOgreLightHandle& ChOgreLightHandle::operator=(const ChOgreLightHandle& other) {
    if (this != &other) {
        m_pLight = other.m_pLight;
    }

    return *this;
}

ChOgreLightHandle& ChOgreLightHandle::operator=(ChOgreLightHandle&& other) {
    if (this != &other) {
        m_pLight = std::move(other.m_pLight);
    }

    return *this;
}

ChOgreLightSharedPtr ChOgreLightHandle::operator->() {
    return m_pLight;
}

ChOgreLightSharedPtr ChOgreLightHandle::light() {
    return m_pLight;
}

void ChOgreLightHandle::setLightPtr(ChOgreLightSharedPtr& LightPtr) {
    m_pLight = LightPtr;
}
}