#include "ChOgreBodyHandle.h"

namespace ChOgre {

ChOgreBodyHandle::ChOgreBodyHandle() {}

ChOgreBodyHandle::ChOgreBodyHandle(const ChOgreBodyHandle& other) {
    m_pBody = other.m_pBody;
}

ChOgreBodyHandle::ChOgreBodyHandle(ChOgreBodyHandle&& other) {
    m_pBody = std::move(other.m_pBody);
}

ChOgreBodyHandle::ChOgreBodyHandle(ChOgreBody& Body) {
    m_pBody = ChOgreBodySharedPtr(&Body);
}

ChOgreBodyHandle::ChOgreBodyHandle(ChOgreBodySharedPtr& BodyPtr) {
    m_pBody = BodyPtr;
}

ChOgreBodyHandle::~ChOgreBodyHandle() {}

ChOgreBodyHandle& ChOgreBodyHandle::operator=(const ChOgreBodyHandle& other) {
    if (this != &other) {
        m_pBody = other.m_pBody;
    }

    return *this;
}

ChOgreBodyHandle& ChOgreBodyHandle::operator=(ChOgreBodyHandle&& other) {
    if (this != &other) {
        m_pBody = std::move(other.m_pBody);
    }

    return *this;
}

chrono::ChSharedPtr<ChBody> ChOgreBodyHandle::operator->() {
    return m_pBody->getChBody();
}

chrono::ChSharedPtr<ChBody> ChOgreBodyHandle::ChBody() {
    return m_pBody->getChBody();
}

ChOgreBody& ChOgreBodyHandle::body() {
    return *m_pBody;
}

void ChOgreBodyHandle::setBodyPtr(ChOgreBodySharedPtr& BodyPtr) {
    m_pBody = BodyPtr;
}
}