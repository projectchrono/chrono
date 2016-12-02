#pragma once

#include "chrono_ogre/ChOgreApi.h"
#include "chrono_ogre/Graphics/ChOgreBody.h"


namespace chrono{
namespace ChOgre {

class CHOGRE_DLL_TAG ChOgreBodyHandle {
  public:
    ChOgreBodyHandle();
    ChOgreBodyHandle(const ChOgreBodyHandle& other);
    ChOgreBodyHandle(ChOgreBodyHandle&& other);
    ChOgreBodyHandle(ChOgreBody& Body);
    ChOgreBodyHandle(ChOgreBodySharedPtr& BodyPtr);
    ~ChOgreBodyHandle();

    ChOgreBodyHandle& operator=(const ChOgreBodyHandle& other);
    ChOgreBodyHandle& operator=(ChOgreBodyHandle&& other);

    std::shared_ptr<ChBody> operator->();
    std::shared_ptr<ChBody> getChBody();
    ChOgreBody& body();
    void setBodyPtr(ChOgreBodySharedPtr& BodyPtr);

  private:
    ChOgreBodySharedPtr m_pBody;
};
}
}
