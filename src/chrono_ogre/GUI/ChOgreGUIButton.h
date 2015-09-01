#pragma once

#include "chrono_ogre/Input/ChOgre_SDLInputHandler.h"
#include "ChOgreGUICallback.h"
#include "ChOgreGUIElement.h"

namespace ChOgre {

class CHOGRE_DLL_TAG ChOgreGUIButton : public ChOgreGUIElement {
  public:
    ChOgreGUIButton();
    ChOgreGUIButton(MyGUI::Gui* GUI);
    ChOgreGUIButton(const ChFloat3& Position, const ChFloat3& Size, MyGUI::Gui* GUI);
    ~ChOgreGUIButton();

    virtual void setColor(float r, float g, float b);
    virtual void setTextColor(float r, float g, float b);
    virtual void setText(const std::string& Text);
    virtual void setFont(const std::string& Name);
    virtual void setPosition(const ChFloat3& Position);
    virtual void setSize(const ChFloat3& Size);
    virtual void update();

    virtual void setClickCallback(ChOgreGUIClickCallback& Callback);
    virtual void emptyClickCallback();

    virtual void setPressCallback(ChOgreGUIPressCallback& Callback);
    virtual void emptyPressCallback();

    virtual void setReleaseCallback(ChOgreGUIReleaseCallback& Callback);
    virtual void emptyReleaseCallback();

    virtual ChFloat3 getPosition() { return ChFloat3((float)m_pButton->getLeft(), (float)m_pButton->getTop(), 0.f); };
    virtual ChFloat3 getSize() { return ChFloat3((float)m_pButton->getWidth(), (float)m_pButton->getHeight(), 0.f); }

  protected:
    bool m_db;
    bool m_pressed;

    MyGUI::Button* m_pButton;

  private:
};

typedef std::unique_ptr<ChOgreGUIButton> ChOgreGUIButtonPtr;
}