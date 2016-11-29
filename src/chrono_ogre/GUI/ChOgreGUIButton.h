#pragma once

#include "chrono_ogre/Input/ChOgre_SDLInputHandler.h"
#include "ChOgreGUICallback.h"
#include "ChOgreGUIElement.h"

namespace chrono{
namespace ChOgre {

class CHOGRE_DLL_TAG ChOgreGUIButton : public ChOgreGUIElement {
  public:
    ChOgreGUIButton();
    ChOgreGUIButton(MyGUI::Gui* GUI);
    ChOgreGUIButton(const ChVector2<>& Position, const ChVector2<>& Size, MyGUI::Gui* GUI);
    ~ChOgreGUIButton();

    virtual void setColor(float r, float g, float b);
    virtual void setTextColor(float r, float g, float b);
    virtual void setText(const std::string& Text);
    virtual void setFont(const std::string& Name);
    virtual void setPosition(const ChVector2<>& Position);
    virtual void setSize(const ChVector2<>& Size);
    virtual void update();

    virtual void setClickCallback(ChOgreGUIClickCallback& Callback);
    virtual void emptyClickCallback();

    virtual void setPressCallback(ChOgreGUIPressCallback& Callback);
    virtual void emptyPressCallback();

    virtual void setReleaseCallback(ChOgreGUIReleaseCallback& Callback);
    virtual void emptyReleaseCallback();

    virtual ChVector2<> getPosition() { return ChVector2<>((double)m_pButton->getLeft(), (double)m_pButton->getTop()); };
    virtual ChVector2<> getSize() { return ChVector2<>((double)m_pButton->getWidth(), (double)m_pButton->getHeight()); }

  protected:
    bool m_db;
    bool m_pressed;

    MyGUI::Button* m_pButton;

  private:
};

typedef std::unique_ptr<ChOgreGUIButton> ChOgreGUIButtonPtr;
}
}
