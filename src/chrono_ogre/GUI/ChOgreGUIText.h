/*
Author: Charles Ricchio
*/

#pragma once

#include "ChOgreGUIElement.h"
#include <core/ChVector.h>

namespace chrono{
namespace ChOgre {

class CHOGRE_DLL_TAG ChOgreGUIText : public ChOgreGUIElement {
  public:
    ChOgreGUIText();
    ChOgreGUIText(MyGUI::Gui* GUI);
    ChOgreGUIText(const ChVector2<>& Position, const ChVector2<>& Size, MyGUI::Gui* GUI);
    ~ChOgreGUIText();

    virtual void setColor(float r, float g, float b);
    virtual void setTextColor(float r, float g, float b);
    virtual void setText(const std::string& Text);
    virtual void setFont(const std::string& Name);
    virtual void setPosition(const ChVector2<>& Position);
    virtual void setSize(const ChVector2<>& Size);

    virtual ChVector2<> getPosition() { return ChVector2<>((double)m_pTextBox->getLeft(), (double)m_pTextBox->getTop()); };
    virtual ChVector2<> getSize() { return ChVector2<>((double)m_pTextBox->getWidth(), (double)m_pTextBox->getHeight()); }

  protected:
    MyGUI::TextBox* m_pTextBox;

    static unsigned int g_count;

  private:
};

typedef std::unique_ptr<ChOgreGUIText> ChOgreGUITextPtr;
}
}
