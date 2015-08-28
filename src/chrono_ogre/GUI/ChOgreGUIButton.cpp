#include "ChOgreGUIButton.h"

namespace ChOgre {

ChOgreGUIButton::ChOgreGUIButton() {
    m_pGUI = nullptr;
}

ChOgreGUIButton::ChOgreGUIButton(MyGUI::Gui* GUI) {
    m_pGUI = GUI;
}

ChOgreGUIButton::ChOgreGUIButton(const ChFloat3& Position, const ChFloat3& Size, MyGUI::Gui* GUI) {
    m_pGUI = GUI;

    m_pButton = m_pGUI->createWidgetReal<MyGUI::Button>("Button", Position.x, Position.y, Size.x, Size.y,
                                                        MyGUI::Align::Center, "Main");
    m_pButton->setDepth(int(Position.z));

    m_db = true;
    m_pressed = false;
}

ChOgreGUIButton::~ChOgreGUIButton() {
    m_pGUI->destroyWidget(m_pButton);
}

void ChOgreGUIButton::setColor(float r, float g, float b) {
    m_pButton->setColour(MyGUI::Colour(r, g, b));
}

void ChOgreGUIButton::setTextColor(float r, float g, float b) {
    m_pButton->setTextColour(MyGUI::Colour(r, g, b));
}

void ChOgreGUIButton::setText(const std::string& Text) {
    m_pButton->setCaption(Text);
}

void ChOgreGUIButton::setFont(const std::string& Name) {
    m_pButton->setFontName(Name);
}

void ChOgreGUIButton::setPosition(const ChFloat3& Position) {
    m_pButton->setRealPosition(Position.x, Position.y);
    m_pButton->setDepth(int(Position.z));
}

void ChOgreGUIButton::setSize(const ChFloat3& Size) {
    m_pButton->setRealSize(Size.x, Size.y);
}

void ChOgreGUIButton::update() {}

void ChOgreGUIButton::setClickCallback(ChOgreGUIClickCallback& Callback) {
    m_pButton->eventMouseButtonClick = MyGUI::newDelegate(&Callback, &ChOgreGUIClickCallback::_c);
}

void ChOgreGUIButton::emptyClickCallback() {
    m_pButton->eventMouseButtonClick.clear();
}

void ChOgreGUIButton::setPressCallback(ChOgreGUIPressCallback& Callback) {
    m_pButton->eventMouseButtonPressed = MyGUI::newDelegate(&Callback, &ChOgreGUIPressCallback::_c);
}

void ChOgreGUIButton::emptyPressCallback() {
    m_pButton->eventMouseButtonPressed.clear();
}

void ChOgreGUIButton::setReleaseCallback(ChOgreGUIReleaseCallback& Callback) {
    m_pButton->eventMouseButtonReleased = MyGUI::newDelegate(&Callback, &ChOgreGUIReleaseCallback::_c);
}

void ChOgreGUIButton::emptyReleaseCallback() {
    m_pButton->eventMouseButtonReleased.clear();
}
}