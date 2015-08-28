/*
Author: Charles Ricchio
*/

#include "ChOgreGUIText.h"

namespace ChOgre {

unsigned int ChOgreGUIText::g_count = 0;

ChOgreGUIText::ChOgreGUIText() {
    m_pGUI = nullptr;
}

ChOgreGUIText::ChOgreGUIText(MyGUI::Gui* GUI) {
    m_pGUI = GUI;
}

ChOgreGUIText::ChOgreGUIText(const ChFloat3& Position, const ChFloat3& Size, MyGUI::Gui* GUI) {
    m_pGUI = GUI;

    m_pTextBox = m_pGUI->createWidgetReal<MyGUI::TextBox>("TextBox", Position.x, Position.y, Size.x, Size.y,
                                                          MyGUI::Align::Center, "Main");
    m_pTextBox->setDepth(int(Position.z));
}

ChOgreGUIText::~ChOgreGUIText() {
    m_pGUI->destroyWidget(m_pTextBox);
}

void ChOgreGUIText::setColor(float r, float g, float b) {
    m_pTextBox->setColour(MyGUI::Colour(r, g, b));
}

void ChOgreGUIText::setTextColor(float r, float g, float b) {
    m_pTextBox->setTextColour(MyGUI::Colour(r, g, b));
}

void ChOgreGUIText::setText(const std::string& Text) {
    m_pTextBox->setCaption(Text);
}

void ChOgreGUIText::setFont(const std::string& Name) {
    m_pTextBox->setFontName(Name);
}

void ChOgreGUIText::setPosition(const ChFloat3& Position) {
    m_pTextBox->setRealPosition(Position.x, Position.y);
    m_pTextBox->setDepth(int(Position.z));
}

void ChOgreGUIText::setSize(const ChFloat3& Size) {
    m_pTextBox->setRealSize(Size.x, Size.y);
}
}