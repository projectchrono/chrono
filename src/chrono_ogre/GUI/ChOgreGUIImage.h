#pragma once

/*
Author: Charles Ricchio
*/

#pragma once

#include "ChOgreGUIElement.h"
#include <core/ChVector.h>

namespace chrono {
	namespace ChOgre {

		class CHOGRE_DLL_TAG ChOgreGUIImage : public ChOgreGUIElement {
		public:
			ChOgreGUIImage();
			ChOgreGUIImage(MyGUI::Gui* GUI);
			ChOgreGUIImage(const ChVector2<>& Position, const ChVector2<>& Size, MyGUI::Gui* GUI);
			~ChOgreGUIImage();

			virtual void setColor(float r, float g, float b);
			virtual void setImage(const std::string& Path);
			virtual void setPosition(const ChVector2<>& Position);
			virtual void setSize(const ChVector2<>& Size);

			virtual ChVector2<> getPosition() { return ChVector2<>(m_pImageBox->getLeft(), m_pImageBox->getTop()); };
			virtual ChVector2<> getSize() { return ChVector2<>(m_pImageBox->getWidth(), m_pImageBox->getHeight()); }

		protected:
			MyGUI::ImageBox* m_pImageBox;

			static unsigned int g_count;

		private:
		};

		typedef std::unique_ptr<ChOgreGUIImage> ChOgreGUIImagePtr;
	}
}