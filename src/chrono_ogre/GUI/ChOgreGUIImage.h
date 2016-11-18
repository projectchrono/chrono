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
			ChOgreGUIImage(const ChFloat3& Position, const ChFloat3& Size, MyGUI::Gui* GUI);
			~ChOgreGUIImage();

			virtual void setColor(float r, float g, float b);
			virtual void setImage(const std::string& Path);
			virtual void setPosition(const ChFloat3& Position);
			virtual void setSize(const ChFloat3& Size);

			virtual ChFloat3 getPosition() { return ChFloat3(m_pImageBox->getLeft(), m_pImageBox->getTop(), 0.f); };
			virtual ChFloat3 getSize() { return ChFloat3(m_pImageBox->getWidth(), m_pImageBox->getHeight(), 0.f); }

		protected:
			MyGUI::ImageBox* m_pImageBox;

			static unsigned int g_count;

		private:
		};

		typedef std::unique_ptr<ChOgreGUIImage> ChOgreGUIImagePtr;
	}
}