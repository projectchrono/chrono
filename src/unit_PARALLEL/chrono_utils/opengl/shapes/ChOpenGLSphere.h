/*	Based on code provided by: Perry Kivolowitz
	Takes a mesh object and wraps it into a sphere
	
	Texture coordinates are inherited through the mesh superclass
	Sphere can be "flipped" so that the inside is shown (used for stars and other cool stuff)
*/

#pragma once
#include "ChOpenGLMesh.h"
	namespace chrono {
		namespace utils{
			class ChOpenGLSphere : public ChOpenGLMesh
			{
			public:
				ChOpenGLSphere();
				bool Initialize(int slices, int stacks ,  ChOpenGLMaterial mat, bool flip = false);
				void TakeDown();

			private:
				typedef ChOpenGLMesh super;
			};
		}
	}
