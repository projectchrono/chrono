#ifndef CHOPENGLMESH_H
#define CHOPENGLMESH_H

/*	Based on code provided by: Perry Kivolowitz
	Draws a planar mesh
	Not super usefull in itself but it can be wrapped into other shapes
	Also, texture coordinates are provided for texturing purposes
*/


#include "utils/opengl/core/ChOpenGLObject.h"

	namespace chrono {
		namespace utils{
			class CH_UTILS_OPENGL_API ChOpenGLMesh :public  ChOpenGLObject
			{
			public:
				ChOpenGLMesh();
				void Generate(int rows, int cols, ChOpenGLMaterial mat, bool flip=false);
				bool Initialize(int rows, int columns, ChOpenGLMaterial mat,bool flip=false);
				bool PostInitialize();
				virtual void Draw(const glm::mat4 & projection,const glm::mat4 & modelview);
				void Bind();
				void RecomputeNormals();
				void TakeDown();
			protected:
				std::vector<ChOpenGLVertexAttributesPADSNT> vertices;
				typedef ChOpenGLObject super;
			};
		}
	}

	#endif  // END of CHOPENGLBASE_H
