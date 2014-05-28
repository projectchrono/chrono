#ifndef CHOPENGLVERTEXATTRIBUTE_H
#define CHOPENGLVERTEXATTRIBUTE_H

/*	Based on code provided by: Perry Kivolowitz
	P = position
	C = color
	N = normal
	T = texture coordinates

	A = Ambient
	D = Diffuse
	S = Specular
*/

#include "chrono_utils/opengl/core/ChOpenGLBase.h"
	namespace chrono {
		namespace utils{
//This class supports ADS lighting with glow and texture coordinates
//Nothing too interesting here, note that the order for the members is important
			class CH_UTILS_OPENGL_API ChOpenGLVertexAttributesPADSNT
			{
			public:
				ChOpenGLVertexAttributesPADSNT();
				ChOpenGLVertexAttributesPADSNT(const glm::vec3 & p, const glm::vec3 & c_a, const glm::vec3 & c_d, const glm::vec3 & c_s, const glm::vec3 & c_g, const glm::vec3 & n, const glm::vec2 & t);
				ChOpenGLVertexAttributesPADSNT(const ChOpenGLVertexAttributesPADSNT & other);
				glm::vec3 position;
				glm::vec3 normal;
				glm::vec3 color_ambient;
				glm::vec3 color_diffuse;
				glm::vec3 color_specular;
				glm::vec3 color_glow;
				glm::vec2 texture_coordinate;
			};

			class CH_UTILS_OPENGL_API ChOpenGLVertexAttributesPCNT
			{
			public:
				ChOpenGLVertexAttributesPCNT();
				ChOpenGLVertexAttributesPCNT(const glm::vec3 & p, const glm::vec3 & c, const glm::vec3 & n, const glm::vec2 & t);
				ChOpenGLVertexAttributesPCNT(const ChOpenGLVertexAttributesPCNT & other);
				glm::vec3 position;
				glm::vec3 normal;
				glm::vec3 color;
				glm::vec2 texture_coordinate;
			};

			class CH_UTILS_OPENGL_API ChOpenGLVertexAttributesPCN
			{
			public:
				ChOpenGLVertexAttributesPCN();
				ChOpenGLVertexAttributesPCN(const glm::vec3 & p, const glm::vec3 & c, const glm::vec3 & n);
				ChOpenGLVertexAttributesPCN(const ChOpenGLVertexAttributesPCN & other);
				glm::vec3 position;
				glm::vec3 normal;
				glm::vec3 color;
				
			};

			class CH_UTILS_OPENGL_API ChOpenGLVertexAttributesPN
			{
			public:
				ChOpenGLVertexAttributesPN();
				ChOpenGLVertexAttributesPN(const glm::vec3 & p, const glm::vec3 & n);
				ChOpenGLVertexAttributesPN(const ChOpenGLVertexAttributesPN & other);
				glm::vec3 position;
				glm::vec3 normal;
			};

			class CH_UTILS_OPENGL_API ChOpenGLVertexAttributesP
			{
			public:
				ChOpenGLVertexAttributesP();
				ChOpenGLVertexAttributesP(const glm::vec3 & p);
				ChOpenGLVertexAttributesP(const ChOpenGLVertexAttributesP & other);
				glm::vec3 position;
			};
		}
	}
#endif  // END of CHOPENGLVERTEXATTRIBUTE_H
