/*	Perry Kivolowitz - University of Wisconsin - Madison 
Computer Sciences Department

A sample hello world like program demonstrating modern
OpenGL techniques. 

Created:	2/25/13
Updates:
*/

#include <iostream>
#include "ChOpenGLSphere.h"

using namespace std;
using namespace glm;
using namespace chrono::utils;
ChOpenGLSphere::ChOpenGLSphere() : ChOpenGLMesh()
{
}

//This initializer creates a sphere either inside out or normal
bool ChOpenGLSphere::Initialize(int slices, int stacks, ChOpenGLMaterial mat, bool flip)
{
	if (this->GLReturnedError("SphereA::Initialize - on entry"))
		return false;

	ChOpenGLMesh::Generate(slices,stacks,mat,flip);

	//http://stackoverflow.com/questions/7946770/calculating-a-sphere-in-opengl


	int rings = stacks;
	int sectors = slices;

	float const R = 1.f/(float)(rings-1);
	float const S = 1.f/(float)(sectors-1);
	mat4 m;
	int counter = 0;

	//Map the planar geometry to spherical coordinates
	//Keep our texture coordinates the same though!!

	//colors are applied at this stage
	for(int r = 0; r <  rings; r++){
		for(int s = 0; s < sectors; s++) {
			float  x = cos(2*M_PI * s * S) * sin( M_PI * (r) * R );
			float  y = sin( -M_PI/2.0f + M_PI * r * R );
			float  z = sin(2*M_PI * s * S) * sin( M_PI * (r) * R );
			vec2 tx = this->vertices[counter].texture_coordinate;
			this->vertices[counter]=(ChOpenGLVertexAttributesPADSNT( vec3(x,y,z), mat.ambient_color,mat.diffuse_color,mat.specular_color,mat.glow_color, normalize(vec3(x,y,z)),tx));
			counter++;
		}
	}

	PostInitialize();

	if (this->GLReturnedError("ChOpenGLSphere::Initialize - on exit"))
		return false;

	return true;
}



void ChOpenGLSphere::TakeDown()
{
	super::TakeDown();
}
