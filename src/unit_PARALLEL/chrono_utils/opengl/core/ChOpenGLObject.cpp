/*	Perry Kivolowitz - University of Wisconsin - Madison 
Computer Sciences Department

A sample hello world like program demonstrating modern OpenGL techniques. 

Object() is a sample base class upon which drawable objects might
be derived. It assumes that all drawable objects have some geometry
to buffer.

Created:	2/25/13
*/

#include "utils/opengl/core/ChOpenGLObject.h"

using namespace std;
using namespace glm;
using namespace chrono::utils;

/*	Notice the destructor in this case asserts that all resources
that don't go away by themselves have ALREADY been released. This
is because the destructor might be called after a GL context has
been destroyed, so I force the user of this class to call the
TakeDown() purposefully.
*/

ChOpenGLObject::ChOpenGLObject()
{
	this->InternalInitialize();
	shader = 0;
}

ChOpenGLObject::~ChOpenGLObject()
{
	assert(this->vertex_array_handle == GLuint(-1));
	assert(this->vertex_coordinate_handle == GLuint(-1));
}
//clear all of the internal data structures used by object
void ChOpenGLObject::TakeDown()
{
	//clear the indicies
	this->vertex_indices.clear();
	//check if handle is valid, if it is delete the associated vertex arrays
	if (this->vertex_array_handle != GLuint(-1))
		glDeleteVertexArrays(1, &this->vertex_array_handle);
	//check if handle is valid, if it is delete the associated vertex arrays
	if (this->vertex_coordinate_handle != GLuint(-1))
		glDeleteBuffers(1, &this->vertex_coordinate_handle);

	this->InternalInitialize();
}

bool ChOpenGLObject::PostGLInitialize(GLuint * vertex_array_handle, GLuint * vertex_coordinate_handle, GLsizeiptr sz, const GLvoid * ptr)
{
	//once all of out data is ready, generate the vertex buffers and bind them
	glGenVertexArrays(1, vertex_array_handle);
	glBindVertexArray(*vertex_array_handle);
	glGenBuffers(1, vertex_coordinate_handle);
	glBindBuffer(GL_ARRAY_BUFFER, *vertex_coordinate_handle);
	glBufferData(GL_ARRAY_BUFFER, sz, ptr, GL_STATIC_DRAW);
	return !this->GLReturnedError("ChOpenGLObject::PostGLInitialize - on exit");
}

bool ChOpenGLObject::Initialize()
{
	this->InternalInitialize();
	return true;
}
//attaches a shader to the object. This can be used in the draw call of any inherited classes
void ChOpenGLObject::AttachShader(ChOpenGLShader * new_shader){
	shader = new_shader;
}

void ChOpenGLObject::InternalInitialize()
{
	this->vertex_array_handle = this->vertex_coordinate_handle = GLuint(-1);
}