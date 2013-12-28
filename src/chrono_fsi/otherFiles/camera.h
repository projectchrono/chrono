#ifndef CAMERA_H
#define CAMERA_H

#include <GL/glut.h>
////////////////////////Quaternion and Vector Code////////////////////////
typedef double camreal;

struct camreal3 {

camreal3(camreal a = 0, camreal b = 0, camreal c = 0): x(a), y(b), z(c) {}

camreal x, y, z;
};
struct camreal4 {

camreal4(camreal d = 0, camreal a = 0, camreal b = 0, camreal c = 0): w(d), x(a), y(b), z(c) {}

camreal w, x, y, z;
};

static camreal3 operator +(const camreal3 rhs, const camreal3 lhs)
{
camreal3 temp;
temp.x = rhs.x + lhs.x;
temp.y = rhs.y + lhs.y;
temp.z = rhs.z + lhs.z;
return temp;
}
static camreal3 operator -(const camreal3 rhs, const camreal3 lhs)
{
camreal3 temp;
temp.x = rhs.x - lhs.x;
temp.y = rhs.y - lhs.y;
temp.z = rhs.z - lhs.z;
return temp;
}
static void operator +=(camreal3 &rhs, const camreal3 lhs)
{
rhs = rhs + lhs;
}

static void operator -=(camreal3 &rhs, const camreal3 lhs)
{
rhs = rhs - lhs;
}

static camreal3 operator *(const camreal3 rhs, const camreal3 lhs)
{
camreal3 temp;
temp.x = rhs.x * lhs.x;
temp.y = rhs.y * lhs.y;
temp.z = rhs.z * lhs.z;
return temp;
}

static camreal3 operator *(const camreal3 rhs, const camreal lhs)
{
camreal3 temp;
temp.x = rhs.x * lhs;
temp.y = rhs.y * lhs;
temp.z = rhs.z * lhs;
return temp;
}

static inline camreal3 cross(camreal3 a, camreal3 b)
{
return camreal3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
gridSize
static camreal4 Q_from_AngAxis(camreal angle, camreal3 axis)
{
camreal4 quat;
camreal halfang;
camreal sinhalf;
halfang = (angle * 0.5);
sinhalf = sin(halfang);
quat.w = cos(halfang);
quat.x = axis.x * sinhalf;
quat.y = axis.y * sinhalf;
quat.z = axis.z * sinhalf;
return (quat);
}

static camreal4 normalize(const camreal4 &a)
{
camreal length = 1.0 / sqrt(a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z);
return camreal4(a.w * length, a.x * length, a.y * length, a.z * length);
}

static inline camreal4 inv(camreal4 a)
{
//return (1.0f / (dot(a, a))) * F4(a.x, -a.y, -a.z, -a.w);
camreal4 temp;
camreal t1 = a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z;
t1 = 1.0 / t1;
temp.w = t1 * a.w;
temp.x = -t1 * a.x;
temp.y = -t1 * a.y;
temp.z = -t1 * a.z;
return temp;
}
gridSize
static inline camreal4 mult(const camreal4 &a, const camreal4 &b)
{
camreal4 temp;
temp.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
temp.x = a.w * b.x + b.w * a.x + a.y * b.z - a.z * b.y;
temp.y = a.w * b.y + b.w * a.y + a.z * b.x - a.x * b.z;
temp.z = a.w * b.z + b.w * a.z + a.x * b.y - a.y * b.x;
return temp;
}

static inline camreal3 quatRotate(const camreal3 &v, const camreal4 &q)
{gridSize
camreal4 r = mult(mult(q, camreal4(0, v.x, v.y, v.z)), inv(q));
return camreal3(r.x, r.y, r.z);
}

static camreal4 operator %(const camreal4 rhs, const camreal4 lhs)
{
return mult(rhs, lhs);
}
////////////////////////END Quaternion and Vector Code////////////////////////

class OpenGLCamera
{
public:
OpenGLCamera(camreal3 pos, camreal3 lookat, camreal3 up, camreal viewscale) {
max_pitch_rate = 5;
max_heading_rate = 5;
camera_pos = pos;
look_at = lookat;gridSize
camera_up = up;
camera_heading = 0;
camera_pitch = 0;
dir = camreal3(0, 0, 1);
mouse_pos = camreal3(0, 0, 0);
camera_pos_delta = camreal3(0, 0, 0);
scale = viewscale;
}
void ChangePitch(GLfloat degrees) {
if (fabs(degrees) < fabs(max_pitch_rate)) {
camera_pitch += degrees;
} else {
if (degrees < 0) {
camera_pitch -= max_pitch_rate;
} else {
camera_pitch += max_pitch_rate;
}
}

if (camera_pitch > 360.0f) {
camera_pitch -= 360.0f;
} else if (camera_pitch < -360.0f) {
camera_pitch += 360.0f;
}
}
void ChangeHeading(GLfloat degrees) {
if (fabs(degrees) < fabs(max_heading_rate)) {
if (camera_pitch > 90 && camera_pitch < 270 || (camera_pitch < -90 && camera_pitch > -270)) {
camera_heading -= degrees;
} else {
camera_heading += degrees;
}
} else {
if (degrees < 0) {
if ((camera_pitch > 90 && camera_pitch < 270) || (camera_pitch < -90 && camera_pitch > -270)) {
camera_heading += max_heading_rate;
} else {
camera_heading -= max_heading_rate;
}
} else {
if (camera_pitch > 90 && camera_pitch < 270 || (camera_pitch < -90 && camera_pitch > -270)) {
camera_heading -= max_heading_rate;
} else {
camera_heading += max_heading_rate;
}
}
}

if (camera_heading > 360.0f) {
camera_heading -= 360.0f;
} else if (camera_heading < -360.0f) {
camera_heading += 360.0f;
}
}
void Move2D(int x, int y) {
camreal3 mouse_delta = mouse_pos - camreal3(x, y, 0);
ChangeHeading(.02 * mouse_delta.x);
ChangePitch(.02 * mouse_delta.y);
mouse_pos = camreal3(x, y, 0);
}
void SetPos(int button, int state, int x, int y) {
mouse_pos = camreal3(x, y, 0);
}
void Update() {
camreal4 pitch_quat, heading_quat;
camreal3 angle;
angle = cross(dir, camera_up);
pitch_quat = Q_from_AngAxis(camera_pitch, angle);
heading_quat = Q_from_AngAxis(camera_heading, camera_up);
camreal4 temp = (pitch_quat % heading_quat);
temp = normalize(temp);
dir = quatRotate(dir, temp);
camera_pos += camera_pos_delta;
look_at = camera_pos + dir * 1;
camera_heading *= .5;
camera_pitch *= .5;
camera_pos_delta = camera_pos_delta * .5;
gluLookAt(camera_pos.x, camera_pos.y, camera_pos.z, look_at.x, look_at.y, look_at.z, camera_up.x, camera_up.y, camera_up.z);
}
void Forward() {
camera_pos_delta += dir * scale;
}
void Back() {
camera_pos_delta -= dir * scale;
}
void Right() {
camera_pos_delta += cross(dir, camera_up) * scale;
}
void Left() {
camera_pos_delta -= cross(dir, camera_up) * scale;
}
void Up() {
camera_pos_delta -= camera_up * scale;
}
void Down() {
camera_pos_delta += camera_up * scale;
}

camreal max_pitch_rate, max_heading_rate;
camreal3 camera_pos, look_at, camera_up;
camreal camera_heading, camera_pitch, scale;
camreal3 dir, mouse_pos, camera_pos_delta;
};
#endif
