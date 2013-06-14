#ifndef CHC_SUPPORTFUNCTIONS_H
#define CHC_SUPPORTFUNCTIONS_H

__device__  __host__  inline real3 GetSupportPoint_Sphere(const real3 &B, const real3 &n) {
	real3 result = normalize(n);
	return B.x * result;
}
__device__  __host__  inline real3 GetSupportPoint_Triangle(const real3 &A, const real3 &B, const real3 &C, const real3 &n) {
	real dist = dot(A, n);
	real3 point = A;

	if (dot(B, n) > dist) {
		dist = dot(B, n);
		point = B;
	}

	if (dot(C, n) > dist) {
		dist = dot(C, n);
		point = C;
	}

	return point;
}
__device__  __host__  inline real3 GetSupportPoint_Box(const real3 &B, const real3 &n) {
	real3 result = R3(0, 0, 0);
	result.x = sign(n.x) * B.x;
	result.y = sign(n.y) * B.y;
	result.z = sign(n.z) * B.z;
	return result;
}
__device__  __host__  inline real3 GetSupportPoint_Ellipsoid(const real3 &B, const real3 &n) {

	real3 normal = normalize(n);
	real3 result = B * B * normal / length(B * normal);
	//cout << result.x << " " << result.y << " " << result.z<<endl;
	return result;

//
//	real3 norm=normalize(n);
//	real3 dim=(norm*norm)/(B*B);
//	real k = sqrt(1/(dim.x+dim.y+dim.z));
//	return k*norm;
}

__device__  __host__  inline real3 GetSupportPoint_Cylinder(const real3 &B, const real3 &n) {
	real3 u = R3(0, 1, 0);
	real3 w = n - (dot(u, n)) * u;
	real3 result;

	if (length(w) != 0) {
		result = sign(dot(u, n)) * B.y * u + B.x * normalize(w);
	} else {
		result = sign(dot(u, n)) * B.y * u;
	}

	return result;
}
__device__  __host__  inline real3 GetSupportPoint_Plane(const real3 &B, const real3 &n) {
	real3 result = B;

	if (n.x < 0)
		result.x = -result.x;

	if (n.y < 0)
		result.y = -result.y;

	return result;
}
__device__  __host__  inline real3 GetSupportPoint_Cone(const real3 &B, const real3 &n) {
	real sigma = sqrt(n.x * n.x + n.z * n.z);
	real sina = B.x / sqrt(B.x * B.x + B.y * B.y);
	real3 result;

	if (n.y > length(n) * sina) {
		result.x = 0.0f;
		result.y = (2.0f / 3.0f) * B.y;
		result.z = 0.0f;
	} else if (sigma > 0.0f) {
		result.x = B.x * n.x / sigma;
		result.y = -(1.0f / 3.0f) * B.y;
		result.z = B.x * n.z / sigma;
	} else {
		result.x = 0.0f;
		result.y = -(1.0f / 3.0f) * B.y;
		result.z = 0.0f;
	}

	return result;
}

__device__  __host__  inline real3 GetCenter_Sphere() {
	return ZERO_VECTOR;
}
__device__  __host__  inline real3 GetCenter_Triangle(const real3 &A, const real3 &B, const real3 &C) {
	return R3((A.x + B.x + C.x) / 3.0f, (A.y + B.y + C.y) / 3.0f, (A.z + B.z + C.z) / 3.0f);
}
__device__  __host__  inline real3 GetCenter_Box() {
	return ZERO_VECTOR;
}
__device__  __host__  inline real3 GetCenter_Ellipsoid() {
	return ZERO_VECTOR;
}
__device__  __host__  inline real3 GetCenter_Cylinder() {
	return ZERO_VECTOR;
}
__device__  __host__  inline real3 GetCenter_Plane() {
	return ZERO_VECTOR;
}
__device__  __host__  inline real3 GetCenter_Cone() {
	return ZERO_VECTOR;
}
#endif
