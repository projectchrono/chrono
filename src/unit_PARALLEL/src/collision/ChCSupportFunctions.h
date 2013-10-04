#ifndef CHC_SUPPORTFUNCTIONS_H
#define CHC_SUPPORTFUNCTIONS_H

__device__          __host__          inline real3 GetSupportPoint_Sphere(const real3 &B, const real3 &n) {
	return B.x * n;
}
__device__          __host__          inline real3 GetSupportPoint_Triangle(const real3 &A, const real3 &B, const real3 &C, const real3 &n) {
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
__device__          __host__          inline real3 GetSupportPoint_Box(const real3 &B, const real3 &n) {
	real3 result = R3(0, 0, 0);
	result.x = sign(n.x) * B.x;
	result.y = sign(n.y) * B.y;
	result.z = sign(n.z) * B.z;
	return result;
}
__device__          __host__          inline real3 GetSupportPoint_Ellipsoid(const real3 &B, const real3 &n) {

	real3 normal = n;
	real3 result = B * B * normal / length(B * normal);
	//cout << result.x << " " << result.y << " " << result.z<<endl;
	return result;

//
//	real3 norm=normalize(n);
//	real3 dim=(norm*norm)/(B*B);
//	real k = sqrt(1/(dim.x+dim.y+dim.z));
//	return k*norm;
}

__device__          __host__          inline real3 GetSupportPoint_Cylinder(const real3 &B, const real3 &n) {
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
__device__          __host__          inline real3 GetSupportPoint_Plane(const real3 &B, const real3 &n) {
	real3 result = B;

	if (n.x < 0)
		result.x = -result.x;

	if (n.y < 0)
		result.y = -result.y;

	return result;
}
__device__          __host__          inline real3 GetSupportPoint_Cone(const real3 &B, const real3 &n) {
	real radius = B.x;
	real height = B.y;

	real m_sinAngle = (radius / sqrt(radius * radius + height * height));

	if (n.y > length(n) * m_sinAngle) {
		return R3(0, height / 2.0, 0);
	} else {
		real s = sqrt(n.x * n.x + n.z * n.z);
		if (s > 1e-9) {
			real d = radius / s;
			real3 tmp;
			tmp.x = n.x * d;
			tmp.y = -height / 2.0;
			tmp.z = n.z * d;
			return tmp;
		} else {
			real3 tmp;
			tmp.x = 0;
			tmp.y = -height / 2.0;
			tmp.z = 0;
			return tmp;
		}
	}
}

__device__          __host__          inline real3 GetSupportPoint_Disc(const real3 &B, const real3 &n) {
	real3 n2 = R3(n.x, n.y, 0);
	n2 = normalize(n2);

	real3 result1 = B.x * n2;

	return result1;

}

__device__          __host__          inline real3 GetCenter_Sphere() {
	return ZERO_VECTOR;
}
__device__          __host__          inline real3 GetCenter_Triangle(const real3 &A, const real3 &B, const real3 &C) {
	return R3((A.x + B.x + C.x) / 3.0f, (A.y + B.y + C.y) / 3.0f, (A.z + B.z + C.z) / 3.0f);
}
__device__          __host__          inline real3 GetCenter_Box() {
	return ZERO_VECTOR;
}
__device__          __host__          inline real3 GetCenter_Ellipsoid() {
	return ZERO_VECTOR;
}
__device__          __host__          inline real3 GetCenter_Cylinder() {
	return ZERO_VECTOR;
}
__device__          __host__          inline real3 GetCenter_Plane() {
	return ZERO_VECTOR;
}
__device__          __host__          inline real3 GetCenter_Cone(const real3 &B) {
	return ZERO_VECTOR;
}
#endif
