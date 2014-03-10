#ifndef CHC_NARROWPHASE_R_UTILS_H
#define CHC_NARROWPHASE_R_UTILS_H


namespace chrono {
namespace collision {


// ----------------------------------------------------------------------------
// This utility function snaps the provided location to a point on a box with
// given half-dimensions. The in/out location is assumed to be specified in
// the frame of the box (which is therefore assumed to be an AABB centered at
// the origin).  The return code indicates the box axes that caused snapping.
//   - first bit (least significant) corresponds to x-axis
//   - second bit corresponds to y-axis
//   - third bit corresponds to z-axis
// Therefore:
//   code = 0 indicates an interior point
//   code = 1 or code = 2 or code = 4  indicates snapping to a face
//   code = 3 or code = 5 or code = 6  indicates snapping to an edge
//   code = 7 indicates snapping to a corner
__host__ __device__
uint snap_to_box(const real3& hdims, real3& loc)
{
	uint code = 0;

	if (fabs(loc.x) > hdims.x) {
		code |= 1;
		loc.x = (loc.x > 0) ? hdims.x : -hdims.x;
	}
	if (fabs(loc.y) > hdims.y) {
		code |= 2;
		loc.y = (loc.y > 0) ? hdims.y : -hdims.y;
	}
	if (fabs(loc.z) > hdims.z) {
		code |= 4;
		loc.z = (loc.z > 0) ? hdims.z : -hdims.z;
	}

	return code;
}


// ----------------------------------------------------------------------------
// These utility functions return the corner of a box of given dimensions that
// if farthest and closest in the direction 'dir', respectively. The direction
// 'dir' is assumed to be given in the frame of the box.
__host__ __device__
real3 box_farthest_corner(const real3& hdims, const real3& dir)
{
	real3 corner;
	corner.x = (dir.x < 0) ? hdims.x : -hdims.x;
	corner.y = (dir.y < 0) ? hdims.y : -hdims.y;
	corner.z = (dir.z < 0) ? hdims.z : -hdims.z;
	return corner;
}

__host__ __device__
real3 box_closest_corner(const real3& hdims, const real3& dir)
{
	real3 corner;
	corner.x = (dir.x > 0) ? hdims.x : -hdims.x;
	corner.y = (dir.y > 0) ? hdims.y : -hdims.y;
	corner.z = (dir.z > 0) ? hdims.z : -hdims.z;
	return corner;
}


// ----------------------------------------------------------------------------
// This utility function returns a code that indicates the closest feature of
// a box in the specified direction. The direction 'dir' is assumed to be
// given in the frame of the box. The return code encodes the box axes that
// define the closest feature:
//   - first bit (least significant) corresponds to x-axis
//   - second bit corresponds to y-axis
//   - third bit corresponds to z-axis
// Therefore:
//   code = 0 indicates a degenerate direction (within a threshold)
//   code = 1 or code = 2 or code = 4  indicates a face
//   code = 3 or code = 5 or code = 6  indicates an edge
//   code = 7 indicates a corner
__host__ __device__
uint box_closest_feature(const real3& dir)
{
	const real threshold = 0.01;

	return ((abs(dir.x) > threshold) << 0) |
	       ((abs(dir.y) > threshold) << 1) |
	       ((abs(dir.z) > threshold) << 2);
}


// ----------------------------------------------------------------------------
// This function returns a boolean indicating whether or not a box1 with
// dimensions hdims1 intersects a second box with the dimensions hdims2.
// The check is performed in the local frame of box1. The transform from the
// other box is given through 'pos' and 'rot'. If an intersection exists, the
// direction of smallest intersection is returned in 'dir'.
//
// This check is performed by testing 15 possible separating planes between the
// two boxes (Gottschalk, Lin, Manocha - Siggraph96).
__host__ __device__
bool box_intersects_box(const real3& hdims1, const real3& hdims2,
                        const real3& pos, const real4& rot,
                        real3& dir)
{
	M33 R = AMat(rot);
	M33 Rabs = AbsMat(R);
	real minOverlap = FLT_MAX;
	real overlap;
	real r1, r2;

	// 1. Test the axes of box1 (3 cases)
	// x-axis
	r2 = Rabs.U.x * hdims2.x + Rabs.V.x * hdims2.y + Rabs.W.x * hdims2.z;
	overlap = hdims1.x + r2 - abs(pos.x);
	if (overlap <= 0) return false;
	if (overlap < minOverlap) {
		dir = R3(1, 0, 0);
		minOverlap = overlap;
	}
	// y-axis
	r2 = Rabs.U.y * hdims2.x + Rabs.V.y * hdims2.y + Rabs.W.y * hdims2.z;
	overlap = hdims1.y + r2 - abs(pos.y);
	if (overlap <= 0) return false;
	if (overlap < minOverlap) {
		dir = R3(0, 1, 0);
		minOverlap = overlap;
	} 
	// z-axis
	r2 = Rabs.U.z * hdims2.x + Rabs.V.z * hdims2.y + Rabs.W.z * hdims2.z;
	overlap = hdims1.z + r2 - abs(pos.z);
	if (overlap <= 0) return false;
	if (overlap < minOverlap) {
		dir = R3(0, 0, 1);
		minOverlap = overlap;
	} 

	// 2. Test the axes of box2 (3 cases)
	// x-axis
	r1 = dot(Rabs.U, hdims1);
	overlap = r1 + hdims2.x - abs(dot(R.U, pos));
	if (overlap <= 0) return false;
	if (overlap < minOverlap) {
		dir = R.U;
		minOverlap = overlap;
	}
	// y-axis
	r1 = dot(Rabs.V, hdims1);
	overlap = r1 + hdims2.y - abs(dot(R.V, pos));
	if (overlap <= 0) return false;
	if (overlap < minOverlap) {
		dir = R.V;
		minOverlap = overlap;
	}
	// z-axis
	r1 = dot(Rabs.W, hdims1);
	overlap = r1 + hdims2.z - abs(dot(R.W, pos));
	if (overlap <= 0) return false;
	if (overlap < minOverlap) {
		dir = R.W;
		minOverlap = overlap;
	}

	// 3. Test the planes that are orthogonal (the cross-product) to pairs of axes
	// of the two boxes (9 cases)


	//// TODO

	return false;
}


// ----------------------------------------------------------------------------


} // end namespace collision
} // end namespace chrono


#endif

