#include "collision/ChCCollisionModel.h"

#include "math/ChParallelMath.h"
#include "collision/ChCNarrowphaseR.h"
#include "collision/ChCNarrowphaseRUtils.h"


using namespace chrono;
using namespace chrono::collision;


//// TODO: what is a good value here?
////       Should this even be a constant?  Maybe set it based on object size?
const real edge_radius = 0.1;


void ChCNarrowphaseR::Process(ChParallelDataManager* data_container)
{
	// Aliases for readibility
	custom_vector<real3>& norm_data = data_container->host_data.norm_rigid_rigid;
	custom_vector<real3>& cpta_data = data_container->host_data.cpta_rigid_rigid;
	custom_vector<real3>& cptb_data = data_container->host_data.cptb_rigid_rigid;
	custom_vector<real>&  dpth_data = data_container->host_data.dpth_rigid_rigid;
	custom_vector<real>&  erad_data = data_container->host_data.erad_rigid_rigid;
	custom_vector<int2>&  bids_data = data_container->host_data.bids_rigid_rigid;

	custom_vector<long long>& potentialCollisions = data_container->host_data.pair_rigid_rigid;
	uint num_potentialCollisions = potentialCollisions.size();

	// Return now if no potential collisions.
	if (num_potentialCollisions == 0) {
		norm_data.resize(0);
		cpta_data.resize(0);
		cptb_data.resize(0);
		dpth_data.resize(0);
		erad_data.resize(0);
		bids_data.resize(0);
		data_container->number_of_rigid_rigid = 0;
		return;
	}

	// Create a vector to hold the maximum number of contacts produced by each potential
	// collision pair identified by the broadphase.
	custom_vector<uint> contact_index(num_potentialCollisions);

	host_count(data_container, num_potentialCollisions, contact_index);

	// Perform an exclusive scan on the contact_index array and calculate the total number
	// of possible contacts.
	int num_potentialContacts = contact_index.back();
	thrust::exclusive_scan(contact_index.begin(), contact_index.end(), contact_index.begin());
	num_potentialContacts += contact_index.back();

	// Allocate enough space for all possible contacts
	norm_data.resize(num_potentialContacts);
	cpta_data.resize(num_potentialContacts);
	cptb_data.resize(num_potentialContacts);
	dpth_data.resize(num_potentialContacts);
	erad_data.resize(num_potentialContacts);
	bids_data.resize(num_potentialContacts);

	// Create a vector of flags to indicate whether actual contact exists
	custom_vector<uint> contact_flag(num_potentialContacts);
	thrust::fill(contact_flag.begin(), contact_flag.end(), 1);

	// Perform actual narrow phase collision detection. For each potential collision between two
	// shapes, calculate and fill in contact information for contacts that actually occur and set
	// the corresponding flag to 0.
	host_process(data_container, num_potentialCollisions, contact_index, contact_flag);

	// Evaluate the number of actual contacts
	uint number_of_contacts = num_potentialContacts - thrust::count(contact_flag.begin(), contact_flag.end(), 1);

	data_container->number_of_rigid_rigid = number_of_contacts;
	data_container->erad_is_set = true;

	// Remove unused array portions
	thrust::remove_if(norm_data.begin(), norm_data.end(), contact_flag.begin(), thrust::identity<int>());
	thrust::remove_if(cpta_data.begin(), cpta_data.end(), contact_flag.begin(), thrust::identity<int>());
	thrust::remove_if(cptb_data.begin(), cptb_data.end(), contact_flag.begin(), thrust::identity<int>());
	thrust::remove_if(dpth_data.begin(), dpth_data.end(), contact_flag.begin(), thrust::identity<int>());
	thrust::remove_if(erad_data.begin(), erad_data.end(), contact_flag.begin(), thrust::identity<int>());
	thrust::remove_if(bids_data.begin(), bids_data.end(), contact_flag.begin(), thrust::identity<int>());

	norm_data.resize(number_of_contacts);
	cpta_data.resize(number_of_contacts);
	cptb_data.resize(number_of_contacts);
	dpth_data.resize(number_of_contacts);
	erad_data.resize(number_of_contacts);
	bids_data.resize(number_of_contacts);
}


// ==========================================================================

__host__ __device__
void function_count(const int&        icoll,            // index of this potential collision
                    const shape_type* obj_data_T,       // shape type (per shape)
                    const long long*  collision_pair,   // encoded shape IDs (per collision pair)
                    uint*             max_contacts)     // max. number of contacts (per collision pair)
{
	// Identify the two candidate shapes and get their types.
	int2       pair = I2(int(collision_pair[icoll] >> 32), int(collision_pair[icoll] & 0xffffffff));
	shape_type type1 = obj_data_T[pair.x];
	shape_type type2 = obj_data_T[pair.y];

	// Set the maximum number of possible contacts for this particular pair
	if (type1 == SPHERE || type2 == SPHERE)
		max_contacts[icoll] = 1;
	else if (type1 == CAPSULE || type2 == CAPSULE)
		max_contacts[icoll] = 2;
	else
		max_contacts[icoll] = 4;
}

void ChCNarrowphaseR::host_count(ChParallelDataManager* data_container,
                                 uint                   num_potentialCollisions,
                                 custom_vector<uint>&   max_contacts)
{
#pragma omp parallel for
	for (int icoll = 0; icoll < num_potentialCollisions; icoll++) {
		function_count(icoll,
		               data_container->host_data.typ_rigid.data(),
		               data_container->host_data.pair_rigid_rigid.data(),
		               max_contacts.data());
	}
}

// ==========================================================================
//              SPHERE - SPHERE

// Sphere-sphere narrow phase collision detection.
// In:  sphere centered at pos1 with radius1
//      sphere centered at pos2 with radius2
__host__ __device__
bool sphere_sphere(const real3& pos1, const real& radius1,
                   const real3& pos2, const real& radius2,
                   real3& norm, real& depth,
                   real3& pt1, real3& pt2,
                   real& eff_radius)
{
	real3 delta = pos2 - pos1;
	real  dist2 = dot(delta, delta);
	real  radSum = radius1 + radius2;

	// If the two sphere centers are separated by more than the sum of their
	// radii, there is no contact. Also ignore contact if the two centers
	// almost coincide, in which case we cannot decide on the direction.
	if (dist2 >= radSum * radSum || dist2 < 1e-12)
		return false;

	// Generate contact information.
	real dist = sqrt(dist2);
	norm = delta / dist;
	pt1 = pos1 + norm * radius1;
	pt2 = pos2 - norm * radius2;
	depth = dist - radSum;
	eff_radius = radius1 * radius2 / radSum;

	return true;
}

// ==========================================================================
//              BOX - SPHERE

// Box-sphere narrow phase collision detection.
// In:  box at position pos1, with orientation rot1, and half-dimensions hdims1
//      sphere centered at pos2 and with radius2
__host__ __device__
bool box_sphere(const real3& pos1, const real4& rot1, const real3& hdims1,
                const real3& pos2, const real& radius2,
                real3& norm, real& depth,
                real3& pt1, real3& pt2,
                real& eff_radius)
{
	// Express the sphere position in the frame of the box.
	real3 spherePos = TransformParentToLocal(pos1, rot1, pos2);

	// Snap the sphere position to the surface of the box.
	real3 boxPos = spherePos;
	uint  code = snap_to_box(hdims1, boxPos);

	// If the sphere doesn't touch the closest point then there is no contact.
	// Also, ignore contact if the sphere center (almost) coincides with the
	// closest point, in which case we couldn't decide on the proper contact
	// direction.
	real3 delta = spherePos - boxPos;
	real  dist2 = dot(delta, delta);

	if (dist2 >= radius2 * radius2 || dist2 <= 1e-12f)
		return false;

	// Generate contact information
	real dist = sqrt(dist2);
	depth = dist - radius2;
	norm = quatRotateMat(delta / dist, rot1);
	pt1 = TransformLocalToParent(pos1, rot1, boxPos);
	pt2 = pos2 - norm * radius2;

	if ((code != 1) & (code != 2) & (code != 4))
		eff_radius = radius2 * edge_radius / (radius2 + edge_radius);
	else
		eff_radius = radius2;

	return true;
}

// ==========================================================================
//              BOX - BOX

// Box-box narrow phase collision detection.
// In:  box at position pos1, with orientation rot1, and half-dimensions hdims1
//      box at position pos2, with orientation rot2, and half-dimensions hdims2

__host__ __device__
bool box_box(const real3& pos1, const real4& rot1, const real3& hdims1,
             const real3& pos2, const real4& rot2, const real3& hdims2,
             real3& norm, real& depth,
             real3& pt1, real3& pt2,
             real& eff_radius)
{
	// Express the second box into the frame of the first box.
	// (this is a bit cryptic with the functions we have available)
	real3 pos = quatRotateMatT(pos2 - pos1, rot1);
	real4 rot = mult(inv(rot1), rot2);

	// Find the direction of closest overlap between boxes. If they don't
	// overlap, we're done. Note that dir is calculated so that it points from
	// box2 to box1.
	real3 dir;
	if (!box_intersects_box(hdims1, hdims2, pos, rot, dir))
		return false;

	if (dot(pos, dir) > 0)
		dir = -dir;

	// Determine the features of the boxes that are interacting.
	real3 dirI = quatRotateMatT(-dir, rot);
	real3 corner1 = box_farthest_corner(hdims1, dir);
	real3 corner2 = box_farthest_corner(hdims2, dirI);
	uint  code1 = box_closest_feature(dir);
	uint  code2 = box_closest_feature(dirI);
	uint  numAxes1 = (code1 & 1) + ((code1 >> 1) & 1) + ((code1 >> 2) & 1);
	uint  numAxes2 = (code2 & 1) + ((code2 >> 1) & 1) + ((code2 >> 2) & 1);

	//// TODO

	return false;
}

// ==========================================================================

// This is the main worker function for narrow phase check of the collision
// candidate pair 'icoll'.  Each candidate pair of shapes can result in 0, 1,
// or more contacts.  For each actual contact, we calculate various geometrical
// quantities and load them in the output arrays beginning at the 'start_index'
// for the processed collision pair:
//   - ct_flag:     if contact actually occurs, set to 0
//   - ct_pt1:      contact point on first shape (in global frame)
//   - ct_pt2:      contact point on second shape (in global frame)
//   - ct_depth:    penetration distance (negative if overlap exists)
//   - ct_norm:     contact normal, from ct_pt2 to ct_pt1 (in global frame)
//   - ct_eff_rad:  effective contact radius
//   - ct_body_ids: IDs of the bodies for the two contact shapes
__host__ __device__
void function_process(const uint&       icoll,           // index of this contact pair candidate
                      const shape_type* obj_data_T,      // shape type (per shape)
                      const real3*      obj_data_A,      //
                      const real3*      obj_data_B,      // shape geometry and local position (per shape)
                      const real3*      obj_data_C,      //
                      const real4*      obj_data_R,      // shape orientation wrt body (per shape)
                      const uint*       obj_data_ID,     // body ID (per shape)
                      const bool*       body_active,     // body active (per body)
                      const real3*      body_pos,        // body position (per body)
                      const real4*      body_rot,        // body rotation (per body)
                      const long long*  collision_pair,  // encoded shape IDs (per collision pair)
                      const uint*       start_index,     // start index in output arrays (per collision pair)
                      uint*             ct_flag,         // [output] flag for actual contact (per contact pair)
                      real3*            ct_norm,         // [output] contact normal (per contact pair)
                      real3*            ct_pt1,          // [output] point on shape1 (per contact pair)
                      real3*            ct_pt2,          // [output] point on shape2 (per contact pair)
                      real*             ct_depth,        // [output] penetration depth (per contact pair)
                      real*             ct_eff_rad,      // [output] effective contact radius (per contact pair)
                      int2*             ct_body_ids)     // [output] body IDs (per contact pair)
{
	// Identify the two candidate shapes and their associated bodies.
	int2 pair = I2(int(collision_pair[icoll] >> 32), int(collision_pair[icoll] & 0xffffffff));
	int shape1 = pair.x;
	int shape2 = pair.y;
	int body1  = obj_data_ID[shape1];
	int body2  = obj_data_ID[shape2];

	// No contact if the two shapes are on the same body or if either body is inactive.
	if (body1 == body2 || (!body_active[body1] && !body_active[body2]))
		return;

	// Get the types of the two candidate shapes.
	shape_type type1 = obj_data_T[shape1];
	shape_type type2 = obj_data_T[shape2];

	// Get the body positions and orientations and calculate the orientation of
	// the two shapes with respect to the global frame.
	const real3& bodyPos1 = body_pos[body1];
	const real3& bodyPos2 = body_pos[body2];
	const real4& bodyRot1 = body_rot[body1];
	const real4& bodyRot2 = body_rot[body2];
	real4 R1 = mult(bodyRot1, obj_data_R[shape1]);
	real4 R2 = mult(bodyRot2, obj_data_R[shape2]);

	// Express shape positions in global frame.
	// For a triangle in a mesh, X, Y, Z contain the vertex positions.
	// For all other shapes, X contains the center of the shape.
	real3 X1 = TransformLocalToParent(bodyPos1, bodyRot1, obj_data_A[shape1]);
	real3 Y1 = obj_data_B[shape1];
	real3 Z1 = obj_data_C[shape1];
	if (type1 == TRIANGLEMESH) {
		Y1 = TransformLocalToParent(bodyPos1, bodyRot1, Y1);
		Z1 = TransformLocalToParent(bodyPos1, bodyRot1, Z1);
	}

	real3 X2 = TransformLocalToParent(bodyPos2, bodyRot2, obj_data_A[shape2]);
	real3 Y2 = obj_data_B[shape2];
	real3 Z2 = obj_data_C[shape2];
	if (type2 == TRIANGLEMESH) {
		Y2 = TransformLocalToParent(bodyPos2, bodyRot2, Y2);
		Z2 = TransformLocalToParent(bodyPos2, bodyRot2, Z2);
	}

	// Now special-case the collision detection based on the types of the
	// two candidate shapes.
	//// TODO: what is the best way to dispatch this?
	uint index = start_index[icoll];

	if (type1 == SPHERE && type2 == SPHERE) {
		if (sphere_sphere(X1, Y1.x,
			              X2, Y2.x,
			              ct_norm[index], ct_depth[index],
			              ct_pt1[index], ct_pt2[index],
			              ct_eff_rad[index])) {
			ct_flag[index] = 0;
			ct_body_ids[index] = I2(body1, body2);
		}
		return;
	}

	if (type1 == BOX && type2 == SPHERE) {
		if (box_sphere(X1, R1, Y1,
			           X2, Y2.x,
			           ct_norm[index], ct_depth[index],
			           ct_pt1[index], ct_pt2[index],
			           ct_eff_rad[index])) {
			ct_flag[index] = 0;
			ct_body_ids[index] = I2(body1, body2);
		}
		return;
	}

	if (type1 == SPHERE && type2 == BOX) {
		if (box_sphere(X2, R2, Y2,
			           X1, Y1.x,
			           ct_norm[index], ct_depth[index],
			           ct_pt2[index], ct_pt1[index],
			           ct_eff_rad[index])) {
			ct_norm[index] = -ct_norm[index];
			ct_flag[index] = 0;
			ct_body_ids[index] = I2(body1, body2);
		}
		return;
	}

	if (type1 == BOX && type2 == BOX) {
		if (box_box(X1, R1, Y1,
			        X2, R2, Y2,
			        ct_norm[index], ct_depth[index],
			        ct_pt1[index], ct_pt2[index],
			        ct_eff_rad[index])) {
			ct_flag[index] = 0;
			ct_body_ids[index] = I2(body1, body2);
		}
		return;
	}
}

// ==========================================================================

void ChCNarrowphaseR::host_process(ChParallelDataManager* data_container,
                                   uint                   num_potentialCollisions,
                                   custom_vector<uint>&   contact_index,
                                   custom_vector<uint>&   contact_flag)
{
#pragma omp parallel for
	for (int icoll = 0; icoll < num_potentialCollisions; icoll++) {
		function_process(icoll,
		                 data_container->host_data.typ_rigid.data(),
		                 data_container->host_data.ObA_rigid.data(),
		                 data_container->host_data.ObB_rigid.data(),
		                 data_container->host_data.ObC_rigid.data(),
		                 data_container->host_data.ObR_rigid.data(),
		                 data_container->host_data.id_rigid.data(),
		                 data_container->host_data.active_data.data(),
		                 data_container->host_data.pos_data.data(),
		                 data_container->host_data.rot_data.data(),
		                 data_container->host_data.pair_rigid_rigid.data(),
		                 contact_index.data(),
		                 contact_flag.data(),
		                 data_container->host_data.norm_rigid_rigid.data(),
		                 data_container->host_data.cpta_rigid_rigid.data(),
		                 data_container->host_data.cptb_rigid_rigid.data(),
		                 data_container->host_data.dpth_rigid_rigid.data(),
		                 data_container->host_data.erad_rigid_rigid.data(),
		                 data_container->host_data.bids_rigid_rigid.data());
	}
}
