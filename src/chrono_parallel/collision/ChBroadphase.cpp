#include <algorithm>

#include <chrono_parallel/collision/ChCollision.h>
#include "chrono_parallel/collision/ChBroadphaseUtils.h"
#include "chrono_parallel/physics/Ch3DOFContainer.h"

//#include <thrust/host_vector.h>
#include <thrust/transform.h>
#include <thrust/transform_reduce.h>
#include <thrust/sort.h>
#include <thrust/sequence.h>
#include <thrust/iterator/constant_iterator.h>

#if defined(CHRONO_OPENMP_ENABLED)
#include <thrust/system/omp/execution_policy.h>
#elif defined(CHRONO_TBB_ENABLED)
#include <thrust/system/tbb/execution_policy.h>
#endif

using thrust::transform;
using thrust::transform_reduce;

namespace chrono {
namespace collision {

// Determine the bounding box for the objects===============================================================

void ChCBroadphase::RigidBoundingBox() {
    custom_vector<real3>& aabb_min = data_manager->host_data.aabb_min;
    custom_vector<real3>& aabb_max = data_manager->host_data.aabb_max;
    if (aabb_min.size() == 0)
        return;

    // determine the bounds on the total space and subdivide based on the bins per axis
    bbox res(aabb_min[0], aabb_min[0]);
    bbox_transformation unary_op;
    bbox_reduction binary_op;
    res = thrust::transform_reduce(aabb_min.begin(), aabb_min.end(), unary_op, res, binary_op);
    res = thrust::transform_reduce(aabb_max.begin(), aabb_max.end(), unary_op, res, binary_op);

    real3& min_bounding_point = data_manager->measures.collision.rigid_min_bounding_point;
    real3& max_bounding_point = data_manager->measures.collision.rigid_max_bounding_point;

    min_bounding_point = res.first;
    max_bounding_point = res.second;
}
void ChCBroadphase::FluidBoundingBox() {
    const custom_vector<real3>& pos_fluid = data_manager->host_data.pos_3dof;
    const real radius = data_manager->node_container->kernel_radius + data_manager->node_container->collision_envelope;

    bbox res(pos_fluid[0], pos_fluid[0]);
    bbox_transformation unary_op;
    bbox_reduction binary_op;
    res = thrust::transform_reduce(pos_fluid.begin(), pos_fluid.end(), unary_op, res, binary_op);

    res.first.x = radius * Floor(res.first.x / radius);
    res.first.y = radius * Floor(res.first.y / radius);
    res.first.z = radius * Floor(res.first.z / radius);

    res.second.x = radius * Ceil(res.second.x / radius);
    res.second.y = radius * Ceil(res.second.y / radius);
    res.second.z = radius * Ceil(res.second.z / radius);

    real3& min_bounding_point = data_manager->measures.collision.ff_min_bounding_point;
    real3& max_bounding_point = data_manager->measures.collision.ff_max_bounding_point;

    max_bounding_point = res.second + radius * 6;
    min_bounding_point = res.first - radius * 6;
}
void ChCBroadphase::TetBoundingBox() {
    custom_vector<real3>& aabb_min_tet = data_manager->host_data.aabb_min_tet;
    custom_vector<real3>& aabb_max_tet = data_manager->host_data.aabb_max_tet;
    // determine the bounds on the total space and subdivide based on the bins per axis
    bbox res(aabb_min_tet[0], aabb_min_tet[0]);
    bbox_transformation unary_op;
    bbox_reduction binary_op;
    res = thrust::transform_reduce(aabb_min_tet.begin(), aabb_min_tet.end(), unary_op, res, binary_op);
    res = thrust::transform_reduce(aabb_max_tet.begin(), aabb_max_tet.end(), unary_op, res, binary_op);

    data_manager->measures.collision.tet_min_bounding_point = res.first;
    data_manager->measures.collision.tet_max_bounding_point = res.second;
}

void ChCBroadphase::DetermineBoundingBox() {
    RigidBoundingBox();

    real3 min_point = data_manager->measures.collision.rigid_min_bounding_point;
    real3 max_point = data_manager->measures.collision.rigid_max_bounding_point;

    if (data_manager->num_fluid_bodies != 0) {
        FluidBoundingBox();
        min_point = Min(min_point, data_manager->measures.collision.ff_min_bounding_point);
        max_point = Max(max_point, data_manager->measures.collision.ff_max_bounding_point);
    }

    if (data_manager->num_fea_tets != 0) {
        TetBoundingBox();
        min_point = Min(min_point, data_manager->measures.collision.tet_min_bounding_point);
        max_point = Max(max_point, data_manager->measures.collision.tet_max_bounding_point);
    }

    // Inflate the overall bounding box by a small percentage.
    // This takes care of corner cases where a degenerate object bounding box is on the
    // boundary of the overall bounding box.
    real fraction = 1e-3;
    real3 size = max_point - min_point;
    min_point = min_point - fraction * size;
    max_point = max_point + fraction * size;

    data_manager->measures.collision.min_bounding_point = min_point;
    data_manager->measures.collision.max_bounding_point = max_point;
    data_manager->measures.collision.global_origin = min_point;

    LOG(TRACE) << "ChCBroadphase::DetermineBoundingBox() Min : [" << min_point.x << ", " << min_point.y << ", "
               << min_point.z << "] Max: [" << max_point.x << ", " << max_point.y << ", " << max_point.z << "]";
}

void ChCBroadphase::OffsetAABB() {
    custom_vector<real3>& aabb_min = data_manager->host_data.aabb_min;
    custom_vector<real3>& aabb_max = data_manager->host_data.aabb_max;

    thrust::constant_iterator<real3> offset(data_manager->measures.collision.global_origin);

    thrust::transform(aabb_min.begin(), aabb_min.end(), offset, aabb_min.begin(), thrust::minus<real3>());
    thrust::transform(aabb_max.begin(), aabb_max.end(), offset, aabb_max.begin(), thrust::minus<real3>());
    // Offset tet aabb
    custom_vector<real3>& aabb_min_tet = data_manager->host_data.aabb_min_tet;
    custom_vector<real3>& aabb_max_tet = data_manager->host_data.aabb_max_tet;
    thrust::transform(aabb_min_tet.begin(), aabb_min_tet.end(), offset, aabb_min_tet.begin(), thrust::minus<real3>());
    thrust::transform(aabb_max_tet.begin(), aabb_max_tet.end(), offset, aabb_max_tet.begin(), thrust::minus<real3>());
}

// Determine resolution of the top level grid
void ChCBroadphase::ComputeTopLevelResolution() {
    const int num_shapes = data_manager->num_rigid_shapes;
    const real3& min_bounding_point = data_manager->measures.collision.min_bounding_point;
    const real3& max_bounding_point = data_manager->measures.collision.max_bounding_point;
    const real3& global_origin = data_manager->measures.collision.global_origin;
    const real density = data_manager->settings.collision.grid_density;

    vec3& bins_per_axis = data_manager->settings.collision.bins_per_axis;
    real3& inv_bin_size = data_manager->measures.collision.inv_bin_size;
    real3& bin_size = data_manager->measures.collision.bin_size;

    // This is the extents of the space aka diameter
    real3 diagonal = (Abs(max_bounding_point - global_origin));
    // Compute the number of slices in this grid level
    if (data_manager->settings.collision.fixed_bins == false) {
        bins_per_axis = function_Compute_Grid_Resolution(num_shapes, diagonal, density);
    }
    bin_size = diagonal / real3(bins_per_axis.x, bins_per_axis.y, bins_per_axis.z);
    LOG(TRACE) << "ChCBroadphase::ComputeTopLevelResolution() bins_per_axis: [" << bins_per_axis.x << ", "
               << bins_per_axis.y << ", " << bins_per_axis.z << "] bin_size: [" << bin_size.x << ", " << bin_size.y
               << ", " << bin_size.z << "]";

    // Store the inverse for use later
    inv_bin_size = 1.0 / bin_size;
}

// =========================================================================================================
ChCBroadphase::ChCBroadphase() {
    data_manager = 0;
}
// =========================================================================================================
// use spatial subdivision to detect the list of POSSIBLE collisions
// let user define their own narrow-phase collision detection
void ChCBroadphase::DispatchRigid() {
    if (data_manager->num_rigid_shapes != 0) {
        OneLevelBroadphase();
        data_manager->num_rigid_contacts = data_manager->measures.collision.number_of_contacts_possible;
    }
    return;
}

void ChCBroadphase::OneLevelBroadphase() {
    LOG(TRACE) << "ChCBroadphase::OneLevelBroadphase()";
    const custom_vector<real3>& aabb_min = data_manager->host_data.aabb_min;
    const custom_vector<real3>& aabb_max = data_manager->host_data.aabb_max;
    const custom_vector<short2>& fam_data = data_manager->shape_data.fam_rigid;
    const custom_vector<char>& obj_active = data_manager->host_data.active_rigid;
    const custom_vector<char>& obj_collide = data_manager->host_data.collide_rigid;
    const custom_vector<uint>& obj_data_id = data_manager->shape_data.id_rigid;
    custom_vector<long long>& contact_pairs = data_manager->host_data.contact_pairs;

    custom_vector<uint>& bin_intersections = data_manager->host_data.bin_intersections;
    custom_vector<uint>& bin_number = data_manager->host_data.bin_number;
    custom_vector<uint>& bin_number_out = data_manager->host_data.bin_number_out;
    custom_vector<uint>& bin_aabb_number = data_manager->host_data.bin_aabb_number;
    custom_vector<uint>& bin_start_index = data_manager->host_data.bin_start_index;
    custom_vector<uint>& bin_num_contact = data_manager->host_data.bin_num_contact;

    vec3& bins_per_axis = data_manager->settings.collision.bins_per_axis;
    const int num_shapes = data_manager->num_rigid_shapes;

    real3& inv_bin_size = data_manager->measures.collision.inv_bin_size;
    uint& number_of_bins_active = data_manager->measures.collision.number_of_bins_active;
    uint& number_of_bin_intersections = data_manager->measures.collision.number_of_bin_intersections;
    uint& number_of_contacts_possible = data_manager->measures.collision.number_of_contacts_possible;

    bin_intersections.resize(num_shapes + 1);
    bin_intersections[num_shapes] = 0;

#pragma omp parallel for
    for (int i = 0; i < num_shapes; i++) {
        f_Count_AABB_BIN_Intersection(i, inv_bin_size, aabb_min, aabb_max, bin_intersections);
    }

    Thrust_Exclusive_Scan(bin_intersections);
    number_of_bin_intersections = bin_intersections.back();

    LOG(TRACE) << "Number of bin intersections: " << number_of_bin_intersections;

    bin_number.resize(number_of_bin_intersections);
    bin_number_out.resize(number_of_bin_intersections);
    bin_aabb_number.resize(number_of_bin_intersections);
    bin_start_index.resize(number_of_bin_intersections);

#pragma omp parallel for
    for (int i = 0; i < num_shapes; i++) {
        f_Store_AABB_BIN_Intersection(i, bins_per_axis, inv_bin_size, aabb_min, aabb_max, bin_intersections, bin_number,
                                      bin_aabb_number);
    }

    Thrust_Sort_By_Key(bin_number, bin_aabb_number);
    number_of_bins_active = (int)(Run_Length_Encode(bin_number, bin_number_out, bin_start_index));

    if (number_of_bins_active <= 0) {
        number_of_contacts_possible = 0;
        return;
    }

    bin_start_index.resize(number_of_bins_active + 1);
    bin_start_index[number_of_bins_active] = 0;

    LOG(TRACE) << "Number of bins active: " << number_of_bins_active;

    Thrust_Exclusive_Scan(bin_start_index);
    bin_num_contact.resize(number_of_bins_active + 1);
    bin_num_contact[number_of_bins_active] = 0;

#pragma omp parallel for
    for (int i = 0; i < (signed)number_of_bins_active; i++) {
        f_Count_AABB_AABB_Intersection(i, inv_bin_size, bins_per_axis, aabb_min, aabb_max, bin_number_out,
                                       bin_aabb_number, bin_start_index, fam_data, obj_active, obj_collide, obj_data_id,
                                       bin_num_contact);
    }

    thrust::exclusive_scan(bin_num_contact.begin(), bin_num_contact.end(), bin_num_contact.begin());
    number_of_contacts_possible = bin_num_contact.back();
    contact_pairs.resize(number_of_contacts_possible);
    LOG(TRACE) << "Number of possible collisions: " << number_of_contacts_possible;

#pragma omp parallel for
    for (int index = 0; index < (signed)number_of_bins_active; index++) {
        f_Store_AABB_AABB_Intersection(index, inv_bin_size, bins_per_axis, aabb_min, aabb_max, bin_number_out,
                                       bin_aabb_number, bin_start_index, bin_num_contact, fam_data, obj_active,
                                       obj_collide, obj_data_id, contact_pairs);
    }
    contact_pairs.resize(number_of_contacts_possible);
    LOG(TRACE) << "Number of unique collisions: " << number_of_contacts_possible;
}
//======
}
}
