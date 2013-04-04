#include "ChCBroadphase.cuh"
using namespace chrono::collision;

__constant__ uint numAABB_const;
__constant__ uint last_active_bin_const;
__constant__ uint number_of_contacts_possible_const;
__constant__ real3 bin_size_vec_const;


typedef thrust::pair<real3, real3> bbox;
// reduce a pair of bounding boxes (a,b) to a bounding box containing a and b
struct bbox_reduction: public thrust::binary_function<bbox, bbox, bbox> {
    bbox __host__ __device__ operator()(
        bbox a,
        bbox b) {
        real3 ll = R3(fmin(a.first.x, b.first.x), fmin(a.first.y, b.first.y), fmin(a.first.z, b.first.z)); // lower left corner
        real3 ur = R3(fmax(a.second.x, b.second.x), fmax(a.second.y, b.second.y), fmax(a.second.z, b.second.z)); // upper right corner
        return bbox(ll, ur);
    }
};

// convert a point to a bbox containing that point, (point) -> (point, point)
struct bbox_transformation: public thrust::unary_function<real3, bbox> {
    bbox __host__ __device__ operator()(
        real3 point) {
        return bbox(point, point);
    }
};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ChCBroadphase::ChCBroadphase() {
    number_of_contacts_possible = 0;
    val = 0;
    last_active_bin = 0;
    number_of_bin_intersections = 0;
    numAABB = 0;
    bins_per_axis = R3(20, 20, 20);
    min_body_per_bin=25;
    max_body_per_bin=50;
    // TODO: Should make aabb_data organization less confusing, compiler should switch depending on if the user passes a host/device vector
    // TODO: Should be able to tune bins_per_axis, it's nice to have as a parameter though!
    // TODO: As the collision detection is progressing, we should free up vectors that are no longer being used! For example, Bin_Intersections is only used in steps 4&5
    // TODO: Make sure that aabb_data isn't being duplicated for this code (should be a reference to user's code to conserve space)
    // TODO: Fix debug mode to work with compiler settings, add more debugging features (a drawing function would be nice!)
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
int ChCBroadphase::setBinsPerAxis(real3 binsPerAxis) {
    bins_per_axis = binsPerAxis;
    return 0;
}
real3 ChCBroadphase::getBinsPerAxis() {
    return bins_per_axis;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
template<class T>
inline int3 __host__ __device__ HashMax( //CHANGED: For maximum point, need to check if point lies on edge of bin (TODO: Hmm, fmod still doesn't work completely)
        const T &A,
        const real3 & bin_size_vec) {
    int3 temp;
    temp.x = A.x / bin_size_vec.x;
    if(!fmod(A.x,bin_size_vec.x)&&temp.x!=0) temp.x--;
    temp.y = A.y / bin_size_vec.y;
    if(!fmod(A.y,bin_size_vec.y)&&temp.y!=0) temp.y--;
    temp.z = A.z / bin_size_vec.z;
    if(!fmod(A.z,bin_size_vec.z)&&temp.z!=0) temp.z--;

    //cout << temp.x << " " << temp.y << " " << temp.z << endl;
    return temp;
}

template<class T>
inline int3 __host__ __device__ HashMin(
    const T &A,
    const real3 &bin_size_vec) {
    int3 temp;
    temp.x = A.x / bin_size_vec.x;
    temp.y = A.y / bin_size_vec.y;
    temp.z = A.z / bin_size_vec.z;
    //cout << temp.x << " " << temp.y << " " << temp.z << endl;
    return temp;
}

template<class T>
inline uint __host__ __device__ Hash_Index(
    const T &A) {
    return ((A.x * 73856093) ^ (A.y * 19349663) ^ (A.z * 83492791));
}

//Function to Count AABB Bin intersections
inline void __host__ __device__ function_Count_AABB_BIN_Intersection(
    const uint &index,
    const real3 *aabb_data,
    const real3 &bin_size_vec,
    const uint &number_of_particles,
    uint *Bins_Intersected) {
    int3 gmin = HashMin(aabb_data[index], bin_size_vec);
    int3 gmax = HashMax(aabb_data[index + number_of_particles], bin_size_vec);
    Bins_Intersected[index] = (gmax.x - gmin.x + 1) * (gmax.y - gmin.y + 1) * (gmax.z - gmin.z + 1);
}
//--------------------------------------------------------------------------
__global__ void device_Count_AABB_BIN_Intersection(
    const real3 *aabb_data,
    uint *Bins_Intersected) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, numAABB_const)
    function_Count_AABB_BIN_Intersection(index, aabb_data, bin_size_vec_const, numAABB_const, Bins_Intersected);
}
//--------------------------------------------------------------------------
void ChCBroadphase::host_Count_AABB_BIN_Intersection(
    const real3 *aabb_data,
    uint *Bins_Intersected) {
    #pragma omp parallel for schedule(guided)
    for (int i = 0; i < numAABB; i++) {
        function_Count_AABB_BIN_Intersection(i, aabb_data, bin_size_vec, numAABB, Bins_Intersected);
    }
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Function to Store AABB Bin Intersections

inline void __host__ __device__ function_Store_AABB_BIN_Intersection(
    const uint &index,
    const real3 *aabb_data,
    const uint *Bins_Intersected,
    const real3 &bin_size_vec,
    const uint &number_of_particles,
    uint *bin_number,
    uint *body_number) {
    uint count = 0, i, j, k;
    int3 gmin = HashMin(aabb_data[index], bin_size_vec);
    int3 gmax = HashMax(aabb_data[index + number_of_particles], bin_size_vec);
    uint mInd = (index == 0) ? 0 : Bins_Intersected[index - 1];

    for (i = gmin.x; i <= gmax.x; i++) {
        for (j = gmin.y; j <= gmax.y; j++) {
            for (k = gmin.z; k <= gmax.z; k++) {
                bin_number[mInd + count] = Hash_Index(U3(i, j, k));
                body_number[mInd + count] = index;
                count++;
            }
        }
    }
}
//--------------------------------------------------------------------------
__global__ void device_Store_AABB_BIN_Intersection(
    const real3 *aabb_data,
    const uint *Bins_Intersected,
    uint *bin_number,
    uint *body_number) {
    INIT_CHECK_THREAD_BOUNDED(INDEX1D, numAABB_const);
    function_Store_AABB_BIN_Intersection(index, aabb_data, Bins_Intersected, bin_size_vec_const, numAABB_const, bin_number, body_number);
}
//--------------------------------------------------------------------------

void ChCBroadphase::host_Store_AABB_BIN_Intersection(
    const real3 *aabb_data,
    const uint *Bins_Intersected,
    uint *bin_number,
    uint *body_number) {
    #pragma omp parallel for schedule(guided)

    for (int i = 0; i < numAABB; i++) {
        function_Store_AABB_BIN_Intersection(i, aabb_data, Bins_Intersected, bin_size_vec, numAABB, bin_number, body_number);
    }
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Function to count AABB AABB intersection

inline void __host__ __device__ function_Count_AABB_AABB_Intersection(
    const uint &index,
    const real3 *aabb_data,
    const uint &number_of_particles,
    const uint *bin_number,
    const uint *body_number,
    const uint *bin_start_index,
    uint *Num_ContactD) {
    uint end = bin_start_index[index], count = 0, i = (!index) ? 0 : bin_start_index[index - 1];
    uint tempa, tempb;
    real3 Amin, Amax, Bmin, Bmax;

    for (; i < end; i++) {
        tempa = body_number[i];
        Amin = aabb_data[tempa];
        Amax = aabb_data[tempa + number_of_particles];

        for (int k = i + 1; k < end; k++) {
            tempb = body_number[k];
            Bmin = aabb_data[tempb];
            Bmax = aabb_data[tempb + number_of_particles];
            bool inContact = (Amin.x <= Bmax.x && Bmin.x <= Amax.x) && (Amin.y <= Bmax.y && Bmin.y <= Amax.y) && (Amin.z <= Bmax.z && Bmin.z <= Amax.z);

            if (inContact) count++;
        }
    }

    Num_ContactD[index] = count;
}
//--------------------------------------------------------------------------
__global__ void device_Count_AABB_AABB_Intersection(
    const real3 *aabb_data,
    const uint *bin_number,
    const uint *body_number,
    const uint *bin_start_index,
    uint *Num_ContactD) {
    INIT_CHECK_THREAD_BOUNDED(INDEX1D, last_active_bin_const);
    function_Count_AABB_AABB_Intersection(index, aabb_data, numAABB_const, bin_number, body_number, bin_start_index, Num_ContactD);
}

//--------------------------------------------------------------------------
void ChCBroadphase::host_Count_AABB_AABB_Intersection(
    const real3 *aabb_data,
    const uint *bin_number,
    const uint *body_number,
    const uint *bin_start_index,
    uint *Num_ContactD) {
    #pragma omp parallel for schedule(guided)

    for (int i = 0; i < last_active_bin; i++) {
        function_Count_AABB_AABB_Intersection(i, aabb_data, numAABB, bin_number, body_number, bin_start_index, Num_ContactD);
    }
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Function to store AABB-AABB intersections
inline void __host__ __device__ function_Store_AABB_AABB_Intersection(
    const uint &index,
    const real3 *aabb_data,
    const uint &number_of_particles,
    const uint *bin_number,
    const uint *body_number,
    const uint *bin_start_index,
    const uint *Num_ContactD,
    long long *potential_contacts) {
    uint end = bin_start_index[index], count = 0, i = (!index) ? 0 : bin_start_index[index - 1], Bin = bin_number[index];
    uint offset = (!index) ? 0 : Num_ContactD[index - 1];

    if (end - i == 1) {
        return;
    }

    uint tempa, tempb;
    real3 Amin, Amax, Bmin, Bmax;

    for (; i < end; i++) {
        ;
        tempa = body_number[i];
        Amin = aabb_data[tempa];
        Amax = aabb_data[tempa + number_of_particles];

        for (int k = i + 1; k < end; k++) {
            tempb = body_number[k];
            Bmin = aabb_data[tempb];
            Bmax = aabb_data[tempb + number_of_particles];
            bool inContact = (Amin.x <= Bmax.x && Bmin.x <= Amax.x) && (Amin.y <= Bmax.y && Bmin.y <= Amax.y) && (Amin.z <= Bmax.z && Bmin.z <= Amax.z);

            if (inContact == true) {
                int a = tempa;
                int b = tempb;

                if (b < a) {
                    int t = a;
                    a = b;
                    b = t;
                }

                potential_contacts[offset + count] = ((long long) a << 32 | (long long) b); //the two indicies of the objects that make up the contact
                count++;
            }
        }
    }
}
//--------------------------------------------------------------------------
__global__ void device_Store_AABB_AABB_Intersection(
    const real3 *aabb_data,
    const uint *bin_number,
    const uint *body_number,
    const uint *bin_start_index,
    const uint *Num_ContactD,
    long long *potential_contacts) {
    INIT_CHECK_THREAD_BOUNDED(INDEX1D, last_active_bin_const);
    function_Store_AABB_AABB_Intersection(index, aabb_data, numAABB_const, bin_number, body_number, bin_start_index, Num_ContactD, potential_contacts);
//--------------------------------------------------------------------------
}
void ChCBroadphase::host_Store_AABB_AABB_Intersection(
    const real3 *aabb_data,
    const uint *bin_number,
    const uint *body_number,
    const uint *bin_start_index,
    const uint *Num_ContactD,
    long long *potential_contacts) {
    #pragma omp parallel for schedule(guided)

    for (int index = 0; index < last_active_bin; index++) {
        function_Store_AABB_AABB_Intersection(index, aabb_data, numAABB, bin_number, body_number, bin_start_index, Num_ContactD, potential_contacts);
    }
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// use spatial subdivision to detect the list of POSSIBLE collisions (let user define their own narrow-phase collision detection)
int ChCBroadphase::detectPossibleCollisions(custom_vector<real3> &aabb_data, custom_vector<long long> &potentialCollisions) {
    custom_vector<uint> Bins_Intersected;
    custom_vector<uint> bin_number;
    custom_vector<uint> body_number;
    custom_vector<uint> bin_start_index;
    custom_vector<uint> Num_ContactD;
    double startTime = omp_get_wtime();
    numAABB = aabb_data.size()/2;
#ifdef DEBUG_GPU
    cout << "Number of AABBs: "<<numAABB<<endl;
#endif
    // STEP 1: Initialization TODO: this could be put in the constructor
#ifdef SIM_ENABLE_GPU_MODE
    // set the default cache configuration on the device to prefer a larger L1 cache and smaller shared memory
    cudaFuncSetCacheConfig(device_Count_AABB_BIN_Intersection, cudaFuncCachePreferL1);
    cudaFuncSetCacheConfig(device_Store_AABB_BIN_Intersection, cudaFuncCachePreferL1);
    cudaFuncSetCacheConfig(device_Count_AABB_AABB_Intersection, cudaFuncCachePreferL1);
    cudaFuncSetCacheConfig(device_Store_AABB_AABB_Intersection, cudaFuncCachePreferL1);
    COPY_TO_CONST_MEM(numAABB);
#endif
    potentialCollisions.clear();
    // END STEP 1
    // STEP 2: determine the bounds on the total space and subdivide based on the bins per axis
    bbox init = bbox(aabb_data[0], aabb_data[0]); // create a zero volume bounding box using the first set of aabb_data (??)
    bbox_transformation unary_op;
    bbox_reduction binary_op;
    bbox result = thrust::transform_reduce(aabb_data.begin(), aabb_data.end(), unary_op, init, binary_op);
    min_bounding_point = result.first;
    max_bounding_point = result.second;
    global_origin = fabs(min_bounding_point); //CHANGED: removed abs
    bin_size_vec = (fabs(max_bounding_point + fabs(min_bounding_point)));
    bin_size_vec =  bin_size_vec/bins_per_axis; //CHANGED: this was supposed to be reversed, CHANGED BACK this is just the inverse for convenience (saves us the divide later)
    thrust::transform(aabb_data.begin(), aabb_data.end(), thrust::constant_iterator<real3>(global_origin), aabb_data.begin(), thrust::plus<real3>()); //CHANGED: Should be a minus
#ifdef DEBUG_GPU
    cout << "Global Origin: (" << global_origin.x << ", " << global_origin.y << ", " << global_origin.z << ")" << endl;
    cout << "Maximum bounding point: (" << max_bounding_point.x << ", " << max_bounding_point.y << ", " << max_bounding_point.z << ")" << endl;
    cout << "Bin size vector: (" << bin_size_vec.x << ", " << bin_size_vec.y << ", " << bin_size_vec.z << ")" << endl;
#endif
    // END STEP 2
    // STEP 3: Count the number AABB's that lie in each bin, allocate space for each AABB
    Bins_Intersected.resize(numAABB); // TODO: how do you know how large to make this vector?
    // TODO: I think there is something wrong with the hash function...
#ifdef SIM_ENABLE_GPU_MODE
    COPY_TO_CONST_MEM(bin_size_vec);
    device_Count_AABB_BIN_Intersection __KERNEL__(BLOCKS(numAABB), THREADS)(CASTR3(aabb_data), CASTU1(Bins_Intersected));
#else
    host_Count_AABB_BIN_Intersection(aabb_data.data(), Bins_Intersected.data());
#endif
    Thrust_Inclusive_Scan_Sum(Bins_Intersected, number_of_bin_intersections);
#ifdef DEBUG_GPU
    cout << "Number of bin intersections: " << number_of_bin_intersections << endl;
#endif
    bin_number.resize(number_of_bin_intersections);
    body_number.resize(number_of_bin_intersections);
    bin_start_index.resize(number_of_bin_intersections);
    // END STEP 3
    // STEP 4: Indicate what bin each AABB belongs to, then sort based on bin number
#ifdef SIM_ENABLE_GPU_MODE
    device_Store_AABB_BIN_Intersection __KERNEL__(BLOCKS(numAABB), THREADS)(CASTR3(aabb_data), CASTU1(Bins_Intersected), CASTU1(bin_number), CASTU1(body_number));
#else
    host_Store_AABB_BIN_Intersection(aabb_data.data(), Bins_Intersected.data(),
                                     bin_number.data(), body_number.data());
#endif
#ifdef DEBUG_GPU
    cout<<"DONE WID DAT (device_Store_AABB_BIN_Intersection)"<<endl;
#endif
//    for(int i=0; i<bin_number.size(); i++){
//    	cout<<bin_number[i]<<" "<<body_number[i]<<endl;
//    }



    Thrust_Sort_By_Key(bin_number, body_number);


//    for(int i=0; i<bin_number.size(); i++){
//    	cout<<bin_number[i]<<" "<<body_number[i]<<endl;
//    }



#ifdef DEBUG_GPU
#endif

   Thrust_Reduce_By_KeyA(last_active_bin, bin_number,bin_start_index);



//    host_vector<uint> bin_number_t=bin_number;
//    host_vector<uint> bin_start_index_t(number_of_bin_intersections);
//    host_vector<uint> Output(number_of_bin_intersections);
//    thrust::pair<uint*,uint*> new_end;
//    last_active_bin= thrust::reduce_by_key(bin_number_t.begin(),bin_number_t.end(),thrust::constant_iterator<uint>(1),Output.begin(),bin_start_index_t.begin()).first-Output.begin();
//
//
//    bin_number=Output;
//    bin_start_index=bin_start_index_t;

#ifdef DEBUG_GPU
#endif
////      //QUESTION: I have no idea what is going on here
    val =
        bin_start_index[thrust::max_element(bin_start_index.begin(),
                                            bin_start_index.begin() + last_active_bin)
                        - bin_start_index.begin()];

    if (val > max_body_per_bin) {
        bins_per_axis = bins_per_axis * 1.1;
    } else if (val < min_body_per_bin && val > 10) {
        bins_per_axis = bins_per_axis * .9;
    }
//cout <<val<<" "<<bins_per_axis.x<<" "<<bins_per_axis.y<<" "<<bins_per_axis.z<<endl;
    bin_start_index.resize(last_active_bin);
#ifdef DEBUG_GPU
    cout << "Last active bin: " << last_active_bin << endl;
#endif
    Thrust_Inclusive_Scan(bin_start_index);
    Num_ContactD.resize(last_active_bin);
    // END STEP 4
    // STEP 5: Count the number of AABB collisions
#ifdef SIM_ENABLE_GPU_MODE
    COPY_TO_CONST_MEM(last_active_bin);
    device_Count_AABB_AABB_Intersection __KERNEL__(BLOCKS(last_active_bin), THREADS)(
        CASTR3(aabb_data),
        CASTU1(bin_number),
        CASTU1(body_number),
        CASTU1(bin_start_index),
        CASTU1(Num_ContactD));
#else
    host_Count_AABB_AABB_Intersection(aabb_data.data(), bin_number.data(), body_number.data(), bin_start_index.data(), Num_ContactD.data());
#endif
    Thrust_Inclusive_Scan_Sum(Num_ContactD, number_of_contacts_possible);
    potentialCollisions.resize(number_of_contacts_possible);
#ifdef DEBUG_GPU
    cout << "Number of possible collisions: " << number_of_contacts_possible << endl;
#endif
    // END STEP 5
    // STEP 6: Store the possible AABB collision pairs
#ifdef SIM_ENABLE_GPU_MODE
    device_Store_AABB_AABB_Intersection __KERNEL__(BLOCKS(last_active_bin), THREADS)(
        CASTR3(aabb_data),
        CASTU1(bin_number),
        CASTU1(body_number),
        CASTU1(bin_start_index),
        CASTU1(Num_ContactD),
        CASTLL(potentialCollisions));
#else
    host_Store_AABB_AABB_Intersection(aabb_data.data(),
                                      bin_number.data(),
                                      body_number.data(),
                                      bin_start_index.data(),
                                      Num_ContactD.data(),
                                      potentialCollisions.data());
#endif
    thrust::sort(potentialCollisions.begin(), potentialCollisions.end());
    number_of_contacts_possible = thrust::unique(potentialCollisions.begin(),
                                  potentialCollisions.end()) - potentialCollisions.begin();

    potentialCollisions.resize(number_of_contacts_possible);
#ifdef DEBUG_GPU
    cout << "Number of possible collisions: " << number_of_contacts_possible << endl;
#endif
    // END STEP 6
    double endTime = omp_get_wtime();
#ifdef DEBUG_GPU
    printf("Time to detect: %lf seconds\n", (endTime - startTime));
#endif
    return 0;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


