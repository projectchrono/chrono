// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#include "collision/ChCCollisionSystemSpheres.h"
#include "collision/ChCModelSphereSet.h"
#include "physics/ChBody.h"
#include "physics/ChContactContainer.h"
#include "physics/ChProximityContainer.h"

namespace chrono {
namespace collision {

ChCollisionSystemSpheres::ChCollisionSystemSpheres(unsigned int max_objects, double scene_size) {
    number_of_particles = 0;
    number_of_bodies = 0;
    particle_list = new ChCollisionSpheres();
    contact_list = new ChContacts();

    bins_per_axis = realV(10, 10, 10);  // 100, 100, 100

    min_bounding_point = realV(0.0, 0.0, 0.0);
    max_bounding_point = realV(0.0, 0.0, 0.0);
    global_origin = realV(0.0, 0.0, 0.0);
}

ChCollisionSystemSpheres::~ChCollisionSystemSpheres() {
    if (particle_list)
        delete particle_list;
    particle_list = 0;
    if (contact_list)
        delete contact_list;
    contact_list = 0;
}

void ChCollisionSystemSpheres::Clear(void) {
    potential_contacts.clear();
    aabb_data.clear();
    Bins_Intersected.clear();
    bin_number.clear();
    body_number.clear();
    bin_start_index.clear();
    Num_ContactD.clear();

    contact_list->clear();
}

void ChCollisionSystemSpheres::Add(ChCollisionModel* model) {
    if (model->GetPhysicsItem()->GetCollide() == true) {
        int bodID = ((ChBody*)model->GetPhysicsItem())->GetIdentifier();

        ChModelSphereSet* body = (ChModelSphereSet*)model;
        body->SyncPosition();
        collModels.push_back(body);
        thrust::host_vector<ChVector<float> > gPosVec;
        body->GetGlobalSpherePos(gPosVec);
        thrust::host_vector<float> gRadVec;
        body->GetSphereRad(gRadVec);
        particle_list->add(bodID, gPosVec, gRadVec);
        number_of_bodies++;
        number_of_particles += gPosVec.size();
    }
}

void ChCollisionSystemSpheres::Remove(ChCollisionModel* model) {
    if (model->GetPhysicsItem()->GetCollide() == true) {
        int bodID = model->GetPhysicsItem()->GetIdentifier();
        int bodyIndex = -1;
        ChModelSphereSet* curBod = (ChModelSphereSet*)model;
        for (uint i = 0; i < number_of_bodies; i++) {
            curBod = collModels[i];
            if (curBod->GetPhysicsItem()->GetIdentifier() == bodID) {
                bodyIndex = i;
                break;
            }
        }
        if (bodyIndex == -1)
            return;

        number_of_particles -= (collModels[bodyIndex]->getNSpheres());
        collModels.erase(collModels.begin() + bodyIndex);
        number_of_bodies--;
    }
}

void ChCollisionSystemSpheres::Run() {
    double startTime = omp_get_wtime();

    updateDataStructures();  // TOBY???

    number_of_particles = particle_list->num_particles;
    aabb_data.resize(number_of_particles * 2);
    potential_contacts.clear();
    contact_list->clear();

    host_Generate_AABB(particle_list->pos.data(), particle_list->radius.data(), aabb_data.data());

    //##############################################################################################
    bbox init = bbox(aabb_data[0], aabb_data[0]);
    bbox_transformation unary_op;
    bbox_reduction binary_op;
    bbox result = thrust::transform_reduce(aabb_data.begin(), aabb_data.end(), unary_op, init, binary_op);
    min_bounding_point = result.first;
    max_bounding_point = result.second;
    global_origin = realV(fabs(min_bounding_point.x), fabs(min_bounding_point.y), fabs(min_bounding_point.z));
    bin_size_vec = realV(fabs((max_bounding_point + global_origin).x), fabs((max_bounding_point + global_origin).y),
                         fabs((max_bounding_point + global_origin).z));
    bin_size_vec = bins_per_axis / bin_size_vec;
    thrust::transform(aabb_data.begin(), aabb_data.end(), thrust::constant_iterator<realV>(global_origin),
                      aabb_data.begin(), thrust::plus<realV>());
    //##############################################################################################
    Bins_Intersected.resize(number_of_particles);

    host_Count_AABB_BIN_Intersection(aabb_data.data(), Bins_Intersected.data());

    Thrust_Inclusive_Scan_Sum(Bins_Intersected, number_of_bin_intersections);
    // std::cout << "NBI " << number_of_bin_intersections << ", ";
    bin_number.resize(number_of_bin_intersections);
    body_number.resize(number_of_bin_intersections);
    bin_start_index.resize(number_of_bin_intersections);

    host_Store_AABB_BIN_Intersection(aabb_data.data(), Bins_Intersected.data(), bin_number.data(), body_number.data());

    Thrust_Sort_By_Key(bin_number, body_number);
    Thrust_Reduce_By_KeyA(last_active_bin, bin_number, bin_start_index);

    val = bin_start_index[thrust::max_element(bin_start_index.begin(), bin_start_index.begin() + last_active_bin) -
                          bin_start_index.begin()];
    if (val > 50) {
        bins_per_axis = bins_per_axis * 1.1;
    } else if (val < 25 && val > 1) {
        bins_per_axis = bins_per_axis * .9;
    }
    bin_start_index.resize(last_active_bin);
    // std::cout << "LAB " << last_active_bin << ", ";
    Thrust_Inclusive_Scan(bin_start_index);
    Num_ContactD.resize(last_active_bin);

    host_Count_AABB_AABB_Intersection(aabb_data.data(), bin_number.data(), body_number.data(),
                                      particle_list->bodyIndex.data(), bin_start_index.data(),
                                      particle_list->active.data(), Num_ContactD.data());

    Thrust_Inclusive_Scan_Sum(Num_ContactD, number_of_contacts_possible);
    potential_contacts.resize(number_of_contacts_possible);
    // std::cout << "NCP1 " << number_of_contacts_possible << ", ";

    host_Store_AABB_AABB_Intersection(aabb_data.data(), bin_number.data(), body_number.data(),
                                      particle_list->bodyIndex.data(), bin_start_index.data(), Num_ContactD.data(),
                                      particle_list->active.data(), potential_contacts.data());

    thrust::sort(potential_contacts.begin(), potential_contacts.end());
    number_of_contacts_possible =
        thrust::unique(potential_contacts.begin(), potential_contacts.end()) - potential_contacts.begin();
    // std::cout << "NCP2 " << number_of_contacts_possible << ", ";
    //##############################################################################################
    contact_list->resize(number_of_contacts_possible);
    Num_ContactD.resize(number_of_contacts_possible);

    Thrust_Fill(Num_ContactD, 1);

    host_Store_Contact(potential_contacts.data(), particle_list->pos.data(), particle_list->radius.data(),
                       particle_list->bodyIndex.data(), contact_list->ida.data(), contact_list->idb.data(),
                       contact_list->pta.data(), contact_list->ptb.data(), contact_list->N.data(),
                       contact_list->depth.data(), contact_list->rest_len.data(), Num_ContactD.data());

    thrust::sort_by_key(
        Num_ContactD.begin(), Num_ContactD.end(),
        thrust::make_zip_iterator(thrust::make_tuple(
            contact_list->ida.begin(), contact_list->idb.begin(), contact_list->pta.begin(), contact_list->ptb.begin(),
            contact_list->N.begin(), contact_list->depth.begin(), contact_list->rest_len.begin())));

    number_of_contacts = Thrust_Count(Num_ContactD, 0);
    // std::cout << "NC " << number_of_contacts << "\n";
    contact_list->resize(number_of_contacts);
    contact_list->num_contacts = number_of_contacts;

    //##############################################################################################

    double endTime = omp_get_wtime();
    // printf("CD %lf  ", (endTime - startTime)); //TOBY: Comment this out
    // printf("BPA %f,%f,%f ",bins_per_axis.x, bins_per_axis.y, bins_per_axis.z); //TOBY: Comment this out
    // printf("Spheres %i Contacts %i\n", number_of_particles,number_of_contacts); //TOBY: Comment this out
}

template <class T>
inline int3 Hash(const T& A, const realV& bin_size_vec) {
    int3 temp;
    temp.x = A.x * bin_size_vec.x;
    temp.y = A.y * bin_size_vec.y;
    temp.z = A.z * bin_size_vec.z;
    return temp;
}

template <class T>
inline uint Hash_Index(const T& A) {
    return ((A.x * 73856093) ^ (A.y * 19349663) ^ (A.z * 83492791));
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Function to Generate AABB

inline void function_Generate_AABB(const uint& index,
                                   const realV* pos,
                                   const real* radius,
                                   const uint& number_of_particles,
                                   realV* aabb_data) {
    aabb_data[index] = pos[index] - realV(radius[index]);
    aabb_data[index + number_of_particles] = pos[index] + realV(radius[index]);
}

void ChCollisionSystemSpheres::host_Generate_AABB(const realV* pos, const real* radius, realV* aabb_data) {
    for (uint i = 0; i < number_of_particles; i++) {
        function_Generate_AABB(i, pos, radius, number_of_particles, aabb_data);
    }
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Function to Count AABB Bin intersections

inline void function_Count_AABB_BIN_Intersection(const uint& index,
                                                 const realV* aabb_data,
                                                 const realV& bin_size_vec,
                                                 const uint& number_of_particles,
                                                 uint* Bins_Intersected) {
    int3 gmin = Hash(aabb_data[index], bin_size_vec);
    int3 gmax = Hash(aabb_data[index + number_of_particles], bin_size_vec);
    Bins_Intersected[index] = (gmax.x - gmin.x + 1) * (gmax.y - gmin.y + 1) * (gmax.z - gmin.z + 1);
}

void ChCollisionSystemSpheres::host_Count_AABB_BIN_Intersection(const realV* aabb_data, uint* Bins_Intersected) {
    for (uint i = 0; i < number_of_particles; i++) {
        function_Count_AABB_BIN_Intersection(i, aabb_data, bin_size_vec, number_of_particles, Bins_Intersected);
    }
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Function to Store AABB Bin Intersections

inline void function_Store_AABB_BIN_Intersection(const uint& index,
                                                 const realV* aabb_data,
                                                 const uint* Bins_Intersected,
                                                 const realV& bin_size_vec,
                                                 const uint& number_of_particles,
                                                 uint* bin_number,
                                                 uint* body_number) {
    uint count = 0, i, j, k;
    int3 gmin = Hash(aabb_data[index], bin_size_vec);
    int3 gmax = Hash(aabb_data[index + number_of_particles], bin_size_vec);
    uint mInd = (index == 0) ? 0 : Bins_Intersected[index - 1];

    for (i = gmin.x; i <= gmax.x; i++) {
        for (j = gmin.y; j <= gmax.y; j++) {
            for (k = gmin.z; k <= gmax.z; k++) {
                bin_number[mInd + count] = Hash_Index(chrono::ChVector<uint>(i, j, k));
                body_number[mInd + count] = index;
                count++;
            }
        }
    }
}

void ChCollisionSystemSpheres::host_Store_AABB_BIN_Intersection(const realV* aabb_data,
                                                                const uint* Bins_Intersected,
                                                                uint* bin_number,
                                                                uint* body_number) {
    for (uint i = 0; i < number_of_particles; i++) {
        function_Store_AABB_BIN_Intersection(i, aabb_data, Bins_Intersected, bin_size_vec, number_of_particles,
                                             bin_number, body_number);
    }
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Function to count AABB AABB intersection

inline void function_Count_AABB_AABB_Intersection(const uint& index,
                                                  const realV* aabb_data,
                                                  const uint& number_of_particles,
                                                  const uint* bin_number,
                                                  const uint* body_number,
                                                  const uint* bodyIndex,
                                                  const uint* bin_start_index,
                                                  const bool* active,
                                                  uint* Num_ContactD) {
    uint end = bin_start_index[index], count = 0, i = (!index) ? 0 : bin_start_index[index - 1];
    uint tempa, tempb;
    AABB A, B;
    for (; i < end; i++) {
        tempa = body_number[i];
        A.min = aabb_data[tempa];
        A.max = aabb_data[tempa + number_of_particles];
        for (int k = i + 1; k < end; k++) {
            tempb = body_number[k];
            B.min = aabb_data[tempb];
            B.max = aabb_data[tempb + number_of_particles];
            bool inContact = (A.min.x <= B.max.x && B.min.x <= A.max.x) && (A.min.y <= B.max.y && B.min.y <= A.max.y) &&
                             (A.min.z <= B.max.z && B.min.z <= A.max.z);
            if (inContact == true && tempa != tempb && bodyIndex[tempa] != bodyIndex[tempb] &&
                (active[tempa] || active[tempb])) {
                count++;
            }
        }
    }
    Num_ContactD[index] = count;
}

void ChCollisionSystemSpheres::host_Count_AABB_AABB_Intersection(const realV* aabb_data,
                                                                 const uint* bin_number,
                                                                 const uint* body_number,
                                                                 const uint* bodyIndex,
                                                                 const uint* bin_start_index,
                                                                 const bool* active,
                                                                 uint* Num_ContactD) {
    for (uint i = 0; i < last_active_bin; i++) {
        function_Count_AABB_AABB_Intersection(i, aabb_data, number_of_particles, bin_number, body_number, bodyIndex,
                                              bin_start_index, active, Num_ContactD);
    }
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Function to process single contact
inline void function_Store_AABB_AABB_Intersection(const uint& index,
                                                  const realV* aabb_data,
                                                  const uint& number_of_particles,
                                                  const uint* bin_number,
                                                  const uint* body_number,
                                                  const uint* bodyIndex,
                                                  const uint* bin_start_index,
                                                  const uint* Num_ContactD,
                                                  const bool* active,
                                                  long long* potential_contacts) {
    uint end = bin_start_index[index], count = 0, i = (!index) ? 0 : bin_start_index[index - 1],
         Bin = bin_number[index];
    uint offset = (!index) ? 0 : Num_ContactD[index - 1];
    if (end - i == 1) {
        return;
    }
    uint tempa, tempb;
    AABB A, B;
    for (; i < end; i++) {
        ;
        tempa = body_number[i];
        A.min = aabb_data[tempa];
        A.max = aabb_data[tempa + number_of_particles];

        for (int k = i + 1; k < end; k++) {
            tempb = body_number[k];

            B.min = aabb_data[tempb];
            B.max = aabb_data[tempb + number_of_particles];

            bool inContact = (A.min.x <= B.max.x && B.min.x <= A.max.x) && (A.min.y <= B.max.y && B.min.y <= A.max.y) &&
                             (A.min.z <= B.max.z && B.min.z <= A.max.z);
            if (inContact == true && tempa != tempb && bodyIndex[tempa] != bodyIndex[tempb] &&
                (active[tempa] || active[tempb])) {
                int a = tempa;
                int b = tempb;
                if (b < a) {
                    int t = a;
                    a = b;
                    b = t;
                }
                potential_contacts[offset + count] =
                    ((long long)a << 32 | (long long)b);  // the two indicies of the objects that make up the contact
                count++;
            }
        }
    }
}

void ChCollisionSystemSpheres::host_Store_AABB_AABB_Intersection(const realV* aabb_data,
                                                                 const uint* bin_number,
                                                                 const uint* body_number,
                                                                 const uint* bodyIndex,
                                                                 const uint* bin_start_index,
                                                                 const uint* Num_ContactD,
                                                                 const bool* active,
                                                                 long long* potential_contacts) {
    for (uint index = 0; index < last_active_bin; index++) {
        function_Store_AABB_AABB_Intersection(index, aabb_data, number_of_particles, bin_number, body_number, bodyIndex,
                                              bin_start_index, Num_ContactD, active, potential_contacts);
    }
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

inline void function_Store_Contact(const uint& index,
                                   const long long* potential_contacts,
                                   const realV* pos,
                                   const float* radius,
                                   const uint* bodyIndex,
                                   uint* id_a,
                                   uint* id_b,
                                   realV* cpt_a,
                                   realV* cpt_b,
                                   realV* Norm,
                                   float* c_dist,
                                   float* rest_len,
                                   uint* counter) {
    int ida = int(potential_contacts[index] >> 32);
    int idb = int(potential_contacts[index] & 0xffffffff);

    realV pa = pos[ida];
    realV pb = pos[idb];
    realV relPos = pb - pa;

    real dist = relPos.Length();
    realV N = relPos.GetNormalized();
    real collideDist = radius[ida] + radius[idb];

    if (dist < collideDist) {
        id_a[index] = bodyIndex[ida];
        id_b[index] = bodyIndex[idb];
        cpt_a[index] = pa + N * radius[ida];
        cpt_b[index] = pb - N * radius[idb];
        Norm[index] = N;
        c_dist[index] = collideDist - dist;
        rest_len[index] = dist;
        counter[index] = 0;
    }
}

void ChCollisionSystemSpheres::host_Store_Contact(const long long* potential_contacts,
                                                  const realV* pos,
                                                  const float* radius,
                                                  const uint* bodyIndex,
                                                  uint* id_a,
                                                  uint* id_b,
                                                  realV* cpt_a,
                                                  realV* cpt_b,
                                                  realV* Norm,
                                                  float* c_dist,
                                                  float* rest_len,
                                                  uint* counter) {
    for (uint i = 0; i < number_of_contacts_possible; i++) {
        function_Store_Contact(i, potential_contacts, pos, radius, bodyIndex, id_a, id_b, cpt_a, cpt_b, Norm, c_dist,
                               rest_len, counter);
    }
}

void ChCollisionSystemSpheres::updateDataStructures() {
    int index = 0;
    ChVector<> tMin, tMax;
    float xm, xM, ym, yM, zm, zM;
    ChVector<> globalPos;
    float size;
    ChModelSphereSet* body;
    int spheresToAdd;
    int fam;
    int noC;

    particle_list->reset(number_of_bodies, number_of_particles);

    for (uint i = 0; i < number_of_bodies; i++) {
        body = collModels[i];
        // update all spheres in this body
        thrust::host_vector<ChVector<float> > gPosVec;
        body->GetGlobalSpherePos(gPosVec);
        thrust::host_vector<float> gRadVec;
        body->GetSphereRad(gRadVec);
        particle_list->set(i, body->GetPhysicsItem()->GetIdentifier(), gPosVec, gRadVec);
    }
}

void ChCollisionSystemSpheres::ReportContacts(ChContactContainer* mcontactcontainer) {
    // This should remove all old contacts (or at least rewind the index)
    mcontactcontainer->BeginAddContact();

    ChCollisionInfo icontact;

    double ptdist;
    double envelopeA;
    double envelopeB;
    chrono::ChVector<> vpA;
    chrono::ChVector<> vpB;
    chrono::ChVector<> vpN;
    ChModelSphereSet* colModelA;
    ChModelSphereSet* colModelB;

    for (int i = 0; i < contact_list->num_contacts; i++) {
        colModelA = collModels[contact_list->ida[i]];
        colModelB = collModels[contact_list->idb[i]];

        envelopeA = colModelA->GetEnvelope();
        envelopeB = colModelB->GetEnvelope();

        vpA.x = contact_list->pta[i].x;
        vpA.y = contact_list->pta[i].y;
        vpA.z = contact_list->pta[i].z;
        vpB.x = contact_list->ptb[i].x;
        vpB.y = contact_list->ptb[i].y;
        vpB.z = contact_list->ptb[i].z;
        vpN.x = contact_list->N[i].x;  // sign?
        vpN.y = contact_list->N[i].y;  // sign?
        vpN.z = contact_list->N[i].z;  // sign?

        vpN.Normalize();
        ptdist = -contact_list->depth[i];  // sign?

        vpA = vpA - vpN * envelopeA;
        vpB = vpB + vpN * envelopeB;
        ptdist = ptdist + envelopeA + envelopeB;

        icontact.modelA = (ChCollisionModel*)colModelA;
        icontact.modelB = (ChCollisionModel*)colModelB;
        icontact.vpA = vpA;
        icontact.vpB = vpB;
        icontact.vN = vpN;
        icontact.distance = ptdist;
        icontact.reaction_cache = 0;  // TOBY ???
        mcontactcontainer->AddContact(icontact);
    }
    mcontactcontainer->EndAddContact();
}

void ChCollisionSystemSpheres::ReportProximities(ChProximityContainer* mproximitycontainer) {
    mproximitycontainer->BeginAddProximities();

    // TOBY ????

    mproximitycontainer->EndAddProximities();
}

bool ChCollisionSystemSpheres::RayHit(const ChVector<>& from, const ChVector<>& to, ChRayhitResult& mresult) {
    return false;
}

ChCollisionSpheres::ChCollisionSpheres() {
    this->num_particles = 0;
    this->num_bodies = 0;
}
ChCollisionSpheres::~ChCollisionSpheres() {
    num_particles = 0;
    num_bodies = 0;
    pos.clear();
    radius.clear();
    active.clear();
    bodyID.clear();
    bodyIndex.clear();
    bID.clear();
}
void ChCollisionSpheres::add(int bID, const thrust::host_vector<realV>& sPos, const thrust::host_vector<real>& sRad) {
    this->bID.push_back(bID);

    for (int i = 0; i < sPos.size(); i++) {
        pos.push_back(sPos[i]);
        radius.push_back(sRad[i]);
        active.push_back(true);
        bodyIndex.push_back(num_bodies);
        bodyID.push_back(bID);
    }
    num_particles += sPos.size();
    num_bodies++;
}

void ChCollisionSpheres::set(int bInd,
                             int bIDnum,
                             const thrust::host_vector<realV>& sPos,
                             const thrust::host_vector<real>& sRad) {
    uint nSph = sPos.size();

    bID[bInd] = bIDnum;
    for (int i = 0; i < nSph; i++) {
        pos[setPos + i] = sPos[i];
        radius[setPos + i] = sRad[i];
        active[setPos + i] = true;
        bodyIndex[setPos + i] = bInd;
        bodyID[setPos + i] = bIDnum;
    }

    setPos += nSph;
}

void ChCollisionSpheres::reset(int n_bodies, int n_particles) {
    setPos = 0;
    num_particles = n_particles;
    num_bodies = n_bodies;
    pos.resize(n_particles);
    radius.resize(n_particles);
    active.resize(n_particles);
    bodyID.resize(n_particles);
    bodyIndex.resize(n_particles);
    bID.resize(n_bodies);
}

ChContacts::ChContacts() {
}
ChContacts::~ChContacts() {
}
void ChContacts::clear() {
    num_contacts = 0;
    ida.clear();
    idb.clear();
    N.clear();
    depth.clear();
    rest_len.clear();
    pta.clear();
    ptb.clear();
}
void ChContacts::resize(uint size) {
    num_contacts = size;
    ida.resize(size);
    idb.resize(size);
    N.resize(size);
    depth.resize(size);
    rest_len.resize(size);
    pta.resize(size);
    ptb.resize(size);
}

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____
