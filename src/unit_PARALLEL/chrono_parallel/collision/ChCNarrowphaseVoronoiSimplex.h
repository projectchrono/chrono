#ifndef CHC_NARROWPHASE_VORONOISIMPLEX_H
#define CHC_NARROWPHASE_VORONOISIMPLEX_H

namespace chrono {
namespace collision {

#define VORONOI_SIMPLEX_MAX_VERTS 5

///disable next define, or use defaultCollisionConfiguration->getSimplexSolver()->setEqualVertexThreshold(0.f) to disable/configure
#define BT_USE_EQUAL_VERTEX_THRESHOLD
#define VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD 0.0001f

struct btUsageBitfield {
   btUsageBitfield() {
      reset();
   }

   void reset() {
      usedVertexA = false;
      usedVertexB = false;
      usedVertexC = false;
      usedVertexD = false;
   }
   unsigned short usedVertexA :1;
   unsigned short usedVertexB :1;
   unsigned short usedVertexC :1;
   unsigned short usedVertexD :1;
   unsigned short unused1 :1;
   unsigned short unused2 :1;
   unsigned short unused3 :1;
   unsigned short unused4 :1;
};

struct btSubSimplexClosestResult {
   real3 m_closestPointOnSimplex;
   //MASK for m_usedVertices
   //stores the simplex vertex-usage, using the MASK,
   // if m_usedVertices & MASK then the related vertex is used
   btUsageBitfield m_usedVertices;
   real m_barycentricCoords[4];
   bool m_degenerate;

   void reset() {
      m_degenerate = false;
      setBarycentricCoordinates();
      m_usedVertices.reset();
   }
   bool isValid() {
      bool valid = (m_barycentricCoords[0] >= real(0.)) && (m_barycentricCoords[1] >= real(0.)) && (m_barycentricCoords[2] >= real(0.)) && (m_barycentricCoords[3] >= real(0.));

      return valid;
   }
   void setBarycentricCoordinates(real a = real(0.),
                                  real b = real(0.),
                                  real c = real(0.),
                                  real d = real(0.)) {
      m_barycentricCoords[0] = a;
      m_barycentricCoords[1] = b;
      m_barycentricCoords[2] = c;
      m_barycentricCoords[3] = d;
   }

};

// Minkovski Portal Refinement narrowphase collision detection
class CH_PARALLEL_API ChCNarrowphaseVoronoiSimplex {
 public:

   int m_numVertices;

   real3 m_simplexVectorW[VORONOI_SIMPLEX_MAX_VERTS];
   real3 m_simplexPointsP[VORONOI_SIMPLEX_MAX_VERTS];
   real3 m_simplexPointsQ[VORONOI_SIMPLEX_MAX_VERTS];

   real3 m_cachedP1;
   real3 m_cachedP2;
   real3 m_cachedV;
   real3 m_lastW;

   real m_equalVertexThreshold;
   bool m_cachedValidClosest;

   btSubSimplexClosestResult m_cachedBC;

   bool m_needsUpdate;

   void removeVertex(int index);
   void reduceVertices(const btUsageBitfield& usedVerts);
   bool updateClosestVectorAndPoints();

   bool closestPtPointTetrahedron(const real3& p,
                                  const real3& a,
                                  const real3& b,
                                  const real3& c,
                                  const real3& d,
                                  btSubSimplexClosestResult& finalResult);
   int pointOutsideOfPlane(const real3& p,
                           const real3& a,
                           const real3& b,
                           const real3& c,
                           const real3& d);
   bool closestPtPointTriangle(const real3& p,
                               const real3& a,
                               const real3& b,
                               const real3& c,
                               btSubSimplexClosestResult& result);

 public:

   ChCNarrowphaseVoronoiSimplex()
         : m_equalVertexThreshold(VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD) {
   }
   void reset();

   void addVertex(const real3& w,
                  const real3& p,
                  const real3& q);

   void setEqualVertexThreshold(real threshold) {
      m_equalVertexThreshold = threshold;
   }

   real getEqualVertexThreshold() const {
      return m_equalVertexThreshold;
   }

   bool closest(real3& v);

   real maxVertex();

   bool fullSimplex() const {
      return (m_numVertices == 4);
   }

   int getSimplex(real3 *pBuf,
                  real3 *qBuf,
                  real3 *yBuf) const;

   bool inSimplex(const real3& w);

   void backup_closest(real3& v);

   bool emptySimplex() const;

   void compute_points(real3& p1,
                       real3& p2);

   int numVertices() const {
      return m_numVertices;
   }

};

}  // end namespace collision
}  // end namespace chrono

#endif

