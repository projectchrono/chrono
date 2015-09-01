/*
 Bullet Continuous Collision Detection and Physics Library
 Copyright (c) 2003-2008 Erwin Coumans  http://continuousphysics.com/Bullet/

 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the
 use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it
 freely,
 subject to the following restrictions:

 1. The origin of this software must not be misrepresented; you must not
 claim that you wrote the original software. If you use this software in a
 product, an acknowledgment in the product documentation would be appreciated
 but is not required.
 2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */

/*
 GJK-EPA collision solver by Nathanael Presson, 2008
 */

#include <algorithm>

#include "collision/ChCCollisionModel.h"

#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/collision/ChCNarrowphaseGJK_EPA.h"
#include "chrono_parallel/collision/ChCNarrowphaseVoronoiSimplex.h"
#include <chrono_parallel/collision/ChCNarrowphaseUtils.h>
#include "core/ChMathematics.h"
namespace chrono {
namespace collision {

/*Future reference:
 * Start by looking at btGjkPairDetector class
 * This will lead to btGjkEpaPenetrationDepthSolver and btGjkEpaSolver2
 * Contact points are stored in btManifoldResult
 * Perturbation is done in btConvexConvexAlgorithm
*/

/* GJK   */
#define GJK_MAX_ITERATIONS 128
#define GJK_ACCURARY ((real)0.0000)
#define GJK_MIN_DISTANCE ((real)0.0000001)
#define GJK_DUPLICATED_EPS ((real)0.0000001)
#define GJK_SIMPLEX2_EPS ((real)0.0)
#define GJK_SIMPLEX3_EPS ((real)0.0)
#define GJK_SIMPLEX4_EPS ((real)0.0)

/* EPA   */
#define EPA_MAX_VERTICES 64
#define EPA_MAX_FACES (EPA_MAX_VERTICES * 2)
#define EPA_MAX_ITERATIONS 255
#define EPA_ACCURACY ((real)0.0000)
#define EPA_FALLBACK (10 * EPA_ACCURACY)
#define EPA_PLANE_EPS ((real)0.0000001)
#define EPA_INSIDE_EPS ((real)0.00001)

// must be above the machine epsilon
#define REL_ERROR2 real(1.0e-6)

typedef unsigned int U;
typedef unsigned char U1;

template <class T>
inline void PlaneSpace1(const T& n, T& p, T& q) {
  if (fabs(n.z) > CH_C_SQRT_1_2) {
    // choose p in y-z plane
    real a = n.y * n.y + n.z * n.z;
    real k = 1.0 / sqrt(a);
    p.x = 0;
    p.y = -n.z * k;
    p.z = n.y * k;
    // set q = n x p
    q.x = a * k;
    q.y = -n.x * p.z;
    q.z = n.x * p.y;
  } else {
    // choose p in x-y plane
    real a = n.x * n.x + n.y * n.y;
    real k = 1.0 / sqrt(a);
    p.x = -n.y * k;
    p.y = n.x * k;
    p.z = 0;
    // set q = n x p
    q.x = -n.z * p.y;
    q.y = n.z * p.x;
    q.z = a * k;
  }
}

// MinkowskiDiff
struct MinkowskiDiff {
  ConvexShape shapeA;
  ConvexShape shapeB;
  real envelope;

  // const btConvexShape* m_shapes[2];
  // btMatrix3x3 m_toshape1;
  // btTransform m_toshape0;

  // real3 (btConvexShape::*Ls)(const real3&) const;

  MinkowskiDiff() {}

  inline real3 Support0(const real3& d) const { return SupportVert(shapeA, d, envelope); }
  inline real3 Support1(const real3& d) const {
    real3 m_toshape0_v = (shapeB.A - shapeA.A);
    real3 m_toshape0_translate = quatRotateT(m_toshape0_v, shapeA.R);

    real4 m_toshape0 = (~shapeA.R) % shapeB.R;
    real4 m_toshape1 = (~shapeB.R) % shapeA.R;

    real3 sv = SupportVert(shapeB, quatRotate(d, m_toshape1), envelope);
    real3 result = TransformLocalToParent(m_toshape0_translate, m_toshape0, sv);

    return result;  // SupportVert(shapeB, d, 0) + shapeB.margin * d;
  }
  inline real3 Support(const real3& d) const { return (Support0(d) - Support1(-d)); }
  real3 Support(const real3& d, U index) const {
    if (index) {
      return (Support1(d));
    } else {
      return (Support0(d));
    }
  }
};

typedef MinkowskiDiff tShape;

// GJK
struct GJK {
  /* Types    */
  struct sSV {
    real3 d, w;
  };
  struct sSimplex {
    sSV* c[4];
    real p[4];
    U rank;
  };
  struct eStatus {
    enum _ { Valid, Inside, Failed };
  };
  /* Fields      */
  tShape m_shape;
  real3 m_ray;
  real m_distance;
  sSimplex m_simplices[2];
  sSV m_store[4];
  sSV* m_free[4];
  U m_nfree;
  U m_current;
  sSimplex* m_simplex;
  eStatus::_ m_status;
  /* Methods     */
  GJK() { Initialize(); }
  void Initialize() {
    m_ray = real3(0, 0, 0);
    m_nfree = 0;
    m_status = eStatus::Failed;
    m_current = 0;
    m_distance = 0;
  }
  eStatus::_ Evaluate(const tShape& shapearg, const real3& guess) {
    U iterations = 0;
    real sqdist = 0;
    real alpha = 0;
    real3 lastw[4];
    U clastw = 0;
    /* Initialize solver    */
    m_free[0] = &m_store[0];
    m_free[1] = &m_store[1];
    m_free[2] = &m_store[2];
    m_free[3] = &m_store[3];
    m_nfree = 4;
    m_current = 0;
    m_status = eStatus::Valid;
    m_shape = shapearg;
    m_distance = 0;
    /* Initialize simplex      */
    m_simplices[0].rank = 0;
    m_ray = guess;
    const real sqrl = m_ray.length2();
    appendvertice(m_simplices[0], sqrl > 0 ? -m_ray : real3(1, 0, 0));
    m_simplices[0].p[0] = 1;
    m_ray = m_simplices[0].c[0]->w;
    sqdist = sqrl;
    lastw[0] = lastw[1] = lastw[2] = lastw[3] = m_ray;
    /* Loop                 */
    do {
      const U next = 1 - m_current;
      sSimplex& cs = m_simplices[m_current];
      sSimplex& ns = m_simplices[next];
      /* Check zero                    */
      const real rl = m_ray.length();
      if (rl < GJK_MIN_DISTANCE) { /* Touching or inside           */
        m_status = eStatus::Inside;
        break;
      }
      /* Append new vertice in -'v' direction   */
      appendvertice(cs, -m_ray);
      const real3& w = cs.c[cs.rank - 1]->w;
      bool found = false;
      for (U i = 0; i < 4; ++i) {
        if ((w - lastw[i]).length2() < GJK_DUPLICATED_EPS) {
          found = true;
          break;
        }
      }
      if (found) { /* Return old simplex            */
        removevertice(m_simplices[m_current]);
        break;
      } else { /* Update lastw             */
        lastw[clastw = (clastw + 1) & 3] = w;
      }
      /* Check for termination            */
      const real omega = dot(m_ray, w) / rl;
      alpha = std::max(omega, alpha);
      if (((rl - alpha) - (GJK_ACCURARY * rl)) <= 0) { /* Return old simplex            */
        removevertice(m_simplices[m_current]);
        break;
      }
      /* Reduce simplex                */
      real weights[4];
      U mask = 0;
      switch (cs.rank) {
        case 2:
          sqdist = projectorigin(cs.c[0]->w, cs.c[1]->w, weights, mask);
          break;
        case 3:
          sqdist = projectorigin(cs.c[0]->w, cs.c[1]->w, cs.c[2]->w, weights, mask);
          break;
        case 4:
          sqdist = projectorigin(cs.c[0]->w, cs.c[1]->w, cs.c[2]->w, cs.c[3]->w, weights, mask);
          break;
      }
      if (sqdist >= 0) { /* Valid */
        ns.rank = 0;
        m_ray = real3(0, 0, 0);
        m_current = next;
        for (U i = 0, ni = cs.rank; i < ni; ++i) {
          if (mask & (1 << i)) {
            ns.c[ns.rank] = cs.c[i];
            ns.p[ns.rank++] = weights[i];
            m_ray += cs.c[i]->w * weights[i];
          } else {
            m_free[m_nfree++] = cs.c[i];
          }
        }
        if (mask == 15)
          m_status = eStatus::Inside;
      } else { /* Return old simplex          */
        removevertice(m_simplices[m_current]);
        break;
      }
      m_status = ((++iterations) < GJK_MAX_ITERATIONS) ? m_status : eStatus::Failed;
    } while (m_status == eStatus::Valid);
    m_simplex = &m_simplices[m_current];
    switch (m_status) {
      case eStatus::Valid:
        m_distance = m_ray.length();
        break;
      case eStatus::Inside:
        m_distance = 0;
        break;
      default: {}
    }
    return (m_status);
  }
  bool EncloseOrigin() {
    switch (m_simplex->rank) {
      case 1: {
        for (U i = 0; i < 3; ++i) {
          real3 axis = real3(0, 0, 0);
          axis[i] = 1;
          appendvertice(*m_simplex, axis);
          if (EncloseOrigin())
            return (true);
          removevertice(*m_simplex);
          appendvertice(*m_simplex, -axis);
          if (EncloseOrigin())
            return (true);
          removevertice(*m_simplex);
        }
      } break;
      case 2: {
        const real3 d = m_simplex->c[1]->w - m_simplex->c[0]->w;
        for (U i = 0; i < 3; ++i) {
          real3 axis = real3(0, 0, 0);
          axis[i] = 1;
          const real3 p = cross(d, axis);
          if (p.length2() > 0) {
            appendvertice(*m_simplex, p);
            if (EncloseOrigin())
              return (true);
            removevertice(*m_simplex);
            appendvertice(*m_simplex, -p);
            if (EncloseOrigin())
              return (true);
            removevertice(*m_simplex);
          }
        }
      } break;
      case 3: {
        const real3 n = cross(m_simplex->c[1]->w - m_simplex->c[0]->w, m_simplex->c[2]->w - m_simplex->c[0]->w);
        if (n.length2() > 0) {
          appendvertice(*m_simplex, n);
          if (EncloseOrigin())
            return (true);
          removevertice(*m_simplex);
          appendvertice(*m_simplex, -n);
          if (EncloseOrigin())
            return (true);
          removevertice(*m_simplex);
        }
      } break;
      case 4: {
        if (fabs(det(m_simplex->c[0]->w - m_simplex->c[3]->w, m_simplex->c[1]->w - m_simplex->c[3]->w,
                     m_simplex->c[2]->w - m_simplex->c[3]->w)) > 0)
          return (true);
      } break;
    }
    return (false);
  }
  /* Internals   */
  void getsupport(const real3& d, sSV& sv) const {
    sv.d = d / d.length();
    sv.w = m_shape.Support(sv.d);
  }
  void removevertice(sSimplex& simplex) { m_free[m_nfree++] = simplex.c[--simplex.rank]; }
  void appendvertice(sSimplex& simplex, const real3& v) {
    simplex.p[simplex.rank] = 0;
    simplex.c[simplex.rank] = m_free[--m_nfree];
    getsupport(v, *simplex.c[simplex.rank++]);
  }
  static real det(const real3& a, const real3& b, const real3& c) {
    return (a.y * b.z * c.x + a.z * b.x * c.y - a.x * b.z * c.y - a.y * b.x * c.z + a.x * b.y * c.z - a.z * b.y * c.x);
  }
  static real projectorigin(const real3& a, const real3& b, real* w, U& m) {
    const real3 d = b - a;
    const real l = d.length2();
    if (l > GJK_SIMPLEX2_EPS) {
      const real t(l > 0 ? -dot(a, d) / l : 0);
      if (t >= 1) {
        w[0] = 0;
        w[1] = 1;
        m = 2;
        return (b.length2());
      } else if (t <= 0) {
        w[0] = 1;
        w[1] = 0;
        m = 1;
        return (a.length2());
      } else {
        w[0] = 1 - (w[1] = t);
        m = 3;
        return ((a + d * t).length2());
      }
    }
    return (-1);
  }
  static real projectorigin(const real3& a, const real3& b, const real3& c, real* w, U& m) {
    static const U imd3[] = {1, 2, 0};
    const real3* vt[] = {&a, &b, &c};
    const real3 dl[] = {a - b, b - c, c - a};
    const real3 n = cross(dl[0], dl[1]);
    const real l = n.length2();
    if (l > GJK_SIMPLEX3_EPS) {
      real mindist = -1;
      real subw[2] = {0.f, 0.f};
      U subm(0);
      for (U i = 0; i < 3; ++i) {
        if (dot(*vt[i], cross(dl[i], n)) > 0) {
          const U j = imd3[i];
          const real subd(projectorigin(*vt[i], *vt[j], subw, subm));
          if ((mindist < 0) || (subd < mindist)) {
            mindist = subd;
            m = static_cast<U>(((subm & 1) ? 1 << i : 0) + ((subm & 2) ? 1 << j : 0));
            w[i] = subw[0];
            w[j] = subw[1];
            w[imd3[j]] = 0;
          }
        }
      }
      if (mindist < 0) {
        const real d = dot(a, n);
        const real s = sqrt(l);
        const real3 p = n * (d / l);
        mindist = p.length2();
        m = 7;
        w[0] = (cross(dl[1], b - p)).length() / s;
        w[1] = (cross(dl[2], c - p)).length() / s;
        w[2] = 1 - (w[0] + w[1]);
      }
      return (mindist);
    }
    return (-1);
  }
  static real projectorigin(const real3& a, const real3& b, const real3& c, const real3& d, real* w, U& m) {
    static const U imd3[] = {1, 2, 0};
    const real3* vt[] = {&a, &b, &c, &d};
    const real3 dl[] = {a - d, b - d, c - d};
    const real vl = det(dl[0], dl[1], dl[2]);
    const bool ng = (vl * dot(a, cross(b - c, a - b))) <= 0;
    if (ng && (fabs(vl) > GJK_SIMPLEX4_EPS)) {
      real mindist = -1;
      real subw[3] = {0.f, 0.f, 0.f};
      U subm(0);
      for (U i = 0; i < 3; ++i) {
        const U j = imd3[i];
        const real s = vl * dot(d, cross(dl[i], dl[j]));
        if (s > 0) {
          const real subd = projectorigin(*vt[i], *vt[j], d, subw, subm);
          if ((mindist < 0) || (subd < mindist)) {
            mindist = subd;
            m = static_cast<U>((subm & 1 ? 1 << i : 0) + (subm & 2 ? 1 << j : 0) + (subm & 4 ? 8 : 0));
            w[i] = subw[0];
            w[j] = subw[1];
            w[imd3[j]] = 0;
            w[3] = subw[2];
          }
        }
      }
      if (mindist < 0) {
        mindist = 0;
        m = 15;
        w[0] = det(c, b, d) / vl;
        w[1] = det(a, c, d) / vl;
        w[2] = det(b, a, d) / vl;
        w[3] = 1 - (w[0] + w[1] + w[2]);
      }
      return (mindist);
    }
    return (-1);
  }
};

// EPA
struct EPA {
  /* Types    */
  typedef GJK::sSV sSV;
  struct sFace {
    real3 n;
    real d;
    sSV* c[3];
    sFace* f[3];
    sFace* l[2];
    U1 e[3];
    U1 pass;
  };
  struct sList {
    sFace* root;
    U count;
    sList() : root(0), count(0) {}
  };
  struct sHorizon {
    sFace* cf;
    sFace* ff;
    U nf;
    sHorizon() : cf(0), ff(0), nf(0) {}
  };
  struct eStatus {
    enum _ {
      Valid,
      Touching,
      Degenerated,
      NonConvex,
      InvalidHull,
      OutOfFaces,
      OutOfVertices,
      AccuraryReached,
      FallBack,
      Failed
    };
  };
  /* Fields      */
  eStatus::_ m_status;
  GJK::sSimplex m_result;
  real3 m_normal;
  real m_depth;
  sSV m_sv_store[EPA_MAX_VERTICES];
  sFace m_fc_store[EPA_MAX_FACES];
  U m_nextsv;
  sList m_hull;
  sList m_stock;
  /* Methods     */
  EPA() { Initialize(); }

  static inline void bind(sFace* fa, U ea, sFace* fb, U eb) {
    fa->e[ea] = (U1)eb;
    fa->f[ea] = fb;
    fb->e[eb] = (U1)ea;
    fb->f[eb] = fa;
  }
  static inline void append(sList& list, sFace* face) {
    face->l[0] = 0;
    face->l[1] = list.root;
    if (list.root)
      list.root->l[0] = face;
    list.root = face;
    ++list.count;
  }
  static inline void remove(sList& list, sFace* face) {
    if (face->l[1])
      face->l[1]->l[0] = face->l[0];
    if (face->l[0])
      face->l[0]->l[1] = face->l[1];
    if (face == list.root)
      list.root = face->l[1];
    --list.count;
  }

  void Initialize() {
    m_status = eStatus::Failed;
    m_normal = real3(0, 0, 0);
    m_depth = 0;
    m_nextsv = 0;
    for (U i = 0; i < EPA_MAX_FACES; ++i) {
      append(m_stock, &m_fc_store[EPA_MAX_FACES - i - 1]);
    }
  }
  eStatus::_ Evaluate(GJK& gjk, const real3& guess) {
    GJK::sSimplex& simplex = *gjk.m_simplex;
    if ((simplex.rank > 1) && gjk.EncloseOrigin()) {
      /* Clean up          */
      while (m_hull.root) {
        sFace* f = m_hull.root;
        remove(m_hull, f);
        append(m_stock, f);
      }
      m_status = eStatus::Valid;
      m_nextsv = 0;
      /* Orient simplex    */
      if (gjk.det(simplex.c[0]->w - simplex.c[3]->w, simplex.c[1]->w - simplex.c[3]->w,
                  simplex.c[2]->w - simplex.c[3]->w) < 0) {
        Swap(simplex.c[0], simplex.c[1]);
        Swap(simplex.p[0], simplex.p[1]);
      }
      /* Build initial hull   */
      sFace* tetra[] = {newface(simplex.c[0], simplex.c[1], simplex.c[2], true),
                        newface(simplex.c[1], simplex.c[0], simplex.c[3], true),
                        newface(simplex.c[2], simplex.c[1], simplex.c[3], true),
                        newface(simplex.c[0], simplex.c[2], simplex.c[3], true)};
      if (m_hull.count == 4) {
        sFace* best = findbest();
        sFace outer = *best;
        U pass = 0;
        U iterations = 0;
        bind(tetra[0], 0, tetra[1], 0);
        bind(tetra[0], 1, tetra[2], 0);
        bind(tetra[0], 2, tetra[3], 0);
        bind(tetra[1], 1, tetra[3], 2);
        bind(tetra[1], 2, tetra[2], 1);
        bind(tetra[2], 2, tetra[3], 1);
        m_status = eStatus::Valid;
        for (; iterations < EPA_MAX_ITERATIONS; ++iterations) {
          if (m_nextsv < EPA_MAX_VERTICES) {
            sHorizon horizon;
            sSV* w = &m_sv_store[m_nextsv++];
            bool valid = true;
            best->pass = (U1)(++pass);
            gjk.getsupport(best->n, *w);
            const real wdist = dot(best->n, w->w) - best->d;
            if (wdist > EPA_ACCURACY) {
              for (U j = 0; (j < 3) && valid; ++j) {
                valid &= expand(pass, w, best->f[j], best->e[j], horizon);
              }
              if (valid && (horizon.nf >= 3)) {
                bind(horizon.cf, 1, horizon.ff, 2);
                remove(m_hull, best);
                append(m_stock, best);
                best = findbest();
                outer = *best;
              } else {
                m_status = eStatus::InvalidHull;
                break;
              }
            } else {
              m_status = eStatus::AccuraryReached;
              break;
            }
          } else {
            m_status = eStatus::OutOfVertices;
            break;
          }
        }
        const real3 projection = outer.n * outer.d;
        m_normal = outer.n;
        m_depth = outer.d;
        m_result.rank = 3;
        m_result.c[0] = outer.c[0];
        m_result.c[1] = outer.c[1];
        m_result.c[2] = outer.c[2];
        m_result.p[0] = cross(outer.c[1]->w - projection, outer.c[2]->w - projection).length();
        m_result.p[1] = cross(outer.c[2]->w - projection, outer.c[0]->w - projection).length();
        m_result.p[2] = cross(outer.c[0]->w - projection, outer.c[1]->w - projection).length();
        const real sum = m_result.p[0] + m_result.p[1] + m_result.p[2];
        m_result.p[0] /= sum;
        m_result.p[1] /= sum;
        m_result.p[2] /= sum;
        return (m_status);
      }
    }
    /* Fallback    */
    m_status = eStatus::FallBack;
    m_normal = -guess;
    const real nl = m_normal.length();
    if (nl > 0)
      m_normal = m_normal / nl;
    else
      m_normal = real3(1, 0, 0);
    m_depth = 0;
    m_result.rank = 1;
    m_result.c[0] = simplex.c[0];
    m_result.p[0] = 1;
    return (m_status);
  }
  bool getedgedist(sFace* face, sSV* a, sSV* b, real& dist) {
    const real3 ba = b->w - a->w;
    const real3 n_ab = cross(ba, face->n);  // Outward facing edge normal direction, on triangle plane
    const real a_dot_nab =
        dot(a->w, n_ab);  // Only care about the sign to determine inside/outside, so not normalization required

    if (a_dot_nab < 0) {
      // Outside of edge a->b

      const real ba_l2 = ba.length2();
      const real a_dot_ba = dot(a->w, ba);
      const real b_dot_ba = dot(b->w, ba);

      if (a_dot_ba > 0) {
        // Pick distance vertex a
        dist = a->w.length();
      } else if (b_dot_ba < 0) {
        // Pick distance vertex b
        dist = b->w.length();
      } else {
        // Pick distance to edge a->b
        const real a_dot_b = dot(a->w, b->w);
        dist = std::sqrt(std::max((a->w.length2() * b->w.length2() - a_dot_b * a_dot_b) / ba_l2, (real)0));
      }

      return true;
    }

    return false;
  }
  sFace* newface(sSV* a, sSV* b, sSV* c, bool forced) {
    if (m_stock.root) {
      sFace* face = m_stock.root;
      remove(m_stock, face);
      append(m_hull, face);
      face->pass = 0;
      face->c[0] = a;
      face->c[1] = b;
      face->c[2] = c;
      face->n = cross(b->w - a->w, c->w - a->w);
      const real l = face->n.length();
      const bool v = l > EPA_ACCURACY;

      if (v) {
        if (!(getedgedist(face, a, b, face->d) || getedgedist(face, b, c, face->d) ||
              getedgedist(face, c, a, face->d))) {
          // Origin projects to the interior of the triangle
          // Use distance to triangle plane
          face->d = dot(a->w, face->n) / l;
        }

        face->n /= l;
        if (forced || (face->d >= -EPA_PLANE_EPS)) {
          return face;
        } else
          m_status = eStatus::NonConvex;
      } else
        m_status = eStatus::Degenerated;

      remove(m_hull, face);
      append(m_stock, face);
      return 0;
    }
    m_status = m_stock.root ? eStatus::OutOfVertices : eStatus::OutOfFaces;
    return 0;
  }
  sFace* findbest() {
    sFace* minf = m_hull.root;
    real mind = minf->d * minf->d;
    for (sFace* f = minf->l[1]; f; f = f->l[1]) {
      const real sqd = f->d * f->d;
      if (sqd < mind) {
        minf = f;
        mind = sqd;
      }
    }
    return (minf);
  }
  bool expand(U pass, sSV* w, sFace* f, U e, sHorizon& horizon) {
    static const U i1m3[] = {1, 2, 0};
    static const U i2m3[] = {2, 0, 1};
    if (f->pass != pass) {
      const U e1 = i1m3[e];
      if ((dot(f->n, w->w) - f->d) < -EPA_PLANE_EPS) {
        sFace* nf = newface(f->c[e1], f->c[e], w, false);
        if (nf) {
          bind(nf, 0, f, e);
          if (horizon.cf)
            bind(horizon.cf, 1, nf, 2);
          else
            horizon.ff = nf;
          horizon.cf = nf;
          ++horizon.nf;
          return (true);
        }
      } else {
        const U e2 = i2m3[e];
        f->pass = (U1)pass;
        if (expand(pass, w, f->f[e1], f->e[e1], horizon) && expand(pass, w, f->f[e2], f->e[e2], horizon)) {
          remove(m_hull, f);
          append(m_stock, f);
          return (true);
        }
      }
    }
    return (false);
  }
};

static void Initialize(const ConvexShape& shape0, const ConvexShape& shape1, const real & envelope, sResults& results, tShape& shape) {
  /* Results     */
  results.witnesses[0] = results.witnesses[1] = real3(0, 0, 0);
  results.status = sResults::Separated;
  /* Shape    */
  shape.shapeA = shape0;
  shape.shapeB = shape1;
  shape.envelope = envelope;
}

bool GJKDistance(const ConvexShape& shape0, const ConvexShape& shape1, const real3& guess, const real & envelope, sResults& results) {
  tShape shape;
  Initialize(shape0, shape1, envelope, results, shape);
  GJK gjk;
  GJK::eStatus::_ gjk_status = gjk.Evaluate(shape, guess);
  if (gjk_status == GJK::eStatus::Valid) {
    real3 w0 = real3(0, 0, 0);
    real3 w1 = real3(0, 0, 0);
    for (U i = 0; i < gjk.m_simplex->rank; ++i) {
      const real p = gjk.m_simplex->p[i];
      w0 += shape.Support(gjk.m_simplex->c[i]->d, 0) * p;
      w1 += shape.Support(-gjk.m_simplex->c[i]->d, 1) * p;
    }
    results.witnesses[0] = TransformLocalToParent(shape0.A, shape0.R, w0);
    results.witnesses[1] = TransformLocalToParent(shape0.A, shape0.R, w1);
    results.normal = w1 - w0;
    results.distance = -results.normal.length();
    results.normal /= results.distance > GJK_MIN_DISTANCE ? results.distance : 1;
    return (true);
  } else {
    results.status = gjk_status == GJK::eStatus::Inside ? sResults::Penetrating : sResults::GJK_Failed;
    return (false);
  }
}

bool GJKPenetration(const ConvexShape& shape0, const ConvexShape& shape1, const real3& guess, const real & envelope, sResults& results) {
  tShape shape;
  Initialize(shape0, shape1, envelope, results, shape);
  GJK gjk;
  GJK::eStatus::_ gjk_status = gjk.Evaluate(shape, -guess);
  switch (gjk_status) {
    case GJK::eStatus::Inside: {
      EPA epa;
      EPA::eStatus::_ epa_status = epa.Evaluate(gjk, -guess);
      if (epa_status != EPA::eStatus::Failed) {
        real3 w0 = real3(0, 0, 0);
        for (U i = 0; i < epa.m_result.rank; ++i) {
          w0 += shape.Support(epa.m_result.c[i]->d, 0) * epa.m_result.p[i];
        }
        results.status = sResults::Penetrating;
        results.witnesses[0] = TransformLocalToParent(shape0.A, shape0.R, w0);
        results.witnesses[1] = TransformLocalToParent(shape0.A, shape0.R, (w0 - epa.m_normal * epa.m_depth));
        results.normal = -epa.m_normal;
        results.distance = -epa.m_depth;
        return (true);
      } else
        results.status = sResults::EPA_Failed;
    } break;
    case GJK::eStatus::Failed:
      results.status = sResults::GJK_Failed;
      break;
    default: {}
  }
  return (false);
}
bool GJKFindPenetration(const ConvexShape& shape0, const ConvexShape& shape1, const real & envelope,  sResults& results) {
  real3 guess = shape0.A - shape1.A;

  if (GJKPenetration(shape0, shape1, guess, envelope, results)) {
    return true;
  } else {
    if (GJKDistance(shape0, shape1, guess, envelope, results)) {
      return false;
    }
  }
  return false;
}


bool GJKCollide(const ConvexShape& shape0,
                const ConvexShape& shape1,
                const real & envelope,
                ContactPoint& contact_point,
                real3& m_cachedSeparatingAxis) {
  sResults results;
  real m_cachedSeparatingDistance;
  int gNumDeepPenetrationChecks = 0;
  int gNumGjkChecks = 0;
  int m_curIter = 0;
  int m_degenerateSimplex = 0;
  int m_lastUsedMethod = 0;

  bool m_catchDegeneracies = true;

  ChCNarrowphaseVoronoiSimplex* m_simplexSolver = new ChCNarrowphaseVoronoiSimplex();
  m_cachedSeparatingDistance = 0.0;
  real distance = 0.0;
  real3 normalInB = real3(0.0, 0.0, 0.0);
  real3 pointOnA, pointOnB;

  ConvexShape shapeA = shape0;
  ConvexShape shapeB = shape1;

  real3 positionOffset = (shapeA.A + shapeB.A) * real(0.5);
  shapeA.A = shapeA.A - positionOffset;
  shapeB.A = shapeB.A - positionOffset;

  real marginA = shapeA.margin;
  real marginB = shapeB.margin;

  m_curIter = 0;
  int gGjkMaxIter = 1000;
  m_cachedSeparatingAxis = real3(0, 1, 0);

  bool isValid = false;
  bool checkSimplex = false;
  bool checkPenetration = true;
  m_degenerateSimplex = 0;

  m_lastUsedMethod = -1;

  {
    real squaredDistance = LARGE_REAL;
    real delta = 0.0;
    real margin = marginA + marginB;
    m_simplexSolver->reset();
    for (;;) {
      real3 seperatingAxisInA = quatRotateT(-m_cachedSeparatingAxis, shapeA.R);
      real3 seperatingAxisInB = quatRotateT(m_cachedSeparatingAxis, shapeB.R);

      real3 pInA = SupportVertNoMargin(shapeA, seperatingAxisInA, 0);
      real3 qInB = SupportVertNoMargin(shapeB, seperatingAxisInB, 0);

      real3 pWorld = TransformLocalToParent(shapeA.A, shapeA.R, pInA);
      real3 qWorld = TransformLocalToParent(shapeB.A, shapeB.R, qInB);

      //      printf("seperatingAxisInA: [%f %f %f], seperatingAxisInB:  [%f %f %f]\n", seperatingAxisInA.x,
      //      seperatingAxisInA.y, seperatingAxisInA.z, seperatingAxisInB.x, seperatingAxisInB.y,
      //             seperatingAxisInB.z);
      //      printf("pInA: [%f %f %f], qInB:  [%f %f %f]\n", pInA.x, pInA.y, pInA.z, qInB.x, qInB.y, qInB.z);
      //      printf("pWorld: [%f %f %f], qWorld:  [%f %f %f]\n", pWorld.x, pWorld.y, pWorld.z, qWorld.x, qWorld.y,
      //      qWorld.z);

      real3 w = pWorld - qWorld;
      delta = m_cachedSeparatingAxis.dot(w);

      // potential exit, they don't overlap
      if ((delta > real(0.0)) && (delta * delta > squaredDistance * LARGE_REAL)) {
        m_degenerateSimplex = 10;
        checkSimplex = true;
        // checkPenetration = false;
        break;
      }

      // exit 0: the new point is already in the simplex, or we didn't come any closer
      if (m_simplexSolver->inSimplex(w)) {
        m_degenerateSimplex = 1;
        checkSimplex = true;
        break;
      }

      // are we getting any closer ?
      real f0 = squaredDistance - delta;
      real f1 = squaredDistance * REL_ERROR2;

      if (f0 <= f1) {
        if (f0 <= real(0.)) {
          m_degenerateSimplex = 2;
        } else {
          m_degenerateSimplex = 11;
        }
        checkSimplex = true;
        break;
      }

      // add current vertex to simplex
      m_simplexSolver->addVertex(w, pWorld, qWorld);
      real3 newCachedSeparatingAxis;

      // calculate the closest point to the origin (update vector v)
      if (!m_simplexSolver->closest(newCachedSeparatingAxis)) {
        m_degenerateSimplex = 3;
        checkSimplex = true;
        break;
      }

      if (newCachedSeparatingAxis.length2() < REL_ERROR2) {
        m_cachedSeparatingAxis = newCachedSeparatingAxis;
        m_degenerateSimplex = 6;
        checkSimplex = true;
        break;
      }

      real previousSquaredDistance = squaredDistance;
      squaredDistance = newCachedSeparatingAxis.length2();
      m_cachedSeparatingAxis = newCachedSeparatingAxis;
      // are we getting any closer ?

      if (previousSquaredDistance - squaredDistance <= ZERO_EPSILON * previousSquaredDistance) {
        m_simplexSolver->backup_closest(m_cachedSeparatingAxis);
        checkSimplex = true;
        m_degenerateSimplex = 12;

        break;
      }

      // degeneracy, this is typically due to invalid/uninitialized worldtransforms for a btCollisionObject
      if (m_curIter++ > gGjkMaxIter) {
        break;
      }

      bool check = (!m_simplexSolver->fullSimplex());

      if (!check) {
        // do we need this backup_closest here ?
        m_simplexSolver->backup_closest(m_cachedSeparatingAxis);
        m_degenerateSimplex = 13;
        break;
      }
    }

    if (checkSimplex) {
      m_simplexSolver->compute_points(pointOnA, pointOnB);
      normalInB = pointOnA - pointOnB;
      real lenSqr = m_cachedSeparatingAxis.length2();

      // valid normal
      if (lenSqr < 0.0001) {
        m_degenerateSimplex = 5;
      }
      if (lenSqr > ZERO_EPSILON * ZERO_EPSILON) {
        real rlen = real(1.) / sqrt(lenSqr);
        normalInB *= rlen;  // normalize
        real s = sqrt(squaredDistance);

        pointOnA -= m_cachedSeparatingAxis * (marginA / s);
        pointOnB += m_cachedSeparatingAxis * (marginB / s);
        distance = ((real(1.) / rlen) - margin);

        // distance = dot(m_cachedSeparatingAxis, pointOnB - pointOnA);

        isValid = true;

        m_lastUsedMethod = 1;
      } else {
        m_lastUsedMethod = 2;
      }
    }

    bool catchDegeneratePenetrationCase = (m_catchDegeneracies && m_degenerateSimplex && ((distance + margin) < 0.01));

    // if (checkPenetration && !isValid)
    if (checkPenetration && (!isValid || catchDegeneratePenetrationCase)) {
      // penetration case
      // Penetration depth case.

      gNumDeepPenetrationChecks++;
      m_cachedSeparatingAxis = real3(0);

      bool isValid2 = GJKFindPenetration(shapeA, shapeB, envelope, results);

      real3 tmpPointOnA = results.witnesses[0];
      real3 tmpPointOnB = results.witnesses[1];

      if (isValid2) {
        real3 tmpNormalInB = tmpPointOnB - tmpPointOnA;
        real lenSqr = tmpNormalInB.length2();
        if (lenSqr <= (ZERO_EPSILON * ZERO_EPSILON)) {
          tmpNormalInB = m_cachedSeparatingAxis;
          lenSqr = m_cachedSeparatingAxis.length2();
        }

        if (lenSqr > (ZERO_EPSILON * ZERO_EPSILON)) {
          tmpNormalInB /= sqrt(lenSqr);
          real distance2 = -real3(tmpPointOnA - tmpPointOnB).length();
          // only replace valid penetrations when the result is deeper (check)
          if (!isValid || (distance2 < distance)) {
            distance = distance2;
            pointOnA = tmpPointOnA;
            pointOnB = tmpPointOnB;
            normalInB = tmpNormalInB;
            isValid = true;
            m_lastUsedMethod = 3;
          } else {
            m_lastUsedMethod = 8;
          }
        } else {
          m_lastUsedMethod = 9;
        }
      } else {
        /// this is another degenerate case, where the initial GJK calculation reports a degenerate case
        /// EPA reports no penetration, and the second GJK (using the supporting vector without margin)
        /// reports a valid positive distance. Use the results of the second GJK instead of failing.
        /// thanks to Jacob.Langford for the reproduction case
        /// http://code.google.com/p/bullet/issues/detail?id=250

        if (m_cachedSeparatingAxis.length2() > real(0.)) {
          real distance2 = real3(tmpPointOnA - tmpPointOnB).length() - margin;
          // only replace valid distances when the distance is less
          if (!isValid || (distance2 < distance)) {
            distance = distance2;
            pointOnA = tmpPointOnA;
            pointOnB = tmpPointOnB;
            pointOnA -= m_cachedSeparatingAxis * marginA;
            pointOnB += m_cachedSeparatingAxis * marginB;
            normalInB = m_cachedSeparatingAxis;
            normalInB.normalize();
            isValid = true;
            m_lastUsedMethod = 6;
          } else {
            m_lastUsedMethod = 5;
          }
        }
      }
    }
  }

  delete m_simplexSolver;

  // printf("last Method: %d", m_lastUsedMethod);

  if (isValid && ((distance < 0) || (distance * distance < LARGE_REAL)) && distance < (marginA + marginB + envelope) ) {
    m_cachedSeparatingAxis = normalInB;
    m_cachedSeparatingDistance = distance;

    contact_point.depth = distance +  envelope *2;
    contact_point.normal = normalInB;
    contact_point.pointA = real3(pointOnB + positionOffset) + normalInB * (distance +  envelope);
    contact_point.pointB = real3(pointOnB + positionOffset) -  normalInB * envelope;
    //    manifold.addContactPoint(shape0, shape1, normalInB, real3(pointOnB + positionOffset), distance);
  } else {
    return false;
  }
  return true;
}

void GJKPerturbedCollide(const ConvexShape& shapeA,
                         const ConvexShape& shapeB,
                         ContactManifold& manifold,
                         real3 m_cachedSeparatingAxis) {
  int m_numPerturbationIterations = 4;

  if (m_numPerturbationIterations && manifold.num_contact_points < 4) {
    int i;
    real3 v0, v1;
    real3 sepNormalWorldSpace;

    sepNormalWorldSpace = m_cachedSeparatingAxis.normalize();
    PlaneSpace1(sepNormalWorldSpace, v0, v1);

    bool perturbeA = true;
    const real angleLimit = 0.125f * CH_C_PI;
    real perturbeAngle;
    real radiusA = GetAngularMotionDisc(shapeA);
    real radiusB = GetAngularMotionDisc(shapeB);
    if (radiusA < radiusB) {
      perturbeAngle = ZERO_EPSILON / radiusA;
      perturbeA = true;
    } else {
      perturbeAngle = ZERO_EPSILON / radiusB;
      perturbeA = false;
    }
    if (perturbeAngle > angleLimit)
      perturbeAngle = angleLimit;

    //    btTransform unPerturbedTransform;
    //    if (perturbeA) {
    //      unPerturbedTransform = input.m_transformA;
    //    } else {
    //      unPerturbedTransform = input.m_transformB;
    //    }

    ConvexShape pShapeA = shapeA;
    ConvexShape pShapeB = shapeB;

    for (i = 0; i < m_numPerturbationIterations; i++) {
      if (v0.length2() > ZERO_EPSILON) {
        real4 perturbeRot = Q_from_AngAxis(perturbeAngle, v0);
        real iterationAngle = i * (CH_C_PI / real(m_numPerturbationIterations));
        real4 rotq = Q_from_AngAxis(iterationAngle, sepNormalWorldSpace);

        if (perturbeA) {
          pShapeA.R = ((~rotq) % perturbeRot % rotq) % shapeA.R;
          pShapeB.R = shapeB.R;

        } else {
          pShapeA.R = shapeA.R;
          pShapeB.R = ((~rotq) % perturbeRot % rotq) % shapeB.R;
        }

        ContactManifold perturbed_manifold;
        real3 sep_axis = real3(0);
        // GJKCollide(pShapeA, pShapeB, perturbed_manifold, sep_axis);

        for (int i = 0; i < perturbed_manifold.num_contact_points; i++) {
          std::cout << perturbed_manifold.points[i].normal << perturbed_manifold.points[i].pointA
                    << perturbed_manifold.points[i].pointB << perturbed_manifold.points[i].depth << std::endl;
        }
      }
    }
  }
}
}
}
