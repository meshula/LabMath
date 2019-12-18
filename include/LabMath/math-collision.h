// source https://gist.github.com/vurtun/95f088e4889da2474ad1ce82d7911fee
#pragma once

/* ============================================================================
 *
 *                                  GJK
 *
 * =========================================================================== */

struct gjk_support {
    int aid, bid;
    float a[3];
    float b[3];
};
struct gjk_vertex {
    float a[3];
    float b[3];
    float p[3];
    int aid, bid;
};
struct gjk_simplex {
    int hit;
    int iter, max_iter;
    int vcnt, scnt;
    int saveA[5], saveB[5];
    struct gjk_vertex v[5];
    float bc[4], D;
};
struct gjk_result {
    int hit;
    float p0[3];
    float p1[3];
    float distance_squared;
    int iterations;
};

int
gjk(struct gjk_simplex *s, const struct gjk_support *sup, float *dv);

struct gjk_result
gjk_analyze(const struct gjk_simplex *s);

/* ============================================================================
 *
 *                                  COLLISION
 *
 * =========================================================================== */
struct manifold {
    float depth;
    float contact_point[3];
    float normal[3];
};
/* segment */
static float segment_closest_point_to_point_sqdist(const float *a, const float *b, const float *p);
static void segment_closest_point_to_point(float *res, const float *a, const float *b, const float *p);
static float segment_closest_point_to_segment(float *t1, float *t2, float *c1, float *c2, const float *p1, const float *q1, const float *p2, const float *q2);
/* plane */
static void planeq(float *plane4, const float *n3, const float *pnt3);
static void planeqf(float *plane4, float nx, float ny, float nz, float px, float py, float pz);
/* ray */
static float ray_intersects_plane(const float *ro, const float *rd, const float *p4);
static float ray_intersects_triangle(const float *ro, const float *rd, const float *p1, const float *p2, const float *p3);
static int ray_intersects_sphere(float *t0, float *t1, const float *ro, const float *rd, const float *c, float r);
static int ray_intersects_aabb(float *t0, float *t1, const float *ro, const float *rd, const float *min, const float *max);
/* sphere */
static void sphere_closest_point_to_point(float *res, const float *c, const float r, const float *p);
static int sphere_intersects_sphere(const float *ca, float ra, const float *cb, float rb);
static int sphere_intersects_sphere_manifold(struct manifold *m, const float *ca, float ra, const float *cb, float rb);
static int sphere_intersect_aabb(const float *c, float r, const float *min, const float *max);
static int sphere_intersect_aabb_manifold(struct manifold *m, const float *c, float r, const float *min, const float *max);
static int sphere_intersect_capsule(const float *sc, float sr, const float *ca, const float *cb, float cr);
static int sphere_intersect_capsule_manifold(struct manifold *m, const float *sc, float sr, const float *ca, const float *cb, float cr);
static int sphere_intersecting_polyhedron(const float *c, float r, const float *verts, int cnt);
/* aabb */
static void aabb_rebalance_transform(float *bmin, float *bmax, const float *amin, const float *amax, const float *m, const float t[3]);
static void aabb_closest_point_to_point(float *res, const float *min, const float *max, const float *p);
static float aabb_sqdist_to_point(const float *min, const float *max, const float *p);
static int aabb_contains_point(const float *amin, const float *amax, const float *p);
static int aabb_intersect_aabb(const float *amin, const float *amax, const float *bmin, const float *bmax);
static int aabb_intersect_aabb_manifold(struct manifold *m, const float *amin, const float *amax, const float *bmin, const float *bmax);
static int aabb_intersect_sphere(const float *min, const float *max, const float *c, float r);
static int aabb_intersect_sphere_manifold(struct manifold *m, const float *min, const float *max, const float *c, float r);
static int aabb_intersect_capsule(const float *min, const float *max, const float *ca, const float *cb, float cr);
static int aabb_intersect_capsule_manifold(struct manifold *m, const float *min, const float *max, const float *ca, const float *cb, float cr);
static int aabb_intersect_polyhedron(const float *min, const float *max, const float *verts, int cnt);
/* capsule */
static float capsule_point_sqdist(const float *ca, const float *cb, float cr, const float *p);
static void capsule_closest_point_to_point(float *res, const float *ca, const float *cb, float cr, const float *p);
static int capsule_intersect_capsule(const float *aa, const float *ab, float ar, const float *ba, const float *bb, float br);
static int capsule_intersect_capsule_manifold(struct manifold *m, const float *aa, const float *ab, float ar, const float *ba, const float *bb, float br);
static int capsule_intersect_sphere(const float *ca, const float *cb, float cr, const float *sc, float sr);
static int capsule_intersect_sphere_manifold(struct manifold *m, const float *ca, const float *cb, float cr, const float *sc, float sr);
static int capsule_intersect_aabb(const float *ca, const float *cb, float cr, const float *min, const float *max);
static int capsule_intersect_aabb_manifold(struct manifold *m, const float *ca, const float *cb, float cr, const float *min, const float *max);
static int capsule_intersect_polyhedron(const float *ca, const float *cb, float cr, const float *verts, int cnt);
/* polyhedron: query */
static int polyhedron_is_intersecting_sphere(const float *verts, int cnt, const float *c, float r);
static int polyhedron_is_intersecting_aabb(const float *verts, int cnt, const float *min, const float *max);
static int polyhedron_is_intersecting_capsule(const float *verts, int cnt, const float *ca, const float *cb, float cr);
static int polyhedron_is_intersecting_polyhedron(const float *averts, int acnt, const float *bverts, int bcnt);
/* polyhedron: query transformed */
static int polyhedron_is_intersecting_sphere_transform(const float *verts, int cnt, const float *pos3, const float *rot33, const float *sc, float sr);
static int polyhedron_is_intersecting_aabb_transform(const float *verts, int cnt, const float *apos3, const float *arot33, const float *min, const float *max);
static int polyhedron_is_intersecting_capsule_transform(const float *verts, int cnt, const float *pos3, const float *rot33, const float *ca, const float *cb, float cr);
static int polyhedron_is_intersecting_polyhedron_transform(const float *averts, int acnt, const float *apos3, const float *arot33, const float *bverts, int bcnt, const float *bpos3, const float *brot33);
/* polyhedron: gjk result */
static int polyhedron_intersect_sphere(struct gjk_result *res, const float *verts, int cnt, const float *sc, float sr);
static int polyhedron_intersect_aabb(struct gjk_result *res, const float *verts, int cnt, const float *min, const float *max);
static int polyhedron_intersect_capsule(struct gjk_result *res, const float *verts, int cnt, const float *ca, const float *cb, float cr);
static int polyhedron_intersect_polyhedron(struct gjk_result *res, const float *averts, int acnt, const float *bverts, int bcnt);
/* polyhedron: gjk result transformed */
static int polyhedron_intersect_sphere_transform(struct gjk_result *res, const float *verts, int cnt, const float *pos3, const float *rot33, const float *sc, float sr);
static int polyhedron_intersect_aabb_transform(struct gjk_result *res, const float *verts, int cnt, const float *pos3, const float *rot33, const float *min, const float *max);
static int polyhedron_intersect_capsule_transform(struct gjk_result *res, const float *verts, int cnt, const float *pos3, const float *rot33, const float *ca, const float *cb, float cr);
static int polyhedron_intersect_polyhedron_transform(struct gjk_result *res, const float *averts, int acnt, const float *at3, const float *ar33, const float *bverts, int bcnt, const float *bt3, const float *br33);

static void
planeq(float *r, const float *n, const float *p)
{
    f3cpy(r,n);
    r[3] = -f3dot(n,p);
}
static void
planeqf(float *r, float nx, float ny, float nz, float px, float py, float pz)
{
    /* Plane: ax + by + cz + d
     * Equation:
     *      n * (p - p0) = 0
     *      n * p - n * p0 = 0
     *      |a b c| * p - |a b c| * p0
     *
     *      |a b c| * p + d = 0
     *          d = -1 * |a b c| * p0
     *
     *  Plane: |a b c d| d = -|a b c| * p0
     */
    float n[3], p[3];
    f3set(n,nx,ny,nz);
    f3set(p,px,py,pz);
    planeq(r, n, p);
}
static void
segment_closest_point_to_point(float *res,
    const float *a, const float *b, const float *p)
{
    float ab[3], pa[3];
    f3sub(ab, b,a);
    f3sub(pa, p,a);
    float t = f3dot(pa,ab) / f3dot(ab,ab);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    f3mul(res, ab, t);
    f3add(res, a, res);
}
static float
segment_closest_point_to_point_sqdist(const float *a, const float *b, const float *p)
{
    float ab[3], ap[3], bp[3];
    f3sub(ab,a,b);
    f3sub(ap,a,p);
    f3sub(bp,a,p);
    float e = f3dot(ap,ab);

    /* handle cases p proj outside ab */
    if (e <= 0.0f) return f3dot(ap,ap);
    float f = f3dot(ab,ab);
    if (e >= f) return f3dot(bp,bp);
    return f3dot(ap,ap) - (e*e)/f;
}
static float
segment_closest_point_to_segment(float *t1, float *t2, float *c1, float *c2,
    const float *p1, const float *q1, const float *p2, const float *q2)
{
    #define EPSILON (1e-6)
    float r[3], d1[3], d2[3];
    f3sub(d1, q1, p1); /* direction vector segment s1 */
    f3sub(d2, q2, p2); /* direction vector segment s2 */
    f3sub(r, p1, p2);

    float a = f3dot(d1, d1);
    float e = f3dot(d2, d2);
    float f = f3dot(d2, r);

    if (a <= EPSILON && e <= EPSILON) {
        /* both segments degenerate into points */
        float d12[3];
        *t1 = *t2 = 0.0f;
        f3cpy(c1, p1);
        f3cpy(c2, p2);
        f3sub(d12, c1, c2);
        return f3dot(d12,d12);
    }
    if (a > EPSILON) {
        float c = f3dot(d1,r);
        if (e > EPSILON) {
            /* non-degenerate case */
            float b = f3dot(d1,d2);
            float denom = a*e - b*b;

            /* compute closest point on L1/L2 if not parallel else pick any t2 */
            if (denom != 0.0f)
                *t1 = clamp(0.0f, (b*f - c*e) / denom, 1.0f);
            else *t1 = 0.0f;

            /* cmpute point on L2 closest to S1(s) */
            *t2 = (b*(*t1) + f) / e;
            if (*t2 < 0.0f) {
                *t2 = 0.0f;
                *t1 = clamp(0.0f, -c/a, 1.0f);
            } else if (*t2 > 1.0f) {
                *t2 = 1.0f;
                *t1 = clamp(0.0f, (b-c)/a, 1.0f);
            }
        } else {
            /* second segment degenerates into a point */
            *t1 = clamp(0.0f, -c/a, 1.0f);
            *t2 = 0.0f;
        }
    } else {
        /* first segment degenerates into a point */
        *t2 = clamp(0.0f, f / e, 1.0f);
        *t1 = 0.0f;
    }
    /* calculate closest points */
    float n[3], d12[3];
    f3mul(n, d1, *t1);
    f3add(c1, p1, n);
    f3mul(n, d2, *t2);
    f3add(c2, p2, n);

    /* calculate squared distance */
    f3sub(d12, c1, c2);
    return f3dot(d12,d12);
}
static float
ray_intersects_plane(const float *ro, const float *rd,
    const float *plane)
{
    /* Ray: P = origin + rd * t
     * Plane: plane_normal * P + d = 0
     *
     * Substitute:
     *      normal * (origin + rd*t) + d = 0
     *
     * Solve for t:
     *      plane_normal * origin + plane_normal * rd*t + d = 0
     *      -(plane_normal*rd*t) = plane_normal * origin + d
     *
     *                  plane_normal * origin + d
     *      t = -1 * -------------------------
     *                  plane_normal * rd
     *
     * Result:
     *      Behind: t < 0
     *      Infront: t >= 0
     *      Parallel: t = 0
     *      Intersection point: ro + rd * t
     */
    float n = -(f3dot(plane,ro) + plane[3]);
    if (fabs(n) < 0.0001f) return 0.0f;
    return n/(f3dot(plane,rd));
}
static float
ray_intersects_triangle(const float *ro, const float *rd,
    const float *p0, const float *p1, const float *p2)
{
    float p[4];
    float t = 0;
    float di0[3], di1[3], di2[3];
    float d21[3], d02[3], in[3];
    float n[3], d10[3], d20[3];
    float in0[3], in1[3], in2[3];

    /* calculate triangle normal */
    f3sub(d10, p1,p0);
    f3sub(d20, p2,p0);
    f3sub(d21, p2,p1);
    f3sub(d02, p0,p2);
    f3cross(n, d10,d20);

    /* check for plane intersection */
    planeq(p, n, p0);
    t = ray_intersects_plane(ro, rd, p);
    if (t <= 0.0f) return t;

    /* intersection point */
    f3mul(in,rd,t);
    f3add(in,in,ro);

    /* check if point inside triangle in plane */
    f3sub(di0, in, p0);
    f3sub(di1, in, p1);
    f3sub(di2, in, p2);

    f3cross(in0, d10, di0);
    f3cross(in1, d21, di1);
    f3cross(in2, d02, di2);

    if (f3dot(in0,n) < 0.0f)
        return -1;
    if (f3dot(in1,n) < 0.0f)
        return -1;
    if (f3dot(in2,n) < 0.0f)
        return -1;
    return t;
}
static int
ray_intersects_sphere(float *t0, float *t1,
    const float *ro, const float *rd,
    const float *c, float r)
{
    float a[3];
    float tc,td,d2,r2;
    f3sub(a,c,ro);
    tc = f3dot(rd,a);
    if (tc < 0) return 0;

    r2 = r*r;
    d2 = f3dot(a,a) - tc*tc;
    if (d2 > r2) return 0;
    td = sqrtf(r2 - d2);

    *t0 = tc - td;
    *t1 = tc + td;
    return 1;
}
static int
ray_intersects_aabb(float *t0, float *t1,
    const float *ro, const float *rd,
    const float *min, const float *max)
{
    float t0x = (min[0] - ro[0]) / rd[0];
    float t0y = (min[1] - ro[1]) / rd[1];
    float t0z = (min[2] - ro[2]) / rd[2];
    float t1x = (max[0] - ro[0]) / rd[0];
    float t1y = (max[1] - ro[1]) / rd[1];
    float t1z = (max[2] - ro[2]) / rd[2];

    float tminx = min(t0x, t1x);
    float tminy = min(t0y, t1y);
    float tminz = min(t0z, t1z);
    float tmaxx = max(t0x, t1x);
    float tmaxy = max(t0y, t1y);
    float tmaxz = max(t0z, t1z);
    if (tminx > tmaxy || tminy > tmaxx)
        return 0;

    *t0 = max(tminx, tminy);
    *t1 = min(tmaxy, tmaxx);
    if (*t0 > tmaxz || tminz> *t1)
        return 0;

    *t0 = max(*t0, tminz);
    *t1 = min(*t1, tmaxz);
    return 1;
}
static void
sphere_closest_point_to_point(float *res,
    const float *c, const float r,
    const float *p)
{
    float d[3], n[3];
    f3sub(d, p, c);
    f3norm(d);
    f3mul(res,n,r);
    f3add(res,c,res);
}
static int
sphere_intersects_sphere(const float *ca, float ra,
    const float *cb, float rb)
{
    float d[3];
    f3sub(d, cb, ca);
    float r = ra + rb;
    if (f3dot(d,d) > r*r)
        return 0;
    return 1;
}
static int
sphere_intersects_sphere_manifold(struct manifold *m,
    const float *ca, float ra, const float *cb, float rb)
{
    float d[3];
    f3sub(d, cb, ca);
    float r = ra + rb;
    float d2 = f3dot(d,d);
    if (d2 <= r*r) {
        float l = sqrtf(d2);
        float linv = 1.0f / ((l != 0) ? l: 1.0f);
        f3mul(m->normal, d, linv);
        m->depth = r - l;
        f3mul(d, m->normal, rb);
        f3sub(m->contact_point, cb, d);
        return 1;
    } return 0;
}
static int
sphere_intersect_aabb(const float *c, float r,
    const float *min, const float *max)
{
    return aabb_intersect_sphere(min, max, c, r);
}
static int
sphere_intersect_aabb_manifold(struct manifold *m,
    const float *c, float r, const float *min, const float *max)
{
    /* find closest aabb point to sphere center point */
    float ap[3], d[3];
    aabb_closest_point_to_point(ap, min, max, c);
    f3sub(d, c, ap);
    float d2 = f3dot(d, d);
    if (d2 > r*r) return 0;

    /* calculate distance vector between sphere and aabb center points */
    float ac[3];
    f3sub(ac, max, min);
    f3mul(ac, ac, 0.5f);
    f3add(ac, min, ac);
    f3sub(d, ac, c);

    /* normalize distance vector */
    float l2 = f3dot(d,d);
    float l = l2 != 0.0f ? sqrtf(l2): 1.0f;
    float linv = 1.0f/l;
    f3mul(d, d, linv);

    f3cpy(m->normal, d);
    f3mul(m->contact_point, m->normal, r);
    f3add(m->contact_point, c, m->contact_point);

    /* calculate penetration depth */
    float sp[3];
    sphere_closest_point_to_point(sp, c, r, ap);
    f3sub(d, sp, ap);
    m->depth = sqrtf(f3dot(d,d)) - l;
    return 1;
}
static int
sphere_intersect_capsule(const float *sc, float sr,
    const float *ca, const float *cb, float cr)
{
    return capsule_intersect_sphere(ca, cb, cr, sc, sr);
}
static int
sphere_intersect_capsule_manifold(struct manifold *m,
    const float *sc, float sr, const float *ca, const float *cb, float cr)
{
    /* find closest capsule point to sphere center point */
    float cp[3];
    capsule_closest_point_to_point(cp, ca, cb, cr, sc);
    f3sub(m->normal, cp, sc);
    float d2 = f3dot(m->normal, m->normal);
    if (d2 > sr*sr) return 0;

    /* normalize manifold normal vector */
    float l = d2 != 0.0f ? sqrtf(d2): 1;
    float linv = 1.0f/l;
    f3mul(m->normal, m->normal, linv);

    /* calculate penetration depth */
    f3mul(m->contact_point, m->normal, sr);
    f3add(m->contact_point, sc, m->contact_point);
    m->depth = d2 - sr*sr;
    m->depth = m->depth != 0.0f ? sqrtf(m->depth): 0.0f;
    return 1;
}
static int
sphere_intersecting_polyhedron(const float *c, float r,
    const float *verts, int cnt)
{
    return polyhedron_is_intersecting_sphere(verts, cnt, c, r);
}
static void
aabb_rebalance_transform(float *bmin, float *bmax,
    const float *amin, const float *amax,
    const float *m, const float *t)
{
    for (int i = 0; i < 3; ++i) {
        bmin[i] = bmax[i] = t[i];
        for (int j = 0; j < 3; ++j) {
            float e = m[i*3+j] * amin[j];
            float f = m[i*3+j] * amax[j];
            if (e < f) {
                bmin[i] += e;
                bmax[i] += f;
            } else {
                bmin[i] += f;
                bmax[i] += e;
            }
        }
    }
}
static void
aabb_closest_point_to_point(float *res,
    const float *min, const float *max,
    const float *p)
{
    for (int i = 0; i < 3; ++i) {
        float v = p[i];
        if (v < min[i]) v = min[i];
        if (v > max[i]) v = max[i];
        res[i] = v;
    }
}
static float
aabb_sqdist_to_point(const float *min, const float *max, const float *p)
{
    float r = 0;
    for (int i = 0; i < 3; ++i) {
        float v = p[i];
        if (v < min[i]) r += (min[i]-v) * (min[i]-v);
        if (v > max[i]) r += (v-max[i]) * (v-max[i]);
    } return r;
}
static int
aabb_contains_point(const float *amin, const float *amax, const float *p)
{
    if (p[0] < amin[0] || p[0] > amax[0]) return 0;
    if (p[1] < amin[1] || p[1] > amax[1]) return 0;
    if (p[2] < amin[2] || p[2] > amax[2]) return 0;
    return 1;
}
static int
aabb_intersect_aabb(const float *amin, const float *amax,
    const float *bmin, const float *bmax)
{
    if (amax[0] < bmin[0] || amin[0] > bmax[0]) return 0;
    if (amax[1] < bmin[1] || amin[1] > bmax[1]) return 0;
    if (amax[2] < bmin[2] || amin[2] > bmax[2]) return 0;
    return 1;
}
static int
aabb_intersect_aabb_manifold(struct manifold *m,
    const float *amin, const float *amax,
    const float *bmin, const float *bmax)
{
    if (!aabb_intersect_aabb(amin, amax, bmin, bmax))
        return 0;

    /* calculate distance vector between both aabb center points */
    float ac[3], bc[3], d[3];
    f3sub(ac, amax, amin);
    f3sub(bc, bmax, bmin);

    f3mul(ac, ac, 0.5f);
    f3mul(bc, bc, 0.5f);

    f3add(ac, amin, ac);
    f3add(bc, bmin, bc);
    f3sub(d, bc, ac);

    /* normalize distance vector */
    float l2 = f3dot(d,d);
    float l = l2 != 0.0f ? sqrtf(l2): 1.0f;
    float linv = 1.0f/l;
    f3mul(d, d, linv);

    /* calculate contact point */
    f3cpy(m->normal, d);
    aabb_closest_point_to_point(m->contact_point, amin, amax, bc);
    f3sub(d, m->contact_point, ac);

    /* calculate penetration depth */
    float r2 = f3dot(d,d);
    float r = sqrtf(r2);
    m->depth = r - l;
    return 1;
}
static int
aabb_intersect_sphere(const float *min, const float *max,
    const float *c, float r)
{
    /* compute squared distance between sphere center and aabb */
    float d2 = aabb_sqdist_to_point(min, max, c);
    /* intersection if distance is smaller/equal sphere radius*/
    return d2 <= r*r;
}
static int
aabb_intersect_sphere_manifold(struct manifold *m,
    const float *min, const float *max,
    const float *c, float r)
{
    /* find closest aabb point to sphere center point */
    float d[3];
    aabb_closest_point_to_point(m->contact_point, min, max, c);
    f3sub(d, c, m->contact_point);
    float d2 = f3dot(d, d);
    if (d2 > r*r) return 0;

    /* calculate distance vector between aabb and sphere center points */
    float ac[3];
    f3sub(ac, max, min);
    f3mul(ac, ac, 0.5f);
    f3add(ac, min, ac);
    f3sub(d, c, ac);

    /* normalize distance vector */
    float l2 = f3dot(d,d);
    float l = l2 != 0.0f ? sqrtf(l2): 1.0f;
    float linv = 1.0f/l;
    f3mul(d, d, linv);

    /* calculate penetration depth */
    f3cpy(m->normal, d);
    f3sub(d, m->contact_point, ac);
    m->depth = sqrtf(f3dot(d,d));
    return 1;
}
static int
aabb_intersect_capsule(const float *min, const float *max,
    const float *ca, const float *cb, float cr)
{
    return capsule_intersect_aabb(ca, cb, cr, min, max);
}
static int
aabb_intersect_capsule_manifold(struct manifold *m,
    const float *min, const float *max,
    const float *ca, const float *cb, float cr)
{
    /* calculate aabb center point */
    float ac[3];
    f3sub(ac, max, min);
    f3mul(ac, ac, 0.5f);
    f3add(ac, min, ac);

    /* calculate closest point from aabb to point on capsule and check if inside aabb */
    float cp[3];
    capsule_closest_point_to_point(cp, ca, cb, cr, ac);
    if (!aabb_contains_point(min, max, cp))
        return 0;

    /* vector and distance between both capsule closests point and aabb center*/
    float d[3], d2;
    f3sub(d, cp, ac);
    d2 = f3dot(d,d);

    /* calculate penetration depth from closest aabb point to capsule */
    float ap[3], dt[3];
    aabb_closest_point_to_point(ap, min, max, cp);
    f3sub(dt, ap, cp);
    m->depth = sqrtf(f3dot(dt,dt));

    /* calculate normal */
    float l = sqrtf(d2);
    float linv = 1.0f / ((l != 0.0f) ? l: 1.0f);
    f3mul(m->normal, d, linv);
    f3cpy(m->contact_point, ap);
    return 1;
}
static int
aabb_intersect_polyhedron(const float *min, const float *max,
    const float *verts, int cnt)
{
    return polyhedron_is_intersecting_aabb(verts, cnt, min, max);
}
static float
capsule_point_sqdist(const float *ca, const float *cb, float cr, const float *p)
{
    float d2 = segment_closest_point_to_point_sqdist(ca, cb, p);
    return d2 - (cr*cr);
}
static void
capsule_closest_point_to_point(float *res,
    const float *ca, const float *cb, float cr, const float *p)
{
    /* calculate closest point to internal capsule segment */
    float pp[3], d[3];
    segment_closest_point_to_point(pp, ca, cb, p);

    /* extend point out by radius in normal direction */
    f3sub(d,p,pp);
    f3norm(d);
    f3mul(res, d, cr);
    f3add(res, pp, res);
}
static int
capsule_intersect_capsule(const float *aa, const float *ab, float ar,
    const float *ba, const float *bb, float br)
{
    float t1, t2;
    float c1[3], c2[3];
    float d2 = segment_closest_point_to_segment(&t1, &t2, c1, c2, aa, ab, ba, bb);
    float r = ar + br;
    return d2 <= r*r;
}
static int
capsule_intersect_capsule_manifold(struct manifold *m,
    const float *aa, const float *ab, float ar,
    const float *ba, const float *bb, float br)
{
    float t1, t2;
    float c1[3], c2[3];
    float d2 = segment_closest_point_to_segment(&t1, &t2, c1, c2, aa, ab, ba, bb);
    float r = ar + br;
    if (d2 > r*r) return 0;

    /* calculate normal from both closest points for each segement */
    float cp[3], d[3];
    f3sub(m->normal, c2, c1);
    f3norm(m->normal);

    /* calculate contact point from closest point and depth */
    capsule_closest_point_to_point(m->contact_point, aa, ab, ar, c2);
    capsule_closest_point_to_point(cp, ba, bb, br, c1);
    f3sub(d, c1, cp);
    m->depth = sqrtf(f3dot(d,d));
    return 1;
}
static int
capsule_intersect_sphere(const float *ca, const float *cb, float cr,
    const float *sc, float sr)
{
    /* squared distance bwetween sphere center and capsule line segment */
    float d2 = segment_closest_point_to_point_sqdist(ca,cb,sc);
    float r = sr + cr;
    return d2 <= r * r;
}
static int
capsule_intersect_sphere_manifold(struct manifold *m,
    const float *ca, const float *cb, float cr,
    const float *sc, float sr)
{
    /* find closest capsule point to sphere center point */
    capsule_closest_point_to_point(m->contact_point, ca, cb, cr, sc);
    f3sub(m->normal, sc, m->contact_point);
    float d2 = f3dot(m->normal, m->normal);
    if (d2 > sr*sr) return 0;

    /* normalize manifold normal vector */
    float l = d2 != 0.0f ? sqrtf(d2): 1;
    float linv = 1.0f/l;
    f3mul(m->normal, m->normal, linv);

    /* calculate penetration depth */
    m->depth = d2 - sr*sr;
    m->depth = m->depth != 0.0f ? sqrtf(m->depth): 0.0f;
    return 1;
}
static int
capsule_intersect_aabb(const float *ca, const float *cb, float cr,
    const float *min, const float *max)
{
    /* calculate aabb center point */
    float ac[3];
    f3sub(ac, max, min);
    f3mul(ac, ac, 0.5f);

    /* calculate closest point from aabb to point on capsule and check if inside aabb */
    float p[3];
    capsule_closest_point_to_point(p, ca, cb, cr, ac);
    return aabb_contains_point(min, max, p);
}
static int
capsule_intersect_aabb_manifold(struct manifold *m,
    const float *ca, const float *cb, float cr,
    const float *min, const float *max)
{
    /* calculate aabb center point */
    float ac[3];
    f3sub(ac, max, min);
    f3mul(ac, ac, 0.5f);
    f3add(ac, min, ac);

    /* calculate closest point from aabb to point on capsule and check if inside aabb */
    float cp[3];
    capsule_closest_point_to_point(cp, ca, cb, cr, ac);
    if (!aabb_contains_point(min, max, cp))
        return 0;

    /* vector and distance between both capsule closests point and aabb center*/
    float d[3], d2;
    f3sub(d, ac, cp);
    d2 = f3dot(d,d);

    /* calculate penetration depth from closest aabb point to capsule */
    float ap[3], dt[3];
    aabb_closest_point_to_point(ap, min, max, cp);
    f3sub(dt, ap, cp);
    m->depth = sqrtf(f3dot(dt,dt));

    /* calculate normal */
    float l = sqrtf(d2);
    float linv = 1.0f / ((l != 0.0f) ? l: 1.0f);
    f3mul(m->normal, d, linv);
    f3cpy(m->contact_point, cp);
    return 1;
}
static int
capsule_intersect_polyhedron(const float *ca, const float *cb, float cr,
    const float *verts, int cnt)
{
    return polyhedron_is_intersecting_capsule(verts, cnt, ca, cb, cr);
}
static int
line_support(float *support, const float *d,
    const float *a, const float *b)
{
    int i = 0;
    float adot = f3dot(a, d);
    float bdot = f3dot(b, d);
    if (adot < bdot) {
        f3cpy(support, b);
        i = 1;
    } else f3cpy(support, a);
    return i;
}
static int
polyhedron_support(float *support, const float *d,
    const float *verts, int cnt)
{
    int imax = 0;
    float dmax = f3dot(verts, d);
    for (int i = 1; i < cnt; ++i) {
        /* find vertex with max dot product in direction d */
        float dot = f3dot(&verts[i*3], d);
        if (dot < dmax) continue;
        imax = i, dmax = dot;
    } f3cpy(support, &verts[imax*3]);
    return imax;
}
static int
polyhedron_intersect_sphere(struct gjk_result *res,
    const float *verts, int cnt,
    const float *sc, float sr)
{
    /* initial guess */
    float d[3] = {0};
    struct gjk_support s = {0};
    f3cpy(s.a, verts);
    f3cpy(s.b, sc);
    f3sub(d, s.b, s.a);

    /* run gjk algorithm */
    struct gjk_simplex gsx = {0};
    while (gjk(&gsx, &s, d)) {
        float n[3]; f3mul(n, d, -1);
        s.aid = polyhedron_support(s.a, n, verts, cnt);
        f3sub(d, s.b, s.a);
    }
    /* check distance between closest points */
    *res = gjk_analyze(&gsx);
    return res->distance_squared <= sr*sr;
}
static int
polyhedron_intersect_sphere_transform(struct gjk_result *res,
    const float *verts, int cnt, const float *pos3, const float *rot33,
    const float *sc, float sr)
{
    /* initial guess */
    float d[3] = {0};
    struct gjk_support s = {0};
    f3cpy(s.a, verts);
    f3cpy(s.b, sc);
    transformS(s.a, rot33, pos3);
    f3sub(d, s.b, s.a);

    /* run gjk algorithm */
    struct gjk_simplex gsx = {0};
    while (gjk(&gsx, &s, d)) {
        float n[3]; f3mul(n, d, -1);
        float da[3]; transformT(da, n, rot33, pos3);

        s.aid = polyhedron_support(s.a, da, verts, cnt);
        transformS(s.a, rot33, pos3);
        f3sub(d, s.b, s.a);
    }
    /* check distance between closest points */
    *res = gjk_analyze(&gsx);
    return res->distance_squared <= sr*sr;
}
static int
polyhedron_is_intersecting_sphere(const float *verts, int cnt,
    const float *sc, float sr)
{
    struct gjk_result res;
    return polyhedron_intersect_sphere(&res, verts, cnt, sc, sr);
}
static int
polyhedron_is_intersecting_sphere_transform(const float *verts, int cnt, const float *pos3,
    const float *rot33, const float *sc, float sr)
{
    struct gjk_result res;
    return polyhedron_intersect_sphere_transform(&res, verts, cnt, pos3, rot33, sc, sr);
}
static int
polyhedron_intersect_capsule(struct gjk_result *res,
    const float *verts, int cnt,
    const float *ca, const float *cb, float cr)
{
    /* initial guess */
    float d[3] = {0};
    struct gjk_support s = {0};
    f3cpy(s.a, verts);
    f3cpy(s.b, ca);
    f3sub(d, s.b, s.a);

    /* run gjk algorithm */
    struct gjk_simplex gsx = {0};
    while (gjk(&gsx, &s, d)) {
        float n[3]; f3mul(n, d, -1);
        s.aid = polyhedron_support(s.a, n, verts, cnt);
        s.bid = line_support(s.b, d, ca, cb);
        f3sub(d, s.b, s.a);
    }
    /* check distance between closest points */
    assert(gsx.iter < gsx.max_iter);
    *res = gjk_analyze(&gsx);
    return res->distance_squared <= cr*cr;
}
static int
polyhedron_is_intersecting_capsule(const float *verts, int cnt,
    const float *ca, const float *cb, float cr)
{
    struct gjk_result res;
    return polyhedron_intersect_capsule(&res, verts, cnt, ca, cb, cr);
}
static int
polyhedron_intersect_capsule_transform(struct gjk_result *res,
    const float *verts, int cnt, const float *pos3, const float *rot33,
    const float *ca, const float *cb, float cr)
{
    /* initial guess */
    float d[3] = {0};
    struct gjk_support s = {0};
    f3cpy(s.a, verts);
    f3cpy(s.b, ca);
    transformS(s.a, rot33, pos3);
    f3sub(d, s.b, s.a);

    /* run gjk algorithm */
    struct gjk_simplex gsx = {0};
    while (gjk(&gsx, &s, d)) {
        float n[3]; f3mul(n, d, -1);
        float da[3]; transformT(da, n, rot33, pos3);

        s.aid = polyhedron_support(s.a, da, verts, cnt);
        s.bid = line_support(s.b, d, ca, cb);
        transformS(s.a, rot33, pos3);
        f3sub(d, s.b, s.a);
    }
    /* check distance between closest points */
    *res = gjk_analyze(&gsx);
    return res->distance_squared <= cr*cr;
}
static int
polyhedron_is_intersecting_capsule_transform(const float *verts, int cnt,
    const float *pos3, const float *rot33,
    const float *ca, const float *cb, float cr)
{
    struct gjk_result res;
    return polyhedron_intersect_capsule_transform(&res, verts, cnt, pos3, rot33, ca, cb, cr);
}
static int
polyhedron_intersect_polyhedron_transform(struct gjk_result *res,
    const float *averts, int acnt, const float *at3, const float *ar33,
    const float *bverts, int bcnt, const float *bt3, const float *br33)
{
    /* initial guess */
    float d[3] = {0};
    struct gjk_support s = {0};
    f3cpy(s.a, averts);
    f3cpy(s.b, bverts);
    transformS(s.a, ar33, at3);
    transformS(s.b, br33, bt3);
    f3sub(d, s.b, s.a);

    /* run gjk algorithm */
    struct gjk_simplex gsx = {0};
    while (gjk(&gsx, &s, d)) {
        /* transform direction */
        float n[3]; f3mul(n, d, -1);
        float da[3]; transformT(da, n, ar33, at3);
        float db[3]; transformT(db, d, br33, bt3);
        /* run support function on tranformed directions  */
        s.aid = polyhedron_support(s.a, da, averts, acnt);
        s.bid = polyhedron_support(s.b, db, bverts, bcnt);
        /* calculate distance vector on transformed points */
        transformS(s.a, ar33, at3);
        transformS(s.b, br33, bt3);
        f3sub(d, s.b, s.a);
    }
    *res = gjk_analyze(&gsx);
    return gsx.hit;
}
static int
polyhedron_intersect_polyhedron(struct gjk_result *res,
    const float *averts, int acnt,
    const float *bverts, int bcnt)
{
    /* initial guess */
    float d[3] = {0};
    struct gjk_support s = {0};
    f3cpy(s.a, averts);
    f3cpy(s.b, bverts);
    f3sub(d, s.b, s.a);

    /* run gjk algorithm */
    struct gjk_simplex gsx = {0};
    while (gjk(&gsx, &s, d)) {
        float n[3]; f3mul(n, d, -1);
        s.aid = polyhedron_support(s.a, n, averts, acnt);
        s.bid = polyhedron_support(s.b, d, bverts, bcnt);
        f3sub(d, s.b, s.a);
    }
    *res = gjk_analyze(&gsx);
    return gsx.hit;
}
static int
polyhedron_is_intersecting_polyhedron(const float *averts, int acnt,
    const float *bverts, int bcnt)
{
    struct gjk_result res;
    return polyhedron_intersect_polyhedron(&res, averts, acnt, bverts, bcnt);
}
static int
polyhedron_is_intersecting_polyhedron_transform(const float *averts, int acnt,
    const float *apos3, const float *arot33,
    const float *bverts, int bcnt,
    const float *bpos3, const float *brot33)
{
    struct gjk_result res;
    return polyhedron_intersect_polyhedron_transform(&res, averts, acnt,
        apos3, arot33, bverts, bcnt, bpos3, brot33);
}
static int
polyhedron_intersect_aabb(struct gjk_result *res,
    const float *verts, int cnt,
    const float *min, const float *max)
{
    float box[24];
    f3set(box+0, min[0], min[1], min[2]),
    f3set(box+3, min[0], min[1], max[2]);
    f3set(box+6, min[0], max[1], min[2]);
    f3set(box+9, min[0], max[1], max[2]);
    f3set(box+12,max[0], min[1], min[2]);
    f3set(box+15,max[0], min[1], max[2]);
    f3set(box+18,max[0], max[1], min[2]);
    f3set(box+21,max[0], max[1], max[2]);
    return polyhedron_intersect_polyhedron(res, verts, cnt, box, 8);
}
static int
polyhedron_intersect_aabb_transform(struct gjk_result *res,
    const float *verts, int cnt, const float *pos3, const float *rot33,
    const float *min, const float *max)
{
    float box[24];
    static const float zero[3];
    static const float id[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    f3set(box+0, min[0], min[1], min[2]),
    f3set(box+3, min[0], min[1], max[2]);
    f3set(box+6, min[0], max[1], min[2]);
    f3set(box+9, min[0], max[1], max[2]);
    f3set(box+12,max[0], min[1], min[2]);
    f3set(box+15,max[0], min[1], max[2]);
    f3set(box+18,max[0], max[1], min[2]);
    f3set(box+21,max[0], max[1], max[2]);
    return polyhedron_intersect_polyhedron_transform(res, verts, cnt, pos3, rot33,
        box, 8, zero, &id[0][0]);
}
static int
polyhedron_is_intersecting_aabb(const float *verts, int cnt,
    const float *min, const float *max)
{
    struct gjk_result res;
    return polyhedron_intersect_aabb(&res, verts, cnt, min, max);
}
static int
polyhedron_is_intersecting_aabb_transform(const float *verts, int cnt,
    const float *apos3, const float *arot33, const float *min, const float *max)
{
    struct gjk_result res;
    return polyhedron_intersect_aabb_transform(&res, verts, cnt, apos3, arot33, min, max);
}

/* ============================================================================
 *
 *                                  MATH OBJECTS
 *
 * =========================================================================== */
#define vf(v) (&((v).x))
typedef struct v2 {float x,y;} v2;
typedef struct v3 {float x,y,z;} v3;
typedef struct v4 {float x,y,z,w;} v4;
typedef struct quat {float x,y,z,w;} quat;
typedef struct mat4 {v4 x,y,z;} mat3;

#define v3unpack(v) (v).x,(v).y,(v).z
static inline v3 v3mk(float x, float y, float z){v3 r; f3set(&r.x, x,y,z); return r;}
static inline v3 v3mkv(const float *v){v3 r; f3set(&r.x, v[0],v[1],v[2]); return r;}
static inline v3 v3add(v3 a, v3 b){f3add(vf(a),vf(a),vf(b));return a;}
static inline v3 v3sub(v3 a, v3 b){f3sub(vf(a),vf(a),vf(b)); return a;}
static inline v3 v3scale(v3 v, float s){f3mul(vf(v),vf(v),s); return v;}
static inline float v3dot(v3 a, v3 b){return f3dot(vf(a),vf(b));}
static inline v3 v3norm(v3 v) {f3norm(vf(v)); return v;}
static inline v3 v3cross(v3 a, v3 b) {v3 r; f3cross(vf(r),vf(a),vf(b)); return r;}
static inline v3 v3lerp(v3 a, float t, v3 b) {v3 r; f3lerp(vf(r),vf(a),t,vf(b)); return r;}

/* ============================================================================
 *
 *                              COLLISION VOLUME
 *
 * =========================================================================== */
struct sphere {v3 p; float r;};
struct aabb {v3 min, max;};
struct plane {v3 p, n;};
struct capsule {v3 a,b; float r;};
struct ray {v3 p, d;};
struct raycast {v3 p, n; float t0, t1;};

/* plane */
static struct plane plane(v3 p, v3 n);
static struct plane planef(float px, float py, float pz, float nx, float ny, float nz);
static struct plane planefv(float px, float py, float pz, float nx, float ny, float nz);
/* ray */
static struct ray ray(v3 p, v3 d);
static struct ray rayf(float px, float py, float pz, float dx, float dy, float dz);
static struct ray rayfv(const float *p, const float *d);
static int ray_test_plane(struct raycast *o, struct ray r,  struct plane p);
static int ray_test_triangle(struct raycast *o, struct ray r, const struct v3 *tri);
static int ray_test_sphere(struct raycast *o, struct ray r, struct sphere s);
static int ray_test_aabb(struct raycast *o, struct ray r, struct aabb a);
/* sphere */
static struct sphere sphere(v3 p, float r);
static struct sphere spheref(float cx, float cy, float cz, float r);
static struct sphere spherefv(const float *p, float r);
static int sphere_test_sphere(struct sphere a, struct sphere b);
static int sphere_test_sphere_manifold(struct manifold *m, struct sphere a, struct sphere b);
static int sphere_test_aabb(struct sphere s, struct aabb a);
static int sphere_test_aabb_manifold(struct manifold *m, struct sphere s, struct aabb a);
static int sphere_test_capsule(struct sphere s, struct capsule c);
static int sphere_test_capsule_manifold(struct manifold *m, struct sphere s, struct capsule c);
/* aabb */
static struct aabb aabb(v3 min, v3 max);
static struct aabb aabbf(float minx, float miny, float minz, float maxx, float maxy, float maxz);
static struct aabb aabbfv(const float *min, const float *max);
static struct aabb aabb_transform(struct aabb a, const float *rot, const v3 t);
static v3 aabb_closest_point(struct aabb, v3 p);
static int aabb_test_aabb(struct aabb, struct aabb);
static int aabb_test_aabb_manifold(struct manifold *m, struct aabb, struct aabb);
static int aabb_test_sphere(struct aabb a, struct sphere s);
static int aabb_test_sphere_manifold(struct manifold *m,struct aabb a, struct sphere s);
static int aabb_test_capsule(struct aabb a, struct capsule c);
static int aabb_test_capsule_manifold(struct manifold *m, struct aabb a, struct capsule c);
/* capsule */
static struct capsule capsule(v3 from, v3 to, float r);
static struct capsule capsulef(float fx, float fy, float fz, float tx, float ty, float tz, float r);
static struct capsule capsulefv(const float *f, const float *t, float r);
static float capsule_sqdist_point(struct capsule, v3 p);
static v3 capsule_closest_point(struct capsule, v3 pnt);
static int capsule_test_sphere(struct capsule, struct sphere s);
static int capsule_test_sphere_manifold(struct manifold *m,struct capsule, struct sphere s);
static int capsule_test_aabb(struct capsule c, struct aabb a);
static int capsule_test_aabb_manifold(struct manifold *m, struct capsule c, struct aabb a);
static int capsule_test_capsule(struct capsule a, struct capsule b);
static int capsule_test_capsule_manifold(struct manifold *m, struct capsule a, struct capsule b);

static struct plane
plane(v3 p, v3 n)
{
    struct plane r;
    r.p = p;
    r.n = n;
    return r;
}
static struct plane
planef(float px, float py, float pz, float nx, float ny, float nz)
{
    struct plane r;
    r.p = v3mk(px,py,pz);
    r.n = v3mk(nx,ny,nz);
    return r;
}
static struct plane
planefv(float px, float py, float pz, float nx, float ny, float nz)
{
    struct plane r;
    r.p = v3mk(px,py,pz);
    r.n = v3mk(nx,ny,nz);
    return r;
}
static struct ray
ray(v3 p, v3 d)
{
    struct ray r;
    r.p = p;
    r.d = v3norm(d);
    return r;
}
static struct ray
rayf(float px, float py, float pz, float dx, float dy, float dz)
{
    struct ray r;
    r.p = v3mk(px,py,pz);
    r.d = v3norm(v3mk(dx,dy,dz));
    return r;
}
static struct ray
rayfv(const float *p, const float *d)
{
    struct ray r;
    r.p = v3mk(p[0],p[1],p[2]);
    r.d = v3norm(v3mk(d[0],d[1],d[2]));
    return r;
}
static int
ray_test_plane(struct raycast *o, struct ray r,  struct plane p)
{
    float pf[4];
    planeq(pf, &p.n.x, &p.p.x);
    float t = ray_intersects_plane(&r.p.x, &r.d.x, pf);
    if (t <= 0.0f) return 0;
    o->p = v3add(r.p, v3scale(r.d, t));
    o->t0 = o->t1 = t;
    o->n = v3scale(p.n, -1.0f);
    return 1;
}
static int
ray_test_triangle(struct raycast *o, struct ray r, const struct v3 *tri)
{
    float t = ray_intersects_triangle(&r.p.x, &r.d.x, &tri[0].x, &tri[1].x, &tri[2].x);
    if (t <= 0) return 0;
    o->t0 = o->t1 = t;
    o->p = v3add(r.p, v3scale(r.d, t));
    o->n = v3norm(v3cross(v3sub(tri[1],tri[0]),v3sub(tri[2],tri[0])));
    return 1;
}
static int
ray_test_sphere(struct raycast *o, struct ray r, struct sphere s)
{
    if (!ray_intersects_sphere(&o->t0, &o->t1, &r.p.x, &r.d.x, &s.p.x, s.r))
        return 0;
    o->p = v3add(r.p, v3scale(r.d, min(o->t0,o->t1)));
    o->n = v3norm(v3sub(o->p, s.p));
    return 1;
}
static int
ray_test_aabb(struct raycast *o, struct ray r, struct aabb a)
{
    v3 pnt, ext, c;
    float d, min;
    if (!ray_intersects_aabb(&o->t0, &o->t1, &r.p.x, &r.d.x, &a.min.x, &a.max.x))
        return 0;

    o->p = v3add(r.p, v3scale(r.d, min(o->t0,o->t1)));
    ext = v3sub(a.max, a.min);
    c = v3add(a.min, v3scale(ext,0.5f));
    pnt = v3sub(o->p, c);

    min = fabs(ext.x - fabs(pnt.x));
    o->n = v3scale(v3mk(1,0,0), sign(pnt.x));
    d = fabs(ext.y - fabs(pnt.y));
    if (d < min) {
        min = d;
        o->n = v3scale(v3mk(0,1,0), sign(pnt.y));
    }
    d = fabs(ext.z - fabs(pnt.z));
    if (d < min)
        o->n = v3scale(v3mk(0,0,1), sign(pnt.z));
    return 1;
}
static struct sphere
sphere(v3 p, float r)
{
    struct sphere s;
    s.p = p;
    s.r = r;
    return s;
}
static struct sphere
spheref(float cx, float cy, float cz, float r)
{
    struct sphere s;
    s.p = v3mk(cx,cy,cz);
    s.r = r;
    return s;
}
static struct sphere
spherefv(const float *p, float r)
{
    return spheref(p[0], p[1], p[2], r);
}
static int
sphere_test_sphere(struct sphere a, struct sphere b)
{
    return sphere_intersects_sphere(&a.p.x, a.r, &b.p.x, b.r);
}
static int
sphere_test_sphere_manifold(struct manifold *m, struct sphere a, struct sphere b)
{
    return sphere_intersects_sphere_manifold(m, &a.p.x, a.r, &b.p.x, b.r);
}
static int
sphere_test_aabb(struct sphere s, struct aabb a)
{
    return sphere_intersect_aabb(&s.p.x, s.r, &a.min.x, &a.max.x);
}
static int
sphere_test_aabb_manifold(struct manifold *m, struct sphere s, struct aabb a)
{
    return sphere_intersect_aabb_manifold(m, &s.p.x, s.r, &a.min.x, &a.max.x);
}
static int
sphere_test_capsule(struct sphere s, struct capsule c)
{
    return sphere_intersect_capsule(&s.p.x, s.r, &c.a.x, &c.b.x, c.r);
}
static int
sphere_test_capsule_manifold(struct manifold *m,
    struct sphere s, struct capsule c)
{
    return sphere_intersect_capsule_manifold(m, &s.p.x, s.r, &c.a.x, &c.b.x, c.r);
}
static struct aabb
aabb(v3 min, v3 max)
{
    struct aabb a;
    a.min = min;
    a.max = max;
    return a;
}
static struct aabb
aabbf(float minx, float miny, float minz, float maxx, float maxy, float maxz)
{
    struct aabb a;
    a.min = v3mk(minx,miny,minz);
    a.max = v3mk(maxx,maxy,maxz);
    return a;
}
static struct aabb
aabbfv(const float *min, const float *max)
{
    struct aabb a;
    a.min = v3mk(min[0],min[1],min[2]);
    a.max = v3mk(max[0],max[1],max[3]);
    return a;
}
static struct aabb
aabb_transform(struct aabb a, const float *rot, const v3 t)
{
    struct aabb res;
    aabb_rebalance_transform(&res.min.x, &res.max.x, &a.min.x, &a.max.x, rot, &t.x);
    return res;
}
static v3
aabb_closest_point(struct aabb a, v3 p)
{
    v3 res = {0};
    aabb_closest_point_to_point(&res.x, &a.min.x, &a.max.x, &p.x);
    return res;
}
static int
aabb_test_aabb(struct aabb a, struct aabb b)
{
    return aabb_intersect_aabb(&a.min.x, &a.max.x, &b.min.x, &b.max.x);
}
static int
aabb_test_aabb_manifold(struct manifold *m, struct aabb a, struct aabb b)
{
    return aabb_intersect_aabb_manifold(m, &a.min.x, &a.max.x, &b.min.x, &b.max.x);
}
static int
aabb_test_sphere(struct aabb a, struct sphere s)
{
    return aabb_intersect_sphere(&a.min.x, &a.max.x, &s.p.x, s.r);
}
static int
aabb_test_sphere_manifold(struct manifold *m,
    struct aabb a, struct sphere s)
{
    return aabb_intersect_sphere_manifold(m, &a.min.x, &a.max.x, &s.p.x, s.r);
}
static int
aabb_test_capsule(struct aabb a, struct capsule c)
{
    return aabb_intersect_capsule(&a.min.x, &a.max.x, &c.a.x, &c.b.x, c.r);
}
static int
aabb_test_capsule_manifold(struct manifold *m, struct aabb a, struct capsule c)
{
    return aabb_intersect_capsule_manifold(m, &a.min.x, &a.max.x, &c.a.x, &c.b.x, c.r);
}
static struct capsule
capsule(v3 a, v3 b, float r)
{
    struct capsule c;
    c.a = a; c.b = b; c.r = r;
    return c;
}
static struct capsule
capsulef(float ax, float ay, float az, float bx, float by, float bz, float r)
{
    struct capsule c;
    c.a = v3mk(ax,ay,az);
    c.b = v3mk(bx,by,bz);
    c.r = r;
    return c;
}
static struct capsule
capsulefv(const float *a, const float *b, float r)
{
    struct capsule c;
    c.a = v3mkv(a);
    c.b = v3mkv(b);
    c.r = r;
    return c;
}
static float
capsule_sqdist_point(struct capsule c, v3 p)
{
    return capsule_point_sqdist(&c.a.x, &c.b.x, c.r, &p.x);
}
static v3
capsule_closest_point(struct capsule c, v3 pnt)
{
    float p[3];
    capsule_closest_point_to_point(p, &c.a.x, &c.b.x, c.r, &pnt.x);
    return v3mkv(p);
}
static int
capsule_test_sphere(struct capsule c, struct sphere s)
{
    return capsule_intersect_sphere(&c.a.x, &c.b.x, c.r, &s.p.x, s.r);
}
static int
capsule_test_sphere_manifold(struct manifold *m, struct capsule c, struct sphere s)
{
    return capsule_intersect_sphere_manifold(m, &c.a.x, &c.b.x, c.r, &s.p.x, s.r);
}
static int
capsule_test_aabb(struct capsule c, struct aabb a)
{
    return capsule_intersect_aabb(&c.a.x, &c.b.x, c.r, &a.min.x, &a.max.x);
}
static int
capsule_test_aabb_manifold(struct manifold *m, struct capsule c, struct aabb a)
{
    return capsule_intersect_aabb_manifold(m, &c.a.x, &c.b.x, c.r, &a.min.x, &a.max.x);
}
static int
capsule_test_capsule(struct capsule a, struct capsule b)
{
    return capsule_intersect_capsule(&a.a.x, &a.b.x, a.r, &b.a.x, &b.b.x, b.r);
}
static int
capsule_test_capsule_manifold(struct manifold *m, struct capsule a, struct capsule b)
{
    return capsule_intersect_capsule_manifold(m, &a.a.x, &a.b.x, a.r, &b.a.x, &b.b.x, b.r);
}
