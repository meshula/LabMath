/*
 * File: math-projection.hpp
 * This header aggregates functions for constructing common projection matrices,
 * along with the extraction and computation of attributes on existing
 * projection matrices (like field of view, focal length, near/far clip, etc). 
 */

#pragma once 

#ifndef math_projection_hpp
#define math_projection_hpp

#include "math-common.hpp"
#include <ratio>
#include <assert.h>

namespace polymer { namespace math
{
    struct fov_tanspace
    {
        float left, right, bottom, top, near, far;
    };

    inline float4x4 make_projection_matrix(float l, float r, float b, float t, float n, float f)
    {
        return{ { 2 * n / (r - l),0,0,0 },
                { 0,2 * n / (t - b),0,0 },
                { (r + l) / (r - l),(t + b) / (t - b),-(f + n) / (f - n),-1 },
                { 0,0,-2 * f*n / (f - n),0 } };
    }

    inline float4x4 make_projection_matrix(float vFovInRadians, float aspectRatio, float nearZ, float farZ)
    {
        const float top = nearZ * std::tan(vFovInRadians / 2.f), right = top * aspectRatio;
        return make_projection_matrix(-right, right, -top, top, nearZ, farZ);
    }

    inline float4x4 make_orthographic_matrix(float l, float r, float b, float t, float n, float f)
    {
        return{ { 2 / (r - l),0,0,0 },
                { 0,2 / (t - b),0,0 },
                { 0,0,-2 / (f - n),0 },
                { -(r + l) / (r - l),-(t + b) / (t - b),-(f + n) / (f - n),1 } };
    }

    inline float4x4 make_lookat_transform(const float3 & eye, const float3 & target, const float3 & up) 
    {
        float3 zaxis = normalize(eye - target);
        float3 xaxis = normalize(cross(up, zaxis));
        float3 yaxis = cross(zaxis, xaxis);
        return { { xaxis.x, yaxis.x, zaxis.x, 0.f },
                 { xaxis.y, yaxis.y, zaxis.y, 0.f },
                 { xaxis.z, yaxis.z, zaxis.z, 0.f },
                 { -dot(xaxis, eye), -dot(yaxis, eye), -dot(zaxis, eye), 1.f } };
    }


    // Based on http://aras-p.info/texts/obliqueortho.html (http://www.terathon.com/lengyel/Lengyel-Oblique.pdf)
    // This is valid for both perspective and orthographic projections. `clip_plane` is defined in camera space.
    inline void calculate_oblique_matrix(float4x4 & projection, const float4 & clip_plane)
    {
        const float4 q = mul(inverse(projection), float4(sign(clip_plane.x), sign(clip_plane.y), 1.f, 1.f));
        const float4 c = clip_plane * (2.f / (dot(clip_plane, q)));
        projection[0][2] = c.x - projection[0][3];
        projection[1][2] = c.y - projection[1][3];
        projection[2][2] = c.z - projection[2][3];
        projection[3][2] = c.w - projection[3][3];
    }

    inline void get_tanspace_fov(const float4x4 & projection, fov_tanspace & fov)
    {
        fov.near = projection[3][2] / (projection[2][2] - 1.0f);
        fov.far = projection[3][2] / (1.0f + projection[2][2]);

        fov.left = fov.near * (projection[2][0] - 1.0f) / projection[0][0];
        fov.right = fov.near * (1.0f + projection[2][0]) / projection[0][0];

        fov.bottom = fov.near * (projection[2][1] - 1.0f) / projection[1][1];
        fov.top = fov.near * (1.0f + projection[2][1]) / projection[1][1];
    }

    inline float vfov_from_projection(const float4x4 & projection)
    {
        return std::atan((1.0f / projection[1][1])) * 2.0f;
    }

    inline float aspect_from_projection(const float4x4 & projection)
    {
        return 1.0f / (projection[0][0] * (1.0f / projection[1][1]));
    }

    inline void near_far_clip_from_projection(const float4x4 & projection, float & near, float & far)
    {
        near = projection[3][2] / (projection[2][2] - 1.0f);
        far = projection[3][2] / (1.0f + projection[2][2]);
    }

    inline float get_focal_length(float vFoV)
    {
        return (1.f / (tan(vFoV * 0.5f) * 2.0f));
    }

    inline float get_focal_length_pixels(const int widthPixels, float vFoV)
    {
        auto f = widthPixels / 2 / std::tan(vFoV * 0.5f);
    }

    inline float dfov_to_vfov(float dFoV, float aspectRatio)
    {
        return 2.f * atan(tan(dFoV / 2.f) / sqrt(1.f + aspectRatio * aspectRatio));
    }

    inline float dfov_to_hfov(float dFoV, float aspectRatio)
    {
        return 2.f * atan(tan(dFoV / 2.f) / sqrt(1.f + 1.f / (aspectRatio * aspectRatio)));
    }

    inline float vfov_to_dfov(float vFoV, float aspectRatio)
    {
        return 2.f * atan(tan(vFoV / 2.f) * sqrt(1.f + aspectRatio * aspectRatio));
    }

    inline float hfov_to_dfov(float hFoV, float aspectRatio)
    {
        return 2.f * atan(tan(hFoV / 2.f) * sqrt(1.f + 1.f / (aspectRatio * aspectRatio)));
    }

    inline float hfov_to_vfov(float hFoV, float aspectRatio)
    {
        return 2.f * atan(tan(hFoV / 2.f) / aspectRatio);
    }

    // https://computergraphics.stackexchange.com/questions/1736/vr-and-frustum-culling
    inline void compute_center_view(const float4x4 & leftProjection, const float4x4 & rightProjection, const float interCameraDistance, float4x4 & outProjection, float3 & outTranslation)
    {
        fov_tanspace leftFov = {};
        fov_tanspace rightFov = {};
        get_tanspace_fov(leftProjection, leftFov);
        get_tanspace_fov(rightProjection, rightFov);

        // In the case of VR SDKs which provide asymmetric frusta, get their extents
        const float tanHalfFovWidth = max(leftFov.left, leftFov.right, rightFov.left, rightFov.right);
        const float tanHalfFovHeight = max(leftFov.top, leftFov.bottom, rightFov.top, rightFov.bottom);

        // Double check that the near and far clip planes on both projections match
        float leftNearClip, leftFarClip;
        float rightNearClip, rightFarClip;
        near_far_clip_from_projection(leftProjection, leftNearClip, leftFarClip);
        near_far_clip_from_projection(rightProjection, rightNearClip, rightFarClip);
        assert(leftNearClip == rightNearClip && leftFarClip == rightFarClip);

        const float4x4 superfrustumProjection = make_projection_matrix(-tanHalfFovWidth, tanHalfFovWidth, -tanHalfFovHeight, tanHalfFovHeight, leftNearClip, leftFarClip);
        const float superfrustumAspect = tanHalfFovWidth / tanHalfFovHeight;
        const float superfrustumvFoV = vfov_from_projection(superfrustumProjection);

        // Follows the technique outlined by Cass Everitt here: https://www.facebook.com/photo.php?fbid=10154006919426632&set=a.46932936631.70217.703211631&type=1&theater
        const float Nc = (interCameraDistance * 0.5f) * superfrustumProjection[0][0];
        const float4x4 superfrustumProjectionFixed = make_projection_matrix(superfrustumvFoV, superfrustumAspect, leftNearClip + Nc, leftFarClip + Nc);

        outProjection = superfrustumProjectionFixed;
        outTranslation = float3(0, 0, Nc);
    }
} } // polymer::math

#endif // end math_projection_hpp

#if 0
// vurtun's camera math from https://gist.github.com/vurtun/95f088e4889da2474ad1ce82d7911fee


/* ============================================================================
 *
 *                              CAMERA
 *
 * ============================================================================*/
#define CAM_INF (-1.0f)
enum cam_orient {CAM_ORIENT_QUAT, CAM_ORIENT_MAT};
enum cam_output_z_range {
    CAM_OUT_Z_NEGATIVE_ONE_TO_ONE,
    CAM_OUT_Z_NEGATIVE_ONE_TO_ZERO,
    CAM_OUT_Z_ZERO_TO_ONE
};
struct cam {
    /*------- [input] --------*/
    int zout;
    enum cam_orient orient_struct;
    /* projection */
    float fov;
    float near, far;
    float aspect_ratio;
    float z_range_epsilon;

    /* view */
    float pos[3];
    float off[3];
    float ear[3];
    float q[4];
    float m[3][3];

    /*------- [output] --------*/
    float view[4][4];
    float view_inv[4][4];
    float proj[4][4];
    float proj_inv[4][4];

    float loc[3];
    float forward[3];
    float backward[3];
    float right[3];
    float down[3];
    float left[3];
    float up[3];
    /*-------------------------*/
};
static void
cam_init(struct cam *c)
{
    static const float qid[] = {0,0,0,1};
    static const float m3id[] = {1,0,0,0,1,0,0,0,1};
    static const float m4id[] = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};

    memset(c, 0, sizeof(*c));
    c->aspect_ratio = 3.0f/2.0f;
    c->fov = PI_CONSTANT / 4.0f;
    c->near = 0.01f;
    c->far = 10000;

    memcpy(c->q, qid, sizeof(qid));
    memcpy(c->m, m3id, sizeof(m3id));
    memcpy(c->view, m4id, sizeof(m4id));
    memcpy(c->view_inv, m4id, sizeof(m4id));
    memcpy(c->proj, m4id, sizeof(m4id));
    memcpy(c->proj_inv, m4id, sizeof(m4id));
}
static void
cam_build(struct cam *c)
{
    assert(c);
    if (!c) return;

    /* convert orientation matrix into quaternion */
    if (c->orient_struct == CAM_ORIENT_MAT) {
        float s,t,
        trace = c->m[0][0];
        trace += c->m[1][1];
        trace += c->m[2][2];
        if (trace > 0.0f) {
            t = trace + 1.0f;
            s = (float)sqrt((double)(1.0f/t)) * 0.5f;

            c->q[3] = s * t;
            c->q[0] = (c->m[2][1] - c->m[1][2]) * s;
            c->q[1] = (c->m[0][2] - c->m[2][0]) * s;
            c->q[2] = (c->m[1][0] - c->m[0][1]) * s;
        } else {
            int i = 0, j, k;
            static const int next[] = {1,2,0};
            if (c->m[1][1] > c->m[0][0] ) i = 1;
            if (c->m[2][2] > c->m[i][i] ) i = 2;

            j = next[i]; k = next[j];
            t = (c->m[i][i] - (c->m[j][j] - c->m[k][k])) + 1.0f;
            s = (float)sqrt((double)(1.0f/t)) * 0.5f;

            c->q[i] = s*t;
            c->q[3] = (c->m[k][j] - c->m[j][k]) * s;
            c->q[j] = (c->m[j][i] + c->m[i][j]) * s;
            c->q[k] = (c->m[k][i] + c->m[i][k]) * s;
        }

        /* normalize quaternion */
        {float len2 = c->q[0]*c->q[0] + c->q[1]*c->q[1];
        len2 += c->q[2]*c->q[2] + c->q[3]*c->q[3];
        if (len2 != 0.0f) {
            float len = (float)sqrt((double)len2);
            float inv_len = 1.0f/len;
            c->q[0] *= inv_len; c->q[1] *= inv_len;
            c->q[2] *= inv_len; c->q[3] *= inv_len;
        }}
    }
    /* Camera euler orientation
    It is not feasible to multiply euler angles directly together to represent the camera
    orientation because of gimbal lock (Even quaternions do not save you against
    gimbal lock under all circumstances). While it is true that it is not a problem for
    FPS style cameras, it is a problem for free cameras. To fix that issue this camera only
    takes in the relative angle rotation and does not store the absolute angle values [7].*/
    {float sx =(float)sin((double)(c->ear[0]*0.5f));
    float sy = (float)sin((double)(c->ear[1]*0.5f));
    float sz = (float)sin((double)(c->ear[2]*0.5f));

    float cx = (float)cos((double)(c->ear[0]*0.5f));
    float cy = (float)cos((double)(c->ear[1]*0.5f));
    float cz = (float)cos((double)(c->ear[2]*0.5f));

    float a[4], b[4];
    a[0] = cz*sx; a[1] = sz*sx; a[2] = sz*cx; a[3] = cz*cx;
    b[0] = c->q[0]*cy - c->q[2]*sy;
    b[1] = c->q[3]*sy + c->q[1]*cy;
    b[2] = c->q[2]*cy + c->q[0]*sy;
    b[3] = c->q[3]*cy - c->q[1]*sy;

    c->q[0] = a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1];
    c->q[1] = a[3]*b[1] + a[1]*b[3] + a[2]*b[0] - a[0]*b[2];
    c->q[2] = a[3]*b[2] + a[2]*b[3] + a[0]*b[1] - a[1]*b[0];
    c->q[3] = a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2];
    memset(c->ear, 0, sizeof(c->ear));}

    /* Convert quaternion to matrix
    Next up we want to convert our camera quaternion orientation into a 3x3 matrix
    to generate our view matrix. So to convert from quaternion to rotation
    matrix we first look at how to transform a vector quaternion by and how by matrix.
    To transform a vector by a unit quaternion you turn the vector into a zero w
    quaternion and left multiply by quaternion and right multiply by quaternion inverse:
        p2  = q * q(P1) * pi
            = q(qx,qy,qz,qw) * q(x,y,z,0) * q(-qx,-qy,-qz,qw)

    To get the same result with a rotation matrix you just multiply the vector by matrix:
        p2 = M * p1

    So to get the matrix M you first multiply out the quaternion transformation and
    group each x,y and z term into a column. The end result is: */
    {float x2 = c->q[0] + c->q[0];
    float y2 = c->q[1] + c->q[1];
    float z2 = c->q[2] + c->q[2];

    float xx = c->q[0]*x2;
    float xy = c->q[0]*y2;
    float xz = c->q[0]*z2;

    float yy = c->q[1]*y2;
    float yz = c->q[1]*z2;
    float zz = c->q[2]*z2;

    float wx = c->q[3]*x2;
    float wy = c->q[3]*y2;
    float wz = c->q[3]*z2;

    c->m[0][0] = 1.0f - (yy + zz);
    c->m[0][1] = xy - wz;
    c->m[0][2] = xz + wy;

    c->m[1][0] = xy + wz;
    c->m[1][1] = 1.0f - (xx + zz);
    c->m[1][2] = yz - wx;

    c->m[2][0] = xz - wy;
    c->m[2][1] = yz + wx;
    c->m[2][2] = 1.0f - (xx + yy);}

    /* View matrix
    The general transform pipeline is object space to world space to local camera
    space to screenspace to clipping space. While this particular matrix, the view matrix,
    transforms from world space to local camera space.

    So we start by trying to find the camera world transform a 4x3 matrix which
    can and will be in this particular implementation extended to a 4x4 matrix
    composed of camera rotation, camera translation and camera offset.
    While pure camera position and orientation is usefull for free FPS style cameras,
    allows the camera offset 3rd person cameras or tracking ball behavior.

        T.......camera position
        F.......camera offset
        R.......camera orientation 3x3 matrix
        C.......camera world transform 4x4 matrix

            |1 T|   |R 0|   |1 F|
        C = |0 1| * |0 1| * |0 1|

            |R   Rf+T|
        C = |0      1|
    */
    /* 1.) copy orientation matrix */
    c->view_inv[0][0] = c->m[0][0]; c->view_inv[0][1] = c->m[0][1];
    c->view_inv[0][2] = c->m[0][2]; c->view_inv[1][0] = c->m[1][0];
    c->view_inv[1][1] = c->m[1][1]; c->view_inv[1][2] = c->m[1][2];
    c->view_inv[2][0] = c->m[2][0]; c->view_inv[2][1] = c->m[2][1];
    c->view_inv[2][2] = c->m[2][2];

    /* 2.) transform offset by camera orientation and add translation */
    c->view_inv[3][0] = c->view_inv[0][0]*c->off[0];
    c->view_inv[3][1] = c->view_inv[0][1]*c->off[0];
    c->view_inv[3][2] = c->view_inv[0][2]*c->off[0];

    c->view_inv[3][0] += c->view_inv[1][0]*c->off[1];
    c->view_inv[3][1] += c->view_inv[1][1]*c->off[1];
    c->view_inv[3][2] += c->view_inv[1][2]*c->off[1];

    c->view_inv[3][0] += c->view_inv[2][0]*c->off[2];
    c->view_inv[3][1] += c->view_inv[2][1]*c->off[2];
    c->view_inv[3][2] += c->view_inv[2][2]*c->off[2];

    c->view_inv[3][0] += c->pos[0];
    c->view_inv[3][1] += c->pos[1];
    c->view_inv[3][2] += c->pos[2];

    /* 3.) fill last empty 4x4 matrix row */
    c->view_inv[0][3] = 0;
    c->view_inv[1][3] = 0;
    c->view_inv[2][3] = 0;
    c->view_inv[3][3] = 1.0f;

    /* Now we have a matrix to transform from local camera space into world
    camera space. But remember we are looking for the opposite transform since we
    want to transform the world around the camera. So to get the other way
    around we have to invert our world camera transformation to transform to
    local camera space.

    Usually inverting matricies is quite a complex endeavour both in needed complexity
    as well as number of calculations required. Luckily we can use a nice property
    of orthonormal matrices (matrices with each column being unit length)
    on the more complicated matrix the rotation matrix.
    The inverse of orthonormal matrices is the same as the transpose of the same
    matrix, which is just a matrix column to row swap.

    So with inverse rotation matrix covered the only thing left is the inverted
    camera translation which just needs to be negated plus and this is the more
    important part tranformed by the inverted rotation matrix.
    Why? simply because the inverse of a matrix multiplication M = A * B
    is NOT Mi = Ai * Bi (i for inverse) but rather Mi = Bi * Ai.
    So if we put everything together we get the following view matrix:

        R.......camera orientation matrix3x3
        T.......camera translation
        F.......camera offset
        Ti......camera inverse translation
        Fi......camera inverse offset
        Ri......camera inverse orientation matrix3x3
        V.......view matrix

               (|R   Rf+T|)
        V = inv(|0      1|)

            |Ri -Ri*Ti-Fi|
        V = |0          1|

    Now we finally have our matrix composition and can fill out the view matrix:

    1.) Inverse camera orientation by transpose */
    c->view[0][0] = c->m[0][0];
    c->view[0][1] = c->m[1][0];
    c->view[0][2] = c->m[2][0];

    c->view[1][0] = c->m[0][1];
    c->view[1][1] = c->m[1][1];
    c->view[1][2] = c->m[2][1];

    c->view[2][0] = c->m[0][2];
    c->view[2][1] = c->m[1][2];
    c->view[2][2] = c->m[2][2];

    /* 2.) Transform inverted position vector by transposed orientation and subtract offset */
    {float pos_inv[3];
    pos_inv[0] = -c->pos[0];
    pos_inv[1] = -c->pos[1];
    pos_inv[2] = -c->pos[2];

    c->view[3][0] = c->view[0][0] * pos_inv[0];
    c->view[3][1] = c->view[0][1] * pos_inv[0];
    c->view[3][2] = c->view[0][2] * pos_inv[0];

    c->view[3][0] += c->view[1][0] * pos_inv[1];
    c->view[3][1] += c->view[1][1] * pos_inv[1];
    c->view[3][2] += c->view[1][2] * pos_inv[1];

    c->view[3][0] += c->view[2][0] * pos_inv[2];
    c->view[3][1] += c->view[2][1] * pos_inv[2];
    c->view[3][2] += c->view[2][2] * pos_inv[2];

    c->view[3][0] -= c->off[0];
    c->view[3][1] -= c->off[1];
    c->view[3][2] -= c->off[2];}

    /* 3.) fill last empty 4x4 matrix row */
    c->view[0][3] = 0; c->view[1][3] = 0;
    c->view[2][3] = 0; c->view[3][3] = 1.0f;

    /*  Projection matrix
    While the view matrix transforms from world space to local camera space,
    tranforms the perspective projection matrix from camera space to screen space.

    The actual work for the transformation is from eye coordinates camera
    frustum far plane to a cube with coordinates (-1,1), (0,1), (-1,0) depending on
    argument `out_z_range` in this particual implementation.

    To actually build the projection matrix we need:
        - Vertical field of view angle
        - Screen aspect ratio which controls the horizontal view angle in
            contrast to the field of view.
        - Z coordinate of the frustum near clipping plane
        - Z coordinate of the frustum far clipping plane

    While I will explain how to incooperate the near,far z clipping
    plane I would recommend reading [7] for the other values since it is quite
    hard to do understand without visual aid and he is probably better in
    explaining than I would be.*/
    {float hfov = (float)tan((double)(c->fov*0.5f));
    c->proj[0][0] = 1.0f/(c->aspect_ratio * hfov);
    c->proj[0][1] = 0;
    c->proj[0][2] = 0;
    c->proj[0][3] = 0;

    c->proj[1][0] = 0;
    c->proj[1][1] = 1.0f/hfov;
    c->proj[1][2] = 0;
    c->proj[1][3] = 0;

    c->proj[2][0] = 0;
    c->proj[2][1] = 0;
    c->proj[2][3] = -1.0f;

    c->proj[3][0] = 0;
    c->proj[3][1] = 0;
    c->proj[3][3] = 0;

    if (c->far <= CAM_INF) {
        /* Up to this point we got perspective matrix:

            |1/aspect*hfov  0       0  0|
            |0              1/hfov  0  0|
            |0              0       A  B|
            |0              0       -1 0|

        but we are still missing A and B to map between the frustum near/far
        and the clipping cube near/far value. So we take the lower right part
        of the matrix and multiply by a vector containing the missing
        z and w value, which gives us the resulting clipping cube z;

            |A  B|   |z|    |Az + B |           B
            |-1 0| * |1| =  | -z    | = -A + ------
                                               -z

        So far so good but now we need to map from the frustum near,
        far clipping plane (n,f) to the clipping cube near and far plane (cn,cf).
        So we plugin the frustum near/far values into z and the resulting
        cube near/far plane we want to end up with.

                   B                    B
            -A + ------ = cn and -A + ------ = cf
                   n                    f

        We now have two equations with two unkown A and B since n,f as well
        as cn,cf are provided, so we can solved them by subtitution either by
        hand or, if you are like me prone to easily make small mistakes,
        with WolframAlpha which solved for:*/
        switch (c->zout) {
            default:
            case CAM_OUT_Z_NEGATIVE_ONE_TO_ONE: {
                /* cn = -1 and cf = 1: */
                c->proj[2][2] = -(c->far + c->near) / (c->far - c->near);
                c->proj[3][2] = -(2.0f * c->far * c->near) / (c->far - c->near);
            } break;
            case CAM_OUT_Z_NEGATIVE_ONE_TO_ZERO: {
                /* cn = -1 and cf = 0: */
                c->proj[2][2] = (c->near) / (c->near - c->far);
                c->proj[3][2] = (c->far * c->near) / (c->near - c->far);
            } break;
            case CAM_OUT_Z_ZERO_TO_ONE: {
                /* cn = 0 and cf = 1: */
                c->proj[2][2] = -(c->far) / (c->far - c->near);
                c->proj[3][2] = -(c->far * c->near) / (c->far - c->near);
            } break;
        }
    } else {
    /*  Infinite projection [1]:
        In general infinite projection matrices map direction to points on the
        infinite distant far plane, which is mainly useful for rendering:
            - skyboxes, sun, moon, stars
            - stencil shadow volume caps

        To actually calculate the infinite perspective matrix you let the
        far clip plane go to infinity. Once again I would recommend using and
        checking WolframAlpha to make sure all values are correct, while still
        doing the calculation at least once by hand.

        While it is mathematically correct to go to infinity, floating point errors
        result in a depth smaller or bigger. This in term results in
        fragment culling since the hardware thinks fragments are beyond the
        far clipping plane. The general solution is to introduce an epsilon
        to fix the calculation error and map it to the infinite far plane.

        Important:
        For 32-bit floating point epsilon should be greater than 2.4*10^-7,
        to account for floating point precision problems. */
        switch (c->zout) {
            default:
            case CAM_OUT_Z_NEGATIVE_ONE_TO_ONE: {
                /*lim f->inf -((f+n)/(f-n)) => -((inf+n)/(inf-n)) => -(inf)/(inf) => -1.0*/
                c->proj[2][2] = c->z_range_epsilon - 1.0f;
                /* lim f->inf -(2*f*n)/(f-n) => -(2*inf*n)/(inf-n) => -(2*inf*n)/inf => -2n*/
                c->proj[3][2] = (c->z_range_epsilon - 2.0f) * c->near;
            } break;
            case CAM_OUT_Z_NEGATIVE_ONE_TO_ZERO: {
                /* lim f->inf  n/(n-f) => n/(n-inf) => n/(n-inf) => -0 */
                c->proj[2][2] = c->z_range_epsilon;
                /* lim f->inf (f*n)/(n-f) => (inf*n)/(n-inf) => (inf*n)/-inf = -n */
                c->proj[3][2] = (c->z_range_epsilon - 1.0f) * c->near;
            } break;
            case CAM_OUT_Z_ZERO_TO_ONE: {
                /* lim f->inf (-f)/(f-n) => (-inf)/(inf-n) => -inf/inf = -1 */
                c->proj[2][2] = c->z_range_epsilon - 1.0f;
                /* lim f->inf (-f*n)/(f-n) => (-inf*n)/(inf-n) => (-inf*n)/(inf) => -n */
                c->proj[3][2] = (c->z_range_epsilon - 1.0f) * c->near;
            } break;
        }
    }}
    /* Invert projection [2][3]:
    Since perspective matrices have a fixed layout, it makes sense
    to calculate the specific perspective inverse instead of relying on a default
    matrix inverse function. Actually calculating the matrix for any perspective
    matrix is quite straight forward:

        I.......identity matrix
        p.......perspective matrix
        I(p)....inverse perspective matrix

    1.) Fill a variable inversion matrix and perspective layout matrix into the
        inversion formula: I(p) * p = I

        |x0  x1  x2  x3 |   |a 0 0 0|   |1 0 0 0|
        |x4  x5  x6  x7 | * |0 b 0 0| = |0 1 0 0|
        |x8  x9  x10 x11|   |0 0 c d|   |0 0 1 0|
        |x12 x13 x14 x15|   |0 0 e 0|   |0 0 0 1|

    2.) Multiply inversion matrix times our perspective matrix

        |x0*a x1*b x2*c+x3*e x2*d|      |1 0 0 0|
        |x4*a x5*b x6*c+x7*e x6*d|    = |0 1 0 0|
        |x8*a x9*b x10*c+x11*e x10*d|   |0 0 1 0|
        |x12*a x13*b x14*c+x15*e x14*d| |0 0 0 1|

    3.) Finally substitute each x value: e.g: x0*a = 1 => x0 = 1/a so I(p) at column 0, row 0 is 1/a.

                |1/a 0 0 0|
        I(p) =  |0 1/b 0 0|
                |0 0 0 1/e|
                |0 0 1/d -c/de|

    These steps basically work for any invertable matrices, but I would recommend
    using WolframAlpha for these specific kinds of matrices, since it can
    automatically generate inversion matrices without any fuss or possible
    human calculation errors. */
    memset(c->proj_inv, 0, sizeof(c->proj_inv));
    c->proj_inv[0][0] = 1.0f/c->proj[0][0];
    c->proj_inv[1][1] = 1.0f/c->proj[1][1];
    c->proj_inv[2][3] = 1.0f/c->proj[3][2];
    c->proj_inv[3][2] = 1.0f/c->proj[2][3];
    c->proj_inv[3][3] = -c->proj[2][2]/(c->proj[3][2] * c->proj[2][3]);

    /* fill vectors with data */
    c->loc[0] = c->view_inv[3][0];
    c->loc[1] = c->view_inv[3][1];
    c->loc[2] = c->view_inv[3][2];

    c->right[0] = c->view_inv[0][0];
    c->right[1] = c->view_inv[0][1];
    c->right[2] = c->view_inv[0][2];

    c->left[0] = -c->view_inv[0][0];
    c->left[1] = -c->view_inv[0][1];
    c->left[2] = -c->view_inv[0][2];

    c->up[0] = c->view_inv[1][0];
    c->up[1] = c->view_inv[1][1];
    c->up[2] = c->view_inv[1][2];

    c->down[0] = -c->view_inv[1][0];
    c->down[1] = -c->view_inv[1][1];
    c->down[2] = -c->view_inv[1][2];

    c->forward[0] = c->view_inv[2][0];
    c->forward[1] = c->view_inv[2][1];
    c->forward[2] = c->view_inv[2][2];

    c->backward[0] = -c->view_inv[2][0];
    c->backward[1] = -c->view_inv[2][1];
    c->backward[2] = -c->view_inv[2][2];
}
static void
cam_lookat(struct cam *c,
    float eye_x, float eye_y, float eye_z,
    float ctr_x, float ctr_y, float ctr_z,
    float up_x, float up_y, float up_z)
{
    float f[3], u[3], r[3];
    f[0] = ctr_x - eye_x, f[1] = ctr_y - eye_y, f[2] = ctr_z - eye_z;

    /* calculate right vector */
    r[0] = (f[1]*up_z) - (f[2]*up_y);
    r[1] = (f[2]*up_x) - (f[0]*up_z);
    r[2] = (f[0]*up_y) - (f[1]*up_x);

    /* calculate up vector */
    u[0] = (r[1]*f[2]) - (r[2]*f[1]);
    u[1] = (r[2]*f[0]) - (r[0]*f[2]);
    u[2] = (r[0]*f[1]) - (r[1]*f[0]);

    /* normlize vectors */
    {float fl = f[0]*f[0]+f[1]*f[1]+f[2]*f[2];
    float rl = r[0]*r[0]+r[1]*r[1]+r[2]*r[2];
    float ul = u[0]*u[0]+u[1]*u[1]+u[2]*u[2];

    fl = (fl == 0.0f) ? 1.0f: 1/sqrtf(fl);
    rl = (rl == 0.0f) ? 1.0f: 1/sqrtf(rl);
    ul = (ul == 0.0f) ? 1.0f: 1/sqrtf(ul);

    f[0] *= fl, f[1] *= fl, f[2] *= fl;
    r[0] *= rl, r[1] *= rl, r[2] *= rl;
    u[0] *= ul, u[1] *= ul, u[2] *= ul;}

    /* setup camera */
    c->pos[0] = eye_x, c->pos[1] = eye_y, c->pos[2] = eye_z;
    c->m[0][0] = r[0], c->m[0][1] = r[1], c->m[0][2] = r[2];
    c->m[1][0] = u[0], c->m[1][1] = u[1], c->m[1][2] = u[2];
    c->m[2][0] = f[0], c->m[2][1] = f[1], c->m[2][2] = f[2];

    /* build camera */
    {enum cam_orient orient = c->orient_struct;
    c->orient_struct = CAM_ORIENT_MAT;
    memset(c->ear, 0, sizeof(c->ear));
    cam_build(c);
    c->orient_struct = orient;}
}
static void
cam_move(struct cam *c, float x, float y, float z)
{
    c->pos[0] += c->view_inv[0][0]*x;
    c->pos[1] += c->view_inv[0][1]*x;
    c->pos[2] += c->view_inv[0][2]*x;

    c->pos[0] += c->view_inv[1][0]*y;
    c->pos[1] += c->view_inv[1][1]*y;
    c->pos[2] += c->view_inv[1][2]*y;

    c->pos[0] += c->view_inv[2][0]*z;
    c->pos[1] += c->view_inv[2][1]*z;
    c->pos[2] += c->view_inv[2][2]*z;
}
static void
cam_screen_to_world(const struct cam *c, float width, float height,
    float *res, float screen_x, float screen_y, float camera_z)
{
    /* Screen space to world space coordinates
    To convert from screen space coordinates to world coordinates we
    basically have to revert all transformations typically done to
    convert from world space to screen space:
        Viewport => NDC => Clip => View => World

    Viewport => NDC => Clip
    -----------------------
    First up is the transform from viewport to clipping space.
    To get from clipping space to viewport we calculate:

                    |((x+1)/2)*w|
        Vn = v =    |((1-y)/2)*h|
                    |((z+1)/2)  |

    Now we need to the the inverse process by solvinging for n:
                |(2*x)/w - 1|
        n =     |(2*y)/h    |
                |(2*z)-1)   |
                | 1         |
    */
    float x = (screen_x / width * 2.0f) - 1.0f;
    float y = (screen_y / height) * 2.0f - 1.0f;
    float z = 2.0f * camera_z - 1.0f;

    /* Clip => View
    -----------------------
    A vector v or position p in view space is tranform to clip
    coordinates c by being transformed by a projection matrix P:

        c = P * v

    To convert from clipping coordinates c to view coordinates we
    just have to transfrom c by the inverse projection matrix Pi:

        v = Pi * c

    The inverse projection matrix for all common projection matrices
    can be calculated by (see Camera_Build for more information):

                |1/a 0 0 0|
        Pi  =   |0 1/b 0 0|
                |0 0 0 1/e|
                |0 0 1/d -c/de|

    View => World
    -----------------------
    Finally we just need to convert from view coordinates to world
    coordinates w by transforming our view coordinates by the inverse view
    matrix Vi which in this context is just the camera translation and
    rotation.

        w = Vi * v

    Now we reached our goal and have our world coordinates. This implementation
    combines both the inverse projection as well as inverse view transformation
    into one since the projection layout is known we can do some optimization:*/
    float ax = c->proj_inv[0][0]*x;
    float by = c->proj_inv[1][1]*y;
    float dz = c->proj_inv[2][3]*z;
    float w = c->proj_inv[3][3] + dz;

    res[0] = c->proj_inv[3][2] * c->view_inv[2][0];
    res[0] += c->proj_inv[3][3] * c->view_inv[3][0];
    res[0] += ax * c->view_inv[0][0];
    res[0] += by * c->view_inv[1][0];
    res[0] += dz * c->view_inv[3][0];

    res[1] = c->proj_inv[3][2] * c->view_inv[2][1];
    res[1] += c->proj_inv[3][3] * c->view_inv[3][1];
    res[1] += ax * c->view_inv[0][1];
    res[1] += by * c->view_inv[1][1];
    res[1] += dz * c->view_inv[3][1];

    res[2] = c->proj_inv[3][2] * c->view_inv[2][2];
    res[2] += c->proj_inv[3][3] * c->view_inv[3][2];
    res[2] += ax * c->view_inv[0][2];
    res[2] += by * c->view_inv[1][2];
    res[2] += dz * c->view_inv[3][2];
    res[0] /= w; res[1] /= w; res[2] /= w;
}
static void
cam_picking_ray(const struct cam *c, float w, float h,
    float mx, float my, float *ro, float *rd)
{
    float world[3];
    cam_screen_to_world(c, w, h, world, mx, my, 0);

    /* calculate direction
    We generate the ray normal vector by first transforming the mouse cursor position
    from screen coordinates into world coordinates. After that we only have to
    subtract our camera position from our calculated mouse world position and
    normalize the result to make sure we have a unit vector as direction. */
    ro[0] = c->loc[0];
    ro[1] = c->loc[1];
    ro[2] = c->loc[2];

    rd[0] = world[0] - ro[0];
    rd[1] = world[1] - ro[1];
    rd[2] = world[2] - ro[2];

    /* normalize */
    {float dot = rd[0]*rd[0] + rd[1]*rd[1] + rd[2]*rd[2];
    if (dot != 0.0f) {
        float len = (float)sqrt((double)dot);
        rd[0] /= len; rd[1] /= len; rd[2] /= len;
    }}
}

#endif

