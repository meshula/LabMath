
// Copyright (c) 2013 Nick Porcino, All rights reserved.
// License is MIT: http://opensource.org/licenses/MIT

#ifndef LABMATH_CAMERA_H
#define LABMATH_CAMERA_H

#include <LabMath/LabMath.h>

namespace lab {

    // This mount is a nodal mount, centered on the camera's sensor
    // The mount's transform is left handed, y is up, -z is forward

    class Mount
    {
        m44f _viewTransform;

    public:
        Mount()
        : _viewTransform(m44f_identity) {}

        void setViewTransform(m44f const& t) { _viewTransform = t; }
        LM_API void setViewTransform(quatf const& q, float r);
        const m44f& viewTransform() const { return _viewTransform; }
        m44f rotationTransform() const { m44f j = _viewTransform; j[3] = {0,0,0,1}; return j; }

        v3f right()    const { return vector_normalize(V3F(_viewTransform[0].x,
                                                           _viewTransform[1].x,
                                                           _viewTransform[2].x)); }
        v3f up()       const { return vector_normalize(V3F(_viewTransform[0].y,
                                                           _viewTransform[1].y,
                                                           _viewTransform[2].y)); }
        v3f forward()  const { return vector_normalize(V3F(_viewTransform[0].z,
                                                           _viewTransform[1].z,
                                                           _viewTransform[2].z)); }
        v3f position() const { return V3F(_viewTransform[0].w,
                                          _viewTransform[1].w,
                                          _viewTransform[2].w); }

        void lookat(v3f eye, v3f target, v3f up) { _viewTransform = lab::make_lookat_transform(eye, target, up); }
    };

    struct Sensor
	{
        // spatial characteristics
        float handedness = -1.f; // left handed
        v2f aperture = { 35.f, 24.5f };
        v2f enlarge = { 1, 1 };
        v2f shift = { 0, 0 };

        // sensoing characteristics
        v3f lift = { 0, 0, 0 };
        v3f gain = { 1, 1, 1 };
        v3f knee = { 1, 1, 1 };
        v3f gamma = { 2.2f, 2.2f, 2.2f };
    };

/*

A and B are on the sensor plane.

There is a point at infinity, O, on the axis of the lens, whose parallel rays
converge on B.

                             a
    ---------------------------+---------------+ A
                             b| |              |
    O--------------------------+---------------+ B
                             c| |              |
    ---------------------------+---------------+ 
    infinity                 lens           sensor plane
    C


A-B is half the sensor plane aperture.
b-B is the focal length

There is a point C at infinity, whose parallel rays through a, b, and c,
converge on the edge of the sensor plane, at A.

The field of view of the lens, at infinity, can therefore be approximated by

   fov = 2 atan((h/2) / f)

Given a field of view, the assumption of a focus at infinity, and a sensor 
aperture, the focal length may be calculated accordingly:

   f = tan(fov/2) / (h/2)

 */


    struct Optics
    {
        MM focalLength = 50.f;
        float zfar = 1e5f;
        float znear = 0.1f;
        float squeeze = 1.f; // w/h
    };

    LM_API m44f    perspective(const Sensor& sensor, const Optics& optics);
    inline Radians verticalFOV(const Sensor& sensor, const Optics& optics) {
        return 2.f * atanf(sensor.aperture.y / (2.f * optics.focalLength) / optics.squeeze);
    }
    inline MM focal_length_from_FOV(MM sensor_aperture, Radians fov) {
        return tanf(fov * 0.5f) / (sensor_aperture * 0.5f);
    }

    LM_API m44f perspective(const Sensor& sensor, const Optics& optics);

    class Camera
    {
    public:
        Mount  mount;
        Sensor sensor;
        Optics optics;

  		v3f position   { 0, 0, 0 };
		v3f worldUp    { 0, 1, 0 };
		v3f focusPoint { 0, 0, -10 };

        Camera() {
            updateViewTransform();
        }

        // Creates a matrix suitable for an OpenGL style MVP matrix
        // Be sure to invert the view transform if your graphics engine pre-multiplies.
        //
        void updateViewTransform() {
            mount.lookat(position, focusPoint, worldUp);
        }

        LM_API void frame(std::pair<v3f, v3f> bounds);
        LM_API void autoSetClippingPlanes(std::pair<v3f, v3f> bounds);
    };

    enum class CameraRigMode
    {
        Dolly, Crane, TurnTableOrbit, Fly
    };

    // delta is the 2d motion of a mouse or gesture in the screen plane,
    // typically computed as scale * (currMousePos - prevMousePos);
    //
    LM_API void cameraRig_interact(Camera& camera, CameraRigMode mode, v2f delta);

} // Lab

#endif
