#pragma once
#include "ray.h"
#include "vec3.h"

class Camera {
public:
    Vec3 position;
    Vec3 target;
    Vec3 up;
    double fov;
    double aspectRatio;
    
    Vec3 u, v, w;
    Vec3 horizontal;
    Vec3 vertical;
    Vec3 lowerLeftCorner;

    Camera(Vec3 pos, Vec3 tar, Vec3 vup, double verticalFov, double aspect) 
        : position(pos), target(tar), up(vup), fov(verticalFov), aspectRatio(aspect) {
        updateCamera();
    }

    void updateCamera() {
        double theta = fov * M_PI / 180.0;
        double halfHeight = tan(theta / 2.0);
        double halfWidth = aspectRatio * halfHeight;

        w = (position - target).normalize();
        u = up.cross(w).normalize();
        v = w.cross(u);

        horizontal = 2.0 * halfWidth * u;
        vertical = 2.0 * halfHeight * v;
        lowerLeftCorner = position - halfWidth * u - halfHeight * v - w;
    }

    Ray getRay(double s, double t) const {
        return Ray(position, lowerLeftCorner + s * horizontal + t * vertical - position);
    }
};