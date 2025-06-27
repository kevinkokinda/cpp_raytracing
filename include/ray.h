#pragma once
#include "vec3.h"
#include <memory>

class Ray {
public:
    Vec3 origin;
    Vec3 direction;

    Ray() {}
    Ray(const Vec3& origin, const Vec3& direction) : origin(origin), direction(direction) {}

    Vec3 at(double t) const {
        return origin + t * direction;
    }
    
    Vec3 getDirection() const { return direction; }
};

class Material;

struct HitRecord {
    Vec3 point;
    Vec3 p;  // alias for point
    Vec3 normal;
    double t;
    bool frontFace;
    Material* material;

    void setFaceNormal(const Ray& ray, const Vec3& outwardNormal) {
        frontFace = ray.direction.dot(outwardNormal) < 0;
        normal = frontFace ? outwardNormal : -1 * outwardNormal;
    }
};

class Hittable {
public:
    virtual ~Hittable() = default;
    virtual bool hit(const Ray& ray, double tMin, double tMax, HitRecord& rec) const = 0;
};