#pragma once
#include "ray.h"
#include "vec3.h"
#include "material.h"
#include <memory>

class Sphere : public Hittable {
public:
    Vec3 center;
    double radius;
    std::shared_ptr<Material> material;

    Sphere() {}
    Sphere(const Vec3& center, double radius, std::shared_ptr<Material> mat = nullptr) 
        : center(center), radius(radius), material(mat) {}

    bool hit(const Ray& ray, double tMin, double tMax, HitRecord& rec) const override {
        Vec3 oc = ray.origin - center;
        double a = ray.direction.lengthSquared();
        double halfB = oc.dot(ray.direction);
        double c = oc.lengthSquared() - radius * radius;
        double discriminant = halfB * halfB - a * c;

        if (discriminant < 0) return false;

        double sqrtd = sqrt(discriminant);
        double root = (-halfB - sqrtd) / a;
        if (root < tMin || tMax < root) {
            root = (-halfB + sqrtd) / a;
            if (root < tMin || tMax < root)
                return false;
        }

        rec.t = root;
        rec.point = ray.at(rec.t);
        rec.p = rec.point;
        Vec3 outwardNormal = (rec.point - center) / radius;
        rec.setFaceNormal(ray, outwardNormal);
        rec.material = material.get();

        return true;
    }
};