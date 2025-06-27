#pragma once
#include "vec3.h"
#include "ray.h"
#include <random>

class Material {
public:
    virtual ~Material() = default;
    virtual bool scatter(const Ray& rayIn, const HitRecord& rec, Vec3& attenuation, Ray& scattered) const = 0;
    virtual Vec3 emitted() const { return Vec3(0, 0, 0); }
};

class Lambertian : public Material {
public:
    Vec3 albedo;
    
    Lambertian(const Vec3& a) : albedo(a) {}
    
    bool scatter(const Ray& rayIn, const HitRecord& rec, Vec3& attenuation, Ray& scattered) const override {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_real_distribution<> dis(-1.0, 1.0);
        
        Vec3 randomInSphere;
        do {
            randomInSphere = Vec3(dis(gen), dis(gen), dis(gen));
        } while (randomInSphere.lengthSquared() >= 1.0);
        
        Vec3 scatterDirection = rec.normal + randomInSphere.normalize();
        if (scatterDirection.nearZero()) {
            scatterDirection = rec.normal;
        }
        
        scattered = Ray(rec.point, scatterDirection);
        attenuation = albedo;
        return true;
    }
};

class Metal : public Material {
public:
    Vec3 albedo;
    double fuzz;
    
    Metal(const Vec3& a, double f) : albedo(a), fuzz(f < 1 ? f : 1) {}
    
    bool scatter(const Ray& rayIn, const HitRecord& rec, Vec3& attenuation, Ray& scattered) const override {
        Vec3 reflected = reflect(rayIn.direction.normalize(), rec.normal);
        
        if (fuzz > 0) {
            static std::random_device rd;
            static std::mt19937 gen(rd());
            static std::uniform_real_distribution<> dis(-1.0, 1.0);
            
            Vec3 randomInSphere;
            do {
                randomInSphere = Vec3(dis(gen), dis(gen), dis(gen));
            } while (randomInSphere.lengthSquared() >= 1.0);
            
            reflected += fuzz * randomInSphere;
        }
        
        scattered = Ray(rec.point, reflected);
        attenuation = albedo;
        return scattered.direction.dot(rec.normal) > 0;
    }
    
private:
    Vec3 reflect(const Vec3& v, const Vec3& n) const {
        return v - 2 * v.dot(n) * n;
    }
};

class Emissive : public Material {
public:
    Vec3 color;
    double intensity;
    
    Emissive(const Vec3& c, double i = 1.0) : color(c), intensity(i) {}
    
    bool scatter(const Ray& rayIn, const HitRecord& rec, Vec3& attenuation, Ray& scattered) const override {
        return false;
    }
    
    Vec3 emitted() const override {
        return color * intensity;
    }
};