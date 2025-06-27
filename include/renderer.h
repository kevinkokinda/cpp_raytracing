#pragma once
#include "vec3.h"
#include "ray.h"
#include "sphere.h"
#include "camera.h"
#include "material.h"
#include <vector>
#include <memory>
#include <cmath>
#include <random>
#include <iostream>

class Renderer {
public:
    int width, height;
    std::vector<std::shared_ptr<Hittable>> objects;
    Camera camera;

    Renderer(int w, int h, Camera cam) 
        : width(w), height(h), camera(cam) {}

    void addObject(std::shared_ptr<Hittable> obj) {
        objects.push_back(obj);
    }

    Vec3 rayColor(const Ray& ray, int depth = 0) const {
        if (depth >= 10) return Vec3(0, 0, 0);

        HitRecord rec;
        if (!hitWorld(ray, 0.001, 1000.0, rec)) {
            Vec3 unitDirection = ray.direction.normalize();
            double t = 0.5 * (unitDirection.y + 1.0);
            return (1.0 - t) * Vec3(1.0, 1.0, 1.0) + t * Vec3(0.5, 0.7, 1.0);
        }

        Vec3 emitted = rec.material->emitted();
        Vec3 attenuation;
        Ray scattered;
        
        if (rec.material->scatter(ray, rec, attenuation, scattered)) {
            return emitted + attenuation * rayColor(scattered, depth + 1);
        }
        
        return emitted;
    }
    
    bool hitWorld(const Ray& ray, double tMin, double tMax, HitRecord& rec) const {
        HitRecord tempRec;
        bool hitAnything = false;
        double closestSoFar = tMax;

        for (const auto& object : objects) {
            if (object->hit(ray, tMin, closestSoFar, tempRec)) {
                hitAnything = true;
                closestSoFar = tempRec.t;
                rec = tempRec;
            }
        }

        return hitAnything;
    }


    char intensityToChar(double intensity) const {
        const char chars[] = " .'`^\",:;Il!i><~+_-?][}{1)(|\\/tfjrxnuvczXYUJCLQ0OZmwqpdbkhao*#MW&8%B@$";
        int index = (int)(intensity * (sizeof(chars) - 2));
        if (index < 0) index = 0;
        if (index >= sizeof(chars) - 1) index = sizeof(chars) - 2;
        return chars[index];
    }

    void render() const {
        std::cout << "\033[2J\033[H";
        
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);
        
        for (int j = height - 1; j >= 0; --j) {
            for (int i = 0; i < width; ++i) {
                Vec3 totalColor(0, 0, 0);
                const int samples = 4;
                
                for (int s = 0; s < samples; ++s) {
                    double u = (double(i) + dis(gen)) / (width - 1);
                    double v = (double(j) + dis(gen)) / (height - 1);
                    
                    Ray ray = camera.getRay(u, v);
                    totalColor += rayColor(ray);
                }
                
                totalColor = totalColor / samples;
                
                double r = totalColor.x;
                double g = totalColor.y;
                double b = totalColor.z;
                
                int red = (int)(255 * std::max(0.0, std::min(1.0, sqrt(r))));
                int green = (int)(255 * std::max(0.0, std::min(1.0, sqrt(g))));
                int blue = (int)(255 * std::max(0.0, std::min(1.0, sqrt(b))));
                
                std::cout << "\033[38;2;" << red << ";" << green << ";" << blue << "m";
                
                double intensity = (r + g + b) / 3.0;
                std::cout << intensityToChar(intensity);
            }
            std::cout << "\033[0m" << std::endl;
        }
    }
};