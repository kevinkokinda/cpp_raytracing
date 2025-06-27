#include "vec3.h"
#include "ray.h"
#include "sphere.h"
#include "camera.h"
#include "renderer.h"
#include "material.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

int main() {
    const int width = 120;
    const int height = 40;
    const double aspectRatio = double(width) / double(height);

    Vec3 cameraPos(0, 2, 8);
    Vec3 cameraTarget(0, 0, 0);
    Vec3 cameraUp(0, 1, 0);
    Camera camera(cameraPos, cameraTarget, cameraUp, 45.0, aspectRatio);

    Renderer renderer(width, height, camera);

    auto lightMaterial = std::make_shared<Emissive>(Vec3(3.0, 2.5, 2.0), 1.0);
    auto sun = std::make_shared<Sphere>(Vec3(0, 0, 0), 0.5, lightMaterial);
    renderer.addObject(sun);
    
    auto groundMaterial = std::make_shared<Lambertian>(Vec3(0.5, 0.5, 0.5));
    auto ground = std::make_shared<Sphere>(Vec3(0, -1000.5, -1), 1000, groundMaterial);
    renderer.addObject(ground);

    double time = 0;
    const double orbitRadius = 2.5;
    const double orbitSpeed = 1.0;

    while (true) {
        double x = orbitRadius * cos(time * orbitSpeed);
        double y = 0.5 * sin(time * orbitSpeed * 0.5);
        double z = orbitRadius * sin(time * orbitSpeed);
        
        auto planetMaterial = std::make_shared<Lambertian>(Vec3(0.2, 0.5, 0.8));
        auto planet = std::make_shared<Sphere>(Vec3(x, y, z), 0.3, planetMaterial);
        
        auto moonMaterial = std::make_shared<Metal>(Vec3(0.8, 0.8, 0.8), 0.3);
        auto moon = std::make_shared<Sphere>(Vec3(x + 1.0, y, z + 0.8), 0.2, moonMaterial);
        
        renderer.objects.clear();
        renderer.addObject(sun);
        renderer.addObject(ground);
        renderer.addObject(planet);
        renderer.addObject(moon);

        renderer.render();

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        time += 0.1;
    }

    return 0;
}