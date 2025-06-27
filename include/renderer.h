#ifndef RENDERER_H
#define RENDERER_H

#include "vec3.h"
#include "ray.h"
#include "material.h"
#include "scene.h"
#include "particle.h"
#include "postprocess.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <random>
#include <vector>
#include <string>
#include <sstream>
#include <chrono>
#include <iomanip>

class Renderer {
private:
    int width;
    int height;
    int maxBounces;
    int samplesPerPixel;
    std::mt19937 rng;
    std::uniform_real_distribution<double> dist;
    
    Image framebuffer;
    std::vector<double> depthBuffer;
    PostProcessingPipeline postProcessing;
    
    bool enablePostProcessing;
    bool enableParticles;
    bool enableDepthOfField;
    bool enableMotionBlur;
    
    struct RenderStats {
        int totalRays;
        int primaryRays;
        int shadowRays;
        double renderTime;
        int particlesRendered;
    } stats;
    
public:
    Renderer(int w, int h, int bounces = 5, int samples = 8) 
        : width(w), height(h), maxBounces(bounces), samplesPerPixel(samples),
          rng(std::random_device{}()), dist(0.0, 1.0),
          framebuffer(w, h), depthBuffer(w * h),
          enablePostProcessing(true), enableParticles(true),
          enableDepthOfField(false), enableMotionBlur(false) {
        
        setupPostProcessing();
    }
    
    void setupPostProcessing() {
        postProcessing.clear();
        postProcessing.addProcessor(std::make_unique<Bloom>(0.8, 0.5, 3));
        postProcessing.addProcessor(std::make_unique<Vignette>(0.3, 1.2));
        postProcessing.addProcessor(std::make_unique<ChromaticAberration>(0.003));
        postProcessing.addProcessor(std::make_unique<ToneMapping>(1.2, 2.2));
    }
    
    void setPostProcessingEnabled(bool enabled) { enablePostProcessing = enabled; }
    void setParticlesEnabled(bool enabled) { enableParticles = enabled; }
    void setDepthOfFieldEnabled(bool enabled) { enableDepthOfField = enabled; }
    void setMotionBlurEnabled(bool enabled) { enableMotionBlur = enabled; }

    Vec3 rayColor(const Ray& r, const Scene& scene, int depth) {
        stats.totalRays++;
        if (depth <= 0) return Vec3(0, 0, 0);

        HitRecord rec;
        if (scene.hit(r, 0.001, INFINITY, rec)) {
            Ray scattered;
            Vec3 attenuation;
            Vec3 emitted = rec.material->emitted();
            
            if (rec.material->scatter(r, rec, attenuation, scattered)) {
                Vec3 indirect = rayColor(scattered, scene, depth - 1);
                Vec3 direct = scene.computeLighting(rec.p, rec.normal, -r.getDirection(), rec.material, depth);
                return emitted + attenuation * (direct + indirect * 0.5);
            }
            return emitted;
        }

        Vec3 unitDirection = r.getDirection().normalized();
        double t = 0.5 * (unitDirection.y + 1.0);
        Vec3 skyColor = Vec3(0.05, 0.05, 0.1) * (1.0 - t) + Vec3(0.1, 0.15, 0.3) * t;
        
        double starThreshold = 0.998;
        if (dist(rng) > starThreshold) {
            skyColor += Vec3(1, 1, 1) * 0.5;
        }
        
        return skyColor;
    }
    
    void renderParticles(const Scene& scene) {
        stats.particlesRendered = 0;
        
        if (!enableParticles) return;
        
        auto camera = scene.getActiveCamera();
        if (!camera) return;
        
        for (const auto& ps : scene.getParticleSystems()) {
            for (const auto& particle : ps->getParticles()) {
                if (!particle.active) continue;
                
                Vec3 screenPos = camera->worldToScreen(particle.position);
                if (screenPos.z < 0) continue;
                
                int x = static_cast<int>(screenPos.x * width);
                int y = static_cast<int>(screenPos.y * height);
                
                int radius = static_cast<int>(particle.size * 10 / screenPos.z);
                radius = std::max(1, radius);
                
                for (int dy = -radius; dy <= radius; ++dy) {
                    for (int dx = -radius; dx <= radius; ++dx) {
                        if (dx * dx + dy * dy > radius * radius) continue;
                        
                        int px = x + dx;
                        int py = y + dy;
                        
                        if (px >= 0 && px < width && py >= 0 && py < height) {
                            double alpha = 1.0 - sqrt(dx * dx + dy * dy) / radius;
                            alpha *= particle.color.length() / sqrt(3.0);
                            
                            Vec3& pixel = framebuffer.at(px, py);
                            pixel = pixel * (1 - alpha) + particle.color * alpha;
                        }
                    }
                }
                
                stats.particlesRendered++;
            }
        }
    }

    void render(const Scene& scene) {
        auto startTime = std::chrono::high_resolution_clock::now();
        
        stats.totalRays = 0;
        stats.primaryRays = 0;
        stats.shadowRays = 0;
        
        auto camera = scene.getActiveCamera();
        if (!camera) return;
        
        #pragma omp parallel for schedule(dynamic)
        for (int j = 0; j < height; ++j) {
            for (int i = 0; i < width; ++i) {
                Vec3 color(0, 0, 0);
                double depth = 0;
                
                for (int s = 0; s < samplesPerPixel; ++s) {
                    double u = (i + dist(rng)) / (width - 1);
                    double v = (j + dist(rng)) / (height - 1);
                    Ray r = camera->getRay(u, v);
                    
                    stats.primaryRays++;
                    color += rayColor(r, scene, maxBounces);
                    
                    HitRecord rec;
                    if (scene.hit(r, 0.001, INFINITY, rec)) {
                        depth += rec.t;
                    } else {
                        depth += 1000.0;
                    }
                }
                
                color = color / samplesPerPixel;
                depth = depth / samplesPerPixel;
                
                framebuffer.at(i, j) = color;
                depthBuffer[j * width + i] = depth;
            }
        }
        
        renderParticles(scene);
        
        if (enablePostProcessing) {
            if (enableDepthOfField) {
                auto dof = std::make_unique<DepthOfField>(30.0, 0.1, 5.0);
                dof->setDepthBuffer(depthBuffer);
                dof->process(framebuffer);
            }
            
            if (enableMotionBlur) {
                auto motionBlur = std::make_unique<MotionBlur>(Vec3(2, 0, 0), 8);
                motionBlur->process(framebuffer);
            }
            
            postProcessing.process(framebuffer);
        }
        
        auto endTime = std::chrono::high_resolution_clock::now();
        stats.renderTime = std::chrono::duration<double>(endTime - startTime).count();
        
        display();
    }
    
    void display() {
        std::stringstream output;
        output << "\033[2J\033[H";
        
        for (int j = 0; j < height; ++j) {
            for (int i = 0; i < width; ++i) {
                Vec3 color = framebuffer.at(i, j);
                
                auto clamp = [](double x, double min, double max) { 
                    return x < min ? min : (x > max ? max : x); 
                };
                
                double gamma = 2.2;
                color = Vec3(
                    std::pow(clamp(color.x, 0.0, 1.0), 1.0 / gamma),
                    std::pow(clamp(color.y, 0.0, 1.0), 1.0 / gamma),
                    std::pow(clamp(color.z, 0.0, 1.0), 1.0 / gamma)
                );
                
                double contrast = 1.2;
                double brightness = 0.1;
                color.x = clamp((color.x - 0.5) * contrast + 0.5 + brightness, 0.0, 1.0);
                color.y = clamp((color.y - 0.5) * contrast + 0.5 + brightness, 0.0, 1.0);
                color.z = clamp((color.z - 0.5) * contrast + 0.5 + brightness, 0.0, 1.0);
                
                int r = static_cast<int>(255.999 * color.x);
                int g = static_cast<int>(255.999 * color.y);
                int b = static_cast<int>(255.999 * color.z);
                
                output << "\033[38;2;" << r << ";" << g << ";" << b << "m";
                
                double intensity = (color.x + color.y + color.z) / 3.0;
                double luminance = 0.299 * color.x + 0.587 * color.y + 0.114 * color.z;
                
                luminance = clamp(luminance, 0.0, 1.0);
                
                char displayChar;
                if (luminance > 0.95) displayChar = '@';
                else if (luminance > 0.85) displayChar = '#';
                else if (luminance > 0.75) displayChar = '&';
                else if (luminance > 0.65) displayChar = '%';
                else if (luminance > 0.55) displayChar = '*';
                else if (luminance > 0.45) displayChar = '=';
                else if (luminance > 0.35) displayChar = '+';
                else if (luminance > 0.25) displayChar = '-';
                else if (luminance > 0.15) displayChar = ':';
                else if (luminance > 0.08) displayChar = '.';
                else if (luminance > 0.04) displayChar = '\'';
                else if (luminance > 0.02) displayChar = '`';
                else displayChar = ' ';
                
                output << displayChar;
            }
            output << "\033[0m\n";
        }
        
        output << "\n\033[90m┌─────────────────────────────────────────────────────────────────────────────────────────┐\033[0m\n";
        output << "\033[90m│\033[0m \033[33mRender Stats:\033[0m";
        output << " \033[36mTime:\033[0m " << std::fixed << std::setprecision(3) << stats.renderTime << "s";
        output << " \033[36m│\033[0m \033[35mRays:\033[0m " << stats.totalRays;
        output << " \033[36m│\033[0m \033[32mParticles:\033[0m " << stats.particlesRendered;
        
        int padding = 85 - (42 + std::to_string(stats.totalRays).length() + std::to_string(stats.particlesRendered).length());
        for (int i = 0; i < padding; ++i) output << " ";
        output << "\033[90m│\033[0m\n";
        
        output << "\033[90m├─────────────────────────────────────────────────────────────────────────────────────────┤\033[0m\n";
        output << "\033[90m│\033[0m \033[93mControls:\033[0m \033[37mO\033[0m=Earth \033[37mI\033[0m=Mars \033[37mU\033[0m=Jupiter \033[37mP\033[0m=Pause \033[37m1-4\033[0m=Effects \033[37mESC\033[0m=Quit";
        
        padding = 44;
        for (int i = 0; i < padding; ++i) output << " ";
        output << "\033[90m│\033[0m\n";
        output << "\033[90m└─────────────────────────────────────────────────────────────────────────────────────────┘\033[0m";
        
        std::cout << output.str() << std::flush;
    }
    
    const RenderStats& getStats() const { return stats; }
};

#endif