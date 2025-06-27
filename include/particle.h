#ifndef PARTICLE_H
#define PARTICLE_H

#include "vec3.h"
#include <vector>
#include <random>
#include <memory>
#include <functional>

class Particle {
public:
    Vec3 position;
    Vec3 velocity;
    Vec3 acceleration;
    Vec3 color;
    Vec3 startColor;
    Vec3 endColor;
    
    double life;
    double maxLife;
    double size;
    double startSize;
    double endSize;
    double mass;
    double drag;
    double bounce;
    
    bool active;
    
    Particle() : position(0,0,0), velocity(0,0,0), acceleration(0,0,0),
                 color(1,1,1), startColor(1,1,1), endColor(1,1,1),
                 life(1.0), maxLife(1.0), size(1.0), startSize(1.0), endSize(1.0),
                 mass(1.0), drag(0.98), bounce(0.5), active(false) {}
    
    void update(double dt) {
        if (!active) return;
        
        velocity += acceleration * dt;
        velocity *= drag;
        position += velocity * dt;
        
        life -= dt;
        if (life <= 0) {
            active = false;
            return;
        }
        
        double t = 1.0 - (life / maxLife);
        color = startColor * (1.0 - t) + endColor * t;
        size = startSize * (1.0 - t) + endSize * t;
    }
    
    void reset(const Vec3& pos, const Vec3& vel, double lifetime) {
        position = pos;
        velocity = vel;
        acceleration = Vec3(0,0,0);
        life = lifetime;
        maxLife = lifetime;
        active = true;
    }
};

class ParticleEmitter {
public:
    Vec3 position;
    Vec3 direction;
    double spread;
    double speed;
    double speedVariation;
    
    double emissionRate;
    double emissionTimer;
    
    Vec3 gravity;
    Vec3 force;
    
    double minLifetime;
    double maxLifetime;
    
    double minSize;
    double maxSize;
    
    Vec3 startColorMin;
    Vec3 startColorMax;
    Vec3 endColorMin;
    Vec3 endColorMax;
    
    bool active;
    bool burst;
    int burstCount;
    
    std::function<void(Particle&)> customInitializer;
    
    ParticleEmitter(const Vec3& pos = Vec3(0,0,0))
        : position(pos), direction(0,1,0), spread(0.5), speed(5.0), speedVariation(1.0),
          emissionRate(10.0), emissionTimer(0), gravity(0,-9.81,0), force(0,0,0),
          minLifetime(1.0), maxLifetime(2.0), minSize(0.1), maxSize(0.5),
          startColorMin(1,1,1), startColorMax(1,1,1), endColorMin(0,0,0), endColorMax(0,0,0),
          active(true), burst(false), burstCount(0), customInitializer(nullptr) {}
    
    void emitParticle(Particle& particle, std::mt19937& rng) {
        std::uniform_real_distribution<double> dist(-1.0, 1.0);
        std::uniform_real_distribution<double> dist01(0.0, 1.0);
        
        particle.position = position;
        
        Vec3 randomDir = direction;
        randomDir.x += dist(rng) * spread;
        randomDir.y += dist(rng) * spread;
        randomDir.z += dist(rng) * spread;
        randomDir.normalize();
        
        double vel = speed + dist(rng) * speedVariation;
        particle.velocity = randomDir * vel;
        
        particle.acceleration = gravity + force;
        
        double lifetime = minLifetime + dist01(rng) * (maxLifetime - minLifetime);
        particle.life = lifetime;
        particle.maxLife = lifetime;
        
        particle.size = minSize + dist01(rng) * (maxSize - minSize);
        particle.startSize = particle.size;
        particle.endSize = particle.size * 0.1;
        
        particle.startColor = Vec3(
            startColorMin.x + dist01(rng) * (startColorMax.x - startColorMin.x),
            startColorMin.y + dist01(rng) * (startColorMax.y - startColorMin.y),
            startColorMin.z + dist01(rng) * (startColorMax.z - startColorMin.z)
        );
        
        particle.endColor = Vec3(
            endColorMin.x + dist01(rng) * (endColorMax.x - endColorMin.x),
            endColorMin.y + dist01(rng) * (endColorMax.y - endColorMin.y),
            endColorMin.z + dist01(rng) * (endColorMax.z - endColorMin.z)
        );
        
        particle.color = particle.startColor;
        particle.active = true;
        
        if (customInitializer) {
            customInitializer(particle);
        }
    }
};

class ParticleSystem {
private:
    std::vector<Particle> particles;
    std::vector<std::shared_ptr<ParticleEmitter>> emitters;
    std::mt19937 randomGen;
    
    int maxParticles;
    int activeParticles;
    
public:
    ParticleSystem(int maxCount = 10000) 
        : maxParticles(maxCount), activeParticles(0), randomGen(std::random_device{}()) {
        particles.resize(maxParticles);
    }
    
    void addEmitter(std::shared_ptr<ParticleEmitter> emitter) {
        emitters.push_back(emitter);
    }
    
    Particle* getInactiveParticle() {
        for (auto& p : particles) {
            if (!p.active) {
                return &p;
            }
        }
        return nullptr;
    }
    
    void update(double dt) {
        activeParticles = 0;
        
        for (auto& emitter : emitters) {
            if (!emitter->active) continue;
            
            if (emitter->burst && emitter->burstCount > 0) {
                for (int i = 0; i < emitter->burstCount; ++i) {
                    Particle* p = getInactiveParticle();
                    if (p) {
                        emitter->emitParticle(*p, randomGen);
                    }
                }
                emitter->burstCount = 0;
                emitter->burst = false;
            } else {
                emitter->emissionTimer += dt;
                
                double spawnInterval = 1.0 / emitter->emissionRate;
                while (emitter->emissionTimer >= spawnInterval) {
                    Particle* p = getInactiveParticle();
                    if (p) {
                        emitter->emitParticle(*p, randomGen);
                    }
                    emitter->emissionTimer -= spawnInterval;
                }
            }
        }
        
        for (auto& p : particles) {
            if (p.active) {
                p.update(dt);
                if (p.active) activeParticles++;
            }
        }
    }
    
    const std::vector<Particle>& getParticles() const { return particles; }
    int getActiveCount() const { return activeParticles; }
    
    void clear() {
        for (auto& p : particles) {
            p.active = false;
        }
        activeParticles = 0;
    }
};

class ParticleEffects {
public:
    static std::shared_ptr<ParticleEmitter> createStarfield(const Vec3& center, double radius) {
        auto emitter = std::make_shared<ParticleEmitter>(center);
        emitter->direction = Vec3(0, 0, 0);
        emitter->spread = 1.0;
        emitter->speed = 0;
        emitter->gravity = Vec3(0, 0, 0);
        emitter->minLifetime = 100000;
        emitter->maxLifetime = 100000;
        emitter->minSize = 0.01;
        emitter->maxSize = 0.05;
        emitter->startColorMin = Vec3(0.8, 0.8, 1.0);
        emitter->startColorMax = Vec3(1.0, 1.0, 0.8);
        emitter->burst = true;
        emitter->burstCount = 1000;
        
        emitter->customInitializer = [center, radius](Particle& p) {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dist(-radius, radius);
            
            p.position = Vec3(
                center.x + dist(gen),
                center.y + dist(gen),
                center.z + dist(gen)
            );
            p.velocity = Vec3(0, 0, 0);
        };
        
        return emitter;
    }
    
    static std::shared_ptr<ParticleEmitter> createCometTail(const Vec3& cometPos) {
        auto emitter = std::make_shared<ParticleEmitter>(cometPos);
        emitter->direction = Vec3(-1, 0, 0);
        emitter->spread = 0.3;
        emitter->speed = 2.0;
        emitter->speedVariation = 1.0;
        emitter->emissionRate = 50;
        emitter->gravity = Vec3(0, 0, 0);
        emitter->minLifetime = 2.0;
        emitter->maxLifetime = 4.0;
        emitter->minSize = 0.1;
        emitter->maxSize = 0.3;
        emitter->startColorMin = Vec3(0.8, 0.9, 1.0);
        emitter->startColorMax = Vec3(1.0, 1.0, 1.0);
        emitter->endColorMin = Vec3(0.0, 0.0, 0.2);
        emitter->endColorMax = Vec3(0.1, 0.1, 0.3);
        
        return emitter;
    }
    
    static std::shared_ptr<ParticleEmitter> createEngineExhaust(const Vec3& pos, const Vec3& dir) {
        auto emitter = std::make_shared<ParticleEmitter>(pos);
        emitter->direction = dir;
        emitter->spread = 0.2;
        emitter->speed = 10.0;
        emitter->speedVariation = 2.0;
        emitter->emissionRate = 100;
        emitter->gravity = Vec3(0, 0, 0);
        emitter->minLifetime = 0.5;
        emitter->maxLifetime = 1.0;
        emitter->minSize = 0.2;
        emitter->maxSize = 0.4;
        emitter->startColorMin = Vec3(1.0, 0.8, 0.0);
        emitter->startColorMax = Vec3(1.0, 1.0, 0.5);
        emitter->endColorMin = Vec3(0.8, 0.0, 0.0);
        emitter->endColorMax = Vec3(1.0, 0.2, 0.0);
        
        return emitter;
    }
    
    static std::shared_ptr<ParticleEmitter> createPlanetaryRing(const Vec3& center, double radius) {
        auto emitter = std::make_shared<ParticleEmitter>(center);
        emitter->burst = true;
        emitter->burstCount = 500;
        emitter->gravity = Vec3(0, 0, 0);
        emitter->minLifetime = 100000;
        emitter->maxLifetime = 100000;
        emitter->minSize = 0.02;
        emitter->maxSize = 0.1;
        emitter->startColorMin = Vec3(0.7, 0.6, 0.5);
        emitter->startColorMax = Vec3(0.9, 0.8, 0.7);
        
        emitter->customInitializer = [center, radius](Particle& p) {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> angle(0, 2 * M_PI);
            std::uniform_real_distribution<> r(radius * 0.8, radius * 1.2);
            std::uniform_real_distribution<> thickness(-0.1, 0.1);
            
            double a = angle(gen);
            double rad = r(gen);
            
            p.position = Vec3(
                center.x + rad * cos(a),
                center.y + thickness(gen),
                center.z + rad * sin(a)
            );
            
            double orbitalSpeed = sqrt(1.0 / rad) * 5.0;
            p.velocity = Vec3(-sin(a) * orbitalSpeed, 0, cos(a) * orbitalSpeed);
        };
        
        return emitter;
    }
};

#endif