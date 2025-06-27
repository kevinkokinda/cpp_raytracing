#ifndef PHYSICS_H
#define PHYSICS_H

#include "vec3.h"
#include <vector>
#include <memory>
#include <cmath>

class RigidBody {
public:
    Vec3 position;
    Vec3 velocity;
    Vec3 acceleration;
    Vec3 force;
    
    double mass;
    double radius;
    double restitution;
    double friction;
    
    Vec3 angularVelocity;
    Vec3 torque;
    double momentOfInertia;
    
    bool isStatic;
    bool affectedByGravity;
    
    RigidBody(const Vec3& pos = Vec3(0,0,0), double m = 1.0, double r = 1.0)
        : position(pos), velocity(0,0,0), acceleration(0,0,0), force(0,0,0),
          mass(m), radius(r), restitution(0.8), friction(0.1),
          angularVelocity(0,0,0), torque(0,0,0),
          momentOfInertia(0.4 * m * r * r),
          isStatic(false), affectedByGravity(true) {}
    
    void applyForce(const Vec3& f) {
        if (!isStatic) {
            force += f;
        }
    }
    
    void applyTorque(const Vec3& t) {
        if (!isStatic) {
            torque += t;
        }
    }
    
    void update(double dt) {
        if (isStatic) return;
        
        acceleration = force / mass;
        velocity += acceleration * dt;
        position += velocity * dt;
        
        Vec3 angularAcceleration = torque / momentOfInertia;
        angularVelocity += angularAcceleration * dt;
        
        force = Vec3(0,0,0);
        torque = Vec3(0,0,0);
    }
    
    double kineticEnergy() const {
        double linear = 0.5 * mass * velocity.lengthSquared();
        double rotational = 0.5 * momentOfInertia * angularVelocity.lengthSquared();
        return linear + rotational;
    }
    
    Vec3 momentum() const {
        return velocity * mass;
    }
};

class CelestialBody : public RigidBody {
public:
    std::string name;
    Vec3 color;
    double temperature;
    double luminosity;
    double albedo;
    
    double orbitalRadius;
    double orbitalPeriod;
    double orbitalPhase;
    double axialTilt;
    double rotationPeriod;
    
    std::vector<std::shared_ptr<CelestialBody>> satellites;
    CelestialBody* parent;
    
    enum BodyType {
        STAR,
        PLANET,
        MOON,
        ASTEROID,
        COMET,
        RING_PARTICLE
    } type;
    
    CelestialBody(const std::string& n, BodyType t, const Vec3& pos, double m, double r)
        : RigidBody(pos, m, r), name(n), color(1,1,1), temperature(300),
          luminosity(0), albedo(0.3), orbitalRadius(0), orbitalPeriod(1),
          orbitalPhase(0), axialTilt(0), rotationPeriod(1), parent(nullptr), type(t) {
        
        if (type == STAR) {
            affectedByGravity = false;
            isStatic = true;
            luminosity = 1.0;
            temperature = 5800;
            color = Vec3(1.0, 0.95, 0.8);
        }
    }
    
    void addSatellite(std::shared_ptr<CelestialBody> satellite) {
        satellites.push_back(satellite);
        satellite->parent = this;
    }
    
    Vec3 getOrbitalPosition(double time) const {
        if (parent == nullptr || orbitalRadius == 0) return position;
        
        double angle = 2.0 * M_PI * (time / orbitalPeriod + orbitalPhase);
        double x = orbitalRadius * cos(angle);
        double z = orbitalRadius * sin(angle);
        
        return parent->position + Vec3(x, 0, z);
    }
};

class Collision {
public:
    RigidBody* bodyA;
    RigidBody* bodyB;
    Vec3 normal;
    double penetration;
    Vec3 contactPoint;
    
    Collision(RigidBody* a, RigidBody* b, const Vec3& n, double p, const Vec3& cp)
        : bodyA(a), bodyB(b), normal(n), penetration(p), contactPoint(cp) {}
};

class PhysicsEngine {
private:
    std::vector<std::shared_ptr<RigidBody>> bodies;
    std::vector<std::shared_ptr<CelestialBody>> celestialBodies;
    
    double gravitationalConstant;
    Vec3 globalGravity;
    double damping;
    
    int solverIterations;
    double restitutionThreshold;
    
public:
    double timeScale;
    PhysicsEngine()
        : gravitationalConstant(6.67430e-11), globalGravity(0, -9.81, 0),
          timeScale(1.0), damping(0.999), solverIterations(10),
          restitutionThreshold(1.0) {}
    
    void addBody(std::shared_ptr<RigidBody> body) {
        bodies.push_back(body);
    }
    
    void addCelestialBody(std::shared_ptr<CelestialBody> body) {
        celestialBodies.push_back(body);
        bodies.push_back(body);
    }
    
    void setGravitationalConstant(double g) { gravitationalConstant = g; }
    void setTimeScale(double t) { timeScale = t; }
    
    void applyGravity() {
        for (auto& body : bodies) {
            if (body->affectedByGravity && !body->isStatic) {
                body->applyForce(globalGravity * body->mass);
            }
        }
        
        for (size_t i = 0; i < celestialBodies.size(); ++i) {
            for (size_t j = i + 1; j < celestialBodies.size(); ++j) {
                auto& bodyA = celestialBodies[i];
                auto& bodyB = celestialBodies[j];
                
                Vec3 delta = bodyB->position - bodyA->position;
                double distSq = delta.lengthSquared();
                if (distSq < 0.01) continue;
                
                double forceMag = gravitationalConstant * bodyA->mass * bodyB->mass / distSq;
                Vec3 force = delta.normalized() * forceMag;
                
                if (!bodyA->isStatic) bodyA->applyForce(force);
                if (!bodyB->isStatic) bodyB->applyForce(-force);
            }
        }
    }
    
    bool checkSphereCollision(RigidBody* a, RigidBody* b, Collision& collision) {
        Vec3 delta = b->position - a->position;
        double dist = delta.length();
        double radSum = a->radius + b->radius;
        
        if (dist < radSum) {
            collision.normal = delta.normalized();
            collision.penetration = radSum - dist;
            collision.contactPoint = a->position + collision.normal * a->radius;
            return true;
        }
        return false;
    }
    
    void resolveCollision(Collision& col) {
        RigidBody* a = col.bodyA;
        RigidBody* b = col.bodyB;
        
        if (a->isStatic && b->isStatic) return;
        
        Vec3 relativeVelocity = b->velocity - a->velocity;
        double velocityAlongNormal = relativeVelocity.dot(col.normal);
        
        if (velocityAlongNormal > 0) return;
        
        double e = std::min(a->restitution, b->restitution);
        double j = -(1 + e) * velocityAlongNormal;
        j /= 1/a->mass + 1/b->mass;
        
        Vec3 impulse = col.normal * j;
        
        if (!a->isStatic) a->velocity -= impulse / a->mass;
        if (!b->isStatic) b->velocity += impulse / b->mass;
        
        double totalMass = a->mass + b->mass;
        double moveA = b->mass / totalMass;
        double moveB = a->mass / totalMass;
        
        if (!a->isStatic) a->position -= col.normal * (col.penetration * moveA);
        if (!b->isStatic) b->position += col.normal * (col.penetration * moveB);
        
        Vec3 tangent = relativeVelocity - col.normal * velocityAlongNormal;
        if (tangent.lengthSquared() > 0.0001) {
            tangent.normalize();
            double jt = -relativeVelocity.dot(tangent);
            jt /= 1/a->mass + 1/b->mass;
            
            double mu = std::sqrt(a->friction * b->friction);
            Vec3 frictionImpulse;
            
            if (std::abs(jt) < j * mu) {
                frictionImpulse = tangent * jt;
            } else {
                frictionImpulse = tangent * (-j * mu);
            }
            
            if (!a->isStatic) a->velocity -= frictionImpulse / a->mass;
            if (!b->isStatic) b->velocity += frictionImpulse / b->mass;
        }
    }
    
    void detectAndResolveCollisions() {
        std::vector<Collision> collisions;
        
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                Collision col(bodies[i].get(), bodies[j].get(), Vec3(0,0,0), 0, Vec3(0,0,0));
                if (checkSphereCollision(bodies[i].get(), bodies[j].get(), col)) {
                    collisions.push_back(col);
                }
            }
        }
        
        for (int iter = 0; iter < solverIterations; ++iter) {
            for (auto& col : collisions) {
                resolveCollision(col);
            }
        }
    }
    
    void updateOrbitalMechanics(double time) {
        for (auto& body : celestialBodies) {
            if (body->parent != nullptr && body->orbitalRadius > 0) {
                Vec3 newPos = body->getOrbitalPosition(time);
                body->velocity = (newPos - body->position) / 0.016;
                body->position = newPos;
            }
        }
    }
    
    void step(double dt) {
        double scaledDt = dt * timeScale;
        
        applyGravity();
        
        for (auto& body : bodies) {
            body->velocity *= damping;
            body->update(scaledDt);
        }
        
        detectAndResolveCollisions();
        
        updateOrbitalMechanics(scaledDt);
    }
    
    double getTotalEnergy() const {
        double total = 0;
        for (const auto& body : bodies) {
            total += body->kineticEnergy();
        }
        return total;
    }
    
    Vec3 getCenterOfMass() const {
        Vec3 com(0,0,0);
        double totalMass = 0;
        
        for (const auto& body : bodies) {
            com += body->position * body->mass;
            totalMass += body->mass;
        }
        
        return totalMass > 0 ? com / totalMass : Vec3(0,0,0);
    }
};

#endif