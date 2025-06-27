#ifndef SCENE_H
#define SCENE_H

#include "vec3.h"
#include "ray.h"
#include "sphere.h"
#include "material.h"
#include "camera.h"
#include "physics.h"
#include "particle.h"
#include <vector>
#include <memory>
#include <string>
#include <unordered_map>
#include <functional>

class SceneNode {
public:
    std::string name;
    Vec3 localPosition;
    Vec3 localRotation;
    Vec3 localScale;
    
    Vec3 worldPosition;
    Vec3 worldRotation;
    Vec3 worldScale;
    
    bool visible;
    bool active;
    
    SceneNode* parent;
    std::vector<std::shared_ptr<SceneNode>> children;
    
    std::function<void(double)> updateCallback;
    
    SceneNode(const std::string& n = "Node") 
        : name(n), localPosition(0,0,0), localRotation(0,0,0), localScale(1,1,1),
          worldPosition(0,0,0), worldRotation(0,0,0), worldScale(1,1,1),
          visible(true), active(true), parent(nullptr) {}
    
    virtual ~SceneNode() = default;
    
    void addChild(std::shared_ptr<SceneNode> child) {
        children.push_back(child);
        child->parent = this;
        child->updateTransform();
    }
    
    void removeChild(std::shared_ptr<SceneNode> child) {
        auto it = std::find(children.begin(), children.end(), child);
        if (it != children.end()) {
            (*it)->parent = nullptr;
            children.erase(it);
        }
    }
    
    void setLocalPosition(const Vec3& pos) {
        localPosition = pos;
        updateTransform();
    }
    
    void setLocalRotation(const Vec3& rot) {
        localRotation = rot;
        updateTransform();
    }
    
    void setLocalScale(const Vec3& scale) {
        localScale = scale;
        updateTransform();
    }
    
    virtual void updateTransform() {
        if (parent) {
            worldPosition = parent->worldPosition + localPosition;
            worldRotation = parent->worldRotation + localRotation;
            worldScale = Vec3(
                parent->worldScale.x * localScale.x,
                parent->worldScale.y * localScale.y,
                parent->worldScale.z * localScale.z
            );
        } else {
            worldPosition = localPosition;
            worldRotation = localRotation;
            worldScale = localScale;
        }
        
        for (auto& child : children) {
            child->updateTransform();
        }
    }
    
    virtual void update(double dt) {
        if (!active) return;
        
        if (updateCallback) {
            updateCallback(dt);
        }
        
        for (auto& child : children) {
            child->update(dt);
        }
    }
    
    SceneNode* findChild(const std::string& name) {
        if (this->name == name) return this;
        
        for (auto& child : children) {
            SceneNode* found = child->findChild(name);
            if (found) return found;
        }
        
        return nullptr;
    }
};

class GeometryNode : public SceneNode {
public:
    std::shared_ptr<Hittable> geometry;
    std::shared_ptr<Material> material;
    
    GeometryNode(const std::string& name, std::shared_ptr<Hittable> geom, std::shared_ptr<Material> mat)
        : SceneNode(name), geometry(geom), material(mat) {}
};

class LightNode : public SceneNode {
public:
    Vec3 color;
    double intensity;
    
    enum LightType {
        POINT,
        DIRECTIONAL,
        SPOT,
        AREA
    } type;
    
    Vec3 direction;
    double spotAngle;
    double spotSoftness;
    
    Vec3 areaSize;
    int areaSamples;
    
    bool castShadows;
    
    LightNode(const std::string& name, LightType t = POINT)
        : SceneNode(name), color(1,1,1), intensity(1.0), type(t),
          direction(0,-1,0), spotAngle(45.0), spotSoftness(0.1),
          areaSize(1,1,1), areaSamples(4), castShadows(true) {}
    
    Vec3 getIntensityAt(const Vec3& point) const {
        switch (type) {
            case POINT: {
                double dist = (point - worldPosition).length();
                double attenuation = 1.0 / (1.0 + 0.09 * dist + 0.032 * dist * dist);
                return color * intensity * attenuation;
            }
            case DIRECTIONAL:
                return color * intensity;
            case SPOT: {
                Vec3 toPoint = (point - worldPosition).normalized();
                double angle = acos(toPoint.dot(direction));
                if (angle > spotAngle * M_PI / 180.0) return Vec3(0,0,0);
                
                double spot = pow(toPoint.dot(direction), spotSoftness * 10);
                double dist = (point - worldPosition).length();
                double attenuation = 1.0 / (1.0 + 0.09 * dist + 0.032 * dist * dist);
                return color * intensity * spot * attenuation;
            }
            case AREA:
                return color * intensity;
        }
        return Vec3(0,0,0);
    }
};

class CameraNode : public SceneNode {
public:
    std::shared_ptr<Camera> camera;
    
    CameraNode(const std::string& name, std::shared_ptr<Camera> cam)
        : SceneNode(name), camera(cam) {}
    
    void updateTransform() override {
        SceneNode::updateTransform();
        if (camera) {
            camera->lookFrom = worldPosition;
        }
    }
};

class PhysicsNode : public GeometryNode {
public:
    std::shared_ptr<RigidBody> rigidBody;
    
    PhysicsNode(const std::string& name, std::shared_ptr<Hittable> geom, 
                std::shared_ptr<Material> mat, std::shared_ptr<RigidBody> body)
        : GeometryNode(name, geom, mat), rigidBody(body) {}
    
    void update(double dt) override {
        GeometryNode::update(dt);
        
        if (rigidBody) {
            setLocalPosition(rigidBody->position);
            
            if (auto sphere = std::dynamic_pointer_cast<Sphere>(geometry)) {
                sphere->center = rigidBody->position;
            }
        }
    }
};

class ParticleSystemNode : public SceneNode {
public:
    std::shared_ptr<ParticleSystem> particleSystem;
    
    ParticleSystemNode(const std::string& name, std::shared_ptr<ParticleSystem> ps)
        : SceneNode(name), particleSystem(ps) {}
    
    void update(double dt) override {
        SceneNode::update(dt);
        
        if (particleSystem) {
            particleSystem->update(dt);
        }
    }
};

class Scene {
private:
    std::shared_ptr<SceneNode> rootNode;
    std::vector<std::shared_ptr<GeometryNode>> geometryNodes;
    std::vector<std::shared_ptr<LightNode>> lights;
    std::vector<std::shared_ptr<CameraNode>> cameras;
    std::shared_ptr<CameraNode> activeCamera;
    
    std::shared_ptr<PhysicsEngine> physics;
    std::vector<std::shared_ptr<ParticleSystem>> particleSystems;
    
    Vec3 ambientLight;
    Vec3 fogColor;
    double fogDensity;
    
    std::unordered_map<std::string, std::shared_ptr<Material>> materials;
    std::unordered_map<std::string, std::shared_ptr<SceneNode>> namedNodes;
    
public:
    Scene() : rootNode(std::make_shared<SceneNode>("Root")),
              ambientLight(0.1, 0.1, 0.1), fogColor(0.5, 0.6, 0.7), fogDensity(0.0),
              physics(std::make_shared<PhysicsEngine>()) {}
    
    std::shared_ptr<SceneNode> getRoot() { return rootNode; }
    
    void addNode(std::shared_ptr<SceneNode> node, const std::string& parentName = "Root") {
        if (parentName == "Root") {
            rootNode->addChild(node);
        } else {
            SceneNode* parent = rootNode->findChild(parentName);
            if (parent) {
                parent->addChild(node);
            }
        }
        
        namedNodes[node->name] = node;
        
        if (auto geom = std::dynamic_pointer_cast<GeometryNode>(node)) {
            geometryNodes.push_back(geom);
            
            if (auto phys = std::dynamic_pointer_cast<PhysicsNode>(node)) {
                if (phys->rigidBody) {
                    physics->addBody(phys->rigidBody);
                }
            }
        } else if (auto light = std::dynamic_pointer_cast<LightNode>(node)) {
            lights.push_back(light);
        } else if (auto cam = std::dynamic_pointer_cast<CameraNode>(node)) {
            cameras.push_back(cam);
            if (!activeCamera) {
                activeCamera = cam;
            }
        } else if (auto ps = std::dynamic_pointer_cast<ParticleSystemNode>(node)) {
            if (ps->particleSystem) {
                particleSystems.push_back(ps->particleSystem);
            }
        }
    }
    
    void removeNode(const std::string& name) {
        auto it = namedNodes.find(name);
        if (it != namedNodes.end()) {
            auto node = it->second;
            
            geometryNodes.erase(
                std::remove_if(geometryNodes.begin(), geometryNodes.end(),
                    [&name](const auto& n) { return n->name == name; }),
                geometryNodes.end()
            );
            
            lights.erase(
                std::remove_if(lights.begin(), lights.end(),
                    [&name](const auto& n) { return n->name == name; }),
                lights.end()
            );
            
            cameras.erase(
                std::remove_if(cameras.begin(), cameras.end(),
                    [&name](const auto& n) { return n->name == name; }),
                cameras.end()
            );
            
            if (node->parent) {
                node->parent->removeChild(node);
            }
            
            namedNodes.erase(it);
        }
    }
    
    SceneNode* findNode(const std::string& name) {
        auto it = namedNodes.find(name);
        return it != namedNodes.end() ? it->second.get() : nullptr;
    }
    
    void setActiveCamera(const std::string& name) {
        for (auto& cam : cameras) {
            if (cam->name == name) {
                activeCamera = cam;
                break;
            }
        }
    }
    
    std::shared_ptr<Camera> getActiveCamera() const {
        return activeCamera ? activeCamera->camera : nullptr;
    }
    
    void addMaterial(const std::string& name, std::shared_ptr<Material> mat) {
        materials[name] = mat;
    }
    
    std::shared_ptr<Material> getMaterial(const std::string& name) {
        auto it = materials.find(name);
        return it != materials.end() ? it->second : nullptr;
    }
    
    void update(double dt) {
        physics->step(dt);
        rootNode->update(dt);
    }
    
    bool hit(const Ray& r, double tMin, double tMax, HitRecord& rec) const {
        HitRecord tempRec;
        bool hitAnything = false;
        double closestSoFar = tMax;
        
        for (const auto& node : geometryNodes) {
            if (!node->visible) continue;
            
            if (node->geometry->hit(r, tMin, closestSoFar, tempRec)) {
                hitAnything = true;
                closestSoFar = tempRec.t;
                rec = tempRec;
                rec.material = node->material.get();
            }
        }
        
        return hitAnything;
    }
    
    Vec3 computeLighting(const Vec3& point, const Vec3& normal, const Vec3& viewDir,
                        const Material* mat, int depth = 0) const {
        Vec3 result = ambientLight;
        
        for (const auto& light : lights) {
            if (!light->visible) continue;
            
            Vec3 lightIntensity = light->getIntensityAt(point);
            
            if (light->castShadows) {
                Vec3 lightDir;
                if (light->type == LightNode::DIRECTIONAL) {
                    lightDir = -light->direction;
                } else {
                    lightDir = (light->worldPosition - point).normalized();
                }
                
                Ray shadowRay(point + normal * 0.001, lightDir);
                HitRecord shadowRec;
                
                double shadowDist = light->type == LightNode::DIRECTIONAL ? 
                    10000.0 : (light->worldPosition - point).length();
                
                if (!hit(shadowRay, 0.001, shadowDist, shadowRec)) {
                    double diff = std::max(0.0, normal.dot(lightDir));
                    result += lightIntensity * diff;
                }
            } else {
                Vec3 lightDir = light->type == LightNode::DIRECTIONAL ?
                    -light->direction : (light->worldPosition - point).normalized();
                    
                double diff = std::max(0.0, normal.dot(lightDir));
                result += lightIntensity * diff;
            }
        }
        
        if (fogDensity > 0) {
            double dist = point.length();
            double fogFactor = exp(-fogDensity * dist);
            result = result * fogFactor + fogColor * (1.0 - fogFactor);
        }
        
        return result;
    }
    
    const std::vector<std::shared_ptr<GeometryNode>>& getGeometry() const { return geometryNodes; }
    const std::vector<std::shared_ptr<LightNode>>& getLights() const { return lights; }
    const std::vector<std::shared_ptr<ParticleSystem>>& getParticleSystems() const { return particleSystems; }
    std::shared_ptr<PhysicsEngine> getPhysics() { return physics; }
    
    void setAmbientLight(const Vec3& color) { ambientLight = color; }
    void setFog(const Vec3& color, double density) { fogColor = color; fogDensity = density; }
};

class SceneBuilder {
public:
    static std::shared_ptr<Scene> createSolarSystem() {
        auto scene = std::make_shared<Scene>();
        
        scene->addMaterial("sun", std::make_shared<Emissive>(Vec3(1.0, 0.9, 0.7), 5.0));
        scene->addMaterial("earth", std::make_shared<Lambertian>(Vec3(0.2, 0.5, 0.8)));
        scene->addMaterial("mars", std::make_shared<Lambertian>(Vec3(0.8, 0.3, 0.1)));
        scene->addMaterial("moon", std::make_shared<Metal>(Vec3(0.8, 0.8, 0.8), 0.3));
        scene->addMaterial("asteroid", std::make_shared<Metal>(Vec3(0.5, 0.4, 0.3), 0.8));
        
        auto sun = std::make_shared<CelestialBody>("Sun", CelestialBody::STAR, Vec3(0,0,0), 1.989e30, 5.0);
        auto sunGeom = std::make_shared<Sphere>(sun->position, sun->radius);
        auto sunNode = std::make_shared<PhysicsNode>("Sun", sunGeom, scene->getMaterial("sun"), sun);
        scene->addNode(sunNode);
        
        scene->getPhysics()->addCelestialBody(sun);
        
        auto earth = std::make_shared<CelestialBody>("Earth", CelestialBody::PLANET, Vec3(20,0,0), 5.972e24, 1.5);
        earth->orbitalRadius = 20;
        earth->orbitalPeriod = 365;
        earth->parent = sun.get();
        auto earthGeom = std::make_shared<Sphere>(earth->position, earth->radius);
        auto earthNode = std::make_shared<PhysicsNode>("Earth", earthGeom, scene->getMaterial("earth"), earth);
        scene->addNode(earthNode);
        scene->getPhysics()->addCelestialBody(earth);
        
        auto moon = std::make_shared<CelestialBody>("Moon", CelestialBody::MOON, Vec3(23,0,0), 7.34e22, 0.5);
        moon->orbitalRadius = 3;
        moon->orbitalPeriod = 27.3;
        moon->parent = earth.get();
        auto moonGeom = std::make_shared<Sphere>(moon->position, moon->radius);
        auto moonNode = std::make_shared<PhysicsNode>("Moon", moonGeom, scene->getMaterial("moon"), moon);
        scene->addNode(moonNode);
        scene->getPhysics()->addCelestialBody(moon);
        
        auto mars = std::make_shared<CelestialBody>("Mars", CelestialBody::PLANET, Vec3(30,0,0), 6.39e23, 1.2);
        mars->orbitalRadius = 30;
        mars->orbitalPeriod = 687;
        mars->parent = sun.get();
        auto marsGeom = std::make_shared<Sphere>(mars->position, mars->radius);
        auto marsNode = std::make_shared<PhysicsNode>("Mars", marsGeom, scene->getMaterial("mars"), mars);
        scene->addNode(marsNode);
        scene->getPhysics()->addCelestialBody(mars);
        
        auto particleSystem = std::make_shared<ParticleSystem>(5000);
        auto starfield = ParticleEffects::createStarfield(Vec3(0,0,0), 100);
        particleSystem->addEmitter(starfield);
        
        auto psNode = std::make_shared<ParticleSystemNode>("Starfield", particleSystem);
        scene->addNode(psNode);
        
        auto keyLight = std::make_shared<LightNode>("KeyLight", LightNode::DIRECTIONAL);
        keyLight->direction = Vec3(1, -1, 1).normalized();
        keyLight->intensity = 0.8;
        scene->addNode(keyLight);
        
        auto camera = std::make_shared<Camera>(Vec3(30, 20, 30), Vec3(0, 0, 0), Vec3(0, 1, 0), 60, 2.0);
        auto camNode = std::make_shared<CameraNode>("MainCamera", camera);
        scene->addNode(camNode);
        
        scene->setAmbientLight(Vec3(0.05, 0.05, 0.1));
        scene->setFog(Vec3(0.0, 0.0, 0.05), 0.001);
        
        scene->getPhysics()->setGravitationalConstant(0.1);
        scene->getPhysics()->setTimeScale(10.0);
        
        return scene;
    }
};

#endif