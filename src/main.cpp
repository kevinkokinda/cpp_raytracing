#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <iomanip>
#include "scene.h"
#include "renderer.h"
#include "input.h"
#include "physics.h"
#include "particle.h"

class Application {
private:
    std::shared_ptr<Scene> scene;
    std::shared_ptr<Renderer> renderer;
    std::shared_ptr<Input> input;
    std::shared_ptr<CameraController> cameraController;
    
    bool running;
    double time;
    double deltaTime;
    
    int windowWidth;
    int windowHeight;
    
public:
    Application(int width = 120, int height = 40) 
        : windowWidth(width), windowHeight(height), running(true), time(0), deltaTime(0.016) {
        
        input = std::make_shared<Input>();
        renderer = std::make_shared<Renderer>(width, height, 8, 2);
        
        setupScene();
        setupInput();
    }
    
    void setupScene() {
        scene = std::make_shared<Scene>();
        
        scene->addMaterial("sun", std::make_shared<Emissive>(Vec3(1.0, 0.95, 0.8), 10.0));
        scene->addMaterial("earth", std::make_shared<Lambertian>(Vec3(0.2, 0.5, 0.8)));
        scene->addMaterial("earth_land", std::make_shared<Lambertian>(Vec3(0.3, 0.5, 0.2)));
        scene->addMaterial("mars", std::make_shared<Lambertian>(Vec3(0.8, 0.3, 0.1)));
        scene->addMaterial("moon", std::make_shared<Metal>(Vec3(0.8, 0.8, 0.8), 0.3));
        scene->addMaterial("jupiter", std::make_shared<Lambertian>(Vec3(0.8, 0.7, 0.5)));
        scene->addMaterial("saturn", std::make_shared<Lambertian>(Vec3(0.9, 0.8, 0.6)));
        scene->addMaterial("asteroid", std::make_shared<Metal>(Vec3(0.5, 0.4, 0.3), 0.8));
        scene->addMaterial("comet", std::make_shared<Metal>(Vec3(0.8, 0.9, 1.0), 0.1));
        
        auto sun = std::make_shared<CelestialBody>("Sun", CelestialBody::STAR, Vec3(0,0,0), 1.989e30, 5.0);
        sun->temperature = 5800;
        sun->luminosity = 1.0;
        auto sunGeom = std::make_shared<Sphere>(sun->position, sun->radius);
        auto sunNode = std::make_shared<PhysicsNode>("Sun", sunGeom, scene->getMaterial("sun"), sun);
        scene->addNode(sunNode);
        
        auto earth = std::make_shared<CelestialBody>("Earth", CelestialBody::PLANET, Vec3(25,0,0), 5.972e24, 2.0);
        earth->orbitalRadius = 25;
        earth->orbitalPeriod = 365;
        earth->axialTilt = 23.5;
        earth->rotationPeriod = 1.0;
        earth->parent = sun.get();
        auto earthGeom = std::make_shared<Sphere>(earth->position, earth->radius);
        auto earthNode = std::make_shared<PhysicsNode>("Earth", earthGeom, scene->getMaterial("earth"), earth);
        scene->addNode(earthNode);
        
        auto moon = std::make_shared<CelestialBody>("Moon", CelestialBody::MOON, Vec3(28,0,0), 7.34e22, 0.5);
        moon->orbitalRadius = 3;
        moon->orbitalPeriod = 27.3;
        moon->parent = earth.get();
        auto moonGeom = std::make_shared<Sphere>(moon->position, moon->radius);
        auto moonNode = std::make_shared<PhysicsNode>("Moon", moonGeom, scene->getMaterial("moon"), moon);
        scene->addNode(moonNode);
        
        auto mars = std::make_shared<CelestialBody>("Mars", CelestialBody::PLANET, Vec3(35,0,0), 6.39e23, 1.5);
        mars->orbitalRadius = 35;
        mars->orbitalPeriod = 687;
        mars->axialTilt = 25.2;
        mars->parent = sun.get();
        auto marsGeom = std::make_shared<Sphere>(mars->position, mars->radius);
        auto marsNode = std::make_shared<PhysicsNode>("Mars", marsGeom, scene->getMaterial("mars"), mars);
        scene->addNode(marsNode);
        
        auto jupiter = std::make_shared<CelestialBody>("Jupiter", CelestialBody::PLANET, Vec3(50,0,0), 1.89e27, 4.0);
        jupiter->orbitalRadius = 50;
        jupiter->orbitalPeriod = 4333;
        jupiter->parent = sun.get();
        auto jupiterGeom = std::make_shared<Sphere>(jupiter->position, jupiter->radius);
        auto jupiterNode = std::make_shared<PhysicsNode>("Jupiter", jupiterGeom, scene->getMaterial("jupiter"), jupiter);
        scene->addNode(jupiterNode);
        
        auto saturn = std::make_shared<CelestialBody>("Saturn", CelestialBody::PLANET, Vec3(70,0,0), 5.68e26, 3.5);
        saturn->orbitalRadius = 70;
        saturn->orbitalPeriod = 10759;
        saturn->parent = sun.get();
        auto saturnGeom = std::make_shared<Sphere>(saturn->position, saturn->radius);
        auto saturnNode = std::make_shared<PhysicsNode>("Saturn", saturnGeom, scene->getMaterial("saturn"), saturn);
        scene->addNode(saturnNode);
        
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> distRadius(40, 60);
        std::uniform_real_distribution<> distPhase(0, 2 * M_PI);
        std::uniform_real_distribution<> distSize(0.1, 0.5);
        std::uniform_real_distribution<> distPeriod(500, 2000);
        
        for (int i = 0; i < 20; ++i) {
            double radius = distRadius(gen);
            double phase = distPhase(gen);
            double size = distSize(gen);
            
            Vec3 pos(radius * cos(phase), 0, radius * sin(phase));
            
            auto asteroid = std::make_shared<CelestialBody>(
                "Asteroid" + std::to_string(i), 
                CelestialBody::ASTEROID, 
                pos, 
                1e20 * size, 
                size
            );
            asteroid->orbitalRadius = radius;
            asteroid->orbitalPeriod = distPeriod(gen);
            asteroid->orbitalPhase = phase / (2 * M_PI);
            asteroid->parent = sun.get();
            
            auto asteroidGeom = std::make_shared<Sphere>(asteroid->position, asteroid->radius);
            auto asteroidNode = std::make_shared<PhysicsNode>(
                asteroid->name, 
                asteroidGeom, 
                scene->getMaterial("asteroid"), 
                asteroid
            );
            scene->addNode(asteroidNode);
        }
        
        auto comet = std::make_shared<CelestialBody>("Halley", CelestialBody::COMET, Vec3(80,0,0), 2.2e14, 0.8);
        comet->orbitalRadius = 80;
        comet->orbitalPeriod = 27375;
        comet->parent = sun.get();
        auto cometGeom = std::make_shared<Sphere>(comet->position, comet->radius);
        auto cometNode = std::make_shared<PhysicsNode>("Halley", cometGeom, scene->getMaterial("comet"), comet);
        scene->addNode(cometNode);
        
        auto particleSystem = std::make_shared<ParticleSystem>(10000);
        
        auto starfield = ParticleEffects::createStarfield(Vec3(0,0,0), 200);
        particleSystem->addEmitter(starfield);
        
        auto saturnRing = ParticleEffects::createPlanetaryRing(saturn->position, saturn->radius * 2.0);
        particleSystem->addEmitter(saturnRing);
        
        auto cometTail = ParticleEffects::createCometTail(comet->position);
        cometNode->updateCallback = [cometTail, comet](double dt) {
            cometTail->position = comet->position;
            Vec3 toSun = -comet->position.normalized();
            cometTail->direction = -toSun;
        };
        particleSystem->addEmitter(cometTail);
        
        auto psNode = std::make_shared<ParticleSystemNode>("ParticleSystem", particleSystem);
        scene->addNode(psNode);
        
        auto keyLight = std::make_shared<LightNode>("KeyLight", LightNode::POINT);
        keyLight->setLocalPosition(Vec3(0, 0, 0));
        keyLight->color = Vec3(1.0, 0.95, 0.8);
        keyLight->intensity = 1.0;
        keyLight->castShadows = true;
        scene->addNode(keyLight);
        
        auto fillLight = std::make_shared<LightNode>("FillLight", LightNode::DIRECTIONAL);
        fillLight->direction = Vec3(1, -0.5, 0.5).normalized();
        fillLight->color = Vec3(0.5, 0.5, 0.7);
        fillLight->intensity = 0.3;
        fillLight->castShadows = false;
        scene->addNode(fillLight);
        
        auto camera = std::make_shared<Camera>(Vec3(40, 30, 40), Vec3(0, 0, 0), Vec3(0, 1, 0), 60, 
                                              static_cast<double>(windowWidth) / windowHeight);
        auto camNode = std::make_shared<CameraNode>("MainCamera", camera);
        scene->addNode(camNode);
        
        scene->setAmbientLight(Vec3(0.05, 0.05, 0.1));
        scene->setFog(Vec3(0.0, 0.0, 0.05), 0.002);
        
        scene->getPhysics()->setGravitationalConstant(0.5);
        scene->getPhysics()->setTimeScale(100.0);
        
        auto initCamera = scene->getActiveCamera();
        if (initCamera) {
            cameraController = std::make_shared<CameraController>(input.get(), initCamera->position);
            cameraController->setOrbitDistance(50);
        }
    }
    
    void setupInput() {
        input->addKeyCallback([this](const Input::KeyEvent& event) {
            if (event.pressed) {
                switch (event.key) {
                    case Input::KEY_ESC:
                        running = false;
                        break;
                    case Input::KEY_1:
                        renderer->setPostProcessingEnabled(!renderer->getStats().totalRays);
                        break;
                    case Input::KEY_2:
                        renderer->setParticlesEnabled(!renderer->getStats().particlesRendered);
                        break;
                    case Input::KEY_3:
                        renderer->setDepthOfFieldEnabled(!renderer->getStats().totalRays);
                        break;
                    case Input::KEY_4:
                        renderer->setMotionBlurEnabled(!renderer->getStats().totalRays);
                        break;
                    case Input::KEY_P:
                        scene->getPhysics()->setTimeScale(
                            scene->getPhysics()->timeScale == 0 ? 100.0 : 0.0
                        );
                        break;
                    case Input::KEY_O:
                        focusOnPlanet("Earth");
                        break;
                    case Input::KEY_I:
                        focusOnPlanet("Mars");
                        break;
                    case Input::KEY_U:
                        focusOnPlanet("Jupiter");
                        break;
                    default:
                        break;
                }
            }
        });
    }
    
    void focusOnPlanet(const std::string& planetName) {
        auto node = scene->findNode(planetName);
        if (node) {
            cameraController->setTarget(node->worldPosition);
            cameraController->setOrbitDistance(10.0);
        }
    }
    
    void update() {
        input->update();
        
        if (cameraController) {
            cameraController->update(deltaTime);
            
            auto camera = scene->getActiveCamera();
            if (camera) {
                camera->position = cameraController->getPosition();
                camera->target = cameraController->getLookAt();
                camera->updateCamera();
            }
        }
        
        scene->update(deltaTime);
        time += deltaTime;
    }
    
    void render() {
        renderer->render(*scene);
    }
    
    void run() {
        input->startCapture();
        
        std::cout << "\033[?25l";
        
        auto lastFrameTime = std::chrono::high_resolution_clock::now();
        
        while (running) {
            auto currentTime = std::chrono::high_resolution_clock::now();
            deltaTime = std::chrono::duration<double>(currentTime - lastFrameTime).count();
            lastFrameTime = currentTime;
            
            update();
            render();
            
            double targetFrameTime = 1.0 / 30.0;
            double sleepTime = targetFrameTime - deltaTime;
            if (sleepTime > 0) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(static_cast<int>(sleepTime * 1000))
                );
            }
        }
        
        std::cout << "\033[?25h";
        std::cout << "\033[2J\033[H";
        
        input->stopCapture();
    }
};

int main() {
    try {
        std::cout << "\033[2J\033[H";
        std::cout << "=== Advanced Solar System Simulator ===\n";
        std::cout << "Initializing...\n";
        
        Application app(120, 40);
        
        std::cout << "\nStarting simulation...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        app.run();
        
        std::cout << "Simulation ended.\n";
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}