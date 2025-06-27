#include <iostream>
#include <vector>
#include <memory>
#include <random>
#include <cmath>
#include <chrono>
#include <thread>
#include <iomanip>

struct Vec3 {
    double x, y, z;
    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(double t) const { return Vec3(x * t, y * t, z * t); }
    Vec3 operator/(double t) const { return Vec3(x / t, y / t, z / t); }
    
    double length() const { return sqrt(x*x + y*y + z*z); }
    double dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    Vec3 normalize() const { return *this / length(); }
    Vec3 cross(const Vec3& v) const { return Vec3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x); }
};

Vec3 operator*(double t, const Vec3& v) { return v * t; }

struct Ray {
    Vec3 origin, direction;
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d) {}
    Vec3 at(double t) const { return origin + t * direction; }
};

struct Sphere {
    Vec3 center;
    double radius;
    Vec3 color;
    double emission;
    
    Sphere(Vec3 c, double r, Vec3 col, double e = 0) 
        : center(c), radius(r), color(col), emission(e) {}
    
    bool hit(const Ray& ray, double& t) const {
        Vec3 oc = ray.origin - center;
        double a = ray.direction.dot(ray.direction);
        double b = 2.0 * oc.dot(ray.direction);
        double c = oc.dot(oc) - radius * radius;
        double discriminant = b * b - 4 * a * c;
        
        if (discriminant < 0) return false;
        
        double t1 = (-b - sqrt(discriminant)) / (2.0 * a);
        double t2 = (-b + sqrt(discriminant)) / (2.0 * a);
        
        t = (t1 > 0.001) ? t1 : t2;
        return t > 0.001;
    }
};

class SolarSystem {
private:
    std::vector<Sphere> bodies;
    double time;
    
public:
    SolarSystem() : time(0) {
        bodies.push_back(Sphere(Vec3(0, 0, 0), 3, Vec3(1, 0.9, 0.3), 1.0));
        
        bodies.push_back(Sphere(Vec3(15, 0, 0), 1, Vec3(0.3, 0.6, 1.0)));
        bodies.push_back(Sphere(Vec3(18, 0, 0), 0.3, Vec3(0.8, 0.8, 0.8)));
        
        bodies.push_back(Sphere(Vec3(25, 0, 0), 1.2, Vec3(0.8, 0.4, 0.1)));
        
        bodies.push_back(Sphere(Vec3(40, 0, 0), 2.5, Vec3(0.9, 0.8, 0.6)));
        
        bodies.push_back(Sphere(Vec3(60, 0, 0), 2, Vec3(0.8, 0.7, 0.5)));
        
        for (int i = 0; i < 50; ++i) {
            double angle = i * 0.5;
            double radius = 30 + 5 * sin(i * 0.3);
            Vec3 pos(radius * cos(angle), 0, radius * sin(angle));
            bodies.push_back(Sphere(pos, 0.1 + 0.2 * sin(i), Vec3(0.6, 0.5, 0.4)));
        }
    }
    
    void update(double dt) {
        time += dt;
        
        for (size_t i = 1; i < bodies.size() && i < 6; ++i) {
            double period = 100 + i * 50;
            double angle = time * 2 * M_PI / period;
            double distance = 15 + i * 10;
            
            if (i == 2) {
                double earthAngle = time * 2 * M_PI / 150;
                Vec3 earthPos(15 * cos(earthAngle), 0, 15 * sin(earthAngle));
                bodies[1].center = earthPos;
                
                bodies[2].center = earthPos + Vec3(3 * cos(angle * 12), 0, 3 * sin(angle * 12));
            } else if (i > 2) {
                bodies[i].center = Vec3(distance * cos(angle), 0, distance * sin(angle));
            } else {
                bodies[i].center = Vec3(distance * cos(angle), 0, distance * sin(angle));
            }
        }
        
        for (size_t i = 6; i < bodies.size(); ++i) {
            double angle = time * 0.5 + i * 0.1;
            double radius = 25 + 10 * sin(i * 0.2);
            bodies[i].center = Vec3(radius * cos(angle), 0, radius * sin(angle));
        }
    }
    
    Vec3 rayColor(const Ray& ray, int depth = 0) const {
        if (depth > 8) return Vec3(0, 0, 0);
        
        double closest_t = 1000;
        int hit_sphere = -1;
        
        for (size_t i = 0; i < bodies.size(); ++i) {
            double t;
            if (bodies[i].hit(ray, t) && t < closest_t) {
                closest_t = t;
                hit_sphere = i;
            }
        }
        
        if (hit_sphere >= 0) {
            const Sphere& sphere = bodies[hit_sphere];
            Vec3 hit_point = ray.at(closest_t);
            Vec3 normal = (hit_point - sphere.center).normalize();
            
            Vec3 light_dir = (Vec3(0, 0, 0) - hit_point).normalize();
            double diffuse = std::max(0.0, normal.dot(light_dir));
            
            double distance_to_sun = (hit_point - Vec3(0, 0, 0)).length();
            double attenuation = 1.0 / (1.0 + 0.1 * distance_to_sun);
            
            Vec3 color = sphere.color * (diffuse * attenuation + 0.1) + sphere.color * sphere.emission;
            
            if (sphere.emission == 0 && depth < 3) {
                Vec3 reflected = ray.direction - 2 * ray.direction.dot(normal) * normal;
                Ray reflect_ray(hit_point + normal * 0.001, reflected);
                color = color + rayColor(reflect_ray, depth + 1) * 0.3;
            }
            
            return color;
        }
        
        Vec3 unit_direction = ray.direction.normalize();
        double t = 0.5 * (unit_direction.y + 1.0);
        return Vec3(0.05, 0.05, 0.15) * (1.0 - t) + Vec3(0.1, 0.15, 0.3) * t;
    }
    
    void render(int width, int height) {
        std::cout << "\033[2J\033[H";
        
        Vec3 camera_pos(50, 30, 50);
        Vec3 camera_target(0, 0, 0);
        Vec3 camera_up(0, 1, 0);
        
        Vec3 w = (camera_pos - camera_target).normalize();
        Vec3 u = camera_up.dot(w) < 0.99 ? camera_up : Vec3(1, 0, 0);
        u = (u - w * u.dot(w)).normalize();
        Vec3 v = w.cross(u);
        
        double viewport_height = 2.0;
        double viewport_width = viewport_height * width / height;
        Vec3 horizontal = viewport_width * u;
        Vec3 vertical = viewport_height * v;
        Vec3 lower_left = camera_pos - horizontal/2 - vertical/2 - w;
        
        for (int j = height - 1; j >= 0; --j) {
            for (int i = 0; i < width; ++i) {
                double u_coord = double(i) / (width - 1);
                double v_coord = double(j) / (height - 1);
                
                Vec3 direction = lower_left + u_coord * horizontal + v_coord * vertical - camera_pos;
                Ray ray(camera_pos, direction);
                
                Vec3 color = rayColor(ray);
                
                color.x = std::min(1.0, std::max(0.0, color.x));
                color.y = std::min(1.0, std::max(0.0, color.y));
                color.z = std::min(1.0, std::max(0.0, color.z));
                
                int r = int(255 * sqrt(color.x));
                int g = int(255 * sqrt(color.y));
                int b = int(255 * sqrt(color.z));
                
                std::cout << "\033[38;2;" << r << ";" << g << ";" << b << "m";
                
                double intensity = (color.x + color.y + color.z) / 3.0;
                if (intensity > 0.8) std::cout << "@";
                else if (intensity > 0.6) std::cout << "#";
                else if (intensity > 0.4) std::cout << "*";
                else if (intensity > 0.2) std::cout << "+";
                else if (intensity > 0.1) std::cout << ".";
                else std::cout << " ";
            }
            std::cout << "\033[0m\n";
        }
        
        std::cout << "\n\033[33mSolar System Simulator - Time: " << std::fixed << std::setprecision(1) << time << "s\033[0m\n";
        std::cout << "Bodies: " << bodies.size() << " | Controls: Press Ctrl+C to exit\n";
    }
};


int main() {
    std::cout << "\033[2J\033[H";
    std::cout << "=== Advanced Solar System Simulator ===\n";
    std::cout << "Loading...\n";
    
    SolarSystem system;
    
    auto last_time = std::chrono::high_resolution_clock::now();
    
    try {
        while (true) {
            auto current_time = std::chrono::high_resolution_clock::now();
            double dt = std::chrono::duration<double>(current_time - last_time).count();
            last_time = current_time;
            
            system.update(dt * 10);
            system.render(100, 30);
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } catch (...) {
        std::cout << "\033[2J\033[H";
        std::cout << "Simulation ended.\n";
    }
    
    return 0;
}