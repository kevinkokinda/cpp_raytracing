#ifndef POSTPROCESS_H
#define POSTPROCESS_H

#include "vec3.h"
#include <vector>
#include <algorithm>
#include <cmath>

class Image {
public:
    int width;
    int height;
    std::vector<Vec3> pixels;
    
    Image(int w, int h) : width(w), height(h), pixels(w * h) {}
    
    Vec3& at(int x, int y) {
        return pixels[y * width + x];
    }
    
    const Vec3& at(int x, int y) const {
        return pixels[y * width + x];
    }
    
    Vec3 sample(double u, double v) const {
        int x = static_cast<int>(u * width);
        int y = static_cast<int>(v * height);
        x = std::clamp(x, 0, width - 1);
        y = std::clamp(y, 0, height - 1);
        return at(x, y);
    }
    
    Vec3 sampleBilinear(double u, double v) const {
        double x = u * width - 0.5;
        double y = v * height - 0.5;
        
        int x0 = static_cast<int>(floor(x));
        int y0 = static_cast<int>(floor(y));
        int x1 = x0 + 1;
        int y1 = y0 + 1;
        
        x0 = std::clamp(x0, 0, width - 1);
        x1 = std::clamp(x1, 0, width - 1);
        y0 = std::clamp(y0, 0, height - 1);
        y1 = std::clamp(y1, 0, height - 1);
        
        double fx = x - x0;
        double fy = y - y0;
        
        Vec3 p00 = at(x0, y0);
        Vec3 p10 = at(x1, y0);
        Vec3 p01 = at(x0, y1);
        Vec3 p11 = at(x1, y1);
        
        Vec3 p0 = p00 * (1 - fx) + p10 * fx;
        Vec3 p1 = p01 * (1 - fx) + p11 * fx;
        
        return p0 * (1 - fy) + p1 * fy;
    }
};

class PostProcessor {
public:
    virtual ~PostProcessor() = default;
    virtual void process(Image& image) = 0;
};

class Bloom : public PostProcessor {
private:
    double threshold;
    double intensity;
    int blurPasses;
    
    void extractBright(const Image& src, Image& dst) {
        for (int y = 0; y < src.height; ++y) {
            for (int x = 0; x < src.width; ++x) {
                Vec3 color = src.at(x, y);
                double brightness = (color.x + color.y + color.z) / 3.0;
                
                if (brightness > threshold) {
                    dst.at(x, y) = color * (brightness - threshold);
                } else {
                    dst.at(x, y) = Vec3(0, 0, 0);
                }
            }
        }
    }
    
    void gaussianBlur(Image& img, double sigma) {
        int kernelSize = static_cast<int>(ceil(sigma * 3)) * 2 + 1;
        std::vector<double> kernel(kernelSize);
        
        double sum = 0;
        int half = kernelSize / 2;
        for (int i = 0; i < kernelSize; ++i) {
            double x = i - half;
            kernel[i] = exp(-(x * x) / (2 * sigma * sigma));
            sum += kernel[i];
        }
        
        for (int i = 0; i < kernelSize; ++i) {
            kernel[i] /= sum;
        }
        
        Image temp(img.width, img.height);
        
        for (int y = 0; y < img.height; ++y) {
            for (int x = 0; x < img.width; ++x) {
                Vec3 color(0, 0, 0);
                
                for (int i = 0; i < kernelSize; ++i) {
                    int sx = x + i - half;
                    sx = std::clamp(sx, 0, img.width - 1);
                    color += img.at(sx, y) * kernel[i];
                }
                
                temp.at(x, y) = color;
            }
        }
        
        for (int y = 0; y < img.height; ++y) {
            for (int x = 0; x < img.width; ++x) {
                Vec3 color(0, 0, 0);
                
                for (int i = 0; i < kernelSize; ++i) {
                    int sy = y + i - half;
                    sy = std::clamp(sy, 0, img.height - 1);
                    color += temp.at(x, sy) * kernel[i];
                }
                
                img.at(x, y) = color;
            }
        }
    }
    
public:
    Bloom(double t = 0.8, double i = 0.5, int passes = 3)
        : threshold(t), intensity(i), blurPasses(passes) {}
    
    void process(Image& image) override {
        Image bright(image.width, image.height);
        extractBright(image, bright);
        
        for (int i = 0; i < blurPasses; ++i) {
            gaussianBlur(bright, 2.0 * (i + 1));
        }
        
        for (int y = 0; y < image.height; ++y) {
            for (int x = 0; x < image.width; ++x) {
                image.at(x, y) += bright.at(x, y) * intensity;
            }
        }
    }
};

class MotionBlur : public PostProcessor {
private:
    Vec3 velocity;
    int samples;
    
public:
    MotionBlur(const Vec3& vel, int s = 8) : velocity(vel), samples(s) {}
    
    void process(Image& image) override {
        Image original = image;
        
        for (int y = 0; y < image.height; ++y) {
            for (int x = 0; x < image.width; ++x) {
                Vec3 color(0, 0, 0);
                
                for (int i = 0; i < samples; ++i) {
                    double t = static_cast<double>(i) / samples;
                    
                    double sx = x - velocity.x * t;
                    double sy = y - velocity.y * t;
                    
                    if (sx >= 0 && sx < image.width && sy >= 0 && sy < image.height) {
                        color += original.sampleBilinear(sx / image.width, sy / image.height);
                    }
                }
                
                image.at(x, y) = color / samples;
            }
        }
    }
};

class DepthOfField : public PostProcessor {
private:
    double focalDistance;
    double aperture;
    double maxBlur;
    std::vector<double> depthBuffer;
    
public:
    DepthOfField(double focal, double ap, double blur)
        : focalDistance(focal), aperture(ap), maxBlur(blur) {}
    
    void setDepthBuffer(const std::vector<double>& depths) {
        depthBuffer = depths;
    }
    
    void process(Image& image) override {
        if (depthBuffer.size() != image.pixels.size()) return;
        
        Image original = image;
        
        for (int y = 0; y < image.height; ++y) {
            for (int x = 0; x < image.width; ++x) {
                double depth = depthBuffer[y * image.width + x];
                double coc = abs(depth - focalDistance) * aperture / depth;
                coc = std::min(coc, maxBlur);
                
                if (coc < 0.5) {
                    continue;
                }
                
                Vec3 color(0, 0, 0);
                double weightSum = 0;
                
                int radius = static_cast<int>(coc);
                for (int dy = -radius; dy <= radius; ++dy) {
                    for (int dx = -radius; dx <= radius; ++dx) {
                        if (dx * dx + dy * dy > radius * radius) continue;
                        
                        int sx = x + dx;
                        int sy = y + dy;
                        
                        if (sx >= 0 && sx < image.width && sy >= 0 && sy < image.height) {
                            double weight = 1.0 - sqrt(dx * dx + dy * dy) / radius;
                            color += original.at(sx, sy) * weight;
                            weightSum += weight;
                        }
                    }
                }
                
                if (weightSum > 0) {
                    image.at(x, y) = color / weightSum;
                }
            }
        }
    }
};

class ToneMapping : public PostProcessor {
private:
    double exposure;
    double gamma;
    
public:
    ToneMapping(double exp = 1.0, double g = 2.2)
        : exposure(exp), gamma(g) {}
    
    void process(Image& image) override {
        for (auto& pixel : image.pixels) {
            pixel = pixel * exposure;
            
            pixel.x = pixel.x / (pixel.x + 1.0);
            pixel.y = pixel.y / (pixel.y + 1.0);
            pixel.z = pixel.z / (pixel.z + 1.0);
            
            pixel.x = pow(pixel.x, 1.0 / gamma);
            pixel.y = pow(pixel.y, 1.0 / gamma);
            pixel.z = pow(pixel.z, 1.0 / gamma);
        }
    }
};

class ChromaticAberration : public PostProcessor {
private:
    double strength;
    Vec3 offset;
    
public:
    ChromaticAberration(double s = 0.01, const Vec3& o = Vec3(1, 0, -1))
        : strength(s), offset(o) {}
    
    void process(Image& image) override {
        Image original = image;
        
        for (int y = 0; y < image.height; ++y) {
            for (int x = 0; x < image.width; ++x) {
                double u = static_cast<double>(x) / image.width;
                double v = static_cast<double>(y) / image.height;
                
                double distFromCenter = sqrt((u - 0.5) * (u - 0.5) + (v - 0.5) * (v - 0.5));
                double aberration = strength * distFromCenter;
                
                Vec3 color;
                color.x = original.sampleBilinear(u + offset.x * aberration, v).x;
                color.y = original.sampleBilinear(u + offset.y * aberration, v).y;
                color.z = original.sampleBilinear(u + offset.z * aberration, v).z;
                
                image.at(x, y) = color;
            }
        }
    }
};

class Vignette : public PostProcessor {
private:
    double strength;
    double radius;
    
public:
    Vignette(double s = 0.5, double r = 1.0)
        : strength(s), radius(r) {}
    
    void process(Image& image) override {
        double cx = image.width * 0.5;
        double cy = image.height * 0.5;
        double maxDist = sqrt(cx * cx + cy * cy) * radius;
        
        for (int y = 0; y < image.height; ++y) {
            for (int x = 0; x < image.width; ++x) {
                double dx = x - cx;
                double dy = y - cy;
                double dist = sqrt(dx * dx + dy * dy);
                
                double vignette = 1.0 - (dist / maxDist) * strength;
                vignette = std::max(0.0, vignette);
                
                image.at(x, y) = image.at(x, y) * vignette;
            }
        }
    }
};

class PostProcessingPipeline {
private:
    std::vector<std::unique_ptr<PostProcessor>> processors;
    
public:
    void addProcessor(std::unique_ptr<PostProcessor> processor) {
        processors.push_back(std::move(processor));
    }
    
    void process(Image& image) {
        for (auto& processor : processors) {
            processor->process(image);
        }
    }
    
    void clear() {
        processors.clear();
    }
};

#endif