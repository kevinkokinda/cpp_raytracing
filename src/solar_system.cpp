#include <iostream>
#include <cmath>
#include <vector>
#include <thread>
#include <chrono>
#include <string>

const int WIDTH = 120;
const int HEIGHT = 40;

struct Object {
    double x, y;
    char symbol;
    std::string color;
};

void clearScreen() {
    std::cout << "\033[2J\033[H";
}

void drawScreen(const std::vector<std::vector<char>>& screen, const std::vector<std::vector<std::string>>& colors) {
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            if (screen[y][x] != ' ') {
                std::cout << colors[y][x] << screen[y][x] << "\033[0m";
            } else {
                std::cout << ' ';
            }
        }
        std::cout << std::endl;
    }
}

void drawCircle(std::vector<std::vector<char>>& screen, std::vector<std::vector<std::string>>& colors, 
                int cx, int cy, int radius, char symbol, const std::string& color) {
    for (int y = -radius; y <= radius; y++) {
        for (int x = -radius; x <= radius; x++) {
            if (x*x + y*y <= radius*radius) {
                int px = cx + x;
                int py = cy + y;
                if (px >= 0 && px < WIDTH && py >= 0 && py < HEIGHT) {
                    screen[py][px] = symbol;
                    colors[py][px] = color;
                }
            }
        }
    }
}

void drawObject(std::vector<std::vector<char>>& screen, std::vector<std::vector<std::string>>& colors,
                int x, int y, char symbol, const std::string& color) {
    if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT) {
        screen[y][x] = symbol;
        colors[y][x] = color;
    }
}

int main() {
    double time = 0;
    
    while (true) {
        std::vector<std::vector<char>> screen(HEIGHT, std::vector<char>(WIDTH, ' '));
        std::vector<std::vector<std::string>> colors(HEIGHT, std::vector<std::string>(WIDTH, ""));
        
        int centerX = WIDTH / 2;
        int centerY = HEIGHT / 2;
        drawCircle(screen, colors, centerX, centerY, 3, '@', "\033[93m");
        
        double planetAngle = time * 0.5;
        int planetOrbitRadius = 15;
        int planetX = centerX + planetOrbitRadius * cos(planetAngle);
        int planetY = centerY + planetOrbitRadius * sin(planetAngle) * 0.5;
        drawCircle(screen, colors, planetX, planetY, 1, 'o', "\033[96m");
        
        double moonAngle = time * 2.0;
        int moonOrbitRadius = 4;
        int moonX = planetX + moonOrbitRadius * cos(moonAngle);
        int moonY = planetY + moonOrbitRadius * sin(moonAngle) * 0.5;
        drawObject(screen, colors, moonX, moonY, '*', "\033[97m");
        
        double planet2Angle = time * 0.3;
        int planet2OrbitRadius = 25;
        int planet2X = centerX + planet2OrbitRadius * cos(planet2Angle);
        int planet2Y = centerY + planet2OrbitRadius * sin(planet2Angle) * 0.5;
        drawCircle(screen, colors, planet2X, planet2Y, 2, 'O', "\033[91m");
        
        clearScreen();
        drawScreen(screen, colors);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        time += 0.1;
    }
    
    return 0;
}