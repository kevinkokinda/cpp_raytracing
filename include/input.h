#ifndef INPUT_H
#define INPUT_H

#include "vec3.h"
#include <unordered_map>
#include <functional>
#include <vector>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class Input {
public:
    enum Key {
        KEY_UNKNOWN = 0,
        KEY_W, KEY_A, KEY_S, KEY_D,
        KEY_Q, KEY_E, KEY_R, KEY_F,
        KEY_SPACE, KEY_ENTER, KEY_ESC,
        KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT,
        KEY_1, KEY_2, KEY_3, KEY_4, KEY_5,
        KEY_6, KEY_7, KEY_8, KEY_9, KEY_0,
        KEY_TAB, KEY_SHIFT, KEY_CTRL, KEY_ALT,
        KEY_P, KEY_O, KEY_I, KEY_U,
        KEY_H, KEY_J, KEY_K, KEY_L,
        KEY_PLUS, KEY_MINUS
    };
    
    struct KeyEvent {
        Key key;
        bool pressed;
        bool shift;
        bool ctrl;
        bool alt;
    };
    
    using KeyCallback = std::function<void(const KeyEvent&)>;
    using MouseCallback = std::function<void(int x, int y, int button)>;
    
private:
    std::unordered_map<Key, bool> keyStates;
    std::unordered_map<Key, bool> previousKeyStates;
    std::vector<KeyCallback> keyCallbacks;
    std::vector<MouseCallback> mouseCallbacks;
    
    struct termios originalTermios;
    bool rawModeEnabled;
    
    int mouseX, mouseY;
    bool mouseButtons[3];
    
    static Input* instance;
    
    void enableRawMode() {
        if (rawModeEnabled) return;
        
        tcgetattr(STDIN_FILENO, &originalTermios);
        
        struct termios raw = originalTermios;
        raw.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
        raw.c_oflag &= ~(OPOST);
        raw.c_cflag |= (CS8);
        raw.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
        
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
        
        rawModeEnabled = true;
    }
    
    void disableRawMode() {
        if (!rawModeEnabled) return;
        
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &originalTermios);
        rawModeEnabled = false;
    }
    
    Key charToKey(char c) {
        switch (c) {
            case 'w': case 'W': return KEY_W;
            case 'a': case 'A': return KEY_A;
            case 's': case 'S': return KEY_S;
            case 'd': case 'D': return KEY_D;
            case 'q': case 'Q': return KEY_Q;
            case 'e': case 'E': return KEY_E;
            case 'r': case 'R': return KEY_R;
            case 'f': case 'F': return KEY_F;
            case 'p': case 'P': return KEY_P;
            case 'o': case 'O': return KEY_O;
            case 'i': case 'I': return KEY_I;
            case 'u': case 'U': return KEY_U;
            case 'h': case 'H': return KEY_H;
            case 'j': case 'J': return KEY_J;
            case 'k': case 'K': return KEY_K;
            case 'l': case 'L': return KEY_L;
            case ' ': return KEY_SPACE;
            case '\n': case '\r': return KEY_ENTER;
            case '\033': return KEY_ESC;
            case '\t': return KEY_TAB;
            case '1': return KEY_1;
            case '2': return KEY_2;
            case '3': return KEY_3;
            case '4': return KEY_4;
            case '5': return KEY_5;
            case '6': return KEY_6;
            case '7': return KEY_7;
            case '8': return KEY_8;
            case '9': return KEY_9;
            case '0': return KEY_0;
            case '+': case '=': return KEY_PLUS;
            case '-': case '_': return KEY_MINUS;
            default: return KEY_UNKNOWN;
        }
    }
    
public:
    Input() : rawModeEnabled(false), mouseX(0), mouseY(0) {
        for (int i = 0; i < 3; ++i) {
            mouseButtons[i] = false;
        }
        instance = this;
    }
    
    ~Input() {
        disableRawMode();
        instance = nullptr;
    }
    
    static Input* getInstance() {
        return instance;
    }
    
    void startCapture() {
        enableRawMode();
    }
    
    void stopCapture() {
        disableRawMode();
    }
    
    void update() {
        previousKeyStates = keyStates;
        
        char buffer[16];
        int bytesRead;
        
        while ((bytesRead = read(STDIN_FILENO, buffer, sizeof(buffer))) > 0) {
            for (int i = 0; i < bytesRead; ++i) {
                if (buffer[i] == '\033' && i + 2 < bytesRead) {
                    if (buffer[i + 1] == '[') {
                        switch (buffer[i + 2]) {
                            case 'A': handleKeyPress(KEY_UP); i += 2; break;
                            case 'B': handleKeyPress(KEY_DOWN); i += 2; break;
                            case 'C': handleKeyPress(KEY_RIGHT); i += 2; break;
                            case 'D': handleKeyPress(KEY_LEFT); i += 2; break;
                        }
                    }
                } else {
                    Key key = charToKey(buffer[i]);
                    if (key != KEY_UNKNOWN) {
                        handleKeyPress(key);
                    }
                }
            }
        }
        
        for (auto& pair : keyStates) {
            if (!pair.second && previousKeyStates[pair.first]) {
                handleKeyRelease(pair.first);
            }
        }
    }
    
    bool isKeyPressed(Key key) const {
        auto it = keyStates.find(key);
        return it != keyStates.end() && it->second;
    }
    
    bool isKeyJustPressed(Key key) const {
        auto it = keyStates.find(key);
        auto prevIt = previousKeyStates.find(key);
        
        bool currentPressed = it != keyStates.end() && it->second;
        bool previousPressed = prevIt != previousKeyStates.end() && prevIt->second;
        
        return currentPressed && !previousPressed;
    }
    
    bool isKeyJustReleased(Key key) const {
        auto it = keyStates.find(key);
        auto prevIt = previousKeyStates.find(key);
        
        bool currentPressed = it != keyStates.end() && it->second;
        bool previousPressed = prevIt != previousKeyStates.end() && prevIt->second;
        
        return !currentPressed && previousPressed;
    }
    
    void addKeyCallback(KeyCallback callback) {
        keyCallbacks.push_back(callback);
    }
    
    void addMouseCallback(MouseCallback callback) {
        mouseCallbacks.push_back(callback);
    }
    
    Vec3 getMousePosition() const {
        return Vec3(mouseX, mouseY, 0);
    }
    
    bool isMouseButtonPressed(int button) const {
        if (button >= 0 && button < 3) {
            return mouseButtons[button];
        }
        return false;
    }
    
private:
    void handleKeyPress(Key key) {
        keyStates[key] = true;
        
        KeyEvent event;
        event.key = key;
        event.pressed = true;
        event.shift = isKeyPressed(KEY_SHIFT);
        event.ctrl = isKeyPressed(KEY_CTRL);
        event.alt = isKeyPressed(KEY_ALT);
        
        for (const auto& callback : keyCallbacks) {
            callback(event);
        }
    }
    
    void handleKeyRelease(Key key) {
        keyStates[key] = false;
        
        KeyEvent event;
        event.key = key;
        event.pressed = false;
        event.shift = isKeyPressed(KEY_SHIFT);
        event.ctrl = isKeyPressed(KEY_CTRL);
        event.alt = isKeyPressed(KEY_ALT);
        
        for (const auto& callback : keyCallbacks) {
            callback(event);
        }
    }
};

Input* Input::instance = nullptr;

class CameraController {
private:
    Vec3 position;
    Vec3 rotation;
    Vec3 velocity;
    
    double moveSpeed;
    double rotateSpeed;
    double friction;
    
    bool firstPerson;
    double orbitDistance;
    Vec3 orbitTarget;
    
    Input* input;
    
public:
    CameraController(Input* in, const Vec3& pos = Vec3(0, 0, 0))
        : position(pos), rotation(0, 0, 0), velocity(0, 0, 0),
          moveSpeed(10.0), rotateSpeed(90.0), friction(0.9),
          firstPerson(false), orbitDistance(10.0), orbitTarget(0, 0, 0),
          input(in) {}
    
    void update(double dt) {
        Vec3 moveDir(0, 0, 0);
        Vec3 rotateDir(0, 0, 0);
        
        if (input->isKeyPressed(Input::KEY_W)) moveDir.z -= 1;
        if (input->isKeyPressed(Input::KEY_S)) moveDir.z += 1;
        if (input->isKeyPressed(Input::KEY_A)) moveDir.x -= 1;
        if (input->isKeyPressed(Input::KEY_D)) moveDir.x += 1;
        if (input->isKeyPressed(Input::KEY_Q)) moveDir.y -= 1;
        if (input->isKeyPressed(Input::KEY_E)) moveDir.y += 1;
        
        if (input->isKeyPressed(Input::KEY_LEFT)) rotateDir.y -= 1;
        if (input->isKeyPressed(Input::KEY_RIGHT)) rotateDir.y += 1;
        if (input->isKeyPressed(Input::KEY_UP)) rotateDir.x -= 1;
        if (input->isKeyPressed(Input::KEY_DOWN)) rotateDir.x += 1;
        
        if (input->isKeyJustPressed(Input::KEY_F)) {
            firstPerson = !firstPerson;
        }
        
        if (input->isKeyPressed(Input::KEY_SHIFT)) {
            moveSpeed = 20.0;
        } else {
            moveSpeed = 10.0;
        }
        
        if (moveDir.lengthSquared() > 0) {
            moveDir.normalize();
            
            double yaw = rotation.y * M_PI / 180.0;
            Vec3 forward(-sin(yaw), 0, -cos(yaw));
            Vec3 right(cos(yaw), 0, -sin(yaw));
            Vec3 up(0, 1, 0);
            
            Vec3 worldMove = forward * moveDir.z + right * moveDir.x + up * moveDir.y;
            velocity += worldMove * moveSpeed * dt;
        }
        
        rotation += rotateDir * rotateSpeed * dt;
        
        rotation.x = std::clamp(rotation.x, -89.0, 89.0);
        
        if (firstPerson) {
            position += velocity * dt;
        } else {
            orbitTarget += velocity * dt;
            
            double pitch = rotation.x * M_PI / 180.0;
            double yaw = rotation.y * M_PI / 180.0;
            
            position.x = orbitTarget.x + orbitDistance * cos(pitch) * sin(yaw);
            position.y = orbitTarget.y + orbitDistance * sin(pitch);
            position.z = orbitTarget.z + orbitDistance * cos(pitch) * cos(yaw);
        }
        
        velocity *= friction;
        
        if (input->isKeyPressed(Input::KEY_PLUS)) {
            orbitDistance = std::max(1.0, orbitDistance - 10.0 * dt);
        }
        if (input->isKeyPressed(Input::KEY_MINUS)) {
            orbitDistance = std::min(100.0, orbitDistance + 10.0 * dt);
        }
    }
    
    Vec3 getPosition() const { return position; }
    Vec3 getRotation() const { return rotation; }
    Vec3 getLookAt() const { return firstPerson ? position + getForward() : orbitTarget; }
    
    Vec3 getForward() const {
        double pitch = rotation.x * M_PI / 180.0;
        double yaw = rotation.y * M_PI / 180.0;
        
        return Vec3(
            -sin(yaw) * cos(pitch),
            sin(pitch),
            -cos(yaw) * cos(pitch)
        );
    }
    
    void setPosition(const Vec3& pos) { position = pos; }
    void setTarget(const Vec3& target) { orbitTarget = target; }
    void setOrbitDistance(double dist) { orbitDistance = dist; }
};

#endif