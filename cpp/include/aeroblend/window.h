#ifndef AEROBLEND_WINDOW_H
#define AEROBLEND_WINDOW_H

#include <SDL.h>
#include <string>

struct WindowConfig {
    std::string title = "AeroBlend";
    int width = 1280;
    int height = 720;
    bool fullscreen = false;
    bool vsync = true;
};

class Window {
public:
    bool init(const WindowConfig& cfg);
    void shutdown();

    void swap();
    bool poll_events(); // returns false if quit requested

    SDL_Window* handle() const { return window_; }
    int width() const { return width_; }
    int height() const { return height_; }
    float aspect() const { return static_cast<float>(width_) / height_; }

    void set_relative_mouse(bool on);
    bool relative_mouse() const { return relative_mouse_; }

private:
    SDL_Window* window_ = nullptr;
    SDL_GLContext gl_ctx_ = nullptr;
    int width_ = 1280;
    int height_ = 720;
    bool relative_mouse_ = false;
};

#endif // AEROBLEND_WINDOW_H
