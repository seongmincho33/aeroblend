#include <aeroblend/window.h>
#include <glad/gl.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <cstdio>

bool Window::init(const WindowConfig& cfg) {
    bool success = false;

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
    } else {
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
        SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
        SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
        SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
#ifdef __APPLE__
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG);
#endif

        Uint32 flags = SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI;
        if (cfg.fullscreen) { flags |= SDL_WINDOW_FULLSCREEN_DESKTOP; }

        window_ = SDL_CreateWindow(cfg.title.c_str(),
                                   SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                   cfg.width, cfg.height, flags);
        if (window_ == nullptr) {
            fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        } else {
            gl_ctx_ = SDL_GL_CreateContext(window_);
            if (gl_ctx_ == nullptr) {
                fprintf(stderr, "SDL_GL_CreateContext failed: %s\n", SDL_GetError());
            } else {
                SDL_GL_MakeCurrent(window_, gl_ctx_);

                if (cfg.vsync) {
                    SDL_GL_SetSwapInterval(1);
                } else {
                    SDL_GL_SetSwapInterval(0);
                }

                int version = gladLoadGL(reinterpret_cast<GLADloadfunc>(SDL_GL_GetProcAddress));
                if (version == 0) {
                    fprintf(stderr, "gladLoadGL failed\n");
                } else {
                    printf("OpenGL %d.%d loaded\n",
                           GLAD_VERSION_MAJOR(version), GLAD_VERSION_MINOR(version));

                    SDL_GL_GetDrawableSize(window_, &width_, &height_);

                    glEnable(GL_DEPTH_TEST);
                    glEnable(GL_CULL_FACE);
                    glCullFace(GL_BACK);

                    success = true;
                }
            }
        }
    }
    return success;
}

void Window::shutdown() {
    if (gl_ctx_ != nullptr) {
        SDL_GL_DeleteContext(gl_ctx_);
        gl_ctx_ = nullptr;
    }
    if (window_ != nullptr) {
        SDL_DestroyWindow(window_);
        window_ = nullptr;
    }
    SDL_Quit();
}

void Window::swap() {
    SDL_GL_SwapWindow(window_);
}

bool Window::poll_events() {
    bool keep_running = true;
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
        ImGui_ImplSDL2_ProcessEvent(&e);
        if (e.type == SDL_QUIT) {
            keep_running = false;
        } else if ((e.type == SDL_WINDOWEVENT) &&
                   (e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED)) {
            SDL_GL_GetDrawableSize(window_, &width_, &height_);
            glViewport(0, 0, width_, height_);
        }
    }
    return keep_running;
}

void Window::set_relative_mouse(bool on) {
    if (on) {
        // 커서를 화면 정중앙으로 워프 후 상대 모드 진입 — 초기 흔들림 방지
        int w = 0, h = 0;
        SDL_GetWindowSize(window_, &w, &h);
        SDL_WarpMouseInWindow(window_, w / 2, h / 2);
    }
    SDL_SetRelativeMouseMode(on ? SDL_TRUE : SDL_FALSE);
    // 상대 모드 전환 시 발생하는 가짜 mouse motion 이벤트 제거
    SDL_FlushEvent(SDL_MOUSEMOTION);
    relative_mouse_ = on;
}
