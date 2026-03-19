#include <aeroblend/ffi.h>
#include <aeroblend/window.h>
#include <aeroblend/renderer.h>
#include <aeroblend/camera.h>
#include <aeroblend/input.h>
#include <aeroblend/hud.h>
#include <aeroblend/projectile_renderer.h>

#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_opengl3.h>

#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

namespace {

constexpr double kPhysicsDt = 1.0 / 120.0; // 120 Hz fixed timestep
constexpr double kMaxFrameDt = 0.1;         // Cap to avoid spiral of death

struct PhysicsEngineDeleter {
    void operator()(PhysicsEngine* p) const {
        if (p != nullptr) { aeroblend_physics_destroy(p); }
    }
};

struct LoadedModelDeleter {
    void operator()(LoadedModel* p) const {
        if (p != nullptr) { aeroblend_model_destroy(p); }
    }
};

struct WeaponSystemDeleter {
    void operator()(WeaponSystemHandle* p) const {
        if (p != nullptr) { aeroblend_weapons_destroy(p); }
    }
};

using PhysicsEnginePtr = std::unique_ptr<PhysicsEngine, PhysicsEngineDeleter>;
using LoadedModelPtr = std::unique_ptr<LoadedModel, LoadedModelDeleter>;
using WeaponSystemPtr = std::unique_ptr<WeaponSystemHandle, WeaponSystemDeleter>;

struct CmdArgs {
    const char* model_path = nullptr;
    bool demo = false;
};

CmdArgs parse_args(int argc, char* argv[]) {
    CmdArgs args;
    for (int i = 1; i < argc; i++) {
        if ((strcmp(argv[i], "--model") == 0) && (i + 1 < argc)) {
            ++i;
            args.model_path = argv[i];
        } else if (strcmp(argv[i], "--demo") == 0) {
            args.demo = true;
        }
    }
    return args;
}

// 데모 모드: 자동 선회 기동 시퀀스
struct DemoSequencer {
    double elapsed = 0.0;

    struct Phase {
        double duration;    // 초
        int mouse_dx;       // 프레임당 마우스 X 이동
        int mouse_dy;       // 프레임당 마우스 Y 이동
        const char* label;
    };

    // 실제 플레이어 패턴: 마우스 짧게 이동 → 멈추고 기체가 따라오길 기다림
    // 90° 선회 = ~1.57rad / 0.005 sens = ~314px, 1초간 5px/frame
    static constexpr int kNumPhases = 9;
    Phase phases[kNumPhases] = {
        { 3.0,   0,  0, "직진 비행" },
        { 1.0,   5,  0, "마우스 우측 (90도 에임)" },
        { 4.0,   0,  0, "기체 우선회 수렴 대기" },
        { 1.0,  -5,  0, "마우스 좌측 (원래 방향)" },
        { 4.0,   0,  0, "기체 직진 수렴 대기" },
        { 1.0,  -5,  0, "마우스 좌측 (90도 에임)" },
        { 4.0,   0,  0, "기체 좌선회 수렴 대기" },
        { 0.5,   0, -3, "마우스 살짝 위" },
        { 4.0,   0,  0, "안정화 → 종료" },
    };

    int current_phase = 0;
    double phase_elapsed = 0.0;
    bool finished = false;

    // 매 프레임 호출: 데모 마우스 입력을 InputHandler에 주입
    void update(double dt, InputHandler& input) {
        if (finished) return;

        elapsed += dt;
        phase_elapsed += dt;

        const Phase& p = phases[current_phase];

        // 마우스 모션 이벤트 주입
        if (p.mouse_dx != 0 || p.mouse_dy != 0) {
            SDL_Event e;
            memset(&e, 0, sizeof(e));
            e.type = SDL_MOUSEMOTION;
            e.motion.xrel = p.mouse_dx;
            e.motion.yrel = p.mouse_dy;
            input.process_event(e);
        }

        // 페이즈 전환
        if (phase_elapsed >= p.duration) {
            printf("[DEMO] Phase %d 완료: \"%s\" (%.1fs)\n",
                   current_phase, p.label, p.duration);
            phase_elapsed = 0.0;
            current_phase++;
            if (current_phase >= kNumPhases) {
                finished = true;
                printf("[DEMO] 데모 시퀀스 완료 (총 %.1fs)\n", elapsed);
            } else {
                printf("[DEMO] Phase %d 시작: \"%s\"\n",
                       current_phase, phases[current_phase].label);
            }
        }
    }

    const char* current_label() const {
        if (finished) return "완료";
        return phases[current_phase].label;
    }
};

void init_imgui(SDL_Window* sdl_window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NoMouseCursorChange;
    ImGui::StyleColorsDark();
    ImGui_ImplSDL2_InitForOpenGL(sdl_window, SDL_GL_GetCurrentContext());
    ImGui_ImplOpenGL3_Init("#version 330");
}

void shutdown_imgui() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
}

AircraftPhysicsStateC load_model_or_default(const char* model_path,
                                             PhysicsEngine* engine,
                                             LoadedModelPtr& model_out,
                                             Renderer& renderer) {
    AircraftPhysicsStateC phys_state{};

    if (model_path != nullptr) {
        printf("Loading model: %s\n", model_path);
        model_out.reset(aeroblend_model_load(model_path));
        if (model_out) {
            phys_state = aeroblend_physics_init(engine, model_out.get());
            renderer.load_aircraft(model_out.get());
            printf("Model loaded successfully. Initial altitude: %.1fm, speed: %.1fm/s\n",
                   phys_state.altitude_m, phys_state.airspeed_ms);
        } else {
            fprintf(stderr, "Failed to load model: %s — using default biplane\n", model_path);
            phys_state = aeroblend_physics_init_default(engine);
        }
    } else {
        printf("No model specified. Use --model <path.glb> to load an aircraft.\n");
        printf("Starting with default An-2 biplane (300m alt, 50 m/s).\n");
        phys_state = aeroblend_physics_init_default(engine);
    }

    return phys_state;
}

void run_game_loop(Window& window,
                   PhysicsEngine* engine,
                   const LoadedModel* model,
                   Renderer& renderer,
                   AircraftPhysicsStateC& phys_state,
                   bool demo_mode = false) {
    Camera camera;
    camera.init();

    InputHandler input;
    input.init();

    HudRenderer hud;
    hud.init();

    // 무장 시스템 초기화 (기본: .50 cal M2 Browning)
    WeaponSystemPtr weapons(aeroblend_weapons_create(BrowningM2));
    WeaponStateC weapon_state{};
    GunPresetC current_preset = BrowningM2;

    // 트레이서 렌더러 초기화
    aeroblend::ProjectileRenderer projectile_renderer;
    projectile_renderer.init("shaders");

    // 더미 비행기 (독립 직진 비행)
    // 플레이어 전방 200m, 우측 30m에 배치 — 동일 자세/속도로 직진
    Vec3C dummy_pos{};
    {
        const auto& o = phys_state.orientation;
        // column 0 = right, column 2 = forward
        double rx = o.m[0], ry = o.m[1], rz = o.m[2];
        double fx = o.m[6], fy = o.m[7], fz = o.m[8];
        dummy_pos.x = phys_state.position.x + fx * 200.0 + rx * 30.0;
        dummy_pos.y = phys_state.position.y + fy * 200.0 + ry * 30.0;
        dummy_pos.z = phys_state.position.z + fz * 200.0 + rz * 30.0;
    }
    Mat3C dummy_orient = phys_state.orientation; // 플레이어와 동일 방향
    constexpr double kDummySpeed = 75.0; // m/s (플레이어 대비 1.5배)

    window.set_relative_mouse(true);

    // 데모 모드 시퀀서
    DemoSequencer demo;
    if (demo_mode) {
        printf("[DEMO] 데모 모드 시작! 자동 선회 기동을 수행합니다.\n");
        printf("[DEMO] Phase 0 시작: \"%s\"\n", demo.phases[0].label);
    }

    Uint64 prev_ticks = SDL_GetPerformanceCounter();
    Uint64 freq = SDL_GetPerformanceFrequency();
    double accumulator = 0.0;
    float fps = 0.0f;
    float fps_timer = 0.0f;
    int32_t frame_count = 0;

    bool running = true;
    while (running) {
        // Timing
        Uint64 now = SDL_GetPerformanceCounter();
        double frame_dt = static_cast<double>(now - prev_ticks) / static_cast<double>(freq);
        prev_ticks = now;
        if (frame_dt > kMaxFrameDt) { frame_dt = kMaxFrameDt; }

        // FPS counter + diagnostic
        fps_timer += static_cast<float>(frame_dt);
        ++frame_count;
        if (fps_timer >= 1.0f) {
            fps = static_cast<float>(frame_count) / fps_timer;
            const ControlStateC& dbg_ctrl = input.controls();
            printf("[%.0ffps] alt=%.0fm(%.0fft) spd=%.1f climb=%.1f AoA=%.1f° "
                   "pitch=%.1f° ctrl(p=%.2f r=%.2f y=%.2f thr=%.0f%%) "
                   "aim_tgt(y=%.1f° p=%.1f°)\n",
                   fps, phys_state.altitude_m, phys_state.altitude_m * 3.28084,
                   phys_state.airspeed_ms,
                   phys_state.climb_rate_ms,
                   phys_state.angle_of_attack_rad * 57.2958,
                   phys_state.pitch_rad * 57.2958,
                   dbg_ctrl.pitch, dbg_ctrl.roll, dbg_ctrl.yaw,
                   dbg_ctrl.throttle * 100.0,
                   input.target_yaw() * 57.2958,
                   input.target_pitch() * 57.2958);
            frame_count = 0;
            fps_timer = 0.0f;
        }

        // Event processing
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            ImGui_ImplSDL2_ProcessEvent(&e);
            // 데모 모드: 실제 마우스/키보드 입력 무시 (ESC만 허용)
            if (demo_mode && (e.type == SDL_MOUSEMOTION ||
                              e.type == SDL_MOUSEBUTTONDOWN ||
                              e.type == SDL_MOUSEBUTTONUP ||
                              e.type == SDL_MOUSEWHEEL ||
                              e.type == SDL_KEYDOWN || e.type == SDL_KEYUP)) {
                // ESC만 통과
                if (e.type == SDL_KEYDOWN && e.key.keysym.scancode == SDL_SCANCODE_ESCAPE) {
                    input.process_event(e);
                }
                continue;
            }
            input.process_event(e);

            if (e.type == SDL_QUIT) { running = false; }
            if ((e.type == SDL_WINDOWEVENT) &&
                (e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED)) {
                int32_t w = 0;
                int32_t h = 0;
                SDL_GL_GetDrawableSize(window.handle(), &w, &h);
                glViewport(0, 0, w, h);
            }
        }

        if (input.quit_requested()) { running = false; }

        // 데모 모드: 자동 마우스 입력 주입
        if (demo_mode && !demo.finished) {
            demo.update(frame_dt, input);
            if (demo.finished) {
                printf("[DEMO] ESC로 종료하세요.\n");
            }
        }

        // Handle toggles
        if (input.toggle_camera()) { camera.toggle_mode(); }
        if (input.toggle_hud()) { hud.toggle(); }
        if (input.toggle_map()) { hud.toggle_map(); }
        input.clear_toggles();

        // Input update (instructor needs aircraft state)
        input.update(frame_dt, phys_state);

        // 무기 전환 (1/2/3 키)
        int ws_switch = input.weapon_switch();
        if (ws_switch >= 0 && ws_switch <= 2) {
            GunPresetC new_preset = static_cast<GunPresetC>(ws_switch);
            if (new_preset != current_preset) {
                current_preset = new_preset;
                weapons.reset(aeroblend_weapons_create(current_preset));
                printf("[weapons] Switched to preset %d\n", ws_switch);
            }
        }

        // 재장전 (R 키)
        if (input.reload_requested()) {
            aeroblend_weapons_reload(weapons.get());
            printf("[weapons] Reloaded\n");
        }

        // Fixed-timestep physics (always runs — engine has default state even without model)
        accumulator += frame_dt;
        while (accumulator >= kPhysicsDt) {
            const ControlStateC& ctrl = input.controls();
            phys_state = aeroblend_physics_step(engine, &ctrl, kPhysicsDt);

            // 무장 시스템 업데이트 (물리 루프 내)
            weapon_state = aeroblend_weapons_update(
                weapons.get(), &phys_state,
                input.is_firing() ? 1 : 0, kPhysicsDt);

            accumulator -= kPhysicsDt;
        }

        // Camera update (tracks its own direction toward mouse aim)
        camera.update(phys_state, frame_dt, input.target_yaw(),
                      input.target_pitch(), input.scroll_y());

        // Render
        int32_t draw_w = 0;
        int32_t draw_h = 0;
        int32_t window_w = 0;
        int32_t window_h = 0;
        SDL_GL_GetDrawableSize(window.handle(), &draw_w, &draw_h);
        SDL_GetWindowSize(window.handle(), &window_w, &window_h);
        glViewport(0, 0, draw_w, draw_h);

        double aspect = (draw_h > 0)
            ? static_cast<double>(draw_w) / static_cast<double>(draw_h)
            : 1.0;
        Mat4C view = camera.view_matrix();
        Mat4C proj = camera.projection_matrix(aspect);

        renderer.render(phys_state, view, proj, camera.state().position);

        // 더미 비행기 업데이트 (독립 직진 비행)
        {
            // forward = orientation column 2
            double fx = dummy_orient.m[6], fy = dummy_orient.m[7], fz = dummy_orient.m[8];
            dummy_pos.x += fx * kDummySpeed * frame_dt;
            dummy_pos.y += fy * kDummySpeed * frame_dt;
            dummy_pos.z += fz * kDummySpeed * frame_dt;
            renderer.render_aircraft_at(dummy_orient, dummy_pos,
                                        view, proj, camera.state().position);
        }

        // 트레이서 렌더링
        ProjectileStateC proj_buffer[512];
        uint32_t proj_count = aeroblend_weapons_get_projectiles(
            weapons.get(), proj_buffer, 512);
        if (proj_count > 0) {
            projectile_renderer.render(proj_buffer, proj_count, view, proj);
        }

        // Compute aim display (project aircraft forward + mouse aim to screen)
        AimDisplay aim = camera.compute_aim_display(
            phys_state, input.target_yaw(), input.target_pitch(),
            view, proj, window_w, window_h);

        // ImGui / HUD
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        hud.render(phys_state, input.controls(), aim, weapon_state, window_w, window_h, fps,
                   camera.state().mode,
                   input.target_yaw(), input.target_pitch(),
                   input.safety_assist_enabled());

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Swap
        window.swap();
    }

    projectile_renderer.cleanup();
    hud.shutdown();
    renderer.shutdown();
}

} // namespace

int main(int argc, char* argv[]) {
    CmdArgs args = parse_args(argc, argv);

    // Window
    Window window;
    WindowConfig wcfg;
    wcfg.title = args.demo ? "AeroBlend - DEMO (자동 선회)" : "AeroBlend Flight Simulator";
    wcfg.width = 1280;
    wcfg.height = 720;
    wcfg.vsync = true;

    int32_t exit_code = 0;

    if (!window.init(wcfg)) {
        exit_code = 1;
    } else {
        init_imgui(window.handle());

        PhysicsEnginePtr engine(aeroblend_physics_create());
        LoadedModelPtr model;

        Renderer renderer;
        if (!renderer.init("shaders")) {
            fprintf(stderr, "Renderer init failed\n");
            exit_code = 1;
        } else {
            AircraftPhysicsStateC phys_state = load_model_or_default(
                args.model_path, engine.get(), model, renderer);

            run_game_loop(window, engine.get(), model.get(),
                          renderer, phys_state, args.demo);
        }

        shutdown_imgui();
        window.shutdown();
    }

    printf("AeroBlend shutdown complete.\n");
    return exit_code;
}
