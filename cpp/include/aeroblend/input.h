#ifndef AEROBLEND_INPUT_H
#define AEROBLEND_INPUT_H

#include <aeroblend/ffi.h>
#include <SDL.h>

struct InputSmoother {
    double current = 0.0;
    double target = 0.0;
    double rate_up = 3.0;
    double rate_down = 5.0;

    void update(double dt);
};

// P4-1: 비행 모드 시스템 (Normal/Cruise/Aggressive)
enum class FlightMode {
    kNormal,      // 균형 잡힌 기본 비행 (피치-롤 커플링 보정 포함)
    kCruise,      // 고도 안정화 (수평선 기준 피치 혼합)
    kAggressive   // 기동성 강화 (비선형 피치 부스트)
};

struct MouseAimComputer {
    // Target direction in world space (absolute angles, radians)
    double target_yaw = 0.0;
    double target_pitch = 0.0;
    bool initialized = false;
    bool configured = false;

    // Derived from error (for camera offset), range [-1, 1]
    double aim_x = 0.0;
    double aim_y = 0.0;

    // P4-1: 비행 모드 (Tab 키로 순환 전환)
    FlightMode flight_mode = FlightMode::kNormal;
    bool safety_assist_enabled = true;

    // Adaptive gains (set by configure(), defaults match original hardcoded values)
    double kp_pitch = 2.5, kd_pitch = 1.52;
    double kp_roll  = 3.0, kd_roll  = 0.8;
    double kp_yaw   = 0.5, kd_yaw   = 0.4;

    // P4-2: 적분항 게인 (PD → PID 확장)
    double ki_pitch = 0.3;
    double ki_roll  = 0.1;
    double ki_yaw   = 0.15;

    // Adaptive thresholds (set by configure(); defaults for generic biplane)
    double stall_aoa       = 0.28;  // begin stall protection (max_aoa * 0.85)
    double critical_aoa    = 0.36;  // force nose-down (max_aoa * 0.95)
    double ref_speed       = 24.0;  // normal cruise speed
    double min_speed       = 19.2;  // stall danger speed (stall * 1.05)
    double auto_throttle_speed = 23.8; // auto-throttle engagement (stall * 1.30)
    double critical_speed  = 15.6;  // emergency full power (stall * 0.85)
    double stall_speed     = 18.3;  // 1g stall speed (for n-adjusted auto-throttle)

    // D-term low-pass filter state (P2-7: tau=0.01s)
    double d_filtered_pitch = 0.0;
    double d_filtered_roll  = 0.0;
    double d_filtered_yaw   = 0.0;
    static constexpr double d_filter_tau = 0.01; // filter time constant [s]

    // P4-2: 적분기 상태 (축별 적분 누적)
    double integral_pitch = 0.0;
    double integral_roll  = 0.0;
    double integral_yaw   = 0.0;
    static constexpr double integral_limit = 0.2; // 출력의 20%까지 제한

    // Sideslip correction state (P1-4)
    double prev_sideslip = 0.0;

    // Back-calculation anti-windup (P2-3)
    double pitch_up_authority_prev = 1.0;

    /// Configure adaptive parameters from aircraft flight characteristics.
    /// Called once after physics init; derives all gains and thresholds.
    void configure(const FlightCharacteristicsC& fc);

    void update_from_mouse(int dx, int dy);
    void compute_instructor(const AircraftPhysicsStateC& state,
                           double& pitch, double& roll, double& yaw);

    /// P4-1: 비행 모드 순환 전환 (Normal → Cruise → Aggressive → Normal)
    void cycle_flight_mode();
    void toggle_safety_assist();
};

class InputHandler {
public:
    void init();
    void process_event(const SDL_Event& e);
    void update(double dt, const AircraftPhysicsStateC& state);

    const ControlStateC& controls() const { return controls_; }
    bool quit_requested() const { return quit_; }
    bool toggle_camera() const { return toggle_cam_; }
    bool toggle_hud() const { return toggle_hud_; }
    bool toggle_map() const { return toggle_map_; }
    double aim_x() const { return mouse_aim_.aim_x; }
    double aim_y() const { return mouse_aim_.aim_y; }
    double target_yaw() const { return mouse_aim_.target_yaw; }
    double target_pitch() const { return mouse_aim_.target_pitch; }
    int scroll_y() const { return scroll_y_; }

    // 무기 입력 상태
    bool is_firing() const { return firing_; }
    bool reload_requested() const { return reload_requested_; }
    int weapon_switch() const { return weapon_switch_; }

    // P4-1: 비행 모드 조회 (HUD 표시용)
    FlightMode flight_mode() const { return mouse_aim_.flight_mode; }
    bool safety_assist_enabled() const { return mouse_aim_.safety_assist_enabled; }

    void clear_toggles();

private:
    ControlStateC controls_{};
    MouseAimComputer mouse_aim_;
    InputSmoother throttle_smooth_;
    InputSmoother pitch_smooth_;
    InputSmoother roll_smooth_;
    InputSmoother yaw_smooth_;

    bool quit_ = false;
    bool toggle_cam_ = false;
    bool toggle_hud_ = false;
    bool toggle_map_ = false;

    // Key states
    bool key_w_ = false, key_s_ = false;
    bool key_a_ = false, key_d_ = false;
    bool key_q_ = false, key_e_ = false;
    bool key_shift_ = false, key_ctrl_ = false;
    bool key_b_ = false, key_f_ = false;
    bool key_g_ = false;
    int scroll_y_ = 0;

    // 무기 입력 상태
    bool firing_ = false;
    bool reload_requested_ = false;
    int weapon_switch_ = -1;  // -1: 없음, 0/1/2: 무기 슬롯 (1/2/3 키)
};

#endif // AEROBLEND_INPUT_H
