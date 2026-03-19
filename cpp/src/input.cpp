#include <aeroblend/input.h>
#include <imgui.h>
#include <algorithm>
#include <cmath>

// --- InputSmoother ---

void InputSmoother::update(double dt) {
    if (std::abs(target - current) < 1e-6) {
        current = target;
    } else {
        double rate = (target > current) ? rate_up : rate_down;
        double delta = rate * dt;
        if (target > current) {
            current = std::min(current + delta, target);
        } else {
            current = std::max(current - delta, target);
        }
    }
}

// --- MouseAimComputer (War Thunder-style instructor) ---

void MouseAimComputer::cycle_flight_mode() {
    switch (flight_mode) {
        case FlightMode::kNormal:     flight_mode = FlightMode::kCruise;     break;
        case FlightMode::kCruise:     flight_mode = FlightMode::kAggressive; break;
        case FlightMode::kAggressive: flight_mode = FlightMode::kNormal;     break;
    }
}

void MouseAimComputer::toggle_safety_assist() {
    safety_assist_enabled = !safety_assist_enabled;
}

void MouseAimComputer::configure(const FlightCharacteristicsC& fc) {
    // --- PD gain scheduling based on per-axis control authority (P3-1) ---
    // Goal: normalize responsiveness across airframes using axis-specific ca.
    // Baseline gains tuned at ca_reference (~0.25, typical GA aircraft).
    constexpr double ca_reference = 0.25;

    // Per-axis control authority (P3-1: replaces avg_inertia approach)
    double ca_p = std::clamp(fc.ca_pitch, 0.05, 5.0);
    double ca_r = std::clamp(fc.ca_roll,  0.05, 5.0);
    double ca_y = std::clamp(fc.ca_yaw,   0.05, 5.0);

    // P_scale = (ca_ref/ca)^(1/2)  -- correct, unchanged
    // D_scale = (ca_ref/ca)^(3/4)  -- P1-1: FIXED from sqrt(ca/ca_ref)
    auto p_scale = [&](double ca) { return std::sqrt(ca_reference / ca); };
    auto d_scale = [&](double ca) { return std::pow(ca_reference / ca, 0.75); };

    // Baseline gains (P2-1: Kd base increased 1.2 → 1.52 for ζ=0.7)
    kp_pitch = 2.5 * p_scale(ca_p);  kd_pitch = 1.52 * d_scale(ca_p);
    kp_roll  = 3.0 * p_scale(ca_r);  kd_roll  = 0.8  * d_scale(ca_r);
    kp_yaw   = 0.5 * p_scale(ca_y);  kd_yaw   = 0.4  * d_scale(ca_y);

    // Clamp to safe range
    kp_pitch = std::clamp(kp_pitch, 1.0, 8.0);
    kd_pitch = std::clamp(kd_pitch, 0.3, 5.0);
    kp_roll  = std::clamp(kp_roll,  1.0, 10.0);
    kd_roll  = std::clamp(kd_roll,  0.2, 4.0);
    kp_yaw   = std::clamp(kp_yaw,   0.2, 2.0);
    kd_yaw   = std::clamp(kd_yaw,   0.1, 2.0);

    // --- Stall protection thresholds (P2-2: raised from 70/90 to 85/95) ---
    stall_aoa    = fc.max_aoa_rad * 0.85;  // begin protection at 85% of max
    critical_aoa = fc.max_aoa_rad * 0.95;  // force nose-down at 95% of max

    // --- Speed thresholds from aircraft flight envelope ---
    ref_speed   = fc.ref_speed_ms;
    stall_speed = fc.stall_speed_ms;                   // store for n-adjusted auto-throttle
    min_speed   = fc.stall_speed_ms * 1.05;            // 5% above stall
    auto_throttle_speed = fc.stall_speed_ms * 1.30;    // engage at 130% of stall speed
    critical_speed      = fc.stall_speed_ms * 0.85;    // emergency below stall speed

    configured = true;
}

void MouseAimComputer::update_from_mouse(int dx, int dy) {
    // 초기화 전 마우스 입력 무시 — 시뮬 시작 시 흔들림 방지.
    // initialized는 첫 compute_instructor() 호출에서 항공기 전방 방향으로 설정됨.
    if (!initialized) return;

    // MouseFlight 참고: 고정 감도 3.0 (Unity degrees).
    // SDL 픽셀 단위에서 등가: 0.005 rad/px (원래 값 복원, 가속 제거)
    // 가속 커브 대신 고정 감도 — MouseFlight가 증명한 심플한 접근.
    constexpr double kSensitivity = 0.005;
    // Screen-space fix: the current chase-camera convention already makes
    // target_yaw and on-screen cursor move opposite horizontally, so invert
    // the raw mouse X input here and keep the rest of the control stack intact.
    target_yaw -= dx * kSensitivity;
    target_pitch -= dy * kSensitivity; // negate: mouse up = pitch up
    while (target_yaw > M_PI) target_yaw -= 2.0 * M_PI;
    while (target_yaw < -M_PI) target_yaw += 2.0 * M_PI;
    target_pitch = std::clamp(target_pitch, -1.2, 1.2); // ~70° limit
}

void MouseAimComputer::compute_instructor(const AircraftPhysicsStateC& state,
                                          double& pitch, double& roll, double& yaw) {
    // Auto-configure on first frame if not already done
    if (!configured) {
        configure(state.flight_chars);
    }

    // Aircraft basis vectors from orientation matrix (column-major)
    const double* m = state.orientation.m;
    double right_x = m[0], right_y = m[1], right_z = m[2]; // col 0
    double up_x    = m[3], up_y    = m[4], up_z    = m[5]; // col 1
    double fwd_x   = m[6], fwd_y   = m[7], fwd_z   = m[8]; // col 2

    // Initialize target from current forward direction on first frame.
    // Trim offset (~5°): aircraft needs slight nose-up for level flight.
    if (!initialized) {
        target_yaw = std::atan2(fwd_x, fwd_z);
        target_pitch = std::asin(std::clamp(fwd_y, -1.0, 1.0)) + 0.08;
        initialized = true;
    }

    // War Thunder-style simplified instructor:
    // mouse points to a direction, instructor chooses bank first, then pulls nose in.
    double airspeed = std::max(state.airspeed_ms, 1.0);
    double aoa = state.angle_of_attack_rad;

    double energy_ratio = std::clamp(
        (airspeed - stall_speed * 1.05) / (ref_speed - stall_speed * 1.05 + 1e-6),
        0.0, 1.0);

    double max_pitch_for_speed = 0.18 + 0.72 * energy_ratio;
    switch (flight_mode) {
        case FlightMode::kCruise:     max_pitch_for_speed *= 0.85; break;
        case FlightMode::kAggressive: max_pitch_for_speed *= 1.08; break;
        case FlightMode::kNormal:     break;
    }
    max_pitch_for_speed = std::clamp(max_pitch_for_speed, 0.15, 0.95);
    double target_pitch_limit = safety_assist_enabled ? max_pitch_for_speed : 1.15;
    target_pitch = std::clamp(target_pitch, -1.15, target_pitch_limit);

    // Build target direction vector in world space.
    double cy = std::cos(target_yaw), sy = std::sin(target_yaw);
    double cp = std::cos(target_pitch), sp = std::sin(target_pitch);
    double tx = sy * cp;
    double ty = sp;
    double tz = cy * cp;

    // Transform target direction to aircraft local frame and normalize.
    double lx = tx * right_x + ty * right_y + tz * right_z;
    double ly = tx * up_x    + ty * up_y    + tz * up_z;
    double lz = tx * fwd_x   + ty * fwd_y   + tz * fwd_z;
    double local_len = std::sqrt(lx * lx + ly * ly + lz * lz);
    if (local_len > 1e-6) {
        lx /= local_len;
        ly /= local_len;
        lz /= local_len;
    }

    double yaw_err = std::atan2(lx, std::max(lz, 1e-3));
    double pitch_err = std::atan2(ly, std::sqrt(lx * lx + lz * lz));

    // HUD reticle should show how far the aircraft still needs to converge.
    aim_x = std::clamp(yaw_err / 0.7, -1.0, 1.0);
    aim_y = std::clamp(-pitch_err / 0.7, -1.0, 1.0);

    // Intent space:
    //   pitch positive = nose up
    //   roll/bank positive = right wing down
    //   yaw positive = nose right
    double pitch_rate = -state.angular_velocity.x;
    double yaw_rate = -state.angular_velocity.y;
    double roll_rate = -state.angular_velocity.z;
    double bank_angle = -state.roll_rad;
    double beta = -state.sideslip_rad;

    constexpr double dt_filter = 1.0 / 120.0;
    constexpr double rate_tau = 0.06;
    static const double alpha_rate = 1.0 - std::exp(-dt_filter / rate_tau);
    d_filtered_pitch += alpha_rate * (pitch_rate - d_filtered_pitch);
    d_filtered_roll  += alpha_rate * (roll_rate  - d_filtered_roll);
    d_filtered_yaw   += alpha_rate * (yaw_rate   - d_filtered_yaw);

    double beta_dot = (beta - prev_sideslip) / dt_filter;
    prev_sideslip = beta;

    // Old PID state is intentionally disabled for the new instructor.
    integral_pitch = 0.0;
    integral_roll = 0.0;
    integral_yaw = 0.0;

    double max_bank = 1.22;      // ~70 deg
    double turn_rate_gain = 0.58;
    double bank_kp = 2.6;
    double bank_kd = 0.70;
    double pitch_kp = 2.25;
    double pitch_kd = 0.55;
    double yaw_kp = 0.40;
    double yaw_kd = 0.20;
    double bank_to_rudder = 0.24;

    switch (flight_mode) {
        case FlightMode::kCruise:
            max_bank = 0.95;
            turn_rate_gain = 0.46;
            bank_kp = 2.2;
            pitch_kp = 1.8;
            bank_to_rudder = 0.18;
            break;
        case FlightMode::kAggressive:
            max_bank = 1.40;
            turn_rate_gain = 0.70;
            bank_kp = 3.0;
            pitch_kp = 2.6;
            pitch_kd = 0.50;
            yaw_kp = 0.48;
            bank_to_rudder = 0.28;
            break;
        case FlightMode::kNormal:
            break;
    }

    // Low-energy aircraft should not snap into extreme banks unless safety assist is disabled.
    if (safety_assist_enabled) {
        max_bank *= 0.45 + 0.55 * energy_ratio;
    }

    constexpr double g = 9.81;
    double desired_turn_rate = std::clamp(yaw_err * turn_rate_gain, -0.95, 0.95);
    double desired_bank = std::atan(
        desired_turn_rate * std::max(airspeed, stall_speed * 1.20) / g);
    desired_bank = std::clamp(desired_bank, -max_bank, max_bank);

    // When the target is almost centered, flatten the wings aggressively.
    double center_blend = std::clamp(1.0 - std::abs(yaw_err) / 0.12, 0.0, 1.0);
    desired_bank *= 1.0 - 0.65 * center_blend;

    double bank_err = desired_bank - bank_angle;
    double roll_cmd = bank_kp * bank_err - bank_kd * d_filtered_roll;

    // Avoid excessive pitch-up while the aircraft is still mostly rolling into the turn.
    double lateral_fraction = std::clamp(std::abs(yaw_err) / 0.9, 0.0, 1.0);
    double pitch_cmd = pitch_kp * pitch_err - pitch_kd * d_filtered_pitch;
    if (pitch_cmd > 0.0) {
        pitch_cmd *= 1.0 - 0.35 * lateral_fraction;
    }

    if (safety_assist_enabled) {
        double nose_up_limit = 0.30 + 0.70 * energy_ratio;
        double nose_down_bias = 0.0;
        if (aoa > stall_aoa) {
            double stall_frac = std::clamp(
                (aoa - stall_aoa) / (critical_aoa - stall_aoa + 1e-6),
                0.0, 1.0);
            nose_up_limit *= 1.0 - stall_frac;
            nose_down_bias -= 0.45 * stall_frac;
        }
        if (airspeed < min_speed) {
            double speed_frac = std::clamp(airspeed / std::max(min_speed, 1.0), 0.0, 1.0);
            nose_up_limit *= speed_frac;
            nose_down_bias -= 0.35 * (1.0 - speed_frac);
        }
        if (pitch_cmd > nose_up_limit) {
            pitch_cmd = nose_up_limit;
        }
        pitch_cmd += nose_down_bias;
    }

    // Rudder is used as a helper, not the primary steering axis.
    double yaw_align_weight = 1.0 - 0.65 * std::clamp(std::abs(yaw_err) / 0.5, 0.0, 1.0);
    double yaw_cmd = yaw_align_weight * (yaw_kp * yaw_err - yaw_kd * d_filtered_yaw);
    yaw_cmd += bank_to_rudder * desired_bank;
    yaw_cmd += -1.10 * beta - 0.18 * beta_dot;

    pitch = std::clamp(pitch_cmd, -1.0, 1.0);
    roll = std::clamp(roll_cmd, -1.0, 1.0);
    yaw = std::clamp(yaw_cmd, -1.0, 1.0);
}

// --- InputHandler ---

void InputHandler::init() {
    controls_ = ControlStateC{};
    controls_.gear_down = 1;
    controls_.throttle = 0.5; // Match physics engine initial throttle

    throttle_smooth_.current = 0.5;
    throttle_smooth_.target = 0.5;
    throttle_smooth_.rate_up = 1.5;
    throttle_smooth_.rate_down = 2.0;
    pitch_smooth_.rate_up = 4.0;
    pitch_smooth_.rate_down = 6.0;
    roll_smooth_.rate_up = 4.0;
    roll_smooth_.rate_down = 6.0;
    yaw_smooth_.rate_up = 3.0;
    yaw_smooth_.rate_down = 5.0;
}

void InputHandler::process_event(const SDL_Event& e) {
    if (e.type == SDL_QUIT) {
        quit_ = true;
    }

    bool pressed = (e.type == SDL_KEYDOWN);
    if (e.type == SDL_KEYDOWN || e.type == SDL_KEYUP) {
        // Use scancodes (physical key position) so keys work regardless of
        // input language (Korean, Japanese, etc.)
        switch (e.key.keysym.scancode) {
            case SDL_SCANCODE_W:      key_w_ = pressed; break;
            case SDL_SCANCODE_S:      key_s_ = pressed; break;
            case SDL_SCANCODE_A:      key_a_ = pressed; break;
            case SDL_SCANCODE_D:      key_d_ = pressed; break;
            case SDL_SCANCODE_Q:      key_q_ = pressed; break;
            case SDL_SCANCODE_E:      key_e_ = pressed; break;
            case SDL_SCANCODE_LSHIFT: key_shift_ = pressed; break;
            case SDL_SCANCODE_LCTRL:  key_ctrl_ = pressed; break;
            case SDL_SCANCODE_B:      key_b_ = pressed; break;
            case SDL_SCANCODE_F:      key_f_ = pressed; break;
            case SDL_SCANCODE_G:
                if (pressed && e.key.repeat == 0) controls_.gear_down = controls_.gear_down ? 0 : 1;
                break;
            case SDL_SCANCODE_V:
                if (pressed && e.key.repeat == 0) toggle_cam_ = true;
                break;
            case SDL_SCANCODE_H:
                if (pressed && e.key.repeat == 0) toggle_hud_ = true;
                break;
            case SDL_SCANCODE_M:
                if (pressed && e.key.repeat == 0) toggle_map_ = true;
                break;
            case SDL_SCANCODE_TAB:
                // P4-1: Tab 키로 비행 모드 순환 전환
                if (pressed && e.key.repeat == 0) mouse_aim_.cycle_flight_mode();
                break;
            case SDL_SCANCODE_I:
                if (pressed && e.key.repeat == 0) mouse_aim_.toggle_safety_assist();
                break;
            case SDL_SCANCODE_R:
                if (pressed && e.key.repeat == 0) reload_requested_ = true;
                break;
            case SDL_SCANCODE_1:
                if (pressed && e.key.repeat == 0) weapon_switch_ = 0;
                break;
            case SDL_SCANCODE_2:
                if (pressed && e.key.repeat == 0) weapon_switch_ = 1;
                break;
            case SDL_SCANCODE_3:
                if (pressed && e.key.repeat == 0) weapon_switch_ = 2;
                break;
            case SDL_SCANCODE_ESCAPE:
                quit_ = true;
                break;
            default: break;
        }
    }

    if (e.type == SDL_MOUSEMOTION) {
        mouse_aim_.update_from_mouse(e.motion.xrel, e.motion.yrel);
    }

    if (e.type == SDL_MOUSEWHEEL) {
        scroll_y_ += e.wheel.y;
    }

    // 마우스 버튼: 좌클릭 = 발사 (ImGui가 마우스 캡처 중이면 무시)
    if (e.type == SDL_MOUSEBUTTONDOWN) {
        if (e.button.button == SDL_BUTTON_LEFT) {
            if (!ImGui::GetIO().WantCaptureMouse) {
                firing_ = true;
            }
        }
    }
    if (e.type == SDL_MOUSEBUTTONUP) {
        if (e.button.button == SDL_BUTTON_LEFT) {
            firing_ = false;
        }
    }
}

void InputHandler::update(double dt, const AircraftPhysicsStateC& state) {
    // Throttle: Shift = increase, Ctrl = decrease
    double throttle_target = controls_.throttle;
    if (key_shift_) throttle_target += 1.5 * dt;
    if (key_ctrl_)  throttle_target -= 1.5 * dt;
    throttle_target = std::clamp(throttle_target, 0.0, 1.0);

    // P0: n-adjusted auto-throttle — accounts for increased stall speed in turns
    // V_stall_turn = V_stall * sqrt(n), n = 1/cos(bank)
    const double* m = state.orientation.m;
    double right_y = m[1], up_y = m[4];
    double current_bank_abs = std::abs(std::atan2(-right_y, up_y));
    double n_current = 1.0 / std::max(std::cos(current_bank_abs), 0.1);
    double Vs_n = mouse_aim_.stall_speed * std::sqrt(n_current);
    double at_speed = Vs_n * 1.30;
    double crit_speed = Vs_n * 0.85;
    if (mouse_aim_.safety_assist_enabled && state.airspeed_ms < at_speed) {
        double urgency = 1.0 - std::clamp(
            (state.airspeed_ms - crit_speed) / (at_speed - crit_speed + 1e-6),
            0.0, 1.0);
        double min_throttle = 0.6 + 0.4 * urgency;  // 60% → 100%
        if (throttle_target < min_throttle) {
            throttle_target = min_throttle;
        }
    }

    throttle_smooth_.target = throttle_target;
    throttle_smooth_.update(dt);
    controls_.throttle = throttle_smooth_.current;

    // Mouse aim instructor → pitch/roll/yaw
    double inst_pitch, inst_roll, inst_yaw;
    mouse_aim_.compute_instructor(state, inst_pitch, inst_roll, inst_yaw);

    // Keyboard direct overrides
    double kb_pitch = 0, kb_roll = 0, kb_yaw = 0;
    if (key_w_) kb_pitch = -1.0;  // W = nose down
    if (key_s_) kb_pitch =  1.0;  // S = nose up
    if (key_a_) kb_roll  = -1.0;  // A = roll left
    if (key_d_) kb_roll  =  1.0;  // D = roll right
    if (key_q_) kb_yaw   = -1.0;  // Q = yaw left
    if (key_e_) kb_yaw   =  1.0;  // E = yaw right

    // Combine: keyboard overrides instructor if active
    double final_pitch = (kb_pitch != 0) ? kb_pitch : inst_pitch;
    double final_roll  = (kb_roll  != 0) ? kb_roll  : inst_roll;
    double final_yaw   = (kb_yaw   != 0) ? kb_yaw   : inst_yaw;

    // Physics roll/yaw axes are opposite to the instructor's intent space:
    // negative control = right roll/yaw, positive control = left roll/yaw.
    // Keep the user-facing intent conventional and flip only at the physics boundary.
    double output_pitch = final_pitch;
    double output_roll = -final_roll;
    double output_yaw = -final_yaw;

    // Keyboard uses smoothers for comfortable feel.
    // Instructor PD output goes direct (already smooth from derivative term).
    if (kb_pitch != 0 || kb_roll != 0 || kb_yaw != 0) {
        pitch_smooth_.target = output_pitch;
        roll_smooth_.target  = output_roll;
        yaw_smooth_.target   = output_yaw;
        pitch_smooth_.update(dt);
        roll_smooth_.update(dt);
        yaw_smooth_.update(dt);
        controls_.pitch = pitch_smooth_.current;
        controls_.roll  = roll_smooth_.current;
        controls_.yaw   = yaw_smooth_.current;
    } else {
        controls_.pitch = output_pitch;
        controls_.roll  = output_roll;
        controls_.yaw   = output_yaw;
        // Sync smoothers so keyboard takeover is seamless
        pitch_smooth_.current = output_pitch;
        roll_smooth_.current  = output_roll;
        yaw_smooth_.current   = output_yaw;
    }

    // Brake and flaps
    controls_.brake = key_b_ ? 1.0 : 0.0;
    controls_.flaps = key_f_ ? 1.0 : 0.0;
    controls_.airbrake = key_b_ ? 1 : 0;

    // Aim direction for HUD crosshair
    controls_.mouse_aim_direction = {mouse_aim_.aim_x, mouse_aim_.aim_y, 1.0};

    // 발사 상태를 controls에 반영
    // Note: ControlStateC에 firing 필드가 추가되면 여기서 설정
    // 현재는 is_firing() 메서드로 외부에서 직접 조회

    // Per-frame 이벤트 리셋 (다음 프레임 전에 초기화)
    reload_requested_ = false;
    weapon_switch_ = -1;
}

void InputHandler::clear_toggles() {
    toggle_cam_ = false;
    toggle_hud_ = false;
    toggle_map_ = false;
    scroll_y_ = 0;
}
