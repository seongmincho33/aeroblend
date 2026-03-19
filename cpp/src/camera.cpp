#include <aeroblend/camera.h>
#include <cmath>
#include <algorithm>

namespace {

// 세계 방향 벡터를 view*proj로 스크린 좌표에 투영 (소실점 계산).
// 트레이서와 동일한 파이프라인을 사용하여 HUD 지표와 3D 렌더링 일관성 보장.
// w=0 (방향만, 위치 무시)으로 뷰 회전 적용 후 프로젝션.
bool project_direction(const Mat4C& view, const Mat4C& proj,
                       double dx, double dy, double dz,
                       int sw, int sh, float& ox, float& oy) {
    const float* v = view.m;
    float fdx = static_cast<float>(dx);
    float fdy = static_cast<float>(dy);
    float fdz = static_cast<float>(dz);

    // 뷰 회전만 적용 (w=0 → 이동 성분 제외, 3x3 상단-좌측)
    float ex = v[0]*fdx + v[4]*fdy + v[8]*fdz;
    float ey = v[1]*fdx + v[5]*fdy + v[9]*fdz;
    float ez = v[2]*fdx + v[6]*fdy + v[10]*fdz;

    // 프로젝션 적용 (w=0)
    const float* p = proj.m;
    float cx = p[0]*ex + p[4]*ey + p[8]*ez;
    float cy = p[1]*ex + p[5]*ey + p[9]*ez;
    float cw = p[3]*ex + p[7]*ey + p[11]*ez;

    // 카메라 뒤편이면 투영 불가
    if (cw < 0.001f) return false;

    float inv = 1.0f / cw;
    ox = (cx * inv * 0.5f + 0.5f) * static_cast<float>(sw);
    oy = (1.0f - (cy * inv * 0.5f + 0.5f)) * static_cast<float>(sh);
    return true;
}

// 두 Vec3C를 선형 보간
Vec3C lerp_vec3(const Vec3C& a, const Vec3C& b, double t) {
    return {
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t
    };
}

// smoothstep: 부드러운 보간 곡선 (3t² - 2t³)
double smoothstep(double t) {
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

} // namespace

void Camera::init() {
    state_ = CameraStateC{};
    state_.position = {0.0, 5.0, -15.0};
    state_.target = {0.0, 0.0, 0.0};
    state_.up = {0.0, 1.0, 0.0};
    state_.fov_deg = 75.0;
    state_.near = 0.1;
    state_.far = 100000.0;
    state_.mode = 0;
    mode_ = Mode::kChase;
    cam_yaw_ = 0.0;
    cam_pitch_ = 0.0;
    cam_initialized_ = false;
    zoom_ = 1.0;

    // 동적 카메라 상태 초기화
    cam_yaw_vel_ = cam_pitch_vel_ = 0.0;
    cam_pos_x_ = cam_pos_y_ = cam_pos_z_ = 0.0;
    cam_vel_x_ = cam_vel_y_ = cam_vel_z_ = 0.0;
    spring_initialized_ = false;
    smooth_dist_ = 20.0;
    smooth_lateral_ = 0.0;
    smooth_height_ = 3.0;
    smooth_look_ahead_ = 30.0;
    smooth_fov_ = 75.0;

    // 전환 상태 초기화
    transition_active_ = false;
    transition_t_ = 0.0;
    transition_from_ = {};
}

void Camera::update(const AircraftPhysicsStateC& state, double dt,
                    double mouse_target_yaw, double mouse_target_pitch,
                    int scroll_y) {
    // Zoom: scroll up = closer, scroll down = farther
    if (scroll_y != 0) {
        constexpr double kZoomStep = 0.1;
        zoom_ -= scroll_y * kZoomStep;
        constexpr double kZoomMin = 0.3;
        constexpr double kZoomMax = 3.0;
        if (zoom_ < kZoomMin) zoom_ = kZoomMin;
        if (zoom_ > kZoomMax) zoom_ = kZoomMax;
    }

    // dt clamp: 프레임 드랍 시 스프링-댐퍼 발산 방지
    dt = std::min(dt, 0.05);

    // 카메라 전환 스무딩: 전환 진행 중이면 보간 비율 업데이트
    if (transition_active_) {
        transition_t_ += dt / kTransitionDuration;
        if (transition_t_ >= 1.0) {
            transition_t_ = 1.0;
            transition_active_ = false;
        }
    }

    if (mode_ == Mode::kChase) {
        const double* m = state.orientation.m;
        double fwd_x = m[6], fwd_y = m[7], fwd_z = m[8];

        // Initialize camera direction from aircraft forward on first frame
        if (!cam_initialized_) {
            cam_yaw_ = std::atan2(fwd_x, fwd_z);
            cam_pitch_ = std::asin(std::clamp(fwd_y, -1.0, 1.0));
            cam_initialized_ = true;
        }

        // 문서 기준 War Thunder식 Orbit chase:
        // 카메라는 에임 방향을 빠르게 추적하고, 항공기보다 먼저 궤도를 돈다.
        // 이후 항공기가 그 방향으로 수렴하면서 3-body convergence가 형성된다.
        double target_cam_yaw = mouse_target_yaw;
        double target_cam_pitch = mouse_target_pitch;

        // 문서 수식 그대로: k=6.0에 G-force 감속을 적용한 지수 추적.
        constexpr double kCamTrackRate = 6.0;
        double track_rate = kCamTrackRate / (0.5 + 0.5 * std::abs(state.g_force));
        double alpha = 1.0 - std::exp(-track_rate * dt);

        double yaw_err = target_cam_yaw - cam_yaw_;
        while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
        while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
        cam_yaw_ += alpha * yaw_err;
        cam_pitch_ += alpha * (target_cam_pitch - cam_pitch_);

        // Positive yaw should mean "look right" in both the instructor and chase camera.
        double cy = std::cos(cam_yaw_), sy = std::sin(cam_yaw_);
        double cp = std::cos(cam_pitch_), sp = std::sin(cam_pitch_);
        double arm_x = sy * cp;
        double arm_y = sp;
        double arm_z = cy * cp;

        constexpr double kCamDistance = 20.0;
        constexpr double kCamHeight = 2.8;
        constexpr double kTargetHeight = 0.8;
        constexpr double kTargetLead = 2.0;
        double dist = kCamDistance * zoom_;
        state_.position.x = state.position.x - arm_x * dist;
        state_.position.y = state.position.y - arm_y * dist + kCamHeight;
        state_.position.z = state.position.z - arm_z * dist;
        state_.target.x = state.position.x + fwd_x * kTargetLead;
        state_.target.y = state.position.y + kTargetHeight;
        state_.target.z = state.position.z + fwd_z * kTargetLead;
        state_.up = {0.0, 1.0, 0.0};
        state_.fov_deg = 75.0;
        state_.mode = 0;
    } else if (mode_ == Mode::kFPV) {
        // ── FPV 모드: 드론 기체 방향 = 카메라 방향 + 헤드트래킹 ──
        // 콕핏과 달리 기체 틸트를 그대로 반영하여 드론 FPV 느낌 제공.
        const double* m = state.orientation.m;
        double right_x = m[0], right_y = m[1], right_z = m[2];
        double up_x    = m[3], up_y    = m[4], up_z    = m[5];
        double fwd_x   = m[6], fwd_y   = m[7], fwd_z   = m[8];

        // 카메라 위치: 기체 전방 1.0m, 상방 0.5m (드론 카메라 위치)
        constexpr double kFpvFwd = 1.0;
        constexpr double kFpvUp  = 0.5;
        state_.position.x = state.position.x + fwd_x * kFpvFwd + up_x * kFpvUp;
        state_.position.y = state.position.y + fwd_y * kFpvFwd + up_y * kFpvUp;
        state_.position.z = state.position.z + fwd_z * kFpvFwd + up_z * kFpvUp;

        // ── FPV head-tracking 시뮬레이션 ──
        // 마우스 에임 방향으로 카메라를 15% 미세 회전 (DJI FPV 고글 헤드트래킹)
        constexpr double kFpvHeadTrack = 0.15; // 에임 방향 15% 반영
        constexpr double kFpvLookDist = 100.0;

        double ac_yaw = std::atan2(fwd_x, fwd_z);
        double ac_pitch = std::asin(std::clamp(fwd_y, -1.0, 1.0));

        // 에임과 기체 방향의 차이 (yaw wrapping 처리)
        double aim_yaw_diff = mouse_target_yaw - ac_yaw;
        while (aim_yaw_diff > M_PI) aim_yaw_diff -= 2.0 * M_PI;
        while (aim_yaw_diff < -M_PI) aim_yaw_diff += 2.0 * M_PI;
        double aim_pitch_diff = mouse_target_pitch - ac_pitch;

        // 기체 전방 + 15% 에임 오프셋 = 최종 시선 방향
        double look_yaw   = ac_yaw   + aim_yaw_diff   * kFpvHeadTrack;
        double look_pitch = ac_pitch + aim_pitch_diff  * kFpvHeadTrack;

        // 시선 방향 벡터 (월드 좌표)
        double cy = std::cos(look_yaw),  sy = std::sin(look_yaw);
        double cp = std::cos(look_pitch), sp = std::sin(look_pitch);
        double look_x = sy * cp;
        double look_y = sp;
        double look_z = cy * cp;

        state_.target.x = state_.position.x + look_x * kFpvLookDist;
        state_.target.y = state_.position.y + look_y * kFpvLookDist;
        state_.target.z = state_.position.z + look_z * kFpvLookDist;

        // FPV up = 기체 up (틸트 반영, 수평 고정 아님)
        state_.up = {up_x, up_y, up_z};

        state_.fov_deg = 120.0; // DJI FPV 고글 표준 FOV
        state_.near = 0.1;
        state_.far = 100000.0;
        state_.mode = 2;
    } else {
        // Cockpit 모드 (기존 Rust FFI 호출)
        state_ = aeroblend_camera_update_cockpit(&state);
    }

    // ── 카메라 전환 스무딩 ──
    // 모드 전환 시 이전 카메라 상태에서 새 상태로 부드럽게 보간
    if (transition_active_) {
        double t = smoothstep(transition_t_);
        CameraStateC target_state = state_; // 현재 모드가 계산한 최종 상태

        state_.position = lerp_vec3(transition_from_.position, target_state.position, t);
        state_.target   = lerp_vec3(transition_from_.target,   target_state.target,   t);
        // up 벡터도 보간 (FPV ↔ Chase 전환 시 틸트 부드럽게)
        state_.up       = lerp_vec3(transition_from_.up,       target_state.up,       t);
        state_.fov_deg  = transition_from_.fov_deg + (target_state.fov_deg - transition_from_.fov_deg) * t;
    }
}

void Camera::toggle_mode() {
    // 전환 시작: 현재 카메라 상태 저장
    transition_from_ = state_;
    transition_active_ = true;
    transition_t_ = 0.0;

    // 3모드 순환: Chase → Cockpit → FPV → Chase
    switch (mode_) {
        case Mode::kChase:   mode_ = Mode::kCockpit;  break;
        case Mode::kCockpit: mode_ = Mode::kFPV;      break;
        case Mode::kFPV:     mode_ = Mode::kChase;    break;
    }
}

Mat4C Camera::view_matrix() const {
    return aeroblend_build_view_matrix(&state_);
}

Mat4C Camera::projection_matrix(double aspect) const {
    return aeroblend_build_projection_matrix(&state_, aspect);
}

AimDisplay Camera::compute_aim_display(const AircraftPhysicsStateC& aircraft,
                                       double mouse_target_yaw,
                                       double mouse_target_pitch,
                                       const Mat4C& view, const Mat4C& proj,
                                       int screen_w, int screen_h) const {
    AimDisplay result{};

    // 모든 HUD 지표를 트레이서와 동일한 view*proj 파이프라인으로 투영.
    // 이전의 angular 투영(ac_dyaw * px_per_rad)은 카메라가 항공기 정후방이 아닐 때
    // 3D 렌더링 결과(트레이서)와 최대 127px까지 어긋나는 문제가 있었음.

    // Gunsight crosshair (green): 항공기 전방 벡터의 소실점
    const double* m = aircraft.orientation.m;
    double fwd_x = m[6], fwd_y = m[7], fwd_z = m[8];

    result.crosshair_visible = project_direction(
        view, proj, fwd_x, fwd_y, fwd_z,
        screen_w, screen_h, result.crosshair_x, result.crosshair_y);

    // Aim circle (white): 마우스 에임 방향의 소실점
    double cy = std::cos(mouse_target_yaw), sy = std::sin(mouse_target_yaw);
    double cp = std::cos(mouse_target_pitch), sp = std::sin(mouse_target_pitch);
    double aim_dx = sy * cp;
    double aim_dy = sp;
    double aim_dz = cy * cp;

    result.circle_visible = project_direction(
        view, proj, aim_dx, aim_dy, aim_dz,
        screen_w, screen_h, result.circle_x, result.circle_y);

    // Velocity vector (cyan): 비행 경로 방향의 소실점
    double vx = aircraft.velocity.x;
    double vy = aircraft.velocity.y;
    double vz = aircraft.velocity.z;
    double spd = std::sqrt(vx * vx + vy * vy + vz * vz);
    result.speed_ms = static_cast<float>(spd);

    if (spd > 0.5) {
        double inv_spd = 1.0 / spd;
        result.velocity_visible = project_direction(
            view, proj, vx * inv_spd, vy * inv_spd, vz * inv_spd,
            screen_w, screen_h, result.velocity_x, result.velocity_y);
    } else {
        float sw2 = static_cast<float>(screen_w) * 0.5f;
        float sh2 = static_cast<float>(screen_h) * 0.5f;
        result.velocity_x = sw2;
        result.velocity_y = sh2;
        result.velocity_visible = false;
    }

    return result;
}
