#ifndef AEROBLEND_CAMERA_H
#define AEROBLEND_CAMERA_H

#include <aeroblend/ffi.h>

/// Screen-space aim indicator data for the HUD.
/// Crosshair = gunsight (aircraft gun line projected on screen).
///   When aircraft converges on mouse aim, crosshair → circle and the
///   connecting line shrinks to zero.
/// Circle = mouse aim direction (where the player wants to go).
/// Velocity = flight path vector (where the aircraft is actually going).
struct AimDisplay {
    float crosshair_x, crosshair_y;
    float circle_x, circle_y;
    float velocity_x, velocity_y;   // flight path vector screen position
    float speed_ms;                  // speed in m/s (for arrow thickness)
    bool crosshair_visible;
    bool circle_visible;
    bool velocity_visible;
};

class Camera {
public:
    void init();
    void update(const AircraftPhysicsStateC& state, double dt,
                double mouse_target_yaw, double mouse_target_pitch,
                int scroll_y);
    void toggle_mode();

    const CameraStateC& state() const { return state_; }
    Mat4C view_matrix() const;
    Mat4C projection_matrix(double aspect) const;

    AimDisplay compute_aim_display(const AircraftPhysicsStateC& aircraft,
                                   double mouse_target_yaw,
                                   double mouse_target_pitch,
                                   const Mat4C& view, const Mat4C& proj,
                                   int screen_w, int screen_h) const;

    // kFPV: 드론 FPV 모드 — 기체 방향 = 카메라 방향 (1인칭, 틸트 포함)
    enum class Mode { kChase, kCockpit, kFPV };
    Mode mode() const { return mode_; }

private:
    CameraStateC state_{};
    Mode mode_ = Mode::kChase;

    // Camera's own tracked look direction (smoothly follows mouse aim,
    // faster than aircraft physics — creates the 3-body convergence effect)
    double cam_yaw_ = 0.0;
    double cam_pitch_ = 0.0;
    double cam_yaw_vel_ = 0.0;    // yaw 스프링-댐퍼 속도
    double cam_pitch_vel_ = 0.0;  // pitch 스프링-댐퍼 속도
    bool cam_initialized_ = false;

    // Zoom (scroll wheel)
    double zoom_ = 1.0; // 1.0 = default, <1 = closer, >1 = farther

    // ── 스프링-댐퍼 위치 추적 (P0-1) ──
    double cam_pos_x_ = 0.0, cam_pos_y_ = 0.0, cam_pos_z_ = 0.0;
    double cam_vel_x_ = 0.0, cam_vel_y_ = 0.0, cam_vel_z_ = 0.0;
    bool spring_initialized_ = false;

    // ── 동적 거리 (P0-2) ──
    double smooth_dist_ = 20.0;

    // ── 측면 오프셋 (P0-3) ──
    double smooth_lateral_ = 0.0;

    // ── 동적 높이 (보완 2) ──
    double smooth_height_ = 3.0;

    // ── look-ahead (P1) ──
    double smooth_look_ahead_ = 30.0;

    // ── 동적 FOV (P2-1) ──
    double smooth_fov_ = 75.0;

    // ── 카메라 전환 스무딩 ──
    // 모드 전환 시 위치/방향을 즉시 점프하지 않고 보간
    bool transition_active_ = false;
    double transition_t_ = 0.0;        // 0.0 = 전환 시작, 1.0 = 완료
    static constexpr double kTransitionDuration = 0.5; // 전환 소요 시간 [s]
    CameraStateC transition_from_{};   // 전환 시작 시점의 카메라 상태
};

#endif // AEROBLEND_CAMERA_H
