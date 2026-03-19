#ifndef AEROBLEND_HUD_H
#define AEROBLEND_HUD_H

#include <aeroblend/ffi.h>

struct AimDisplay; // forward declaration (defined in camera.h)

class HudRenderer {
public:
    bool init();
    void render(const AircraftPhysicsStateC& state,
                const ControlStateC& controls,
                const AimDisplay& aim,
                const WeaponStateC& weapon_state,
                int screen_w, int screen_h,
                float fps, int camera_mode,
                double aim_target_yaw, double aim_target_pitch,
                bool safety_assist_enabled);
    void shutdown();

    void set_visible(bool v) { visible_ = v; }
    bool visible() const { return visible_; }
    void toggle() { visible_ = !visible_; }
    void set_map_visible(bool v) { map_visible_ = v; }
    bool map_visible() const { return map_visible_; }
    void toggle_map() { map_visible_ = !map_visible_; }

private:
    bool visible_ = true;
    bool map_visible_ = false;

    void draw_airspeed(float x, float y, float w, float h, double airspeed_ms);
    void draw_altimeter(float x, float y, float w, float h, double alt_m);
    void draw_attitude(float x, float y, float r, double pitch, double roll);
    void draw_heading(float x, float y, float w, float h, double heading_rad);
    void draw_throttle(float x, float y, float w, float h, double throttle);
    void draw_status(float x, float y, const ControlStateC& ctrl, bool safety_assist_enabled);
    void draw_gforce(float x, float y, float r, double g);
    void draw_aim_indicator(const AimDisplay& aim);
    void draw_velocity_arrow(const AimDisplay& aim);
    void draw_fps(float x, float y, float fps);
    void draw_map(int screen_w, int screen_h, const AircraftPhysicsStateC& state);
    void draw_debug(float x, float y, const AircraftPhysicsStateC& state,
                    const ControlStateC& controls,
                    double aim_target_yaw, double aim_target_pitch,
                    bool safety_assist_enabled);
    void draw_weapon(float x, float y, const WeaponStateC& weapon_state);
};

#endif // AEROBLEND_HUD_H
