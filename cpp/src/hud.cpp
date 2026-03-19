#include <aeroblend/hud.h>
#include <aeroblend/camera.h> // for AimDisplay definition
#include <imgui.h>
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {
constexpr float kPi = 3.14159265358979f;
constexpr float kRad2Deg = 180.0f / kPi;
constexpr float kMsToKnots = 1.94384f;
constexpr float kMToFt = 3.28084f;
constexpr double kRunwayLength = 3000.0;
constexpr double kRunwayWidth = 45.0;
constexpr double kMapMinRange = 2000.0;
constexpr double kMapMargin = 250.0;
} // namespace

bool HudRenderer::init() {
    return true;
}

void HudRenderer::shutdown() {
}

void HudRenderer::render(const AircraftPhysicsStateC& state,
                         const ControlStateC& controls,
                         const AimDisplay& aim,
                         const WeaponStateC& weapon_state,
                         int sw, int sh, float fps, int camera_mode,
                         double aim_target_yaw, double aim_target_pitch,
                         bool safety_assist_enabled) {
    // Always show camera mode indicator (even when HUD is hidden)
    {
        ImDrawList* dl = ImGui::GetBackgroundDrawList();
        const char* mode_str = (camera_mode == 0) ? "CHASE CAM [V]" : "COCKPIT [V]";
        float sw_f = static_cast<float>(sw);
        dl->AddText(ImVec2(sw_f * 0.5f - 40.0f, 10.0f), IM_COL32(255, 255, 0, 220), mode_str);
    }

    // Aim indicator + velocity arrow always visible (like War Thunder)
    draw_aim_indicator(aim);
    draw_velocity_arrow(aim);
    if (map_visible_) {
        draw_map(sw, sh, state);
    }

    if (visible_) {
        float margin = 10.0f;
        float gauge_w = 120.0f;
        float gauge_h = 200.0f;
        float att_r = 80.0f;
        float sw_f = static_cast<float>(sw);
        float sh_f = static_cast<float>(sh);

        // Left side: Airspeed, Attitude, Altimeter
        draw_airspeed(margin, sh_f * 0.3f, gauge_w, gauge_h, state.airspeed_ms);
        draw_attitude(margin + gauge_w + 20.0f + att_r, sh_f * 0.3f + att_r,
                      att_r, state.pitch_rad, state.roll_rad);
        draw_altimeter(margin + gauge_w + 20.0f + att_r * 2 + 20.0f, sh_f * 0.3f,
                       gauge_w, gauge_h, state.altitude_m);

        // Bottom center: Heading
        draw_heading(sw_f * 0.5f - 100.0f, sh_f - 50.0f, 200.0f, 40.0f, state.heading_rad);

        // Right side: Throttle, G-Force, Status
        draw_throttle(sw_f - margin - 40.0f, sh_f * 0.3f, 30.0f, gauge_h, state.throttle);
        draw_gforce(sw_f - margin - 100.0f, sh_f * 0.3f + 50.0f, 40.0f, state.g_force);
        draw_status(sw_f - margin - 200.0f, sh_f * 0.6f, controls, safety_assist_enabled);

        // 우하단: 무장 상태
        draw_weapon(sw_f - 200.0f, sh_f - 100.0f, weapon_state);

        // Top-left: FPS + Debug
        draw_fps(margin, margin, fps);
        draw_debug(margin, margin + 20.0f, state, controls,
                   aim_target_yaw, aim_target_pitch, safety_assist_enabled);
    }
}

void HudRenderer::draw_airspeed(float x, float y, float w, float h, double airspeed_ms) {
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    float knots = static_cast<float>(airspeed_ms * kMsToKnots);

    ImVec2 tl(x, y), br(x + w, y + h);
    dl->AddRectFilled(tl, br, IM_COL32(0, 0, 0, 140), 4.0f);
    dl->AddRect(tl, br, IM_COL32(0, 255, 0, 200), 4.0f);

    char buf[32];
    snprintf(buf, sizeof(buf), "%.0f", knots);
    dl->AddText(ImVec2(x + 10, y + 5), IM_COL32(0, 255, 0, 255), "SPD kt");
    dl->AddText(ImVec2(x + 10, y + 25), IM_COL32(255, 255, 255, 255), buf);

    // Speed tape
    float tape_top = y + 50;
    float tape_bot = y + h - 10;
    float tape_h = tape_bot - tape_top;
    float center_y = tape_top + tape_h * 0.5f;
    float range = 200.0f; // knots visible range

    for (float spd = 0; spd < 1000; spd += 20) {
        float offset = (spd - knots) / range * tape_h;
        float ty = center_y - offset;
        if (ty < tape_top || ty > tape_bot) continue;
        dl->AddLine(ImVec2(x + w - 30, ty), ImVec2(x + w - 10, ty), IM_COL32(0, 255, 0, 150));
        if (static_cast<int>(spd) % 100 == 0) {
            snprintf(buf, sizeof(buf), "%.0f", spd);
            dl->AddText(ImVec2(x + 5, ty - 7), IM_COL32(0, 255, 0, 200), buf);
        }
    }
    // Center marker
    dl->AddTriangleFilled(ImVec2(x + w - 10, center_y),
                          ImVec2(x + w, center_y - 6),
                          ImVec2(x + w, center_y + 6),
                          IM_COL32(255, 255, 0, 255));
}

void HudRenderer::draw_altimeter(float x, float y, float w, float h, double alt_m) {
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    float ft = static_cast<float>(alt_m * kMToFt);

    ImVec2 tl(x, y), br(x + w, y + h);
    dl->AddRectFilled(tl, br, IM_COL32(0, 0, 0, 140), 4.0f);
    dl->AddRect(tl, br, IM_COL32(0, 255, 0, 200), 4.0f);

    char buf[32];
    snprintf(buf, sizeof(buf), "%.0f", ft);
    dl->AddText(ImVec2(x + 10, y + 5), IM_COL32(0, 255, 0, 255), "ALT ft");
    dl->AddText(ImVec2(x + 10, y + 25), IM_COL32(255, 255, 255, 255), buf);

    // Altitude tape
    float tape_top = y + 50;
    float tape_bot = y + h - 10;
    float tape_h = tape_bot - tape_top;
    float center_y = tape_top + tape_h * 0.5f;
    float range = 2000.0f;

    for (float alt = 0; alt < 50000; alt += 200) {
        float offset = (alt - ft) / range * tape_h;
        float ty = center_y - offset;
        if (ty < tape_top || ty > tape_bot) continue;
        dl->AddLine(ImVec2(x + 10, ty), ImVec2(x + 30, ty), IM_COL32(0, 255, 0, 150));
        if (static_cast<int>(alt) % 1000 == 0) {
            snprintf(buf, sizeof(buf), "%.0f", alt);
            dl->AddText(ImVec2(x + 35, ty - 7), IM_COL32(0, 255, 0, 200), buf);
        }
    }
    dl->AddTriangleFilled(ImVec2(x + 10, center_y),
                          ImVec2(x, center_y - 6),
                          ImVec2(x, center_y + 6),
                          IM_COL32(255, 255, 0, 255));
}

void HudRenderer::draw_attitude(float cx, float cy, float r, double pitch, double roll) {
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    float pitch_deg = static_cast<float>(pitch * kRad2Deg);

    // Background circle
    dl->AddCircleFilled(ImVec2(cx, cy), r, IM_COL32(0, 0, 0, 140), 48);
    dl->AddCircle(ImVec2(cx, cy), r, IM_COL32(0, 255, 0, 200), 48, 1.5f);

    // Horizon line (rotated by roll, shifted by pitch)
    float pitch_px = pitch_deg * (r / 30.0f); // 30 degrees fills the circle
    float sin_r = sinf(static_cast<float>(roll));
    float cos_r = cosf(static_cast<float>(roll));

    ImVec2 left(-r, pitch_px);
    ImVec2 right(r, pitch_px);
    // Rotate by roll
    ImVec2 rl(left.x * cos_r - left.y * sin_r, left.x * sin_r + left.y * cos_r);
    ImVec2 rr(right.x * cos_r - right.y * sin_r, right.x * sin_r + right.y * cos_r);

    dl->AddLine(ImVec2(cx + rl.x, cy + rl.y), ImVec2(cx + rr.x, cy + rr.y),
                IM_COL32(0, 255, 0, 255), 2.0f);

    // Pitch ladder
    for (int deg = -30; deg <= 30; deg += 10) {
        if (deg == 0) continue;
        float py = static_cast<float>(deg) * (r / 30.0f) + pitch_px;
        float lw = (deg % 20 == 0) ? r * 0.4f : r * 0.2f;
        ImVec2 pl(-lw, py), pr(lw, py);
        ImVec2 rpl(pl.x * cos_r - pl.y * sin_r, pl.x * sin_r + pl.y * cos_r);
        ImVec2 rpr(pr.x * cos_r - pr.y * sin_r, pr.x * sin_r + pr.y * cos_r);
        dl->AddLine(ImVec2(cx + rpl.x, cy + rpl.y), ImVec2(cx + rpr.x, cy + rpr.y),
                    IM_COL32(0, 255, 0, 150), 1.0f);
    }

    // Aircraft reference (fixed W shape)
    dl->AddLine(ImVec2(cx - 20, cy), ImVec2(cx - 8, cy + 4), IM_COL32(255, 255, 0, 255), 2.0f);
    dl->AddLine(ImVec2(cx - 8, cy + 4), ImVec2(cx, cy), IM_COL32(255, 255, 0, 255), 2.0f);
    dl->AddLine(ImVec2(cx, cy), ImVec2(cx + 8, cy + 4), IM_COL32(255, 255, 0, 255), 2.0f);
    dl->AddLine(ImVec2(cx + 8, cy + 4), ImVec2(cx + 20, cy), IM_COL32(255, 255, 0, 255), 2.0f);
}

void HudRenderer::draw_heading(float x, float y, float w, float h, double heading_rad) {
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    float hdg = static_cast<float>(heading_rad * kRad2Deg);
    if (hdg < 0) hdg += 360.0f;

    ImVec2 tl(x, y), br(x + w, y + h);
    dl->AddRectFilled(tl, br, IM_COL32(0, 0, 0, 140), 4.0f);
    dl->AddRect(tl, br, IM_COL32(0, 255, 0, 200), 4.0f);

    float cx = x + w * 0.5f;
    char buf[16];

    // Heading tape
    float range = 60.0f;
    for (float h_deg = -180; h_deg <= 540; h_deg += 10) {
        float offset = h_deg - hdg;
        if (offset > 180) offset -= 360;
        if (offset < -180) offset += 360;
        float px = cx + offset / range * w;
        if (px < x || px > x + w) continue;

        dl->AddLine(ImVec2(px, y + h - 15), ImVec2(px, y + h - 5), IM_COL32(0, 255, 0, 150));
        int32_t d = ((static_cast<int32_t>(h_deg) % 360) + 360) % 360;
        const char* label = nullptr;
        if (d == 0) {
            label = "N";
        } else if (d == 90) {
            label = "E";
        } else if (d == 180) {
            label = "S";
        } else if (d == 270) {
            label = "W";
        } else {
            // no cardinal label for this degree
        }

        if (label != nullptr) {
            dl->AddText(ImVec2(px - 4, y + 3), IM_COL32(255, 255, 255, 255), label);
        } else if (d % 30 == 0) {
            snprintf(buf, sizeof(buf), "%d", d);
            dl->AddText(ImVec2(px - 8, y + 3), IM_COL32(0, 255, 0, 200), buf);
        } else {
            // no label for this tick
        }
    }

    // Center marker
    dl->AddTriangleFilled(ImVec2(cx, y + h), ImVec2(cx - 5, y + h + 8),
                          ImVec2(cx + 5, y + h + 8), IM_COL32(255, 255, 0, 255));

    snprintf(buf, sizeof(buf), "%.0f", hdg);
    dl->AddText(ImVec2(cx - 10, y + h - 32), IM_COL32(255, 255, 255, 255), buf);
}

void HudRenderer::draw_throttle(float x, float y, float w, float h, double throttle) {
    ImDrawList* dl = ImGui::GetBackgroundDrawList();

    ImVec2 tl(x, y), br(x + w, y + h);
    dl->AddRectFilled(tl, br, IM_COL32(0, 0, 0, 140), 4.0f);
    dl->AddRect(tl, br, IM_COL32(0, 255, 0, 200), 4.0f);

    float fill_h = static_cast<float>(throttle * h);
    ImU32 col = (throttle > 0.9) ? IM_COL32(255, 100, 0, 200) : IM_COL32(0, 200, 0, 200);
    dl->AddRectFilled(ImVec2(x + 2, y + h - fill_h), ImVec2(x + w - 2, y + h - 2), col);

    char buf[16];
    snprintf(buf, sizeof(buf), "%.0f%%", throttle * 100);
    dl->AddText(ImVec2(x - 30, y + h + 5), IM_COL32(0, 255, 0, 255), "THR");
    dl->AddText(ImVec2(x - 30, y + h + 20), IM_COL32(255, 255, 255, 255), buf);
}

void HudRenderer::draw_status(float x, float y, const ControlStateC& ctrl,
                              bool safety_assist_enabled) {
    ImDrawList* dl = ImGui::GetBackgroundDrawList();

    auto indicator = [&](float yy, const char* label, bool active) {
        ImU32 col = active ? IM_COL32(0, 255, 0, 255) : IM_COL32(80, 80, 80, 200);
        dl->AddRectFilled(ImVec2(x, yy), ImVec2(x + 60, yy + 20), IM_COL32(0, 0, 0, 140), 2.0f);
        dl->AddRect(ImVec2(x, yy), ImVec2(x + 60, yy + 20), col, 2.0f);
        dl->AddText(ImVec2(x + 5, yy + 3), col, label);
    };

    indicator(y, "GEAR", ctrl.gear_down != 0);
    indicator(y + 25, "FLAP", ctrl.flaps > 0.5);
    indicator(y + 50, "BRK", ctrl.brake > 0.5);
    indicator(y + 75, "ABRK", ctrl.airbrake != 0);
    indicator(y + 100, "SAFE", safety_assist_enabled);
}

void HudRenderer::draw_gforce(float x, float y, float r, double g) {
    ImDrawList* dl = ImGui::GetBackgroundDrawList();

    dl->AddCircleFilled(ImVec2(x, y), r, IM_COL32(0, 0, 0, 140), 32);
    dl->AddCircle(ImVec2(x, y), r, IM_COL32(0, 255, 0, 200), 32, 1.0f);

    char buf[16];
    snprintf(buf, sizeof(buf), "%.1fG", g);
    ImU32 col = IM_COL32(0, 255, 0, 255);
    if ((g > 6.0) || (g < -2.0)) {
        col = IM_COL32(255, 0, 0, 255);
    } else if ((g > 4.0) || (g < -1.0)) {
        col = IM_COL32(255, 255, 0, 255);
    } else {
        // normal range, keep green
    }

    dl->AddText(ImVec2(x - 15, y - 7), col, buf);
}

void HudRenderer::draw_aim_indicator(const AimDisplay& aim) {
    ImDrawList* dl = ImGui::GetBackgroundDrawList();

    // Gunsight crosshair (green +): where guns would fire.
    // Converges toward the white aim circle as the aircraft tracks the mouse.
    if (aim.crosshair_visible) {
        ImU32 col = IM_COL32(0, 255, 0, 220);
        float cx = aim.crosshair_x;
        float cy = aim.crosshair_y;
        float s = 12.0f;
        float g = 4.0f;
        dl->AddLine(ImVec2(cx - s, cy), ImVec2(cx - g, cy), col, 1.5f);
        dl->AddLine(ImVec2(cx + g, cy), ImVec2(cx + s, cy), col, 1.5f);
        dl->AddLine(ImVec2(cx, cy - s), ImVec2(cx, cy - g), col, 1.5f);
        dl->AddLine(ImVec2(cx, cy + g), ImVec2(cx, cy + s), col, 1.5f);
        dl->AddCircle(ImVec2(cx, cy), 2.0f, col, 12, 1.5f);
    }

    // Mouse aim circle (white): where the player wants to go
    if (aim.circle_visible) {
        ImU32 col = IM_COL32(255, 255, 255, 200);
        float cx = aim.circle_x;
        float cy = aim.circle_y;
        float r = 20.0f;
        dl->AddCircle(ImVec2(cx, cy), r, col, 32, 1.5f);
    }

    // Convergence line: gunsight → aim circle. Shrinks to zero when aircraft
    // is on target. Length = visual feedback of tracking error.
    if (aim.crosshair_visible && aim.circle_visible) {
        ImU32 col = IM_COL32(255, 255, 255, 100);
        dl->AddLine(ImVec2(aim.crosshair_x, aim.crosshair_y),
                    ImVec2(aim.circle_x, aim.circle_y), col, 1.0f);
    }
}

void HudRenderer::draw_velocity_arrow(const AimDisplay& aim) {
    if (!aim.velocity_visible) return;

    ImDrawList* dl = ImGui::GetBackgroundDrawList();

    // Arrow from crosshair (aircraft nose) to velocity vector (flight path)
    float from_x = aim.crosshair_visible ? aim.crosshair_x : aim.velocity_x;
    float from_y = aim.crosshair_visible ? aim.crosshair_y : aim.velocity_y;
    float to_x = aim.velocity_x;
    float to_y = aim.velocity_y;

    float arrow_dx = to_x - from_x;
    float arrow_dy = to_y - from_y;
    float arrow_len = std::sqrt(arrow_dx * arrow_dx + arrow_dy * arrow_dy);

    // Thickness: scales with speed (1.5 at 0 m/s, up to 5.0 at 200+ m/s)
    float thickness = 1.5f + std::min(aim.speed_ms / 60.0f, 3.5f);

    // Color: cyan, brighter at higher speeds
    int alpha = 150 + static_cast<int>(std::min(aim.speed_ms / 200.0f, 1.0f) * 105);
    ImU32 col = IM_COL32(0, 220, 255, alpha);

    // Flight path marker: small circle with horizontal tails at velocity position
    float marker_r = 5.0f + std::min(aim.speed_ms / 100.0f, 3.0f);
    dl->AddCircle(ImVec2(to_x, to_y), marker_r, col, 16, thickness);
    dl->AddLine(ImVec2(to_x - marker_r - 8, to_y),
                ImVec2(to_x - marker_r, to_y), col, thickness);
    dl->AddLine(ImVec2(to_x + marker_r, to_y),
                ImVec2(to_x + marker_r + 8, to_y), col, thickness);
    dl->AddLine(ImVec2(to_x, to_y + marker_r),
                ImVec2(to_x, to_y + marker_r + 6), col, thickness);

    // Arrow line from crosshair to flight path vector (if they're separated enough)
    if (arrow_len > 15.0f) {
        float nx = arrow_dx / arrow_len;
        float ny = arrow_dy / arrow_len;

        // Shorten to not overlap markers
        float start_x = from_x + nx * 14.0f;
        float start_y = from_y + ny * 14.0f;
        float end_x = to_x - nx * (marker_r + 10.0f);
        float end_y = to_y - ny * (marker_r + 10.0f);

        // Only draw if there's still space
        float seg_dx = end_x - start_x;
        float seg_dy = end_y - start_y;
        if (seg_dx * nx + seg_dy * ny > 0) {
            dl->AddLine(ImVec2(start_x, start_y), ImVec2(end_x, end_y),
                        col, thickness * 0.6f);

            // Arrowhead
            float head_len = 8.0f;
            float head_w = 4.0f;
            float px = -ny; // perpendicular
            float py = nx;
            dl->AddTriangleFilled(
                ImVec2(end_x, end_y),
                ImVec2(end_x - nx * head_len + px * head_w,
                       end_y - ny * head_len + py * head_w),
                ImVec2(end_x - nx * head_len - px * head_w,
                       end_y - ny * head_len - py * head_w),
                col);
        }
    }

    // Speed text near the marker
    char buf[32];
    snprintf(buf, sizeof(buf), "%.0f kt", aim.speed_ms * 1.94384f);
    dl->AddText(ImVec2(to_x + marker_r + 12, to_y - 7), col, buf);
}

void HudRenderer::draw_fps(float x, float y, float fps) {
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    char buf[32];
    snprintf(buf, sizeof(buf), "FPS: %.0f", fps);
    dl->AddText(ImVec2(x, y), IM_COL32(255, 255, 255, 200), buf);
}

void HudRenderer::draw_map(int screen_w, int screen_h, const AircraftPhysicsStateC& state) {
    ImDrawList* dl = ImGui::GetBackgroundDrawList();

    const float size = 220.0f;
    const float margin = 18.0f;
    const float x0 = static_cast<float>(screen_w) - size - margin;
    const float y0 = margin + 28.0f;
    const float x1 = x0 + size;
    const float y1 = y0 + size;
    const float center_x = x0 + size * 0.5f;
    const float center_y = y0 + size * 0.5f;

    double extent = std::max({kMapMinRange,
                              std::abs(state.position.x) + kMapMargin,
                              std::abs(state.position.z) + kMapMargin,
                              kRunwayLength * 0.5 + kMapMargin});
    float pixels_per_meter = (size * 0.5f - 16.0f) / static_cast<float>(extent);

    auto map_x = [&](double world_x) {
        return center_x + static_cast<float>(world_x) * pixels_per_meter;
    };
    auto map_y = [&](double world_z) {
        return center_y - static_cast<float>(world_z) * pixels_per_meter;
    };

    dl->AddRectFilled(ImVec2(x0, y0), ImVec2(x1, y1), IM_COL32(8, 12, 18, 210), 6.0f);
    dl->AddRect(ImVec2(x0, y0), ImVec2(x1, y1), IM_COL32(80, 180, 255, 180), 6.0f, 0, 1.5f);
    dl->AddText(ImVec2(x0 + 10.0f, y0 + 8.0f), IM_COL32(255, 255, 255, 220), "MAP [M]");

    for (int i = 1; i < 4; i++) {
        float t = static_cast<float>(i) / 4.0f;
        float gx = x0 + t * size;
        float gy = y0 + t * size;
        dl->AddLine(ImVec2(gx, y0 + 26.0f), ImVec2(gx, y1 - 8.0f), IM_COL32(80, 110, 130, 60), 1.0f);
        dl->AddLine(ImVec2(x0 + 8.0f, gy), ImVec2(x1 - 8.0f, gy), IM_COL32(80, 110, 130, 60), 1.0f);
    }

    dl->AddLine(ImVec2(center_x, y0 + 26.0f), ImVec2(center_x, y1 - 8.0f),
                IM_COL32(120, 160, 180, 100), 1.0f);
    dl->AddLine(ImVec2(x0 + 8.0f, center_y), ImVec2(x1 - 8.0f, center_y),
                IM_COL32(120, 160, 180, 100), 1.0f);

    float runway_half_w = kRunwayWidth * 0.5f * pixels_per_meter;
    float runway_half_l = kRunwayLength * 0.5f * pixels_per_meter;
    ImVec2 runway_tl(center_x - runway_half_w, center_y - runway_half_l);
    ImVec2 runway_br(center_x + runway_half_w, center_y + runway_half_l);
    dl->AddRectFilled(runway_tl, runway_br, IM_COL32(70, 72, 78, 255), 2.0f);
    dl->AddRect(runway_tl, runway_br, IM_COL32(220, 220, 220, 220), 2.0f, 0, 1.0f);
    dl->AddLine(ImVec2(center_x, runway_tl.y + 8.0f), ImVec2(center_x, runway_br.y - 8.0f),
                IM_COL32(245, 245, 245, 180), 1.0f);
    dl->AddText(ImVec2(center_x + 8.0f, center_y - 8.0f), IM_COL32(255, 255, 255, 200), "RWY");

    float aircraft_x = map_x(state.position.x);
    float aircraft_y = map_y(state.position.z);
    float dir_x = std::sin(static_cast<float>(state.heading_rad));
    float dir_y = -std::cos(static_cast<float>(state.heading_rad));
    float perp_x = -dir_y;
    float perp_y = dir_x;
    const float nose = 10.0f;
    const float wing = 6.0f;

    ImVec2 tip(aircraft_x + dir_x * nose, aircraft_y + dir_y * nose);
    ImVec2 left(aircraft_x - dir_x * 5.0f + perp_x * wing,
                aircraft_y - dir_y * 5.0f + perp_y * wing);
    ImVec2 right(aircraft_x - dir_x * 5.0f - perp_x * wing,
                 aircraft_y - dir_y * 5.0f - perp_y * wing);
    dl->AddTriangleFilled(tip, left, right, IM_COL32(255, 200, 70, 240));
    dl->AddTriangle(tip, left, right, IM_COL32(20, 20, 20, 220), 1.0f);

    char buf[96];
    double runway_dist = std::sqrt(state.position.x * state.position.x +
                                   state.position.z * state.position.z);
    snprintf(buf, sizeof(buf), "AC %.0fm, %.0fm  DME %.0fm",
             state.position.x, state.position.z, runway_dist);
    dl->AddText(ImVec2(x0 + 10.0f, y1 - 20.0f), IM_COL32(255, 220, 120, 220), buf);
}

void HudRenderer::draw_debug(float x, float y, const AircraftPhysicsStateC& state,
                             const ControlStateC& controls,
                             double aim_target_yaw, double aim_target_pitch,
                             bool safety_assist_enabled) {
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    ImU32 col = IM_COL32(200, 200, 200, 180);
    ImU32 col_warn = IM_COL32(255, 200, 100, 220);
    char buf[128];
    float line = 15.0f;
    float cy = y;

    snprintf(buf, sizeof(buf), "POS: %.0f, %.0f, %.0f",
             state.position.x, state.position.y, state.position.z);
    dl->AddText(ImVec2(x, cy), col, buf);
    cy += line;

    snprintf(buf, sizeof(buf), "VEL: %.1f, %.1f, %.1f (%.1f m/s)",
             state.velocity.x, state.velocity.y, state.velocity.z, state.airspeed_ms);
    dl->AddText(ImVec2(x, cy), col, buf);
    cy += line;

    snprintf(buf, sizeof(buf), "AoA: %.1f deg  Climb: %.1f m/s",
             state.angle_of_attack_rad * kRad2Deg, state.climb_rate_ms);
    dl->AddText(ImVec2(x, cy), (state.climb_rate_ms < -5.0) ? col_warn : col, buf);
    cy += line;

    snprintf(buf, sizeof(buf), "Pitch: %.1f  Roll: %.1f  Hdg: %.0f",
             state.pitch_rad * kRad2Deg,
             state.roll_rad * kRad2Deg,
             state.heading_rad * kRad2Deg);
    dl->AddText(ImVec2(x, cy), col, buf);
    cy += line;

    // Control inputs (what physics engine receives)
    snprintf(buf, sizeof(buf), "CTRL  P:%.2f R:%.2f Y:%.2f Thr:%.0f%%",
             controls.pitch, controls.roll, controls.yaw,
             controls.throttle * 100.0);
    dl->AddText(ImVec2(x, cy), IM_COL32(100, 255, 100, 200), buf);
    cy += line;

    // Aim target (where the mouse instructor is pointing)
    snprintf(buf, sizeof(buf), "AIM   Yaw:%.1f  Pitch:%.1f deg",
             aim_target_yaw * kRad2Deg,
             aim_target_pitch * kRad2Deg);
    dl->AddText(ImVec2(x, cy), IM_COL32(255, 255, 100, 200), buf);
    cy += line;

    snprintf(buf, sizeof(buf), "SAFE  %s [I]", safety_assist_enabled ? "ON" : "OFF");
    dl->AddText(ImVec2(x, cy),
                safety_assist_enabled ? IM_COL32(100, 255, 100, 200)
                                      : IM_COL32(255, 180, 100, 220),
                buf);
    cy += line;
}

void HudRenderer::draw_weapon(float x, float y, const WeaponStateC& ws) {
    ImDrawList* dl = ImGui::GetBackgroundDrawList();

    // 기총 프리셋 이름
    const char* gun_name = "UNKNOWN";
    switch (ws.gun_preset) {
        case BrowningM2: gun_name = ".50 CAL M2"; break;
        case AnM2Cannon: gun_name = "20MM AN/M2"; break;
        case Vickers77: gun_name = "7.7MM VICKERS"; break;
    }

    // 잔탄 비율 계산
    float ammo_ratio = (ws.total_ammo_capacity > 0)
        ? static_cast<float>(ws.total_ammo_remaining) / static_cast<float>(ws.total_ammo_capacity)
        : 0.0f;

    // 색상: 녹색 → 노란색 → 빨간색
    ImU32 ammo_col;
    if (ammo_ratio > 0.5f) {
        ammo_col = IM_COL32(0, 255, 0, 255);
    } else if (ammo_ratio > 0.2f) {
        ammo_col = IM_COL32(255, 255, 0, 255);
    } else {
        ammo_col = IM_COL32(255, 50, 50, 255);
    }

    // 배경 박스
    float w = 190.0f;
    float h = 85.0f;
    dl->AddRectFilled(ImVec2(x, y), ImVec2(x + w, y + h), IM_COL32(0, 0, 0, 140), 4.0f);
    dl->AddRect(ImVec2(x, y), ImVec2(x + w, y + h), IM_COL32(0, 255, 0, 200), 4.0f);

    float cx = x + 8.0f;
    float cy = y + 5.0f;
    float line = 17.0f;

    // 기총 이름
    dl->AddText(ImVec2(cx, cy), IM_COL32(200, 200, 200, 255), gun_name);
    cy += line;

    // 잔탄수
    char buf[64];
    snprintf(buf, sizeof(buf), "AMMO %u / %u", ws.total_ammo_remaining, ws.total_ammo_capacity);
    dl->AddText(ImVec2(cx, cy), ammo_col, buf);
    cy += line;

    // 발사 중 표시
    if (ws.is_firing) {
        dl->AddText(ImVec2(cx, cy), IM_COL32(255, 128, 0, 255), ">>> FIRING <<<");
    } else if (ws.total_ammo_remaining == 0) {
        dl->AddText(ImVec2(cx, cy), IM_COL32(255, 0, 0, 255), "AMMO EMPTY");
    } else if (ammo_ratio < 0.2f) {
        dl->AddText(ImVec2(cx, cy), IM_COL32(255, 0, 0, 255), "! AMMO LOW !");
    }
}
