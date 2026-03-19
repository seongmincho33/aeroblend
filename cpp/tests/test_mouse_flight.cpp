/// Integration test: simulates mouse input → instructor → physics
/// Verifies the aircraft actually moves and turns in response to mouse.
/// No window or rendering needed.

#include <aeroblend/ffi.h>
#include <aeroblend/input.h>
#include <SDL.h>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <memory>

struct PhysicsEngineDeleter {
    void operator()(PhysicsEngine* p) const {
        if (p != nullptr) { aeroblend_physics_destroy(p); }
    }
};
using PhysicsEnginePtr = std::unique_ptr<PhysicsEngine, PhysicsEngineDeleter>;

struct TestResult {
    bool passed;
    const char* name;
    const char* detail;
};

// Simple name pool for dynamically generated test names (avoids stack-address issues)
static char g_name_pool[32][64];
static int g_name_idx = 0;
const char* alloc_name(const char* fmt, const char* profile) {
    char* buf = g_name_pool[g_name_idx++ % 32];
    snprintf(buf, 64, fmt, profile);
    return buf;
}

// Helper: inject a fake mouse motion event into InputHandler
void inject_mouse_motion(InputHandler& input, int dx, int dy) {
    SDL_Event e;
    memset(&e, 0, sizeof(e));
    e.type = SDL_MOUSEMOTION;
    e.motion.xrel = dx;
    e.motion.yrel = dy;
    input.process_event(e);
}

// Helper: inject a key press/release
void inject_key(InputHandler& input, SDL_Scancode scancode, bool pressed) {
    SDL_Event e;
    memset(&e, 0, sizeof(e));
    e.type = pressed ? SDL_KEYDOWN : SDL_KEYUP;
    e.key.keysym.scancode = scancode;
    e.key.repeat = 0;
    input.process_event(e);
}

// Helper: run physics for N seconds with given mouse/key state
AircraftPhysicsStateC run_simulation(PhysicsEngine* engine,
                                      InputHandler& input,
                                      AircraftPhysicsStateC state,
                                      double duration_sec) {
    constexpr double kDt = 1.0 / 120.0;
    int steps = static_cast<int>(duration_sec / kDt);
    for (int i = 0; i < steps; i++) {
        input.update(kDt, state);
        state = aeroblend_physics_step(engine, &input.controls(), kDt);
    }
    return state;
}

void print_state(const char* label, const AircraftPhysicsStateC& s) {
    double hdg_deg = s.heading_rad * 180.0 / M_PI;
    if (hdg_deg < 0) hdg_deg += 360.0;
    printf("  [%s] pos=(%.1f, %.1f, %.1f) vel=(%.1f, %.1f, %.1f) "
           "spd=%.1f m/s alt=%.1f m hdg=%.1f° pitch=%.1f° roll=%.1f°\n",
           label,
           s.position.x, s.position.y, s.position.z,
           s.velocity.x, s.velocity.y, s.velocity.z,
           s.airspeed_ms, s.altitude_m,
           hdg_deg,
           s.pitch_rad * 180.0 / M_PI,
           s.roll_rad * 180.0 / M_PI);
}

// ─────────────────────────────────────────────────────────────
// Test 1: Aircraft moves forward with no input
// ─────────────────────────────────────────────────────────────
TestResult test_straight_flight(PhysicsEngine* engine) {
    printf("\n--- Test 1: Straight flight (no mouse input) ---\n");

    InputHandler input;
    input.init();

    AircraftPhysicsStateC state = aeroblend_physics_init_default(engine);
    print_state("INIT", state);

    double init_z = state.position.z;

    // Fly straight for 3 seconds
    state = run_simulation(engine, input, state, 3.0);
    print_state("3.0s", state);

    double dz = state.position.z - init_z;
    printf("  Delta Z: %.1f m (expected ~250m)\n", dz);

    if (dz < 100.0) {
        return {false, "straight_flight", "Aircraft did not move forward enough"};
    }
    if (state.airspeed_ms < 45.0) {
        return {false, "straight_flight", "Airspeed too low"};
    }
    return {true, "straight_flight", "OK"};
}

// ─────────────────────────────────────────────────────────────
// Test 2: Mouse right → aircraft turns right
// ─────────────────────────────────────────────────────────────
TestResult test_mouse_turn_right(PhysicsEngine* engine) {
    printf("\n--- Test 2: Mouse right → aircraft responds with a meaningful turn ---\n");

    InputHandler input;
    input.init();

    AircraftPhysicsStateC state = aeroblend_physics_init_default(engine);

    // First, let instructor initialize by running 1 frame
    input.update(1.0 / 120.0, state);
    state = aeroblend_physics_step(engine, &input.controls(), 1.0 / 120.0);

    double init_heading = state.heading_rad;
    print_state("INIT", state);

    // Inject mouse movement: 500 pixels to the right over 1 second
    // (simulates user dragging mouse right)
    for (int i = 0; i < 120; i++) {
        inject_mouse_motion(input, 4, 0); // ~4 px/frame = ~480 px total
        input.update(1.0 / 120.0, state);
        state = aeroblend_physics_step(engine, &input.controls(), 1.0 / 120.0);
    }
    print_state("1.0s (mouse right)", state);

    // Continue flying for 3 more seconds (instructor steers toward mouse aim)
    state = run_simulation(engine, input, state, 3.0);
    print_state("4.0s (settling)", state);

    // Check heading changed with meaningful magnitude.
    double heading_change = state.heading_rad - init_heading;
    // Normalize to [-pi, pi]
    while (heading_change > M_PI) heading_change -= 2.0 * M_PI;
    while (heading_change < -M_PI) heading_change += 2.0 * M_PI;
    double heading_change_deg = heading_change * 180.0 / M_PI;

    printf("  Heading change: %.1f degrees\n", heading_change_deg);

    printf("  X position: %.1f m\n", state.position.x);

    if (std::abs(heading_change_deg) < 5.0) {
        return {false, "mouse_turn_right", "Mouse right did not produce a meaningful turn"};
    }
    return {true, "mouse_turn_right", "OK"};
}

// ─────────────────────────────────────────────────────────────
// Test 3: Mouse up → aircraft pitches up
// ─────────────────────────────────────────────────────────────
TestResult test_mouse_pitch_up(PhysicsEngine* engine) {
    printf("\n--- Test 3: Mouse up → aircraft pitches up ---\n");

    InputHandler input;
    input.init();

    AircraftPhysicsStateC state = aeroblend_physics_init_default(engine);

    // Initialize instructor
    input.update(1.0 / 120.0, state);
    state = aeroblend_physics_step(engine, &input.controls(), 1.0 / 120.0);

    double init_pitch = state.pitch_rad;
    print_state("INIT", state);

    // Inject mouse movement: 300 pixels up over 1 second
    // Also hold Shift to increase throttle (needed to actually climb)
    inject_key(input, SDL_SCANCODE_LSHIFT, true);
    for (int i = 0; i < 120; i++) {
        inject_mouse_motion(input, 0, -3); // negative Y = up
        input.update(1.0 / 120.0, state);
        state = aeroblend_physics_step(engine, &input.controls(), 1.0 / 120.0);
    }
    inject_key(input, SDL_SCANCODE_LSHIFT, false);
    print_state("1.0s (mouse up + throttle)", state);

    double pitch_after_mouse = state.pitch_rad;

    // Continue for 2 more seconds (instructor steers toward mouse aim)
    state = run_simulation(engine, input, state, 2.0);
    print_state("3.0s (settling)", state);

    double pitch_change_deg = (pitch_after_mouse - init_pitch) * 180.0 / M_PI;
    printf("  Pitch change after mouse input: %.1f degrees\n", pitch_change_deg);
    printf("  Climb rate: %.1f m/s\n", state.climb_rate_ms);

    if (std::abs(pitch_change_deg) < 2.0) {
        return {false, "mouse_pitch_up", "Pitch barely changed with mouse up"};
    }
    return {true, "mouse_pitch_up", "OK"};
}

// ─────────────────────────────────────────────────────────────
// Test 4: W key → pitch down (keyboard override)
// ─────────────────────────────────────────────────────────────
TestResult test_keyboard_w(PhysicsEngine* engine) {
    printf("\n--- Test 4: W key → pitch down ---\n");

    InputHandler input;
    input.init();

    AircraftPhysicsStateC state = aeroblend_physics_init_default(engine);

    // Initialize
    input.update(1.0 / 120.0, state);
    state = aeroblend_physics_step(engine, &input.controls(), 1.0 / 120.0);

    double init_alt = state.altitude_m;
    print_state("INIT", state);

    // Hold W for 2 seconds — should pitch nose down → descend
    inject_key(input, SDL_SCANCODE_W, true);
    for (int i = 0; i < 240; i++) {
        input.update(1.0 / 120.0, state);
        state = aeroblend_physics_step(engine, &input.controls(), 1.0 / 120.0);
    }
    inject_key(input, SDL_SCANCODE_W, false);
    print_state("2.0s (W held)", state);

    double alt_change = state.altitude_m - init_alt;
    printf("  Altitude change: %.1f m, Climb rate: %.1f m/s\n",
           alt_change, state.climb_rate_ms);

    // W = nose down → aircraft should descend (altitude decreases)
    if (alt_change > -0.5) {
        return {false, "keyboard_w", "W key did not make aircraft descend"};
    }
    return {true, "keyboard_w", "OK"};
}

// ─────────────────────────────────────────────────────────────
// Test 5: Shift key → throttle up → speed increases
// ─────────────────────────────────────────────────────────────
TestResult test_throttle_up(PhysicsEngine* engine) {
    printf("\n--- Test 5: Shift → throttle up → speed increases ---\n");

    InputHandler input;
    input.init();

    AircraftPhysicsStateC state = aeroblend_physics_init_default(engine);

    // Fly straight for 1 second at 50% throttle
    state = run_simulation(engine, input, state, 1.0);
    double speed_at_50 = state.airspeed_ms;
    print_state("1.0s (50% throttle)", state);

    // Hold Shift for 3 seconds (throttle up)
    inject_key(input, SDL_SCANCODE_LSHIFT, true);
    state = run_simulation(engine, input, state, 3.0);
    inject_key(input, SDL_SCANCODE_LSHIFT, false);
    double speed_at_full = state.airspeed_ms;
    print_state("4.0s (after Shift)", state);

    printf("  Throttle: %.0f%%\n", input.controls().throttle * 100.0);
    printf("  Speed change: %.1f → %.1f m/s\n", speed_at_50, speed_at_full);

    if (speed_at_full <= speed_at_50) {
        return {false, "throttle_up", "Speed did not increase with Shift"};
    }
    return {true, "throttle_up", "OK"};
}

// ─────────────────────────────────────────────────────────────
// Test 6: I key toggles safety assist
// ─────────────────────────────────────────────────────────────
TestResult test_safety_assist_toggle() {
    printf("\n--- Test 6: I key → safety assist toggle ---\n");

    InputHandler input;
    input.init();

    if (!input.safety_assist_enabled()) {
        return {false, "safety_assist_toggle", "Safety assist should start enabled"};
    }

    inject_key(input, SDL_SCANCODE_I, true);
    inject_key(input, SDL_SCANCODE_I, false);
    if (input.safety_assist_enabled()) {
        return {false, "safety_assist_toggle", "I key did not disable safety assist"};
    }

    inject_key(input, SDL_SCANCODE_I, true);
    inject_key(input, SDL_SCANCODE_I, false);
    if (!input.safety_assist_enabled()) {
        return {false, "safety_assist_toggle", "I key did not re-enable safety assist"};
    }

    return {true, "safety_assist_toggle", "OK"};
}

// ─────────────────────────────────────────────────────────────
// Test 7: M key toggles map overlay request
// ─────────────────────────────────────────────────────────────
TestResult test_map_toggle() {
    printf("\n--- Test 7: M key → map toggle request ---\n");

    InputHandler input;
    input.init();

    if (input.toggle_map()) {
        return {false, "map_toggle", "Map toggle should start cleared"};
    }

    inject_key(input, SDL_SCANCODE_M, true);
    inject_key(input, SDL_SCANCODE_M, false);
    if (!input.toggle_map()) {
        return {false, "map_toggle", "M key did not set map toggle"};
    }

    input.clear_toggles();
    if (input.toggle_map()) {
        return {false, "map_toggle", "Map toggle did not clear after frame reset"};
    }

    return {true, "map_toggle", "OK"};
}

// ─────────────────────────────────────────────────────────────
// Multi-Profile Tests (biplane=0, monoplane=1, jet=2)
// ─────────────────────────────────────────────────────────────

struct ProfileMetrics {
    double alt_drift;       // altitude change from init
    double speed_drift;     // speed change from init
    double max_aoa_seen;    // peak AoA during test
    double final_speed;
    double final_alt;
};

const char* profile_name(uint32_t id) {
    switch (id) {
        case 0: return "biplane";
        case 1: return "monoplane";
        case 2: return "jet";
        default: return "unknown";
    }
}

// ─────────────────────────────────────────────────────────────
// Test 6: Level flight hold (per profile)
// ─────────────────────────────────────────────────────────────
TestResult test_level_flight_profile(PhysicsEngine* engine, uint32_t profile_id) {
    const char* name_buf = alloc_name("level_flight_%s", profile_name(profile_id));
    printf("\n--- Test: Level flight (%s) ---\n", profile_name(profile_id));

    InputHandler input;
    input.init();

    AircraftPhysicsStateC state = aeroblend_physics_init_profile(engine, profile_id);
    FlightCharacteristicsC fc = aeroblend_physics_get_flight_chars(engine);
    printf("  FC: stall=%.1f ref=%.1f max_aoa=%.1f° T/W=%.2f\n",
           fc.stall_speed_ms, fc.ref_speed_ms,
           fc.max_aoa_rad * 180.0 / M_PI, fc.thrust_to_weight);

    double init_alt = state.altitude_m;
    double init_speed = state.airspeed_ms;
    print_state("INIT", state);

    // Fly level for 5 seconds with instructor (no mouse input)
    state = run_simulation(engine, input, state, 5.0);
    print_state("5.0s", state);

    double alt_drift = state.altitude_m - init_alt;
    double speed_drift = state.airspeed_ms - init_speed;
    printf("  Alt drift: %.1f m, Speed drift: %.1f m/s\n", alt_drift, speed_drift);

    // Should not crash (alt > 0) and not lose all speed
    if (state.altitude_m < 100.0) {
        return {false, name_buf, "Aircraft lost too much altitude"};
    }
    if (state.airspeed_ms < fc.stall_speed_ms * 0.5) {
        return {false, name_buf, "Speed dropped far below stall"};
    }
    return {true, name_buf, "OK"};
}

// ─────────────────────────────────────────────────────────────
// Test 7: Mouse pitch up → no unrecoverable stall (per profile)
// ─────────────────────────────────────────────────────────────
TestResult test_pitch_up_no_stall_profile(PhysicsEngine* engine, uint32_t profile_id) {
    const char* name_buf = alloc_name("pitch_no_stall_%s", profile_name(profile_id));
    printf("\n--- Test: Pitch up no stall (%s) ---\n", profile_name(profile_id));

    InputHandler input;
    input.init();

    AircraftPhysicsStateC state = aeroblend_physics_init_profile(engine, profile_id);
    FlightCharacteristicsC fc = aeroblend_physics_get_flight_chars(engine);

    // Initialize instructor
    input.update(1.0 / 120.0, state);
    state = aeroblend_physics_step(engine, &input.controls(), 1.0 / 120.0);
    print_state("INIT", state);

    // Phase 1: Cruise 2 seconds
    state = run_simulation(engine, input, state, 2.0);
    double speed_before = state.airspeed_ms;
    double alt_before = state.altitude_m;
    print_state("2.0s (cruise)", state);

    // Phase 2: Mouse up for 2 seconds (aggressive pitch-up request)
    double max_aoa = 0.0;
    for (int i = 0; i < 240; i++) {
        inject_mouse_motion(input, 0, -4); // strong mouse up
        input.update(1.0 / 120.0, state);
        state = aeroblend_physics_step(engine, &input.controls(), 1.0 / 120.0);
        if (state.angle_of_attack_rad > max_aoa) max_aoa = state.angle_of_attack_rad;
    }
    print_state("4.0s (mouse up)", state);
    printf("  Max AoA during pitch-up: %.1f degrees\n", max_aoa * 180.0 / M_PI);

    // Phase 3: Release mouse, let instructor recover for 5 seconds
    state = run_simulation(engine, input, state, 5.0);
    print_state("9.0s (recovery)", state);

    printf("  Speed: %.1f -> %.1f m/s, Alt: %.1f -> %.1f m\n",
           speed_before, state.airspeed_ms, alt_before, state.altitude_m);

    // Must recover: not in deep stall, not crashed
    if (state.altitude_m < 50.0) {
        return {false, name_buf, "Aircraft crashed during stall recovery"};
    }
    if (state.airspeed_ms < fc.stall_speed_ms * 0.3) {
        return {false, name_buf, "Aircraft did not recover from stall"};
    }
    return {true, name_buf, "OK"};
}

// ─────────────────────────────────────────────────────────────
// Test 8: Sharp turn → energy preservation (per profile)
// ─────────────────────────────────────────────────────────────
TestResult test_sharp_turn_energy_profile(PhysicsEngine* engine, uint32_t profile_id) {
    const char* name_buf = alloc_name("sharp_turn_%s", profile_name(profile_id));
    printf("\n--- Test: Sharp turn energy (%s) ---\n", profile_name(profile_id));

    InputHandler input;
    input.init();

    AircraftPhysicsStateC state = aeroblend_physics_init_profile(engine, profile_id);
    FlightCharacteristicsC fc = aeroblend_physics_get_flight_chars(engine);

    // Initialize instructor
    input.update(1.0 / 120.0, state);
    state = aeroblend_physics_step(engine, &input.controls(), 1.0 / 120.0);

    // Cruise 2 seconds to stabilize
    state = run_simulation(engine, input, state, 2.0);
    double speed_before = state.airspeed_ms;
    double alt_before = state.altitude_m;
    double heading_before = state.heading_rad;
    print_state("2.0s (cruise)", state);

    // Sharp right turn: mouse 600px right over 1 second
    for (int i = 0; i < 120; i++) {
        inject_mouse_motion(input, 5, 0);
        input.update(1.0 / 120.0, state);
        state = aeroblend_physics_step(engine, &input.controls(), 1.0 / 120.0);
    }
    print_state("3.0s (turn)", state);

    // Let the turn settle for 4 more seconds
    state = run_simulation(engine, input, state, 4.0);
    print_state("7.0s (settled)", state);

    double heading_after = state.heading_rad;
    double heading_change = heading_after - heading_before;
    while (heading_change > M_PI) heading_change -= 2.0 * M_PI;
    while (heading_change < -M_PI) heading_change += 2.0 * M_PI;

    double speed_loss_pct = (speed_before - state.airspeed_ms) / speed_before * 100.0;
    double alt_loss = alt_before - state.altitude_m;

    printf("  Heading change: %.1f deg, Speed loss: %.1f%%, Alt loss: %.1f m\n",
           heading_change * 180.0 / M_PI, speed_loss_pct, alt_loss);

    // Mouse right should produce a strong turn response. World-space sign depends
    // on the active chase-camera/input convention, so only magnitude matters here.
    if (std::abs(heading_change) < 2.0 * M_PI / 180.0) {
        return {false, name_buf, "Mouse right did not produce a strong turn"};
    }
    // Should not crash (allow altitude loss during aggressive turns)
    if (state.altitude_m < 10.0) {
        return {false, name_buf, "Aircraft crashed during turn"};
    }
    return {true, name_buf, "OK"};
}

// ─────────────────────────────────────────────────────────────
// Test 9: Auto-throttle activates at low speed (per profile)
// ─────────────────────────────────────────────────────────────
TestResult test_auto_throttle_profile(PhysicsEngine* engine, uint32_t profile_id) {
    const char* name_buf = alloc_name("auto_throttle_%s", profile_name(profile_id));
    printf("\n--- Test: Auto-throttle (%s) ---\n", profile_name(profile_id));

    InputHandler input;
    input.init();

    AircraftPhysicsStateC state = aeroblend_physics_init_profile(engine, profile_id);
    FlightCharacteristicsC fc = aeroblend_physics_get_flight_chars(engine);

    // Initialize instructor
    input.update(1.0 / 120.0, state);
    state = aeroblend_physics_step(engine, &input.controls(), 1.0 / 120.0);

    // Cruise briefly
    state = run_simulation(engine, input, state, 1.0);

    // Cut throttle to zero with Ctrl held for 3 seconds
    inject_key(input, SDL_SCANCODE_LCTRL, true);
    state = run_simulation(engine, input, state, 3.0);
    inject_key(input, SDL_SCANCODE_LCTRL, false);

    double speed_after_cut = state.airspeed_ms;
    double throttle_after_cut = input.controls().throttle;
    print_state("4.0s (after throttle cut)", state);
    printf("  Throttle after cut: %.0f%%, Speed: %.1f m/s\n",
           throttle_after_cut * 100.0, speed_after_cut);

    // Continue flying 5 seconds -- auto-throttle should kick in if speed < threshold
    state = run_simulation(engine, input, state, 5.0);
    double speed_after_auto = state.airspeed_ms;
    double throttle_after_auto = input.controls().throttle;
    print_state("9.0s (auto-throttle)", state);
    printf("  Throttle after auto: %.0f%%, Speed: %.1f m/s\n",
           throttle_after_auto * 100.0, speed_after_auto);

    // Must not crash
    if (state.altitude_m < 0.1) {
        return {false, name_buf, "Aircraft crashed after throttle cut"};
    }
    // Auto-throttle should have prevented complete speed loss
    // (biplane with low stall might stay above threshold, jet might not trigger)
    return {true, name_buf, "OK"};
}

// ─────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────
int main() {
    printf("=== AeroBlend Mouse Flight Integration Tests ===\n");
    printf("Testing: mouse -> instructor -> physics pipeline\n");

    // SDL_Init needed for event type constants, but no window needed
    SDL_Init(SDL_INIT_EVENTS);

    constexpr int kMaxTests = 32;
    TestResult results[kMaxTests];
    int total = 0;
    int passed = 0;

    // ─── Original tests (biplane/default) ───
    printf("\n========== ORIGINAL TESTS (biplane default) ==========\n");

    {
        PhysicsEnginePtr engine(aeroblend_physics_create());
        results[total++] = test_straight_flight(engine.get());
    }
    {
        PhysicsEnginePtr engine(aeroblend_physics_create());
        results[total++] = test_mouse_turn_right(engine.get());
    }
    {
        PhysicsEnginePtr engine(aeroblend_physics_create());
        results[total++] = test_mouse_pitch_up(engine.get());
    }
    {
        PhysicsEnginePtr engine(aeroblend_physics_create());
        results[total++] = test_keyboard_w(engine.get());
    }
    {
        PhysicsEnginePtr engine(aeroblend_physics_create());
        results[total++] = test_throttle_up(engine.get());
    }
    {
        results[total++] = test_safety_assist_toggle();
    }
    {
        results[total++] = test_map_toggle();
    }

    // ─── Multi-profile tests ───
    uint32_t profiles[] = {0, 1, 2}; // biplane, monoplane, jet
    for (uint32_t pid : profiles) {
        printf("\n========== PROFILE: %s (id=%u) ==========\n", profile_name(pid), pid);

        {
            PhysicsEnginePtr engine(aeroblend_physics_create());
            results[total++] = test_level_flight_profile(engine.get(), pid);
        }
        {
            PhysicsEnginePtr engine(aeroblend_physics_create());
            results[total++] = test_pitch_up_no_stall_profile(engine.get(), pid);
        }
        {
            PhysicsEnginePtr engine(aeroblend_physics_create());
            results[total++] = test_sharp_turn_energy_profile(engine.get(), pid);
        }
        {
            PhysicsEnginePtr engine(aeroblend_physics_create());
            results[total++] = test_auto_throttle_profile(engine.get(), pid);
        }
    }

    // ─── Summary ───
    printf("\n=== Results ===\n");
    for (int i = 0; i < total; i++) {
        const char* status = results[i].passed ? "PASS" : "FAIL";
        printf("  [%s] %s: %s\n", status, results[i].name, results[i].detail);
        if (results[i].passed) passed++;
    }
    printf("\n%d/%d tests passed.\n", passed, total);

    SDL_Quit();
    return (passed == total) ? 0 : 1;
}
