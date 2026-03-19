//! 탄도 물리 시뮬레이션 크레이트.
//!
//! 기총 발사, 탄환 비행 (중력 + 공기저항), 탄약 관리를 처리한다.
//! 물리 엔진과 동일한 120Hz 고정 타임스텝에서 오일러 적분을 사용한다.

use glam::{DMat3, DVec3};

// ──────────────────────────────────────────────
// 상수
// ──────────────────────────────────────────────

/// 중력 가속도 (m/s²)
const G: f64 = 9.80665;

/// 해수면 공기 밀도 (kg/m³)
const RHO_SEA_LEVEL: f64 = 1.225;

/// 탄환 최대 수명 (초)
const MAX_LIFETIME_S: f64 = 5.0;

/// 동시 활성 탄환 최대 수
const MAX_PROJECTILES: usize = 512;

// ──────────────────────────────────────────────
// 탄종 (ProjectileType)
// ──────────────────────────────────────────────

/// 탄환 종류 — 질량, 구경, 초속, 항력 계수, 단면적을 정의한다.
#[derive(Debug, Clone, Copy)]
pub struct ProjectileType {
    /// 탄환 질량 (kg)
    pub mass_kg: f64,
    /// 구경 (m)
    pub caliber_m: f64,
    /// 초속 (m/s)
    pub muzzle_velocity_ms: f64,
    /// 항력 계수 (무차원)
    pub drag_coefficient: f64,
    /// 단면적 (m²) — π × (caliber/2)²
    pub cross_section_m2: f64,
}

// ──────────────────────────────────────────────
// 기본 탄종 프리셋
// ──────────────────────────────────────────────

/// .50 cal M2 Browning — 12.7mm 중기관총
pub fn m2_browning_projectile() -> ProjectileType {
    let caliber = 0.0127; // 12.7mm
    ProjectileType {
        mass_kg: 0.045,
        caliber_m: caliber,
        muzzle_velocity_ms: 890.0,
        drag_coefficient: 0.295,
        cross_section_m2: std::f64::consts::PI * (caliber / 2.0) * (caliber / 2.0),
    }
}

/// 20mm AN/M2 — 항공기용 기관포
pub fn an_m2_20mm_projectile() -> ProjectileType {
    let caliber = 0.020; // 20mm
    ProjectileType {
        mass_kg: 0.130,
        caliber_m: caliber,
        muzzle_velocity_ms: 840.0,
        drag_coefficient: 0.32,
        cross_section_m2: std::f64::consts::PI * (caliber / 2.0) * (caliber / 2.0),
    }
}

/// 7.7mm Vickers — 경기관총
pub fn vickers_7_7mm_projectile() -> ProjectileType {
    let caliber = 0.0077; // 7.7mm
    ProjectileType {
        mass_kg: 0.011,
        caliber_m: caliber,
        muzzle_velocity_ms: 745.0,
        drag_coefficient: 0.30,
        cross_section_m2: std::f64::consts::PI * (caliber / 2.0) * (caliber / 2.0),
    }
}

// ──────────────────────────────────────────────
// 탄환 (Projectile)
// ──────────────────────────────────────────────

/// 비행 중인 개별 탄환.
#[derive(Debug, Clone, Copy)]
pub struct Projectile {
    /// 월드 좌표 위치 (m)
    pub position: DVec3,
    /// 속도 벡터 (m/s)
    pub velocity: DVec3,
    /// 탄종 참조
    pub proj_type: ProjectileType,
    /// 발사 후 경과 시간 (초)
    pub time_alive: f64,
    /// 활성 여부 — false면 재활용 슬롯
    pub active: bool,
}

impl Projectile {
    /// 새 탄환 생성.
    pub fn new(position: DVec3, velocity: DVec3, proj_type: ProjectileType) -> Self {
        Self {
            position,
            velocity,
            proj_type,
            time_alive: 0.0,
            active: true,
        }
    }
}

// ──────────────────────────────────────────────
// 기총 스펙 (GunSpec)
// ──────────────────────────────────────────────

/// 기총 제원 — 발사율, 포구 위치/방향, 장탄수, 산포각.
#[derive(Debug, Clone)]
pub struct GunSpec {
    /// 사용 탄종
    pub projectile_type: ProjectileType,
    /// 분당 발사 수 (RPM)
    pub fire_rate_rpm: f64,
    /// 항공기 로컬 좌표계 포구 위치
    pub muzzle_position_local: DVec3,
    /// 항공기 로컬 좌표계 발사 방향 (정규화)
    pub muzzle_direction_local: DVec3,
    /// 최대 장탄수
    pub ammo: u32,
    /// 산포 반각 (rad)
    pub spread_rad: f64,
}

// ──────────────────────────────────────────────
// 기본 기총 프리셋
// ──────────────────────────────────────────────

/// .50 cal M2 Browning 기총 스펙 (기수 장착 기본값)
pub fn m2_browning_gun() -> GunSpec {
    GunSpec {
        projectile_type: m2_browning_projectile(),
        fire_rate_rpm: 750.0,
        muzzle_position_local: DVec3::new(0.0, 0.0, 3.0),
        muzzle_direction_local: DVec3::new(0.0, 0.0, 1.0),
        ammo: 400,
        spread_rad: 0.003,
    }
}

/// 20mm AN/M2 기관포 스펙
pub fn an_m2_20mm_gun() -> GunSpec {
    GunSpec {
        projectile_type: an_m2_20mm_projectile(),
        fire_rate_rpm: 600.0,
        muzzle_position_local: DVec3::new(0.0, -0.2, 3.0),
        muzzle_direction_local: DVec3::new(0.0, 0.0, 1.0),
        ammo: 200,
        spread_rad: 0.005,
    }
}

/// 7.7mm Vickers 기총 스펙
pub fn vickers_7_7mm_gun() -> GunSpec {
    GunSpec {
        projectile_type: vickers_7_7mm_projectile(),
        fire_rate_rpm: 1000.0,
        muzzle_position_local: DVec3::new(0.0, 0.1, 2.5),
        muzzle_direction_local: DVec3::new(0.0, 0.0, 1.0),
        ammo: 600,
        spread_rad: 0.004,
    }
}

// ──────────────────────────────────────────────
// 기총 상태 (GunState)
// ──────────────────────────────────────────────

/// 개별 기총의 런타임 상태.
#[derive(Debug, Clone)]
pub struct GunState {
    /// 기총 제원
    pub spec: GunSpec,
    /// 남은 탄약 수
    pub ammo_remaining: u32,
    /// 마지막 발사 후 경과 시간 (초)
    pub time_since_last_shot: f64,
    /// 발사 중 여부 (트리거 눌림)
    pub firing: bool,
}

impl GunState {
    /// 새 기총 상태 — 탄약 만충.
    pub fn new(spec: GunSpec) -> Self {
        let ammo = spec.ammo;
        let interval = 60.0 / spec.fire_rate_rpm;
        Self {
            spec,
            ammo_remaining: ammo,
            // 첫 프레임부터 즉시 1발 발사 가능하도록 정확히 1간격
            time_since_last_shot: interval,
            firing: false,
        }
    }

    /// 발사 간격 (초) — 60 / RPM.
    #[inline]
    pub fn fire_interval(&self) -> f64 {
        60.0 / self.spec.fire_rate_rpm
    }

    /// 발사 가능 여부: 탄약이 남아있고, 발사 간격이 지났으며, 트리거가 눌려있을 때.
    #[inline]
    pub fn can_fire(&self) -> bool {
        self.firing && self.ammo_remaining > 0 && self.time_since_last_shot >= self.fire_interval()
    }
}

// ──────────────────────────────────────────────
// 무장 시스템 (WeaponSystem)
// ──────────────────────────────────────────────

/// 항공기 무장 시스템 — 기총 배열과 활성 탄환 풀.
#[derive(Debug, Clone)]
pub struct WeaponSystem {
    /// 장착된 기총들
    pub guns: Vec<GunState>,
    /// 탄환 풀 (최대 MAX_PROJECTILES)
    pub projectiles: Vec<Projectile>,
}

impl WeaponSystem {
    /// 기총 목록으로 무장 시스템 생성.
    pub fn new(gun_specs: Vec<GunSpec>) -> Self {
        let guns = gun_specs.into_iter().map(GunState::new).collect();
        Self {
            guns,
            projectiles: Vec::with_capacity(MAX_PROJECTILES),
        }
    }

    /// 전체 무장 시스템 업데이트: 기총 발사 → 탄환 물리.
    ///
    /// - `dt`: 타임스텝 (초)
    /// - `aircraft_position`: 항공기 월드 좌표 위치
    /// - `aircraft_orientation`: 항공기 로컬→월드 회전 행렬
    /// - `aircraft_velocity`: 항공기 속도 (탄환 초속에 합산)
    pub fn update(
        &mut self,
        dt: f64,
        aircraft_position: DVec3,
        aircraft_orientation: DMat3,
        aircraft_velocity: DVec3,
    ) {
        // 1) 기총 발사 처리
        self.update_guns(dt, aircraft_position, aircraft_orientation, aircraft_velocity);

        // 2) 탄환 물리 스텝
        self.update_projectiles(dt);
    }

    /// 기총 타이머 갱신 및 발사.
    fn update_guns(
        &mut self,
        dt: f64,
        aircraft_position: DVec3,
        aircraft_orientation: DMat3,
        aircraft_velocity: DVec3,
    ) {
        // 발사된 탄환을 임시 버퍼에 수집 (borrow checker 우회)
        let mut new_projectiles: Vec<(DVec3, DVec3, ProjectileType)> = Vec::new();

        for gun in &mut self.guns {
            gun.time_since_last_shot += dt;

            // 한 프레임에 여러 발 발사 가능 (고RPM 기총)
            while gun.can_fire() {
                // 포구 위치: 로컬 → 월드 변환
                let world_muzzle_pos =
                    aircraft_position + aircraft_orientation * gun.spec.muzzle_position_local;

                // 발사 방향: 로컬 → 월드 + 산포 적용
                let base_dir =
                    (aircraft_orientation * gun.spec.muzzle_direction_local).normalize();
                let spread_dir = apply_spread(base_dir, gun.spec.spread_rad, gun.ammo_remaining);

                // 탄환 초속 = 항공기 속도 + 초속 × 방향
                let muzzle_vel =
                    aircraft_velocity + spread_dir * gun.spec.projectile_type.muzzle_velocity_ms;

                new_projectiles.push((world_muzzle_pos, muzzle_vel, gun.spec.projectile_type));

                gun.ammo_remaining -= 1;
                gun.time_since_last_shot -= gun.fire_interval();
            }
        }

        // 수집된 탄환을 풀에 추가
        for (pos, vel, ptype) in new_projectiles {
            self.spawn_projectile(pos, vel, ptype);
        }
    }

    /// 탄환 슬롯에 새 탄환 추가. 최대 수 초과 시 무시.
    fn spawn_projectile(&mut self, position: DVec3, velocity: DVec3, proj_type: ProjectileType) {
        // 비활성 슬롯 재활용 시도
        for p in &mut self.projectiles {
            if !p.active {
                *p = Projectile::new(position, velocity, proj_type);
                return;
            }
        }

        // 슬롯이 없으면 새로 추가 (상한 확인)
        if self.projectiles.len() < MAX_PROJECTILES {
            self.projectiles
                .push(Projectile::new(position, velocity, proj_type));
        }
        // MAX_PROJECTILES 도달 시 발사 포기 (탄약은 이미 소모됨)
    }

    /// 모든 활성 탄환에 물리 스텝 적용.
    fn update_projectiles(&mut self, dt: f64) {
        for p in &mut self.projectiles {
            if !p.active {
                continue;
            }
            step_projectile(p, dt);
        }
    }

    /// 현재 활성 탄환 수.
    pub fn active_projectile_count(&self) -> usize {
        self.projectiles.iter().filter(|p| p.active).count()
    }
}

// ──────────────────────────────────────────────
// 탄환 물리
// ──────────────────────────────────────────────

/// 간이 ISA 대기 밀도 (kg/m³).
///
/// ρ(h) = 1.225 × (1 − 0.0000226 × h)^4.256
/// 고도 0~11 km 범위에서 충분히 정확하다.
#[inline]
pub fn air_density(altitude_m: f64) -> f64 {
    let h = altitude_m.max(0.0);
    RHO_SEA_LEVEL * (1.0 - 0.0000226 * h).max(0.0).powf(4.256)
}

/// 단일 탄환에 오일러 적분 적용.
///
/// 가속도 = 중력 + 공기저항
/// - 중력: (0, -g, 0)
/// - 항력: F_drag = 0.5 × ρ(h) × |v|² × Cd × A, 속도 반대 방향
pub fn step_projectile(p: &mut Projectile, dt: f64) {
    let speed = p.velocity.length();

    // 중력
    let gravity = DVec3::new(0.0, -G, 0.0);

    // 공기저항 가속도
    let drag_accel = if speed > 1e-6 {
        let rho = air_density(p.position.y);
        let drag_force_mag =
            0.5 * rho * speed * speed * p.proj_type.drag_coefficient * p.proj_type.cross_section_m2;
        let drag_dir = -p.velocity / speed; // 속도 반대 방향
        drag_dir * (drag_force_mag / p.proj_type.mass_kg)
    } else {
        DVec3::ZERO
    };

    // 오일러 적분
    let accel = gravity + drag_accel;
    p.velocity += accel * dt;
    p.position += p.velocity * dt;

    // 수명 갱신
    p.time_alive += dt;
    if p.time_alive >= MAX_LIFETIME_S {
        p.active = false;
    }
}

// ──────────────────────────────────────────────
// 산포 (Spread)
// ──────────────────────────────────────────────

/// 결정론적 산포 적용 — seed 값으로 원뿔 내 방향 오프셋 생성.
///
/// 난수 대신 seed 기반 sin/cos로 재현 가능한 산포 패턴을 만든다.
fn apply_spread(direction: DVec3, spread_rad: f64, seed: u32) -> DVec3 {
    if spread_rad <= 0.0 {
        return direction;
    }

    // seed 기반 의사 난수 각도
    let angle = (seed as f64) * 2.394_816_3; // 황금비 기반 회전
    let radius_frac = ((seed as f64 * 0.618_033_988) % 1.0).sqrt(); // 균등 분포 보정
    let offset_rad = spread_rad * radius_frac;

    // direction에 수직인 두 기저 벡터 구하기
    let (tangent, bitangent) = perpendicular_basis(direction);

    // 원뿔 오프셋 적용
    let offset = tangent * (offset_rad.sin() * angle.cos())
        + bitangent * (offset_rad.sin() * angle.sin());
    let result = direction * offset_rad.cos() + offset;

    result.normalize()
}

/// 주어진 벡터에 수직인 직교 기저 쌍을 반환.
fn perpendicular_basis(n: DVec3) -> (DVec3, DVec3) {
    // 가장 작은 성분 축과 외적
    let abs_n = DVec3::new(n.x.abs(), n.y.abs(), n.z.abs());
    let helper = if abs_n.x <= abs_n.y && abs_n.x <= abs_n.z {
        DVec3::X
    } else if abs_n.y <= abs_n.z {
        DVec3::Y
    } else {
        DVec3::Z
    };
    let tangent = n.cross(helper).normalize();
    let bitangent = n.cross(tangent).normalize();
    (tangent, bitangent)
}

// ──────────────────────────────────────────────
// 테스트
// ──────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// 수평 발사 시 중력으로 Y 좌표 감소 확인.
    #[test]
    fn test_projectile_gravity() {
        let proj_type = m2_browning_projectile();
        let mut p = Projectile::new(
            DVec3::new(0.0, 1000.0, 0.0),
            DVec3::new(0.0, 0.0, 890.0), // 수평 발사
            proj_type,
        );

        let initial_y = p.position.y;
        // 10스텝 × 1/120초
        for _ in 0..10 {
            step_projectile(&mut p, 1.0 / 120.0);
        }

        assert!(
            p.position.y < initial_y,
            "중력으로 Y가 감소해야 함: 초기={}, 현재={}",
            initial_y,
            p.position.y
        );
    }

    /// 전방 발사 시 공기저항으로 속력 감소 확인.
    #[test]
    fn test_projectile_drag() {
        let proj_type = m2_browning_projectile();
        let mut p = Projectile::new(
            DVec3::new(0.0, 0.0, 0.0),
            DVec3::new(0.0, 0.0, 890.0),
            proj_type,
        );

        let initial_speed = p.velocity.length();
        for _ in 0..120 {
            // 1초 시뮬레이션
            step_projectile(&mut p, 1.0 / 120.0);
        }

        let final_speed = p.velocity.length();
        assert!(
            final_speed < initial_speed,
            "항력으로 속력 감소해야 함: 초기={:.1}, 현재={:.1}",
            initial_speed,
            final_speed
        );
    }

    /// 750 RPM에서 1초간 ~12.5발 발사 확인.
    #[test]
    fn test_fire_rate() {
        let gun_spec = m2_browning_gun();
        let mut ws = WeaponSystem::new(vec![gun_spec]);
        ws.guns[0].firing = true;

        let dt = 1.0 / 120.0;
        let orientation = DMat3::IDENTITY;
        let pos = DVec3::ZERO;
        let vel = DVec3::ZERO;

        // 1초 시뮬레이션 (120프레임)
        for _ in 0..120 {
            ws.update(dt, pos, orientation, vel);
        }

        // 750 RPM = 12.5발/초, 1초간 12~13발 예상
        let fired = ws.guns[0].spec.ammo - ws.guns[0].ammo_remaining;
        assert!(
            (12..=13).contains(&fired),
            "750 RPM에서 1초간 12~13발 예상, 실제={}",
            fired
        );
    }

    /// 탄약 소진 후 더 이상 발사 불가 확인.
    #[test]
    fn test_ammo_depletion() {
        let mut spec = m2_browning_gun();
        spec.ammo = 5; // 5발만 장전
        let mut ws = WeaponSystem::new(vec![spec]);
        ws.guns[0].firing = true;

        let dt = 1.0 / 120.0;
        let orientation = DMat3::IDENTITY;
        let pos = DVec3::ZERO;
        let vel = DVec3::ZERO;

        // 충분히 긴 시간 시뮬레이션
        for _ in 0..600 {
            // 5초
            ws.update(dt, pos, orientation, vel);
        }

        assert_eq!(
            ws.guns[0].ammo_remaining, 0,
            "탄약이 모두 소진되어야 함"
        );

        // 탄약 소진 상태에서 발사 불가
        assert!(
            !ws.guns[0].can_fire(),
            "탄약 소진 후 발사 불가해야 함"
        );
    }

    /// 512발 상한 초과 시 탄환 수 제한 확인.
    #[test]
    fn test_max_projectiles() {
        let mut spec = m2_browning_gun();
        spec.ammo = 1000;
        spec.fire_rate_rpm = 60_000.0; // 초고속 — 프레임마다 여러 발
        let mut ws = WeaponSystem::new(vec![spec]);
        ws.guns[0].firing = true;

        let dt = 1.0 / 120.0;
        let orientation = DMat3::IDENTITY;
        let pos = DVec3::ZERO;
        let vel = DVec3::ZERO;

        // 강제로 많이 발사
        for _ in 0..600 {
            ws.update(dt, pos, orientation, vel);
        }

        assert!(
            ws.projectiles.len() <= MAX_PROJECTILES,
            "탄환 수 상한 초과: {} > {}",
            ws.projectiles.len(),
            MAX_PROJECTILES
        );
    }

    /// 5초 경과 후 탄환 비활성화 확인.
    #[test]
    fn test_projectile_lifetime() {
        let proj_type = m2_browning_projectile();
        let mut p = Projectile::new(
            DVec3::new(0.0, 5000.0, 0.0), // 높은 고도 (지면 충돌 없도록)
            DVec3::new(0.0, 0.0, 890.0),
            proj_type,
        );

        let dt = 1.0 / 120.0;
        // 5초 = 600프레임
        for _ in 0..600 {
            step_projectile(&mut p, dt);
        }

        assert!(
            !p.active,
            "5초 경과 후 탄환이 비활성화되어야 함 (time_alive={:.3})",
            p.time_alive
        );
    }

    /// 포구 위치 로컬→월드 변환 정확성 확인.
    #[test]
    fn test_muzzle_transform() {
        let gun_spec = m2_browning_gun();
        let mut ws = WeaponSystem::new(vec![gun_spec]);
        ws.guns[0].firing = true;

        let aircraft_pos = DVec3::new(100.0, 200.0, 300.0);
        // 90도 요(yaw) 회전: Z축 → -X축 방향
        let yaw_90 = DMat3::from_cols(
            DVec3::new(0.0, 0.0, -1.0), // X열
            DVec3::new(0.0, 1.0, 0.0),  // Y열
            DVec3::new(1.0, 0.0, 0.0),  // Z열
        );

        let dt = 1.0 / 120.0;
        ws.update(dt, aircraft_pos, yaw_90, DVec3::ZERO);

        // 1발 이상 발사되었어야 함
        assert!(
            !ws.projectiles.is_empty(),
            "최소 1발 발사되어야 함"
        );

        let p = &ws.projectiles[0];
        // 포구 로컬 (0,0,3) → 90도 요 회전 → 월드 (3,0,0) + 항공기 위치
        let expected_muzzle = aircraft_pos + yaw_90 * DVec3::new(0.0, 0.0, 3.0);
        let pos_diff = (p.position - expected_muzzle).length();

        // 1프레임(dt=1/120) 동안 이동했으므로 약간의 허용 오차
        // 탄환은 발사 후 한 프레임 이동: ~890/120 ≈ 7.4m
        assert!(
            pos_diff < 10.0,
            "포구 위치 변환 오차가 과대: diff={:.2}, 발사위치={:?}, 예상={:?}",
            pos_diff,
            p.position,
            expected_muzzle
        );

        // 탄환 주 속도 방향이 회전된 Z축(= 월드 X축) 방향인지 확인
        let vel_dir = p.velocity.normalize();
        let expected_dir = (yaw_90 * DVec3::new(0.0, 0.0, 1.0)).normalize();
        let dot = vel_dir.dot(expected_dir);
        assert!(
            dot > 0.99,
            "탄환 방향이 회전된 포구 방향과 일치해야 함: dot={:.4}",
            dot
        );
    }

    /// 간이 대기 밀도 함수 해수면/고고도 검증.
    #[test]
    fn test_air_density() {
        let rho_0 = air_density(0.0);
        assert!(
            (rho_0 - 1.225).abs() < 0.001,
            "해수면 밀도 ≈ 1.225: 실제={:.4}",
            rho_0
        );

        let rho_5k = air_density(5000.0);
        assert!(
            rho_5k < rho_0 && rho_5k > 0.5,
            "5000m 밀도는 해수면보다 낮고 0.5 이상: {:.4}",
            rho_5k
        );

        let rho_neg = air_density(-100.0);
        assert!(
            (rho_neg - 1.225).abs() < 0.001,
            "음수 고도는 해수면으로 클램프: {:.4}",
            rho_neg
        );
    }

    /// 산포 적용 시 방향이 원래 방향과 유사하되 약간 벗어남 확인.
    #[test]
    fn test_spread_deviation() {
        let dir = DVec3::new(0.0, 0.0, 1.0);
        let spread_dir = apply_spread(dir, 0.003, 42);
        let dot = dir.dot(spread_dir);
        // 산포 0.003 rad ≈ 0.17° — cos(0.003) ≈ 0.9999955
        assert!(
            dot > 0.999,
            "산포가 너무 큼: dot={:.6}",
            dot
        );
        // 완전히 동일하지는 않아야 함 (seed=42이면 오프셋 존재)
        assert!(
            dot < 1.0,
            "산포가 전혀 적용되지 않음"
        );
    }
}
