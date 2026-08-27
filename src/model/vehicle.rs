use serde::{Deserialize, Serialize};

/// 速度-引張力（または走行抵抗）のテーブル
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct VelocityForceTable {
    /// (速度 [km/h], 引張力/抵抗力 [N]) の配列
    pub value: Vec<(f64, f64)>,
}

impl VelocityForceTable {
    pub fn new() -> Self {
        Self::default()
    }

    /// 速度 [km/h] に対する引張力/抵抗力 [N] を線形補間で算出する
    pub fn force_at(&self, speed: f64) -> f64 {
        if self.value.is_empty() {
            return 0.0;
        }

        // 範囲外（最下限以下 / 最上限以上）のガード
        if speed <= self.value[0].0 {
            return self.value[0].1;
        }
        let last_idx = self.value.len() - 1;
        if speed >= self.value[last_idx].0 {
            return self.value[last_idx].1;
        }

        // 線形探索による区間特定（要素数が少ないテーブル向け）
        // ※データ数が非常に多い場合は binary_search_by を使用可能
        for i in 0..last_idx {
            let (x1, y1) = self.value[i];
            let (x2, y2) = self.value[i + 1];

            if x1 <= speed && speed <= x2 {
                if (x2 - x1).abs() < f64::EPSILON {
                    return y1;
                }
                // 線形補間
                let t = (speed - x1) / (x2 - x1);
                return y1 + t * (y2 - y1);
            }
        }

        self.value[last_idx].1
    }
}

/// 車両データ
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct Vehicle {
    /// 車両名
    pub name: String,
    /// 最高速度 [km/h]
    pub max_speed: f64,
    /// 編成両数
    pub number_of_cars: u32,
    /// 編成長 [m]
    pub train_length: f64,
    /// 編成重量 [t]
    pub train_weight: f64,
    /// 編成定員
    pub capacity: u32,
    /// 1人あたり重量 [kg]
    pub human_weight: f64,
    /// 乗車率 [%]
    pub occupancy_rate: f64,
    /// ユニット数
    pub unit_count: u32,

    /// 起動加速度 [km/h/s]
    pub startup_acceleration: f64,
    /// 減速度 [km/h/s]
    pub deceleration: f64,
    /// 定トルク領域終了速度 [km/h]
    pub fixed_torque_speed: f64,
    /// 定出力領域終了速度 [km/h]
    pub constant_power_speed: f64,
    /// M車の両数
    pub m_cars: u32,
    /// T車の両数
    pub t_cars: u32,
    /// M車の重量 [t]
    pub m_weight: f64,
    /// T車の重量 [t]
    pub t_weight: f64,
    /// 定出力領域の定数
    pub coefficient0: f64,
    /// 特性領域の定数
    pub coefficient1: f64,

    /// 加速時の引張力データ
    pub acceleration_force: VelocityForceTable,
    /// 減速時の引張力データ
    pub deceleration_force: VelocityForceTable,
    /// 走行抵抗データ
    pub running_resist: VelocityForceTable,
}

/// デフォルトの走行抵抗 [kgf] を計算する（1km/h刻み）
/// 
/// - `m_cars` / `t_cars`: M車 / T車の両数
/// - `m_weight` / `t_weight`: M車 / T車の重量 [t]
/// - `max_speed`: 最高速度 [km/h]
pub fn default_running_resistance(
    m_cars: f64,
    t_cars: f64,
    m_weight: f64,
    t_weight: f64,
    max_speed: usize,
) -> Vec<(f64, f64)> {
    let mut result = Vec::with_capacity(max_speed);
    let total_m_weight = m_cars * m_weight;
    let total_t_weight = t_cars * t_weight;
    let total_weight = total_m_weight + total_t_weight;

    for i in 0..max_speed {
        let v = i as f64;
        // 機械抵抗・空気抵抗計算
        let motor_car_resistance = (1.65 + 0.0247 * v) * total_m_weight;
        let trailer_car_resistance = (0.78 + 0.028 * v) * total_t_weight;
        let air_resistance = 0.028 + 0.0078 * (m_cars + t_cars - 1.0) * v.powi(2);

        let total_resistance = motor_car_resistance + trailer_car_resistance + air_resistance;
        result.push((v, total_resistance));
    }

    // 出発抵抗を補正 (0km/h: +3kgf/t, 1km/h: +2kgf/t, 2km/h: +1kgf/t)
    if result.len() > 0 {
        result[0].1 += 3.0 * total_weight;
    }
    if result.len() > 1 {
        result[1].1 += 2.0 * total_weight;
    }
    if result.len() > 2 {
        result[2].1 += 1.0 * total_weight;
    }

    result
}

/// デフォルトの引張力曲線 [kgf] を計算する（VVVF制御車・1km/h刻み）
pub fn default_tractive_force(
    startup_acceleration: f64,
    fixed_torque_speed: usize,
    constant_power_speed: usize,
    max_speed: usize,
    m_cars: f64,
    t_cars: f64,
    m_weight: f64,
    t_weight: f64,
    coefficient0: f64,
    coefficient1: f64,
) -> Vec<(f64, f64)> {
    let mut result = Vec::with_capacity(max_speed);
    let margin_ratio = 0.1;
    let total_weight_kg = (m_cars * m_weight + t_cars * t_weight) * 1000.0;

    // 1. 定トルク領域 [0..fixed_torque_speed)
    // F[kgf] = 編成重量[kg] * 起動加速度[m/s^2] / 9.807
    let fixed_torque_base = (startup_acceleration / 3.6) * total_weight_kg / 9.807;
    let fixed_torque = fixed_torque_base * (1.0 - margin_ratio);

    let end_fixed_torque = fixed_torque_speed.min(max_speed);
    for i in 0..end_fixed_torque {
        result.push((i as f64, fixed_torque));
    }

    // 2. 定出力領域 [fixed_torque_speed..constant_power_speed)
    if fixed_torque_speed < max_speed && !result.is_empty() {
        let last_force = result.last().unwrap().1;
        let v_base = fixed_torque_speed as f64;
        let constant = v_base.powf(coefficient0) * last_force;

        let end_constant_power = constant_power_speed.min(max_speed);
        for i in fixed_torque_speed..end_constant_power {
            let v = i as f64;
            // i=0 によるゼロ割りを防ぐガード
            let force = if v == 0.0 {
                fixed_torque
            } else {
                (constant / v.powf(coefficient0)) * (1.0 - margin_ratio)
            };
            result.push((v, force));
        }
    }

    // 3. 特性領域 [constant_power_speed..max_speed)
    if constant_power_speed < max_speed && !result.is_empty() {
        let last_force = result.last().unwrap().1;
        let v_base = constant_power_speed as f64;
        let constant = v_base.powf(coefficient1) * last_force;

        for i in constant_power_speed..max_speed {
            let v = i as f64;
            let force = if v == 0.0 {
                fixed_torque
            } else {
                (constant / v.powf(coefficient1)) * (1.0 - margin_ratio)
            };
            result.push((v, force));
        }
    }

    result
}

/// 車両オブジェクトに力学データ（引張力・減速度・走行抵抗）をセットする
pub fn set_force_data(mut vehicle: Vehicle) -> Vehicle {
    let max_speed = vehicle.max_speed as usize;

    vehicle.acceleration_force = VelocityForceTable {
        value: default_tractive_force(
            vehicle.startup_acceleration,
            vehicle.fixed_torque_speed as usize,
            vehicle.constant_power_speed as usize,
            max_speed,
            vehicle.m_cars as f64,
            vehicle.t_cars as f64,
            vehicle.m_weight,
            vehicle.t_weight,
            vehicle.coefficient0,
            vehicle.coefficient1,
        ),
    };

    let decel_force_kgf = (vehicle.deceleration / 3.6) * vehicle.train_weight * 1000.0 / 9.807;
    vehicle.deceleration_force = VelocityForceTable {
        value: vec![(0.0, decel_force_kgf)],
    };

    vehicle.running_resist = VelocityForceTable {
        value: default_running_resistance(
            vehicle.m_cars as f64,
            vehicle.t_cars as f64,
            vehicle.m_weight,
            vehicle.t_weight,
            max_speed,
        ),
    };

    vehicle
}