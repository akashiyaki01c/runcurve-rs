use crate::model::{
    route::{
        Curve,
        CurveDirection::{Left, Right},
        Gradient, LimitSpeed, Route, StopPosition,
    },
    vehicle::{Vehicle, VelocityForceTable, default_running_resistance, default_tractive_force},
};

/// テスト用の路線データ（テスト線）を取得する
pub fn get_test_route() -> Route {
    Route {
        name: "テスト線".to_string(),
        gradients: vec![
            Gradient {
                position: 1000.0,
                value: 35.0,
            },
            Gradient {
                position: 2000.0,
                value: -35.0,
            },
            Gradient {
                position: 2500.0,
                value: 0.0,
            },
        ],
        curves: vec![
            Curve {
                start: 1200.0,
                end: 1400.0,
                direction: Left,
                radius: 600.0,
                speed: 0.0,
            },
            Curve {
                start: 2400.0,
                end: 2600.0,
                direction: Right,
                radius: 600.0,
                speed: 0.0,
            },
        ],
        tunnels: vec![],
        limit_speeds: vec![LimitSpeed {
            start: 12100.0 - 100.0,
            end: 12100.0 + 20.0,
            speed: 45.0,
        }],
        stop_positions: vec![
            StopPosition {
                position: 0.0,
                station_name: "A駅".to_string(),
                track_name: "1番線".to_string(),
                is_pass: false,
            },
            StopPosition {
                position: 1800.0,
                station_name: "B駅".to_string(),
                track_name: "1番線".to_string(),
                is_pass: true,
            },
            StopPosition {
                position: 3000.0,
                station_name: "C駅".to_string(),
                track_name: "1番線".to_string(),
                is_pass: false,
            },
        ],
    }
}

/// テスト用の車両データ（テスト編成）を取得する関数
pub fn get_test_vehicle() -> Vehicle {
    let m_cars = 3;
    let t_cars = 3;
    let m_weight = 42.1;
    let t_weight = 33.4;
    let max_speed = 100.0;
    let startup_acceleration = 2.5;
    let fixed_torque_speed = 60.0;
    let constant_power_speed = 80.0;
    let coefficient0 = 1.0;
    let coefficient1 = 2.0;
    let deceleration = 2.5;
    let train_weight = 184.4;

    let max_speed_idx = max_speed as usize;

    Vehicle {
        name: "テスト編成".to_string(),
        max_speed,
        number_of_cars: 6,
        train_length: 19.5 * 6.0,
        train_weight,
        capacity: 808,
        human_weight: 55.0,
        occupancy_rate: 100.0,
        unit_count: 3,

        startup_acceleration,
        fixed_torque_speed,
        constant_power_speed,
        m_cars,
        t_cars,
        m_weight,
        t_weight,
        coefficient0,
        coefficient1,

        deceleration,

        acceleration_force: VelocityForceTable {
            value: default_tractive_force(
                startup_acceleration,
                fixed_torque_speed as usize,
                constant_power_speed as usize,
                max_speed_idx,
                m_cars as f64,
                t_cars as f64,
                // 元コードの引数順序（m_weight, t_weight）に合わせて評価
                t_weight, // 注: TS側でm_weightの位置にt_weightが渡されている元コードの挙動を再現する場合は数値を調整
                m_weight,
                coefficient0,
                coefficient1,
            ),
        },
        deceleration_force: VelocityForceTable {
            value: vec![(0.0, (deceleration / 3.6) * train_weight * 1000.0 / 9.807)],
        },
        running_resist: VelocityForceTable {
            value: default_running_resistance(
                m_cars as f64,
                t_cars as f64,
                t_weight,
                m_weight,
                max_speed_idx,
            ),
        },
    }
}
