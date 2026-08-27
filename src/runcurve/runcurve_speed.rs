use crate::{
    model::{
        route::{LimitSpeed, Route},
        runcurve::{NotchOperate, NotchType, Runcurve, RuncurveResult},
        vehicle::Vehicle,
    },
    runcurve::route_data::{get_curve_radius, get_gradient, get_tunnel},
};

/// 1m移動した際の等加速度運動による終端速度 [km/h] を計算
///
/// v_end = sqrt(v_start^2 + 2 * a * dx) を基本式とし、
/// 速度を m/s に換算して計算後に km/h へ戻す。
#[inline]
fn calc_next_speed(current_speed_kmh: f64, acceleration_kmhs: f64, dx_m: f64) -> f64 {
    let current_speed_ms = current_speed_kmh / 3.6;
    // 加速度 [km/h/s] を [m/s^2] に変換: a [m/s^2] = a [km/h/s] / 3.6
    let accel_ms2 = acceleration_kmhs / 3.6;

    let speed_sq = current_speed_ms.powi(2) + 2.0 * accel_ms2 * dx_m;

    if speed_sq <= 0.0 {
        0.0
    } else {
        speed_sq.sqrt() * 3.6
    }
}

/// 曲線抵抗 [kgf/t] を算出する (800 / R)
#[inline]
fn curve_resistance(radius: f64) -> f64 {
    if radius != 0.0 { 800.0 / radius } else { 0.0 }
}

/// 惰行時、1m先の速度を求める [km/h]
pub fn get_notch_off_next_speed(
    current_speed: f64,
    vehicle: &Vehicle,
    radius: f64,
    gradient: f64,
    tunnel: f64,
) -> f64 {
    // 走行抵抗 (引張力テーブルから取得) [N] または [kgf] に応じた単位質量あたりの力 [kgf/t]
    let running_resist = vehicle.running_resist.force_at(current_speed);

    // 単位質量あたりの合力 [kgf/t]
    let mut force = -running_resist / vehicle.train_weight;
    force -= tunnel + gradient;
    force -= curve_resistance(radius);

    // 引張力・抵抗 [kgf/t] から加速度 [km/h/s] への換算定数 (30.9)
    let acceleration = force / 30.9;

    calc_next_speed(current_speed, acceleration, 1.0)
}

/// 加速時、1m先の速度を求める [km/h]
pub fn get_accel_next_speed(
    current_speed: f64,
    vehicle: &Vehicle,
    radius: f64,
    gradient: f64,
    tunnel: f64,
) -> f64 {
    let accel_force = vehicle.acceleration_force.force_at(current_speed);
    let running_resist = vehicle.running_resist.force_at(current_speed);

    let mut force = (accel_force - running_resist) / vehicle.train_weight;
    force -= tunnel + gradient;
    force -= curve_resistance(radius);

    let acceleration = force / 30.9;

    calc_next_speed(current_speed, acceleration, 1.0)
}

/// 減速した際の1m前の速度（逆算）を求める [km/h]
///
/// 減速パターン（ブレーキ曲線）を終点から起点に向かって逆算描画する際に使用。
pub fn get_decel_before_speed(
    current_speed: f64,
    vehicle: &Vehicle,
    radius: f64,
    gradient: f64,
    tunnel: f64,
) -> f64 {
    let running_resist = vehicle.running_resist.force_at(current_speed);
    let decel_force = vehicle.deceleration_force.force_at(current_speed);

    let mut force = -running_resist / vehicle.train_weight;
    force += tunnel + gradient + (decel_force / vehicle.train_weight);
    force += curve_resistance(radius);

    let acceleration = force / 30.9;

    calc_next_speed(current_speed, acceleration, 1.0)
}

/// ランカーブ（距離ごとの速度列とノッチ操作履歴）を計算する
pub fn get_runcurve_speed(
    route: &Route,
    vehicle: &Vehicle,
    start_pos: usize,
    end_pos: usize,
    max_speed: f64,
) -> (Vec<f64>, Vec<NotchOperate>) {
    let limit_margin_speed = 2.0;
    let re_acceleration_ratio = 0.85;

    let length = end_pos.saturating_sub(start_pos);
    if length == 0 {
        return (Vec::new(), Vec::new());
    }

    // 1mごとの線路条件・制限速度配列の取得
    let limit_speed_array = get_limit_speed_array(route, vehicle, start_pos, end_pos, max_speed);
    let curve_array = get_curve_radius(route, vehicle, start_pos, end_pos);
    let gradient_array = get_gradient(route, vehicle, start_pos, end_pos);
    let tunnel_array = get_tunnel(route, vehicle, start_pos, end_pos);

    // ブレーキパターン配列
    let brake_pattern_array = get_limit_speed_brake_pattern_array(
        route,
        vehicle,
        start_pos,
        end_pos,
        &limit_speed_array,
        &curve_array,
        &gradient_array,
        &tunnel_array,
    );

    let mut speed_array = vec![0.0; length];
    let mut notch_operates = Vec::new();

    let mut speed = 0.0;
    let mut notch_type = NotchType::Power;

    notch_operates.push(NotchOperate {
        distance: start_pos as f64,
        notch_type: NotchType::Power,
        detail: Some("駅発車力行".to_string()),
    });

    for i in 0..length {
        let current_pos = (start_pos + i) as f64;

        // ----------------------------------------------------
        // 1. ノッチ状態の遷移判定
        // ----------------------------------------------------
        match notch_type {
            NotchType::Power => {
                if i % 10 == 0
                    && get_10s_later_notch_off_speed(
                        route,
                        vehicle,
                        start_pos,
                        end_pos,
                        &limit_speed_array,
                        &curve_array,
                        &gradient_array,
                        &tunnel_array,
                        i,
                        speed,
                    ) > (limit_speed_array[i] - limit_margin_speed)
                {
                    notch_type = NotchType::NotchOff;
                    notch_operates.push(NotchOperate {
                        distance: current_pos,
                        notch_type: NotchType::NotchOff,
                        detail: Some("10秒惰行で目標速度を超過のためノッチオフ".to_string()),
                    });
                }

                if speed > (limit_speed_array[i] - limit_margin_speed) {
                    if get_notch_off_next_speed(
                        speed,
                        vehicle,
                        curve_array[i],
                        gradient_array[i],
                        tunnel_array[i],
                    ) > speed
                    {
                        notch_type = NotchType::Constant;
                        notch_operates.push(NotchOperate {
                            distance: current_pos,
                            notch_type: NotchType::Constant,
                            detail: Some("目標速度超過 かつ 惰行で加速のため抑速".to_string()),
                        });
                    } else {
                        notch_type = NotchType::NotchOff;
                        notch_operates.push(NotchOperate {
                            distance: current_pos,
                            notch_type: NotchType::NotchOff,
                            detail: Some("目標速度超過のためノッチオフ".to_string()),
                        });
                    }
                }

                if let Some(target_idx) = get_brake_pattern_distance(&brake_pattern_array, speed, i)
                {
                    let dist_diff = target_idx as f64 - i as f64;
                    if speed > 0.0 && 3.6 * (dist_diff / speed) < 5.0 {
                        notch_type = NotchType::NotchOff;
                        notch_operates.push(NotchOperate {
                            distance: current_pos,
                            notch_type: NotchType::NotchOff,
                            detail: Some("ブレーキパターンが10秒後に接近".to_string()),
                        });
                    }
                }
            }

            NotchType::NotchOff => {
                let mut approaching_brake = false;
                if let Some(target_idx) = get_brake_pattern_distance(&brake_pattern_array, speed, i)
                {
                    let dist_diff = target_idx as f64 - i as f64;
                    if speed > 0.0 && 3.6 * (dist_diff / speed) < 5.0 {
                        approaching_brake = true;
                    }
                }

                if !approaching_brake {
                    if i % 10 == 0 {
                        let is_10s_over = get_10s_later_notch_off_speed(
                            route,
                            vehicle,
                            start_pos,
                            end_pos,
                            &limit_speed_array,
                            &curve_array,
                            &gradient_array,
                            &tunnel_array,
                            i,
                            speed,
                        ) > (limit_speed_array[i] - limit_margin_speed);

                        if limit_speed_array[i] * re_acceleration_ratio > speed && !is_10s_over {
                            notch_type = NotchType::Power;
                            notch_operates.push(NotchOperate {
                                distance: current_pos,
                                notch_type: NotchType::Power,
                                detail: Some("車速が目標速度の一定速度以下のため力行".to_string()),
                            });
                        }
                    }

                    if speed > (limit_speed_array[i] - limit_margin_speed)
                        && get_notch_off_next_speed(
                            speed,
                            vehicle,
                            curve_array[i],
                            gradient_array[i],
                            tunnel_array[i],
                        ) > speed
                    {
                        notch_type = NotchType::Constant;
                        notch_operates.push(NotchOperate {
                            distance: current_pos,
                            notch_type: NotchType::Constant,
                            detail: Some("目標速度超過 かつ 惰行で加速のため抑速".to_string()),
                        });
                    }
                }

                if brake_pattern_array[i] != -1.0 && speed > brake_pattern_array[i] {
                    notch_type = NotchType::Brake;
                    notch_operates.push(NotchOperate {
                        distance: current_pos,
                        notch_type: NotchType::Brake,
                        detail: Some("制動パターン超過のため制動".to_string()),
                    });
                }
            }

            NotchType::Constant => {
                if i > 0 && limit_speed_array[i - 1] < limit_speed_array[i] {
                    notch_type = NotchType::Power;
                    notch_operates.push(NotchOperate {
                        distance: current_pos,
                        notch_type: NotchType::Power,
                        detail: Some("制限が更新された".to_string()),
                    });
                } else {
                    if limit_speed_array[i] * re_acceleration_ratio > speed {
                        notch_type = NotchType::Power;
                        notch_operates.push(NotchOperate {
                            distance: current_pos,
                            notch_type: NotchType::Power,
                            detail: Some("車速が目標速度の一定速度以下のため力行".to_string()),
                        });
                    }

                    if speed > (limit_speed_array[i] - limit_margin_speed) {
                        if get_notch_off_next_speed(
                            speed,
                            vehicle,
                            curve_array[i],
                            gradient_array[i],
                            tunnel_array[i],
                        ) <= speed
                        {
                            notch_type = NotchType::NotchOff;
                            notch_operates.push(NotchOperate {
                                distance: current_pos,
                                notch_type: NotchType::NotchOff,
                                detail: Some("目標速度超過 かつ 惰行で加速のため抑速".to_string()),
                            });
                        }
                    }

                    if brake_pattern_array[i] != -1.0 && speed > brake_pattern_array[i] {
                        notch_type = NotchType::Brake;
                        notch_operates.push(NotchOperate {
                            distance: current_pos,
                            notch_type: NotchType::Brake,
                            detail: Some("制動パターン超過のため制動".to_string()),
                        });
                    }
                }
            }

            NotchType::Brake => {
                let next_pattern = if i + 1 < length {
                    brake_pattern_array[i + 1]
                } else {
                    -1.0
                };

                if next_pattern
                    > get_notch_off_next_speed(
                        speed,
                        vehicle,
                        curve_array[i],
                        gradient_array[i],
                        tunnel_array[i],
                    )
                {
                    notch_type = NotchType::NotchOff;
                    notch_operates.push(NotchOperate {
                        distance: current_pos,
                        notch_type: NotchType::NotchOff,
                        detail: Some("惰行時 制動パターンより車速が低いためノッチオフ".to_string()),
                    });
                }
            }
        }

        // ----------------------------------------------------
        // 2. 確定したノッチに基づく速度更新と状態確定
        // ----------------------------------------------------
        match notch_type {
            NotchType::Power => {
                speed = get_accel_next_speed(
                    speed,
                    vehicle,
                    curve_array[i],
                    gradient_array[i],
                    tunnel_array[i],
                );
                speed_array[i] = speed;
            }
            NotchType::NotchOff => {
                speed = get_notch_off_next_speed(
                    speed,
                    vehicle,
                    curve_array[i],
                    gradient_array[i],
                    tunnel_array[i],
                );
                speed_array[i] = speed;
            }
            NotchType::Constant => {
                let next_notch_off_speed = get_notch_off_next_speed(
                    speed,
                    vehicle,
                    curve_array[i],
                    gradient_array[i],
                    tunnel_array[i],
                );

                if next_notch_off_speed < speed {
                    notch_type = NotchType::NotchOff;
                    notch_operates.push(NotchOperate {
                        distance: current_pos,
                        notch_type: NotchType::NotchOff,
                        detail: Some("目標速度超過 かつ 惰行で加速のため抑速".to_string()),
                    });
                    speed = next_notch_off_speed;
                }
                speed_array[i] = speed;
            }
            NotchType::Brake => {
                if brake_pattern_array[i] == -1.0 {
                    if get_notch_off_next_speed(
                        speed,
                        vehicle,
                        curve_array[i],
                        gradient_array[i],
                        tunnel_array[i],
                    ) > speed
                    {
                        notch_type = NotchType::Constant;
                        notch_operates.push(NotchOperate {
                            distance: current_pos,
                            notch_type: NotchType::Constant,
                            detail: Some("目標速度超過 かつ 惰行で加速のため抑速".to_string()),
                        });
                    } else {
                        notch_type = NotchType::NotchOff;
                        notch_operates.push(NotchOperate {
                            distance: current_pos,
                            notch_type: NotchType::NotchOff,
                            detail: Some("目標速度超過のためノッチオフ".to_string()),
                        });
                        speed = get_notch_off_next_speed(
                            speed,
                            vehicle,
                            curve_array[i],
                            gradient_array[i],
                            tunnel_array[i],
                        );
                    }
                    speed_array[i] = speed;
                } else {
                    speed = brake_pattern_array[i];
                    speed_array[i] = speed;
                }
            }
        }
    }

    (speed_array, notch_operates)
}

/// 1mごとの速度配列から累積経過時間 [s] の配列を算出する
pub fn get_runcurve_time(speed_array: &[f64]) -> Vec<f64> {
    let mut result = vec![0.0; speed_array.len()];
    let mut current_time = 0.0;

    for (i, &speed) in speed_array.iter().enumerate() {
        if speed == 0.0 {
            result[i] = current_time;
            continue;
        }
        // Δt = Δx / v = 1m / (v / 3.6 m/s) = 3.6 / v [s]
        current_time += 3.6 / speed;
        result[i] = current_time;
    }

    result
}

/// ランカーブ（速度・時間・位置・ノッチ操作）の総合算出
pub fn get_runcurve_speed_and_time(
    route: &Route,
    vehicle: &Vehicle,
    start_pos: usize,
    end_pos: usize,
    max_speed: f64,
) -> RuncurveResult {
    let (speed_array, notches) = get_runcurve_speed(route, vehicle, start_pos, end_pos, max_speed);
    let time_array = get_runcurve_time(&speed_array);

    let runcurve_array = speed_array
        .iter()
        .zip(time_array.iter())
        .enumerate()
        .map(|(i, (&speed, &time))| Runcurve {
            distance: (i + start_pos) as f64,
            speed,
            time,
        })
        .collect();

    RuncurveResult {
        notches,
        runcurve_array,
    }
}

/// 制限速度の1mごとの配列を生成
pub fn get_limit_speed_array(
    route: &Route,
    _vehicle: &Vehicle,
    start_pos: usize,
    end_pos: usize,
    max_speed: f64,
) -> Vec<f64> {
    let length = end_pos.saturating_sub(start_pos);
    let mut result = vec![max_speed; length];

    // 速度が大きい順にソートして適用（厳しい制限速度で上書きするため）
    let mut sorted_limits = route.limit_speeds.clone();
    sorted_limits.sort_by(|a, b| {
        b.speed
            .partial_cmp(&a.speed)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    for limit_speed in &sorted_limits {
        let ls_start = limit_speed.start as usize;
        let ls_end = limit_speed.end as usize;

        // 制限速度区間が計算範囲外の場合はスキップ
        if end_pos < ls_start || ls_end < start_pos {
            continue;
        }

        let curve_start = ls_start.max(start_pos) - start_pos;
        let curve_end = ls_end.min(end_pos) - start_pos;

        for i in curve_start..curve_end {
            if i < result.len() {
                result[i] = limit_speed.speed;
            }
        }
    }

    result
}

/// 制限速度および終点停車へのブレーキパターン配列（1mごと）を生成
pub fn get_limit_speed_brake_pattern_array(
    route: &Route,
    vehicle: &Vehicle,
    start_pos: usize,
    end_pos: usize,
    limit_speed_array: &[f64],
    curve_array: &[f64],
    gradient_array: &[f64],
    tunnel_array: &[f64],
) -> Vec<f64> {
    let length = end_pos.saturating_sub(start_pos);
    let mut result = vec![-1.0; length];
    let limit_margin_speed = 2.0;

    // 既存の制限速度リストに、終点位置の停止条件 (speed = 0) を追加
    let mut limit_speeds = route.limit_speeds.clone();
    limit_speeds.push(LimitSpeed {
        start: end_pos.saturating_sub(1) as f64,
        end: end_pos as f64,
        speed: 0.0,
    });

    for limit_speed in &limit_speeds {
        let ls_start = limit_speed.start as usize;

        if ls_start < start_pos || end_pos < ls_start {
            continue;
        }

        let mut i = ls_start - start_pos; // 開始インデックス
        let mut speed = (limit_speed.speed - limit_margin_speed).max(0.0);

        if i < result.len() {
            result[i] = speed;
        }

        while i > 0 {
            if speed >= limit_speed_array[i] {
                break;
            }

            i -= 1;
            speed = get_decel_before_speed(
                speed,
                vehicle,
                curve_array[i],
                gradient_array[i],
                tunnel_array[i],
            );

            if result[i] == -1.0 {
                result[i] = speed;
            } else if result[i] > speed {
                // より厳しい（低い）ブレーキパターンを採用
                result[i] = speed;
            }
        }
    }

    result
}

/// 惰行で10秒間走った時の予測到達速度 [km/h] を計算する
pub fn get_10s_later_notch_off_speed(
    _route: &Route,
    vehicle: &Vehicle,
    _start_pos: usize,
    _end_pos: usize,
    limit_speed_array: &[f64],
    curve_array: &[f64],
    gradient_array: &[f64],
    tunnel_array: &[f64],
    mut index: usize,
    mut current_speed: f64,
) -> f64 {
    if current_speed == 0.0 {
        return 0.0;
    }

    let mut total_time = 0.0;

    loop {
        if limit_speed_array.len() <= index {
            return 0.0;
        }

        current_speed = get_notch_off_next_speed(
            current_speed,
            vehicle,
            curve_array[index],
            gradient_array[index],
            tunnel_array[index],
        );

        if current_speed <= 0.0 {
            return 0.0;
        }

        // Δt = 3.6 / v [s] (1m移動にかかる時間)
        total_time += 3.6 / current_speed;

        if total_time > 10.0 {
            return current_speed;
        }

        index += 1;
    }
}

/// 現在速度がブレーキパターンと交差（接触）するインデックス（距離位置）を検索する
///
/// パターンが存在し、かつ速度がパターンを下回る（交差する）位置を見つけた場合は `Some(usize)` を返す
pub fn get_brake_pattern_distance(
    brake_pattern_array: &[f64],
    current_speed: f64,
    mut index: usize,
) -> Option<usize> {
    if brake_pattern_array.is_empty() {
        return None;
    }

    while index < brake_pattern_array.len() - 1 {
        if brake_pattern_array[index] == -1.0 || brake_pattern_array[index + 1] == -1.0 {
            break;
        }

        // 現在速度が [index] と [index + 1] の間でブレーキパターン線をまたいだか判定
        if brake_pattern_array[index + 1] < current_speed
            && current_speed < brake_pattern_array[index]
        {
            return Some(index);
        }

        index += 1;
    }

    None
}
