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
/// 基本式: v_end = sqrt(v_start^2 + 2 * a * dx)
/// 速度を m/s に換算して計算後に km/h へ戻す。
///
/// # 引数
///
/// - `current_speed_kmh`: 現在の速度 [km/h]
/// - `acceleration_kmhs`: 加速度 [km/h/s]
/// - `dx_m`: 移動距離 [m]
///
/// # 戻り値
///
/// 1m 移動した際の等加速度運動による終端速度 [km/h]
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
///
/// # 引数
///
/// - `radius`: 曲線半径 [m]
///
/// # 戻り値
///
/// 曲線抵抗 [kgf/t]。半径が 0.0 の場合は 0.0 を返します。
#[inline]
fn curve_resistance(radius: f64) -> f64 {
    if radius != 0.0 { 800.0 / radius } else { 0.0 }
}

/// 惰行時、1m先の速度を求める [km/h]
///
/// ノッチオフ状態（惰行走行）で、現在の速度から 1m 進んだ位置での速度を求めます。
/// 走行抵抗・勾配抵抗・曲線抵抗による減速を考慮します。
///
/// # 引数
///
/// - `current_speed`: 現在の速度 [km/h]
/// - `vehicle`: 車両データ
/// - `radius`: 曲線半径 [m]（曲線がない区間では 0.0）
/// - `gradient`: 勾配値 [‰]（上り正、下り負）
/// - `tunnel`: トンネル抵抗値
///
/// # 戻り値
///
/// 1m 先の速度 [km/h]
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
///
/// 加速走行時に、現在の速度から 1m 進んだ位置での速度を求めます。
/// 引張力と走行抵抗・勾配抵抗・曲線抵抗の影響を考慮します。
///
/// # 引数
///
/// - `current_speed`: 現在の速度 [km/h]
/// - `vehicle`: 車両データ
/// - `radius`: 曲線半径 [m]（曲線がない区間では 0.0）
/// - `gradient`: 勾配値 [‰]（上り正、下り負）
/// - `tunnel`: トンネル抵抗値
///
/// # 戻り値
///
/// 1m 先の速度 [km/h]
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
///
/// # 引数
///
/// - `current_speed`: 現在の速度 [km/h]
/// - `vehicle`: 車両データ
/// - `radius`: 曲線半径 [m]（曲線がない区間では 0.0）
/// - `gradient`: 勾配値 [‰]（上り正、下り負）
/// - `tunnel`: トンネル抵抗値
///
/// # 戻り値
///
/// 1m 前の速度（逆算値） [km/h]
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
///
/// 指定された区間について、1m ごとの速度とノッチ操作履歴を計算します。
/// 制限速度・勾配・曲線・トンネルなどの路線を考慮します。
///
/// # 引数
///
/// - `route`: 路線データ
/// - `vehicle`: 車両データ
/// - `start_pos`: 開始位置 [m]
/// - `end_pos`: 終了位置 [m]
/// - `max_speed`: 最高速度 [km/h]
///
/// # 戻り値
///
/// タプル (速度配列 [km/h], ノッチ操作履歴のベクタ)
/// - 速度配列: 各位置での速度 [km/h]
/// - ノッチ操作履歴: ノッチ操作の位置・種別・詳細
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
///
/// 速度配列から各位置での累積経過時間を計算します。
///
/// # 引数
///
/// - `speed_array`: 各1m区間での速度配列 [km/h]
///
/// # 戻り値
///
/// 各位置での累積経過時間 [s] のベクタ
///
/// # 計算式
///
/// Δt = Δx / v = 1m / (v / 3.6 m/s) = 3.6 / v [s]
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
///
/// 速度配列、ノッチ操作履歴、累積時間配列を組み合わせて、
/// 位置・速度・時間・ノッチ操作を含む完全なランカーブ結果を生成します。
///
/// # 引数
///
/// - `route`: 路線データ
/// - `vehicle`: 車両データ
/// - `start_pos`: 計算開始位置 [m]
/// - `end_pos`: 計算終了位置 [m]
/// - `max_speed`: 最高速度 [km/h]
///
/// # 戻り値
///
/// ランカーブ計算結果。各位置での距離・速度・時間、およびノッチ操作履歴を含みます。
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
///
/// 指定された区間について、路線データの制限速度情報を参照し、
/// 各1m区間ごとの制限速度を計算します。制限速度が設定されていない区間は
/// `max_speed` 引数の値が使用されます。
///
/// # 引数
///
/// - `route`: 路線データ
/// - `_vehicle`: 車両データ（未使用）
/// - `start_pos`: 計算開始位置 [m]
/// - `end_pos`: 計算終了位置 [m]
/// - `max_speed`: 最高速度 [km/h]（制限速度が設定されていない区間で使用）
///
/// # 戻り値
///
/// 各1m区間の制限速度 [km/h] のベクタ
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
///
/// 各制限速度の開始点から終点停車までのブレーキパターン（速度曲線）を逆算し、
/// 1m区間ごとにブレーキパターン上の速度を求めます。
/// 制限速度の変化点ごとに減速パターンが計算され、最も厳しい（低い）パターンが採用されます。
///
/// # 引数
///
/// - `route`: 路線データ
/// - `vehicle`: 車両データ
/// - `start_pos`: 計算開始位置 [m]
/// - `end_pos`: 計算終了位置 [m]
/// - `limit_speed_array`: 1mごとの制限速度 [km/h]
/// - `curve_array`: 1mごとの曲線半径 [m]
/// - `gradient_array`: 1mごとの勾配値 [‰]
/// - `tunnel_array`: 1mごとのトンネル種別
///
/// # 戻り値
///
/// 各1m区間のブレーキパターン上の速度 [km/h] のベクタ。
/// パターンがない区間は -1.0。
///
/// # 計算の流れ
///
/// 1. 終点位置に停止条件 (speed = 0) を追加
/// 2. 各制限速度の開始点から `get_decel_before_speed` で逆算
/// 3. 複数の制限速度が重なる場合、最も厳しい（低い）パターンを採用
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
///
/// ノッチオフ（惰行）状態から10秒間走行した場合の予測到達速度を求めます。
/// ノッチオフ遷移の判定などに使用されます。
///
/// # 引数
///
/// - `_route`: 路線データ（未使用）
/// - `vehicle`: 車両データ
/// - `_start_pos`: 計算開始位置 [m]（未使用）
/// - `_end_pos`: 計算終了位置 [m]（未使用）
/// - `limit_speed_array`: 1mごとの制限速度 [km/h]
/// - `curve_array`: 1mごとの曲線半径 [m]
/// - `gradient_array`: 1mごとの勾配値 [‰]
/// - `tunnel_array`: 1mごとのトンネル種別
/// - `index`: 開始インデックス
/// - `current_speed`: 現在の速度 [km/h]
///
/// # 戻り値
///
/// 10秒間惰行走行後の予測到達速度 [km/h]。
/// 速度が0以下になった場合、または配列の末尾に達した場合は 0.0。
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
/// ブレーキパターン配列と現在速度を比較し、現在速度がパターンと交差する
/// （パターン線をまたぐ）位置のインデックスを検索します。
///
/// # 引数
///
/// - `brake_pattern_array`: 1mごとのブレーキパターン上の速度 [km/h]（-1.0 はパターンなし）
/// - `current_speed`: 現在速度 [km/h]
/// - `index`: 検索開始インデックス
///
/// # 戻り値
///
/// 交差位置のインデックスを `Some(usize)` で返します。
/// パターンがない区間 (-1.0) で中断された場合、
/// または交差位置が見つからなかった場合は `None`。
///
/// # 交差判定
///
/// 現在速度が区間 [index, index+1] で以下を満たす場合に交差と判定:
/// - `brake_pattern_array[index + 1] < current_speed < brake_pattern_array[index]`
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
