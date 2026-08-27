use crate::model::route::{Route, TunnelType};
use crate::model::vehicle::Vehicle;

/// 路線上の1mごとの曲線半径 [m] を求める関数（曲線がない区間は 0.0）
pub fn get_curve_radius(route: &Route, _vehicle: &Vehicle, start: usize, end: usize) -> Vec<f64> {
    let length = end.saturating_sub(start);
    let mut result = vec![0.0; length];

    for curve in &route.curves {
        let curve_start = curve.start as usize;
        let curve_end = curve.end as usize;

        // 曲線が計算範囲外の場合はスキップ
        if end < curve_start || curve_end < start {
            continue;
        }

        let slice_start = curve_start.max(start) - start;
        let slice_end = curve_end.min(end) - start;

        for i in slice_start..slice_end {
            if i < result.len() {
                result[i] = curve.radius;
            }
        }
    }

    result
}

/// 路線上の1mごとの勾配値 [‰] を求める関数（車両編成長による平滑化処理処理を含む）
pub fn get_gradient(route: &Route, vehicle: &Vehicle, start: usize, end: usize) -> Vec<f64> {
    let length = end.saturating_sub(start);
    let mut result = vec![0.0; length];

    // 距離順にソート（位置順に適用していくため）
    let mut gradients = route.gradients.clone();
    gradients.sort_by(|a, b| {
        a.position
            .partial_cmp(&b.position)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    for gradient in &gradients {
        let grad_pos = gradient.position as usize;

        if end < grad_pos {
            continue;
        }

        let gradient_start = grad_pos.max(start) - start;

        for i in gradient_start..result.len() {
            result[i] = gradient.value;
        }
    }

    // 車両編成長（trainLength）に応じた平均勾配の平滑化（移動平均）
    let mut result_blur = vec![0.0; length];
    let half_length = (vehicle.train_length / 2.0).round() as i64;
    let total_length = result.len() as i64;

    for i in 0..result.len() {
        let mut sum = 0.0;

        for j in -half_length..half_length {
            let mut index = i as i64 + j;
            if index < 0 {
                index = 0;
            } else if index >= total_length {
                index = total_length - 1;
            }

            sum += result[index as usize];
        }

        result_blur[i] = sum / vehicle.train_length;
    }

    result_blur
}

/// 路線上の1mごとのトンネル種別を求める関数（0: なし, 1: 複線, 2: 単線）
pub fn get_tunnel(route: &Route, _vehicle: &Vehicle, start: usize, end: usize) -> Vec<f64> {
    let length = end.saturating_sub(start);
    let mut result = vec![0.0; length];

    for tunnel in &route.tunnels {
        let tunnel_start = tunnel.start as usize;
        let tunnel_end = tunnel.end as usize;

        // トンネルが計算範囲外の場合はスキップ
        if end < tunnel_start || tunnel_end < start {
            continue;
        }

        let slice_start = tunnel_start.max(start) - start;
        let slice_end = tunnel_end.min(end) - start;

        // トンネルタイプに応じた値設定（double -> 1.0, 単線等その他 -> 2.0）
        let tunnel_value = if tunnel.tunnel_type == TunnelType::Double {
            1.0
        } else {
            2.0
        };

        for i in slice_start..slice_end {
            if i < result.len() {
                result[i] = tunnel_value;
            }
        }
    }

    result
}
