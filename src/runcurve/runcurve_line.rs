use crate::model::route::{Route, StopPosition};
use crate::model::runcurve::RuncurveResult;
use crate::model::vehicle::Vehicle;
use crate::runcurve::runcurve_speed::get_runcurve_speed_and_time;

/// 駅間所要時間結果
#[derive(Debug, Clone)]
pub struct TimeResult {
    pub from_station: StopPosition,
    pub to_station: StopPosition,
    pub time: f64,
}

/// 路線全体の停車駅間ごとにランカーブを計算する
pub fn get_runcurve_line(route: &Route, vehicle: &Vehicle, max_speed: f64) -> Vec<RuncurveResult> {
    // 通過駅（isPass == true）を除外した停車駅リストを抽出
    let stop_positions: Vec<&StopPosition> =
        route.stop_positions.iter().filter(|v| !v.is_pass).collect();

    if stop_positions.len() < 2 {
        return Vec::new();
    }

    // 隣り合う2駅（窓幅2のウィンドウ）ごとにランカーブを算出
    stop_positions
        .windows(2)
        .map(|pair| {
            let start = pair[0].position as usize;
            let end = pair[1].position as usize;
            get_runcurve_speed_and_time(route, vehicle, start, end, max_speed)
        })
        .collect()
}

/// 各駅間（通過駅による途中分割を含む）の所要時間を算出する
pub fn get_runcurve_line_time(
    route: &Route,
    runcurves: &[RuncurveResult],
) -> Vec<TimeResult> {
    let mut result = Vec::new();

    for rc in runcurves {
        let mut split_times = split_time(rc, route);
        result.append(&mut split_times);
    }

    // 出発駅の位置（position）順にソート
    result.sort_by(|a, b| {
        a.from_station
            .position
            .partial_cmp(&b.from_station.position)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    result
}

/// 単一のランカーブ結果から、通過駅等を含めた各区間の所要時間を分割抽出する
fn split_time(runcurve: &RuncurveResult, route: &Route) -> Vec<TimeResult> {
    if runcurve.runcurve_array.is_empty() {
        return Vec::new();
    }

    let start = runcurve.runcurve_array.first().unwrap().distance;
    let end = runcurve.runcurve_array.last().unwrap().distance;

    // 計算範囲内（start - 1 〜 end + 1）に存在するすべての駅（通過駅含む）を取得
    let split_distances: Vec<StopPosition> = route
        .stop_positions
        .iter()
        .filter(|v| start <= v.position - 1.0 && v.position <= end + 1.0)
        .cloned()
        .collect();

    // 途中に通過駅等がない場合（直通区間）
    if split_distances.len() < 2 {
        let from_station = route
            .stop_positions
            .iter()
            .find(|v| (v.position - start).abs() < f64::EPSILON)
            .cloned()
            .unwrap_or_default();

        let to_station = route
            .stop_positions
            .iter()
            .find(|v| (v.position - (end + 1.0)).abs() < f64::EPSILON)
            .cloned()
            .unwrap_or_default();

        let time = runcurve.runcurve_array.last().map_or(0.0, |rc| rc.time);

        return vec![TimeResult {
            from_station,
            to_station,
            time,
        }];
    }

    let mut result = Vec::new();

    // 始点駅から最初の分割点までの所要時間
    let first_from_station = route
        .stop_positions
        .iter()
        .find(|v| (v.position - start).abs() < f64::EPSILON)
        .cloned()
        .unwrap_or_default();

    let first_time = runcurve
        .runcurve_array
        .iter()
        .find(|v| (v.distance - split_distances[0].position).abs() < f64::EPSILON)
        .map_or(0.0, |v| v.time);

    result.push(TimeResult {
        from_station: first_from_station,
        to_station: split_distances[0].clone(),
        time: first_time,
    });

    // 途中の通過駅間の所要時間差分を計算
    for i in 1..split_distances.len() {
        let before = &split_distances[i - 1];
        let current = &split_distances[i];

        let before_time = runcurve
            .runcurve_array
            .iter()
            .find(|v| (v.distance - before.position).abs() < f64::EPSILON)
            .map_or(0.0, |v| v.time);

        let current_time = runcurve
            .runcurve_array
            .iter()
            .find(|v| (v.distance - (current.position - 1.0)).abs() < f64::EPSILON)
            .map_or(0.0, |v| v.time);

        result.push(TimeResult {
            from_station: before.clone(),
            to_station: current.clone(),
            time: current_time - before_time,
        });
    }

    result
}
