use itertools::Itertools;
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
use runcurve_rs::{
    model::{
        route::{CurveDirection, Gradient, LimitSpeed, Route},
        runcurve::RuncurveResult,
    },
    runcurve::runcurve_line::{get_runcurve_line, get_runcurve_line_time},
    svg::to_pdf,
    testdata::get_test_vehicle,
};

fn main() {
    let routes = vec![
        "001_hanshin",
        "003_hanshin_new",
        "004_juso",
        "005_kaigan",
        "006_mukogawa",
        "007_mukogawa_renraku",
        "011_amaho",
        "012_arima",
        "021_nishiosaka",
        "031_shinki",
        "033_fukusaki",
        "034_taisanji",
        "035_kato",
        "036_meimi",
        "037_okayama",
        "038_mitsu",
        "041_seiban",
        "043_yamasaki",
        "044_himeji",
    ];
    
    routes.par_iter().for_each(|route| {
        calc_wrapper(route, false);
        calc_wrapper(route, true);
    });

}

fn calc_wrapper(route: &str, is_reversed: bool) {
    let timer = std::time::Instant::now();
    let direction =  if is_reversed {"up"} else {"down"};

    calc(
        &format!("./base-data/{route}.json"),
        &format!("./base-data/{route}.tsv"),
        &format!("./output/{route}-{direction}.tsv"),
        &format!("./output/{route}-{direction}"),
        is_reversed,
    );
    let end = timer.elapsed();
    println!("[{}-{}] {}ms", route, direction, end.as_millis());
}

fn calc(
    route_path: &str,
    train_type_path: &str,
    output_tsv_path: &str,
    output_pdf_path: &str,
    is_reversed: bool,
) {
    let route = std::fs::read_to_string(route_path).unwrap();
    let mut route = serde_json::from_str::<Route>(&route).unwrap();

    if is_reversed {
        route = reverse_route(route);
    }

    // [前処理] 曲線半径に対応する速度制限を追加
    for curve in route.curves.clone() {
        if let Some(speed) = get_curve_limit_speed(curve.radius) {
            route.limit_speeds.push(LimitSpeed {
                start: curve.start,
                end: curve.end,
                speed,
            });
        }
    }
    // [前処理] 下り勾配に対応する速度制限を追加
    for (gradient, next) in std::iter::once(Gradient {
        position: 0.0,
        value: 0.0,
    })
    .chain(route.gradients.clone())
    .tuple_windows()
    {
        let gradient = gradient;
        let next = next;
        if let Some(speed) = get_gradient_limit_speed(gradient.value) {
            route.limit_speeds.push(LimitSpeed {
                start: gradient.position,
                end: next.position,
                speed,
            });
        }
    }
    let vehicle = get_test_vehicle();

    #[derive(Debug)]
    struct TrainType {
        pub name: String,
        pub limit_speeds: Vec<LimitSpeed>,
        pub is_pass: Vec<(String, bool, String)>,
    }

    // 列車種別データ読み込み
    let mut train_types: Vec<TrainType> = vec![];
    for (i, row) in std::fs::read_to_string(train_type_path)
        .unwrap()
        .split("\n")
        .enumerate()
    {
        if i == 0 {
            for r in row.split("\t").skip(1) {
                train_types.push(TrainType {
                    name: r.to_string(),
                    limit_speeds: vec![],
                    is_pass: vec![],
                })
            }
        } else if i == 1 {
            for (i, r) in row.split("\t").skip(1).enumerate() {
                let train_type = train_types.get_mut(i).unwrap();
                train_type.name += "-";
                train_type.name += r;
            }
        } else {
            let (sta_name, _) = row.split_once("\t").unwrap();

            let sta = route
                .stop_positions
                .iter()
                .find(|v| v.station_name == sta_name)
                .unwrap();
            for (i, r) in row.split("\t").skip(1).enumerate() {
                let train_type = train_types.get_mut(i).unwrap();
                train_type.is_pass.push((
                    sta_name.to_string(),
                    r.is_empty() || r == "レ",
                    r.to_string(),
                ));
                for s in get_signal_limit_speeds(r, sta.position) {
                    train_type.limit_speeds.push(s);
                }
            }
        }
    }

    // 列車種別データの前後空白を削除
    for train_type in &mut train_types {
        let first = train_type.is_pass.iter().find_position(|v| !v.1).unwrap().0;
        let end = train_type.is_pass.len()
            - train_type
                .is_pass
                .iter()
                .rev()
                .find_position(|v| !v.1)
                .unwrap()
                .0
            - 1;
        train_type.is_pass = train_type.is_pass[first..=end]
            .iter()
            .map(|v| v.clone())
            .collect();
    }

    let mut result: Vec<Vec<RuncurveResult>> = vec![];
    for train_type in &train_types {
        let mut route = route.clone();
        for lim in &train_type.limit_speeds {
            route.limit_speeds.push(lim.clone());
        }
        for sta in &train_type.is_pass {
            let pos = route
                .stop_positions
                .iter_mut()
                .find(|v| v.station_name == sta.0)
                .unwrap();
            pos.is_pass = sta.1;
        }
        let res = get_runcurve_line(&route, &vehicle, 120.0);
        result.push(res);
    }

    for (i, r) in result.iter().enumerate() {
        let pdf = to_pdf(&route, &vehicle, &[r.to_vec()], 120.0).unwrap();
        let _ = std::fs::write(&format!("{output_pdf_path}_{i:?}.pdf"), pdf);
    }

    let result: Vec<_> = result
        .iter()
        .map(|v| get_runcurve_line_time(&route, &v))
        .collect();

    // 区間時間の一覧を出力
    let mut str = "\t".to_string();
    for r in result.first().unwrap() {
        str += &format!(
            "{}→{}\t",
            r.from_station.station_name, r.to_station.station_name
        );
    }
    str += "\n";
    for (i, r) in result.iter().enumerate() {
        str += &train_types[i].name;
        str += "\t";
        for r in r {
            str += &format!("{}\t", r.time.to_string());
        }
        str += "\n";
    }
    let _ = std::fs::write("./test.tsv", str);

    // 駅ごとの着発時刻を含む時刻表形式で出力
    use std::fmt::Write;

    let mut output = String::new();
    for train_type in &train_types {
        let mut names = train_type.name.splitn(2, '-');
        let group = names.next().unwrap_or_default();
        let service = names.next().unwrap_or_default();
        write!(output, "\t{}\t{}\t\t\t", service, group).unwrap();
    }
    output.push('\n');

    output.push_str("駅名\t停車\t計算時秒\t運転時秒\t着時刻\t発時刻");
    for _ in train_types.iter().skip(1) {
        output.push_str("\t停車\t計算時秒\t運転時秒\t着時刻\t発時刻");
    }
    output.push('\n');

    // 各列車種別の走行区間に含まれる駅を、路線上の順番で出力する。
    for station in &route.stop_positions {
        if !train_types.iter().any(|train_type| {
            train_type
                .is_pass
                .iter()
                .any(|v| v.0 == station.station_name)
        }) {
            continue;
        }

        output.push_str(&station.station_name);
        for (train_index, train_type) in train_types.iter().enumerate() {
            let Some(station_index) = train_type
                .is_pass
                .iter()
                .position(|v| v.0 == station.station_name)
            else {
                output.push_str("\t\t\t\t\t");
                continue;
            };

            let first = station_index == 0;
            let last = station_index + 1 == train_type.is_pass.len();
            let passed = train_type.is_pass[station_index].1;
            let incoming = if first {
                None
            } else {
                result[train_index]
                    .iter()
                    .find(|v| v.to_station.station_name == station.station_name)
            };

            let (calculated, running, arrival, departure) = if let Some(incoming) = incoming {
                let running_time = (incoming.time / 5.0).ceil() * 5.0;
                // 運転時刻は、直前駅までの運転時分を積み上げて求める。
                let previous_departure = train_type
                    .is_pass
                    .iter()
                    .take(station_index)
                    .enumerate()
                    .map(|(i, _)| {
                        let segment = &result[train_index][i];
                        (segment.time / 5.0).ceil() * 5.0
                            + if i > 0 && !train_type.is_pass[i].1 {
                                if train_type.is_pass[i].2 == "◎" {
                                    30.0
                                } else {
                                    40.0
                                }
                            } else {
                                0.0
                            }
                    })
                    .sum::<f64>();
                let arrival = previous_departure + running_time;
                let departure = if passed || last {
                    arrival
                } else {
                    arrival
                        + if train_type.is_pass[station_index].2 == "◎" {
                            30.0
                        } else {
                            20.0
                        }
                };
                (incoming.time.to_string(), running_time, arrival, departure)
            } else {
                (String::new(), 0.0, 0.0, 0.0)
            };

            let stop = if first {
                "発"
            } else if last {
                "着"
            } else if passed {
                "↓"
            } else if train_type.is_pass[station_index].2 == "◎" {
                "30"
            } else {
                "20"
            };
            let arrival = if first {
                "---".to_string()
            } else if passed {
                "↓".to_string()
            } else {
                format_time(arrival)
            };
            let departure = if last {
                "---".to_string()
            } else {
                format_time(departure)
            };

            output.push_str(&format!(
                "\t{}\t{}\t{}\t{}\t{}",
                stop, calculated, running as u64, arrival, departure
            ));
        }
        output.push('\n');
    }
    let _ = std::fs::write(output_tsv_path, output);

    fn format_time(seconds: f64) -> String {
        let seconds = seconds.round() as u64;
        format!(
            "{:02}:{:02}:{:02}",
            seconds / 3600,
            (seconds / 60) % 60,
            seconds % 60
        )
    }

    /// 曲線制限速度
    fn get_curve_limit_speed(radius: f64) -> Option<f64> {
        let margin = 10.0;

        if radius < 70.0 {
            Some(30.0 - margin)
        } else if radius < 90.0 {
            Some(35.0 - margin)
        } else if radius < 110.0 {
            Some(40.0 - margin)
        } else if radius < 140.0 {
            Some(45.0 - margin)
        } else if radius < 165.0 {
            Some(50.0 - margin)
        } else if radius < 195.0 {
            Some(55.0 - margin)
        } else if radius < 230.0 {
            Some(60.0 - margin)
        } else if radius < 270.0 {
            Some(65.0 - margin)
        } else if radius < 300.0 {
            Some(70.0 - margin)
        } else if radius < 305.0 {
            Some(75.0 - margin)
        } else if radius < 345.0 {
            Some(80.0 - margin)
        } else if radius < 385.0 {
            Some(85.0 - margin)
        } else if radius < 430.0 {
            Some(90.0 - margin)
        } else if radius < 475.0 {
            Some(95.0 - margin)
        } else if radius < 525.0 {
            Some(100.0 - margin)
        } else if radius < 575.0 {
            Some(105.0 - margin)
        } else if radius < 625.0 {
            Some(110.0 - margin)
        } else if radius < 685.0 {
            Some(115.0 - margin)
        } else {
            None
        }
    }

    /// 勾配制限速度
    fn get_gradient_limit_speed(gradient: f64) -> Option<f64> {
        let margin = 10.0;

        if gradient < -45.0 {
            Some(95.0 - margin)
        } else if gradient < -35.0 {
            Some(100.0 - margin)
        } else if gradient < -25.0 {
            Some(105.0)
        } else if gradient < -20.0 {
            Some(110.0)
        } else if gradient < -10.0 {
            Some(115.0)
        } else {
            None
        }
    }

    /// 信号制限速度
    fn get_signal_limit_speeds(signal_type: &str, stop_position: f64) -> Vec<LimitSpeed> {
        match signal_type {
            // 通過駅(制限あり)
            "レ" => vec![LimitSpeed {
                start: stop_position - 130.0,
                end: stop_position + 180.0,
                speed: 45.0,
            }],
            // 連動駅(出発停止)
            "◎" => vec![
                LimitSpeed {
                    start: stop_position,
                    end: stop_position + 180.0,
                    speed: 45.0,
                },
                LimitSpeed {
                    start: stop_position - 150.0 + 30.0, // 30m重複させる
                    end: stop_position,
                    speed: 25.0,
                },
                LimitSpeed {
                    start: stop_position - 280.0,
                    end: stop_position - 150.0 + 30.0, // 30m重複させる
                    speed: 45.0,
                },
                LimitSpeed {
                    start: stop_position - 840.0,
                    end: stop_position - 280.0,
                    speed: 65.0,
                },
            ],
            "A" => vec![
                // 頭端式
                LimitSpeed {
                    start: stop_position - 25.0,
                    end: stop_position,
                    speed: 15.0,
                },
                LimitSpeed {
                    start: stop_position - 150.0,
                    end: stop_position - 25.0,
                    speed: 25.0,
                },
                LimitSpeed {
                    start: stop_position - 280.0,
                    end: stop_position - 150.0,
                    speed: 45.0,
                },
                LimitSpeed {
                    start: stop_position - 840.0,
                    end: stop_position - 280.0,
                    speed: 65.0,
                },
            ],
            _ => vec![],
        }
    }
}

fn reverse_route(mut route: Route) -> Route {
    if route.stop_positions.is_empty() {
        return route;
    }

    let route_length = route
        .stop_positions
        .iter()
        .map(|stop| stop.position)
        .fold(0.0, f64::max);
    let reverse_interval = |start: f64, end: f64| (route_length - end, route_length - start);

    for curve in &mut route.curves {
        let start = curve.start;
        let end = curve.end;
        curve.start = route_length - end;
        curve.end = route_length - start;
        curve.direction = match curve.direction {
            CurveDirection::Left => CurveDirection::Right,
            CurveDirection::Right => CurveDirection::Left,
        }
    }

    let mut gradients = route.gradients.clone();
    gradients.sort_by(|a, b| a.position.total_cmp(&b.position));
    let terminal_gradient = gradients.last().map_or(0.0, |gradient| gradient.value);
    let mut reversed_gradients = vec![Gradient {
        position: 0.0,
        value: -terminal_gradient,
    }];
    for (index, gradient) in gradients.iter().enumerate().rev() {
        let position = route_length - gradient.position;
        if position <= 0.0 {
            continue;
        }
        let previous_value = gradients
            .get(index.wrapping_sub(1))
            .map_or(0.0, |previous| previous.value);
        reversed_gradients.push(Gradient {
            position,
            value: -previous_value,
        });
    }
    reversed_gradients.sort_by(|a, b| a.position.total_cmp(&b.position));
    route.gradients = reversed_gradients;

    for limit in &mut route.limit_speeds {
        let (start, end) = reverse_interval(limit.start, limit.end);
        limit.start = start;
        limit.end = end;
    }

    for sta in &mut route.stop_positions {
        sta.position = route_length - sta.position;
    }
    route.stop_positions.reverse();

    for tunnel in &mut route.tunnels {
        let start = tunnel.start;
        let end = tunnel.end;
        tunnel.start = route_length - end;
        tunnel.end = route_length - start;
    }

    route.curves.sort_by(|a, b| a.start.total_cmp(&b.start));
    route.tunnels.sort_by(|a, b| a.start.total_cmp(&b.start));
    route
        .limit_speeds
        .sort_by(|a, b| a.start.total_cmp(&b.start));

    route
}

#[cfg(test)]
mod tests {
    use super::reverse_route;
    use runcurve_rs::model::route::{
        Curve, CurveDirection, Gradient, LimitSpeed, Route, StopPosition, Tunnel, TunnelType,
    };

    #[test]
    fn reverse_route_maps_distances_and_reverses_directional_data() {
        let route = Route {
            gradients: vec![
                Gradient {
                    position: 20.0,
                    value: 10.0,
                },
                Gradient {
                    position: 80.0,
                    value: -5.0,
                },
            ],
            curves: vec![Curve {
                start: 10.0,
                end: 30.0,
                direction: CurveDirection::Left,
                ..Curve::default()
            }],
            tunnels: vec![Tunnel {
                start: 40.0,
                end: 60.0,
                tunnel_type: TunnelType::Single,
                ..Tunnel::default()
            }],
            limit_speeds: vec![LimitSpeed {
                start: 0.0,
                end: 50.0,
                speed: 65.0,
            }],
            stop_positions: vec![
                StopPosition {
                    position: 0.0,
                    station_name: "A".to_string(),
                    ..StopPosition::default()
                },
                StopPosition {
                    position: 100.0,
                    station_name: "B".to_string(),
                    ..StopPosition::default()
                },
            ],
            ..Route::default()
        };

        let reversed = reverse_route(route);

        assert_eq!(reversed.stop_positions[0].station_name, "B");
        assert_eq!(reversed.stop_positions[0].position, 0.0);
        assert_eq!(reversed.stop_positions[1].station_name, "A");
        assert_eq!(reversed.stop_positions[1].position, 100.0);
        assert_eq!(reversed.curves[0].start, 70.0);
        assert_eq!(reversed.curves[0].end, 90.0);
        assert_eq!(reversed.curves[0].direction, CurveDirection::Right);
        assert_eq!(reversed.tunnels[0].start, 40.0);
        assert_eq!(reversed.tunnels[0].end, 60.0);
        assert_eq!(reversed.limit_speeds[0].start, 50.0);
        assert_eq!(reversed.limit_speeds[0].end, 100.0);
        assert_eq!(
            reversed.gradients,
            vec![
                Gradient {
                    position: 0.0,
                    value: 5.0
                },
                Gradient {
                    position: 20.0,
                    value: -10.0
                },
                Gradient {
                    position: 80.0,
                    value: 0.0
                }
            ]
        );
    }
}
