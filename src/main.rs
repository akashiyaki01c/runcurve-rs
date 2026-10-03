use itertools::Itertools;
use runcurve_rs::{
    model::{
        route::{Gradient, LimitSpeed, Route},
        runcurve::RuncurveResult,
    },
    runcurve::runcurve_line::{get_runcurve_line, get_runcurve_line_time},
    svg::{draw_svg, to_pdf},
    testdata::{get_test_route, get_test_vehicle},
};

fn main() {
    let route = get_test_route();
    let vehicle = get_test_vehicle();

    let runcurve_result = get_runcurve_line(&route, &vehicle, 100.0);

    let result = get_runcurve_line_time(&route, &runcurve_result);

    for r in &result {
        println!("{:?}", r);
    }

    let pdf = to_pdf(&route, &vehicle, &vec![runcurve_result], 120.0).unwrap();
    std::fs::write("./test.pdf", pdf);
}

#[test]
fn test() {
    let route = std::fs::read_to_string("./base-data/001_hanshin.json").unwrap();
    let mut route = serde_json::from_str::<Route>(&route).unwrap();

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

    struct TrainType {
        pub name: String,
        pub limit_speeds: Vec<LimitSpeed>,
        pub is_pass: Vec<(String, bool)>,
    }

    // 列車種別データ読み込み
    let mut train_types: Vec<TrainType> = vec![];
    for (i, row) in std::fs::read_to_string("./base-data/001_hanshin_stop.tsv")
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
            println!("{sta_name}");
            let sta = route
                .stop_positions
                .iter()
                .find(|v| v.station_name == sta_name)
                .unwrap();
            for (i, r) in row.split("\t").skip(1).enumerate() {
                let train_type = train_types.get_mut(i).unwrap();
                println!("{r} {}", train_type.name);
                train_type
                    .is_pass
                    .push((sta_name.to_string(), r.is_empty()));
                for s in get_signal_limit_speeds(r, sta.position) {
                    train_type.limit_speeds.push(s);
                }
            }
        }
    }

    // 列車種別データの前後空白を削除
    for train_type in &mut train_types {
        let first = train_type.is_pass.iter().find_position(|v: &&(String, bool)| v.1).unwrap().0;
        let end = train_type.is_pass.iter().rev().find_position(|v| v.1).unwrap().0;
        train_type.is_pass = train_type.is_pass[first..=end].iter().map(|v| v.clone()).collect();
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

    // let pdf = to_pdf(&route, &vehicle, &result, 120.0).unwrap();
    // std::fs::write("./test.pdf", pdf);

    let result: Vec<_> = result
        .iter()
        .map(|v| get_runcurve_line_time(&route, &v))
        .collect();

    // CSV出力
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
    std::fs::write("./test.tsv", str);

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
            // 連動駅(出発停止)
            "◎" => vec![
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
