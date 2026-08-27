pub mod axis;

use itertools::Itertools;
use svg::node::element::{path::Data, *};
use svg2pdf::{ConversionOptions, PageOptions};

use crate::model::{
    route::{Curve, CurveDirection, Route},
    runcurve::RuncurveResult,
    vehicle::Vehicle,
};

/// 距離軸の拡大率
const X_SCALE: f64 = 0.5;
/// 時間軸の拡大率
const Y_SCALE: f64 = 5.0;
/// ヘッダの横幅
const X_HEADER: f64 = 50.0;
const Y_HEADER: f64 = 20.0;
const MARGIN: f64 = 50.0;

#[derive(Debug)]
pub enum DrawSvgError {
    StationNotFound,
}

pub fn to_pdf(
    route: &Route,
    vehicle: &Vehicle,
    result: &[RuncurveResult],
    max_speed: f64,
) -> Result<Vec<u8>, DrawSvgError> {
    let svg = draw_svg(route, vehicle, result, max_speed)?.to_string();
    let mut options = svg2pdf::usvg::Options::default();
    options.fontdb_mut().load_system_fonts();
    let tree = svg2pdf::usvg::Tree::from_str(&svg, &options).unwrap();
    let pdf = svg2pdf::to_pdf(&tree, ConversionOptions::default(), PageOptions::default()).unwrap();

    Ok(pdf)
}

pub fn draw_svg(
    route: &Route,
    _vehicle: &Vehicle,
    result: &[RuncurveResult],
    max_speed: f64,
) -> Result<SVG, DrawSvgError> {
    let start_position = route
        .stop_positions
        .first()
        .ok_or(DrawSvgError::StationNotFound)?
        .position;
    let end_position = route
        .stop_positions
        .last()
        .ok_or(DrawSvgError::StationNotFound)?
        .position;

    let mut svg = svg::Document::new().set(
        "viewBox",
        (
            -MARGIN,
            -MARGIN,
            (end_position - start_position) * X_SCALE + X_HEADER + MARGIN * 2.0,
            max_speed * Y_SCALE + Y_HEADER * 6.0 + MARGIN * 2.0,
        ),
    );
    svg = draw_axis(
        svg,
        max_speed,
        end_position - start_position,
        start_position,
    );
    svg = draw_train(svg, result, max_speed, start_position);
    svg = draw_curve(
        svg,
        route,
        max_speed,
        end_position - start_position,
        start_position,
    );
    svg = draw_gradient(
        svg,
        route,
        max_speed,
        end_position - start_position,
        start_position,
    );
    svg = draw_limit(
        svg,
        route,
        max_speed,
        end_position - start_position,
        start_position,
    );
    svg = draw_station(
        svg,
        route,
        max_speed,
        end_position - start_position,
        start_position,
        result,
    );
    svg = draw_notch(
        svg,
        route,
        max_speed,
        end_position - start_position,
        start_position,
        result,
    );

    Ok(svg)
}

fn draw_axis(svg: SVG, max_speed: f64, distance: f64, start_position: f64) -> SVG {
    let mut horizon = vec![];
    let mut horizon_text = vec![];
    for speed in 0..=max_speed as usize {
        let stroke_width = if speed % 10 == 0 { "0.5" } else { "0.1" };
        horizon.push(
            Line::new()
                .set("x1", X_HEADER)
                .set("x2", distance * X_SCALE + X_HEADER)
                .set("y1", speed as f64 * Y_SCALE + Y_HEADER)
                .set("y2", speed as f64 * Y_SCALE + Y_HEADER)
                .set("fill", "none")
                .set("stroke", "#676")
                .set("stroke-width", stroke_width),
        );
        if speed % 10 == 0 {
            horizon_text.push(
                Text::new((max_speed as usize - speed).to_string())
                    .set("x", X_HEADER / 2.0)
                    .set("y", speed as f64 * Y_SCALE + Y_HEADER)
                    .set("fill", "black")
                    .set("font-size", "15")
                    .set("font-family", "Ubuntu")
                    .set("text-anchor", "middle")
                    .set("dominant-baseline", "central"),
            );
        }
    }

    let mut vertical = vec![];
    let mut vertical_text = vec![];
    for distance in start_position as usize..=(distance + start_position) as usize {
        if distance % 10 != 0 {
            continue;
        }
        let stroke_width = if distance % 100 == 0 { "0.5" } else { "0.1" };
        vertical.push(
            Line::new()
                .set(
                    "x1",
                    (distance - start_position as usize) as f64 * X_SCALE + X_HEADER,
                )
                .set(
                    "x2",
                    (distance - start_position as usize) as f64 * X_SCALE + X_HEADER,
                )
                .set("y1", Y_HEADER)
                .set("y2", max_speed as f64 * Y_SCALE + Y_HEADER)
                .set("fill", "none")
                .set("stroke", "#676")
                .set("stroke-width", stroke_width),
        );
        if distance % 100 == 0 {
            let text = if distance % 1000 == 0 {
                (distance / 1000).to_string()
            } else {
                ((distance / 100) % 10).to_string()
            };
            let text_size = if distance % 1000 == 0 { "25" } else { "15" };
            vertical_text.push(
                Text::new(text)
                    .set(
                        "x",
                        (distance - start_position as usize) as f64 * X_SCALE + X_HEADER,
                    )
                    .set("y", max_speed as f64 * Y_SCALE + Y_HEADER * 1.5)
                    .set("fill", "black")
                    .set("font-size", text_size)
                    .set("font-family", "Ubuntu")
                    .set("text-anchor", "middle")
                    .set("dominant-baseline", "central"),
            );
        }
    }

    let mut group = Group::new();
    group = group.add(
        Rectangle::new()
            .set("x", X_HEADER)
            .set("y", Y_HEADER)
            .set("width", distance as f64 * X_SCALE)
            .set("height", max_speed as f64 * Y_SCALE)
            .set("fill", "none")
            .set("stroke", "black")
            .set("stroke-width", "1"),
    );
    for h in horizon {
        group = group.add(h);
    }
    for h in horizon_text {
        group = group.add(h);
    }
    for v in vertical {
        group = group.add(v);
    }
    for v in vertical_text {
        group = group.add(v);
    }

    svg.add(group)
}

fn draw_train(svg: SVG, result: &[RuncurveResult], max_speed: f64, start_position: f64) -> SVG {
    let mut elements = vec![];
    for result in result {
        let mut speed_data = Data::new();
        let mut time_data = Data::new();
        let mut time_offset = 0.0;
        for runcurve in &result.runcurve_array {
            if speed_data.is_empty() {
                speed_data = speed_data.move_to((
                    (runcurve.distance - start_position) * X_SCALE + X_HEADER,
                    (max_speed - runcurve.speed) * Y_SCALE + Y_HEADER,
                ));
                time_data = time_data.move_to((
                    (runcurve.distance - start_position) * X_SCALE + X_HEADER,
                    (max_speed - (runcurve.time - time_offset)) * Y_SCALE + Y_HEADER,
                ));
            } else {
                speed_data = speed_data.line_to((
                    (runcurve.distance - start_position) * X_SCALE + X_HEADER,
                    (max_speed - runcurve.speed) * Y_SCALE + Y_HEADER,
                ));
                if runcurve.time - time_offset > max_speed {
                    time_offset += max_speed;
                    time_data = time_data.move_to((
                        (runcurve.distance - start_position) * X_SCALE + X_HEADER,
                        (max_speed - (runcurve.time - time_offset)) * Y_SCALE + Y_HEADER,
                    ));
                } else {
                    time_data = time_data.line_to((
                        (runcurve.distance - start_position) * X_SCALE + X_HEADER,
                        (max_speed - (runcurve.time - time_offset)) * Y_SCALE + Y_HEADER,
                    ));
                }
            }
        }
        elements.push(
            Path::new()
                .set("d", speed_data)
                .set("fill", "none")
                .set("stroke", "#33a")
                .set("stroke-width", "1"),
        );
        elements.push(
            Path::new()
                .set("d", time_data)
                .set("fill", "none")
                .set("stroke", "#a33")
                .set("stroke-width", "1"),
        );
    }

    let mut group = Group::new();
    for e in elements {
        group = group.add(e);
    }

    svg.add(group)
}

fn draw_curve(svg: SVG, route: &Route, max_speed: f64, distance: f64, start_position: f64) -> SVG {
    let mut group = Group::new();
    group = group.add(
        Rectangle::new()
            .set("x", 0)
            .set("y", max_speed * Y_SCALE + Y_HEADER * 2.0)
            .set("width", X_HEADER)
            .set("height", Y_HEADER * 2.0)
            .set("fill", "none")
            .set("stroke", "black")
            .set("stroke-width", "1"),
    );
    group = group.add(
        Rectangle::new()
            .set("x", X_HEADER)
            .set("y", max_speed * Y_SCALE + Y_HEADER * 2.0)
            .set("width", distance * X_SCALE)
            .set("height", Y_HEADER * 2.0)
            .set("fill", "none")
            .set("stroke", "black")
            .set("stroke-width", "1"),
    );

    for curve in &route.curves {
        let direction_position = if curve.direction == CurveDirection::Right {
            2.5
        } else {
            3.5
        };
        group = group.add(
            Path::new()
                .set(
                    "d",
                    Data::new()
                        .move_to((
                            (curve.start - start_position) * X_SCALE + X_HEADER,
                            max_speed * Y_SCALE + Y_HEADER * 3.0,
                        ))
                        .line_to((
                            (curve.start - start_position) * X_SCALE + X_HEADER,
                            max_speed * Y_SCALE + Y_HEADER * direction_position,
                        ))
                        .line_to((
                            (curve.end - start_position) * X_SCALE + X_HEADER,
                            max_speed * Y_SCALE + Y_HEADER * direction_position,
                        ))
                        .line_to((
                            (curve.end - start_position) * X_SCALE + X_HEADER,
                            max_speed * Y_SCALE + Y_HEADER * 3.0,
                        )),
                )
                .set("fill", "none")
                .set("stroke", "black")
                .set("stroke-width", "1"),
        );
        group = group.add(
            Text::new(curve.radius.to_string())
                .set(
                    "x",
                    ((curve.start + curve.end) / 2.0 - start_position) * X_SCALE + X_HEADER,
                )
                .set("y", max_speed * Y_SCALE + Y_HEADER * 3.0)
                .set("fill", "black")
                .set("font-size", "15")
                .set("font-family", "Ubuntu")
                .set("text-anchor", "middle")
                .set("dominant-baseline", "central"),
        );
    }

    let iter = std::iter::once(Curve {
        end: start_position,
        ..Default::default()
    })
    .chain(route.curves.clone())
    .chain(std::iter::once(Curve {
        start: distance + start_position,
        ..Default::default()
    }));
    for (before, after) in iter.tuple_windows() {
        group = group.add(
            Line::new()
                .set("x1", (before.end - start_position) * X_SCALE + X_HEADER)
                .set("x2", (after.start - start_position) * X_SCALE + X_HEADER)
                .set("y1", max_speed * Y_SCALE + Y_HEADER * 3.0)
                .set("y2", max_speed * Y_SCALE + Y_HEADER * 3.0)
                .set("fill", "none")
                .set("stroke", "black")
                .set("stroke-width", "1"),
        );
    }

    svg.add(group)
}

fn draw_gradient(
    svg: SVG,
    route: &Route,
    max_speed: f64,
    distance: f64,
    start_position: f64,
) -> SVG {
    let mut group = Group::new();
    group = group.add(
        Rectangle::new()
            .set("x", 0)
            .set("y", max_speed * Y_SCALE + Y_HEADER * 4.0)
            .set("width", X_HEADER)
            .set("height", Y_HEADER * 2.0)
            .set("fill", "none")
            .set("stroke", "black")
            .set("stroke-width", "1"),
    );
    group = group.add(
        Rectangle::new()
            .set("x", X_HEADER)
            .set("y", max_speed * Y_SCALE + Y_HEADER * 4.0)
            .set("width", distance * X_SCALE)
            .set("height", Y_HEADER * 2.0)
            .set("fill", "none")
            .set("stroke", "black")
            .set("stroke-width", "1"),
    );

    for gradient in &route.gradients {
        group = group.add(
            Line::new()
                .set(
                    "x1",
                    (gradient.position - start_position) * X_SCALE + X_HEADER,
                )
                .set(
                    "x2",
                    (gradient.position - start_position) * X_SCALE + X_HEADER,
                )
                .set("y1", max_speed * Y_SCALE + Y_HEADER * 4.0)
                .set("y2", max_speed * Y_SCALE + Y_HEADER * 6.0)
                .set("fill", "none")
                .set("stroke", "black")
                .set("stroke-width", "1"),
        );
        group = group.add(
            Text::new(gradient.value.to_string())
                .set(
                    "x",
                    (gradient.position - start_position) * X_SCALE + X_HEADER,
                )
                .set("y", max_speed * Y_SCALE + Y_HEADER * 5.0)
                .set("fill", "black")
                .set("font-size", "15")
                .set("font-family", "Ubuntu")
                .set("text-anchor", "left")
                .set("dominant-baseline", "central"),
        );
    }

    svg.add(group)
}

fn draw_limit(svg: SVG, route: &Route, max_speed: f64, distance: f64, start_position: f64) -> SVG {
    let mut group = Group::new();

    for limit in &route.limit_speeds {
        group = group.add(
            Path::new()
                .set(
                    "d",
                    Data::new()
                        .move_to((
                            (limit.start - start_position) * X_SCALE + X_HEADER,
                            (max_speed - limit.speed) * Y_SCALE + Y_HEADER - 10.0,
                        ))
                        .line_to((
                            (limit.start - start_position) * X_SCALE + X_HEADER,
                            (max_speed - limit.speed) * Y_SCALE + Y_HEADER,
                        ))
                        .line_to((
                            (limit.end - start_position) * X_SCALE + X_HEADER,
                            (max_speed - limit.speed) * Y_SCALE + Y_HEADER,
                        ))
                        .line_to((
                            (limit.end - start_position) * X_SCALE + X_HEADER,
                            (max_speed - limit.speed) * Y_SCALE + Y_HEADER - 10.0,
                        )),
                )
                .set("fill", "none")
                .set("stroke", "black")
                .set("stroke-width", "1"),
        );
    }

    svg.add(group)
}

fn draw_station(
    svg: SVG,
    route: &Route,
    max_speed: f64,
    _distance: f64,
    start_position: f64,
    result: &[RuncurveResult],
) -> SVG {
    let mut group = Group::new();

    for pos in &route.stop_positions {
        group = group.add(
            Path::new()
                .set(
                    "d",
                    Data::new()
                        .move_to((
                            (pos.position - start_position) * X_SCALE + X_HEADER,
                            Y_HEADER,
                        ))
                        .line_to((
                            (pos.position - start_position) * X_SCALE + X_HEADER,
                            max_speed * Y_SCALE + Y_HEADER,
                        )),
                )
                .set("fill", "none")
                .set("stroke", "black")
                .set("stroke-width", "1"),
        );
        group = group.add(
            Text::new(&pos.station_name)
                .set("x", (pos.position - start_position) * X_SCALE + X_HEADER)
                .set("y", Y_HEADER + 10.0)
                .set("fill", "black")
                .set("font-size", "15")
                .set("font-family", "Ubuntu")
                .set("text-anchor", "left")
                .set("dominant-baseline", "central"),
        );
        let time = result
            .iter()
            .flat_map(|v| v.runcurve_array.clone())
            .find(|v| v.distance as usize == (pos.position - 1.0) as usize);
        if let Some(time) = time {
            group = group.add(
                Path::new()
                    .set(
                        "d",
                        Data::new()
                            .move_to((
                                (pos.position - start_position) * X_SCALE + X_HEADER - 10.0,
                                (max_speed - (time.time % max_speed)) * Y_SCALE + Y_HEADER,
                            ))
                            .line_to((
                                (pos.position - start_position) * X_SCALE + X_HEADER + 10.0,
                                (max_speed - (time.time % max_speed)) * Y_SCALE + Y_HEADER,
                            )),
                    )
                    .set("fill", "none")
                    .set("stroke", "black")
                    .set("stroke-width", "1"),
            );
            let time_string = format!(
                "{}:{:2}",
                (time.time as usize) / 60,
                time.time as usize % 60
            );
            group = group.add(
                Text::new(time_string)
                    .set("x", (pos.position - start_position) * X_SCALE + X_HEADER)
                    .set(
                        "y",
                        (max_speed - (time.time % max_speed)) * Y_SCALE + Y_HEADER - 10.0,
                    )
                    .set("fill", "black")
                    .set("font-size", "15")
                    .set("font-family", "Ubuntu")
                    .set("text-anchor", "middle")
                    .set("dominant-baseline", "central"),
            );
            let time_string = format!(
                "({}:{:2})",
                (time.time as usize) / 60,
                (time.time as usize % 60) / 5 * 5 + 5
            );
            group = group.add(
                Text::new(time_string)
                    .set("x", (pos.position - start_position) * X_SCALE + X_HEADER)
                    .set(
                        "y",
                        (max_speed - (time.time % max_speed)) * Y_SCALE + Y_HEADER - 25.0,
                    )
                    .set("fill", "black")
                    .set("font-size", "10")
                    .set("font-family", "Ubuntu")
                    .set("text-anchor", "middle")
                    .set("dominant-baseline", "central"),
            );
        }
    }

    svg.add(group)
}

fn draw_notch(
    svg: SVG,
    _route: &Route,
    _max_speed: f64,
    _distance: f64,
    start_position: f64,
    result: &[RuncurveResult],
) -> SVG {
    let mut group = Group::new();

    for notch in result.iter().flat_map(|v| &v.notches) {
        let text = match notch.notch_type {
            crate::model::runcurve::NotchType::Power => "P",
            crate::model::runcurve::NotchType::Brake => "B",
            crate::model::runcurve::NotchType::NotchOff => "O",
            crate::model::runcurve::NotchType::Constant => "T",
        };
        group = group.add(
            Text::new(text)
                .set("x", (notch.distance - start_position) * X_SCALE + X_HEADER)
                .set("y", Y_HEADER + 20.0)
                .set("fill", "black")
                .set("font-size", "10")
                .set("font-family", "Ubuntu")
                .set("text-anchor", "left")
                .set("dominant-baseline", "central"),
        );
    }

    svg.add(group)
}
