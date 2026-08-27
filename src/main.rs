use runcurve_rs::{runcurve::runcurve_line::{get_runcurve_line, get_runcurve_line_time}, svg::{draw_svg, to_pdf}, testdata::{get_test_route, get_test_vehicle}};

fn main() {
    let route = get_test_route();
    let vehicle = get_test_vehicle();

    let runcurve_result = get_runcurve_line(&route, &vehicle, 100.0);

    let result = get_runcurve_line_time(&route, &runcurve_result);

    for r in &result {
        println!("{:?}", r);
    }

    let pdf = to_pdf(&route, &vehicle, &runcurve_result, 120.0).unwrap();
    std::fs::write("./test.pdf", pdf);
}
