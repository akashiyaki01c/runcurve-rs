use runcurve_rs::{runcurve::runcurve_line::{get_runcurve_line, get_runcurve_line_time}, testdata::{get_test_route, get_test_vehicle}};

fn main() {
    let route = get_test_route();
    let vehicle = get_test_vehicle();

    let result = get_runcurve_line(&route, &vehicle, 100.0);

    let result = get_runcurve_line_time(&route, &result);

    for r in result {
        println!("{:?}", r);
    }
}
