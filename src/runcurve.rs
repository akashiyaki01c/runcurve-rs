/// モジュール: ランカーブ計算のコアロジック。
///
/// このモジュールは、以下のサブモジュールから構成されます:
/// - `runcurve_speed`: 加速・減速・惰行時の速度計算とパターン生成
/// - `route_data`: 路線データから1mごとの配列データを生成
/// - `runcurve_line`: 路線全体のランカーブと所要時間の計算

pub mod runcurve_speed;
pub mod route_data;
pub mod runcurve_line;
