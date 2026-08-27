use serde::{Deserialize, Serialize};

/// 路線データを表す
///
/// 路線の基本情報を保持します。
///
/// # フィールド
///
/// - `name`: 路線の名前
/// - `gradients`: 路線上の勾配変化点（位置と勾配値の配列）
/// - `curves`: 曲線データ（曲線半径、方向、制限速度など）
/// - `tunnels`: トンネルデータ（種類、位置、種別）
/// - `limit_speeds`: 各区間の制限速度
/// - `stop_positions`: 駅や停車点の位置情報
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct Route {
    /// 路線名
    pub name: String,
    /// 路線上の勾配変化点
    pub gradients: Vec<Gradient>,
    /// 曲線データ
    pub curves: Vec<Curve>,
    /// トンネルデータ
    pub tunnels: Vec<Tunnel>,
    /// 各区間の制限速度
    pub limit_speeds: Vec<LimitSpeed>,
    /// 駅や停車点の位置
    pub stop_positions: Vec<StopPosition>,
}

/// 勾配データ
///
/// 路線上の勾配変化点を保持します。
/// 位置 [m] と勾配 [‰] のペアで構成され、位置順にソートされています。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct Gradient {
    /// 勾配変更点 [m]
    pub position: f64,
    /// 勾配度 [‰]（上向きは正、下向きは負）
    pub value: f64,
}

/// 曲線方向
///
/// 曲線の走行方向を表します。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum CurveDirection {
    Left,
    Right,
}

impl Default for CurveDirection {
    fn default() -> Self {
        Self::Left
    }
}

/// 曲線データ
///
/// 曲線の幾何学的情報を保持します。
///
/// # フィールド
///
/// - `start`: 曲線の開始位置 [m]
/// - `end`: 曲線の終了位置 [m]
/// - `radius`: 曲線の半径 [m]
/// - `direction`: 曲線の方向 (左/右)
/// - `speed`: 曲線上の制限速度 [km/h]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct Curve {
    /// 曲線起点 [m]
    pub start: f64,
    /// 曲線終点 [m]
    pub end: f64,
    /// 曲線半径 [m]
    pub radius: f64,
    /// 曲線方向
    pub direction: CurveDirection,
    /// 制限速度 [km/h]
    pub speed: f64,
}

/// トンネル種別
///
/// トンネルの種類を表します。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TunnelType {
    Single,
    Double,
}

impl Default for TunnelType {
    fn default() -> Self {
        Self::Double
    }
}

/// トンネルデータ
///
/// トンネルの詳細情報を保持します。
///
/// # フィールド
///
/// - `name`: トンネル名
/// - `start`: トンネル開始位置 [m]
/// - `end`: トンネル終了位置 [m]
/// - `tunnel_type`: トンネル種別 (Single/Double)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct Tunnel {
    /// トンネル名
    pub name: String,
    /// トンネル開始地点 [m]
    pub start: f64,
    /// トンネル終了地点 [m]
    pub end: f64,
    /// トンネル種別
    pub tunnel_type: TunnelType,
}

/// 制限速度データ
///
/// 路線上の各区間の制限速度を保持します。
///
/// # フィールド
///
/// - `start`: 制限速度が適用される区間の開始位置 [m]
/// - `end`: 制限速度が適用される区間の終了位置 [m]
/// - `speed`: 制限速度 [km/h]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct LimitSpeed {
    /// 制限速度開始地点 [m]
    pub start: f64,
    /// 制限速度終了地点 [m]
    pub end: f64,
    /// 制限速度 [km/h]
    pub speed: f64,
}

/// 停車位置データ
///
/// 路線上の駅や停車点の位置情報を保持します。
///
/// # フィールド
///
/// - `position`: 位置 [m]
/// - `station_name`: 駅名
/// - `track_name`: 線路名
/// - `is_pass`: 通過するかどうか (true = 通過, false = 停留)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct StopPosition {
    /// 停止位置 [m]
    pub position: f64,
    /// 駅名
    pub station_name: String,
    /// 線路名
    pub track_name: String,
    /// 通過するか
    pub is_pass: bool,
}
