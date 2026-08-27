use serde::{Deserialize, Serialize};

/// 路線データを表す
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct Route {
    /// 路線名
    pub name: String,
    pub gradients: Vec<Gradient>,
    pub curves: Vec<Curve>,
    pub tunnels: Vec<Tunnel>,
    pub limit_speeds: Vec<LimitSpeed>,
    pub stop_positions: Vec<StopPosition>,
}

/// 勾配データ
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct Gradient {
    /// 勾配変更点 [m]
    pub position: f64,
    /// 勾配度 [‰] 上りが正, 下りが負
    pub value: f64,
}

/// 曲線方向
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
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(rename_all = "camelCase")]
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
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
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
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(rename_all = "camelCase")]
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
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct LimitSpeed {
    /// 制限速度開始地点 [m]
    pub start: f64,
    /// 制限速度終了地点 [m]
    pub end: f64,
    /// 制限速度 [km/h]
    pub speed: f64,
}

/// 停止位置データ
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct StopPosition {
    /// 停止位置 [m]
    pub position: f64,
    /// 駅名
    pub station_name: String,
    /// 番線名
    pub track_name: String,
    /// 通過するか
    pub is_pass: bool,
}