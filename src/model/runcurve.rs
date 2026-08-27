use serde::{Deserialize, Serialize};

/// ランカーブの算出結果
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct RuncurveResult {
    pub notches: Vec<NotchOperate>,
    pub runcurve_array: Vec<Runcurve>,
}

/// ランカーブの距離ごとの算出結果
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct Runcurve {
    /// 距離 [m]
    pub distance: f64,
    /// 速度 [km/h]（または m/s）
    pub speed: f64,
    /// 経過時間 [s]
    pub time: f64,
}

/// ノッチの種類
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum NotchType {
    Power,
    Brake,
    NotchOff,
    Constant,
}

impl Default for NotchType {
    fn default() -> Self {
        Self::NotchOff
    }
}

/// ノッチの操作履歴
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct NotchOperate {
    /// 操作位置 [m]
    pub distance: f64,
    /// ノッチ種別
    pub notch_type: NotchType,
    /// 詳細情報（任意）
    #[serde(skip_serializing_if = "Option::is_none")]
    pub detail: Option<String>,
}