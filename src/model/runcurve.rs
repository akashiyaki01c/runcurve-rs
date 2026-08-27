use serde::{Deserialize, Serialize};

/// ランカーブの算出結果
///
/// ランカーブ計算の総合結果。ノッチ操作履歴と距離ごとの
/// ランカーブ配列を含みます。
///
/// # フィールド
///
/// - `notches`: ノッチ操作履歴のベクタ
/// - `runcurve_array`: 各位置での距離・速度・時間の配列
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct RuncurveResult {
    /// ノッチ操作履歴
    pub notches: Vec<NotchOperate>,
    /// ランカーブ配列（距離 [m]、速度 [km/h]、時間 [s]）
    pub runcurve_array: Vec<Runcurve>,
}

/// ランカーブの距離ごとの算出結果
///
/// 各1m区間での計算結果を保持します。
///
/// # フィールド
///
/// - `distance`: 距離 [m]
/// - `speed`: 速度 [km/h]
/// - `time`: 累積経過時間 [s]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct Runcurve {
    /// 距離 [m]
    pub distance: f64,
    /// 速度 [km/h]
    pub speed: f64,
    /// 経過時間 [s]
    pub time: f64,
}

/// ノッチの種類
///
/// 列車のノッチ（運転操作）の状態を表します。
///
/// # バリアント
///
/// - `Power`: 力行（加速）
/// - `Brake`: ブレーキ（減速）
/// - `NotchOff`: ノッチオフ（惰行）
/// - `Constant`: 一定速度維持
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum NotchType {
    /// 力行（加速）
    Power,
    /// ブレーキ（減速）
    Brake,
    /// ノッチオフ（惰行）
    NotchOff,
    /// 一定速度維持
    Constant,
}

impl Default for NotchType {
    fn default() -> Self {
        Self::NotchOff
    }
}

/// ノッチの操作履歴
///
/// ランカーブ計算中に発生したノッチ操作の履歴を保持します。
///
/// # フィールド
///
/// - `distance`: 操作位置 [m]
/// - `notch_type`: ノッチ種別
/// - `detail`: 詳細情報（オプション、None の場合はシリアライズ時に省略）
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