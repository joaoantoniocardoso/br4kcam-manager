use indexmap::IndexMap;
use serde::{Deserialize, Serialize};
use uuid::Uuid;

use crate::{RawSettingsData, SettingsDataImpl, v0::SettingsDataV0};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SettingsDataV1 {
    pub actuators: IndexMap<Uuid, CameraActuatorsSettings>,
    /// PRO Program's shutter limit (the camera's raw `max_exposure`) per camera while it is in
    /// Manual, which lifts the limit; put back when Program returns. Absent from older files.
    #[serde(default, skip_serializing_if = "IndexMap::is_empty")]
    pub program_max_exposure: IndexMap<Uuid, u32>,
    /// What each camera's Focus now stops at. Absent from older files.
    #[serde(default, skip_serializing_if = "IndexMap::is_empty")]
    pub focus_now_limits: IndexMap<Uuid, FocusNowLimits>,
    /// What each camera's Focus now learned from its certified runs. Absent from older files.
    #[serde(default, skip_serializing_if = "IndexMap::is_empty")]
    pub focus_now_learned: IndexMap<Uuid, FocusNowLearned>,
    /// The cameras that keep their focus after a Focus now (true), the others absent. Absent from
    /// older files.
    #[serde(default, skip_serializing_if = "IndexMap::is_empty")]
    pub focus_now_continuous: IndexMap<Uuid, bool>,
    // todo: pub image: IndexMap<Uuid, CameraImageSettings>,
}

/// A camera's Focus now limits (`br4kcam_api::pro::FocusNowLimits`).
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct FocusNowLimits {
    pub sharpness: f32,
    pub max_seconds: f32,
    pub max_passes: u8,
    /// Absent from older files: the 3 × 3 zones of before.
    #[serde(default = "centre_zones")]
    pub centre_zones: u8,
    /// Absent from older files: no retry window.
    #[serde(default)]
    pub retry_zones: u8,
    #[serde(default)]
    pub retry_first: bool,
    /// Absent from older files, or a value this build does not offer: the defaults, the parabola
    /// on the ramp and at the landing, and the measured certificate.
    #[serde(default)]
    pub estimator: FocusNowEstimator,
    #[serde(default)]
    pub close: FocusNowClose,
    #[serde(default)]
    pub certificate: FocusNowCertificate,
}

fn centre_zones() -> u8 {
    3
}

/// `br4kcam_api::pro::FocusNowEstimator`.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FocusNowEstimator {
    Midpoint,
    #[default]
    #[serde(other)]
    Parabola,
}

/// `br4kcam_api::pro::FocusNowClose`.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FocusNowClose {
    Hill,
    #[default]
    #[serde(other)]
    Parabola,
}

/// `br4kcam_api::pro::FocusNowCertificate`.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FocusNowCertificate {
    #[default]
    #[serde(other)]
    Measured,
}

/// The biases a camera's certified Focus now runs measured, µs, newest last: its coarse pass's,
/// falling and rising, and its ramp's.
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
pub struct FocusNowLearned {
    pub falling: Vec<f64>,
    pub rising: Vec<f64>,
    pub ramp: Vec<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CameraActuatorsSettings {
    pub parameters: ActuatorsParameters,
    pub calibration: Calibration,
    pub state: ActuatorsState,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Calibration {
    pub closest_points: FocusZoomPoints,
    pub furthest_points: FocusZoomPoints,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ActuatorsParameters {
    // Lens. Settings saved before lens types existed have no `lens` and load as motorized.
    #[serde(default)]
    pub lens: LensType,

    // Camera parameters
    pub camera_id: CameraID,

    // Focus channel parameters
    pub focus_channel: ServoChannel,
    pub focus_channel_min: u16,
    pub focus_channel_trim: u16,
    pub focus_channel_max: u16,
    pub focus_margin_gain: f32,

    // Script channel parameters
    pub script_function: ScriptFunction,
    pub script_channel: ServoChannel,
    pub script_channel_min: u16,
    pub script_channel_trim: u16,
    pub script_channel_max: u16,
    pub enable_focus_and_zoom_correlation: bool,

    // Zoom channel parameters
    pub zoom_channel: ServoChannel,
    pub zoom_channel_min: u16,
    pub zoom_channel_trim: u16,
    pub zoom_channel_max: u16,

    // Tilt channel parameters
    pub tilt_channel: ServoChannel,
    pub tilt_channel_min: u16,
    pub tilt_channel_trim: u16,
    pub tilt_channel_max: u16,
    pub tilt_channel_reversed: bool,

    // Mount (MNTx) parameters
    pub tilt_mnt_type: MountType,
    pub tilt_mnt_pitch_min: i32,
    pub tilt_mnt_pitch_max: i32,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LensType {
    #[default]
    Motorized,
    FixedFocus,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CameraID {
    #[default]
    CAM1 = 1,
    CAM2 = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum ServoChannel {
    SERVO1 = 1,
    SERVO2 = 2,
    SERVO3 = 3,
    SERVO4 = 4,
    SERVO5 = 5,
    SERVO6 = 6,
    SERVO7 = 7,
    SERVO8 = 8,
    SERVO9 = 9,
    SERVO10 = 10,
    SERVO11 = 11,
    SERVO12 = 12,
    SERVO13 = 13,
    SERVO14 = 14,
    SERVO15 = 15,
    SERVO16 = 16,
    SERVO17 = 17,
    SERVO18 = 18,
    SERVO19 = 19,
    SERVO20 = 20,
    SERVO21 = 21,
    SERVO22 = 22,
    SERVO23 = 23,
    SERVO24 = 24,
    SERVO25 = 25,
    SERVO26 = 26,
    SERVO27 = 27,
    SERVO28 = 28,
    SERVO29 = 29,
    SERVO30 = 30,
    SERVO31 = 31,
    SERVO32 = 32,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum ScriptFunction {
    #[default]
    SCRIPT1 = 94,
    SCRIPT2 = 95,
    SCRIPT3 = 96,
    SCRIPT4 = 97,
    SCRIPT5 = 98,
    SCRIPT6 = 99,
    SCRIPT7 = 100,
    SCRIPT8 = 101,
    SCRIPT9 = 102,
    SCRIPT10 = 103,
    SCRIPT11 = 104,
    SCRIPT12 = 105,
    SCRIPT13 = 106,
    SCRIPT14 = 107,
    SCRIPT15 = 108,
    SCRIPT16 = 109,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum MountType {
    Servo = 1,
    #[default]
    BrushlessPWM = 7,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum TiltChannelFunction {
    #[default]
    /// Mount1Pitch
    MNT1 = 7,
    /// Mount2Pitch
    MNT2 = 13,
}

pub type FocusZoomPoints = Vec<FocusZoomPoint>;

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct FocusZoomPoint {
    pub focus: u32,
    pub zoom: u32,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct ActuatorsState {
    pub focus: Option<f32>,
    pub zoom: Option<f32>,
    pub tilt: Option<f32>,
}

impl From<SettingsDataV0> for SettingsDataV1 {
    fn from(_v1: SettingsDataV0) -> Self {
        SettingsDataV1 {
            actuators: IndexMap::default(),
            program_max_exposure: IndexMap::default(),
            focus_now_limits: IndexMap::default(),
            focus_now_learned: IndexMap::default(),
            focus_now_continuous: IndexMap::default(),
        }
    }
}

impl SettingsDataImpl for SettingsDataV1 {
    fn get_actuators(&self) -> &IndexMap<Uuid, CameraActuatorsSettings> {
        &self.actuators
    }

    fn get_actuators_mut(&mut self) -> &mut IndexMap<Uuid, CameraActuatorsSettings> {
        &mut self.actuators
    }

    fn get_program_max_exposure_mut(&mut self) -> &mut IndexMap<Uuid, u32> {
        &mut self.program_max_exposure
    }

    fn get_focus_now_limits_mut(&mut self) -> &mut IndexMap<Uuid, FocusNowLimits> {
        &mut self.focus_now_limits
    }

    fn get_focus_now_learned_mut(&mut self) -> &mut IndexMap<Uuid, FocusNowLearned> {
        &mut self.focus_now_learned
    }

    fn get_focus_now_continuous_mut(&mut self) -> &mut IndexMap<Uuid, bool> {
        &mut self.focus_now_continuous
    }

    fn to_raw(&self) -> RawSettingsData {
        RawSettingsData::V1(self.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A camera saved before lens types existed.
    const SETTINGS_WITHOUT_LENS: &str = r#"{
        "version": "V1",
        "actuators": {
            "00000000-0000-0000-0000-000000000001": {
                "parameters": {
                    "camera_id": "CAM1",
                    "focus_channel": "SERVO10",
                    "focus_channel_min": 870,
                    "focus_channel_trim": 918,
                    "focus_channel_max": 2130,
                    "focus_margin_gain": 1.0,
                    "script_function": "SCRIPT1",
                    "script_channel": "SERVO12",
                    "script_channel_min": 870,
                    "script_channel_trim": 1500,
                    "script_channel_max": 2130,
                    "enable_focus_and_zoom_correlation": true,
                    "zoom_channel": "SERVO11",
                    "zoom_channel_min": 935,
                    "zoom_channel_trim": 935,
                    "zoom_channel_max": 1850,
                    "tilt_channel": "SERVO16",
                    "tilt_channel_min": 865,
                    "tilt_channel_trim": 1500,
                    "tilt_channel_max": 2250,
                    "tilt_channel_reversed": false,
                    "tilt_mnt_type": "BrushlessPWM",
                    "tilt_mnt_pitch_min": -70,
                    "tilt_mnt_pitch_max": 70
                },
                "calibration": { "closest_points": [], "furthest_points": [] },
                "state": { "focus": null, "zoom": null, "tilt": null }
            }
        }
    }"#;

    #[test]
    fn settings_without_lens_load_as_motorized_and_keep_a_new_lens() {
        let RawSettingsData::V1(mut settings) =
            serde_json::from_str::<RawSettingsData>(SETTINGS_WITHOUT_LENS).unwrap()
        else {
            panic!("expected V1 settings");
        };
        let parameters = &mut settings.actuators[0].parameters;
        assert_eq!(parameters.lens, LensType::Motorized);

        parameters.lens = LensType::FixedFocus;
        let saved = serde_json::to_string(&settings.to_raw()).unwrap();
        let RawSettingsData::V1(reloaded) =
            serde_json::from_str::<RawSettingsData>(&saved).unwrap()
        else {
            panic!("expected V1 settings");
        };
        assert_eq!(reloaded.actuators[0].parameters.lens, LensType::FixedFocus);
        // Files without kept shutter limits load without any and save without the key.
        assert!(reloaded.program_max_exposure.is_empty());
        assert!(!saved.contains("program_max_exposure"));
    }

    #[test]
    fn kept_shutter_limits_survive_a_reload() {
        let RawSettingsData::V1(mut settings) =
            serde_json::from_str::<RawSettingsData>(SETTINGS_WITHOUT_LENS).unwrap()
        else {
            panic!("expected V1 settings");
        };
        settings.program_max_exposure.insert(Uuid::nil(), 250);
        let saved = serde_json::to_string(&settings.to_raw()).unwrap();
        let RawSettingsData::V1(reloaded) =
            serde_json::from_str::<RawSettingsData>(&saved).unwrap()
        else {
            panic!("expected V1 settings");
        };
        assert_eq!(reloaded.program_max_exposure.get(&Uuid::nil()), Some(&250));
    }

    #[test]
    fn focus_now_limits_survive_a_reload() {
        let RawSettingsData::V1(mut settings) =
            serde_json::from_str::<RawSettingsData>(SETTINGS_WITHOUT_LENS).unwrap()
        else {
            panic!("expected V1 settings");
        };
        // Files from before them load without any and save without the key.
        assert!(settings.focus_now_limits.is_empty());
        assert!(
            !serde_json::to_string(&settings.to_raw())
                .unwrap()
                .contains("focus_now_limits")
        );
        let limits = FocusNowLimits {
            sharpness: 0.9,
            max_seconds: 12.0,
            max_passes: 10,
            centre_zones: 5,
            retry_zones: 17,
            retry_first: true,
            estimator: FocusNowEstimator::Midpoint,
            close: FocusNowClose::Hill,
            certificate: FocusNowCertificate::Measured,
        };
        settings.focus_now_limits.insert(Uuid::nil(), limits);
        let saved = serde_json::to_string(&settings.to_raw()).unwrap();
        let RawSettingsData::V1(reloaded) =
            serde_json::from_str::<RawSettingsData>(&saved).unwrap()
        else {
            panic!("expected V1 settings");
        };
        assert_eq!(reloaded.focus_now_limits.get(&Uuid::nil()), Some(&limits));
    }

    #[test]
    fn focus_now_learned_biases_survive_a_reload() {
        let RawSettingsData::V1(mut settings) =
            serde_json::from_str::<RawSettingsData>(SETTINGS_WITHOUT_LENS).unwrap()
        else {
            panic!("expected V1 settings");
        };
        // Files from before them load without any and save without the key.
        assert!(settings.focus_now_learned.is_empty());
        assert!(
            !serde_json::to_string(&settings.to_raw())
                .unwrap()
                .contains("focus_now_learned")
        );
        let learned = FocusNowLearned {
            falling: vec![-4.5, -2.0],
            rising: vec![31.25],
            ramp: vec![0.5, -0.25, 0.0],
        };
        settings
            .focus_now_learned
            .insert(Uuid::nil(), learned.clone());
        let saved = serde_json::to_string(&settings.to_raw()).unwrap();
        let RawSettingsData::V1(reloaded) =
            serde_json::from_str::<RawSettingsData>(&saved).unwrap()
        else {
            panic!("expected V1 settings");
        };
        assert_eq!(reloaded.focus_now_learned.get(&Uuid::nil()), Some(&learned));
    }

    #[test]
    fn focus_now_continuous_survives_a_reload() {
        let RawSettingsData::V1(mut settings) =
            serde_json::from_str::<RawSettingsData>(SETTINGS_WITHOUT_LENS).unwrap()
        else {
            panic!("expected V1 settings");
        };
        // Files from before it load without any and save without the key.
        assert!(settings.focus_now_continuous.is_empty());
        assert!(
            !serde_json::to_string(&settings.to_raw())
                .unwrap()
                .contains("focus_now_continuous")
        );
        settings.focus_now_continuous.insert(Uuid::nil(), true);
        let saved = serde_json::to_string(&settings.to_raw()).unwrap();
        let RawSettingsData::V1(reloaded) =
            serde_json::from_str::<RawSettingsData>(&saved).unwrap()
        else {
            panic!("expected V1 settings");
        };
        assert_eq!(reloaded.focus_now_continuous.get(&Uuid::nil()), Some(&true));
    }
}
