//! Wire types of the `/v1/pro` API: traditional camera controls (exposure mode, shutter,
//! ISO, white balance, focus) for integrators, on top of the raw camera protocol.

use serde_derive::{Deserialize, Serialize};
use ts_rs::TS;

/// Exposure mode. The aperture is fixed, so there is no aperture priority; the camera has no
/// ISO priority either (it cannot pick the shutter for a set ISO).
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, TS)]
#[serde(rename_all = "snake_case")]
pub enum ExposureMode {
    /// The camera picks shutter and ISO.
    Program,
    /// Shutter priority: the shutter is set, the camera picks ISO.
    Shutter,
    /// Shutter and ISO are both set.
    Manual,
}

/// A maximum ISO: an ISO, or `"full"` for the camera's full gain, which is past the calibrated
/// ISOs (its ISO is not known).
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, TS)]
#[serde(untagged)]
pub enum MaxIso {
    Iso(u32),
    Full(FullGain),
}

/// The `"full"` of [`MaxIso`].
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, TS)]
#[serde(rename_all = "snake_case")]
pub enum FullGain {
    Full,
}

/// Why the picture is off its brightness target when the camera picks the exposure.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, TS)]
#[serde(rename_all = "snake_case")]
pub enum ExposureWarning {
    /// Darker than the target: the camera is at its maximum ISO (and, in `program`, its
    /// longest shutter).
    TooDark,
    /// Brighter than the target: the camera is at its lowest ISO (and, in `program`, its
    /// shortest shutter).
    TooBright,
}

/// Water the automatic white balance is tuned for.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, TS)]
#[serde(rename_all = "snake_case")]
pub enum WhiteBalanceScene {
    GreenWater,
    BlueWater,
}

/// White balance, as the camera implements it.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, TS)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum WhiteBalance {
    /// The camera tracks white balance, tuned for one water scene. The state always has the
    /// scene; a change can leave it out to keep the camera's (it holds it while in Manual).
    Auto {
        #[serde(default, skip_serializing_if = "Option::is_none")]
        #[ts(optional)]
        scene: Option<WhiteBalanceScene>,
    },
    /// Fixed red, green and blue gains, each 0 to 255. A gain left out of a change keeps
    /// the camera's current value; the state always has all three.
    Manual {
        #[serde(default, skip_serializing_if = "Option::is_none")]
        #[ts(optional)]
        red: Option<u8>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        #[ts(optional)]
        green: Option<u8>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        #[ts(optional)]
        blue: Option<u8>,
        /// The water scene Auto returns to, which the camera holds while in Manual. Always in
        /// the state; a change can set it too.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        #[ts(optional)]
        scene: Option<WhiteBalanceScene>,
    },
    /// Fixed gains the camera computes for light of a colour temperature, in kelvin (see the
    /// capabilities' `white_balance_kelvin`). Left out of a change, it keeps the camera's,
    /// or 5500 K when the camera has none in range; the state always has it.
    Temperature {
        #[serde(default, skip_serializing_if = "Option::is_none")]
        #[ts(optional)]
        kelvin: Option<u32>,
        /// The water scene Auto returns to, held meanwhile, as in Manual.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        #[ts(optional)]
        scene: Option<WhiteBalanceScene>,
    },
}

/// Lens aperture.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, TS)]
#[serde(rename_all = "snake_case")]
pub enum Aperture {
    /// Not adjustable.
    Fixed,
}

/// Current controls of one camera.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, TS)]
pub struct ProState {
    pub mode: ExposureMode,
    /// Shutter speed as `"1/<x>"` seconds. In `program` it is what the camera is using now,
    /// measured (the sensor exposes in 14.8 us lines, so 1/4000 reads 1/3968), read only, or
    /// null when the camera does not say.
    pub shutter: Option<String>,
    /// ISO. In `program` and `shutter` it is what the camera is using now, read only, or null
    /// when the camera does not say; above the calibrated ISO 204800 it is extrapolated.
    pub iso: Option<u32>,
    /// Highest ISO the camera may pick (shared by `program` and `shutter`), or null when ISO
    /// is set by hand.
    pub max_iso: Option<MaxIso>,
    /// Longest shutter the camera may pick in `program`, as `"1/<x>"` seconds, or null in the
    /// other modes. In dim light the camera holds the shutter there and raises ISO.
    pub max_shutter: Option<String>,
    /// Exposure compensation in EV (stops) when the camera picks the exposure, or null when
    /// it is set by hand.
    pub exposure_compensation: Option<f32>,
    /// Set when the camera picks the exposure and cannot reach its brightness target.
    pub exposure_warning: Option<ExposureWarning>,
    /// Color temperature the camera estimates for the scene, in kelvin, or null when the
    /// camera does not say.
    pub color_temperature: Option<u32>,
    pub white_balance: WhiteBalance,
    /// Focus position, 0 to 100 %, or null when the camera has no motorized focus.
    pub focus: Option<f32>,
    /// Zoom magnification, or null when the camera has no motorized zoom.
    pub zoom: Option<f32>,
    /// Mount tilt in degrees, or null when no tilt is set up.
    pub tilt: Option<f32>,
}

/// Controls to change. Omitted fields are left as they are. A shutter or ISO the mode
/// lets the camera pick is rejected, and so are a maximum ISO and an exposure compensation in
/// `manual`, and a maximum shutter outside `program`.
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq, TS)]
#[serde(deny_unknown_fields)]
pub struct ProSettings {
    #[serde(skip_serializing_if = "Option::is_none")]
    #[ts(optional)]
    pub mode: Option<ExposureMode>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[ts(optional)]
    pub shutter: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[ts(optional)]
    pub iso: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[ts(optional)]
    pub max_iso: Option<MaxIso>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[ts(optional)]
    pub max_shutter: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[ts(optional)]
    pub exposure_compensation: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[ts(optional)]
    pub white_balance: Option<WhiteBalance>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[ts(optional)]
    pub focus: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[ts(optional)]
    pub zoom: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[ts(optional)]
    pub tilt: Option<f32>,
}

/// Inclusive range a control accepts.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, TS)]
pub struct Range {
    pub min: f32,
    pub max: f32,
}

/// What one camera accepts.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, TS)]
pub struct ProCapabilities {
    pub modes: Vec<ExposureMode>,
    /// Every shutter the camera takes.
    pub shutter: Vec<String>,
    /// The shutters that expose what they say at 30 fps. Longer ones are held to the frame
    /// time unless the frame rate is lowered; 1/100000 s exposes no shorter than about 1/33000 s.
    pub shutter_calibrated: Vec<String>,
    pub iso: Vec<u32>,
    /// Maximum ISOs the camera may be limited to when it picks ISO itself, ending with
    /// `"full"`.
    pub max_iso: Vec<MaxIso>,
    /// Longest shutters the camera may be limited to in `program`.
    pub max_shutter: Vec<String>,
    /// Exposure compensations, in EV, the camera takes when it picks the exposure.
    pub exposure_compensation: Vec<f32>,
    pub white_balance_scenes: Vec<WhiteBalanceScene>,
    /// Colour temperatures, in kelvin, the `temperature` white balance takes.
    pub white_balance_kelvin: Range,
    /// Focus range in %, or null without a motorized lens set up on the autopilot.
    pub focus: Option<Range>,
    /// Zoom magnification range, or null without a motorized lens.
    pub zoom: Option<Range>,
    /// Tilt range in degrees, or null without a mount set up on the autopilot.
    pub tilt: Option<Range>,
    pub aperture: Aperture,
}

/// How a Focus now ended.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, TS)]
#[serde(rename_all = "snake_case")]
pub enum FocusNowOutcome {
    /// Landed on the sharpest focus the camera's statistics saw, as sharp as the limits ask.
    Focused,
    /// Landed on the best estimate when a limit (time, passes) came first, or where the sharpness the
    /// limits ask could not be certified: `sharpness` says what could.
    Approximate,
    /// Landed, but the sharpness there is well under the peak the search saw: the scene
    /// changed, or the landing missed.
    SceneChanged,
    /// No sharpness peak across the whole focus range: too little detail at the centre.
    LowTexture,
    /// The fine search around the coarse peak saw none.
    NoPeak,
    /// No AF statistics arrive from the camera: its reader is not running.
    ReaderMissing,
    /// A focus command never reached the lens.
    CommandLost,
    /// Focus or zoom moved by something else meanwhile: stopped there.
    FocusMoved,
    /// Stopped on request.
    Stopped,
    /// The camera has no motorized lens, or is not a Scripting camera.
    NotAvailable,
}

/// Which step a Focus now is on.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, TS)]
#[serde(rename_all = "snake_case")]
pub enum FocusNowPhase {
    /// One fast pass through the whole focus range, to find the peak roughly.
    Coarse,
    /// A slow pass over the peak, to place it.
    Fine,
    /// Moving onto the peak.
    Land,
}

/// The end of the last Focus now.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, TS)]
pub struct FocusNowResult {
    pub outcome: FocusNowOutcome,
    /// What happened, worded for the user.
    pub message: String,
    /// When it ended.
    #[ts(type = "string")]
    pub finished_at: chrono::DateTime<chrono::Utc>,
    /// How long it ran, in seconds.
    pub seconds: f32,
    /// Sharpness at the landing over the peak's, from the camera's statistics, or null when
    /// it did not land.
    pub ratio: Option<f32>,
    /// Where it left the focus, in %, or null when it did not move it there.
    pub focus: Option<f32>,
    /// The sharpness at the landing it certifies, as a fraction of the best (90 % confidence),
    /// or null when it did not land.
    pub sharpness: Option<f32>,
}

/// What a Focus now stops at: the first of the sharpness it certifies, its time, or its passes.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, TS)]
pub struct FocusNowLimits {
    /// Sharpness to reach, as a fraction of the best, 0.3 to 0.95: 0.5 fast, 0.7 balanced
    /// (the default), 0.9 precise.
    pub sharpness: f32,
    /// Longest run, in seconds, 4 to 30.
    pub max_seconds: f32,
    /// Most passes (the coarse pass, each ramp, each reading at rest), 3 to 20.
    pub max_passes: u8,
}

impl Default for FocusNowLimits {
    fn default() -> Self {
        Self {
            sharpness: 0.7,
            max_seconds: 9.0,
            max_passes: 12,
        }
    }
}

/// Whether a camera keeps its focus after a Focus now: it watches the sharpness at the centre
/// and focuses again around where it is when it drops under the limit's (the scene moved).
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, PartialEq, Eq, TS)]
pub struct FocusNowContinuous {
    pub enabled: bool,
}

/// Focus now of one camera: a search for the sharpest focus at the centre of the picture,
/// on the camera's own AF statistics.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, TS)]
pub struct FocusNowStatus {
    /// Whether the camera's AF statistics arrive; Focus now needs them.
    pub reader_running: bool,
    /// Whether a Focus now is running.
    pub running: bool,
    /// Its step while it runs.
    pub phase: Option<FocusNowPhase>,
    pub last: Option<FocusNowResult>,
    /// The camera's limits, which the next run stops at.
    pub limits: FocusNowLimits,
    /// Whether the camera keeps its focus after a run (see [`FocusNowContinuous`]).
    pub continuous: bool,
    /// Whether it is watching the sharpness now, to focus again when it drops.
    pub watching: bool,
}
