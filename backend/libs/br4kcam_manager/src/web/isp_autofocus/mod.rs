//! Focus now: one search for the sharpest focus at the centre of the picture, on the
//! camera's own ISP AF statistics ([`afstat`]), driven by [`algorithm`].
//!
//! The focus goes out as the focus servo's output, `DO_SET_SERVO` under the Lua script's
//! focus lease, so the script's 10 Hz run is not in the loop, and every move is confirmed on
//! the reported servo outputs. Only a motorized Scripting camera takes it: there the script
//! hands its focus output over. The search covers the whole focus slider; the landing is then
//! sent as the camera's focus command, and the script's focus and zoom correlation keeps the
//! focus as the zoom moves.

pub(crate) mod afstat;
mod algorithm;

use std::{
    collections::{HashMap, VecDeque},
    net::Ipv4Addr,
    sync::{
        Arc, LazyLock, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};

use autopilot::{
    LensOutputs,
    api::{Action as AutopilotAction, ActuatorsConfig, ActuatorsControl, FocusZoomPoints},
};
use br4kcam_api::pro::{
    FocusNowLimits, FocusNowOutcome, FocusNowPhase, FocusNowResult, FocusNowStatus,
};
use tokio::{sync::mpsc, task::JoinHandle};
use tracing::*;
use uuid::Uuid;

use crate::web::control_bridge::{ControlError, UNKNOWN_CAMERA};
use algorithm::{CONSTANTS, Constants, Env, Frame, LANDED_RATIO, Landing, READER_TIMEOUT};

/// How long the first reported outputs may take, at the stream's 10 Hz.
const FIRST_OUTPUTS: Duration = Duration::from_secs(2);
/// How long the first statistics may take from a reader not streaming yet.
const FIRST_FRAMES: Duration = Duration::from_millis(500);
/// How long a status check or a wait of a run keeps the statistics coming.
const WANTED: Duration = Duration::from_secs(3);
/// How often a run renews its focus lease: a third of it, so one lost renewal does not end it.
const LEASE_RENEWAL: Duration = Duration::from_millis(500);

/// One camera's Focus now.
#[derive(Default)]
struct Run {
    /// Set while it runs, from its start checks on.
    running: bool,
    phase: Option<FocusNowPhase>,
    task: Option<JoinHandle<()>>,
    last: Option<FocusNowResult>,
}

static RUNS: LazyLock<Mutex<HashMap<Uuid, Run>>> = LazyLock::new(Default::default);

fn runs() -> std::sync::MutexGuard<'static, HashMap<Uuid, Run>> {
    RUNS.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// How many of a camera's own coarse biases per direction, and ramp biases, are kept.
const LEARNED: usize = 5;
/// How far a camera's coarse bias may be from the model's, µs: Focus now on both units
/// measured −26 to +2 falling and +12 to +35 rising (`reports/isp-af-r2.md`).
const LEARN_RANGE: f64 = 40.0;
/// How far a camera's ramp bias may be from the model's, µs: ten times its scatter within a
/// unit (`reports/isp-af-r4.md`).
const RAMP_LEARN_RANGE: f64 = 8.0;

/// The coarse biases each camera's focused runs measured, falling and rising, and its ramp
/// biases, newest last. They moved by 25 µs on one unit between two sessions
/// (`reports/isp-af-r2.md`), and a certified landing measures its own passes' biases to about a
/// µs.
static BIASES: LazyLock<Mutex<HashMap<Uuid, [VecDeque<f64>; 3]>>> = LazyLock::new(Default::default);

/// The constants for a camera: its own coarse biases, once its runs measured them.
fn constants(camera_uuid: Uuid) -> Constants {
    let mut k = CONSTANTS;
    let biases = BIASES
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    if let Some([falling, rising, ramp]) = biases.get(&camera_uuid) {
        k.coarse_bias_f = median(falling).unwrap_or(k.coarse_bias_f);
        k.coarse_bias_r = median(rising).unwrap_or(k.coarse_bias_r);
        k.ramp_bias = median(ramp).unwrap_or(k.ramp_bias);
        (k.coarse_learned_f, k.coarse_learned_r, k.ramp_learned) =
            (falling.len(), rising.len(), ramp.len());
    }
    k
}

/// Keeps the coarse and ramp biases a certified run measured, unless one is further from the
/// model's than the units' spread allows: a lock on something else.
fn learn(camera_uuid: Uuid, landing: &Landing) {
    let model = if landing.falling {
        CONSTANTS.coarse_bias_f
    } else {
        CONSTANTS.coarse_bias_r
    };
    let mut biases = BIASES
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    let kept = biases.entry(camera_uuid).or_default();
    // A run started on the peak, or with no ramp, measures no such bias (NaN).
    for (index, bias, model, range) in [
        (
            usize::from(!landing.falling),
            landing.coarse_bias,
            model,
            LEARN_RANGE,
        ),
        (2, landing.ramp_bias, CONSTANTS.ramp_bias, RAMP_LEARN_RANGE),
    ] {
        if bias.is_nan() || (bias - model).abs() > range {
            continue;
        }
        kept[index].push_back(bias);
        if kept[index].len() > LEARNED {
            kept[index].pop_front();
        }
    }
}

fn median(values: &VecDeque<f64>) -> Option<f64> {
    let mut sorted: Vec<f64> = values.iter().copied().collect();
    sorted.sort_by(f64::total_cmp);
    let n = sorted.len();
    (n > 0).then(|| (sorted[(n - 1) / 2] + sorted[n / 2]) / 2.0)
}

/// Starts listening for the cameras' AF statistics.
pub(crate) fn start_listening() {
    afstat::start();
}

pub(crate) async fn status(camera_uuid: Uuid) -> Result<FocusNowStatus, ControlError> {
    let address = address(camera_uuid).await?;
    reader_answers(address).await;
    Ok(status_of(camera_uuid, address, limits(camera_uuid).await))
}

fn status_of(camera_uuid: Uuid, address: Ipv4Addr, limits: FocusNowLimits) -> FocusNowStatus {
    let runs = runs();
    let run = runs.get(&camera_uuid);
    FocusNowStatus {
        reader_running: reader_running(address),
        running: run.is_some_and(|run| run.running),
        phase: run.and_then(|run| run.phase),
        last: run.and_then(|run| run.last.clone()),
        limits,
        continuous: false,
        watching: false,
    }
}

/// The camera's saved limits, or the defaults.
async fn limits(camera_uuid: Uuid) -> FocusNowLimits {
    let Some(manager) = settings::MANAGER.get() else {
        return FocusNowLimits::default();
    };
    let mut manager = manager.write().await;
    manager
        .settings
        .get_focus_now_limits_mut()
        .get(&camera_uuid)
        .map_or_else(FocusNowLimits::default, |limits| FocusNowLimits {
            sharpness: limits.sharpness,
            max_seconds: limits.max_seconds,
            max_passes: limits.max_passes,
            centre_zones: limits.centre_zones,
            retry_zones: limits.retry_zones,
            retry_first: limits.retry_first,
        })
}

/// Saves the limits the camera's next runs stop at, or refuses ones out of their ranges.
pub(crate) async fn set_limits(
    camera_uuid: Uuid,
    limits: FocusNowLimits,
) -> Result<FocusNowStatus, ControlError> {
    let address = address(camera_uuid).await?;
    if !(0.3..=0.95).contains(&limits.sharpness)
        || !(4.0..=30.0).contains(&limits.max_seconds)
        || !(3..=20).contains(&limits.max_passes)
    {
        return Err(ControlError {
            status: 400,
            message:
                "Focus now limits: sharpness 0.3 to 0.95, max_seconds 4 to 30, max_passes 3 to 20"
                    .into(),
        });
    }
    if let Some(manager) = settings::MANAGER.get() {
        let stored = &mut manager.write().await.settings;
        stored.get_focus_now_limits_mut().insert(
            camera_uuid,
            settings::FocusNowLimits {
                sharpness: limits.sharpness,
                max_seconds: limits.max_seconds,
                max_passes: limits.max_passes,
                centre_zones: limits.centre_zones,
                retry_zones: limits.retry_zones,
                retry_first: limits.retry_first,
            },
        );
        // A failed save shows in the health report; the limits still hold until a restart.
        if let Err(error) = stored.save().await {
            warn!("Failed saving the Focus now limits: {error:?}");
        }
    }
    Ok(status_of(camera_uuid, address, limits))
}

/// Starts a Focus now, or refuses it with what is missing (also kept as its result).
pub(crate) async fn start(camera_uuid: Uuid) -> Result<FocusNowStatus, ControlError> {
    let address = address(camera_uuid).await?;
    {
        let mut runs = runs();
        let run = runs.entry(camera_uuid).or_default();
        if run.running {
            return Err(ControlError {
                status: 409,
                message: "Focus now is already running".into(),
            });
        }
        run.running = true;
        run.phase = None;
    }
    let started = Instant::now();
    let limits = limits(camera_uuid).await;
    match prepare(camera_uuid, address).await {
        Ok(env) => {
            let task = tokio::spawn(focus(env, limits));
            if let Some(run) = runs().get_mut(&camera_uuid) {
                run.task = Some(task);
            }
            Ok(status_of(camera_uuid, address, limits))
        }
        Err((outcome, message)) => {
            finish(camera_uuid, result(outcome, message.clone(), started, None));
            Err(ControlError {
                status: 409,
                message,
            })
        }
    }
}

/// Stops a running Focus now where the focus is.
pub(crate) async fn stop(camera_uuid: Uuid) -> Result<FocusNowStatus, ControlError> {
    let address = address(camera_uuid).await?;
    let task = runs()
        .get_mut(&camera_uuid)
        .filter(|run| run.running)
        .and_then(|run| run.task.take());
    if let Some(task) = task {
        task.abort();
        let message = "Stopped".to_string();
        finish(
            camera_uuid,
            result(FocusNowOutcome::Stopped, message, Instant::now(), None),
        );
    }
    Ok(status_of(camera_uuid, address, limits(camera_uuid).await))
}

async fn address(camera_uuid: Uuid) -> Result<Ipv4Addr, ControlError> {
    mcm_client::camera_address(&camera_uuid)
        .await
        .ok_or_else(|| ControlError {
            status: 404,
            message: UNKNOWN_CAMERA.to_string(),
        })
}

/// Wants the camera's statistics a while and says whether they arrive, waiting for the
/// first ones when its reader was not streaming.
async fn reader_answers(address: Ipv4Addr) -> bool {
    afstat::want(address, WANTED);
    let deadline = Instant::now() + FIRST_FRAMES;
    while !reader_running(address) {
        if Instant::now() >= deadline {
            return false;
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    true
}

fn reader_running(address: Ipv4Addr) -> bool {
    let now = afstat::seconds(Instant::now());
    !afstat::frames(address, now - READER_TIMEOUT, f64::INFINITY).is_empty()
}

/// Everything a run needs, checked before anything moves.
async fn prepare(
    camera_uuid: Uuid,
    address: Ipv4Addr,
) -> Result<LiveEnv, (FocusNowOutcome, String)> {
    if !reader_answers(address).await {
        return Err((
            FocusNowOutcome::ReaderMissing,
            format!(
                "No AF statistics from the camera at {address}: start its reader (afsub) on UDP port {}",
                afstat::PORT
            ),
        ));
    }
    let not_available = |message: String| (FocusNowOutcome::NotAvailable, message);
    let config = autopilot::handle_control(ActuatorsControl {
        camera_uuid,
        action: AutopilotAction::GetActuatorsConfig,
    })
    .await
    .and_then(|config| Ok(serde_json::from_value::<ActuatorsConfig>(config)?))
    .map_err(|error| not_available(format!("{error:#}")))?;
    let mut outputs = autopilot::watch_lens_outputs(camera_uuid)
        .await
        .map_err(|error| not_available(format!("{error:#}")))?;
    let Ok(Some((_, start))) = tokio::time::timeout(FIRST_OUTPUTS, outputs.recv()).await else {
        return Err(not_available(
            "The autopilot does not report the lens outputs".into(),
        ));
    };
    let window = Window::of(&config, start.zoom)
        .ok_or_else(|| not_available("The camera's focus calibration is not set up".into()))?;
    // A release first: a renewal still on its way when the last run released its lease takes
    // it again, and the user moving the focus then leaves holds refused until a release.
    let _ = autopilot::focus_lease(camera_uuid, false, false).await;
    autopilot::focus_lease(camera_uuid, true, true)
        .await
        .map_err(|error| {
            not_available(format!(
                "The Lua script does not hand the focus over, update it with the camera's setup: {error:#}"
            ))
        })?;
    let renewal = tokio::spawn(async move {
        loop {
            tokio::time::sleep(LEASE_RENEWAL).await;
            if let Err(error) = autopilot::focus_lease(camera_uuid, true, false).await {
                warn!("Focus now: focus lease not renewed: {error:#}");
            }
        }
    });
    let (commands, failed) = command_sender(camera_uuid);
    Ok(LiveEnv {
        camera_uuid,
        address,
        window,
        start,
        last_sent: f64::from(start.focus),
        sent: vec![start.focus],
        outputs,
        reported: Vec::new(),
        commands,
        failed,
        renewal,
    })
}

/// A queue that sends focus outputs in order, so a slow one delays the next instead of
/// being overtaken; the flag is raised when one fails.
fn command_sender(camera_uuid: Uuid) -> (mpsc::UnboundedSender<u16>, Arc<AtomicBool>) {
    let (commands, mut queue) = mpsc::unbounded_channel::<u16>();
    let failed = Arc::new(AtomicBool::new(false));
    let flag = failed.clone();
    tokio::spawn(async move {
        while let Some(focus) = queue.recv().await {
            if let Err(error) = autopilot::send_focus_output(camera_uuid, focus).await {
                warn!("Focus now: focus output {focus} not sent: {error:#}");
                flag.store(true, Ordering::SeqCst);
            }
        }
    });
    (commands, failed)
}

async fn focus(mut env: LiveEnv, limits: FocusNowLimits) {
    let started = Instant::now();
    let camera_uuid = env.camera_uuid;
    let k = constants(camera_uuid);
    info!(
        "Focus now of {camera_uuid}: coarse biases F {:.1} R {:.1}, ramp bias {:.2} ({}), limits {limits:?}",
        k.coarse_bias_f, k.coarse_bias_r, k.ramp_bias, k.ramp_learned
    );
    // The camera on every line of the run: two cameras can focus at once.
    let span = info_span!("focus_now", camera = %camera_uuid);
    let result = match algorithm::run(&mut env, &k, &limits).instrument(span).await {
        Ok(landing) => {
            let focus = env.window.percent(env.window.input_for(landing.u));
            let ratio = Some(landing.ratio as f32);
            let sharpness = Some(landing.sharpness as f32);
            // Focused when the sharpness it reports (90 % confidence) is the one asked, so the
            // verdict and the number shown agree (`reports/isp-af-r5.md`, round 6).
            let focused = landing.sharpness >= f64::from(limits.sharpness);
            // Under the pass's peak and short of the sharpness asked: a Fast landing at its limit
            // reads 0.42 of the peak.
            if !focused && landing.ratio < LANDED_RATIO {
                let message = format!(
                    "Landed, but the centre is only {:.0} % as sharp as the peak the search saw: the scene changed or the landing missed",
                    100.0 * landing.ratio
                );
                FocusNowResult {
                    ratio,
                    focus: Some(focus),
                    sharpness,
                    ..result(FocusNowOutcome::SceneChanged, message, started, None)
                }
            } else {
                // Only a certified landing is the truth the passes' biases are measured against.
                if landing.certified {
                    learn(camera_uuid, &landing);
                }
                // The script's input takes the landing, so the focus slider shows it and the
                // zoom correlation keeps it; its output is the one landed on.
                if let Err(error) = autopilot::send_focus(camera_uuid, focus).await {
                    warn!("Focus now: focus {focus} % not sent: {error:#}");
                }
                let (outcome, message) = if focused {
                    let message = format!(
                        "Focused in {:.1} s: at least {:.0} % as sharp as the best",
                        landing.seconds,
                        100.0 * landing.sharpness
                    );
                    (FocusNowOutcome::Focused, message)
                } else {
                    let message = format!(
                        "Approximate in {:.1} s: at least {:.0} % as sharp as the best ({:.0} % asked)",
                        landing.seconds,
                        100.0 * landing.sharpness,
                        100.0 * limits.sharpness
                    );
                    (FocusNowOutcome::Approximate, message)
                };
                FocusNowResult {
                    ratio,
                    focus: Some(focus),
                    sharpness,
                    ..result(outcome, message, started, None)
                }
            }
        }
        Err(outcome) => {
            let restore = !matches!(
                outcome,
                FocusNowOutcome::FocusMoved | FocusNowOutcome::Stopped
            );
            let mut focus = None;
            if restore {
                // Back where it started: what the search left is no better.
                env.send(f64::from(env.start.focus));
                focus = Some(env.window.percent(env.window.input_for(env.last_sent)));
            }
            result(outcome, message_of(outcome).into(), started, focus)
        }
    };
    info!("Focus now of {camera_uuid}: {}", result.message);
    finish(camera_uuid, result);
}

fn result(
    outcome: FocusNowOutcome,
    message: String,
    started: Instant,
    focus: Option<f32>,
) -> FocusNowResult {
    FocusNowResult {
        outcome,
        message,
        finished_at: chrono::Utc::now(),
        seconds: started.elapsed().as_secs_f32(),
        ratio: None,
        focus,
        sharpness: None,
        fallback: None,
    }
}

fn finish(camera_uuid: Uuid, result: FocusNowResult) {
    let mut runs = runs();
    let run = runs.entry(camera_uuid).or_default();
    *run = Run {
        last: Some(result),
        ..Default::default()
    };
}

/// What a run that failed with `outcome` tells the user.
fn message_of(outcome: FocusNowOutcome) -> &'static str {
    match outcome {
        FocusNowOutcome::Focused => "Focused",
        FocusNowOutcome::Approximate => "Landed short of the sharpness asked",
        FocusNowOutcome::SceneChanged => "The scene changed",
        FocusNowOutcome::LowTexture => {
            "No sharpness peak across the focus range: too little detail at the centre of the picture"
        }
        FocusNowOutcome::NoPeak => "The fine search saw no peak: the scene may have changed",
        FocusNowOutcome::ReaderMissing => {
            "The camera's AF statistics stopped: its reader (afsub) is not running"
        }
        FocusNowOutcome::CommandLost => "A focus command did not reach the lens",
        FocusNowOutcome::FocusMoved => "Focus or zoom moved meanwhile: stopped there",
        FocusNowOutcome::Stopped => "Stopped",
        FocusNowOutcome::NotAvailable => "Focus now is not available on this camera",
    }
}

/// The script's map from a focus command to the focus output at one zoom output, as the
/// Lua template computes it (`follow`, `calculate_focus`), operation for operation, so the
/// outputs it predicts are the ones reported.
#[derive(Debug, Clone, PartialEq)]
struct Window {
    /// Focus input range: the script channel's.
    input_min: i64,
    input_max: i64,
    zoom: i64,
    closest: Vec<(i64, i64)>,
    furthest: Vec<(i64, i64)>,
    gain: f64,
    correlation: bool,
    /// The focus channel's highest output: the servo goes no further, whatever the script sends
    /// (camera .10 at zoom 2.0: a window to 2527 µs, the servo stopping near 2130,
    /// `reports/isp-af-r5.md`, round 7).
    output_max: i64,
}

impl Window {
    fn of(config: &ActuatorsConfig, zoom: u16) -> Option<Self> {
        let parameters = config.parameters.as_ref()?;
        let points = |points: &FocusZoomPoints| {
            points
                .0
                .iter()
                .map(|point| (i64::from(point.zoom), i64::from(point.focus)))
                .collect::<Vec<_>>()
        };
        let window = Self {
            input_min: i64::from(parameters.script_channel_min?),
            input_max: i64::from(parameters.script_channel_max?),
            zoom: i64::from(zoom),
            closest: points(config.closest_points.as_ref()?),
            furthest: points(config.furthest_points.as_ref()?),
            gain: f64::from(parameters.focus_margin_gain?),
            correlation: parameters.enable_focus_and_zoom_correlation?,
            output_max: i64::from(parameters.focus_channel_max?),
        };
        (window.input_max > window.input_min
            && !window.closest.is_empty()
            && !window.furthest.is_empty())
        .then_some(window)
    }

    /// The focus command in % that gives `input`.
    fn percent(&self, input: i64) -> f32 {
        ((input - self.input_min) as f64 * 100.0 / (self.input_max - self.input_min) as f64) as f32
    }

    /// The focus output of `input` (`calculate_focus`).
    fn output(&self, input: i64) -> i64 {
        if !self.correlation {
            return input;
        }
        let closest = interpolate_focus(self.zoom, &self.closest);
        let furthest = interpolate_focus(self.zoom, &self.furthest);
        let delta = 0.5 + (input - 1500) as f64 / 400.0;
        let range = (furthest - closest) * self.gain;
        let mid_point = closest + range / 2.0;
        (mid_point + (delta - 0.5) * range).floor() as i64
    }

    /// Lowest and highest focus output over the focus slider, within the servo's reach.
    fn ends(&self) -> (f64, f64) {
        let (a, b) = (self.output(self.input_min), self.output(self.input_max));
        (a.min(b) as f64, a.max(b).min(self.output_max) as f64)
    }

    /// The input whose output is nearest `u`: the middle one of those that give it.
    fn input_for(&self, u: f64) -> i64 {
        let target = u.round() as i64;
        let distance = |input: i64| (self.output(input) - target).abs();
        let nearest = (self.input_min..=self.input_max)
            .map(distance)
            .min()
            .unwrap_or_default();
        let inputs: Vec<i64> = (self.input_min..=self.input_max)
            .filter(|&input| distance(input) == nearest)
            .collect();
        inputs[inputs.len() / 2]
    }
}

/// `interpolate_focus` of the Lua template.
fn interpolate_focus(zoom: i64, points: &[(i64, i64)]) -> f64 {
    let (first, last) = (points[0], points[points.len() - 1]);
    if zoom <= first.0 {
        return first.1 as f64;
    }
    if zoom >= last.0 {
        return last.1 as f64;
    }
    points
        .windows(2)
        .find(|pair| zoom >= pair[0].0 && zoom < pair[1].0)
        .map(|pair| {
            let ((x1, y1), (x2, y2)) = (pair[0], pair[1]);
            y1 as f64 + ((zoom - x1) * (y2 - y1)) as f64 / (x2 - x1) as f64
        })
        .unwrap_or(last.1 as f64)
}

/// A run on the camera: focus commands in-process, outputs from `SERVO_OUTPUT_RAW`, frames
/// from its reader.
struct LiveEnv {
    camera_uuid: Uuid,
    address: Ipv4Addr,
    window: Window,
    /// The outputs at the start.
    start: LensOutputs,
    last_sent: f64,
    /// Every focus output commanded, the start's included: any other means someone else
    /// moved the focus.
    sent: Vec<u16>,
    outputs: mpsc::UnboundedReceiver<(Instant, LensOutputs)>,
    /// The focus outputs reported so far, `(s, µs)`.
    reported: Vec<(f64, f64)>,
    commands: mpsc::UnboundedSender<u16>,
    failed: Arc<AtomicBool>,
    /// Renews the focus lease while the run lives.
    renewal: JoinHandle<()>,
}

impl Drop for LiveEnv {
    /// Hands the focus back to the script, also when the run is stopped: it keeps the focus
    /// where the run left it.
    fn drop(&mut self) {
        self.renewal.abort();
        let camera_uuid = self.camera_uuid;
        tokio::spawn(async move {
            if let Err(error) = autopilot::focus_lease(camera_uuid, false, true).await {
                warn!("Focus now: focus lease not released: {error:#}");
            }
        });
    }
}

impl LiveEnv {
    /// Takes in reported outputs; fails when focus or zoom moved by something else, or when
    /// a command could not be sent.
    fn take(&mut self, at: Instant, outputs: LensOutputs) -> Result<(), FocusNowOutcome> {
        self.reported
            .push((afstat::seconds(at), f64::from(outputs.focus)));
        // The script's own rounding can differ from ours by one µs at a boundary.
        let foreign = !self
            .sent
            .iter()
            .any(|&sent| sent.abs_diff(outputs.focus) <= 1);
        if outputs.zoom != self.start.zoom || foreign {
            warn!(
                "Focus now: outputs moved by something else (focus {}, zoom {} from {})",
                outputs.focus, outputs.zoom, self.start.zoom
            );
            return Err(FocusNowOutcome::FocusMoved);
        }
        if self.failed.load(Ordering::SeqCst) {
            return Err(FocusNowOutcome::CommandLost);
        }
        Ok(())
    }
}

impl Env for LiveEnv {
    fn window(&self) -> (f64, f64) {
        self.window.ends()
    }

    fn now(&self) -> f64 {
        afstat::seconds(Instant::now())
    }

    fn last_sent(&self) -> f64 {
        // A start past the servo's reach is where the servo stopped.
        let (lo, hi) = self.window.ends();
        self.last_sent.clamp(lo, hi)
    }

    fn send(&mut self, u: f64) {
        let (lo, hi) = self.window.ends();
        let input = self.window.input_for(u.clamp(lo, hi));
        let output = self.window.output(input);
        self.last_sent = output as f64;
        if let Ok(output) = u16::try_from(output) {
            self.sent.push(output);
            // The receiving end lives as long as the queue is not dropped.
            let _ = self.commands.send(output);
        }
    }

    async fn wait(&mut self, seconds: f64) -> Result<(), FocusNowOutcome> {
        afstat::want(self.address, WANTED);
        let deadline = tokio::time::Instant::now() + Duration::from_secs_f64(seconds);
        loop {
            tokio::select! {
                biased;
                received = self.outputs.recv() => match received {
                    Some((at, outputs)) => self.take(at, outputs)?,
                    None => return Err(FocusNowOutcome::CommandLost),
                },
                _ = tokio::time::sleep_until(deadline) => break,
            }
        }
        if self.failed.load(Ordering::SeqCst) {
            return Err(FocusNowOutcome::CommandLost);
        }
        Ok(())
    }

    fn frames(&self, t0: f64, t1: f64) -> Vec<Frame> {
        afstat::frames(self.address, t0, t1)
    }

    fn outputs(&self, t0: f64, t1: f64) -> Vec<(f64, f64)> {
        self.reported
            .iter()
            .filter(|&&(t, _)| t0 <= t && t < t1)
            .copied()
            .collect()
    }

    fn phase(&mut self, phase: FocusNowPhase) {
        if let Some(run) = runs().get_mut(&self.camera_uuid) {
            run.phase = Some(phase);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Camera .10's focus calibration, from its `getActuatorsConfig`.
    fn window(zoom: i64) -> Window {
        Window {
            input_min: 870,
            input_max: 2130,
            zoom,
            closest: vec![
                (870, 949),
                (986, 975),
                (1191, 1102),
                (1413, 1277),
                (1617, 1518),
                (1771, 1836),
                (1831, 2057),
                (1951, 2556),
            ],
            furthest: vec![
                (870, 834),
                (986, 905),
                (1191, 1011),
                (1413, 1168),
                (1617, 1359),
                (1771, 1571),
                (1831, 1699),
                (1951, 2159),
            ],
            gain: 1.0,
            correlation: true,
            output_max: 2130,
        }
    }

    /// The focus input the script takes from a focus command in % (`follow`, then rounded).
    fn input(window: &Window, percent: f32) -> i64 {
        let span = (window.input_max - window.input_min) as f64;
        let value = f64::from(percent).clamp(0.0, 100.0);
        (window.input_min as f64 + value * span / 100.0 + 0.5).floor() as i64
    }

    #[test]
    fn maps_the_focus_slider_as_the_script() {
        // The windows measured on the camera, zoom 1.06, 1.4 and 1.7 (af-isp-10 §2).
        for (zoom, ends) in [
            (990, (831.0, 1053.0)),
            (1301, (981.0, 1296.0)),
            (1576, (1160.0, 1629.0)),
            // Zoom 2.0: the slider reaches 2527, the servo stops at its channel's max.
            (1850, (1380.0, 2130.0)),
        ] {
            assert_eq!(window(zoom).ends(), ends);
        }
        let window = window(1301);
        for one in window.input_min..=window.input_max {
            assert_eq!(input(&window, window.percent(one)), one);
        }
        for u in [981.0, 1100.4, 1296.0] {
            assert!((window.output(window.input_for(u)) as f64 - u).abs() <= 0.5);
        }
    }
}
