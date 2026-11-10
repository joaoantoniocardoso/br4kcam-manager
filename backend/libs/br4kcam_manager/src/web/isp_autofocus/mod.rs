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
    FocusNowContinuous, FocusNowLimits, FocusNowOutcome, FocusNowPhase, FocusNowResult,
    FocusNowStatus,
};
use tokio::{sync::mpsc, task::JoinHandle};
use tracing::*;
use uuid::Uuid;

use crate::web::control_bridge::{ControlError, UNKNOWN_CAMERA};
use algorithm::{
    CONSTANTS, Constants, Env, Floor, Frame, LANDED_RATIO, Landing, READER_TIMEOUT, STILL_OVER_V2,
    Seed,
};

/// How long the first reported outputs may take, at the stream's 10 Hz.
const FIRST_OUTPUTS: Duration = Duration::from_secs(2);
/// How long the first statistics may take from a reader not streaming yet.
const FIRST_FRAMES: Duration = Duration::from_millis(500);
/// How long a status check or a wait of a run keeps the statistics coming.
const WANTED: Duration = Duration::from_secs(3);
/// How often a run renews its focus lease: a third of it, so one lost renewal does not end it.
const LEASE_RENEWAL: Duration = Duration::from_millis(500);

/// Continuous mode's watch after a landing (`reports/isp-af-r5.md` §4): it never refocuses within
/// `HOLD_OFF` of a landing, and takes its baseline of v2 above the floor, the centre's sharpness,
/// from the frames over the last `BASELINE` of it.
const HOLD_OFF: Duration = Duration::from_secs(2);
const BASELINE: f64 = 0.5;
/// The watch refocuses once v2 has stayed under its baseline by more than the drop of the limit's
/// sharpness less 0.1 (the design's hysteresis), summed over frames to `ALARM` (a CUSUM); and
/// never on a drop under 4 times its frame noise at the baseline.
const ALARM: f64 = 0.15;
const NOISE_LEVELS: f64 = 4.0;
/// After a zoom move, the watch refocuses once the zoom has been still this long; and never
/// while the focus or the zoom moved more recently than that: a refocus started while the
/// script still moves the lens to the user's command stops at once (`reports/isp-af-r5.md`).
const ZOOM_STILL: f64 = 1.0;
/// How far the focus can be off after a zoom move, µs: the spread of the coarse pass between
/// units, which also covers the zoom correlation's curve.
const ZOOM_SD: f64 = 14.0;
/// After a refocus that did not land, the watch waits this long before it can refocus again.
const BACKOFF: Duration = Duration::from_secs(10);
/// A centre whose v2 at rest stands under this many times its frame sd has too little detail to
/// watch, as a spike over its floor (`algorithm::SIGNIFICANT`).
const TEXTURE: f64 = 6.0;

/// One camera's Focus now.
#[derive(Default)]
struct Run {
    /// Set while it runs, from its start checks on.
    running: bool,
    phase: Option<FocusNowPhase>,
    task: Option<JoinHandle<()>>,
    last: Option<FocusNowResult>,
    /// Continuous mode's watch, after a landing, and the v2 it took at that landing.
    watch: Option<JoinHandle<()>>,
    base: Option<f64>,
    /// What the last landing measured, which a watch turned on later starts from.
    scene: Option<Seed>,
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

/// Takes the camera's learned biases from the settings, where they outlive a restart and a
/// settings reset clears them.
async fn load_learned(camera_uuid: Uuid) {
    let Some(manager) = settings::MANAGER.get() else {
        return;
    };
    let saved = manager
        .write()
        .await
        .settings
        .get_focus_now_learned_mut()
        .get(&camera_uuid)
        .cloned()
        .unwrap_or_default();
    BIASES
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
        .insert(
            camera_uuid,
            [saved.falling.into(), saved.rising.into(), saved.ramp.into()],
        );
}

/// Saves the camera's learned biases in the settings.
async fn save_learned(camera_uuid: Uuid) {
    let Some(manager) = settings::MANAGER.get() else {
        return;
    };
    let Some([falling, rising, ramp]) = BIASES
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
        .get(&camera_uuid)
        .cloned()
    else {
        return;
    };
    let stored = &mut manager.write().await.settings;
    stored.get_focus_now_learned_mut().insert(
        camera_uuid,
        settings::FocusNowLearned {
            falling: falling.into(),
            rising: rising.into(),
            ramp: ramp.into(),
        },
    );
    // A failed save shows in the health report; the biases still hold until a restart.
    if let Err(error) = stored.save().await {
        warn!("Failed saving the Focus now biases: {error:?}");
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
    Ok(status_of(
        camera_uuid,
        address,
        limits(camera_uuid).await,
        continuous(camera_uuid).await,
    ))
}

fn status_of(
    camera_uuid: Uuid,
    address: Ipv4Addr,
    limits: FocusNowLimits,
    continuous: bool,
) -> FocusNowStatus {
    let runs = runs();
    let run = runs.get(&camera_uuid);
    FocusNowStatus {
        reader_running: reader_running(address),
        running: run.is_some_and(|run| run.running),
        phase: run.and_then(|run| run.phase),
        last: run.and_then(|run| run.last.clone()),
        limits,
        continuous,
        watching: run
            .is_some_and(|run| run.watch.as_ref().is_some_and(|watch| !watch.is_finished())),
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
    Ok(status_of(
        camera_uuid,
        address,
        limits,
        continuous(camera_uuid).await,
    ))
}

/// Whether the camera keeps its focus after a run.
async fn continuous(camera_uuid: Uuid) -> bool {
    let Some(manager) = settings::MANAGER.get() else {
        return false;
    };
    let mut manager = manager.write().await;
    manager
        .settings
        .get_focus_now_continuous_mut()
        .get(&camera_uuid)
        .is_some_and(|&continuous| continuous)
}

/// Saves whether the camera keeps its focus after its runs; off stops its watch.
pub(crate) async fn set_continuous(
    camera_uuid: Uuid,
    continuous: FocusNowContinuous,
) -> Result<FocusNowStatus, ControlError> {
    let address = address(camera_uuid).await?;
    if let Some(manager) = settings::MANAGER.get() {
        let stored = &mut manager.write().await.settings;
        if continuous.enabled {
            stored
                .get_focus_now_continuous_mut()
                .insert(camera_uuid, true);
        } else {
            stored
                .get_focus_now_continuous_mut()
                .shift_remove(&camera_uuid);
        }
        // A failed save shows in the health report; the mode still holds until a restart.
        if let Err(error) = stored.save().await {
            warn!("Failed saving the Focus now mode: {error:?}");
        }
    }
    if !continuous.enabled {
        stop_watching(camera_uuid);
    } else if let Some(scene) = {
        // Turned on between runs: it watches now, from the last landing if there is one.
        let mut runs = runs();
        let run = runs.entry(camera_uuid).or_default();
        (!run.running && run.watch.as_ref().is_none_or(|watch| watch.is_finished()))
            .then(|| run.scene.unwrap_or_else(unanchored))
    } {
        let limits = limits(camera_uuid).await;
        let watch = tokio::spawn(watch(
            camera_uuid,
            address,
            scene,
            limits,
            Duration::ZERO,
            true,
            true,
        ));
        if let Some(run) = runs().get_mut(&camera_uuid) {
            run.watch = Some(watch);
        }
    }
    Ok(status_of(
        camera_uuid,
        address,
        limits(camera_uuid).await,
        continuous.enabled,
    ))
}

/// Ends the camera's watch, if any.
fn stop_watching(camera_uuid: Uuid) {
    if let Some(watch) = runs()
        .get_mut(&camera_uuid)
        .and_then(|run| run.watch.take())
    {
        watch.abort();
    }
}

/// Starts a Focus now, or refuses it with what is missing (also kept as its result).
pub(crate) async fn start(camera_uuid: Uuid) -> Result<FocusNowStatus, ControlError> {
    start_from(camera_uuid, None).await
}

/// [`start`], or with `seed` a refocus around where the lens is (continuous mode).
async fn start_from(camera_uuid: Uuid, seed: Option<Seed>) -> Result<FocusNowStatus, ControlError> {
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
        if let Some(watch) = run.watch.take() {
            watch.abort();
        }
    }
    let started = Instant::now();
    let limits = limits(camera_uuid).await;
    match prepare(camera_uuid, address).await {
        Ok(env) => {
            let task = tokio::spawn(focus(env, limits, seed));
            if let Some(run) = runs().get_mut(&camera_uuid) {
                run.task = Some(task);
            }
            Ok(status_of(
                camera_uuid,
                address,
                limits,
                continuous(camera_uuid).await,
            ))
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
    stop_watching(camera_uuid);
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
    Ok(status_of(
        camera_uuid,
        address,
        limits(camera_uuid).await,
        continuous(camera_uuid).await,
    ))
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

async fn focus(mut env: LiveEnv, limits: FocusNowLimits, seed: Option<Seed>) {
    let started = Instant::now();
    let camera_uuid = env.camera_uuid;
    load_learned(camera_uuid).await;
    let k = constants(camera_uuid);
    info!(
        "Focus now of {camera_uuid}: coarse biases F {:.1} R {:.1}, ramp bias {:.2} ({}), limits {limits:?}",
        k.coarse_bias_f, k.coarse_bias_r, k.ramp_bias, k.ramp_learned
    );
    // The camera on every line of the run: two cameras can focus at once.
    let span = info_span!("focus_now", camera = %camera_uuid);
    let landed = match seed {
        // Around where the lens rests now.
        Some(seed) => {
            let seed = Seed {
                mu: env.last_sent,
                ..seed
            };
            info!(
                "Focus now of {camera_uuid}: again around {:.0} ± {:.1}",
                seed.mu,
                seed.var.sqrt()
            );
            match algorithm::refocus(&mut env, &k, &limits, seed)
                .instrument(span.clone())
                .await
            {
                // No hump around it: the whole search.
                Err(FocusNowOutcome::NoPeak) => {
                    algorithm::run(&mut env, &k, &limits).instrument(span).await
                }
                landed => landed,
            }
        }
        None => algorithm::run(&mut env, &k, &limits).instrument(span).await,
    };
    let again = if seed.is_some() { "again " } else { "" };
    let mut watch_from = None;
    let result = match landed {
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
                    save_learned(camera_uuid).await;
                }
                // The script's input takes the landing, so the focus slider shows it and the
                // zoom correlation keeps it; its output is the one landed on.
                if let Err(error) = autopilot::send_focus(camera_uuid, focus).await {
                    warn!("Focus now: focus {focus} % not sent: {error:#}");
                }
                let (outcome, message) = if focused {
                    let message = format!(
                        "Focused {again}in {:.1} s: at least {:.0} % as sharp as the best",
                        landing.seconds,
                        100.0 * landing.sharpness
                    );
                    (FocusNowOutcome::Focused, message)
                } else {
                    let message = format!(
                        "Approximate {again}in {:.1} s: at least {:.0} % as sharp as the best ({:.0} % asked)",
                        landing.seconds,
                        100.0 * landing.sharpness,
                        100.0 * limits.sharpness
                    );
                    (FocusNowOutcome::Approximate, message)
                };
                watch_from = Some(landing);
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
    let address = env.address;
    let failed_refocus = seed.filter(|_| watch_from.is_none());
    // The user moved the focus during the refocus: their choice, watched from there.
    let by_hand = result.outcome == FocusNowOutcome::FocusMoved;
    finish(camera_uuid, result);
    let scene = watch_from.map(|landing: Landing| Seed {
        mu: landing.u,
        var: 0.0,
        w: landing.w,
        height: landing.height,
        floor: landing.floor,
        half_width: landing.half_width,
        step: landing.step,
    });
    if let Some(run) = runs().get_mut(&camera_uuid) {
        run.scene = scene.or(run.scene);
    }
    // Continuous mode watches from a landing, and after a refocus that did not land it watches
    // on against the last landing, later.
    if continuous(camera_uuid).await
        && let Some((scene, wait, fresh)) =
            scene
                .map(|scene| (scene, HOLD_OFF, true))
                .or(failed_refocus.map(|seed| {
                    if by_hand {
                        (seed, HOLD_OFF, true)
                    } else {
                        (seed, BACKOFF, false)
                    }
                }))
    {
        let watch = tokio::spawn(watch(
            camera_uuid,
            address,
            scene,
            limits,
            wait,
            fresh,
            false,
        ));
        if let Some(run) = runs().get_mut(&camera_uuid) {
            run.watch = Some(watch);
        }
    }
}

/// Continuous mode: watches v2 at the centre after a landing (`scene`), and focuses again
/// around where the lens is once it drops under the limit's sharpness less 0.1, or once a zoom
/// move has ended. It watches v2 above the floor, its drop relative to the landing's, with no
/// shape. A focus move of the user's own is their choice: it watches from where that settles
/// instead. `fresh`: its baseline from now, else the last landing's. `settle` (turned on by
/// hand): it first waits for the lens to be still as after a focus move, and focuses now when
/// the centre has too little detail to watch. Without a landing to focus again around
/// ([`unanchored`]), it focuses again with a whole Focus now.
fn watch(
    camera_uuid: Uuid,
    address: Ipv4Addr,
    scene: Seed,
    limits: FocusNowLimits,
    wait: Duration,
    fresh: bool,
    mut settle: bool,
) -> std::pin::Pin<Box<dyn Future<Output = ()> + Send>> {
    // Boxed with its type spelled out: its refocus's run ends by starting a watch.
    Box::pin(async move {
        let now = || afstat::seconds(Instant::now());
        let watched = |frame: &Frame| frame.v2 - scene.floor.level();
        // Its drop from `base`, and where it focuses again for a drop past `level`.
        let fall = |base: f64, now: f64| 1.0 - now / base;
        let under = |base: f64, level: f64| base * (1.0 - level);
        tokio::time::sleep(wait).await;
        afstat::want(address, WANTED);
        tokio::time::sleep(Duration::from_secs_f64(BASELINE)).await;
        let mut t = now();
        let Some((mean, noise)) = ratio_now(address, watched) else {
            warn!("Focus now watch of {camera_uuid}: no AF statistics, not watching");
            return;
        };
        let mut base = {
            let mut runs = runs();
            let run = runs.entry(camera_uuid).or_default();
            if fresh || run.base.is_none() {
                run.base = Some(mean);
            }
            run.base.unwrap_or(mean)
        };
        // v2's relative drop is the sharpness lost, as the measured certificate takes it.
        let level_of = |noise: f64, base: f64| {
            (1.1 - f64::from(limits.sharpness)).max(NOISE_LEVELS * noise / base)
        };
        let mut level = level_of(noise, base);
        info!(
            "Focus now watch of {camera_uuid}: v2 {base:.3} (now {mean:.3} ± {noise:.3}), focuses again under {:.3}",
            under(base, level)
        );
        // The lens from the reported outputs; without them, no refocus on a zoom move.
        let mut outputs = autopilot::watch_lens_outputs(camera_uuid).await.ok();
        let (mut lens, mut zoom_moved, mut moved, mut by_hand) = (
            None,
            None,
            if settle { now() } else { f64::NEG_INFINITY },
            settle,
        );
        let (mut cusum, mut down) = (0.0, 0.0);
        let seed = loop {
            tokio::time::sleep(Duration::from_millis(100)).await;
            afstat::want(address, WANTED);
            for frame in afstat::frames(address, t, f64::INFINITY) {
                t = frame.t + 1e-6;
                let y = fall(base, watched(&frame));
                if y.is_finite() {
                    cusum = (cusum + y - level).max(0.0);
                    down = y;
                }
            }
            if let Some(receiver) = outputs.as_mut() {
                while let Ok((_, now_lens)) = receiver.try_recv() {
                    if lens.is_some_and(|lens: LensOutputs| lens.zoom != now_lens.zoom) {
                        zoom_moved = Some(now());
                    } else if lens.is_some_and(|lens| lens.focus != now_lens.focus) {
                        by_hand = true;
                    }
                    if lens.is_some_and(|lens| lens != now_lens) {
                        moved = now();
                    }
                    lens = Some(now_lens);
                }
            }
            if zoom_moved.is_some_and(|at| now() - at > ZOOM_STILL) {
                info!("Focus now watch of {camera_uuid}: the zoom moved, focusing again");
                break Seed {
                    var: ZOOM_SD * ZOOM_SD,
                    ..scene
                };
            }
            // The user's own focus: once it has been still a while, the new base.
            if by_hand {
                if now() - moved > ZOOM_STILL + BASELINE {
                    by_hand = false;
                    if std::mem::take(&mut settle)
                        && !textured(&afstat::frames(address, now() - BASELINE, f64::INFINITY))
                    {
                        warn!(
                            "Keep in focus of {camera_uuid}: too little detail at the centre to watch, focusing now"
                        );
                        break Seed {
                            height: 0.0,
                            ..scene
                        };
                    }
                    if let Some((mean, noise)) = ratio_now(address, watched) {
                        (base, level, cusum) = (mean, level_of(noise, mean), 0.0);
                        if let Some(run) = runs().get_mut(&camera_uuid) {
                            run.base = Some(base);
                        }
                        info!(
                            "Focus now watch of {camera_uuid}: focus moved by hand, watching from v2 {base:.3} ± {noise:.3}, focuses again under {:.3}",
                            under(base, level)
                        );
                    }
                }
                continue;
            }
            if cusum > ALARM && now() - moved > ZOOM_STILL {
                // How far off, from the drop, its sign unknown: along the chord from the still's
                // top to half its height (as the ramp plans), up to that half width, and the
                // landing's last step for what it left.
                let (off, margin) = (
                    2.0 * STILL_OVER_V2 * scene.half_width * down.clamp(0.0, 0.5),
                    scene.step,
                );
                info!(
                    "Focus now watch of {camera_uuid}: v2 down {down:.2}, about {off:.1} µs off, focusing again"
                );
                break Seed {
                    var: (off + margin).powi(2),
                    ..scene
                };
            }
        };
        drop(outputs);
        // The refocus starts the next watch; this one ends.
        if let Some(run) = runs().get_mut(&camera_uuid) {
            run.watch = None;
        }
        if let Err(error) = start_from(camera_uuid, (seed.height > 0.0).then_some(seed)).await {
            info!(
                "Focus now watch of {camera_uuid}: not focusing again: {}",
                error.message
            );
        }
    })
}

/// What a watch turned on with no landing yet starts from: no height, so it focuses again with a
/// whole Focus now.
fn unanchored() -> Seed {
    Seed {
        mu: 0.0,
        var: 0.0,
        w: CONSTANTS.w,
        height: 0.0,
        floor: Floor::default(),
        half_width: algorithm::HALF_WIDTH,
        step: 2.0,
    }
}

/// Whether the centre's v2 in `frames` at rest stands out of its frame noise enough to watch.
fn textured(frames: &[Frame]) -> bool {
    let n = frames.len() as f64;
    let mean = frames.iter().map(|frame| frame.v2).sum::<f64>() / n;
    let var = frames
        .iter()
        .map(|frame| (frame.v2 - mean).powi(2))
        .sum::<f64>()
        / (n - 1.0);
    frames.len() >= 5 && mean >= TEXTURE * var.sqrt()
}

/// What the watch watches (`watched`, v2) at the camera's centre over the last [`BASELINE`]:
/// its mean and frame sd.
fn ratio_now(address: Ipv4Addr, watched: impl Fn(&Frame) -> f64) -> Option<(f64, f64)> {
    let now = afstat::seconds(Instant::now());
    let ratios: Vec<f64> = afstat::frames(address, now - BASELINE, f64::INFINITY)
        .iter()
        .map(watched)
        .filter(|ratio| ratio.is_finite())
        .collect();
    let n = ratios.len() as f64;
    let mean = ratios.iter().sum::<f64>() / n;
    let noise = (ratios
        .iter()
        .map(|ratio| (ratio - mean).powi(2))
        .sum::<f64>()
        / n)
        .sqrt();
    (ratios.len() >= 5).then_some((mean, noise))
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
        watch: run.watch.take(),
        base: run.base,
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
    fn learns_a_cameras_coarse_biases() {
        let camera_uuid = Uuid::from_u128(0x5eed);
        assert_eq!(
            constants(camera_uuid).coarse_bias_f,
            CONSTANTS.coarse_bias_f
        );
        // The median of the last five falling ones, past one far off; the rising one stays
        // the model's.
        for coarse_bias in [-9.0, -1.0, -90.0, -7.0, -4.0, -6.0, -2.0] {
            let landing = Landing {
                u: 1000.0,
                ratio: 1.0,
                seconds: 8.0,
                falling: true,
                coarse_bias,
                ramp_bias: f64::NAN,
                certified: true,
                sharpness: 0.9,
                w: 3.4,
                height: 100.0,
                floor: Default::default(),
                half_width: 4.0,
                step: 2.0,
            };
            learn(camera_uuid, &landing);
        }
        let k = constants(camera_uuid);
        assert_eq!(k.coarse_bias_f, -4.0);
        assert_eq!(k.coarse_bias_r, CONSTANTS.coarse_bias_r);
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

    #[test]
    fn watches_only_a_centre_with_detail() {
        // Keep in focus turned on by hand watches the centre from its v2 at rest: on camera .10's
        // floor (13 ± 4, `reports/isp-af-r2.md`) there is nothing to watch, and it focuses now;
        // on a hump's top it watches. Without frames, nothing to watch either.
        let frames = |mean: f64, sd: f64| -> Vec<Frame> {
            (0..15)
                .map(|n| Frame {
                    t: f64::from(n) / 30.0,
                    v2: mean + if n % 2 == 0 { sd } else { -sd },
                    v1: 0.0,
                })
                .collect()
        };
        assert!(!textured(&frames(13.0, 4.0)));
        assert!(textured(&frames(110.0, 4.0)));
        assert!(!textured(&frames(110.0, 4.0)[..3]));
        assert!(!textured(&[]));
    }
}
