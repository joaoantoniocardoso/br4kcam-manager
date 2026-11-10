//! The Focus now procedure, derived from the model of the focus drive of a 4K Cam
//! (`reports/focus-sysid-10.md` §5, reference `afalgo.py`). Positions are focus servo outputs
//! in µs; "falling" means the output falls. Every final move is falling, because the lens
//! follows a falling move 1:1 and lands at the same offset whatever its length (§3.2).
//!
//! 1. Coarse: one full-speed jump towards the far end of the window, stopped once the spike
//!    of the centre statistic is seen whole. Its time, less the command path's latency,
//!    gives where the peak was crossed; a rising jump also takes the play out.
//! 2. Fine: a posterior on the peak's output, from the coarse pass and its learned error; while
//!    it is not sharp enough for the user's limit, one falling ramp, fast down to where the
//!    peak cannot be yet, then at the slowest speed that limit needs, until it has crossed the
//!    hump. The time of its peak, less the ramp's measured lag, gives the output at the peak;
//!    when a ramp command came late, the reported outputs replace the send times.
//! 3. Land on the posterior, past the play and falling, read at rest; certified when the
//!    reading bounds the error within the limit, else closed on the hump from its readings at
//!    rest while the limits allow. It says `focused` only when certified.
//!
//! The design and its simulation: `reports/af-control-design.md` §3, measured again in
//! `reports/isp-af-r4.md`.
//!
//! Every move is confirmed on the reported outputs (`SERVO_OUTPUT_RAW`) and re-sent once
//! when they do not show it; the wait after it is the model's travel time.

use br4kcam_api::pro::{
    FocusNowClose, FocusNowEstimator, FocusNowLimits, FocusNowOutcome, FocusNowPhase,
};
use tracing::*;

/// The drive model's constants (§1.2), fitted on camera .10 at zoom 1.4 unless noted.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Constants {
    /// Play between the servo shaft and the lens, µs (§3.3: 15.5 to 19).
    pub h: f64,
    /// Where a falling move lands below its output, µs (§3.2). It cancels between the
    /// ramp and the landing, which are both falling.
    pub c_f: f64,
    /// Slew rate of a falling output, µs/s (§1.2).
    pub v_f: f64,
    /// Slew rate of a rising output, µs/s (§1.2).
    pub v_r: f64,
    /// Lua tick to lens motion, one frame included, s (§3.2).
    pub dead: f64,
    /// Time constant of the shaft for small errors, s (§3.2).
    pub tau: f64,
    /// Statistic latency, s (§3.8: one frame, 33 ms, plus receive).
    pub frame: f64,
    /// From a ramp command's send to the v2 peak's frame time when the lens crosses the peak
    /// following a steady falling ramp, s: 0.373, the same at 35 to 150 µs/s, sd 4 to 14 ms a
    /// pass (`reports/isp-af-r4.md`).
    pub ramp_lag: f64,
    /// What a ramp's estimate reads below the peak, µs, until a camera's own runs measure it
    /// (`reports/isp-af-r4.md`: 0 within 0.3 µs on camera .11).
    pub ramp_bias: f64,
    /// How many of a camera's runs measured its ramp bias; none: the spread between units.
    pub ramp_learned: usize,
    /// How many of a camera's runs measured its coarse biases, falling and rising.
    pub coarse_learned_f: usize,
    pub coarse_learned_r: usize,
    /// Send to output change, mean, s: `DO_SET_SERVO` straight to the output, measured from
    /// the laptop 0.023 s to its report at 100 Hz, 0.013 to 0.033 s (p5 to p95), with no Lua tick
    /// (`reports/isp-af-r3.md`).
    pub t_cmd: f64,
    /// Output change to its report, s (§3.1).
    pub seen_lag: f64,
    /// Least height of a spike over the floor of the centre v2 (af-isp-10 §4, coring 90).
    pub min_spike: f64,
    /// Coarse error of a falling jump, µs, until a camera's own runs measure it: the mean of
    /// 10 runs of Focus now itself on both units, −26 to +2 (`reports/isp-af-r2.md`; jumps sent
    /// from outside the manager time differently).
    pub coarse_bias_f: f64,
    /// Coarse error of a rising jump, µs, until a camera's own runs measure it: the mean of 10
    /// runs on both units, +12 to +35 (`reports/isp-af-r2.md`).
    pub coarse_bias_r: f64,
    /// sd of the centre v2 hump, µs (§1.2).
    pub w: f64,
}

/// The constants `afalgo.py` ran live with on camera .10 (§5.3), its coarse biases measured again.
pub(crate) const CONSTANTS: Constants = Constants {
    h: 15.5,
    c_f: 2.6,
    v_f: 257.0,
    v_r: 222.0,
    dead: 0.15,
    tau: 0.16,
    frame: 0.05,
    ramp_lag: 0.373,
    ramp_bias: 0.0,
    ramp_learned: 0,
    coarse_learned_f: 0,
    coarse_learned_r: 0,
    t_cmd: 0.02,
    seen_lag: 0.13,
    min_spike: 12.0,
    coarse_bias_f: -8.0,
    coarse_bias_r: 25.0,
    w: 3.4,
};

/// The statistic's frame period, s.
const FRAME: f64 = 1.0 / 30.0;
/// How far a hump's sides may be from its top, s: at full speed, 77 µs.
const HUMP_SIDE: f64 = 0.3;
/// Frames a floor is taken from at least: the 0.28 s before a pass's lens moves.
const FLOOR_FRAMES: usize = 8;
/// How long before a pass its floor also takes the frames at rest from, s: enough of them for
/// their own sd and the correlation of successive ones.
const FLOOR_REST: f64 = 1.0;
/// A peak stands this many sd of the floor's frame noise above it: on camera .10 the floor
/// is 9 to 15 with noise up to 4.3 sd, and a spike of 25 on it stopped a pass
/// (`reports/isp-af-r2.md`).
const SIGNIFICANT: f64 = 6.0;
/// How long after the coarse pass's modelled end its frames are still looked at, s: the
/// shaft slows over its last v tau and settles in about 3 tau (§3.2).
const REST: f64 = 0.5;
/// A move whose output is reported this long after its send came late (§3.1: on-time
/// commands show by 0.18 s, late ones 1.1 to 1.6 s).
const LATE: f64 = 0.35;
/// Typical delay from an output change to its report, s.
const REPORT_DELAY: f64 = 0.11;
/// How long a move's output may take to be reported before it is re-sent, then given up, s:
/// on-time ones show within a report period (0.1 s), and 1 % of the commands are lost on the
/// way (`reports/isp-af-r3.md`).
const CONFIRM_TIMEOUT: f64 = 0.4;
/// How many times a move is sent before it is given up: the autopilot drops the commands of
/// 33 to 200 ms every 5.4 s, and of 0.35 s now and then (`reports/isp-af-r5.md`).
const SENDS: usize = 3;
/// Without a frame for this long, the camera's reader stopped, s.
pub(crate) const READER_TIMEOUT: f64 = 0.5;
/// Landing over peak v2 under which a landing does not count (§6).
pub(crate) const LANDED_RATIO: f64 = 0.6;
/// A ramp has crossed a hump once its frames are back under this fraction of its height: the
/// peak's time and the width take the frames over 0.3 of it.
const CROSSED: f64 = 0.25;
/// The spread of a coarse estimate's offset between cameras before a camera's runs measure it,
/// µs (`reports/af-control-design.md` §2.4).
const COARSE_SPREAD: f64 = 14.0;
/// Measuring speeds, fastest first, µs/s: above 100 the hump spans under a frame and the
/// estimate scatters 2 µs (`reports/isp-af-r4.md`).
const SPEEDS: [f64; 4] = [100.0, 75.0, 50.0, 35.0];
/// How far above the run-in's start a turn waits for the shaft, µs.
const TURN_SLACK: f64 = 10.0;
/// The 4K still's sd over the v2 hump's (`reports/isp-af-r2.md`).
pub(crate) const STILL_OVER_V2: f64 = 1.12;
/// The sharpness a landing reports is certified with 90 % confidence: its one-sided z.
const CONFIDENCE: f64 = 1.645;
/// A falling step shorter than this, µs, keeps a third of the last landing's error, scatters
/// `short_sd` and stops `short_stop` above its output (`reports/isp-af-r4.md`: 0.53 and 0.45).
const SHORT: f64 = 10.0;
/// A landing's variance besides the posterior's and its own scatter, µs²: the whole-µs output
/// (0.29) and v2's peak against the still's (0.3).
const OUTPUT_VAR: f64 = 0.29 * 0.29 + 0.3 * 0.3;

/// The centre v2 hump's half width at half its height, µs, until a ramp measures it: about 4 on
/// the bench's (§1.2).
pub(crate) const HALF_WIDTH: f64 = 4.0;
/// A hill-climbing close's first step over the hump's half width: the desk's port's
/// (`reports/af-estimators.md`).
const HILL_STEP: f64 = 0.85;

/// What each preset tunes: robust optimisation over the model calibrated on round 4, one set per
/// preset (`reports/af-optimization.md` §4.4, fork F11). Round 4's values, the measured ones,
/// are in the comments.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Tuning {
    /// The close stops once the landing certifies the sharpness asked plus `stop_dq`, and says
    /// `focused` only once it certifies it plus `claim_dq` (round 4: 0 and 0, one certificate).
    pub stop_dq: f64,
    pub claim_dq: f64,
    /// "Certified" is within the limit at this one-sided z (1.645, 90 %).
    pub confidence: f64,
    /// sd of a coarse estimate within a camera, µs (6.5; bench 3.6 to 6.6).
    pub coarse_sd: f64,
    /// sd of an estimate from a hump crossed at full speed, µs (7).
    pub fast_sd: f64,
    /// The run-in speed of a ramp down to where it measures, µs/s (200).
    pub fast: f64,
    /// The fastest measuring speed, µs/s (100: above it the hump spans under a frame and the
    /// estimate scatters 2 µs).
    pub top_speed: f64,
    /// A ramp may run as fast as its estimate's sd is this fraction of the hump's (0.6), and its
    /// slow part starts this many posterior sd above the peak (2.5).
    pub capture: f64,
    pub guard: f64,
    /// About how long a seat above the peak takes, a landing (the reseat past the play and the
    /// reading after it), and a reading after a short step, s (0.6, 1.8, 0.9).
    pub seat: f64,
    pub land: f64,
    pub stop: f64,
    /// How far v2's peak lies above the 4K still's, µs (0.35: 0.0 to 0.7 over 54 runs, round 2):
    /// the landing aims at the still's.
    pub v2_over_still: f64,
    /// A reading certifies against the ramp's height this much over it (0.08).
    pub certify_margin: f64,
    /// The close's step within these, µs (2 to 6: a short falling step scatters by 0.53 µs).
    pub min_step: f64,
    pub max_step: f64,
    /// Where a short falling step stops above its output, and its sd, µs (0.45, 0.53).
    pub short_stop: f64,
    pub short_sd: f64,
    /// A falling landing's sd, µs (0.32 measured; larger, it also covers the zero's walk).
    pub land_sd: f64,
    /// A stop is read once its lens is this close to it, µs, over this many frames (0.2, 6); a
    /// move has arrived this close to its output, µs (5: a reseat overshoots by the play plus the
    /// margin, against the 20 µs the play takes).
    pub settled: f64,
    pub read: u8,
    pub reached: f64,
    /// The height a static pair places may be this many times the passes' (1.3).
    pub pair_cap: f64,
    /// A ramp's lag scatter, s (0.0065, round 4's bench).
    pub lag_sd: f64,
    /// Reseat overshoot past the play, µs (10: the play takes ≥ 20 µs to take up).
    pub margin: f64,
}

/// Fast, Balanced and Precise, at the sharpness each asks.
const PRESETS: [(f32, Tuning); 3] = [
    (
        0.5,
        Tuning {
            stop_dq: -0.109,
            claim_dq: 0.067,
            confidence: 2.09,
            coarse_sd: 4.1,
            fast_sd: 8.98,
            fast: 248.0,
            top_speed: 75.0,
            capture: 0.603,
            guard: 1.97,
            seat: 0.459,
            land: 1.46,
            stop: 1.17,
            v2_over_still: 0.314,
            certify_margin: 0.145,
            min_step: 2.41,
            max_step: 6.3,
            short_stop: 0.544,
            short_sd: 0.601,
            land_sd: 0.575,
            settled: 0.618,
            read: 2,
            reached: 8.75,
            pair_cap: 1.29,
            lag_sd: 0.00536,
            margin: 8.59,
        },
    ),
    (
        0.7,
        Tuning {
            stop_dq: -0.107,
            claim_dq: 0.0653,
            confidence: 1.63,
            coarse_sd: 6.37,
            fast_sd: 9.99,
            fast: 222.0,
            top_speed: 50.0,
            capture: 0.429,
            guard: 2.01,
            seat: 0.916,
            land: 1.6,
            stop: 0.549,
            v2_over_still: 0.423,
            certify_margin: 0.0213,
            min_step: 2.21,
            max_step: 7.52,
            short_stop: 0.425,
            short_sd: 0.619,
            land_sd: 0.542,
            settled: 0.454,
            read: 3,
            reached: 8.63,
            pair_cap: 1.64,
            lag_sd: 0.00983,
            margin: 11.5,
        },
    ),
    (
        0.9,
        Tuning {
            stop_dq: -0.111,
            claim_dq: 0.0688,
            confidence: 1.38,
            coarse_sd: 8.96,
            fast_sd: 9.03,
            fast: 244.0,
            top_speed: 75.0,
            capture: 0.302,
            guard: 2.11,
            seat: 0.606,
            land: 2.49,
            stop: 0.936,
            v2_over_still: 0.0457,
            certify_margin: 0.123,
            min_step: 2.41,
            max_step: 6.31,
            short_stop: 0.466,
            short_sd: 0.555,
            land_sd: 0.565,
            settled: 0.714,
            read: 4,
            reached: 8.75,
            pair_cap: 1.3,
            lag_sd: 0.00832,
            margin: 9.11,
        },
    ),
];

/// The tuning of the preset nearest the sharpness asked.
pub(crate) fn tuning(sharpness: f32) -> Tuning {
    PRESETS
        .iter()
        .min_by(|a, b| (a.0 - sharpness).abs().total_cmp(&(b.0 - sharpness).abs()))
        .map_or(PRESETS[1].1, |preset| preset.1)
}

/// One frame of the statistic at the centre of the picture.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Frame {
    /// Frame time, s, on the environment's clock.
    pub t: f64,
    pub v2: f64,
    pub v1: f64,
}

/// What the procedure drives and reads.
pub(crate) trait Env {
    /// Lowest and highest focus output of the window, µs.
    fn window(&self) -> (f64, f64);
    /// Seconds.
    fn now(&self) -> f64;
    /// The focus output last commanded (µs), or the one at the start.
    fn last_sent(&self) -> f64;
    /// Commands a focus output (µs), without waiting: the nearest one the camera takes.
    fn send(&mut self, u: f64);
    /// Lets time pass, or fails when the run has to stop.
    async fn wait(&mut self, seconds: f64) -> Result<(), FocusNowOutcome>;
    /// The frames with `t0 <= t < t1`, oldest first.
    fn frames(&self, t0: f64, t1: f64) -> Vec<Frame>;
    /// The reported focus outputs `(time reported, µs)` with `t0 <= t < t1`, oldest first.
    fn outputs(&self, t0: f64, t1: f64) -> Vec<(f64, f64)>;
    /// Called as the procedure enters each phase.
    fn phase(&mut self, phase: FocusNowPhase);
    /// What the run measured of the peak: a reading at rest at an output (`Some(v2)`), or where
    /// the passes place it (`None`); a failed run falls back on the best of them.
    fn note(&mut self, u: f64, v2: Option<f64>) {
        let _ = (u, v2);
    }
    /// Reads the frames of the wider retry window from now on, if there is one (whether so).
    fn widen(&mut self) -> bool {
        false
    }
}

/// Where a run left the focus.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Landing {
    /// The output landed on, µs.
    pub u: f64,
    /// v2 at the landing over the hump's peak v2.
    pub ratio: f64,
    /// From the start, s.
    pub seconds: f64,
    /// Whether the coarse pass was falling.
    pub falling: bool,
    /// The coarse bias that would have put that pass on the peak the run placed, µs.
    pub coarse_bias: f64,
    /// The ramp bias that would have put the ramp's estimate on the peak the run placed, µs
    /// (NaN without a ramp).
    pub ramp_bias: f64,
    /// Whether the landing is within the limit's sharpness, at 90 % confidence.
    pub certified: bool,
    /// The sharpness the landing certifies, as a fraction of the best.
    pub sharpness: f64,
    /// The hump's sd, µs, its height above the floor, and the floor: what a refocus starts from.
    pub w: f64,
    pub height: f64,
    pub floor: Floor,
    /// The hump's half width at half its height, µs, and the close's last hill-climbing step.
    pub half_width: f64,
    pub step: f64,
}

/// What a refocus starts from: a posterior on the peak's output, and what the last run measured
/// of the hump and its floor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Seed {
    pub mu: f64,
    pub var: f64,
    pub w: f64,
    pub height: f64,
    pub floor: Floor,
    pub half_width: f64,
    pub step: f64,
}

/// Finds the sharpest focus at the centre, starting from [`Env::last_sent`].
pub(crate) async fn run<E: Env>(
    env: &mut E,
    k: &Constants,
    limits: &FocusNowLimits,
) -> Result<Landing, FocusNowOutcome> {
    search(env, k, limits, None).await
}

/// Focuses again where the lens is, from `seed`: the fine ramp over its posterior and the
/// landing, without the coarse pass (`reports/af-control-design.md` §5.2).
pub(crate) async fn refocus<E: Env>(
    env: &mut E,
    k: &Constants,
    limits: &FocusNowLimits,
    seed: Seed,
) -> Result<Landing, FocusNowOutcome> {
    search(env, k, limits, Some(seed)).await
}

/// What the coarse pass found.
struct Coarse {
    falling: bool,
    /// Its speed, µs/s, and when its output changed and the lens set off, s.
    v: f64,
    tc: f64,
    t_on: f64,
    on_start: bool,
    floor: Floor,
    /// The spike's height above the floor.
    height: f64,
    x_raw: f64,
    x_c: f64,
}

async fn coarse<E: Env>(env: &mut E, k: &Constants) -> Result<Coarse, FocusNowOutcome> {
    let (lo, hi) = env.window();
    let u0 = env.last_sent();

    // 1. Coarse: one jump to the far end of the window, watching the v2 spike.
    env.phase(FocusNowPhase::Coarse);
    let falling = (u0 - lo) >= (hi - u0);
    let end = if falling { lo } else { hi };
    let v = if falling { k.v_f } else { k.v_r };
    // A second of frames at rest first, for its floor's noise: a slow flicker of the scene in
    // them (a blinking LED's 1 Hz square wave in the real dark, round 10) and not taken for a
    // hump. A reader just started has only a fraction of it.
    while env
        .frames(f64::NEG_INFINITY, f64::INFINITY)
        .first()
        .is_none_or(|frame| frame.t > env.now() - FLOOR_REST)
    {
        wait(env, FRAME).await?;
    }
    let t0 = env.now();
    env.send(end);
    // A late jump shifts the whole timeline: anchored on its report.
    let tc = confirm(env, t0).await?;
    // Onset from the model (ACK, mean Lua wait, dead time): the statistic is flat far from
    // focus, so it cannot anchor the timing; the Lua tick's ±50 ms (±13 µs at full speed)
    // is within the fine span. A rising move takes the play up first.
    let t_on = tc + k.t_cmd + k.dead + k.frame;
    // The pass ends at the spike's hump, as soon as it is seen whole: the rest of the
    // window is travel there and back for nothing. A hump needs v2 to fall on its far side,
    // so a lens resting at the end of the window, sharper unblurred than the spike of a
    // weak peak crossed at full speed, is not one (`reports/isp-af-r2.md`).
    let until = t_on + (end - u0).abs() / v + REST;
    let (frames, floor, spike, on_start) = loop {
        let frames = env.frames(t0, env.now());
        // The frames from before the lens moves are no spike, and the floor only once
        // the frames since tell whether the lens rested on the peak.
        let moving = frames.iter().filter(|frame| frame.t >= t_on).count();
        // With the frames at rest before it: more of them for the noise.
        let rest = env.frames(t0 - FLOOR_REST, t_on);
        let start = on_start(&frames, &rest, t_on, k);
        let floor = start.unwrap_or(Floor::of(&env.frames(t0 - FLOOR_REST, env.now()), &rest));
        let spike = (start.is_none() && moving >= FLOOR_FRAMES)
            .then(|| hump(&frames, floor, floor.least(k), HUMP_SIDE))
            .flatten();
        if spike.is_some() || start.is_some() || env.now() >= until {
            break (frames, floor, spike, start.is_some());
        }
        wait(env, FRAME).await?;
    };
    let (i, t_pk, x_raw, x_c) = if let Some(i) = spike {
        let t_pk = centroid(&frames, i, 0.5, 0.15);
        let s_pk = if falling {
            u0 - v * (t_pk - t_on)
        } else {
            u0 + v * (t_pk - t_on)
        };
        let x_raw = if falling {
            s_pk - k.c_f
        } else {
            s_pk - k.c_f - k.h
        };
        // Within the lens's reach on this pass: near its end the shaft slows, and the spike
        // comes later than at full speed.
        let x_c = if falling {
            (x_raw + k.coarse_bias_f).clamp(end - k.c_f, u0 - k.c_f)
        } else {
            (x_raw + k.coarse_bias_r).clamp(u0 - k.c_f - k.h, end - k.c_f - k.h)
        };
        (i, t_pk, x_raw, x_c)
    } else if on_start {
        // The lens rested on the peak: where a falling move left it, the fine ramp's span
        // covers the play when a rising one did. No coarse bias to learn from it.
        let i = frames.iter().rposition(|frame| frame.t < t_on).unwrap_or(0);
        (i, frames[i].t, f64::NAN, u0 - k.c_f)
    } else {
        info!("Focus now coarse: no spike");
        return Err(FocusNowOutcome::LowTexture);
    };
    info!(
        "Focus now coarse: {}{} x_c {x_c:.1} at {:.3} s, floor {:.1} ± {:.1}",
        if falling { "F" } else { "R" },
        if on_start {
            " on the peak at the start"
        } else {
            ""
        },
        t_pk - t0,
        floor.level,
        floor.noise
    );
    Ok(Coarse {
        falling,
        v,
        tc,
        t_on,
        on_start,
        floor,
        height: frames[i].v2 - floor.level,
        x_raw,
        x_c,
    })
}

async fn search<E: Env>(
    env: &mut E,
    k: &Constants,
    limits: &FocusNowLimits,
    seed: Option<Seed>,
) -> Result<Landing, FocusNowOutcome> {
    let (lo, hi) = env.window();
    let t_start = env.now();
    let u0 = env.last_sent();
    let tn = tuning(limits.sharpness);
    let Coarse {
        falling,
        v,
        tc,
        t_on,
        on_start,
        floor,
        height,
        x_raw,
        x_c,
    } = match seed {
        // From where the lens rests, as if a falling pass had left it there.
        Some(seed) => Coarse {
            falling: true,
            v: k.v_f,
            tc: t_start,
            t_on: f64::INFINITY,
            on_start: false,
            floor: seed.floor,
            height: seed.height,
            x_raw: f64::NAN,
            x_c: seed.mu - k.c_f,
        },
        None => match coarse(env, k).await {
            // Started between the peak and the nearer end, the pass went the other way: once
            // more from the end it reached, over the whole window.
            // Still nothing (or at once, with `retry_first`): on the wider window, when the
            // limits have one (a dark or flat centre, `reports/isp-af-r5.md`, rounds 8 and 9).
            Err(FocusNowOutcome::LowTexture)
                if limits.retry_first && limits.retry_zones > 0 && env.widen() =>
            {
                coarse(env, k).await?
            }
            Err(FocusNowOutcome::LowTexture) => match coarse(env, k).await {
                Err(FocusNowOutcome::LowTexture) if limits.retry_zones > 0 && env.widen() => {
                    coarse(env, k).await?
                }
                coarse => coarse?,
            },
            coarse => coarse?,
        },
    };
    let end = if falling { lo } else { hi };
    if !x_c.is_nan() {
        env.note(x_c + k.c_f, None);
    }

    // 2. Fine: a posterior on the output that, reached falling, puts the lens on the peak (its
    // mean and variance), updated by each pass; it stops once the landing is certified within
    // the user's sharpness, or at a limit (`reports/af-control-design.md` §3, `isp-af-r4.md`).
    let mut passes = u8::from(seed.is_none());
    let elapsed = |env: &E| env.now() - t_start;
    let learned = |n: usize, sd: f64, spread: f64| {
        if n > 0 {
            sd * sd / n as f64
        } else {
            spread * spread
        }
    };
    let (mut mu, mut var) = if on_start {
        // Where a falling move left it, or up to the play below where a rising one did.
        (u0, k.h * k.h)
    } else {
        let n = if falling {
            k.coarse_learned_f
        } else {
            k.coarse_learned_r
        };
        (
            x_c + k.c_f,
            tn.coarse_sd * tn.coarse_sd + learned(n, tn.coarse_sd, COARSE_SPREAD),
        )
    };
    let mut w = k.w;
    let mut half_width = HALF_WIDTH;
    // The spike's height.
    let mut h_ref = height;
    if let Some(seed) = seed {
        (mu, var, w, h_ref) = (seed.mu, seed.var, seed.w, seed.height);
        half_width = seed.half_width;
    }
    // The sharpness the close stops at, and the one `focused` claims.
    let q_stop = (f64::from(limits.sharpness) + tn.stop_dq).clamp(0.3, 0.99);
    let q_claim = (f64::from(limits.sharpness) + tn.claim_dq).clamp(0.3, 0.99);
    let land_var0 = tn.land_sd * tn.land_sd + OUTPUT_VAR;
    // How far off the still keeps the sharpness `q`: along the chord from its top to half its
    // height, 2 (1 − q) of its half width (the still's, `STILL_OVER_V2` times v2's). A hump of
    // exponent 1.5 or more stays above its chord there.
    let tolerance = |half_width: f64, q: f64| 2.0 * STILL_OVER_V2 * half_width * (1.0 - q);
    let need = |half_width: f64| {
        ((tolerance(half_width, q_stop) / tn.confidence).powi(2) - land_var0).max(0.05)
    };
    let ramp_var = |v: f64| {
        let spread = (1.0 + (0.025 * v).powi(2)).sqrt();
        ramp_sd(v, tn.lag_sd).powi(2) + learned(k.ramp_learned, ramp_sd(v, tn.lag_sd), spread)
    };
    let mut ramp_raw = f64::NAN;
    let mut tries = 0;
    let mut from = {
        let travelled = v * (env.now() + k.t_cmd + k.dead + k.frame - t_on).max(0.0);
        if falling {
            (u0 - travelled).max(end)
        } else {
            (u0 + travelled).min(end)
        }
    };
    // Where a rising coarse pass has the shaft when a command sent now acts: it still rises
    // towards the window's end.
    let rising = |env: &E| (u0 + v * (env.now() - tc)).min(end);
    while var > need(half_width) && passes + 1 < limits.max_passes && tries < 2 {
        tries += 1;
        // After a rising coarse pass the shaft is past the hump and the play is taken up: the
        // first ramp turns it there without stopping it (`reports/isp-af-r4.md` bench item 5: the
        // same estimate as a ramp started from rest), once it is past the run-in's start.
        let turn = tries == 1 && !falling && !on_start;
        // The fastest speed whose estimate meets the need, else the fastest that lands within
        // reach of the readings at rest, else the slowest.
        let speeds = || SPEEDS.into_iter().filter(|&v| v <= tn.top_speed);
        let v = speeds()
            .find(|&v| ramp_var(v) <= need(half_width))
            .or_else(|| speeds().find(|&v| ramp_var(v).sqrt() <= tn.capture * w))
            .unwrap_or(SPEEDS[SPEEDS.len() - 1]);
        // It measures from where the lens follows it steadily: above where the peak can be by
        // what the lens lags at v.
        let u_slow = (mu + tn.guard * var.sqrt() + v * (k.dead + 2.0 * k.tau)).min(hi);
        let top = if turn {
            // The slew's spread over the pass (±10 %) leaves the shaft up to 10 µs short.
            (u_slow + k.h + TURN_SLACK).max(rising(env)).min(end)
        } else {
            hi.min(
                u_slow
                    + if u_slow > from - k.c_f {
                        k.h + tn.margin
                    } else {
                        0.0
                    },
            )
        };
        // The seat, the ramp past the peak and the landing after it: without time for them,
        // land on what is known.
        let ramp_time =
            tn.seat + (top - u_slow) / tn.fast + (u_slow - mu + 2.0 * w) / v + k.ramp_lag;
        if elapsed(env) + ramp_time + tn.land > f64::from(limits.max_seconds) {
            break;
        }
        env.phase(FocusNowPhase::Fine);
        let top = if turn {
            while rising(env) < top {
                wait(env, FRAME).await?;
            }
            top
        } else {
            let t = env.now();
            env.send(top);
            settled(env, t, top - from, k, &tn).await?;
            env.last_sent()
        };
        let u_end = lo.max((mu - 3.0 * var.sqrt() - 10.0).min(u_slow - 150.0));
        let Some(pass) = ramp(env, k, &tn, top, u_slow, v, u_end, floor, limits.estimator).await?
        else {
            info!("Focus now fine: no peak");
            return Err(FocusNowOutcome::NoPeak);
        };
        passes += 1;
        from = env.last_sent();
        if pass.fast {
            // Crossed while the ramp still ran fast: a full-speed estimate, and another ramp. A
            // crossing further from the posterior than its guard says the posterior was wrong
            // (a learned coarse bias gone stale): the crossing replaces it.
            (mu, var) = if (pass.u_at - mu).abs() > tn.guard * var.sqrt() {
                (pass.u_at, tn.fast_sd * tn.fast_sd)
            } else {
                update(mu, var, pass.u_at, tn.fast_sd * tn.fast_sd)
            };
            info!("Focus now fine: crossed fast at {:.1}, again", pass.u_at);
            continue;
        }
        if let Some(half) = pass.half_width {
            half_width = half.clamp(2.5, 6.0);
        }
        h_ref = pass.height;
        ramp_raw = pass.u_at;
        (mu, var) = update(mu, var, pass.u_at + k.ramp_bias, ramp_var(v));
        env.note(mu, None);
        info!(
            "Focus now fine: {v} µs/s, peak at {:.1} w {w:.2} v2 {:.1}{}, posterior {mu:.1} ± {:.2}",
            pass.u_at,
            pass.height,
            if pass.late {
                " (late ramp command: reported outputs)"
            } else {
                ""
            },
            var.sqrt()
        );
        break;
    }

    // 3. Land on the posterior, past the play and falling (onto the still's peak, under v2's),
    // read at rest. Certified when that reading alone bounds the error
    // within the limit; else the readings at rest place the peak, then a falling step onto it.
    env.phase(FocusNowPhase::Land);
    let above = |v: f64| (v - floor.level).max(f64::MIN_POSITIVE);
    // A reading over the height `h` of the peak, at the reading's lower bound (its frames' noise
    // over their root count): v2's sharpness against what the run measured, with no shape and no
    // distance. It bounds the still's wherever the still's hump is no narrower than v2's.
    let measured =
        |v: f64, h: f64| (above(v) - CONFIDENCE * floor.noise / f64::from(tn.read).sqrt()) / h;
    // What the readings at rest `seen` bound the peak's height by: the highest of them.
    let resting = |seen: &[(f64, f64)]| {
        seen.iter()
            .map(|&(_, v)| v - floor.level)
            .fold(0.0, f64::max)
    };
    // Only a ramp measures the peak's height (a refocus's, the run it seeds from): the coarse
    // spike, blurred at full speed, is under it and bounds no reading.
    let h_measured = seed.is_some() || !ramp_raw.is_nan();
    // The peak's height that bounds it: the passes' `h` with its margin, or `rest`, the
    // readings at rest's bound ([`resting`]).
    let peak = |h: f64, rest: f64| ((1.0 + tn.certify_margin) * h).max(rest);
    let certified =
        |v: f64, h: f64, q: f64, rest: f64| h_measured && measured(v, peak(h, rest)) >= q;
    let target = (mu - tn.v2_over_still).round().clamp(lo, hi);
    let t = env.now();
    env.send(hi.min(target + k.h + tn.margin));
    settled(env, t, env.last_sent() - from, k, &tn).await?;
    // Each stop as (output, reading, where the lens is): a short falling step stops short.
    let mut stops = vec![(
        target,
        stop(env, target, k.h + tn.margin, k, &tn).await?,
        target,
    )];
    // Every reading at rest, as (where the lens is, reading).
    let mut seen = vec![(target, stops[0].1)];
    passes += 1;
    // A reading is the mean of its frames: its noise is the frames' over their root count. With
    // the frames' own noise a weak hump (6 sd of a frame high) has no valid reading 2 µs off its
    // top (`reports/isp-af-r5.md`, round 7).
    let reading_noise = floor.noise / f64::from(tn.read).sqrt();
    let valid = |v: f64, h: f64| above(v) >= (3.0 * reading_noise).max(0.03 * h);
    // Where the passes placed the peak, and how many probes looked for it off the hump.
    let (mu_pass, mut probes) = (mu, 0u32);
    // What the passes measured the peak's height to be, at most.
    let h_pass = h_ref;
    // The step of a hill-climbing close.
    let mut hill_step = (HILL_STEP * half_width).max(tn.min_step);
    while !certified(stops[stops.len() - 1].1, h_ref, q_stop, resting(&seen))
        && passes < limits.max_passes
        && elapsed(env) + tn.stop < f64::from(limits.max_seconds)
    {
        let (ua, va, xa) = stops[stops.len() - 1];
        let pair = (stops.len() >= 2).then(|| stops[stops.len() - 2]);
        let (next, d, fresh) = match pair {
            // Two readings at one output (a step held at the window's end) place nothing. The
            // search also climbs back from a step off the hump.
            Some((_, vb, xb)) if valid(vb, h_ref) && xb != xa => {
                // No height beyond what it read.
                let x0 = (limits.close == FocusNowClose::Parabola)
                    .then(|| vertex(&seen, half_width))
                    .flatten()
                    .unwrap_or_else(|| hill(&mut hill_step, xa, va, xb, vb));
                let fit = above(va).max(above(vb));
                // The pair's own height cannot certify the pair: what it places wrong, it also
                // puts lower. The higher of it and the passes' bound holds, the pair's no more
                // than 1.3 times the passes' (one far down the flank extrapolates wildly).
                h_ref = h_pass.max(fit.min(tn.pair_cap * h_pass));
                mu = x0;
                let target = x0 - tn.v2_over_still;
                let target = if target < xa {
                    target - tn.short_stop
                } else {
                    target
                }
                .round()
                .clamp(lo, hi);
                if target == ua {
                    break;
                }
                if target > ua {
                    // Below the peak: past the play, and back down onto it.
                    let t = env.now();
                    env.send(hi.min(target + k.h + tn.margin));
                    settled(env, t, env.last_sent() - ua, k, &tn).await?;
                    (target, k.h + tn.margin, true)
                } else {
                    (target, ua - target, true)
                }
            }
            _ if valid(va, h_ref) => {
                let step = hill_step.round().clamp(tn.min_step, tn.max_step);
                ((ua - step).max(lo), step, false)
            }
            // Off the hump, its side unknown: around where the passes placed the peak, on
            // either side in turn and further each time (2 w, −2 w, 4 w, …). A fixed direction
            // walked off a weak hump (`reports/isp-af-r5.md`, round 7).
            _ => {
                probes += 1;
                let side = if probes % 2 == 1 { 1.0 } else { -1.0 };
                let offset = side * f64::from(probes.div_ceil(2)) * 2.0 * w;
                let target = (mu_pass + offset - tn.v2_over_still).round().clamp(lo, hi);
                if target > ua {
                    let t = env.now();
                    env.send(hi.min(target + k.h + tn.margin));
                    settled(env, t, env.last_sent() - ua, k, &tn).await?;
                    (target, k.h + tn.margin, true)
                } else {
                    (target, ua - target, true)
                }
            }
        };
        let reading = stop(env, next, d, k, &tn).await?;
        if fresh {
            // A fresh pair from there, if it still does not certify.
            stops.clear();
        }
        let short = next < ua && ua - next < SHORT;
        let x = next + if short { tn.short_stop } else { 0.0 };
        stops.push((next, reading, x));
        seen.push((x, reading));
        passes += 1;
    }
    let now = env.now();
    let (landed, v_landed, _) = stops[stops.len() - 1];
    let rest = resting(&seen);
    let ok = certified(v_landed, h_ref, q_claim, rest);
    // What it can certify, at the 90 % the result promises, whatever the preset certifies at.
    let sharpness = if !h_measured {
        0.0
    } else {
        measured(v_landed, peak(h_ref, rest)).clamp(0.0, 1.0)
    };
    info!(
        "Focus now land: w {w:.2} passes {passes} stops {}",
        stops
            .iter()
            .map(|(u, v, _)| format!("{u:.0}:{v:.1}"))
            .collect::<Vec<_>>()
            .join(" ")
    );
    // What stands out of the floor no more than its noise does is not a peak landed on.
    if above(v_landed) < k.min_spike.max(SIGNIFICANT * reading_noise) {
        info!("Focus now land: no peak at {landed:.0}");
        return Err(FocusNowOutcome::NoPeak);
    }
    let landing = Landing {
        u: landed,
        ratio: above(v_landed) / h_ref,
        seconds: now - t_start,
        falling,
        // Against where it places the peak in the end, not where it landed: a landing certified
        // without a pair is the posterior itself, and teaches nothing.
        coarse_bias: mu - k.c_f - x_raw,
        ramp_bias: mu - ramp_raw,
        certified: ok,
        sharpness,
        w,
        // Unmeasured, it is nothing to focus again around.
        height: if h_measured { h_ref } else { 0.0 },
        floor,
        half_width,
        step: hill_step,
    };
    info!(
        "Focus now land: u {:.0} ratio {:.2} sharpness >= {:.2}{} in {:.2} s",
        landing.u,
        landing.ratio,
        landing.sharpness,
        if ok { " certified" } else { "" },
        landing.seconds
    );
    Ok(landing)
}

/// Frames a fall-back reads at each place: enough for their noise to tell a side of the play.
pub(crate) const FALL_BACK_READ: u8 = 12;
/// How long a fall-back takes to reach and read a place at most, s: 1.6 to 2.2 live (t9c, the
/// night of 2026-11-10).
const FALL_BACK_STEP: f64 = 2.2;

/// Where a failed run left the lens, with its reading at rest.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum FallBack {
    /// On the peak the run measured best (`u` of [`fall_back`]): sharper than the start, read
    /// just before (the second value).
    Best(f64, f64),
    /// Back on the start: its output reached falling, or falling to one play below it, as the
    /// lens was where the start was reached rising (`reports/isp-af-r5.md`, round 8).
    Start(f64),
}

/// After a failed run, reads at rest, each reached falling past the play: the peak it measured
/// best (`u`, when it has one), the start's output, and one play below it; it leaves the lens on
/// the sharpest of them: the start's output unless another reads sharper by four times the
/// readings' noise (in noise, a reading cannot tell the sides of the play apart). It ends by
/// `until` (the run's time limit): short of time, without the start's other side of the play,
/// and without time for even that, it reads nothing (an error).
pub(crate) async fn fall_back<E: Env>(
    env: &mut E,
    k: &Constants,
    limits: &FocusNowLimits,
    u: Option<f64>,
    start: f64,
    until: f64,
) -> Result<FallBack, FocusNowOutcome> {
    let tn = tuning(limits.sharpness);
    let (lo, hi) = env.window();
    let mut targets: Vec<f64> = u
        .map(|u| u - tn.v2_over_still)
        .into_iter()
        .chain([start, start - k.h])
        .map(|target| target.round().clamp(lo, hi))
        .collect();
    // Each position, and the move back onto the one it keeps.
    let fits =
        |env: &E, targets: &[f64]| env.now() + (targets.len() + 1) as f64 * FALL_BACK_STEP <= until;
    if !fits(env, &targets) {
        targets.pop();
    }
    if !fits(env, &targets) {
        return Err(FocusNowOutcome::Stopped);
    }
    let mut readings = Vec::with_capacity(targets.len());
    for &target in &targets {
        readings.push(read_falling(env, k, &tn, target).await?);
    }
    let first = usize::from(u.is_some());
    let beats = |(a, sa): (f64, f64), (b, sb): (f64, f64)| a - b > 4.0 * sa.hypot(sb);
    let start = if readings.len() > first + 1 && beats(readings[first + 1], readings[first]) {
        first + 1
    } else {
        first
    };
    let i = if u.is_some() && beats(readings[0], readings[start]) {
        0
    } else {
        start
    };
    if i + 1 != targets.len() {
        read_falling(env, k, &tn, targets[i]).await?;
    }
    Ok(if i == 0 && u.is_some() {
        FallBack::Best(readings[0].0, readings[start].0)
    } else {
        FallBack::Start(readings[i].0)
    })
}

/// Reaches `target` falling past the play and reads it at rest over [`FALL_BACK_READ`] frames:
/// their mean and its noise (their sd over √n).
async fn read_falling<E: Env>(
    env: &mut E,
    k: &Constants,
    tn: &Tuning,
    target: f64,
) -> Result<(f64, f64), FocusNowOutcome> {
    let hi = env.window().1;
    let t = env.now();
    let from = env.last_sent();
    env.send(hi.min(target + k.h + tn.margin));
    settled(env, t, env.last_sent() - from, k, tn).await?;
    stop(env, target, k.h + tn.margin, k, tn).await?;
    at_rest(env, FALL_BACK_READ.saturating_sub(tn.read)).await
}

/// After `more` frames, the v2 of the last [`FALL_BACK_READ`] frames: their mean and its noise
/// (their sd over √n).
pub(crate) async fn at_rest<E: Env>(env: &mut E, more: u8) -> Result<(f64, f64), FocusNowOutcome> {
    wait(env, f64::from(more) * FRAME).await?;
    let now = env.now();
    let frames = env.frames(now - f64::from(FALL_BACK_READ) * FRAME, now);
    if frames.len() < 2 {
        return Err(FocusNowOutcome::ReaderMissing);
    }
    let n = frames.len() as f64;
    let mean = frames.iter().map(|frame| frame.v2).sum::<f64>() / n;
    let var = frames
        .iter()
        .map(|frame| (frame.v2 - mean).powi(2))
        .sum::<f64>()
        / (n - 1.0);
    Ok((mean, (var / n).sqrt()))
}

/// A measuring ramp's result.
struct Pass {
    /// The output the ramp had sent `ramp_lag` before the peak's frame, µs.
    u_at: f64,
    /// Half its width at half its height, µs, when it crosses that on both sides.
    half_width: Option<f64>,
    /// Its height above the floor.
    height: f64,
    /// It came while the ramp still ran fast.
    fast: bool,
    /// A ramp command came late: the reported outputs timed it.
    late: bool,
}

/// A falling ramp from `top`, one command a frame: fast down to `u_slow`, then at `v` until its
/// frames have crossed a hump, or down to `u_end`.
#[allow(clippy::too_many_arguments)]
async fn ramp<E: Env>(
    env: &mut E,
    k: &Constants,
    tn: &Tuning,
    top: f64,
    u_slow: f64,
    v: f64,
    u_end: f64,
    floor: Floor,
    estimator: FocusNowEstimator,
) -> Result<Option<Pass>, FocusNowOutcome> {
    let (lo, _) = env.window();
    let tr0 = env.now();
    let crossed = |frames: &[Frame]| {
        hump(frames, floor, floor.least(k), f64::INFINITY).filter(|&i| {
            let last = frames[frames.len() - 1].v2 - floor.level;
            last < CROSSED * (frames[i].v2 - floor.level)
        })
    };
    let mut sched: Vec<(f64, f64)> = Vec::new();
    let mut t_slow = None;
    let mut u = top;
    loop {
        let fast = u > u_slow;
        u = if fast {
            (u - tn.fast * FRAME).max(u_slow)
        } else {
            u - v * FRAME
        }
        .max(lo);
        env.send(u);
        sched.push((env.now(), env.last_sent()));
        if !fast && t_slow.is_none() {
            t_slow = Some(env.now());
        }
        wait(env, FRAME).await?;
        if crossed(&env.frames(tr0, env.now())).is_some() {
            break;
        }
        if u <= u_end {
            wait(env, k.ramp_lag + 0.1).await?;
            break;
        }
    }
    let frames = env.frames(tr0, env.now());
    // None of its commands ever showed on the outputs: they do not reach the lens.
    if !env
        .outputs(tr0, env.now())
        .iter()
        .any(|&(_, p)| sched.iter().any(|&(_, u)| (u - p).abs() < 0.5))
    {
        return Err(FocusNowOutcome::CommandLost);
    }
    let (Some(i), Some(&(_, last))) = (
        hump(&frames, floor, floor.least(k), f64::INFINITY),
        sched.last(),
    ) else {
        return Ok(None);
    };
    // Not a Gaussian peak fit: deferred because of patent US 12,242,169 B2.
    let (t_pk, height) = match estimator {
        FocusNowEstimator::Midpoint => midpoint(&frames, i, floor.level),
        FocusNowEstimator::Parabola => parabola(&frames, i, floor.level),
    };
    let reported = env.outputs(tr0 - 1.0, env.now());
    // A ramp step came late, stalling the ramp, when an output is reported still on a command
    // older than [`LATE`] (the ramp sends 30 commands a second, the outputs are reported 10
    // times): the reported outputs time it then.
    let late = reported.len() > 2
        && reported.iter().any(|&(t, p)| {
            let on_time = sched.partition_point(|&(ts, _)| ts <= t - LATE);
            on_time > 0
                && !sched[on_time - 1..]
                    .iter()
                    .any(|&(_, u)| (u - p).abs() < 0.5)
        });
    let u_at = if late {
        lagged_output(&reported, t_pk - k.seen_lag, k.tau)
    } else {
        interpolate(&sched, t_pk - k.ramp_lag, top, last)
    };
    Ok(Some(Pass {
        u_at,
        half_width: half_height(&frames, i, floor.level)
            .map(|(before, after)| v * (after - before) / 2.0),
        height,
        // The lens follows the slow speed steadily only a dead time and two tau after the switch;
        // a hump before that was crossed in the run-in, and its time says little (`reports/
        // isp-af-r5.md`, round 7: camera .10 at zoom 2.0, 50 to 70 µs off).
        fast: t_slow.is_none_or(|t_slow| t_pk - k.ramp_lag < t_slow + k.dead + 2.0 * k.tau),
        late,
    }))
}

/// The posterior `(mu, var)` after a measurement `y` of variance `r` (a scalar Kalman step).
fn update(mu: f64, var: f64, y: f64, r: f64) -> (f64, f64) {
    let gain = var / (var + r);
    (mu + gain * (y - mu), (1.0 - gain) * var)
}

/// A ramp's own scatter at `v`, µs: 0.3 at rest and 6.5 ms of lag scatter (0.24 to 0.68 µs at 35
/// to 100 µs/s, `reports/isp-af-r4.md`).
fn ramp_sd(v: f64, lag_sd: f64) -> f64 {
    (0.09 + (v * lag_sd).powi(2)).sqrt()
}

/// [`Env::wait`] for `seconds` (none when negative), failing when the frames stopped.
async fn wait<E: Env>(env: &mut E, seconds: f64) -> Result<(), FocusNowOutcome> {
    env.wait(seconds.max(0.0)).await?;
    if env
        .frames(env.now() - READER_TIMEOUT, f64::INFINITY)
        .is_empty()
    {
        return Err(FocusNowOutcome::ReaderMissing);
    }
    Ok(())
}

/// When the last command, sent at `t_send`, changed the output: when it was reported less
/// the report delay, or `t_send` when it was on time. Waits for the report, re-sends it while
/// it does not come, and fails when it never does.
async fn confirm<E: Env>(env: &mut E, t_send: f64) -> Result<f64, FocusNowOutcome> {
    let u = env.last_sent();
    for sent in 1..=SENDS {
        let t_end = env.now() + CONFIRM_TIMEOUT;
        loop {
            // From the output in place at the send on: it may be there already.
            let reported = env.outputs(f64::NEG_INFINITY, f64::INFINITY);
            let from = reported
                .iter()
                .rposition(|&(t, _)| t < t_send - 0.05)
                .unwrap_or(0);
            let seen = reported[from..].iter().find(|&&(_, p)| (p - u).abs() < 0.5);
            if let Some(&(t, _)) = seen {
                return Ok(if t < t_send - 0.05 {
                    t_send
                } else if t - t_send < LATE {
                    t_send.min(t - REPORT_DELAY)
                } else {
                    t - REPORT_DELAY
                });
            }
            if env.now() >= t_end {
                break;
            }
            wait(env, 0.02).await?;
        }
        if sent < SENDS {
            warn!("Focus now: output {u} not reported, sending it again");
            env.send(u);
        }
    }
    Err(FocusNowOutcome::CommandLost)
}

/// Waits for the move sent at `t_from` over `d` µs to come within [`Tuning::reached`] of its
/// output: its confirmation, then the model's travel (dead time, slew, then first order, §3.2).
async fn settled<E: Env>(
    env: &mut E,
    t_from: f64,
    d: f64,
    k: &Constants,
    tn: &Tuning,
) -> Result<(), FocusNowOutcome> {
    let ts = confirm(env, t_from).await?;
    let v = k.v_f.min(k.v_r);
    // The shaft slews until v tau from its output, then closes in first order.
    let first_order = d.abs().min(v * k.tau);
    let until = ts
        + k.t_cmd
        + k.dead
        + (d.abs() - first_order) / v
        + k.tau * (first_order / tn.reached).max(1.0).ln();
    wait(env, until - env.now()).await
}

/// The statistic away from focus: its level and its frame noise (sd).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct Floor {
    level: f64,
    noise: f64,
}

impl Floor {
    /// The median of `frames`' v2, and 1.4826 times the median absolute deviation of the
    /// differences between successive frames, over √2: a spike's few frames move neither. The
    /// differences leave out the slow signal, the scene's pedestal and a hump's flanks, which the
    /// deviation from the median counted as noise (52 to 68 against 20 at rest at 1/2000 s
    /// ISO 1600, live on t9c, the night of 2026-11-10: real humps fell under `least`). At high
    /// gain successive frames' noise correlates (temporal noise reduction: lag-1 ρ 0.75 to 0.8 at
    /// 1/60 s ISO 40 000), and the differences read it low by √(1 − ρ): corrected by the ρ of
    /// the frames at rest `rest`, and at least their own sd.
    fn of(frames: &[Frame], rest: &[Frame]) -> Self {
        let median = |mut values: Vec<f64>| {
            values.sort_by(f64::total_cmp);
            values.get(values.len() / 2).copied().unwrap_or_default()
        };
        let level = median(frames.iter().map(|frame| frame.v2).collect());
        let steps: Vec<f64> = frames
            .windows(2)
            .map(|pair| pair[1].v2 - pair[0].v2)
            .collect();
        let step = median(steps.clone());
        let deviation = 1.4826 * median(steps.iter().map(|d| (d - step).abs()).collect());
        // Their rms within 3 of that sd (a spike's steps out): twice as steady as the deviation.
        let kept: Vec<f64> = steps
            .iter()
            .map(|d| d - step)
            .filter(|d| d.abs() <= 3.0 * deviation)
            .collect();
        let rms = (kept.iter().map(|d| d * d).sum::<f64>() / kept.len().max(1) as f64).sqrt();
        let (sd, rho) = if rest.len() >= FLOOR_FRAMES {
            let n = rest.len() as f64;
            let mean = rest.iter().map(|frame| frame.v2).sum::<f64>() / n;
            let var = rest
                .iter()
                .map(|frame| (frame.v2 - mean).powi(2))
                .sum::<f64>();
            let lag = rest
                .windows(2)
                .map(|pair| (pair[0].v2 - mean) * (pair[1].v2 - mean))
                .sum::<f64>();
            // Their correlation where it stands out of its own scatter, 2 / √n, less the low
            // bias of a short series (Kendall 1954, (1 + 3ρ) / n), at most 0.85 (past it, 1 s of
            // frames tells too little); and their variance, less the low bias of a mean taken out
            // of correlated frames.
            let rho = lag / var;
            let rho = if rho > 2.0 / n.sqrt() {
                (rho + (1.0 + 3.0 * rho) / n).min(0.85)
            } else {
                0.0
            };
            let kept = (1.0 - (1.0 + rho) / ((1.0 - rho) * n)).max(0.25);
            ((var / (n - 1.0) / kept).sqrt(), rho)
        } else {
            (0.0, 0.0)
        };
        Self {
            level,
            noise: (rms / (2.0 * (1.0 - rho)).sqrt()).max(sd),
        }
    }

    /// Its level.
    pub(crate) fn level(&self) -> f64 {
        self.level
    }

    /// The least height above it of a peak.
    fn least(&self, k: &Constants) -> f64 {
        k.min_spike.max(SIGNIFICANT * self.noise)
    }
}

/// The frame of highest v2 at least `least` above the floor that stands on a hump: within
/// `side` s on either side of it, a frame under half its height above the floor.
fn hump(frames: &[Frame], floor: Floor, least: f64, side: f64) -> Option<usize> {
    let height = |i: usize| frames[i].v2 - floor.level;
    let under = |i: usize, before: bool| {
        frames.iter().any(|frame| {
            let dt = if before {
                frames[i].t - frame.t
            } else {
                frame.t - frames[i].t
            };
            0.0 < dt && dt <= side && frame.v2 - floor.level < 0.5 * height(i)
        })
    };
    (0..frames.len())
        .filter(|&i| height(i) >= least && under(i, true) && under(i, false))
        .reduce(|best, i| if height(i) > height(best) { i } else { best })
}

/// The floor of the frames since the lens moved, when the lens rested on a peak at the start:
/// the frames from before it moved stand out of it. Their own level is then no floor, and the
/// pass sees only the falling side of that peak. Its noise also from the frames at rest `rest`:
/// a flicker of the scene in them is no peak to rest on.
fn on_start(frames: &[Frame], rest: &[Frame], t_on: f64, k: &Constants) -> Option<Floor> {
    let (still, moving): (Vec<Frame>, Vec<Frame>) = frames.iter().partition(|frame| frame.t < t_on);
    let floor = Floor::of(&moving, rest);
    (!still.is_empty()
        && moving.len() >= FLOOR_FRAMES
        && Floor::of(&still, &[]).level - floor.level >= floor.least(k))
    .then_some(floor)
}

/// Moves falling by `d` µs onto `u` and reads its v2 at rest: the mean over
/// [`Tuning::read`] frames once the lens is within [`Tuning::settled`] of it.
async fn stop<E: Env>(
    env: &mut E,
    u: f64,
    d: f64,
    k: &Constants,
    tn: &Tuning,
) -> Result<f64, FocusNowOutcome> {
    let t = env.now();
    env.send(u);
    let tc = confirm(env, t).await?;
    // The dead time holds the frames' own latency (§3.2).
    let from =
        tc + k.t_cmd + k.dead + d.abs() / k.v_f + k.tau * (d.abs() / tn.settled).max(1.0).ln();
    let read = f64::from(tn.read) * FRAME;
    wait(env, from + read - env.now()).await?;
    let frames = env.frames(from, from + read);
    if frames.is_empty() {
        return Err(FocusNowOutcome::ReaderMissing);
    }
    let reading = frames.iter().map(|frame| frame.v2).sum::<f64>() / frames.len() as f64;
    env.note(u, Some(reading));
    info!(
        "Focus now stop: {u:.0} from {:.1} µs, output at +{:.3} s, read from +{:.3} s: v2 {reading:.1}",
        d.abs(),
        tc - t,
        from - t
    );
    Ok(reading)
}

/// The next lens position of a hill-climb with a shrinking step, from the last reading `va` at `xa`
/// and the one before, `vb` at `xb`: on by `step` from the higher one away from the lower, the step
/// halved (to 0.5 µs at least) once a step lost. The Fibonacci and golden-section search family of
/// Kiefer (1953, Proc. AMS 4, 502); search on the focus measure for autofocus: Tenenbaum (1970) and
/// Krotkov (1987, IJCV 1, 223). It only compares two readings: no shape, no fit, no log.
fn hill(step: &mut f64, xa: f64, va: f64, xb: f64, vb: f64) -> f64 {
    if va >= vb {
        xa + step.copysign(xa - xb)
    } else {
        *step = (*step / 2.0).max(0.5);
        xb + step.copysign(xb - xa)
    }
}

/// The lens position of the vertex of the parabola through the highest reading at rest of `seen`
/// (lens position, reading) and its neighbours on either side, the latest at each position, within
/// `reach` of them; none without three positions or when the parabola opens upwards. Successive
/// parabolic interpolation, the step inside Brent's search (1973, *Algorithms for Minimization
/// without Derivatives*, ch. 5). Through the readings themselves, not their log.
fn vertex(seen: &[(f64, f64)], reach: f64) -> Option<f64> {
    let mut points: Vec<(f64, f64)> = Vec::new();
    for &(x, v) in seen {
        points.retain(|&(at, _)| at != x);
        points.push((x, v));
    }
    points.sort_by(|a, b| a.0.total_cmp(&b.0));
    let best = (0..points.len()).max_by(|&a, &b| points[a].1.total_cmp(&points[b].1))?;
    if points.len() < 3 {
        return None;
    }
    let k = best.clamp(1, points.len() - 2);
    let ((x1, y1), (x2, y2), (x3, y3)) = (points[k - 1], points[k], points[k + 1]);
    let den = (x1 - x2) * (x1 - x3) * (x2 - x3);
    let a = (x3 * (y2 - y1) + x2 * (y1 - y3) + x1 * (y3 - y2)) / den;
    let b = (x3 * x3 * (y1 - y2) + x2 * x2 * (y3 - y1) + x1 * x1 * (y2 - y3)) / den;
    (a < 0.0).then(|| (-b / (2.0 * a)).clamp(x1 - reach, x3 + reach))
}

/// The peak of the hump at frame `i` above `level`: midway between where it crosses half its
/// height on either side, as (time, the top frame's height). The centre of the full width at half
/// maximum, the bisector of a spectral line at half its depth (Gray 1982, ApJ 255, 200; Dravins
/// 1982, ARA&A 20, 61). No shape is fitted and no log taken: two crossings, linear between frames,
/// and their mean.
fn midpoint(frames: &[Frame], i: usize, level: f64) -> (f64, f64) {
    let t =
        half_height(frames, i, level).map_or(frames[i].t, |(before, after)| (before + after) / 2.0);
    (t, frames[i].v2 - level)
}

/// The peak of the hump at frame `i` above `level`: the vertex of the parabola through it and the
/// frames on either side, as (time, the vertex's height); else the top frame's. Quadratic
/// (Lagrange) interpolation of a sampled peak, as for the peaks of a spectrum in Smith & Serra
/// (1987, PARSHL, ICMC). On v2 itself, not on its log, with no shape assumed beyond the top.
fn parabola(frames: &[Frame], i: usize, level: f64) -> (f64, f64) {
    let height = frames[i].v2 - level;
    if 0 < i && i + 1 < frames.len() {
        let (ya, yc) = (frames[i - 1].v2 - level, frames[i + 1].v2 - level);
        let den = ya - 2.0 * height + yc;
        if den < 0.0 {
            let x0 = (frames[i + 1].t - frames[i - 1].t) / 4.0 * (ya - yc) / den;
            if x0.abs() <= FRAME {
                return (frames[i].t + x0, height - (ya - yc).powi(2) / (8.0 * den));
            }
        }
    }
    (frames[i].t, height)
}

/// When the hump at frame `i` crosses half its height above `level` before and after it, linear
/// between the frames on either side of it, s.
fn half_height(frames: &[Frame], i: usize, level: f64) -> Option<(f64, f64)> {
    let half = level + 0.5 * (frames[i].v2 - level);
    let cross = |a: &Frame, b: &Frame| a.t + (half - a.v2) / (b.v2 - a.v2) * (b.t - a.t);
    let before = (1..=i)
        .rev()
        .find(|&j| frames[j - 1].v2 < half)
        .map(|j| cross(&frames[j - 1], &frames[j]))?;
    let after = (i..frames.len() - 1)
        .find(|&j| frames[j + 1].v2 < half)
        .map(|j| cross(&frames[j], &frames[j + 1]))?;
    Some((before, after))
}

/// The v2-weighted time of the frames of at least `fraction` of the peak `i`'s v2 within
/// `within` s of it.
fn centroid(frames: &[Frame], i: usize, fraction: f64, within: f64) -> f64 {
    let peak = frames[i];
    let (weights, sum) = frames
        .iter()
        .filter(|frame| frame.v2 >= fraction * peak.v2 && (frame.t - peak.t).abs() < within)
        .fold((0.0, 0.0), |(weights, sum), frame| {
            (weights + frame.v2, sum + frame.t * frame.v2)
        });
    sum / weights
}

/// Where the lens was at `t` on the reported output staircase: it follows it first order,
/// so the staircase averaged over the last `3 tau`, weighted by `exp((t' - t) / tau)`.
fn lagged_output(reported: &[(f64, f64)], t: f64, tau: f64) -> f64 {
    const POINTS: usize = 60;
    let (weights, sum) = (0..POINTS)
        .map(|n| t - 3.0 * tau * (1.0 - n as f64 / (POINTS - 1) as f64))
        .fold((0.0, 0.0), |(weights, sum), at| {
            let index = reported
                .partition_point(|&(time, _)| time <= at)
                .saturating_sub(1);
            let weight = ((at - t) / tau).exp();
            (weights + weight, sum + reported[index].1 * weight)
        });
    sum / weights
}

/// The output the schedule `(time, µs)` had at `t`, linear between its points, `before`
/// before it and `after` after it.
fn interpolate(schedule: &[(f64, f64)], t: f64, before: f64, after: f64) -> f64 {
    let index = schedule.partition_point(|&(time, _)| time <= t);
    match (
        index.checked_sub(1).map(|i| schedule[i]),
        schedule.get(index),
    ) {
        (None, _) => before,
        (Some(_), None) => after,
        (Some((t0, u0)), Some(&(t1, u1))) => u0 + (u1 - u0) * (t - t0) / (t1 - t0),
    }
}
