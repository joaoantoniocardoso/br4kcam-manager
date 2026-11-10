//! Receiver of the ISP AF statistics of the cameras' `afsub` readers, kept per camera by the
//! sender's address. The reader runs on the camera and answers subscribers: while a camera's
//! statistics are wanted (see [`want`]), this sends its reader the 4 bytes `AFSK` from its own
//! socket every second, and the reader streams one UDP packet per frame back to that socket,
//! until 3 s after the last one. The packets come back as replies, through Docker's NAT, so
//! nothing is published on the vehicle.
//!
//! A packet is 3088 bytes, little-endian: magic `AFS1`, `u32` sequence, `u64` camera
//! monotonic ns, `u64` frame pts in µs, `u32` statistics key, then the 15 × 17 zones,
//! row-major, each `u16` v1, h1, v2, h2, y, hl_cnt (`reports/af-isp-10.md` §6).

use std::{
    collections::{HashMap, VecDeque},
    net::{IpAddr, Ipv4Addr},
    sync::{LazyLock, Mutex, Once, OnceLock},
    time::{Duration, Instant},
};

use tokio::net::UdpSocket;
use tracing::*;

use super::algorithm::Frame;

/// Where the readers take subscriptions.
pub(crate) const PORT: u16 = 9110;
const SUBSCRIPTION: &[u8] = b"AFSK";
/// How often a subscription is renewed; a reader drops it 3 s after the last one.
const KEEPALIVE: Duration = Duration::from_secs(1);
const MAGIC: u32 = 0x3153_4641;
const HEADER_LEN: usize = 28;
const ZONE_COLUMNS: usize = 17;
const ZONE_FIELDS: usize = 6;
const PACKET_LEN: usize = HEADER_LEN + ZONE_ROWS * ZONE_COLUMNS * ZONE_FIELDS * 2;
const ZONE_ROWS: usize = 15;
/// The side of the centre window, in zones, until a run sets another: 3 × 3 zones, rows 6 to
/// 8, columns 7 to 9 (678 × 432 px of the 4K frame).
const CENTRE: usize = 3;
const V1: usize = 0;
const V2: usize = 2;
/// Frames kept per camera: 20 s at 30 fps.
const KEEP: usize = 600;
/// Frames over which a frame's receive time is matched to its pts: the fastest receive in
/// them is taken as no delay, so the camera's clock may drift against ours.
const ALIGN: usize = 300;

static EPOCH: LazyLock<Instant> = LazyLock::new(Instant::now);

#[derive(Default)]
struct Camera {
    frames: VecDeque<Frame>,
    /// Receive time less pts of each kept frame, s.
    delays: VecDeque<f64>,
    /// Until when its statistics are wanted.
    wanted_until: Option<Instant>,
    /// The side of its centre window, in zones, when not [`CENTRE`].
    centre: Option<usize>,
}

static CAMERAS: LazyLock<Mutex<HashMap<Ipv4Addr, Camera>>> = LazyLock::new(Default::default);
static SOCKET: OnceLock<UdpSocket> = OnceLock::new();

fn cameras() -> std::sync::MutexGuard<'static, HashMap<Ipv4Addr, Camera>> {
    CAMERAS
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// Seconds since this process's epoch, the clock of every time here.
pub(crate) fn seconds(at: Instant) -> f64 {
    at.saturating_duration_since(*EPOCH).as_secs_f64()
}

/// Starts listening, once per process.
pub(crate) fn start() {
    static STARTED: Once = Once::new();
    STARTED.call_once(|| {
        tokio::spawn(listen());
        tokio::spawn(keep_alive());
    });
}

/// Wants the statistics of the camera at `address` for `duration` more: subscribes to its
/// reader at once when they were not wanted, and keeps the subscription meanwhile.
pub(crate) fn want(address: Ipv4Addr, duration: Duration) {
    let now = Instant::now();
    let cold = {
        let mut cameras = cameras();
        let camera = cameras.entry(address).or_default();
        let cold = camera.wanted_until.is_none_or(|until| until < now);
        camera.wanted_until = camera.wanted_until.max(Some(now + duration));
        cold
    };
    if cold {
        subscribe(address);
    }
}

/// Measures the centre of the camera at `address` on its `side` × `side` zones (odd, at most
/// the rows high), from the next frame: the kept frames go when it changes.
pub(crate) fn set_centre(address: Ipv4Addr, side: usize) {
    let mut cameras = cameras();
    let camera = cameras.entry(address).or_default();
    if camera.centre.unwrap_or(CENTRE) != side {
        camera.centre = Some(side);
        camera.frames.clear();
        camera.delays.clear();
    }
}

fn centre(address: Ipv4Addr) -> usize {
    cameras()
        .get(&address)
        .and_then(|camera| camera.centre)
        .unwrap_or(CENTRE)
}

fn subscribe(address: Ipv4Addr) {
    if let Some(socket) = SOCKET.get()
        && let Err(error) = socket.try_send_to(SUBSCRIPTION, (address, PORT).into())
    {
        debug!("Failed subscribing to the AF statistics of {address}: {error}");
    }
}

async fn keep_alive() {
    let mut interval = tokio::time::interval(KEEPALIVE);
    loop {
        interval.tick().await;
        let now = Instant::now();
        let wanted: Vec<Ipv4Addr> = cameras()
            .iter()
            .filter(|(_, camera)| camera.wanted_until.is_some_and(|until| until >= now))
            .map(|(address, _)| *address)
            .collect();
        wanted.into_iter().for_each(subscribe);
    }
}

async fn listen() {
    let socket = loop {
        match UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0)).await {
            Ok(socket) => break SOCKET.get_or_init(|| socket),
            Err(error) => {
                warn!("Failed opening the socket for AF statistics: {error}");
                tokio::time::sleep(Duration::from_secs(5)).await;
            }
        }
    };
    let mut packet = [0u8; PACKET_LEN + 1];
    loop {
        match socket.recv_from(&mut packet).await {
            Ok((len, sender)) => {
                let IpAddr::V4(address) = sender.ip() else {
                    continue;
                };
                if let Some((pts, v2, v1)) = parse(&packet[..len], centre(address)) {
                    receive(address, Instant::now(), pts, v2, v1);
                }
            }
            Err(error) => {
                warn!("Failed receiving AF statistics: {error}");
                tokio::time::sleep(Duration::from_secs(1)).await;
            }
        }
    }
}

/// Frame pts in µs, and the mean v2 and v1 of the centre `side` × `side` zones, of a packet.
fn parse(packet: &[u8], side: usize) -> Option<(u64, f64, f64)> {
    if packet.len() != PACKET_LEN || u32::from_le_bytes(packet[0..4].try_into().ok()?) != MAGIC {
        return None;
    }
    let pts = u64::from_le_bytes(packet[16..24].try_into().ok()?);
    let field = |row: usize, column: usize, field: usize| {
        let at = HEADER_LEN + ((row * ZONE_COLUMNS + column) * ZONE_FIELDS + field) * 2;
        f64::from(u16::from_le_bytes([packet[at], packet[at + 1]]))
    };
    let around = |middle: usize, side: usize| middle - side / 2..middle + side / 2 + 1;
    let rows = around(ZONE_ROWS / 2, side.min(ZONE_ROWS));
    let columns = around(ZONE_COLUMNS / 2, side.min(ZONE_COLUMNS));
    let centre = |index: usize| {
        let zones = rows.len() * columns.len();
        rows.clone()
            .flat_map(|row| columns.clone().map(move |column| (row, column)))
            .map(|(row, column)| field(row, column, index))
            .sum::<f64>()
            / zones as f64
    };
    Some((pts, centre(V2), centre(V1)))
}

fn receive(address: Ipv4Addr, at: Instant, pts: u64, v2: f64, v1: f64) {
    let pts = pts as f64 / 1e6;
    let mut cameras = cameras();
    let camera = cameras.entry(address).or_default();
    camera.delays.push_back(seconds(at) - pts);
    let offset = camera
        .delays
        .iter()
        .rev()
        .take(ALIGN)
        .copied()
        .fold(f64::INFINITY, f64::min);
    camera.frames.push_back(Frame {
        t: pts + offset,
        v2,
        v1,
    });
    if camera.frames.len() > KEEP {
        camera.frames.pop_front();
        camera.delays.pop_front();
    }
}

/// The frames of the camera at `address` with `t0 <= t < t1`, oldest first.
pub(crate) fn frames(address: Ipv4Addr, t0: f64, t1: f64) -> Vec<Frame> {
    cameras()
        .get(&address)
        .map(|camera| {
            camera
                .frames
                .iter()
                .filter(|frame| t0 <= frame.t && frame.t < t1)
                .copied()
                .collect()
        })
        .unwrap_or_default()
}
