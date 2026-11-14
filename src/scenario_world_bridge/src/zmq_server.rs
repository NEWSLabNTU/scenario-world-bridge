use prost::Message;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::time::{Duration, Instant};

use crate::budgets;
use crate::coordinator::Coordinator;
use crate::frame_stats::{FrameStats, Outcome, RequestKind, SUMMARY_WINDOW_FRAMES};
use crate::proto::simulation_api_schema::{
    self as api, simulation_request, simulation_response, SimulationRequest, SimulationResponse,
};
use crate::session::Sessions;
use crate::world_services::WorldCommand;

// No idle watchdog. Until roadmap 015 step 7 the bridge handed CARLA back to async after
// 10 s without a request (300 s with an ego), so the world would not stay frozen if a
// scenario died. That free-running is what put /clock leaps and idle-gap timeouts into the
// ego's Autoware, and a REP socket cannot tell a dead SSv2 from a long pause anyway. CARLA
// now stays synchronous and paused between scenarios, swb its only ticker; a graceful
// shutdown still restores async (Coordinator::shutdown). See
// docs/design/time-and-ticking.md.

pub struct ZmqServer {
    /// `None` once closed after a hung CARLA call ([`ZmqServer::close_flushed`]).
    socket: Option<zmq::Socket>,
    coordinator: Coordinator,
    /// Set when a handler panicked: why. Every request but `Initialize` is refused until the
    /// next `Initialize`, because a handler that died half-way may have left the entity map
    /// and CARLA disagreeing (docs/design/failure-and-frame-budget.md, "Panics").
    poisoned: Option<String>,
    frame_stats: FrameStats,
    /// Requests from the ROS services (`world_services`), applied between SSv2 requests on
    /// this thread, which owns the world server's client.
    world_commands: Option<mpsc::Receiver<WorldCommand>>,
    /// Which SSv2 connection holds the scenario (roadmap 077, G21).
    sessions: Sessions,
}

impl ZmqServer {
    pub fn new(
        ctx: &zmq::Context,
        port: u16,
        coordinator: Coordinator,
        world_commands: Option<mpsc::Receiver<WorldCommand>>,
    ) -> eyre::Result<Self> {
        Self::bind(ctx, &format!("tcp://*:{port}"), coordinator, world_commands)
    }

    /// The server on `endpoint` (the tests bind an inproc one).
    pub fn bind(
        ctx: &zmq::Context,
        endpoint: &str,
        coordinator: Coordinator,
        world_commands: Option<mpsc::Receiver<WorldCommand>>,
    ) -> eyre::Result<Self> {
        // ROUTER, not REP: the same one-request-one-reply exchange with SSv2's REQ socket,
        // and each request names the connection it came from, so a second SSv2 can be told
        // apart and refused (`session`).
        let socket = ctx.socket(zmq::ROUTER)?;
        socket.bind(endpoint)?;
        tracing::info!("ZMQ ROUTER socket bound to {endpoint}");
        Ok(Self {
            socket: Some(socket),
            coordinator,
            poisoned: None,
            frame_stats: FrameStats::new(),
            world_commands,
            sessions: Sessions::default(),
        })
    }

    /// Apply whatever the ROS services have queued. Between SSv2 requests, so never in the
    /// middle of a frame's ticks or a map load.
    fn serve_world_commands(&mut self) {
        // Taken while applied: a load needs `self`.
        let Some(rx) = self.world_commands.take() else {
            return;
        };
        while let Ok(cmd) = rx.try_recv() {
            match cmd {
                WorldCommand::SetWeather(request, reply) => {
                    let _ = reply.send(self.coordinator.set_weather(request));
                }
                WorldCommand::GetWeather(reply) => {
                    let _ = reply.send(self.coordinator.get_weather());
                }
                WorldCommand::LoadMap(path, reply) => {
                    let _ = reply.send(self.load_map(&path));
                }
            }
        }
        self.world_commands = Some(rx);
    }

    /// `~/load_map`: the map's world now, unless a scenario is running on this bridge.
    fn load_map(&mut self, path: &str) -> Result<String, String> {
        let entities = self.coordinator.entity_count();
        if let Some(running) = self.sessions.running(entities, Instant::now()) {
            return Err(format!(
                "not loaded: a scenario is running on this bridge, {running}; a map is \
                 loaded between scenarios"
            ));
        }
        let lanelet2 = crate::world_services::lanelet2_path(path)?;
        let started = Instant::now();
        let loaded = self
            .coordinator
            .load_map(&lanelet2)
            .map_err(|e| e.to_string())?;
        // The world was loaded anew: whatever session there was has no scenario left.
        self.sessions.clear();
        Ok(format!(
            "{loaded} in {:.1} s",
            started.elapsed().as_secs_f64()
        ))
    }

    /// Run the server loop until shutdown is signaled.
    pub fn run(&mut self, shutdown: Arc<AtomicBool>) {
        tracing::info!("ZMQ server ready, waiting for SSv2 requests...");

        let Some(socket) = self.socket.take() else {
            return;
        };
        self.serve(&socket, &shutdown);
        // A world request never returned, or the world server cannot go on (roadmap 032 N5,
        // 034 step 2, 046): the request in flight has
        // its error. Answer whatever SSv2 sends next -- its deactivate's DespawnEntity comes
        // within milliseconds -- with the same error and without a word to the world server, until it
        // has been quiet ~2 s (10 s at most); `main` then ends the process for a fresh bridge.
        if let Some(reason) = self.coordinator.wedged().map(str::to_owned) {
            tracing::error!("A world request never returned, or the world server cannot go on: this bridge drains, then exits");
            let answered = drain(&socket, &reason, DrainPolicy::default());
            tracing::warn!(
                "Drained {answered} request(s) with the hang error; closing the SSv2 socket"
            );
        }
        self.socket = Some(socket);
        tracing::info!("ZMQ server shutting down");
    }

    fn serve(&mut self, socket: &zmq::Socket, shutdown: &AtomicBool) {
        let mut last_request = Instant::now();
        let mut last_probe = Instant::now();
        while !shutdown.load(Ordering::SeqCst) {
            self.serve_world_commands();
            // Poll with 100ms timeout so we can check shutdown
            let mut items = [socket.as_poll_item(zmq::POLLIN)];
            match zmq::poll(&mut items, 100) {
                Ok(0) => {
                    // Between scenarios: is the world server still there? A dead one
                    // ends the bridge (Coordinator::probe_connection_idle).
                    if last_request.elapsed() >= budgets::IDLE_PROBE_AFTER
                        && last_probe.elapsed() >= budgets::IDLE_PROBE_EVERY
                    {
                        self.coordinator.probe_connection_idle();
                        last_probe = Instant::now();
                        if self.coordinator.server_lost().is_some() {
                            break;
                        }
                    }
                    continue; // timeout, no message
                }
                Ok(_) => {} // message ready
                Err(e) => {
                    if e == zmq::Error::EINTR {
                        continue; // interrupted by signal
                    }
                    tracing::error!("zmq::poll error: {e}");
                    break;
                }
            }

            last_request = Instant::now();
            // Receive the request: the connection's envelope, then the message.
            let Some((envelope, msg)) = recv_request(socket) else {
                continue;
            };
            let peer = envelope.first().cloned().unwrap_or_default();

            // One SSv2 at a time (`session`): a request from a connection that does not hold
            // the scenario is answered with why, and never dispatched.
            let variant = peek_request_variant(&msg);
            let initialize = (variant == Some(INITIALIZE_FIELD)).then(|| initialize_map(&msg));
            let admitted = self.sessions.admit(
                &peer,
                initialize.as_deref(),
                self.coordinator.entity_count(),
                last_request,
            );
            let response_bytes = match admitted {
                Ok(()) => {
                    // Decode, dispatch, encode, send -- a panic in a handler becomes a
                    // failure response instead of the end of the process.
                    let mut poisoned = self.poisoned.take();
                    let bytes = guarded_dispatch(&mut poisoned, variant, || self.dispatch(&msg));
                    self.poisoned = poisoned;
                    self.sessions.answered(&peer, Instant::now());
                    bytes
                }
                Err(why) => {
                    tracing::warn!("{why}");
                    encode_error_response(variant, &why)
                }
            };

            send_reply(socket, &envelope, &response_bytes);
            // A world request never returned (roadmap 032 N5): the request has its error; stop
            // serving, and drain (see `run`).
            if self.coordinator.wedged().is_some() {
                break;
            }
        }
    }

    fn dispatch(&mut self, msg: &[u8]) -> Vec<u8> {
        let request = match SimulationRequest::decode(msg) {
            Ok(r) => r,
            Err(e) => {
                tracing::error!(
                    "Failed to decode SimulationRequest ({} bytes): {e}",
                    msg.len()
                );
                return encode_error_response(
                    peek_request_variant(msg),
                    "Failed to decode request",
                );
            }
        };

        let request_inner = match request.request {
            Some(r) => r,
            None => {
                tracing::warn!("Empty SimulationRequest (no oneof set)");
                return encode_error_response(None, "Empty request");
            }
        };

        let kind = match &request_inner {
            simulation_request::Request::Initialize(_) => RequestKind::Initialize,
            simulation_request::Request::UpdateFrame(_) => RequestKind::UpdateFrame,
            simulation_request::Request::UpdateEntityStatus(_) => RequestKind::UpdateEntityStatus,
            _ => RequestKind::Other,
        };
        // Initialize starts a new run: close the previous one first, so its summary is
        // logged and this run's frames count from 1.
        if kind == RequestKind::Initialize {
            self.end_run();
        }
        let frame = match kind {
            RequestKind::Initialize => 0,
            _ => self.frame_stats.current_frame(),
        };

        // One span per handler, named after the request and carrying the SSv2 step it
        // belongs to, so every line the handler logs says which request and frame made it.
        // The subscriber does not log span enter/exit (FmtSpan::NONE): at 20 Hz that would be
        // 100+ lines a second to a disk that can stall the loop (CLAUDE.md).
        macro_rules! handle {
            ($handler:ident, $variant:ident, $req:expr) => {{
                let _span = tracing::info_span!(stringify!($handler), frame).entered();
                simulation_response::Response::$variant(self.coordinator.$handler($req))
            }};
        }

        let started = Instant::now();
        let response = match request_inner {
            simulation_request::Request::Initialize(req) => handle!(initialize, Initialize, req),
            simulation_request::Request::UpdateFrame(req) => {
                handle!(update_frame, UpdateFrame, req)
            }
            simulation_request::Request::UpdateStepTime(req) => {
                handle!(update_step_time, UpdateStepTime, req)
            }
            simulation_request::Request::SpawnVehicleEntity(req) => {
                handle!(spawn_vehicle_entity, SpawnVehicleEntity, req)
            }
            simulation_request::Request::SpawnPedestrianEntity(req) => {
                handle!(spawn_pedestrian_entity, SpawnPedestrianEntity, req)
            }
            simulation_request::Request::SpawnMiscObjectEntity(req) => {
                handle!(spawn_misc_object_entity, SpawnMiscObjectEntity, req)
            }
            simulation_request::Request::DespawnEntity(req) => {
                handle!(despawn_entity, DespawnEntity, req)
            }
            simulation_request::Request::UpdateEntityStatus(req) => {
                handle!(update_entity_status, UpdateEntityStatus, req)
            }
            simulation_request::Request::AttachLidarSensor(req) => {
                handle!(attach_lidar_sensor, AttachLidarSensor, req)
            }
            simulation_request::Request::AttachDetectionSensor(req) => {
                handle!(attach_detection_sensor, AttachDetectionSensor, req)
            }
            simulation_request::Request::AttachOccupancyGridSensor(req) => {
                handle!(attach_occupancy_grid_sensor, AttachOccupancyGridSensor, req)
            }
            simulation_request::Request::AttachImuSensor(req) => {
                handle!(attach_imu_sensor, AttachImuSensor, req)
            }
            simulation_request::Request::AttachPseudoTrafficLightDetector(req) => handle!(
                attach_pseudo_traffic_light_detector,
                AttachPseudoTrafficLightDetector,
                req
            ),
            simulation_request::Request::UpdateTrafficLights(req) => {
                handle!(update_traffic_lights, UpdateTrafficLights, req)
            }
            simulation_request::Request::UpdateEntityGoal(req) => {
                handle!(update_entity_goal, UpdateEntityGoal, req)
            }
            simulation_request::Request::UpdateEnvironment(req) => {
                handle!(update_environment, UpdateEnvironment, req)
            }
        };
        let handler = started.elapsed();

        let tick = self.coordinator.take_tick_time();
        let entities = self.coordinator.entity_count();
        let outcome = self.frame_stats.record(kind, handler, tick, entities);
        report_frame(outcome);

        let sim_response = SimulationResponse {
            response: Some(response),
        };
        sim_response.encode_to_vec()
    }

    /// Log the finished run's frame-budget summary, if it stepped at all, and reset.
    fn end_run(&mut self) {
        if let Some(summary) = self.frame_stats.end_run() {
            tracing::info!("Frame budget, whole run: {summary}");
        }
    }

    /// Why the bridge cannot go on after a hung world request, if one did not return.
    pub fn wedged(&self) -> Option<String> {
        self.coordinator.wedged().map(str::to_owned)
    }

    /// Why the bridge exits after its idle check found the world server gone.
    pub fn server_lost(&self) -> Option<String> {
        self.coordinator.server_lost().map(str::to_owned)
    }

    /// Close the SSv2 socket so the replies already queued on it leave before the process
    /// ends: a linger, then the close. The context's termination (when its last handle goes,
    /// `main`'s first) waits out the linger.
    pub fn close_flushed(&mut self) {
        if let Some(socket) = self.socket.take() {
            if let Err(e) = socket.set_linger(SOCKET_LINGER_MS) {
                tracing::warn!("Could not set the SSv2 socket's linger: {e}");
            }
            drop(socket);
        }
    }

    /// Close what the run recorded after a hung request, without a word to the world server.
    pub fn abandon_after_hang(&mut self) {
        self.coordinator.abandon_after_hang();
    }

    /// Release the world server: it undoes what it did for this bridge (destroys its
    /// entities, restores the simulator).
    pub fn cleanup(&mut self) {
        self.end_run();
        self.coordinator.shutdown();
    }
}

/// How long closing the SSv2 socket may wait for queued replies to leave (ms).
const SOCKET_LINGER_MS: i32 = 1000;

/// When a bridge that is draining after a hung world request stops answering (034 step 2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DrainPolicy {
    /// Quiet this long since the last request (or the start): SSv2 has had its answers.
    pub idle: Duration,
    /// Never longer than this, whatever arrives.
    pub max: Duration,
}

impl Default for DrainPolicy {
    fn default() -> Self {
        Self {
            idle: Duration::from_secs(2),
            max: Duration::from_secs(10),
        }
    }
}

impl DrainPolicy {
    pub fn should_end(&self, since_start: Duration, since_last: Duration) -> bool {
        since_last >= self.idle || since_start >= self.max
    }
}

/// Answer every request on `socket` with `reason` -- each in its own response variant,
/// so SSv2 reads the description -- touching nothing else, until `policy` says stop.
/// Returns how many were answered.
pub fn drain(socket: &zmq::Socket, reason: &str, policy: DrainPolicy) -> usize {
    let description = format!(
        "the bridge is restarting after a world request that never returned; this request was not \
         run: {reason}"
    );
    let started = Instant::now();
    let mut last = started;
    let mut answered = 0;
    loop {
        let now = Instant::now();
        if policy.should_end(now - started, now - last) {
            break;
        }
        let mut items = [socket.as_poll_item(zmq::POLLIN)];
        match zmq::poll(&mut items, 50) {
            Ok(0) => continue,
            Ok(_) => {}
            Err(zmq::Error::EINTR) => continue,
            Err(e) => {
                tracing::error!("Drain: zmq::poll error: {e}");
                break;
            }
        }
        let Some((envelope, msg)) = recv_request(socket) else {
            continue;
        };
        let variant = peek_request_variant(&msg);
        send_reply(
            socket,
            &envelope,
            &encode_error_response(variant, &description),
        );
        answered += 1;
        last = Instant::now();
        tracing::warn!("Drain: answered request variant {variant:?} with the hang error");
    }
    answered
}

/// One request from the ROUTER socket: the envelope (the connection's routing id, then the
/// REQ socket's empty delimiter) and the message. `None` for a recv error or a message
/// without an envelope (not from a REQ socket), which has no one to answer.
fn recv_request(socket: &zmq::Socket) -> Option<(Vec<Vec<u8>>, Vec<u8>)> {
    let mut frames = match socket.recv_multipart(0) {
        Ok(f) => f,
        Err(e) => {
            tracing::error!("recv error: {e}");
            return None;
        }
    };
    if frames.len() < 2 {
        tracing::warn!(
            "Dropped a message without an envelope ({} frame(s))",
            frames.len()
        );
        return None;
    }
    let msg = frames.pop()?;
    Some((frames, msg))
}

/// Answer the connection `envelope` names.
fn send_reply(socket: &zmq::Socket, envelope: &[Vec<u8>], reply: &[u8]) {
    let parts = envelope
        .iter()
        .map(Vec::as_slice)
        .chain(std::iter::once(reply));
    if let Err(e) = socket.send_multipart(parts, 0) {
        tracing::error!("send error: {e}");
    }
}

/// The map an `Initialize` names (its directory's name, as the logs know maps), for the
/// session; empty when the request does not decode.
fn initialize_map(msg: &[u8]) -> String {
    match SimulationRequest::decode(msg).ok().and_then(|r| r.request) {
        Some(simulation_request::Request::Initialize(i)) => {
            let path = std::path::Path::new(&i.lanelet2_map_path);
            let dir = if path.extension().is_some() {
                path.parent()
            } else {
                Some(path)
            };
            dir.and_then(|d| d.file_name())
                .map(|n| n.to_string_lossy().into_owned())
                .unwrap_or(i.lanelet2_map_path)
        }
        _ => String::new(),
    }
}

/// Log what one request produced for the frame budget: the step at DEBUG, warnings, and a
/// window summary when one closed.
fn report_frame(outcome: Outcome) {
    if let Some(s) = outcome.sample {
        tracing::debug!(
            frame = s.frame,
            processing_us = s.processing.as_micros() as u64,
            tick_us = s.tick.as_micros() as u64,
            update_entity_status_us = s.update_entity_status.as_micros() as u64,
            entities = s.entities,
            "frame timing"
        );
    }
    for warning in outcome.warnings {
        tracing::warn!("{warning}");
    }
    if let Some(summary) = outcome.window_summary {
        tracing::info!("Frame budget, last {SUMMARY_WINDOW_FRAMES} frames: {summary}");
    }
}

/// Recover the oneof field number from an encoded `SimulationRequest`.
///
/// Only used when full decoding failed. Protobuf encodes each field as a varint tag of
/// `(field_number << 3) | wire_type`, and `SimulationRequest` is a bare `oneof`, so the
/// first tag identifies which request was intended even when the payload after it is
/// malformed.
///
/// Returns `None` if the message is empty or the leading varint is itself unreadable.
/// `SimulationRequest`'s field number for `Initialize`, the one request a poisoned session
/// still serves.
const INITIALIZE_FIELD: u32 = 1;

/// Run `dispatch` so that a panic in it answers this request with a failure and poisons the
/// session instead of unwinding out of `main`.
///
/// While `poisoned` is set, every request but `Initialize` is refused without dispatching.
/// An `Initialize` that returns (whatever its result) resets the session and clears it.
fn guarded_dispatch(
    poisoned: &mut Option<String>,
    variant: Option<u32>,
    dispatch: impl FnOnce() -> Vec<u8>,
) -> Vec<u8> {
    let is_initialize = variant == Some(INITIALIZE_FIELD);
    if let (Some(reason), false) = (poisoned.as_deref(), is_initialize) {
        return encode_error_response(
            variant,
            &format!(
                "swb refused the request: an internal error ({reason}) ended this session; \
                 the next Initialize starts a new one"
            ),
        );
    }

    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(dispatch)) {
        Ok(bytes) => {
            if is_initialize {
                *poisoned = None;
            }
            bytes
        }
        Err(payload) => {
            let message = panic_message(payload.as_ref());
            tracing::error!(
                "Handler panicked on request variant {variant:?}: {message}; refusing \
                 everything but Initialize until the next one"
            );
            *poisoned = Some(message.clone());
            encode_error_response(variant, &format!("swb internal error: {message}"))
        }
    }
}

fn panic_message(payload: &(dyn std::any::Any + Send)) -> String {
    payload
        .downcast_ref::<&str>()
        .map(|s| (*s).to_string())
        .or_else(|| payload.downcast_ref::<String>().cloned())
        .unwrap_or_else(|| "panic with a non-string payload".to_string())
}

fn peek_request_variant(msg: &[u8]) -> Option<u32> {
    // Decode a base-128 varint. Tags are small, so cap the read: field numbers here are
    // all <= 15, giving a single-byte tag, but tolerate multi-byte for robustness.
    let mut value: u64 = 0;
    for (i, &byte) in msg.iter().take(5).enumerate() {
        value |= u64::from(byte & 0x7f) << (7 * i);
        if byte & 0x80 == 0 {
            let field_number = (value >> 3) as u32;
            return (field_number != 0).then_some(field_number);
        }
    }
    None
}

/// Build a failure response, matching the request's oneof variant when it is known.
///
/// The variant matters: SSv2 calls `client.call(request).update_frame()`, and protobuf
/// returns a default-constructed message when the response holds a different variant. The
/// caller still sees `success == false`, so the failure is not lost -- but `description` is,
/// leaving the operator with a generic failure and no reason. Matching the variant keeps the
/// description attached.
///
/// `variant` is `None` when the intended request genuinely cannot be known -- an empty
/// oneof, or a message too corrupt to yield a leading tag. The response then falls back to
/// `Initialize`, which is wrong for any other request but still reads as a failure.
fn encode_error_response(variant: Option<u32>, description: &str) -> Vec<u8> {
    let result = api::Result {
        success: false,
        description: description.to_string(),
    };

    // Field numbers are from proto/simulation_api_schema.proto. SimulationRequest and
    // SimulationResponse assign the same number to each corresponding variant.
    let response = match variant {
        Some(2) => simulation_response::Response::UpdateFrame(api::UpdateFrameResponse {
            result: Some(result),
            simulation_time_ns: 0,
        }),
        Some(3) => {
            simulation_response::Response::SpawnVehicleEntity(api::SpawnVehicleEntityResponse {
                result: Some(result),
            })
        }
        Some(4) => simulation_response::Response::SpawnPedestrianEntity(
            api::SpawnPedestrianEntityResponse {
                result: Some(result),
            },
        ),
        Some(5) => simulation_response::Response::SpawnMiscObjectEntity(
            api::SpawnMiscObjectEntityResponse {
                result: Some(result),
            },
        ),
        Some(6) => simulation_response::Response::DespawnEntity(api::DespawnEntityResponse {
            result: Some(result),
        }),
        Some(7) => {
            simulation_response::Response::UpdateEntityStatus(api::UpdateEntityStatusResponse {
                result: Some(result),
                status: Vec::new(),
            })
        }
        Some(8) => {
            simulation_response::Response::AttachLidarSensor(api::AttachLidarSensorResponse {
                result: Some(result),
            })
        }
        Some(9) => simulation_response::Response::AttachDetectionSensor(
            api::AttachDetectionSensorResponse {
                result: Some(result),
            },
        ),
        Some(10) => simulation_response::Response::AttachOccupancyGridSensor(
            api::AttachOccupancyGridSensorResponse {
                result: Some(result),
            },
        ),
        Some(11) => {
            simulation_response::Response::UpdateTrafficLights(api::UpdateTrafficLightsResponse {
                result: Some(result),
            })
        }
        Some(13) => simulation_response::Response::AttachPseudoTrafficLightDetector(
            api::AttachPseudoTrafficLightDetectorResponse {
                result: Some(result),
            },
        ),
        Some(14) => simulation_response::Response::UpdateStepTime(api::UpdateStepTimeResponse {
            result: Some(result),
        }),
        Some(15) => simulation_response::Response::AttachImuSensor(api::AttachImuSensorResponse {
            result: Some(result),
        }),
        Some(16) => {
            simulation_response::Response::UpdateEntityGoal(api::UpdateEntityGoalResponse {
                result: Some(result),
            })
        }
        Some(17) => {
            simulation_response::Response::UpdateEnvironment(api::UpdateEnvironmentResponse {
                result: Some(result),
            })
        }
        // Field 1 is Initialize, and it is also the documented fallback for None and for
        // any field number this build does not know.
        _ => simulation_response::Response::Initialize(api::InitializeResponse {
            result: Some(result),
            simulation_time_ns: 0,
        }),
    };

    SimulationResponse {
        response: Some(response),
    }
    .encode_to_vec()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_drain_ends_when_idle_or_at_its_limit() {
        let p = DrainPolicy::default();
        assert_eq!(p.idle, Duration::from_secs(2));
        assert_eq!(p.max, Duration::from_secs(10));
        let s = Duration::from_secs_f64;
        // Requests keep arriving: answer them, up to the limit.
        assert!(!p.should_end(s(1.0), s(0.5)));
        assert!(!p.should_end(s(9.9), s(1.9)));
        // Nobody has asked for ~2 s: SSv2's deactivate despawn has its answer.
        assert!(p.should_end(s(2.0), s(2.0)));
        // A client that never stops asking cannot keep a wedged bridge alive.
        assert!(p.should_end(s(10.0), s(0.1)));
    }

    #[test]
    fn a_drain_answers_every_request_with_the_hang_and_its_own_variant() {
        let ctx = zmq::Context::new();
        let rep = ctx.socket(zmq::ROUTER).unwrap();
        rep.bind("inproc://drain-test").unwrap();
        let client = {
            let ctx = ctx.clone();
            std::thread::spawn(move || {
                let req = ctx.socket(zmq::REQ).unwrap();
                req.connect("inproc://drain-test").unwrap();
                let mut answers = Vec::new();
                for request in [
                    simulation_request::Request::DespawnEntity(api::DespawnEntityRequest {
                        name: "ego".into(),
                    }),
                    simulation_request::Request::UpdateFrame(api::UpdateFrameRequest::default()),
                ] {
                    let bytes = SimulationRequest {
                        request: Some(request),
                    }
                    .encode_to_vec();
                    req.send(&bytes, 0).unwrap();
                    let reply = req.recv_bytes(0).unwrap();
                    answers.push(SimulationResponse::decode(&reply[..]).unwrap());
                }
                answers
            })
        };
        let policy = DrainPolicy {
            idle: Duration::from_millis(300),
            max: Duration::from_secs(5),
        };
        let started = Instant::now();
        let answered = drain(&rep, "CARLA's world.tick() hung", policy);
        assert_eq!(answered, 2);
        assert!(started.elapsed() < Duration::from_secs(5), "ended on idle");
        let answers = client.join().unwrap();
        match &answers[0].response {
            Some(simulation_response::Response::DespawnEntity(r)) => {
                let r = r.result.as_ref().unwrap();
                assert!(!r.success);
                assert!(r.description.contains("world.tick() hung"));
            }
            other => panic!("expected a DespawnEntity failure, got {other:?}"),
        }
        assert!(matches!(
            &answers[1].response,
            Some(simulation_response::Response::UpdateFrame(r))
                if !r.result.as_ref().unwrap().success
        ));
    }

    /// Encode a real request, then confirm the tag peek recovers its variant. Guards the
    /// field numbers against a proto change.
    fn variant_of(request: simulation_request::Request) -> Option<u32> {
        let bytes = SimulationRequest {
            request: Some(request),
        }
        .encode_to_vec();
        peek_request_variant(&bytes)
    }

    #[test]
    fn peek_recovers_the_variant_of_a_real_request() {
        assert_eq!(
            variant_of(simulation_request::Request::Initialize(
                api::InitializeRequest::default()
            )),
            Some(1)
        );
        assert_eq!(
            variant_of(simulation_request::Request::UpdateFrame(
                api::UpdateFrameRequest::default()
            )),
            Some(2)
        );
        assert_eq!(
            variant_of(simulation_request::Request::UpdateTrafficLights(
                api::UpdateTrafficLightsRequest::default()
            )),
            Some(11)
        );
        assert_eq!(
            variant_of(simulation_request::Request::AttachImuSensor(
                api::AttachImuSensorRequest::default()
            )),
            Some(15)
        );
        assert_eq!(
            variant_of(simulation_request::Request::UpdateEntityGoal(
                api::UpdateEntityGoalRequest::default()
            )),
            Some(16)
        );
        assert_eq!(
            variant_of(simulation_request::Request::UpdateEnvironment(
                api::UpdateEnvironmentRequest::default()
            )),
            Some(17)
        );
    }

    #[test]
    fn peek_gives_up_on_unusable_input() {
        assert_eq!(peek_request_variant(&[]), None);
        // Continuation bit set the whole way: never terminates within the cap.
        assert_eq!(peek_request_variant(&[0x80, 0x80, 0x80, 0x80, 0x80]), None);
        // Field number 0 is not valid protobuf.
        assert_eq!(peek_request_variant(&[0x00]), None);
    }

    /// The point of E1: an error for an UpdateFrame must come back as an UpdateFrame
    /// response, or SSv2's `call(req).update_frame()` default-constructs and the
    /// description is lost.
    #[test]
    fn error_response_matches_the_requested_variant() {
        let bytes = encode_error_response(Some(2), "boom");
        let decoded = SimulationResponse::decode(bytes.as_slice()).unwrap();

        match decoded.response {
            Some(simulation_response::Response::UpdateFrame(r)) => {
                let result = r.result.expect("result present");
                assert!(!result.success);
                assert_eq!(result.description, "boom");
            }
            other => panic!("expected UpdateFrame response, got {other:?}"),
        }
    }

    #[test]
    fn error_response_falls_back_to_initialize_when_variant_is_unknown() {
        let bytes = encode_error_response(None, "unknown");
        let decoded = SimulationResponse::decode(bytes.as_slice()).unwrap();

        match decoded.response {
            Some(simulation_response::Response::Initialize(r)) => {
                assert!(!r.result.expect("result present").success);
            }
            other => panic!("expected Initialize fallback, got {other:?}"),
        }
    }

    /// Every variant must produce a failure carrying the description, whichever one it is.
    #[test]
    fn every_known_variant_round_trips_as_a_failure() {
        for field in [1u32, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15, 16, 17] {
            let bytes = encode_error_response(Some(field), "why");
            let decoded = SimulationResponse::decode(bytes.as_slice())
                .unwrap_or_else(|e| panic!("field {field} failed to decode: {e}"));
            let response = decoded
                .response
                .unwrap_or_else(|| panic!("field {field} produced no response"));

            let result = match response {
                simulation_response::Response::Initialize(r) => r.result,
                simulation_response::Response::UpdateFrame(r) => r.result,
                simulation_response::Response::SpawnVehicleEntity(r) => r.result,
                simulation_response::Response::SpawnPedestrianEntity(r) => r.result,
                simulation_response::Response::SpawnMiscObjectEntity(r) => r.result,
                simulation_response::Response::DespawnEntity(r) => r.result,
                simulation_response::Response::UpdateEntityStatus(r) => r.result,
                simulation_response::Response::AttachLidarSensor(r) => r.result,
                simulation_response::Response::AttachDetectionSensor(r) => r.result,
                simulation_response::Response::AttachOccupancyGridSensor(r) => r.result,
                simulation_response::Response::UpdateTrafficLights(r) => r.result,
                simulation_response::Response::AttachPseudoTrafficLightDetector(r) => r.result,
                simulation_response::Response::UpdateStepTime(r) => r.result,
                simulation_response::Response::AttachImuSensor(r) => r.result,
                simulation_response::Response::UpdateEntityGoal(r) => r.result,
                simulation_response::Response::UpdateEnvironment(r) => r.result,
            };

            let result = result.unwrap_or_else(|| panic!("field {field} produced no result"));
            assert!(!result.success, "field {field} should be a failure");
            assert_eq!(result.description, "why", "field {field} lost description");
        }
    }

    fn failure_description(bytes: &[u8]) -> Option<String> {
        let response = SimulationResponse::decode(bytes).ok()?.response?;
        let result = match response {
            simulation_response::Response::Initialize(r) => r.result,
            simulation_response::Response::UpdateFrame(r) => r.result,
            _ => None,
        }?;
        (!result.success).then_some(result.description)
    }

    fn ok_bytes() -> Vec<u8> {
        b"ok".to_vec()
    }

    #[test]
    fn a_panicking_handler_answers_with_a_failure_and_poisons_the_session() {
        let mut poisoned = None;
        let bytes = guarded_dispatch(&mut poisoned, Some(2), || panic!("entity map broke"));
        let why = failure_description(&bytes).expect("an UpdateFrame failure");
        assert!(
            why.contains("swb internal error: entity map broke"),
            "{why}"
        );
        assert_eq!(poisoned.as_deref(), Some("entity map broke"));
    }

    #[test]
    fn a_poisoned_session_refuses_everything_but_initialize() {
        let mut poisoned = Some("earlier panic".to_string());
        let mut ran = false;
        let bytes = guarded_dispatch(&mut poisoned, Some(2), || {
            ran = true;
            ok_bytes()
        });
        assert!(!ran, "a poisoned session must not dispatch UpdateFrame");
        let why = failure_description(&bytes).expect("an UpdateFrame failure");
        assert!(
            why.contains("earlier panic") && why.contains("Initialize"),
            "{why}"
        );
        assert!(poisoned.is_some());
    }

    #[test]
    fn initialize_clears_the_poison() {
        let mut poisoned = Some("earlier panic".to_string());
        assert_eq!(
            guarded_dispatch(&mut poisoned, Some(1), ok_bytes),
            ok_bytes()
        );
        assert!(poisoned.is_none());
        assert_eq!(
            guarded_dispatch(&mut poisoned, Some(2), ok_bytes),
            ok_bytes()
        );
    }

    #[test]
    fn a_panicking_initialize_keeps_the_session_poisoned() {
        let mut poisoned = Some("earlier panic".to_string());
        let bytes = guarded_dispatch(&mut poisoned, Some(1), || {
            panic!("{}", String::from("again"))
        });
        assert!(failure_description(&bytes).unwrap().contains("again"));
        assert_eq!(poisoned.as_deref(), Some("again"));
    }

    // --- one SSv2 at a time (roadmap 077, G21), and load_map (G9), over the socket ---------

    use crate::backend::fake::{Call, FakeBackend};
    use crate::proto::traffic_simulator_msgs::{self as ts, VehicleParameters};

    fn initialize(map: &str) -> simulation_request::Request {
        simulation_request::Request::Initialize(api::InitializeRequest {
            step_time: 0.05,
            realtime_factor: 1.0,
            lanelet2_map_path: format!("/maps/{map}/lanelet2_map.osm"),
            ..Default::default()
        })
    }

    fn spawn(name: &str) -> simulation_request::Request {
        simulation_request::Request::SpawnVehicleEntity(api::SpawnVehicleEntityRequest {
            parameters: Some(VehicleParameters {
                name: name.into(),
                subtype: Some(ts::EntitySubtype {
                    value: ts::entity_subtype::Enum::Car as i32,
                }),
                ..Default::default()
            }),
            behavior: "simulator_autopilot".into(),
            ..Default::default()
        })
    }

    fn despawn(name: &str) -> simulation_request::Request {
        simulation_request::Request::DespawnEntity(api::DespawnEntityRequest { name: name.into() })
    }

    fn frame() -> simulation_request::Request {
        simulation_request::Request::UpdateFrame(api::UpdateFrameRequest::default())
    }

    /// Whether a reply succeeded, and its description.
    fn outcome(bytes: &[u8]) -> (bool, String) {
        use simulation_response::Response as R;
        let r = match SimulationResponse::decode(bytes).unwrap().response.unwrap() {
            R::Initialize(r) => r.result,
            R::UpdateFrame(r) => r.result,
            R::SpawnVehicleEntity(r) => r.result,
            R::DespawnEntity(r) => r.result,
            other => panic!("unexpected {other:?}"),
        }
        .unwrap();
        (r.success, r.description)
    }

    /// A REQ client, as SSv2's, on its own connection.
    struct Ssv2(zmq::Socket);

    impl Ssv2 {
        fn connect(ctx: &zmq::Context, endpoint: &str) -> Self {
            let s = ctx.socket(zmq::REQ).unwrap();
            s.connect(endpoint).unwrap();
            Self(s)
        }

        fn call(&self, request: simulation_request::Request) -> (bool, String) {
            let bytes = SimulationRequest {
                request: Some(request),
            }
            .encode_to_vec();
            self.0.send(&bytes, 0).unwrap();
            outcome(&self.0.recv_bytes(0).unwrap())
        }
    }

    type Commands = mpsc::Sender<WorldCommand>;

    /// The server on a fake backend, serving on this thread until `clients` returns.
    fn serve_with(
        name: &str,
        clients: impl FnOnce(zmq::Context, String, Commands) + Send + 'static,
    ) -> FakeBackend {
        let fake = FakeBackend::default();
        let ctx = zmq::Context::new();
        let endpoint = format!("inproc://{name}");
        let (tx, rx) = mpsc::channel();
        let mut server = ZmqServer::bind(
            &ctx,
            &endpoint,
            Coordinator::new(Box::new(fake.clone())),
            Some(rx),
        )
        .unwrap();
        let shutdown = Arc::new(AtomicBool::new(false));
        let done = shutdown.clone();
        let client = std::thread::spawn(move || {
            clients(ctx, endpoint, tx);
            done.store(true, Ordering::SeqCst);
        });
        server.run(shutdown);
        client.join().unwrap();
        fake
    }

    #[test]
    fn a_stray_ssv2_is_refused_while_a_scenario_runs() {
        let fake = serve_with("g21-stray", |ctx, endpoint, _| {
            let a = Ssv2::connect(&ctx, &endpoint);
            let b = Ssv2::connect(&ctx, &endpoint);
            assert!(a.call(initialize("Town01")).0);
            assert!(a.call(spawn("ego")).0);
            assert!(a.call(frame()).0);
            // The stray: its Initialize and everything else is refused, saying why.
            let (ok, why) = b.call(initialize("Town01"));
            assert!(!ok && why.contains("another SSv2 is running"), "{why}");
            assert!(why.contains("Town01") && why.contains("1 entity"), "{why}");
            let (ok, why) = b.call(spawn("ego"));
            assert!(!ok && why.contains("did not Initialize"), "{why}");
            assert!(!b.call(frame()).0);
            // The scenario goes on undisturbed, and ends.
            assert!(a.call(frame()).0);
            assert!(a.call(despawn("ego")).0);
        });
        // The backend saw one client's requests only: one Initialize, one spawn, two frames.
        let calls = fake.calls();
        let n = |f: fn(&Call) -> bool| calls.iter().filter(|c| f(c)).count();
        assert_eq!(n(|c| matches!(c, Call::Initialize(_))), 1);
        assert_eq!(n(|c| matches!(c, Call::Spawn(_))), 1);
        assert_eq!(n(|c| matches!(c, Call::UpdateFrame(_))), 2);
    }

    #[test]
    fn back_to_back_scenarios_each_initialize() {
        let fake = serve_with("g21-next", |ctx, endpoint, _| {
            // SSv2 opens a connection per scenario (variant); the next one Initializes as
            // soon as the last despawned everything.
            for map in ["Town01", "Town01", "Town02"] {
                let s = Ssv2::connect(&ctx, &endpoint);
                assert!(s.call(initialize(map)).0, "{map}");
                assert!(s.call(spawn("ego")).0);
                assert!(s.call(spawn("npc")).0);
                assert!(s.call(frame()).0);
                assert!(s.call(despawn("npc")).0);
                assert!(s.call(despawn("ego")).0);
            }
            // A scenario whose first spawn failed (nothing to despawn) does not block the next.
            let s = Ssv2::connect(&ctx, &endpoint);
            assert!(s.call(initialize("Town02")).0);
            let t = Ssv2::connect(&ctx, &endpoint);
            assert!(t.call(initialize("Town02")).0);
        });
        let inits = fake
            .calls()
            .iter()
            .filter(|c| matches!(c, Call::Initialize(_)))
            .count();
        assert_eq!(inits, 5);
    }

    fn ask_load_map(tx: &Commands, path: &str) -> Result<String, String> {
        let (reply, rx) = futures::channel::oneshot::channel();
        tx.send(WorldCommand::LoadMap(path.into(), reply)).unwrap();
        futures::executor::block_on(rx).unwrap()
    }

    #[test]
    fn load_map_waits_for_no_scenario() {
        let dir = std::env::temp_dir().join(format!("swb-zmq-load-map-{}", std::process::id()));
        let map = dir.join("Town02");
        std::fs::create_dir_all(&map).unwrap();
        std::fs::write(map.join("lanelet2_map.osm"), "<osm/>").unwrap();
        let map_path = map.to_string_lossy().into_owned();
        let fake = serve_with("g9-load-map", move |ctx, endpoint, tx| {
            // No scenario yet: loaded.
            let loaded = ask_load_map(&tx, &map_path).unwrap();
            assert!(loaded.contains("Town02"), "{loaded}");
            // A scenario is running: refused, naming it.
            let a = Ssv2::connect(&ctx, &endpoint);
            assert!(a.call(initialize("Town01")).0);
            assert!(a.call(spawn("ego")).0);
            let why = ask_load_map(&tx, &map_path).unwrap_err();
            assert!(
                why.contains("a scenario is running") && why.contains("Town01"),
                "{why}"
            );
            // It ended: loaded again; the next SSv2 Initializes.
            assert!(a.call(despawn("ego")).0);
            assert!(ask_load_map(&tx, &map_path).is_ok());
            let b = Ssv2::connect(&ctx, &endpoint);
            assert!(b.call(initialize("Town02")).0);
            // A path that is not a map is refused without a load.
            assert!(ask_load_map(&tx, "/nonexistent/Town09")
                .unwrap_err()
                .contains("does not exist"));
        });
        let loads: Vec<String> = fake
            .calls()
            .iter()
            .filter_map(|c| match c {
                Call::Initialize(e) => Some(e.lanelet2_map_path.clone()),
                _ => None,
            })
            .collect();
        let osm = map.join("lanelet2_map.osm").to_string_lossy().into_owned();
        assert_eq!(
            loads,
            vec![
                osm.clone(),
                "/maps/Town01/lanelet2_map.osm".into(),
                osm,
                "/maps/Town02/lanelet2_map.osm".into()
            ]
        );
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
