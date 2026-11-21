//! The bridge's ROS node: services that configure the world (roadmaps 018, 051).
//!
//!   /scenario_world_bridge/set_weather  scensim_interfaces/srv/SetWeather
//!   /scenario_world_bridge/get_weather  scensim_interfaces/srv/GetWeather
//!   /scenario_world_bridge/load_map     scensim_interfaces/srv/LoadMap (roadmap 077)
//!
//! and, with `truth_topic:=true`, the truth stream's publishers (roadmap 092 K4,
//! [`crate::truth_topic`]), which the world-owning thread publishes on.
//!
//! The weather is a preset or parameters by name, passed through to the world server, whose
//! names they are (CARLA's: its presets and `WeatherParameters` fields). `load_map` loads a
//! map's world between scenarios, as an `Initialize` would (a stack's keeper calls it on a
//! CARLA map switch before it starts the vehicle stacks again); refused while a scenario runs.
//!
//! The node runs its own executor on its own thread, but never touches CARLA: each request
//! is handed to the ZMQ loop -- the thread that owns the world client and ticks the world --
//! as a [`WorldCommand`], applied there between SSv2 requests, and answered when done. The
//! services are async, so a request waiting out a map load blocks nothing else.
//!
//! The node is named after the executable (`scenario_world_bridge`, as the launch files
//! name it); `--ros-args` remaps and `__node:=` apply as for any ROS node. Parameters are
//! still read by [`crate::ros_args`], before the node exists; the node declares the config
//! file the bridge chose as its read-only `config_file` parameter, so
//! `ros2 param get /scenario_world_bridge config_file` names the file this process loaded.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::sync::Arc;
use std::time::Duration;

use futures::channel::oneshot;
use rclrs::{CreateBasicExecutor, IntoPrimitiveOptions};
use scensim_interfaces::srv::get_weather::{GetWeather_Request, GetWeather_Response};
use scensim_interfaces::srv::load_map::{LoadMap_Request, LoadMap_Response};
use scensim_interfaces::srv::set_weather::{SetWeather_Request, SetWeather_Response};

use scensim_interfaces::msg::{TruthEntities, TruthFrame};

use crate::truth_topic::{self, TruthTopic};
use crate::weather::{self, WeatherRequest};

pub const SET_WEATHER: &str = "~/set_weather";
pub const GET_WEATHER: &str = "~/get_weather";
pub const LOAD_MAP: &str = "~/load_map";

/// The Lanelet2 file a map directory holds, as SSv2 names it.
pub const LANELET2_FILE: &str = "lanelet2_map.osm";

/// A request for the world-owning thread.
pub enum WorldCommand {
    SetWeather(WeatherRequest, oneshot::Sender<WeatherReply>),
    GetWeather(oneshot::Sender<WeatherReply>),
    /// A map directory or Lanelet2 file; what was loaded, or why not.
    LoadMap(String, oneshot::Sender<Result<String, String>>),
}

/// The Lanelet2 file of `path`: a map directory's `lanelet2_map.osm`, or the file itself.
pub fn lanelet2_path(path: &str) -> Result<String, String> {
    let given = std::path::Path::new(path.trim());
    if given.as_os_str().is_empty() {
        return Err("not loaded: no map_path given (a map directory or its lanelet2 file)".into());
    }
    if !given.is_absolute() {
        return Err(format!(
            "not loaded: map_path {} is relative; give an absolute path (the bridge's working \
             directory is not the caller's)",
            given.display()
        ));
    }
    let file = if given.is_dir() {
        given.join(LANELET2_FILE)
    } else {
        given.to_path_buf()
    };
    if !file.is_file() {
        return Err(format!("not loaded: {} does not exist", file.display()));
    }
    Ok(file.to_string_lossy().into_owned())
}

pub use crate::weather::WeatherReply;

/// A reply's weather as the services' fields: the preset, the names, the values.
fn fields(reply: &WeatherReply) -> (String, Vec<String>, Vec<f64>) {
    match &reply.weather {
        Some(w) => (
            w.preset.clone(),
            w.parameters.keys().cloned().collect(),
            w.parameters.values().copied().collect(),
        ),
        None => Default::default(),
    }
}

/// Start the node on its own thread. Commands arrive on the returned receiver; the thread
/// ends when `shutdown` is set. An error means the node could not be created (ROS not
/// sourced, bad `--ros-args`), and nothing was started.
pub fn start(
    shutdown: Arc<AtomicBool>,
    config_file: &std::path::Path,
    truth: bool,
) -> eyre::Result<(mpsc::Receiver<WorldCommand>, Option<TruthTopic>)> {
    let config_file: Arc<str> = config_file.display().to_string().into();
    let (tx, rx) = mpsc::channel();
    let (ready_tx, ready_rx) = mpsc::channel::<Result<Option<TruthTopic>, String>>();
    std::thread::Builder::new()
        .name("ros_node".into())
        .spawn(move || {
            let setup = (|| -> Result<_, rclrs::RclrsError> {
                let ctx = rclrs::Context::new(std::env::args(), rclrs::InitOptions::default())?;
                let executor = ctx.create_basic_executor();
                let node = executor.create_node(crate::ros_args::NODE_NAME)?;
                // The file the bridge resolved (the parameter, or the installed default it
                // fell back to), not the raw override: read-only, so `ros2 param set` cannot
                // suggest a config change the running bridge would not make.
                let config_param = node
                    .declare_parameter::<Arc<str>>("config_file")
                    .default(config_file)
                    .ignore_override()
                    .description("bridge_config.yaml this bridge loaded (set at start only)")
                    .read_only()?;
                let set = {
                    let tx = tx.clone();
                    node.create_async_service::<scensim_interfaces::srv::SetWeather, _>(
                        SET_WEATHER,
                        move |req: SetWeather_Request| set_weather(tx.clone(), req),
                    )?
                };
                let get = {
                    let tx = tx.clone();
                    node.create_async_service::<scensim_interfaces::srv::GetWeather, _>(
                        GET_WEATHER,
                        move |_: GetWeather_Request| get_weather(tx.clone()),
                    )?
                };
                let load = {
                    let tx = tx.clone();
                    node.create_async_service::<scensim_interfaces::srv::LoadMap, _>(
                        LOAD_MAP,
                        move |req: LoadMap_Request| load_map(tx.clone(), req),
                    )?
                };
                // The truth topics (roadmap 092 K4): created here, published from the
                // world-owning thread.
                let truth = if truth {
                    let frames = node.create_publisher::<TruthFrame>(
                        truth_topic::FRAME_TOPIC
                            .keep_last(truth_topic::FRAME_DEPTH)
                            .reliable(),
                    )?;
                    let entities = node.create_publisher::<TruthEntities>(
                        truth_topic::ENTITIES_TOPIC
                            .keep_last(1)
                            .reliable()
                            .transient_local(),
                    )?;
                    Some(TruthTopic::new(frames, entities))
                } else {
                    None
                };
                Ok((executor, (node, config_param), set, get, load, truth))
            })();
            let (mut executor, _node, _set, _get, _load) = match setup {
                Ok((executor, node, set, get, load, truth)) => {
                    let _ = ready_tx.send(Ok(truth));
                    (executor, node, set, get, load)
                }
                Err(e) => {
                    let _ = ready_tx.send(Err(e.to_string()));
                    return;
                }
            };
            while !shutdown.load(Ordering::SeqCst) {
                executor.spin(rclrs::SpinOptions::default().timeout(Duration::from_millis(200)));
            }
        })?;
    match ready_rx.recv() {
        Ok(Ok(truth)) => {
            tracing::info!("ROS node up: serving {SET_WEATHER}, {GET_WEATHER}, {LOAD_MAP}");
            if truth.is_some() {
                tracing::info!(
                    "Truth topics on: {} and {} (latched)",
                    truth_topic::FRAME_TOPIC,
                    truth_topic::ENTITIES_TOPIC
                );
            }
            Ok((rx, truth))
        }
        Ok(Err(e)) => Err(eyre::eyre!("ROS node: {e}")),
        Err(_) => Err(eyre::eyre!("ROS node thread ended before starting")),
    }
}

async fn ask(
    tx: mpsc::Sender<WorldCommand>,
    make: impl FnOnce(oneshot::Sender<WeatherReply>) -> WorldCommand,
) -> WeatherReply {
    let (reply_tx, reply_rx) = oneshot::channel();
    if tx.send(make(reply_tx)).is_err() {
        return WeatherReply::failed("the bridge is shutting down", None);
    }
    reply_rx
        .await
        .unwrap_or_else(|_| WeatherReply::failed("the bridge dropped the request", None))
}

async fn set_weather(
    tx: mpsc::Sender<WorldCommand>,
    req: SetWeather_Request,
) -> SetWeather_Response {
    let reply = match weather::decode_request(&req.preset, &req.names, &req.values) {
        Ok(request) => ask(tx, |r| WorldCommand::SetWeather(request, r)).await,
        Err(e) => WeatherReply::failed(e, None),
    };
    let (preset, names, values) = fields(&reply);
    tracing::info!(
        "{SET_WEATHER} {:?} {:?} -> {}",
        req.preset,
        req.names,
        reply.message
    );
    SetWeather_Response {
        success: reply.success,
        message: reply.message,
        preset,
        names,
        values,
    }
}

async fn get_weather(tx: mpsc::Sender<WorldCommand>) -> GetWeather_Response {
    let reply = ask(tx, WorldCommand::GetWeather).await;
    let (preset, names, values) = fields(&reply);
    GetWeather_Response {
        success: reply.success,
        message: reply.message,
        preset,
        names,
        values,
    }
}

async fn load_map(tx: mpsc::Sender<WorldCommand>, req: LoadMap_Request) -> LoadMap_Response {
    let (reply_tx, reply_rx) = oneshot::channel();
    let reply = if tx
        .send(WorldCommand::LoadMap(req.map_path.clone(), reply_tx))
        .is_err()
    {
        Err("the bridge is shutting down".to_string())
    } else {
        reply_rx
            .await
            .unwrap_or_else(|_| Err("the bridge dropped the request".into()))
    };
    let (success, message) = match reply {
        Ok(m) => (true, m),
        Err(m) => (false, m),
    };
    tracing::info!("{LOAD_MAP} {:?} -> {message}", req.map_path);
    LoadMap_Response { success, message }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::weather::Weather;

    #[test]
    fn a_reply_becomes_the_services_fields() {
        let w = Weather {
            preset: "ClearNoon".into(),
            parameters: [
                ("cloudiness".to_string(), 5.0),
                ("wetness".to_string(), 0.0),
            ]
            .into_iter()
            .collect(),
        };
        let (preset, names, values) = fields(&WeatherReply::ok(w, ""));
        assert_eq!(preset, "ClearNoon");
        assert_eq!(names, ["cloudiness", "wetness"]);
        assert_eq!(values, [5.0, 0.0]);
        assert_eq!(fields(&WeatherReply::failed("x", None)), Default::default());
    }

    #[test]
    fn a_map_is_named_by_its_directory_or_its_lanelet2_file() {
        let dir = std::env::temp_dir().join(format!("scensim-load-map-{}", std::process::id()));
        let map = dir.join("Town02");
        std::fs::create_dir_all(&map).unwrap();
        let osm = map.join(LANELET2_FILE);
        std::fs::write(&osm, "<osm/>").unwrap();
        let want = osm.to_string_lossy().into_owned();
        assert_eq!(lanelet2_path(&map.to_string_lossy()).unwrap(), want);
        assert_eq!(lanelet2_path(&want).unwrap(), want);
        let why = lanelet2_path(&dir.to_string_lossy()).unwrap_err();
        assert!(why.contains("does not exist"), "{why}");
        assert!(lanelet2_path("maps/Town02")
            .unwrap_err()
            .contains("relative"));
        assert!(lanelet2_path(" ").unwrap_err().contains("no map_path"));
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
