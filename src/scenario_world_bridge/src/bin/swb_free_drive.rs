//! Free drive (roadmap 073 S5): the ego in a world with no scenario. In SSv2's place, this
//! speaks SSv2's protocol to the bridge: it initializes the episode on a map, spawns the ego
//! from its vehicle catalog entry at a pose, and steps time; nothing else. The ego's autopilot
//! is driven by hand (RViz: a goal, then engage).
//!
//! ```text
//! swb_free_drive --map <map dir> --catalog <vehicles.xosc> --entry <name> --pose x,y,z,yaw
//!     [--port 5555] [--step 0.05] [--realtime true|false] [--name ego]
//!     [--max-speed 50] [--max-acceleration 3] [--max-deceleration 5] [--max-steering 0.6]
//! ```
//!
//! Time is synchronous, as under a scenario: the world advances one step per frame this
//! sends. `--realtime true` (the default) paces the frames to the wall clock; `false` sends
//! the next as soon as the last returns. The pose is the map frame's (base_link, yaw in rad
//! counter-clockwise from east), as a scenario's WorldPosition. Ctrl-C ends it; the world
//! stays where it is until the next Initialize.

use std::time::{Duration, Instant};

use eyre::{bail, eyre, Result, WrapErr};
use prost::Message;
use swb_core::proto::geometry_msgs::{Point, Pose, Quaternion, Vector3};
use swb_core::proto::simulation_api_schema::{
    self as api, simulation_request, simulation_response, SimulationRequest, SimulationResponse,
};
use swb_core::proto::traffic_simulator_msgs::{
    Axle, Axles, BoundingBox, EntitySubtype, Performance, VehicleParameters,
};

struct Args {
    map: String,
    catalog: String,
    entry: String,
    pose: [f64; 4],
    port: u16,
    step: f64,
    realtime: bool,
    name: String,
    max_speed: f64,
    max_acceleration: f64,
    max_deceleration: f64,
    max_steering: f64,
}

fn parse_args() -> Result<Args> {
    let mut a = Args {
        map: String::new(),
        catalog: String::new(),
        entry: String::new(),
        pose: [0.0; 4],
        port: 5555,
        step: 0.05,
        realtime: true,
        name: "ego".into(),
        max_speed: 50.0,
        max_acceleration: 3.0,
        max_deceleration: 5.0,
        max_steering: 0.6,
    };
    let mut pose = None;
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        if flag == "--ros-args" {
            break;
        }
        let (key, value) = match flag.split_once('=') {
            Some((k, v)) => (k.to_string(), v.to_string()),
            None => (
                flag.clone(),
                it.next().ok_or_else(|| eyre!("{flag} needs a value"))?,
            ),
        };
        let num = |v: &str| v.parse::<f64>().wrap_err_with(|| format!("{key} {v}"));
        match key.as_str() {
            "--map" => a.map = value,
            "--catalog" => a.catalog = value,
            "--entry" => a.entry = value,
            "--pose" => pose = Some(value),
            "--port" => a.port = value.parse().wrap_err("--port")?,
            "--step" => a.step = num(&value)?,
            "--realtime" => a.realtime = matches!(value.as_str(), "true" | "1"),
            "--name" => a.name = value,
            "--max-speed" => a.max_speed = num(&value)?,
            "--max-acceleration" => a.max_acceleration = num(&value)?,
            "--max-deceleration" => a.max_deceleration = num(&value)?,
            "--max-steering" => a.max_steering = num(&value)?,
            _ => bail!("unknown argument {key}"),
        }
    }
    for (what, v) in [
        ("--map", &a.map),
        ("--catalog", &a.catalog),
        ("--entry", &a.entry),
    ] {
        if v.is_empty() {
            bail!("{what} is required");
        }
    }
    let pose = pose.ok_or_else(|| eyre!("--pose x,y,z,yaw is required"))?;
    let p: Vec<f64> = pose
        .split(',')
        .map(|s| s.trim().parse::<f64>())
        .collect::<Result<_, _>>()
        .wrap_err_with(|| format!("--pose {pose}"))?;
    if p.len() != 4 {
        bail!("--pose {pose}: x,y,z,yaw");
    }
    a.pose = [p[0], p[1], p[2], p[3]];
    if a.step <= 0.0 {
        bail!("--step must be positive");
    }
    Ok(a)
}

/// The catalog entry as SSv2 would send it: the box and axles are the car's; the
/// performance parameters are the scenario's (here, arguments).
fn vehicle_parameters(a: &Args) -> Result<(VehicleParameters, String)> {
    let text = std::fs::read_to_string(&a.catalog).wrap_err_with(|| a.catalog.clone())?;
    let doc = roxmltree::Document::parse(&text).wrap_err_with(|| a.catalog.clone())?;
    let v = doc
        .descendants()
        .find(|n| n.has_tag_name("Vehicle") && n.attribute("name") == Some(a.entry.as_str()))
        .ok_or_else(|| eyre!("{}: no Vehicle {}", a.catalog, a.entry))?;
    fn child<'a, 'i>(n: roxmltree::Node<'a, 'i>, tag: &str) -> Result<roxmltree::Node<'a, 'i>> {
        n.children()
            .find(|c| c.has_tag_name(tag))
            .ok_or_else(|| eyre!("the catalog entry has no {tag}"))
    }
    let f = |n: roxmltree::Node, attr: &str, steering: f64| -> Result<f64> {
        let s = n.attribute(attr).unwrap_or("0");
        if s.starts_with('$') {
            return Ok(steering);
        }
        s.parse::<f64>()
            .wrap_err_with(|| format!("{}: {attr}={s}", a.catalog))
    };
    let bb = child(v, "BoundingBox")?;
    let (c, d) = (child(bb, "Center")?, child(bb, "Dimensions")?);
    let axles = child(v, "Axles")?;
    let axle = |tag: &str| -> Result<Axle> {
        let n = child(axles, tag)?;
        Ok(Axle {
            max_steering: f(n, "maxSteering", a.max_steering)?,
            wheel_diameter: f(n, "wheelDiameter", 0.0)?,
            track_width: f(n, "trackWidth", 0.0)?,
            position_x: f(n, "positionX", 0.0)?,
            position_z: f(n, "positionZ", 0.0)?,
        })
    };
    let params = VehicleParameters {
        name: a.name.clone(),
        performance: Some(Performance {
            max_speed: a.max_speed,
            max_acceleration: a.max_acceleration,
            max_acceleration_rate: 0.0,
            max_deceleration: a.max_deceleration,
            max_deceleration_rate: 0.0,
        }),
        bounding_box: Some(BoundingBox {
            center: Some(Point {
                x: f(c, "x", 0.0)?,
                y: f(c, "y", 0.0)?,
                z: f(c, "z", 0.0)?,
            }),
            dimensions: Some(Vector3 {
                x: f(d, "length", 0.0)?,
                y: f(d, "width", 0.0)?,
                z: f(d, "height", 0.0)?,
            }),
        }),
        axles: Some(Axles {
            front_axle: Some(axle("FrontAxle")?),
            rear_axle: Some(axle("RearAxle")?),
        }),
        property: None,
        subtype: Some(EntitySubtype {
            value: swb_core::proto::traffic_simulator_msgs::entity_subtype::Enum::Car as i32,
        }),
    };
    Ok((params, v.attribute("model3d").unwrap_or("").to_string()))
}

struct Bridge {
    socket: zmq::Socket,
}

impl Bridge {
    fn call(&self, request: simulation_request::Request) -> Result<simulation_response::Response> {
        let msg = SimulationRequest {
            request: Some(request),
        };
        self.socket.send(msg.encode_to_vec(), 0)?;
        let reply = self.socket.recv_bytes(0)?;
        SimulationResponse::decode(reply.as_slice())?
            .response
            .ok_or_else(|| eyre!("empty response"))
    }
}

fn check(what: &str, result: Option<api::Result>) -> Result<()> {
    match result {
        Some(r) if r.success => Ok(()),
        Some(r) => bail!("{what}: {}", r.description),
        None => bail!("{what}: no result"),
    }
}

fn main() -> Result<()> {
    let a = parse_args()?;
    let (params, model3d) = vehicle_parameters(&a)?;
    let lanelet = std::path::Path::new(&a.map).join("lanelet2_map.osm");
    if !lanelet.is_file() {
        bail!("{}: no lanelet2_map.osm", a.map);
    }

    let ctx = zmq::Context::new();
    let socket = ctx.socket(zmq::REQ)?;
    socket.set_linger(0)?;
    socket.connect(&format!("tcp://127.0.0.1:{}", a.port))?;
    let bridge = Bridge { socket };
    eprintln!(
        "free drive: map {}, ego {} ({}) at {:?}, step {} s, {}",
        a.map,
        a.entry,
        model3d,
        a.pose,
        a.step,
        if a.realtime {
            "real time"
        } else {
            "as fast as it goes"
        }
    );

    use simulation_request::Request as Q;
    use simulation_response::Response as R;
    match bridge.call(Q::Initialize(api::InitializeRequest {
        realtime_factor: 1.0,
        step_time: a.step,
        initialize_time: 0.0,
        initialize_ros_time: None,
        lanelet2_map_path: lanelet.to_string_lossy().into_owned(),
    }))? {
        R::Initialize(r) => check("Initialize", r.result)?,
        other => bail!("Initialize: unexpected {other:?}"),
    }

    let [x, y, z, yaw] = a.pose;
    let pose = Pose {
        position: Some(Point { x, y, z }),
        orientation: Some(Quaternion {
            x: 0.0,
            y: 0.0,
            z: (yaw / 2.0).sin(),
            w: (yaw / 2.0).cos(),
        }),
    };
    match bridge.call(Q::SpawnVehicleEntity(api::SpawnVehicleEntityRequest {
        parameters: Some(params),
        is_ego: true,
        asset_key: model3d,
        pose: Some(pose),
        initial_speed: 0.0,
        behavior: "Autoware".into(),
    }))? {
        R::SpawnVehicleEntity(r) => check("spawn the ego", r.result)?,
        other => bail!("spawn: unexpected {other:?}"),
    }
    eprintln!("free drive: the ego is in; driving by hand (RViz), Ctrl-C ends");

    let step = Duration::from_secs_f64(a.step);
    let start = Instant::now();
    let mut t = 0.0_f64;
    let mut frames: u64 = 0;
    loop {
        match bridge.call(Q::UpdateEntityStatus(api::UpdateEntityStatusRequest {
            status: vec![],
            npc_logic_started: true,
            overwrite_ego_status: false,
        }))? {
            R::UpdateEntityStatus(r) => check("UpdateEntityStatus", r.result)?,
            other => bail!("UpdateEntityStatus: unexpected {other:?}"),
        }
        t += a.step;
        match bridge.call(Q::UpdateFrame(api::UpdateFrameRequest {
            current_simulation_time: t,
            current_scenario_time: t,
            current_ros_time: None,
        }))? {
            R::UpdateFrame(r) => check("UpdateFrame", r.result)?,
            other => bail!("UpdateFrame: unexpected {other:?}"),
        }
        frames += 1;
        if frames.is_multiple_of(200) {
            let wall = start.elapsed().as_secs_f64();
            eprintln!(
                "free drive: {t:.1} s simulated in {wall:.1} s ({:.2}x)",
                t / wall
            );
        }
        if a.realtime {
            let due = step.mul_f64(frames as f64);
            if let Some(wait) = due.checked_sub(start.elapsed()) {
                std::thread::sleep(wait);
            }
        }
    }
}
