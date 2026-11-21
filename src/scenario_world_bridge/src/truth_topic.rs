//! The live truth stream (roadmap 092 K4, `docs/design/truth-stream.md` section 4):
//!
//!   /scensim/truth           scensim_interfaces/msg/TruthFrame     each SSv2 step, reliable, keep last 10
//!   /scensim/truth/entities  scensim_interfaces/msg/TruthEntities  latched (transient local), at each
//!                                                                  episode start, spawn and despawn
//!
//! On when the bridge's `truth_topic` parameter is true (a testbench's `logging.truth_topic`, or any
//! attachment). The content is the run's truth file's (the gap trace at schema 2): the same poses,
//! velocities, contacts and signal changes, published as soon as each step's reply arrives.
//!
//! [`TruthState`] builds the messages and knows nothing of ROS, so it is tested on its own;
//! [`TruthTopic`] holds it with the two publishers, which the ROS node's thread creates and the
//! world-owning thread publishes on.

use rclrs::Publisher;
use scensim_interfaces::msg::{TruthContact, TruthEntities, TruthEntity, TruthFrame, TruthSignal};

use crate::gap_trace::SimState;

pub const FRAME_TOPIC: &str = "/scensim/truth";
pub const ENTITIES_TOPIC: &str = "/scensim/truth/entities";
/// The frame topic's depth: a slow reader loses frames beyond it, never the world's time.
pub const FRAME_DEPTH: u32 = 10;

/// What the stream knows between steps, independent of ROS.
#[derive(Debug, Default)]
pub struct TruthState {
    episode: u64,
    map: String,
    step_time: f64,
    next_index: u32,
    /// The entities alive now, in spawn order.
    entities: Vec<TruthEntity>,
    /// Since the last frame: contacts the server reported and signal colours the bridge set.
    contacts: Vec<TruthContact>,
    signals: Vec<TruthSignal>,
}

impl TruthState {
    /// A new episode: no entities, slots from 0.
    pub fn begin_episode(&mut self, episode: u64, map: &str, step_time: f64) -> TruthEntities {
        *self = Self {
            episode,
            map: map.to_string(),
            step_time,
            ..Self::default()
        };
        self.entities_msg()
    }

    /// An entity spawned: its slot is the next one, never reused in the episode (a respawn
    /// under the same name replaces the old entry and gets a new slot).
    pub fn spawn(&mut self, mut entity: TruthEntity) -> TruthEntities {
        self.entities.retain(|e| e.name != entity.name);
        entity.index = self.next_index;
        self.next_index += 1;
        self.entities.push(entity);
        self.entities_msg()
    }

    /// An entity despawned; `None` when the stream did not have it.
    pub fn despawn(&mut self, name: &str) -> Option<TruthEntities> {
        let before = self.entities.len();
        self.entities.retain(|e| e.name != name);
        (self.entities.len() != before).then(|| self.entities_msg())
    }

    pub fn contact(&mut self, contact: TruthContact) {
        self.contacts.push(contact);
    }

    pub fn signal(&mut self, opendrive_id: &str, colour: &str) {
        self.signals.push(TruthSignal {
            opendrive_id: opendrive_id.to_string(),
            colour: colour.to_string(),
        });
    }

    /// The frame after SSv2 step `frame`: one row per alive entity, in spawn order, with the
    /// simulator's state from `state_of` (by sim id; `None`: not in the snapshot, the row's
    /// values 0 and `present` false). Takes the contacts and signals gathered since the last.
    pub fn frame<F>(
        &mut self,
        frame: u64,
        sim_time_ns: i64,
        sim_frame: u64,
        mut state_of: F,
    ) -> TruthFrame
    where
        F: FnMut(u32) -> Option<SimState>,
    {
        let n = self.entities.len();
        let mut out = TruthFrame {
            episode: self.episode,
            frame,
            sim_time_ns,
            sim_frame,
            index: Vec::with_capacity(n),
            names: Vec::with_capacity(n),
            present: Vec::with_capacity(n),
            poses: Vec::with_capacity(6 * n),
            velocities: Vec::with_capacity(3 * n),
            angular_velocities: Vec::with_capacity(3 * n),
            velocity_measured: Vec::with_capacity(n),
            contacts: std::mem::take(&mut self.contacts),
            signals: std::mem::take(&mut self.signals),
        };
        for e in &self.entities {
            let s = state_of(e.sim_id);
            out.index.push(e.index);
            out.names.push(e.name.clone());
            out.present.push(s.is_some());
            out.poses.extend(s.map_or([0.0; 6], |s| s.pose));
            out.velocities.extend(s.map_or([0.0; 3], |s| s.velocity));
            out.angular_velocities
                .extend(s.map_or([0.0; 3], |s| s.angular_velocity));
            out.velocity_measured.push(s.is_some_and(|s| s.measured));
        }
        out
    }

    fn entities_msg(&self) -> TruthEntities {
        TruthEntities {
            episode: self.episode,
            map: self.map.clone(),
            step_time: self.step_time,
            entities: self.entities.clone(),
        }
    }
}

/// The stream with its publishers. A failed publish is logged once and the run goes on: the
/// stream serves readers, it never holds the world.
pub struct TruthTopic {
    pub state: TruthState,
    frames: Publisher<TruthFrame>,
    entities: Publisher<TruthEntities>,
    warned: bool,
}

impl TruthTopic {
    pub fn new(frames: Publisher<TruthFrame>, entities: Publisher<TruthEntities>) -> Self {
        Self {
            state: TruthState::default(),
            frames,
            entities,
            warned: false,
        }
    }

    pub fn publish_entities(&mut self, msg: TruthEntities) {
        let r = self.entities.publish(msg);
        self.check(r);
    }

    pub fn publish_frame(&mut self, msg: TruthFrame) {
        let r = self.frames.publish(msg);
        self.check(r);
    }

    fn check(&mut self, r: Result<(), rclrs::RclrsError>) {
        if let Err(e) = r {
            if !self.warned {
                self.warned = true;
                tracing::warn!("Truth topic: publish failed ({e}); later failures are not logged");
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entity(name: &str, sim_id: u32) -> TruthEntity {
        TruthEntity {
            name: name.into(),
            sim_id,
            role: "scenario".into(),
            ..Default::default()
        }
    }

    fn sim(x: f64, measured: bool) -> SimState {
        SimState {
            pose: [x, 1.0, 0.0, 0.0, 0.0, 0.5],
            velocity: [2.0, 0.0, 0.0],
            angular_velocity: [0.0, 0.0, 0.1],
            measured,
        }
    }

    #[test]
    fn slots_are_assigned_at_spawn_and_never_reused() {
        let mut t = TruthState::default();
        let m = t.begin_episode(3, "Town01", 0.05);
        assert_eq!(
            (m.episode, m.map.as_str(), m.step_time),
            (3, "Town01", 0.05)
        );
        assert!(m.entities.is_empty());
        t.spawn(entity("ego", 10));
        t.spawn(entity("npc", 11));
        let m = t.despawn("ego").expect("had it");
        assert_eq!(m.entities.len(), 1);
        assert!(t.despawn("ego").is_none());
        // A respawn under a known name replaces it, with a new slot.
        let m = t.spawn(entity("npc", 12));
        assert_eq!(
            m.entities
                .iter()
                .map(|e| (e.name.as_str(), e.index, e.sim_id))
                .collect::<Vec<_>>(),
            [("npc", 2, 12)]
        );
        // A new episode starts the slots again.
        t.begin_episode(4, "Town01", 0.05);
        assert_eq!(t.spawn(entity("ego", 20)).entities[0].index, 0);
    }

    #[test]
    fn a_frame_is_rows_of_flat_arrays_and_drains_events() {
        let mut t = TruthState::default();
        t.begin_episode(1, "field", 0.05);
        t.spawn(entity("ego", 10));
        t.spawn(entity("npc", 11));
        t.signal("12", "red");
        t.contact(TruthContact {
            name: "ego".into(),
            other: "npc".into(),
            other_sim_id: 11,
            other_type: "vehicle".into(),
            impulse: 30.0,
        });
        let f = t.frame(7, 1_000_350_000_000, 5007, |id| {
            (id == 10).then(|| sim(4.0, true))
        });
        assert_eq!(
            (f.episode, f.frame, f.sim_time_ns, f.sim_frame),
            (1, 7, 1_000_350_000_000, 5007)
        );
        assert_eq!(f.index, [0, 1]);
        assert_eq!(f.names, ["ego", "npc"]);
        assert_eq!(f.present, [true, false]);
        assert_eq!(f.poses.len(), 12);
        assert_eq!(&f.poses[..6], &[4.0, 1.0, 0.0, 0.0, 0.0, 0.5]);
        assert_eq!(&f.poses[6..], &[0.0; 6]);
        assert_eq!(f.velocities, [2.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        assert_eq!(f.angular_velocities[2], 0.1);
        assert_eq!(f.velocity_measured, [true, false]);
        assert_eq!(f.signals.len(), 1);
        assert_eq!(f.contacts[0].impulse, 30.0);
        // Events go out once.
        let f = t.frame(8, 1_000_400_000_000, 5008, |_| None);
        assert!(f.signals.is_empty() && f.contacts.is_empty());
    }

    /// The topics on a real ROS graph: a frame arrives, and a subscriber that joins after the
    /// entities were published still gets them (latched). Its own domain, so a live stack's
    /// graph is untouched; skipped without ROS.
    #[test]
    fn the_topics_carry_frames_and_latch_the_entities() {
        use rclrs::{CreateBasicExecutor, IntoPrimitiveOptions, SpinOptions};
        use std::sync::{Arc, Mutex};
        use std::time::{Duration, Instant};

        let opts = rclrs::InitOptions::new().with_domain_id(Some(53));
        let ctx = match rclrs::Context::new(["truth_topic_test".to_string()], opts) {
            Ok(c) => c,
            Err(e) => {
                eprintln!("SKIPPED: no ROS context ({e})");
                return;
            }
        };
        let mut executor = ctx.create_basic_executor();
        let node = executor.create_node("truth_topic_test_pub").unwrap();
        let frames = node
            .create_publisher::<TruthFrame>(FRAME_TOPIC.keep_last(FRAME_DEPTH).reliable())
            .unwrap();
        let entities = node
            .create_publisher::<TruthEntities>(
                ENTITIES_TOPIC.keep_last(1).reliable().transient_local(),
            )
            .unwrap();
        let mut topic = TruthTopic::new(frames, entities);
        let m = topic.state.begin_episode(1, "Town01", 0.05);
        topic.publish_entities(m);
        let m = topic.state.spawn(entity("ego", 10));
        topic.publish_entities(m);

        // The reader joins after the entities went out.
        let reader = executor.create_node("truth_topic_test_sub").unwrap();
        let got_entities = Arc::new(Mutex::new(Vec::<TruthEntities>::new()));
        let got_frames = Arc::new(Mutex::new(Vec::<TruthFrame>::new()));
        let ge = got_entities.clone();
        let _se = reader
            .create_subscription::<TruthEntities, _>(
                ENTITIES_TOPIC.keep_last(1).reliable().transient_local(),
                move |m: TruthEntities| ge.lock().unwrap().push(m),
            )
            .unwrap();
        let gf = got_frames.clone();
        let _sf = reader
            .create_subscription::<TruthFrame, _>(
                FRAME_TOPIC.keep_last(FRAME_DEPTH).reliable(),
                move |m: TruthFrame| gf.lock().unwrap().push(m),
            )
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(10);
        let mut frame = 0;
        while Instant::now() < deadline
            && (got_frames.lock().unwrap().is_empty() || got_entities.lock().unwrap().is_empty())
        {
            frame += 1;
            let f = topic
                .state
                .frame(frame, 1_000_000_000_000, frame, |_| Some(sim(1.0, true)));
            topic.publish_frame(f);
            executor.spin(SpinOptions::default().timeout(Duration::from_millis(100)));
        }
        let e = got_entities.lock().unwrap();
        assert!(!e.is_empty(), "the late reader got no latched entities");
        let last = e.last().unwrap();
        assert_eq!((last.map.as_str(), last.entities.len()), ("Town01", 1));
        assert_eq!(last.entities[0].name, "ego");
        let f = got_frames.lock().unwrap();
        assert!(!f.is_empty(), "no frame arrived");
        assert_eq!(f[0].names, ["ego"]);
        assert_eq!(f[0].poses.len(), 6);
    }
}
