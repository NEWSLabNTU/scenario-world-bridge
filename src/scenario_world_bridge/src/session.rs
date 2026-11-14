//! One SSv2 at a time (roadmap 077, G21): the session a client's `Initialize` starts.
//!
//! The bridge serves SSv2 on a ROUTER socket, so it knows which connection each request came
//! from (ZMQ's routing id, one per REQ socket; SSv2 opens one per scenario, in
//! `traffic_simulator::API`, and Initializes on it first). The connection whose `Initialize`
//! was accepted last holds the session:
//!
//! - any other request from another connection is refused: a stray SSv2 never steps the
//!   world or spawns into the scenario of the one that holds it;
//! - an `Initialize` from another connection is refused while the session is **live**: it
//!   has entities and its client asked within [`LIVE_WINDOW`]. Otherwise it takes the
//!   session over: the previous scenario ended (SSv2 despawns every entity when it
//!   deactivates), never spawned, or its SSv2 died without a word (silent for the window).
//!
//! No session (a fresh bridge, or after `load_map`): every request is served, as before.

use std::time::{Duration, Instant};

/// How long a session with entities stays live without a request from its client. A running
/// SSv2 asks every frame (50 ms at the default step); a dead one stops at once. Short, so the
/// next run's Initialize after an SSv2 killed mid-scenario (its entities left) is not refused:
/// a new SSv2 takes longer than this to start.
pub const LIVE_WINDOW: Duration = Duration::from_secs(5);

#[derive(Debug, Clone)]
struct Session {
    peer: Vec<u8>,
    map: String,
    since: Instant,
    last_request: Instant,
}

#[derive(Debug, Default)]
pub struct Sessions {
    current: Option<Session>,
}

/// A connection's routing id, for messages.
pub fn peer_name(peer: &[u8]) -> String {
    if peer.is_empty() {
        return "?".into();
    }
    peer.iter().map(|b| format!("{b:02x}")).collect()
}

impl Session {
    fn live(&self, entities: usize, now: Instant) -> bool {
        entities > 0 && now.saturating_duration_since(self.last_request) < LIVE_WINDOW
    }

    fn describe(&self, entities: usize, now: Instant) -> String {
        format!(
            "SSv2 client {} (Initialized {:.0} s ago on {}; {entities} entit{}, last request \
             {:.1} s ago)",
            peer_name(&self.peer),
            now.saturating_duration_since(self.since).as_secs_f64(),
            if self.map.is_empty() {
                "no map"
            } else {
                &self.map
            },
            if entities == 1 { "y" } else { "ies" },
            now.saturating_duration_since(self.last_request)
                .as_secs_f64(),
        )
    }
}

impl Sessions {
    /// May `peer` send this request? `initialize`: the request is an `Initialize` (for `map`);
    /// `entities`: what the bridge has spawned now. An accepted `Initialize` makes `peer` the
    /// session's client. `Err` is the refusal, for SSv2's `description`.
    pub fn admit(
        &mut self,
        peer: &[u8],
        initialize: Option<&str>,
        entities: usize,
        now: Instant,
    ) -> Result<(), String> {
        match (initialize, &mut self.current) {
            (Some(map), current) => {
                if let Some(s) = current {
                    if s.peer != peer && s.live(entities, now) {
                        return Err(format!(
                            "swb refused Initialize from SSv2 client {}: another SSv2 is running \
                             a scenario on this bridge, {}. One bridge serves one SSv2 at a time; \
                             stop the other (a scenario_test_runner left over from an earlier \
                             run?) or wait for its scenario to end",
                            peer_name(peer),
                            s.describe(entities, now)
                        ));
                    }
                }
                *current = Some(Session {
                    peer: peer.to_vec(),
                    map: map.to_string(),
                    since: now,
                    last_request: now,
                });
                Ok(())
            }
            (None, None) => Ok(()),
            (None, Some(s)) if s.peer == peer => {
                s.last_request = now;
                Ok(())
            }
            (None, Some(s)) => Err(format!(
                "swb refused a request from SSv2 client {}, which did not Initialize this \
                 bridge's session: it is {}'s. One bridge serves one SSv2 at a time; a client \
                 starts with Initialize",
                peer_name(peer),
                s.describe(entities, now)
            )),
        }
    }

    /// The session's client was answered at `now`: a long request (a world load, a spawn
    /// that waits for sensors) does not count as silence.
    pub fn answered(&mut self, peer: &[u8], now: Instant) {
        if let Some(s) = &mut self.current {
            if s.peer == peer {
                s.last_request = now;
            }
        }
    }

    /// The scenario that is running, if one is: what a world load between scenarios refuses.
    pub fn running(&self, entities: usize, now: Instant) -> Option<String> {
        self.current
            .as_ref()
            .filter(|s| s.live(entities, now))
            .map(|s| s.describe(entities, now))
    }

    /// Forget the session (the world was loaded anew: its scenario is gone).
    pub fn clear(&mut self) {
        self.current = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const A: &[u8] = &[0, 0x6b, 0x8b, 0x45, 0x67];
    const B: &[u8] = &[0, 0x6b, 0x8b, 0x45, 0x68];

    fn at(t0: Instant, s: f64) -> Instant {
        t0 + Duration::from_secs_f64(s)
    }

    #[test]
    fn a_fresh_bridge_serves_whoever_asks() {
        let mut s = Sessions::default();
        let t0 = Instant::now();
        assert!(s.admit(A, None, 0, t0).is_ok());
        assert!(s.admit(B, None, 3, t0).is_ok());
        assert!(s.running(3, t0).is_none());
    }

    #[test]
    fn a_second_ssv2_cannot_initialize_into_a_running_scenario() {
        let mut s = Sessions::default();
        let t0 = Instant::now();
        s.admit(A, Some("Town01"), 0, t0).unwrap();
        s.admit(A, None, 4, at(t0, 10.0)).unwrap();
        let why = s.admit(B, Some("Town01"), 4, at(t0, 10.05)).unwrap_err();
        assert!(why.contains("refused Initialize"), "{why}");
        assert!(
            why.contains(&peer_name(B)) && why.contains(&peer_name(A)),
            "{why}"
        );
        assert!(
            why.contains("Town01") && why.contains("4 entities"),
            "{why}"
        );
        // The session stays A's: it goes on, and B is still refused for anything else.
        assert!(s.admit(A, None, 4, at(t0, 10.1)).is_ok());
        assert!(s.admit(B, None, 4, at(t0, 10.1)).is_err());
        assert!(s.running(4, at(t0, 10.1)).is_some());
    }

    #[test]
    fn a_client_that_did_not_initialize_is_refused() {
        let mut s = Sessions::default();
        let t0 = Instant::now();
        s.admit(A, Some("Town01"), 0, t0).unwrap();
        // Even with nothing spawned yet, and long after: only an Initialize takes over.
        let why = s.admit(B, None, 0, at(t0, 60.0)).unwrap_err();
        assert!(why.contains("did not Initialize"), "{why}");
    }

    #[test]
    fn the_next_scenario_after_one_ended_initializes() {
        // SSv2's next scenario (or variant) is a new connection: its Initialize comes right
        // after the last one despawned everything.
        let mut s = Sessions::default();
        let t0 = Instant::now();
        s.admit(A, Some("Town01"), 0, t0).unwrap();
        s.admit(A, None, 3, at(t0, 30.0)).unwrap();
        s.admit(A, None, 0, at(t0, 30.1)).unwrap(); // its last despawn
        assert!(s.running(0, at(t0, 30.2)).is_none());
        s.admit(B, Some("Town02"), 0, at(t0, 30.5)).unwrap();
        // B holds it now; A's leftovers are refused.
        assert!(s.admit(A, None, 0, at(t0, 30.6)).is_err());
        assert!(s.admit(B, None, 1, at(t0, 30.6)).is_ok());
    }

    #[test]
    fn a_scenario_that_never_spawned_does_not_block_the_next() {
        // An ego refused at its spawn: SSv2 ends the scenario at once, with nothing to despawn.
        let mut s = Sessions::default();
        let t0 = Instant::now();
        s.admit(A, Some("Town01"), 0, t0).unwrap();
        s.admit(A, None, 0, at(t0, 0.2)).unwrap();
        assert!(s.admit(B, Some("Town01"), 0, at(t0, 0.5)).is_ok());
    }

    #[test]
    fn an_ssv2_that_died_mid_scenario_lets_the_next_one_in() {
        let mut s = Sessions::default();
        let t0 = Instant::now();
        s.admit(A, Some("Town01"), 0, t0).unwrap();
        s.admit(A, None, 5, at(t0, 20.0)).unwrap();
        // killed: its entities stay, its requests stop
        assert!(s.admit(B, Some("Town01"), 5, at(t0, 24.0)).is_err());
        assert!(s.admit(B, Some("Town01"), 5, at(t0, 25.1)).is_ok());
    }

    #[test]
    fn the_same_client_may_initialize_again() {
        let mut s = Sessions::default();
        let t0 = Instant::now();
        s.admit(A, Some("Town01"), 0, t0).unwrap();
        s.admit(A, None, 2, at(t0, 1.0)).unwrap();
        assert!(s.admit(A, Some("Town01"), 2, at(t0, 1.1)).is_ok());
    }

    #[test]
    fn a_long_answer_is_not_silence() {
        let mut s = Sessions::default();
        let t0 = Instant::now();
        s.admit(A, Some("Town01"), 0, t0).unwrap();
        s.admit(A, None, 1, at(t0, 1.0)).unwrap();
        // the spawn's answer took 30 s (an Isaac ego waits for its sensors)
        s.answered(A, at(t0, 31.0));
        assert!(s.admit(B, Some("Town01"), 1, at(t0, 33.0)).is_err());
        assert!(s.running(1, at(t0, 33.0)).is_some());
    }

    #[test]
    fn clearing_forgets_the_session() {
        let mut s = Sessions::default();
        let t0 = Instant::now();
        s.admit(A, Some("Town01"), 0, t0).unwrap();
        s.clear();
        assert!(s.admit(B, None, 0, t0).is_ok());
    }
}
