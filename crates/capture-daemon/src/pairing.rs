//! FIFO pairing of `ShotTriggered` events with camera frames.
//!
//! Pure logic: every call takes the current time, so tests need no sleeping.
//! Items older than `max_age` are dropped and reported, never mispaired.
//! Dropping a stale item does not try to "resync" anything else: the
//! hardware delivers one shot message and one frame per trigger, in order,
//! so after a loss the next arrivals simply pair with each other.

use std::collections::VecDeque;
use std::time::{Duration, Instant};

pub const DEFAULT_MAX_AGE: Duration = Duration::from_secs(2);

#[derive(Debug, Clone, PartialEq)]
pub enum Pairing<S, F> {
    Paired { shot: S, frame: F },
    FrameWithoutShot(F),
    ShotWithoutFrame(S),
}

pub struct Pairer<S, F> {
    max_age: Duration,
    shots: VecDeque<(Instant, S)>,
    frames: VecDeque<(Instant, F)>,
}

impl<S, F> Pairer<S, F> {
    pub fn new(max_age: Duration) -> Self {
        Self {
            max_age,
            shots: VecDeque::new(),
            frames: VecDeque::new(),
        }
    }

    /// Items currently waiting for a partner.
    pub fn pending(&self) -> (usize, usize) {
        (self.shots.len(), self.frames.len())
    }

    pub fn push_shot(&mut self, shot: S, at: Instant) -> Vec<Pairing<S, F>> {
        let mut out = self.expire(at);
        match self.frames.pop_front() {
            Some((_, frame)) => out.push(Pairing::Paired { shot, frame }),
            None => self.shots.push_back((at, shot)),
        }
        out
    }

    pub fn push_frame(&mut self, frame: F, at: Instant) -> Vec<Pairing<S, F>> {
        let mut out = self.expire(at);
        match self.shots.pop_front() {
            Some((_, shot)) => out.push(Pairing::Paired { shot, frame }),
            None => self.frames.push_back((at, frame)),
        }
        out
    }

    /// Drop everything that has waited longer than `max_age` at `now`.
    pub fn expire(&mut self, now: Instant) -> Vec<Pairing<S, F>> {
        let max_age = self.max_age;
        let stale = |t: &Instant| now.saturating_duration_since(*t) > max_age;
        let mut out = Vec::new();
        while self.shots.front().is_some_and(|(t, _)| stale(t)) {
            out.push(Pairing::ShotWithoutFrame(self.shots.pop_front().unwrap().1));
        }
        while self.frames.front().is_some_and(|(t, _)| stale(t)) {
            out.push(Pairing::FrameWithoutShot(
                self.frames.pop_front().unwrap().1,
            ));
        }
        out
    }
}

impl<S, F> Default for Pairer<S, F> {
    fn default() -> Self {
        Self::new(DEFAULT_MAX_AGE)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    type P = Pairer<u32, &'static str>;

    fn ms(base: Instant, n: u64) -> Instant {
        base + Duration::from_millis(n)
    }

    #[test]
    fn shot_then_frame_pairs() {
        let t0 = Instant::now();
        let mut p = P::default();
        assert!(p.push_shot(1, t0).is_empty());
        assert_eq!(
            p.push_frame("a", ms(t0, 30)),
            vec![Pairing::Paired {
                shot: 1,
                frame: "a"
            }]
        );
        assert_eq!(p.pending(), (0, 0));
    }

    #[test]
    fn frame_then_shot_pairs() {
        let t0 = Instant::now();
        let mut p = P::default();
        assert!(p.push_frame("a", t0).is_empty());
        assert_eq!(
            p.push_shot(1, ms(t0, 5)),
            vec![Pairing::Paired {
                shot: 1,
                frame: "a"
            }]
        );
    }

    #[test]
    fn stale_frame_is_dropped_not_paired_with_a_later_shot() {
        let t0 = Instant::now();
        let mut p = P::default();
        p.push_frame("old", t0);
        let out = p.push_shot(7, ms(t0, 2_500));
        assert_eq!(out, vec![Pairing::FrameWithoutShot("old")]);
        // The shot now waits for its own frame.
        assert_eq!(p.pending(), (1, 0));
        assert_eq!(
            p.push_frame("new", ms(t0, 2_530)),
            vec![Pairing::Paired {
                shot: 7,
                frame: "new"
            }]
        );
    }

    #[test]
    fn shot_without_frame_times_out() {
        let t0 = Instant::now();
        let mut p = P::default();
        p.push_shot(1, t0);
        assert!(
            p.expire(ms(t0, 2_000)).is_empty(),
            "exactly max_age is kept"
        );
        assert_eq!(p.expire(ms(t0, 2_001)), vec![Pairing::ShotWithoutFrame(1)]);
        assert_eq!(p.pending(), (0, 0));
    }

    #[test]
    fn extra_frame_is_reported_after_the_window() {
        let t0 = Instant::now();
        let mut p = P::default();
        p.push_shot(1, t0);
        assert_eq!(p.push_frame("a", ms(t0, 20)).len(), 1);
        assert!(p.push_frame("extra", ms(t0, 40)).is_empty());
        assert_eq!(
            p.expire(ms(t0, 2_100)),
            vec![Pairing::FrameWithoutShot("extra")]
        );
    }

    #[test]
    fn interleaving_preserves_fifo_order() {
        let t0 = Instant::now();
        let mut p = P::default();
        assert!(p.push_shot(1, t0).is_empty());
        assert!(p.push_shot(2, ms(t0, 10)).is_empty());
        assert_eq!(
            p.push_frame("a", ms(t0, 40)),
            vec![Pairing::Paired {
                shot: 1,
                frame: "a"
            }]
        );
        assert_eq!(
            p.push_frame("b", ms(t0, 50)),
            vec![Pairing::Paired {
                shot: 2,
                frame: "b"
            }]
        );
        // frame arrives before its shot, then a second shot/frame pair.
        assert!(p.push_frame("c", ms(t0, 400)).is_empty());
        assert_eq!(
            p.push_shot(3, ms(t0, 410)),
            vec![Pairing::Paired {
                shot: 3,
                frame: "c"
            }]
        );
        assert!(p.push_shot(4, ms(t0, 800)).is_empty());
        assert_eq!(
            p.push_frame("d", ms(t0, 830)),
            vec![Pairing::Paired {
                shot: 4,
                frame: "d"
            }]
        );
    }

    #[test]
    fn custom_window() {
        let t0 = Instant::now();
        let mut p = P::new(Duration::from_millis(100));
        p.push_shot(1, t0);
        assert_eq!(p.expire(ms(t0, 101)), vec![Pairing::ShotWithoutFrame(1)]);
    }
}
