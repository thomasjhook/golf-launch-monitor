//! Synthetic frame generator (test / dev helper).
//!
//! Pixel centres are at integer coordinates (OpenCV convention), i.e. pixel
//! `(i, j)` covers `[i-0.5, i+0.5] x [j-0.5, j+0.5]`.

use crate::GrayImage;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SynthBall {
    pub center_px: [f64; 2],
    pub radius_px: f64,
    pub brightness: u8,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SynthParams {
    /// Std-dev of additive gaussian noise in gray levels (0 = none).
    pub noise_sigma: f64,
    /// Uniform background level.
    pub background: u8,
    pub seed: u64,
    /// Gaussian blur sigma in pixels (0 = none).
    pub blur_sigma_px: f64,
    /// Number of single saturated (255) pixels placed at random.
    pub hot_pixels: usize,
}

impl Default for SynthParams {
    fn default() -> Self {
        Self {
            noise_sigma: 0.0,
            background: 0,
            seed: 1,
            blur_sigma_px: 0.0,
            hot_pixels: 0,
        }
    }
}

struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ 0xD1B5_4A32_D192_ED03 | 1)
    }
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    /// Uniform in (0, 1).
    fn uniform(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }
    fn gaussian(&mut self) -> f64 {
        let u1 = self.uniform();
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

const SS: usize = 8;

fn coverage(px: f64, py: f64, b: &SynthBall) -> f64 {
    let dx = px - b.center_px[0];
    let dy = py - b.center_px[1];
    let d = (dx * dx + dy * dy).sqrt();
    let r = b.radius_px;
    if d <= r - 0.75 {
        return 1.0;
    }
    if d >= r + 0.75 {
        return 0.0;
    }
    let mut n = 0;
    for sy in 0..SS {
        let y = py - 0.5 + (sy as f64 + 0.5) / SS as f64 - b.center_px[1];
        for sx in 0..SS {
            let x = px - 0.5 + (sx as f64 + 0.5) / SS as f64 - b.center_px[0];
            if x * x + y * y <= r * r {
                n += 1;
            }
        }
    }
    n as f64 / (SS * SS) as f64
}

fn blur(buf: &mut [f64], w: usize, h: usize, sigma: f64) {
    let rad = (3.0 * sigma).ceil() as isize;
    let mut k: Vec<f64> = (-rad..=rad)
        .map(|i| (-(i * i) as f64 / (2.0 * sigma * sigma)).exp())
        .collect();
    let s: f64 = k.iter().sum();
    k.iter_mut().for_each(|v| *v /= s);
    let mut tmp = vec![0.0; buf.len()];
    for y in 0..h {
        for x in 0..w {
            let mut acc = 0.0;
            for (i, kv) in k.iter().enumerate() {
                let xx = (x as isize + i as isize - rad).clamp(0, w as isize - 1) as usize;
                acc += kv * buf[y * w + xx];
            }
            tmp[y * w + x] = acc;
        }
    }
    for y in 0..h {
        for x in 0..w {
            let mut acc = 0.0;
            for (i, kv) in k.iter().enumerate() {
                let yy = (y as isize + i as isize - rad).clamp(0, h as isize - 1) as usize;
                acc += kv * tmp[yy * w + x];
            }
            buf[y * w + x] = acc;
        }
    }
}

/// Render anti-aliased discs on a flat background, optionally blurred, with
/// deterministic gaussian noise and hot pixels.
pub fn render(width: u32, height: u32, balls: &[SynthBall], p: &SynthParams) -> GrayImage {
    let (w, h) = (width as usize, height as usize);
    let mut buf = vec![p.background as f64; w * h];
    for b in balls {
        let m = b.radius_px + 2.0;
        let x0 = (b.center_px[0] - m).floor().max(0.0) as usize;
        let y0 = (b.center_px[1] - m).floor().max(0.0) as usize;
        let x1 = (b.center_px[0] + m).ceil().min(w as f64 - 1.0);
        let y1 = (b.center_px[1] + m).ceil().min(h as f64 - 1.0);
        if x1 < 0.0 || y1 < 0.0 {
            continue;
        }
        for y in y0..=(y1 as usize) {
            for x in x0..=(x1 as usize) {
                let c = coverage(x as f64, y as f64, b);
                if c > 0.0 {
                    let v = &mut buf[y * w + x];
                    *v += (b.brightness as f64 - *v) * c;
                }
            }
        }
    }
    if p.blur_sigma_px > 0.0 && w > 0 && h > 0 {
        blur(&mut buf, w, h, p.blur_sigma_px);
    }
    let mut rng = Rng::new(p.seed);
    if p.noise_sigma > 0.0 {
        for v in buf.iter_mut() {
            *v += p.noise_sigma * rng.gaussian();
        }
    }
    let mut data: Vec<u8> = buf
        .iter()
        .map(|v| v.round().clamp(0.0, 255.0) as u8)
        .collect();
    if w > 0 && h > 0 {
        for _ in 0..p.hot_pixels {
            let x = (rng.next_u64() % w as u64) as usize;
            let y = (rng.next_u64() % h as u64) as usize;
            data[y * w + x] = 255;
        }
    }
    GrayImage {
        width,
        height,
        data,
    }
}
