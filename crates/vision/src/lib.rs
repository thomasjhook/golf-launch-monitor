//! Ball detection for multi-exposure strobe frames.
//!
//! Pixel coordinates follow the OpenCV convention: the centre of pixel
//! `(i, j)` is at `(i, j)`.
//!
//! Algorithm: automatic threshold (median background + Otsu foreground level,
//! with a contrast guard for empty frames) -> 8-connected components ->
//! shape filter (covariance eccentricity + bbox fill) -> sub-pixel refinement
//! per blob using local background subtraction: radius from the integrated
//! flux `sum((I-b)/(F-b)) = pi r^2` (invariant to blur), centre from a
//! half-level-weighted centroid. Blobs that fail the shape test are tried as
//! merged (touching) balls: distance-transform peaks + radical-axis
//! partition + Kasa circle fit on the free outer boundary.
//!
//! Limits: blobs touching the image border are rejected; balls overlapping
//! deeper than ~ 0.8 of a diameter of centre separation are not split;
//! split balls are less accurate (~0.5-1 px) with confidence capped at 0.4;
//! a hot pixel touching a ball nudges its centre slightly.

use launch_types::BallDetection;

pub mod synth;

#[derive(Debug, thiserror::Error)]
pub enum VisionError {
    #[error("invalid image dimensions")]
    InvalidDimensions,
    #[error("data length {actual} does not match width*height {expected}")]
    DataLengthMismatch { expected: usize, actual: usize },
}

#[derive(Debug, Clone, PartialEq)]
pub struct GrayImage {
    pub width: u32,
    pub height: u32,
    /// Row-major, `len == width * height`.
    pub data: Vec<u8>,
}

impl GrayImage {
    /// Black image. 0-sized images are allowed (and detect nothing).
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            width,
            height,
            data: vec![0; width as usize * height as usize],
        }
    }

    pub fn from_raw(width: u32, height: u32, data: Vec<u8>) -> Result<Self, VisionError> {
        if width == 0 || height == 0 {
            return Err(VisionError::InvalidDimensions);
        }
        let expected = (width as u64 * height as u64) as usize;
        if data.len() != expected {
            return Err(VisionError::DataLengthMismatch {
                expected,
                actual: data.len(),
            });
        }
        Ok(Self {
            width,
            height,
            data,
        })
    }

    /// Pixel value; out-of-range coordinates return 0.
    pub fn get(&self, x: u32, y: u32) -> u8 {
        if x >= self.width || y >= self.height {
            return 0;
        }
        self.data
            .get(y as usize * self.width as usize + x as usize)
            .copied()
            .unwrap_or(0)
    }

    /// Set a pixel; out-of-range coordinates are ignored.
    pub fn set(&mut self, x: u32, y: u32, v: u8) {
        if x >= self.width || y >= self.height {
            return;
        }
        if let Some(p) = self
            .data
            .get_mut(y as usize * self.width as usize + x as usize)
        {
            *p = v;
        }
    }
}

#[derive(Debug, Clone)]
pub struct DetectParams {
    /// `None` = automatic. Pixels strictly greater than the value are foreground.
    pub threshold: Option<u8>,
    pub min_radius_px: f64,
    pub max_radius_px: f64,
    /// 0..1; eccentricity/fill based shape score a blob must reach.
    pub min_circularity: f64,
    pub max_balls: usize,
}

impl Default for DetectParams {
    fn default() -> Self {
        Self {
            threshold: None,
            min_radius_px: 6.0,
            max_radius_px: 120.0,
            min_circularity: 0.7,
            max_balls: 8,
        }
    }
}

const MIN_CONTRAST: f64 = 48.0;
const PI: f64 = std::f64::consts::PI;

fn auto_threshold(img: &GrayImage) -> Option<u8> {
    let mut hist = [0u64; 256];
    for &v in &img.data {
        hist[v as usize] += 1;
    }
    let total: u64 = hist.iter().sum();
    if total == 0 {
        return None;
    }
    let mut cum = 0;
    let mut median = 0usize;
    for (v, &c) in hist.iter().enumerate() {
        cum += c;
        if cum * 2 >= total {
            median = v;
            break;
        }
    }
    let maxv = hist.iter().rposition(|&c| c > 0)?;
    if (maxv as f64 - median as f64) < MIN_CONTRAST {
        return None;
    }
    // Otsu, taking the middle of the plateau of maxima.
    let sum_all: f64 = hist
        .iter()
        .enumerate()
        .map(|(v, &c)| v as f64 * c as f64)
        .sum();
    let (mut w0, mut s0) = (0.0f64, 0.0f64);
    let mut between = [0.0f64; 256];
    let mut best = 0.0f64;
    for t in 0..255 {
        w0 += hist[t] as f64;
        s0 += t as f64 * hist[t] as f64;
        let w1 = total as f64 - w0;
        if w0 > 0.0 && w1 > 0.0 {
            let d = s0 / w0 - (sum_all - s0) / w1;
            between[t] = w0 * w1 * d * d;
            best = best.max(between[t]);
        }
    }
    let first = between.iter().position(|&b| b >= best * 0.999_999)?;
    let last = between.iter().rposition(|&b| b >= best * 0.999_999)?;
    let t_otsu = (first + last) / 2;
    let (mut n, mut s) = (0.0, 0.0);
    for (v, &count) in hist.iter().enumerate().skip(t_otsu + 1) {
        n += count as f64;
        s += v as f64 * count as f64;
    }
    if n == 0.0 {
        return None;
    }
    let fg = s / n;
    if fg - (median as f64) < MIN_CONTRAST * 0.8 {
        return None;
    }
    Some(((median as f64 + fg) / 2.0).round().clamp(0.0, 254.0) as u8)
}

struct Ctx<'a> {
    img: &'a GrayImage,
    w: usize,
    h: usize,
    labels: Vec<u32>,
    p: &'a DetectParams,
}

/// Detect ball images. Sorted by centre x ascending.
pub fn detect_balls(img: &GrayImage, params: &DetectParams) -> Vec<BallDetection> {
    let (w, h) = (img.width as usize, img.height as usize);
    if w == 0 || h == 0 || img.data.len() != w * h {
        return Vec::new();
    }
    let thr = match params.threshold {
        Some(t) => t,
        None => match auto_threshold(img) {
            Some(t) => t,
            None => return Vec::new(),
        },
    };
    // Connected components, 8-connectivity.
    let mut labels = vec![0u32; w * h];
    let mut comps: Vec<Vec<usize>> = Vec::new();
    let mut stack = Vec::new();
    for start in 0..w * h {
        if img.data[start] <= thr || labels[start] != 0 {
            continue;
        }
        let id = comps.len() as u32 + 1;
        let mut pix = Vec::new();
        labels[start] = id;
        stack.push(start);
        while let Some(i) = stack.pop() {
            pix.push(i);
            let (x, y) = ((i % w) as isize, (i / w) as isize);
            for dy in -1..=1isize {
                for dx in -1..=1isize {
                    let (nx, ny) = (x + dx, y + dy);
                    if nx < 0 || ny < 0 || nx >= w as isize || ny >= h as isize {
                        continue;
                    }
                    let j = ny as usize * w + nx as usize;
                    if img.data[j] > thr && labels[j] == 0 {
                        labels[j] = id;
                        stack.push(j);
                    }
                }
            }
        }
        comps.push(pix);
    }
    let min_area = (0.5 * PI * params.min_radius_px * params.min_radius_px).max(4.0);
    let ctx = Ctx {
        img,
        w,
        h,
        labels,
        p: params,
    };
    let mut out: Vec<BallDetection> = Vec::new();
    for (k, pix) in comps.iter().enumerate() {
        if (pix.len() as f64) < min_area {
            continue;
        }
        if pix.iter().any(|&i| {
            let (x, y) = (i % w, i / w);
            x == 0 || y == 0 || x == w - 1 || y == h - 1
        }) {
            continue; // clipped by the border
        }
        out.extend(analyze(&ctx, k as u32 + 1, pix));
    }
    out.sort_by(|a, b| b.confidence.total_cmp(&a.confidence));
    out.truncate(params.max_balls);
    out.sort_by(|a, b| a.center_px[0].total_cmp(&b.center_px[0]));
    out
}

struct Shape {
    cx: f64,
    cy: f64,
    circ: f64,
}

fn shape(pix: &[usize], w: usize) -> Shape {
    let n = pix.len() as f64;
    let (mut sx, mut sy) = (0.0, 0.0);
    let (mut x0, mut x1, mut y0, mut y1) = (usize::MAX, 0, usize::MAX, 0);
    for &i in pix {
        let (x, y) = (i % w, i / w);
        sx += x as f64;
        sy += y as f64;
        x0 = x0.min(x);
        x1 = x1.max(x);
        y0 = y0.min(y);
        y1 = y1.max(y);
    }
    let (cx, cy) = (sx / n, sy / n);
    let (mut a, mut b, mut c) = (0.0, 0.0, 0.0);
    for &i in pix {
        let (dx, dy) = ((i % w) as f64 - cx, (i / w) as f64 - cy);
        a += dx * dx;
        b += dy * dy;
        c += dx * dy;
    }
    let (a, b, c) = (a / n, b / n, c / n);
    let disc = (((a - b) / 2.0).powi(2) + c * c).sqrt();
    let (l1, l2) = ((a + b) / 2.0 + disc, (a + b) / 2.0 - disc);
    let ecc = if l1 > 0.0 {
        (l2.max(0.0) / l1).sqrt()
    } else {
        0.0
    };
    let bw = (x1 - x0 + 1) as f64;
    let bh = (y1 - y0 + 1) as f64;
    let extent = n / (PI / 4.0 * bw * bh);
    let extent_score = (1.0 - (extent - 0.97).abs() * 3.0).clamp(0.0, 1.0);
    Shape {
        cx,
        cy,
        circ: ecc.min(extent_score),
    }
}

fn analyze(ctx: &Ctx, label: u32, pix: &[usize]) -> Vec<BallDetection> {
    let sh = shape(pix, ctx.w);
    if sh.circ >= ctx.p.min_circularity {
        return refine(ctx, label, pix, &sh).into_iter().collect();
    }
    split(ctx, pix)
}

fn median(v: &mut [f64]) -> f64 {
    v.sort_by(|a, b| a.total_cmp(b));
    v[v.len() / 2]
}

fn refine(ctx: &Ctx, label: u32, pix: &[usize], sh: &Shape) -> Option<BallDetection> {
    let (w, h) = (ctx.w, ctx.h);
    let img = &ctx.img.data;
    let (mut x0, mut x1, mut y0, mut y1) = (usize::MAX, 0, usize::MAX, 0);
    for &i in pix {
        x0 = x0.min(i % w);
        x1 = x1.max(i % w);
        y0 = y0.min(i / w);
        y1 = y1.max(i / w);
    }
    let clampx = |v: isize| v.clamp(0, w as isize - 1) as usize;
    let clampy = |v: isize| v.clamp(0, h as isize - 1) as usize;
    // Background: median of unlabelled pixels in a ring around the blob.
    let (rx0, rx1) = (clampx(x0 as isize - 10), clampx(x1 as isize + 10));
    let (ry0, ry1) = (clampy(y0 as isize - 10), clampy(y1 as isize + 10));
    let mut ring = Vec::new();
    for y in ry0..=ry1 {
        for x in rx0..=rx1 {
            let inside = x + 5 >= x0 && x <= x1 + 5 && y + 5 >= y0 && y <= y1 + 5;
            if !inside && ctx.labels[y * w + x] == 0 {
                ring.push(img[y * w + x] as f64);
            }
        }
    }
    let bg = if ring.len() >= 8 {
        median(&mut ring)
    } else {
        let mut all: Vec<f64> = img.iter().map(|&v| v as f64).collect();
        median(&mut all)
    };
    // Rough centre / radius from the mask.
    let r0 = (pix.len() as f64 / PI).sqrt();
    // Window = bbox + margin, excluding other blobs.
    let (wx0, wx1) = (clampx(x0 as isize - 5), clampx(x1 as isize + 5));
    let (wy0, wy1) = (clampy(y0 as isize - 5), clampy(y1 as isize + 5));
    let ok = |i: usize| ctx.labels[i] == 0 || ctx.labels[i] == label;
    // Foreground level: mean inside 0.5 r0 of the mask centroid.
    let (mut fs, mut fnn) = (0.0, 0.0);
    for y in wy0..=wy1 {
        for x in wx0..=wx1 {
            let (dx, dy) = (x as f64 - sh.cx, y as f64 - sh.cy);
            if dx * dx + dy * dy <= 0.25 * r0 * r0 && ok(y * w + x) {
                fs += img[y * w + x] as f64;
                fnn += 1.0;
            }
        }
    }
    if fnn == 0.0 {
        return None;
    }
    let f = fs / fnn - bg;
    if f < 20.0 {
        return None;
    }
    let (mut flux, mut sw, mut sx, mut sy) = (0.0, 0.0, 0.0, 0.0);
    for y in wy0..=wy1 {
        for x in wx0..=wx1 {
            let i = y * w + x;
            if !ok(i) {
                continue;
            }
            let v = (img[i] as f64 - bg) / f;
            flux += v;
            let wt = (v - 0.5).max(0.0);
            sw += wt;
            sx += wt * x as f64;
            sy += wt * y as f64;
        }
    }
    if sw <= 0.0 || flux <= 0.0 {
        return None;
    }
    let (cx, cy) = (sx / sw, sy / sw);
    let r = (flux / PI).sqrt();
    if r < ctx.p.min_radius_px || r > ctx.p.max_radius_px {
        return None;
    }
    // Residual: radial scatter of the mask boundary about the fitted centre.
    let mut d = Vec::new();
    for &i in pix {
        let (x, y) = (i % w, i / w);
        let edge = [(1i32, 0i32), (-1, 0), (0, 1), (0, -1)]
            .iter()
            .any(|&(dx, dy)| {
                let j = (y as i32 + dy) as usize * w + (x as i32 + dx) as usize;
                ctx.labels[j] != label
            });
        if edge {
            d.push(((x as f64 - cx).powi(2) + (y as f64 - cy).powi(2)).sqrt());
        }
    }
    let mean = d.iter().sum::<f64>() / d.len().max(1) as f64;
    let rms = (d.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / d.len().max(1) as f64).sqrt();
    let conf = sh.circ.clamp(0.0, 1.0) * (1.0 - 3.0 * rms / mean.max(1.0)).clamp(0.0, 1.0);
    Some(BallDetection {
        center_px: [cx, cy],
        radius_px: r,
        confidence: conf,
    })
}

/// Try to split a non-round blob into touching discs.
fn split(ctx: &Ctx, pix: &[usize]) -> Vec<BallDetection> {
    let w = ctx.w;
    let (mut x0, mut x1, mut y0, mut y1) = (usize::MAX, 0, usize::MAX, 0);
    for &i in pix {
        x0 = x0.min(i % w);
        x1 = x1.max(i % w);
        y0 = y0.min(i / w);
        y1 = y1.max(i / w);
    }
    let (gw, gh) = (x1 - x0 + 3, y1 - y0 + 3);
    let gi = |i: usize| ((i / w) - y0 + 1) * gw + (i % w) - x0 + 1;
    let mut mask = vec![false; gw * gh];
    for &i in pix {
        mask[gi(i)] = true;
    }
    // Chamfer 3-4 distance transform.
    let inf = 1.0e9f64;
    let mut dt: Vec<f64> = mask.iter().map(|&m| if m { inf } else { 0.0 }).collect();
    for y in 1..gh - 1 {
        for x in 1..gw - 1 {
            let i = y * gw + x;
            if mask[i] {
                dt[i] = dt[i]
                    .min(dt[i - 1] + 3.0)
                    .min(dt[i - gw] + 3.0)
                    .min(dt[i - gw - 1] + 4.0)
                    .min(dt[i - gw + 1] + 4.0);
            }
        }
    }
    for y in (1..gh - 1).rev() {
        for x in (1..gw - 1).rev() {
            let i = y * gw + x;
            if mask[i] {
                dt[i] = dt[i]
                    .min(dt[i + 1] + 3.0)
                    .min(dt[i + gw] + 3.0)
                    .min(dt[i + gw + 1] + 4.0)
                    .min(dt[i + gw - 1] + 4.0);
            }
        }
    }
    dt.iter_mut().for_each(|v| *v /= 3.0);
    // Local maxima as seed candidates.
    let mut cands: Vec<(f64, f64, f64)> = Vec::new(); // (dt, x, y) grid coords
    for y in 1..gh - 1 {
        for x in 1..gw - 1 {
            let i = y * gw + x;
            if !mask[i] || dt[i] - 0.5 < 0.8 * ctx.p.min_radius_px {
                continue;
            }
            let mut is_max = true;
            for dy in -1..=1isize {
                for dx in -1..=1isize {
                    let j = (y as isize + dy) as usize * gw + (x as isize + dx) as usize;
                    if dt[j] > dt[i] {
                        is_max = false;
                    }
                }
            }
            if is_max {
                cands.push((dt[i], x as f64, y as f64));
            }
        }
    }
    cands.sort_by(|a, b| b.0.total_cmp(&a.0));
    let mut seeds: Vec<(f64, f64, f64)> = Vec::new();
    'c: for c in cands {
        for s in &seeds {
            let d = ((c.1 - s.1).powi(2) + (c.2 - s.2).powi(2)).sqrt();
            if d < 0.8 * s.0 {
                continue 'c;
            }
            // Neck test along the segment.
            let steps = (d * 2.0).ceil().max(2.0) as usize;
            let mut minv = f64::MAX;
            for k in 0..=steps {
                let t = k as f64 / steps as f64;
                let px = (s.1 + (c.1 - s.1) * t).round() as usize;
                let py = (s.2 + (c.2 - s.2) * t).round() as usize;
                minv = minv.min(dt[py * gw + px]);
            }
            if minv > 0.88 * c.0 {
                continue 'c;
            }
        }
        seeds.push(c);
        if seeds.len() >= ctx.p.max_balls {
            break;
        }
    }
    if seeds.len() < 2 {
        return Vec::new();
    }
    // Partition by radical axis; fit circles to the free outer boundary.
    let rs: Vec<f64> = seeds.iter().map(|s| s.0 - 0.5).collect();
    let mut parts: Vec<Vec<(f64, f64)>> = vec![Vec::new(); seeds.len()];
    for &i in pix {
        let g = gi(i);
        let (x, y) = ((g % gw) as f64, (g / gw) as f64);
        let edge = [1isize, -1, gw as isize, -(gw as isize)]
            .iter()
            .any(|&o| !mask[(g as isize + o) as usize]);
        if !edge {
            continue;
        }
        let k = (0..seeds.len())
            .min_by(|&a, &b| {
                let da = (x - seeds[a].1).powi(2) + (y - seeds[a].2).powi(2) - rs[a] * rs[a];
                let db = (x - seeds[b].1).powi(2) + (y - seeds[b].2).powi(2) - rs[b] * rs[b];
                da.total_cmp(&db)
            })
            .unwrap_or(0);
        // Free boundary: outside every other seed's circle.
        let free = (0..seeds.len()).all(|j| {
            j == k || ((x - seeds[j].1).powi(2) + (y - seeds[j].2).powi(2)).sqrt() > rs[j] + 1.5
        });
        if free {
            parts[k].push((x, y));
        }
    }
    let mut out = Vec::new();
    for (k, pts) in parts.iter().enumerate() {
        let (mut cx, mut cy, mut r) = (seeds[k].1, seeds[k].2, rs[k]);
        if pts.len() >= 8 {
            if let Some((fx, fy, fr)) = kasa(pts) {
                if (fx - cx).hypot(fy - cy) < 0.5 * rs[k] && (fr - r).abs() < 0.3 * r {
                    cx = fx;
                    cy = fy;
                    r = fr + 0.4; // boundary pixels sit ~0.4 px inside the edge
                }
            }
        }
        if r < ctx.p.min_radius_px || r > ctx.p.max_radius_px {
            continue;
        }
        out.push(BallDetection {
            center_px: [cx + x0 as f64 - 1.0, cy + y0 as f64 - 1.0],
            radius_px: r,
            confidence: 0.4,
        });
    }
    out
}

/// Algebraic (Kasa) circle fit.
fn kasa(pts: &[(f64, f64)]) -> Option<(f64, f64, f64)> {
    let n = pts.len() as f64;
    let mx = pts.iter().map(|p| p.0).sum::<f64>() / n;
    let my = pts.iter().map(|p| p.1).sum::<f64>() / n;
    let (mut suu, mut suv, mut svv, mut suuu, mut svvv, mut suvv, mut svuu) =
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for &(x, y) in pts {
        let (u, v) = (x - mx, y - my);
        suu += u * u;
        suv += u * v;
        svv += v * v;
        suuu += u * u * u;
        svvv += v * v * v;
        suvv += u * v * v;
        svuu += v * u * u;
    }
    let det = suu * svv - suv * suv;
    if det.abs() < 1e-9 {
        return None;
    }
    let (a, b) = ((suuu + suvv) / 2.0, (svvv + svuu) / 2.0);
    let uc = (a * svv - b * suv) / det;
    let vc = (b * suu - a * suv) / det;
    let r = (uc * uc + vc * vc + (suu + svv) / n).sqrt();
    Some((uc + mx, vc + my, r))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synth::{render, SynthBall, SynthParams};

    fn ball(x: f64, y: f64, r: f64) -> SynthBall {
        SynthBall {
            center_px: [x, y],
            radius_px: r,
            brightness: 220,
        }
    }

    fn check(dets: &[BallDetection], truth: &[SynthBall], ctol: f64, rtol: f64) -> (f64, f64) {
        assert_eq!(dets.len(), truth.len(), "count: {dets:?}");
        let (mut mc, mut mr) = (0.0f64, 0.0f64);
        for (d, t) in dets.iter().zip(truth) {
            let ce = (d.center_px[0] - t.center_px[0]).hypot(d.center_px[1] - t.center_px[1]);
            let re = (d.radius_px - t.radius_px).abs();
            assert!(ce < ctol, "centre err {ce} for {t:?} got {d:?}");
            assert!(re < rtol, "radius err {re} for {t:?} got {d:?}");
            mc = mc.max(ce);
            mr = mr.max(re);
        }
        (mc, mr)
    }

    #[test]
    fn clean_single_and_triple() {
        let p = DetectParams::default();
        let (mut mc, mut mr) = (0.0f64, 0.0f64);
        for (i, &r) in [8.0, 12.3, 20.0, 35.7, 60.0].iter().enumerate() {
            let off = 0.13 + 0.19 * i as f64;
            let one = [ball(300.0 + off, 200.0 + 0.5 - off, r)];
            let img = render(640, 480, &one, &SynthParams::default());
            let (c, rr) = check(&detect_balls(&img, &p), &one, 0.1, 0.15);
            mc = mc.max(c);
            mr = mr.max(rr);
            let tri = [
                ball(120.0 + off, 200.0 + 0.3, r),
                ball(320.0 + off * 0.5, 240.0 + off, r),
                ball(520.0 - off, 280.0 + 0.7, r),
            ];
            let img = render(640, 480, &tri, &SynthParams::default());
            let (c, rr) = check(&detect_balls(&img, &p), &tri, 0.1, 0.15);
            mc = mc.max(c);
            mr = mr.max(rr);
        }
        println!("clean: max centre err {mc:.4} px, max radius err {mr:.4} px");
    }

    #[test]
    fn noisy_frames() {
        let p = DetectParams::default();
        let (mut mc, mut mr) = (0.0f64, 0.0f64);
        for (seed, &(sigma, bg, r)) in [
            (4.0, 10u8, 10.0),
            (6.0, 15, 25.0),
            (8.0, 20, 14.5),
            (8.0, 20, 8.0),
            (5.0, 12, 40.0),
        ]
        .iter()
        .enumerate()
        {
            let truth = [
                ball(110.3, 200.6, r),
                ball(310.7, 230.2, r),
                ball(510.1, 260.9, r),
            ];
            let sp = SynthParams {
                noise_sigma: sigma,
                background: bg,
                seed: seed as u64 + 7,
                blur_sigma_px: 1.0,
                hot_pixels: 8,
            };
            let img = render(640, 480, &truth, &sp);
            let (c, rr) = check(&detect_balls(&img, &p), &truth, 0.3, 0.3);
            mc = mc.max(c);
            mr = mr.max(rr);
        }
        println!("noisy: max centre err {mc:.4} px, max radius err {mr:.4} px");
    }

    #[test]
    fn empty_and_noise_only() {
        let p = DetectParams::default();
        assert!(detect_balls(&GrayImage::new(640, 480), &p).is_empty());
        for sigma in [4.0, 8.0] {
            let sp = SynthParams {
                noise_sigma: sigma,
                background: 15,
                seed: 3,
                blur_sigma_px: 0.0,
                hot_pixels: 5,
            };
            assert!(detect_balls(&render(640, 480, &[], &sp), &p).is_empty());
            let sp = SynthParams {
                blur_sigma_px: 1.0,
                ..sp
            };
            assert!(detect_balls(&render(640, 480, &[], &sp), &p).is_empty());
        }
    }

    #[test]
    fn border_and_degenerate() {
        let p = DetectParams::default();
        let b = [ball(3.0, 240.0, 20.0), ball(320.0, 240.0, 20.0)];
        let img = render(640, 480, &b, &SynthParams::default());
        let d = detect_balls(&img, &p); // clipped ball is rejected
        assert_eq!(d.len(), 1);
        assert!((d[0].center_px[0] - 320.0).abs() < 0.1);
        for (w, h) in [(0, 0), (1, 1), (0, 5), (5, 0), (3, 2)] {
            assert!(detect_balls(&GrayImage::new(w, h), &p).is_empty());
        }
        let mut one = GrayImage::new(1, 1);
        one.set(0, 0, 255);
        assert!(detect_balls(&one, &p).is_empty());
        one.set(5, 5, 1);
        assert_eq!(one.get(5, 5), 0);
        let _ = render(0, 0, &b, &SynthParams::default());
        let _ = render(
            10,
            10,
            &b,
            &SynthParams {
                hot_pixels: 3,
                ..Default::default()
            },
        );
    }

    #[test]
    fn from_raw_errors() {
        assert!(matches!(
            GrayImage::from_raw(0, 4, vec![]),
            Err(VisionError::InvalidDimensions)
        ));
        assert!(matches!(
            GrayImage::from_raw(4, 0, vec![]),
            Err(VisionError::InvalidDimensions)
        ));
        assert!(matches!(
            GrayImage::from_raw(2, 2, vec![0; 3]),
            Err(VisionError::DataLengthMismatch { .. })
        ));
        assert!(GrayImage::from_raw(2, 2, vec![0; 4]).is_ok());
    }

    #[test]
    fn touching_balls_are_split() {
        let p = DetectParams::default();
        let truth = [ball(300.0, 240.0, 20.0), ball(331.0, 240.0, 20.0)];
        let img = render(640, 480, &truth, &SynthParams::default());
        let d = detect_balls(&img, &p);
        println!("touching: {d:?}");
        assert_eq!(d.len(), 2);
        for (a, t) in d.iter().zip(&truth) {
            assert!((a.center_px[0] - t.center_px[0]).abs() < 2.0, "{a:?}");
            assert!((a.radius_px - 20.0).abs() < 2.0, "{a:?}");
            assert!(a.confidence <= 0.4);
        }
    }

    #[test]
    fn max_balls_and_explicit_threshold() {
        let truth = [
            ball(100.0, 100.0, 15.0),
            ball(200.0, 100.0, 15.0),
            ball(300.0, 100.0, 15.0),
        ];
        let img = render(400, 200, &truth, &SynthParams::default());
        let p = DetectParams {
            max_balls: 2,
            threshold: Some(100),
            ..Default::default()
        };
        let d = detect_balls(&img, &p);
        assert_eq!(d.len(), 2);
        assert!(d[0].center_px[0] < d[1].center_px[0]);
    }
}
