//! On-disk format for raw shot captures, so any real shot can be replayed
//! through the pipeline later (`lm-replay frame`).
//!
//! One capture is two files side by side:
//!
//! * `<stem>.pgm`  binary PGM (P5, 8-bit): the raw camera frame
//! * `<stem>.json` metadata: strobe timing and capture time
//!
//! where `<stem>` is `<unix_ms, 13 digits>-<shot_id, 6 digits>`, so a plain
//! directory listing sorts chronologically. The JSON is written last, so a
//! `.json` file always refers to a complete frame.

use std::fs;
use std::path::{Path, PathBuf};

use launch_types::StrobeTiming;
use serde::{Deserialize, Serialize};
use vision::GrayImage;

#[derive(Debug, thiserror::Error)]
pub enum StoreError {
    #[error("io error on {path}: {source}")]
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
    #[error("invalid metadata in {path}: {source}")]
    Metadata {
        path: PathBuf,
        source: serde_json::Error,
    },
    #[error("invalid PGM {path}: {reason}")]
    Pgm { path: PathBuf, reason: String },
}

/// A frame plus everything needed to reprocess it.
#[derive(Debug, Clone, PartialEq)]
pub struct Capture {
    pub timing: StrobeTiming,
    pub frame: GrayImage,
    /// Wall-clock time the frame was captured, milliseconds since the Unix epoch.
    pub captured_at_unix_ms: u64,
}

#[derive(Serialize, Deserialize)]
struct Meta {
    shot_id: u32,
    pulse_times_us: Vec<u32>,
    captured_at_unix_ms: u64,
    width: u32,
    height: u32,
    /// File name of the frame, relative to the metadata file.
    image: String,
}

fn io_err(path: &Path) -> impl FnOnce(std::io::Error) -> StoreError + '_ {
    move |source| StoreError::Io {
        path: path.to_path_buf(),
        source,
    }
}

fn write_atomic(path: &Path, bytes: &[u8]) -> Result<(), StoreError> {
    let tmp = path.with_extension("tmp");
    fs::write(&tmp, bytes).map_err(io_err(&tmp))?;
    fs::rename(&tmp, path).map_err(io_err(path))
}

/// Save a capture into `dir` (created if needed). Returns the metadata path,
/// which is what [`load`] takes.
pub fn save(dir: &Path, capture: &Capture) -> Result<PathBuf, StoreError> {
    fs::create_dir_all(dir).map_err(io_err(dir))?;
    let stem = format!(
        "{:013}-{:06}",
        capture.captured_at_unix_ms, capture.timing.shot_id
    );
    let pgm_name = format!("{stem}.pgm");
    let meta_path = dir.join(format!("{stem}.json"));

    write_atomic(&dir.join(&pgm_name), &encode_pgm(&capture.frame))?;
    let meta = Meta {
        shot_id: capture.timing.shot_id,
        pulse_times_us: capture.timing.pulse_times_us.clone(),
        captured_at_unix_ms: capture.captured_at_unix_ms,
        width: capture.frame.width,
        height: capture.frame.height,
        image: pgm_name,
    };
    let json = serde_json::to_vec_pretty(&meta).map_err(|source| StoreError::Metadata {
        path: meta_path.clone(),
        source,
    })?;
    write_atomic(&meta_path, &json)?;
    Ok(meta_path)
}

/// Load a capture from its metadata (`.json`) path.
pub fn load(meta_path: &Path) -> Result<Capture, StoreError> {
    let bytes = fs::read(meta_path).map_err(io_err(meta_path))?;
    let meta: Meta = serde_json::from_slice(&bytes).map_err(|source| StoreError::Metadata {
        path: meta_path.to_path_buf(),
        source,
    })?;
    // `image` is a bare file name; refuse anything that could escape the directory.
    if Path::new(&meta.image).components().count() != 1 {
        return Err(StoreError::Pgm {
            path: meta_path.to_path_buf(),
            reason: format!("image name {:?} must be a plain file name", meta.image),
        });
    }
    let pgm_path = meta_path.with_file_name(&meta.image);
    let frame = decode_pgm(&fs::read(&pgm_path).map_err(io_err(&pgm_path))?).map_err(|reason| {
        StoreError::Pgm {
            path: pgm_path.clone(),
            reason,
        }
    })?;
    if (frame.width, frame.height) != (meta.width, meta.height) {
        return Err(StoreError::Pgm {
            path: pgm_path,
            reason: format!(
                "metadata says {}x{} but image is {}x{}",
                meta.width, meta.height, frame.width, frame.height
            ),
        });
    }
    Ok(Capture {
        timing: StrobeTiming {
            shot_id: meta.shot_id,
            pulse_times_us: meta.pulse_times_us,
        },
        frame,
        captured_at_unix_ms: meta.captured_at_unix_ms,
    })
}

/// Metadata paths of all captures in `dir`, oldest first.
pub fn list(dir: &Path) -> Result<Vec<PathBuf>, StoreError> {
    let mut paths: Vec<PathBuf> = fs::read_dir(dir)
        .map_err(io_err(dir))?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "json"))
        .collect();
    paths.sort();
    Ok(paths)
}

fn encode_pgm(img: &GrayImage) -> Vec<u8> {
    let mut out = format!("P5\n{} {}\n255\n", img.width, img.height).into_bytes();
    out.extend_from_slice(&img.data);
    out
}

/// Parse a binary 8-bit PGM. Accepts `#` comments in the header.
fn decode_pgm(bytes: &[u8]) -> Result<GrayImage, String> {
    let mut pos = 0;
    let mut token = |bytes: &[u8]| -> Result<String, String> {
        loop {
            match bytes.get(pos) {
                Some(b) if b.is_ascii_whitespace() => pos += 1,
                Some(b'#') => {
                    while bytes.get(pos).is_some_and(|&b| b != b'\n') {
                        pos += 1;
                    }
                }
                Some(_) => break,
                None => return Err("truncated header".into()),
            }
        }
        let start = pos;
        while bytes.get(pos).is_some_and(|b| !b.is_ascii_whitespace()) {
            pos += 1;
        }
        Ok(String::from_utf8_lossy(&bytes[start..pos]).into_owned())
    };
    let magic = token(bytes)?;
    if magic != "P5" {
        return Err(format!("expected P5, found {magic:?}"));
    }
    let num = |t: String, what: &str| t.parse::<u32>().map_err(|_| format!("bad {what}: {t:?}"));
    let width = num(token(bytes)?, "width")?;
    let height = num(token(bytes)?, "height")?;
    let maxval = num(token(bytes)?, "maxval")?;
    if maxval != 255 {
        return Err(format!("only maxval 255 is supported, found {maxval}"));
    }
    // Exactly one whitespace byte separates the header from the pixels.
    pos += 1;
    let expected = (width as usize)
        .checked_mul(height as usize)
        .ok_or("dimensions overflow")?;
    let data = bytes.get(pos..).unwrap_or_default();
    if data.len() != expected {
        return Err(format!(
            "expected {expected} pixel bytes, found {}",
            data.len()
        ));
    }
    GrayImage::from_raw(width, height, data.to_vec()).map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp_dir(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("lm-store-{}-{name}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        dir
    }

    fn sample(shot_id: u32, ms: u64) -> Capture {
        let mut frame = GrayImage::new(7, 5);
        for (i, px) in frame.data.iter_mut().enumerate() {
            *px = (i * 7) as u8;
        }
        Capture {
            timing: StrobeTiming {
                shot_id,
                pulse_times_us: vec![0, 1000, 2000],
            },
            frame,
            captured_at_unix_ms: ms,
        }
    }

    #[test]
    fn round_trip() {
        let dir = temp_dir("rt");
        let cap = sample(42, 1_700_000_000_123);
        let meta = save(&dir, &cap).unwrap();
        assert_eq!(
            meta.file_name().unwrap().to_str().unwrap(),
            "1700000000123-000042.json"
        );
        assert_eq!(load(&meta).unwrap(), cap);
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn list_is_chronological_and_ignores_other_files() {
        let dir = temp_dir("list");
        save(&dir, &sample(2, 2_000)).unwrap();
        save(&dir, &sample(1, 1_000)).unwrap();
        save(&dir, &sample(3, 3_000)).unwrap();
        fs::write(dir.join("notes.txt"), "x").unwrap();
        let ids: Vec<u32> = list(&dir)
            .unwrap()
            .iter()
            .map(|p| load(p).unwrap().timing.shot_id)
            .collect();
        assert_eq!(ids, vec![1, 2, 3]);
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn pgm_header_comments_are_accepted() {
        let img = decode_pgm(b"P5\n# made by hand\n2 2\n255\n\x01\x02\x03\x04").unwrap();
        assert_eq!((img.width, img.height), (2, 2));
        assert_eq!(img.data, vec![1, 2, 3, 4]);
    }

    #[test]
    fn bad_pgm_is_rejected() {
        assert!(decode_pgm(b"P6\n1 1\n255\n\x00").is_err());
        assert!(
            decode_pgm(b"P5\n2 2\n255\n\x00\x00\x00").is_err(),
            "short data"
        );
        assert!(decode_pgm(b"P5\n1 1\n65535\n\x00\x00").is_err(), "16-bit");
        assert!(decode_pgm(b"P5\n1").is_err(), "truncated header");
    }

    #[test]
    fn corrupt_or_hostile_metadata_is_an_error() {
        let dir = temp_dir("bad");
        let meta = save(&dir, &sample(1, 1)).unwrap();

        // Dimension mismatch between metadata and image.
        let text = fs::read_to_string(&meta)
            .unwrap()
            .replace("\"width\": 7", "\"width\": 9");
        fs::write(&meta, text).unwrap();
        assert!(matches!(load(&meta), Err(StoreError::Pgm { .. })));

        // Path traversal in the image name.
        let text = fs::read_to_string(&meta)
            .unwrap()
            .replace("0000000000001-000001.pgm", "../../etc/passwd");
        fs::write(&meta, text).unwrap();
        assert!(matches!(load(&meta), Err(StoreError::Pgm { .. })));

        // Not JSON at all.
        fs::write(&meta, "nope").unwrap();
        assert!(matches!(load(&meta), Err(StoreError::Metadata { .. })));
        fs::remove_dir_all(dir).unwrap();
    }
}
