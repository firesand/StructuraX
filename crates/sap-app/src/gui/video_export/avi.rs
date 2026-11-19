//! Uncompressed AVI (RIFF) and BMP writers.
//!
//! Layout (Microsoft AVI RIFF file reference): `RIFF 'AVI '` { `LIST 'hdrl'` {
//! `avih` (56 bytes), `LIST 'strl'` { `strh` (56), `strf` = BITMAPINFOHEADER (40) } },
//! `LIST 'movi'` { `00db` chunks }, `idx1` (16 bytes per frame) }, little endian.
//! Pixels are 24 bit BGR, BI_RGB, bottom-up, each row padded to a multiple of four bytes.
//! The file has no OpenDML index, so it is limited to 1 GiB.

use super::RgbImage;

pub(super) const MIN_SIDE: usize = 2;
pub(super) const MAX_SIDE: usize = 4096;
pub(super) const MIN_FPS: u32 = 1;
pub(super) const MAX_FPS: u32 = 120;
pub(super) const MAX_FILE_BYTES: u64 = 1 << 30;

/// Bytes of one padded bottom-up row of 24 bit pixels.
pub(super) fn row_stride(width: usize) -> usize {
    (width * 3).div_ceil(4) * 4
}

/// Bytes of one frame (`00db` payload).
pub(super) fn frame_bytes(width: usize, height: usize) -> usize {
    row_stride(width) * height
}

/// Size of the finished file for `frames` frames of `width x height`.
pub(super) fn file_bytes(width: usize, height: usize, frames: usize) -> u64 {
    // RIFF header 12, hdrl LIST 8 + 192, movi LIST header 12, frames, idx1 8 + 16 n.
    let frame = frame_bytes(width, height) as u64;
    let n = frames as u64;
    12 + 200 + 12 + n * (8 + frame) + 8 + 16 * n
}

fn push_u16(v: &mut Vec<u8>, x: u16) {
    v.extend_from_slice(&x.to_le_bytes());
}
fn push_u32(v: &mut Vec<u8>, x: u32) {
    v.extend_from_slice(&x.to_le_bytes());
}

/// 24 bit BGR, bottom-up, padded rows.
fn dib_rows(image: &RgbImage, out: &mut Vec<u8>) {
    let stride = row_stride(image.width);
    for y in (0..image.height).rev() {
        let start = out.len();
        for p in &image.pixels[y * image.width..(y + 1) * image.width] {
            out.extend_from_slice(&[p[2], p[1], p[0]]);
        }
        out.resize(start + stride, 0);
    }
}

fn check_image(image: &RgbImage) -> Result<(), String> {
    if image.pixels.len() != image.width * image.height {
        return Err(format!(
            "image has {} pixels but is declared {} x {}",
            image.pixels.len(),
            image.width,
            image.height
        ));
    }
    Ok(())
}

/// Collects frames and writes the AVI; keeps only the encoded frames.
pub(super) struct AviBuilder {
    width: usize,
    height: usize,
    fps: u32,
    movi: Vec<u8>,
    frames: u32,
}

impl AviBuilder {
    pub(super) fn new(width: usize, height: usize, fps: u32) -> Result<Self, String> {
        if !(MIN_SIDE..=MAX_SIDE).contains(&width) || !(MIN_SIDE..=MAX_SIDE).contains(&height) {
            return Err(format!(
                "frame size {width} x {height} pixels is outside {MIN_SIDE}..{MAX_SIDE} per side"
            ));
        }
        if !(MIN_FPS..=MAX_FPS).contains(&fps) {
            return Err(format!(
                "{fps} frames per second is outside {MIN_FPS}..{MAX_FPS}"
            ));
        }
        Ok(Self {
            width,
            height,
            fps,
            movi: Vec::new(),
            frames: 0,
        })
    }

    pub(super) fn push(&mut self, image: &RgbImage) -> Result<(), String> {
        check_image(image)?;
        if image.width != self.width || image.height != self.height {
            return Err(format!(
                "frame {} is {} x {} but the video is {} x {}",
                self.frames + 1,
                image.width,
                image.height,
                self.width,
                self.height
            ));
        }
        if file_bytes(self.width, self.height, self.frames as usize + 1) > MAX_FILE_BYTES {
            return Err(format!(
                "the video would exceed 1 GiB at frame {} (no OpenDML index is written); \
                 use a smaller frame size or fewer frames",
                self.frames + 1
            ));
        }
        let size = frame_bytes(self.width, self.height);
        self.movi.extend_from_slice(b"00db");
        push_u32(&mut self.movi, size as u32);
        dib_rows(image, &mut self.movi);
        self.frames += 1;
        Ok(())
    }

    pub(super) fn finish(self) -> Result<Vec<u8>, String> {
        if self.frames == 0 {
            return Err("a video needs at least one frame".into());
        }
        let n = self.frames;
        let frame = frame_bytes(self.width, self.height) as u32;
        let mut out = Vec::with_capacity(self.movi.len() + 1024);
        out.extend_from_slice(b"RIFF");
        push_u32(&mut out, 0); // patched below
        out.extend_from_slice(b"AVI ");
        // hdrl
        out.extend_from_slice(b"LIST");
        push_u32(&mut out, 192);
        out.extend_from_slice(b"hdrl");
        out.extend_from_slice(b"avih");
        push_u32(&mut out, 56);
        push_u32(&mut out, 1_000_000 / self.fps); // dwMicroSecPerFrame
        push_u32(&mut out, frame.saturating_mul(self.fps)); // dwMaxBytesPerSec
        push_u32(&mut out, 0); // dwPaddingGranularity
        push_u32(&mut out, 0x10); // dwFlags: AVIF_HASINDEX
        push_u32(&mut out, n); // dwTotalFrames
        push_u32(&mut out, 0); // dwInitialFrames
        push_u32(&mut out, 1); // dwStreams
        push_u32(&mut out, frame); // dwSuggestedBufferSize
        push_u32(&mut out, self.width as u32);
        push_u32(&mut out, self.height as u32);
        for _ in 0..4 {
            push_u32(&mut out, 0); // dwReserved
        }
        out.extend_from_slice(b"LIST");
        push_u32(&mut out, 116);
        out.extend_from_slice(b"strl");
        out.extend_from_slice(b"strh");
        push_u32(&mut out, 56);
        out.extend_from_slice(b"vids");
        out.extend_from_slice(b"DIB "); // fccHandler: uncompressed
        push_u32(&mut out, 0); // dwFlags
        push_u16(&mut out, 0); // wPriority
        push_u16(&mut out, 0); // wLanguage
        push_u32(&mut out, 0); // dwInitialFrames
        push_u32(&mut out, 1); // dwScale
        push_u32(&mut out, self.fps); // dwRate
        push_u32(&mut out, 0); // dwStart
        push_u32(&mut out, n); // dwLength
        push_u32(&mut out, frame); // dwSuggestedBufferSize
        push_u32(&mut out, 0xFFFF_FFFF); // dwQuality: default
        push_u32(&mut out, 0); // dwSampleSize
        push_u16(&mut out, 0);
        push_u16(&mut out, 0);
        push_u16(&mut out, self.width as u16);
        push_u16(&mut out, self.height as u16); // rcFrame
        out.extend_from_slice(b"strf");
        push_u32(&mut out, 40);
        push_u32(&mut out, 40); // biSize
        push_u32(&mut out, self.width as u32);
        push_u32(&mut out, self.height as u32); // positive: bottom-up
        push_u16(&mut out, 1); // biPlanes
        push_u16(&mut out, 24); // biBitCount
        push_u32(&mut out, 0); // BI_RGB
        push_u32(&mut out, frame); // biSizeImage
        for _ in 0..4 {
            push_u32(&mut out, 0); // pels per metre x/y, colours used/important
        }
        // movi
        out.extend_from_slice(b"LIST");
        push_u32(&mut out, 4 + self.movi.len() as u32);
        out.extend_from_slice(b"movi");
        out.extend_from_slice(&self.movi);
        // idx1: offsets are counted from the 'movi' fourcc.
        out.extend_from_slice(b"idx1");
        push_u32(&mut out, 16 * n);
        for k in 0..n {
            out.extend_from_slice(b"00db");
            push_u32(&mut out, 0x10); // AVIIF_KEYFRAME
            push_u32(&mut out, 4 + k * (8 + frame));
            push_u32(&mut out, frame);
        }
        let riff = (out.len() - 8) as u32;
        out[4..8].copy_from_slice(&riff.to_le_bytes());
        debug_assert_eq!(
            out.len() as u64,
            file_bytes(self.width, self.height, n as usize)
        );
        Ok(out)
    }
}

#[allow(dead_code)] // Used by the tests and kept as the plain entry point; the menu goes through `record` and `save`.
pub(super) fn avi_bytes(frames: &[RgbImage], fps: u32) -> Result<Vec<u8>, String> {
    let first = frames
        .first()
        .ok_or_else(|| "a video needs at least one frame".to_string())?;
    let mut b = AviBuilder::new(first.width, first.height, fps)?;
    for f in frames {
        b.push(f)?;
    }
    b.finish()
}

/// A 24 bit BI_RGB bitmap file (14 byte file header, BITMAPINFOHEADER, bottom-up rows).
/// An image whose pixel count does not match its size gives the headers only.
#[allow(dead_code)] // Used by the tests and kept as the plain entry point; the menu goes through `record` and `save`.
pub(super) fn bmp_bytes(image: &RgbImage) -> Vec<u8> {
    let stride = row_stride(image.width);
    let data = stride * image.height;
    let mut out = Vec::with_capacity(54 + data);
    out.extend_from_slice(b"BM");
    push_u32(&mut out, (54 + data) as u32);
    push_u32(&mut out, 0);
    push_u32(&mut out, 54);
    push_u32(&mut out, 40);
    push_u32(&mut out, image.width as u32);
    push_u32(&mut out, image.height as u32);
    push_u16(&mut out, 1);
    push_u16(&mut out, 24);
    push_u32(&mut out, 0);
    push_u32(&mut out, data as u32);
    for _ in 0..4 {
        push_u32(&mut out, 0);
    }
    if check_image(image).is_ok() {
        dib_rows(image, &mut out);
    }
    out
}
