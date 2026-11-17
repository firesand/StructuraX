//! Software rasteriser for egui's tessellated meshes.
//!
//! It follows what the egui 0.29 painters do: vertex colours are sRGBA with
//! premultiplied alpha and are interpolated in that (gamma) encoding; the fragment
//! colour is `vertex colour * texel`; blending is premultiplied, `dst = src + dst (1 - src_a)`,
//! in gamma space (the `fs_main_gamma_framebuffer` path of `egui-wgpu` and the default of
//! `egui_glow`); the scissor rectangle is the clip rectangle scaled by `pixels_per_point` and
//! rounded to whole pixels. The font texture arrives as `ImageDelta`s of
//! `TextureId::Managed(0)`, full or partial (`pos`), and is converted like the painters do
//! (`FontImage::srgba_pixels(None)`: coverage^0.55, premultiplied white). Not reproduced:
//! the dithering of the GPU path and any colour management of the display.

use super::RgbImage;
use eframe::egui::{
    self, epaint::Primitive, ClippedPrimitive, Color32, TextureFilter, TextureId, TexturesDelta,
};

/// The managed font texture, assembled from `TexturesDelta`s.
#[derive(Default)]
pub(in crate::gui) struct FontAtlas {
    size: [usize; 2],
    /// Premultiplied sRGBA texels, row-major, top row first.
    texels: Vec<[u8; 4]>,
    /// Magnification filter of the texture (egui's font texture is linear).
    linear: bool,
}

impl FontAtlas {
    pub(in crate::gui) fn is_empty(&self) -> bool {
        self.texels.is_empty()
    }

    #[allow(dead_code)] // Used by the tests and kept as the plain entry point; the menu goes through `record` and `save`.
    pub(in crate::gui) fn size(&self) -> [usize; 2] {
        self.size
    }

    /// The premultiplied texel at `(x, y)`.
    #[allow(dead_code)] // Used by the tests and kept as the plain entry point; the menu goes through `record` and `save`.
    pub(in crate::gui) fn texel(&self, x: usize, y: usize) -> Option<[u8; 4]> {
        if x < self.size[0] && y < self.size[1] {
            Some(self.texels[y * self.size[0] + x])
        } else {
            None
        }
    }

    /// Applies the deltas of `TextureId::Managed(0)` (the font texture) in order; other
    /// textures are not kept (a mesh that uses one is refused by [`rasterize`]).
    pub(in crate::gui) fn apply(&mut self, delta: &TexturesDelta) -> Result<(), String> {
        for (id, d) in &delta.set {
            if *id != TextureId::Managed(0) {
                continue;
            }
            let [w, h] = d.image.size();
            let pixels: Vec<[u8; 4]> = match &d.image {
                egui::ImageData::Color(c) => c.pixels.iter().map(|p| p.to_array()).collect(),
                egui::ImageData::Font(f) => f
                    .srgba_pixels(None)
                    .map(|p: Color32| p.to_array())
                    .collect(),
            };
            if pixels.len() != w * h {
                return Err(format!(
                    "font texture delta has {} texels for {w} x {h}",
                    pixels.len()
                ));
            }
            match d.pos {
                None => {
                    self.size = [w, h];
                    self.texels = pixels;
                    self.linear = d.options.magnification == TextureFilter::Linear;
                }
                Some([x0, y0]) => {
                    if self.texels.is_empty() {
                        return Err("partial font texture update before the full texture".into());
                    }
                    if x0 + w > self.size[0] || y0 + h > self.size[1] {
                        return Err(format!(
                            "partial font texture update {w} x {h} at ({x0}, {y0}) exceeds the \
                             {} x {} texture",
                            self.size[0], self.size[1]
                        ));
                    }
                    for row in 0..h {
                        let dst = (y0 + row) * self.size[0] + x0;
                        self.texels[dst..dst + w].copy_from_slice(&pixels[row * w..(row + 1) * w]);
                    }
                }
            }
        }
        if delta.free.contains(&TextureId::Managed(0)) {
            *self = FontAtlas::default();
        }
        Ok(())
    }

    /// Texel colour at normalised `uv`, premultiplied, components in 0..1.
    fn sample(&self, u: f64, v: f64) -> [f64; 4] {
        let [w, h] = self.size;
        let fetch = |x: i64, y: i64| -> [f64; 4] {
            let x = x.clamp(0, w as i64 - 1) as usize;
            let y = y.clamp(0, h as i64 - 1) as usize;
            let t = self.texels[y * w + x];
            [
                t[0] as f64 / 255.0,
                t[1] as f64 / 255.0,
                t[2] as f64 / 255.0,
                t[3] as f64 / 255.0,
            ]
        };
        if !self.linear {
            return fetch((u * w as f64).floor() as i64, (v * h as f64).floor() as i64);
        }
        let fx = u * w as f64 - 0.5;
        let fy = v * h as f64 - 0.5;
        let (x0, y0) = (fx.floor(), fy.floor());
        let (tx, ty) = (fx - x0, fy - y0);
        let (x0, y0) = (x0 as i64, y0 as i64);
        let (a, b, c, d) = (
            fetch(x0, y0),
            fetch(x0 + 1, y0),
            fetch(x0, y0 + 1),
            fetch(x0 + 1, y0 + 1),
        );
        let mut out = [0.0; 4];
        for k in 0..4 {
            out[k] =
                (a[k] * (1.0 - tx) + b[k] * tx) * (1.0 - ty) + (c[k] * (1.0 - tx) + d[k] * tx) * ty;
        }
        out
    }
}

/// `E(p, q, pt)`: twice the signed area of `(p, q, pt)`; the edge is evaluated in a canonical
/// direction so that two triangles sharing it get exactly opposite values.
fn edge(p: [f64; 2], q: [f64; 2], pt: [f64; 2]) -> f64 {
    let e =
        |p: [f64; 2], q: [f64; 2]| (q[0] - p[0]) * (pt[1] - p[1]) - (q[1] - p[1]) * (pt[0] - p[0]);
    if (p[0], p[1]) <= (q[0], q[1]) {
        e(p, q)
    } else {
        -e(q, p)
    }
}

/// Top-left rule for a triangle of positive area in y-down coordinates (interior on the
/// positive side of every edge): a top edge is horizontal and runs toward +x, a left edge
/// runs toward -y.
fn top_left(p: [f64; 2], q: [f64; 2]) -> bool {
    let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
    (dy == 0.0 && dx > 0.0) || dy < 0.0
}

struct Target<'a> {
    width: usize,
    pixels: &'a mut [[u8; 3]],
}

#[allow(clippy::too_many_arguments)]
fn fill_triangle(
    t: &mut Target,
    scissor: [usize; 4], // x0, y0, x1, y1 (exclusive)
    pos: [[f64; 2]; 3],
    col: [[f64; 4]; 3],
    uv: [[f64; 2]; 3],
    atlas: &FontAtlas,
) {
    let [a, mut b, mut c] = pos;
    let (ca, mut cb, mut cc) = (col[0], col[1], col[2]);
    let (ua, mut ub, mut uc) = (uv[0], uv[1], uv[2]);
    let area = edge(a, b, c);
    if area == 0.0 || !area.is_finite() {
        return;
    }
    if area < 0.0 {
        std::mem::swap(&mut b, &mut c);
        std::mem::swap(&mut cb, &mut cc);
        std::mem::swap(&mut ub, &mut uc);
    }
    let min_x = a[0].min(b[0]).min(c[0]).floor().max(scissor[0] as f64) as usize;
    let max_x = (a[0].max(b[0]).max(c[0]).ceil().min(scissor[2] as f64)).max(0.0) as usize;
    let min_y = a[1].min(b[1]).min(c[1]).floor().max(scissor[1] as f64) as usize;
    let max_y = (a[1].max(b[1]).max(c[1]).ceil().min(scissor[3] as f64)).max(0.0) as usize;
    let (tla, tlb, tlc) = (top_left(b, c), top_left(c, a), top_left(a, b));
    for y in min_y..max_y {
        for x in min_x..max_x {
            let p = [x as f64 + 0.5, y as f64 + 0.5];
            let wa = edge(b, c, p);
            let wb = edge(c, a, p);
            let wc = edge(a, b, p);
            let inside = (wa > 0.0 || (wa == 0.0 && tla))
                && (wb > 0.0 || (wb == 0.0 && tlb))
                && (wc > 0.0 || (wc == 0.0 && tlc));
            if !inside {
                continue;
            }
            let sum = wa + wb + wc;
            let (la, lb, lc) = (wa / sum, wb / sum, wc / sum);
            let mut vc = [0.0; 4];
            for k in 0..4 {
                vc[k] = la * ca[k] + lb * cb[k] + lc * cc[k];
            }
            let u = la * ua[0] + lb * ub[0] + lc * uc[0];
            let v = la * ua[1] + lb * ub[1] + lc * uc[1];
            let tx = atlas.sample(u, v);
            let alpha = vc[3] * tx[3];
            let dst = &mut t.pixels[y * t.width + x];
            for k in 0..3 {
                let src = vc[k] * tx[k];
                let out = src * 255.0 + dst[k] as f64 * (1.0 - alpha);
                dst[k] = out.round().clamp(0.0, 255.0) as u8;
            }
        }
    }
}

/// Rasterises `prims` (egui points, scaled by `pixels_per_point`) onto `background`.
pub(super) fn rasterize(
    prims: &[ClippedPrimitive],
    atlas: &FontAtlas,
    width: usize,
    height: usize,
    pixels_per_point: f32,
    background: Color32,
) -> Result<RgbImage, String> {
    if width == 0 || height == 0 {
        return Err(format!("image size {width} x {height} is empty"));
    }
    if !(pixels_per_point.is_finite() && pixels_per_point > 0.0) {
        return Err(format!(
            "pixels_per_point {pixels_per_point} is not positive"
        ));
    }
    let bg = [background.r(), background.g(), background.b()];
    let mut pixels = vec![bg; width * height];
    let ppp = pixels_per_point as f64;
    let mut target = Target {
        width,
        pixels: &mut pixels,
    };
    for prim in prims {
        let mesh = match &prim.primitive {
            Primitive::Mesh(m) => m,
            Primitive::Callback(_) => {
                return Err("a paint callback (custom GPU painting) cannot be rasterised".into())
            }
        };
        if mesh.indices.is_empty() {
            continue;
        }
        if mesh.texture_id != TextureId::Managed(0) {
            return Err(format!(
                "a mesh uses texture {:?}; only the font texture (Managed(0)) is supported",
                mesh.texture_id
            ));
        }
        if atlas.is_empty() {
            return Err("a mesh needs the font texture but none was applied".into());
        }
        if mesh.indices.len() % 3 != 0 {
            return Err(format!(
                "mesh has {} indices, not a multiple of 3",
                mesh.indices.len()
            ));
        }
        if let Some(bad) = mesh
            .indices
            .iter()
            .find(|&&i| i as usize >= mesh.vertices.len())
        {
            return Err(format!(
                "mesh index {bad} is outside its {} vertices",
                mesh.vertices.len()
            ));
        }
        let r = prim.clip_rect;
        let clamp = |v: f32, hi: usize| -> usize {
            let v = (v as f64 * ppp).round();
            if v.is_nan() {
                0
            } else {
                v.clamp(0.0, hi as f64) as usize
            }
        };
        let scissor = [
            clamp(r.min.x, width),
            clamp(r.min.y, height),
            clamp(r.max.x, width),
            clamp(r.max.y, height),
        ];
        if scissor[0] >= scissor[2] || scissor[1] >= scissor[3] {
            continue;
        }
        for tri in mesh.indices.chunks_exact(3) {
            let vs = [
                &mesh.vertices[tri[0] as usize],
                &mesh.vertices[tri[1] as usize],
                &mesh.vertices[tri[2] as usize],
            ];
            let pos = vs.map(|v| [v.pos.x as f64 * ppp, v.pos.y as f64 * ppp]);
            let col = vs.map(|v| {
                let c = v.color;
                [
                    c.r() as f64 / 255.0,
                    c.g() as f64 / 255.0,
                    c.b() as f64 / 255.0,
                    c.a() as f64 / 255.0,
                ]
            });
            let uv = vs.map(|v| [v.uv.x as f64, v.uv.y as f64]);
            fill_triangle(&mut target, scissor, pos, col, uv, atlas);
        }
    }
    Ok(RgbImage {
        width,
        height,
        pixels,
    })
}
