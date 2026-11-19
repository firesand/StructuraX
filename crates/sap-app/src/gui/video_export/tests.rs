//! Tests of the video export. References: the AVI/BMP layouts are written out below from the
//! published format descriptions (Microsoft AVI RIFF reference, BITMAPINFOHEADER), the
//! rasteriser cases are closed forms (areas, linear interpolation, premultiplied blending),
//! and an independent decoder (ffmpeg) reads the AVI back when it is installed.

use super::*;
use eframe::egui::epaint::ImageDelta;
use eframe::egui::{
    epaint::{Mesh, Primitive, Vertex},
    pos2, vec2, Color32, ColorImage, ImageData, Pos2, Rect, TextureId, TextureOptions,
    TexturesDelta,
};
use sap_model::{CaseResults, ModalModeResult, SapModel};
use sap_units::Units;

// ----------------------------------------------------------------------------- helpers

fn le32(x: u32) -> [u8; 4] {
    x.to_le_bytes()
}

fn solid(w: usize, h: usize, rgb: [u8; 3]) -> RgbImage {
    RgbImage {
        width: w,
        height: h,
        pixels: vec![rgb; w * h],
    }
}

fn gradient(w: usize, h: usize, seed: u8) -> RgbImage {
    let pixels = (0..w * h)
        .map(|i| {
            let (x, y) = ((i % w) as u8, (i / w) as u8);
            [
                x.wrapping_mul(17).wrapping_add(seed),
                y.wrapping_mul(23).wrapping_add(seed),
                seed.wrapping_mul(7).wrapping_add(x + y),
            ]
        })
        .collect();
    RgbImage {
        width: w,
        height: h,
        pixels,
    }
}

fn white_atlas() -> FontAtlas {
    let mut a = FontAtlas::default();
    let img = ColorImage::new([1, 1], Color32::WHITE);
    let mut d = TexturesDelta::default();
    d.set.push((
        TextureId::Managed(0),
        ImageDelta::full(img, TextureOptions::NEAREST),
    ));
    a.apply(&d).unwrap();
    a
}

fn vertex(x: f32, y: f32, c: Color32) -> Vertex {
    Vertex {
        pos: pos2(x, y),
        uv: egui::epaint::WHITE_UV,
        color: c,
    }
}

fn mesh_prim(verts: &[(f32, f32, Color32)], indices: &[u32], clip: Rect) -> egui::ClippedPrimitive {
    let mut m = Mesh::default();
    m.vertices = verts.iter().map(|&(x, y, c)| vertex(x, y, c)).collect();
    m.indices = indices.to_vec();
    egui::ClippedPrimitive {
        clip_rect: clip,
        primitive: Primitive::Mesh(m),
    }
}

fn full(w: usize, h: usize) -> Rect {
    Rect::from_min_size(Pos2::ZERO, vec2(w as f32, h as f32))
}

fn raster(prims: &[egui::ClippedPrimitive], w: usize, h: usize, bg: Color32) -> RgbImage {
    rasterize(prims, &white_atlas(), w, h, 1.0, bg).unwrap()
}

fn px(img: &RgbImage, x: usize, y: usize) -> [u8; 3] {
    img.pixels[y * img.width + x]
}

// ------------------------------------------------------------------- AVI container

/// Walks the RIFF tree: every chunk's size field must equal its payload, payloads are padded
/// to even length, LIST chunks are descended. Returns the chunk path names in file order.
fn walk(bytes: &[u8], out: &mut Vec<String>, prefix: &str) {
    let mut p = 0;
    while p < bytes.len() {
        assert!(p + 8 <= bytes.len(), "truncated chunk header at {p}");
        let id = String::from_utf8_lossy(&bytes[p..p + 4]).to_string();
        let size = u32::from_le_bytes(bytes[p + 4..p + 8].try_into().unwrap()) as usize;
        assert!(
            p + 8 + size <= bytes.len(),
            "chunk {id} at {p} overruns its parent"
        );
        let payload = &bytes[p + 8..p + 8 + size];
        if id == "LIST" {
            let kind = String::from_utf8_lossy(&payload[..4]).to_string();
            out.push(format!("{prefix}LIST {kind}"));
            walk(&payload[4..], out, &format!("{prefix}{kind}/"));
        } else {
            out.push(format!("{prefix}{id}"));
        }
        p += 8 + size + (size & 1);
    }
    assert_eq!(p, bytes.len(), "children do not fill their parent exactly");
}

#[test]
fn avi_sizes_match_the_layout_arithmetic() {
    // 8 x 6, 3 frames: row 8*3 = 24 bytes (already a multiple of 4), frame 24*6 = 144;
    // hdrl LIST = 4 + (8+56) + [LIST strl: 8 + 4 + (8+56) + (8+40)] = 192;
    // movi LIST = 4 + 3*(8+144) = 460; idx1 data = 3*16 = 48;
    // RIFF = 4 + (8+192) + (8+460) + (8+48) = 728; file = 736.
    let frames: Vec<_> = (0..3).map(|k| gradient(8, 6, k)).collect();
    let f = avi_bytes(&frames, 10).unwrap();
    assert_eq!(f.len(), 736);
    assert_eq!(&f[0..4], b"RIFF");
    assert_eq!(u32::from_le_bytes(f[4..8].try_into().unwrap()), 728);
    assert_eq!(&f[8..12], b"AVI ");
    assert_eq!(u32::from_le_bytes(f[16..20].try_into().unwrap()), 192);
    // 7 x 5, 2 frames: row 21 -> 24, frame 120, movi 4 + 2*128 = 260, idx1 32,
    // RIFF = 4 + 200 + 268 + 40 = 512, file 520.
    let frames: Vec<_> = (0..2).map(|k| gradient(7, 5, k)).collect();
    let g = avi_bytes(&frames, 10).unwrap();
    assert_eq!(g.len(), 520);
    assert_eq!(u32::from_le_bytes(g[4..8].try_into().unwrap()), 512);
    assert_eq!(super::avi::file_bytes(7, 5, 2), 520);
    // chunk tree
    let mut names = Vec::new();
    walk(&g[12..], &mut names, "");
    assert_eq!(
        names,
        [
            "LIST hdrl",
            "hdrl/avih",
            "hdrl/LIST strl",
            "hdrl/strl/strh",
            "hdrl/strl/strf",
            "LIST movi",
            "movi/00db",
            "movi/00db",
            "idx1"
        ]
    );
}

#[test]
fn avi_tiny_video_equals_the_hand_written_layout() {
    // 3 x 2 pixels, 2 frames, 25 fps. Row = 9 bytes, padded to 12; frame = 24 bytes.
    let f0 = RgbImage {
        width: 3,
        height: 2,
        pixels: vec![
            [1, 2, 3],
            [4, 5, 6],
            [7, 8, 9],
            [10, 11, 12],
            [13, 14, 15],
            [16, 17, 18],
        ],
    };
    let f1 = solid(3, 2, [200, 100, 50]);
    let got = avi_bytes(&[f0, f1], 25).unwrap();

    let mut e: Vec<u8> = Vec::new();
    let tag = |e: &mut Vec<u8>, s: &[u8; 4]| e.extend_from_slice(s);
    let w32 = |e: &mut Vec<u8>, x: u32| e.extend_from_slice(&le32(x));
    let w16 = |e: &mut Vec<u8>, x: u16| e.extend_from_slice(&x.to_le_bytes());
    tag(&mut e, b"RIFF");
    w32(&mut e, 320); // 4 + (8+192) + (8+68) + (8+32)
    tag(&mut e, b"AVI ");
    tag(&mut e, b"LIST");
    w32(&mut e, 192);
    tag(&mut e, b"hdrl");
    tag(&mut e, b"avih");
    w32(&mut e, 56);
    w32(&mut e, 40_000); // microseconds per frame
    w32(&mut e, 600); // max bytes per second = 24 * 25
    w32(&mut e, 0); // padding granularity
    w32(&mut e, 0x10); // AVIF_HASINDEX
    w32(&mut e, 2); // total frames
    w32(&mut e, 0); // initial frames
    w32(&mut e, 1); // streams
    w32(&mut e, 24); // suggested buffer size
    w32(&mut e, 3); // width
    w32(&mut e, 2); // height
    e.extend_from_slice(&[0; 16]); // reserved
    tag(&mut e, b"LIST");
    w32(&mut e, 116);
    tag(&mut e, b"strl");
    tag(&mut e, b"strh");
    w32(&mut e, 56);
    tag(&mut e, b"vids");
    tag(&mut e, b"DIB ");
    w32(&mut e, 0); // flags
    w16(&mut e, 0); // priority
    w16(&mut e, 0); // language
    w32(&mut e, 0); // initial frames
    w32(&mut e, 1); // scale
    w32(&mut e, 25); // rate: frames per second = rate / scale
    w32(&mut e, 0); // start
    w32(&mut e, 2); // length in frames
    w32(&mut e, 24); // suggested buffer size
    w32(&mut e, 0xFFFF_FFFF); // quality
    w32(&mut e, 0); // sample size
    for v in [0u16, 0, 3, 2] {
        w16(&mut e, v); // rcFrame left, top, right, bottom
    }
    tag(&mut e, b"strf");
    w32(&mut e, 40);
    w32(&mut e, 40); // biSize
    w32(&mut e, 3); // biWidth
    w32(&mut e, 2); // biHeight (positive: bottom-up)
    w16(&mut e, 1); // planes
    w16(&mut e, 24); // bit count
    w32(&mut e, 0); // BI_RGB
    w32(&mut e, 24); // size image
    e.extend_from_slice(&[0; 16]); // x/y pels per metre, colours used, important
    tag(&mut e, b"LIST");
    w32(&mut e, 68); // 'movi' + 2 * (8 + 24)
    tag(&mut e, b"movi");
    tag(&mut e, b"00db");
    w32(&mut e, 24);
    // frame 0: bottom row first, BGR, padded with 3 zero bytes
    e.extend_from_slice(&[12, 11, 10, 15, 14, 13, 18, 17, 16, 0, 0, 0]);
    e.extend_from_slice(&[3, 2, 1, 6, 5, 4, 9, 8, 7, 0, 0, 0]);
    tag(&mut e, b"00db");
    w32(&mut e, 24);
    for _ in 0..2 {
        e.extend_from_slice(&[50, 100, 200, 50, 100, 200, 50, 100, 200, 0, 0, 0]);
    }
    tag(&mut e, b"idx1");
    w32(&mut e, 32);
    for (off, size) in [(4u32, 24u32), (36, 24)] {
        tag(&mut e, b"00db");
        w32(&mut e, 0x10);
        w32(&mut e, off); // from the 'movi' fourcc
        w32(&mut e, size);
    }
    assert_eq!(e.len(), 328);
    assert_eq!(got, e);
}

#[test]
fn bmp_equals_the_hand_written_layout() {
    let img = RgbImage {
        width: 3,
        height: 2,
        pixels: vec![
            [1, 2, 3],
            [4, 5, 6],
            [7, 8, 9],
            [10, 11, 12],
            [13, 14, 15],
            [16, 17, 18],
        ],
    };
    let mut e = Vec::new();
    e.extend_from_slice(b"BM");
    e.extend_from_slice(&le32(54 + 24)); // file size
    e.extend_from_slice(&le32(0)); // reserved
    e.extend_from_slice(&le32(54)); // offset of the pixels
    e.extend_from_slice(&le32(40));
    e.extend_from_slice(&le32(3));
    e.extend_from_slice(&le32(2));
    e.extend_from_slice(&[1, 0, 24, 0]); // planes, bit count
    e.extend_from_slice(&le32(0)); // BI_RGB
    e.extend_from_slice(&le32(24));
    e.extend_from_slice(&[0; 16]);
    e.extend_from_slice(&[12, 11, 10, 15, 14, 13, 18, 17, 16, 0, 0, 0]);
    e.extend_from_slice(&[3, 2, 1, 6, 5, 4, 9, 8, 7, 0, 0, 0]);
    assert_eq!(bmp_bytes(&img), e);
}

#[test]
fn avi_refusals() {
    let ok = solid(4, 4, [0; 3]);
    assert!(avi_bytes(&[], 10)
        .unwrap_err()
        .contains("at least one frame"));
    assert!(avi_bytes(std::slice::from_ref(&ok), 0)
        .unwrap_err()
        .contains("frames per second"));
    assert!(avi_bytes(std::slice::from_ref(&ok), 121)
        .unwrap_err()
        .contains("frames per second"));
    assert!(avi_bytes(&[solid(1, 4, [0; 3])], 10)
        .unwrap_err()
        .contains("frame size"));
    assert!(avi_bytes(&[solid(4, 1, [0; 3])], 10)
        .unwrap_err()
        .contains("frame size"));
    assert!(avi_bytes(&[solid(4097, 2, [0; 3])], 10)
        .unwrap_err()
        .contains("frame size"));
    assert!(avi_bytes(&[ok.clone(), solid(5, 4, [0; 3])], 10)
        .unwrap_err()
        .contains("frame 2"));
    let bad = RgbImage {
        width: 4,
        height: 4,
        pixels: vec![[0; 3]; 3],
    };
    assert!(avi_bytes(&[bad], 10).unwrap_err().contains("pixels"));
    // boundary values are accepted
    assert!(avi_bytes(&[solid(2, 2, [0; 3])], 1).is_ok());
    assert!(avi_bytes(&[solid(2, 2, [0; 3])], 120).is_ok());
}

#[test]
fn avi_one_gib_limit_is_arithmetic_on_the_layout() {
    // 4096 x 4096 frames are 4096*3 = 12288 bytes per row (a multiple of 4): 50_331_648
    // bytes plus 8 for the chunk header and 16 for the index entry; 21 frames fit
    // (21 * 50_331_672 + 232 + 8 = 1_056_964_...) below 1 GiB = 1_073_741_824, 22 do not.
    assert!(super::avi::file_bytes(4096, 4096, 21) <= 1 << 30);
    assert!(super::avi::file_bytes(4096, 4096, 22) > 1 << 30);
    assert_eq!(
        super::avi::file_bytes(4096, 4096, 21),
        12 + 200 + 12 + 21 * (8 + 50_331_648) + 8 + 16 * 21
    );
}

#[test]
fn ffmpeg_reads_back_count_size_rate_and_pixels() {
    let have = |p: &str| {
        std::process::Command::new(p)
            .arg("-version")
            .output()
            .map(|o| o.status.success())
            .unwrap_or(false)
    };
    if !have("ffmpeg") || !have("ffprobe") {
        return;
    }
    let (w, h) = (9usize, 7usize);
    let frames: Vec<_> = (0..4).map(|k| gradient(w, h, 40 * k as u8)).collect();
    let bytes = avi_bytes(&frames, 12).unwrap();
    let dir = std::env::temp_dir().join(format!("structurax_video_ffmpeg_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("t.avi");
    std::fs::write(&path, &bytes).unwrap();
    let probe = std::process::Command::new("ffprobe")
        .args([
            "-v",
            "error",
            "-count_frames",
            "-select_streams",
            "v:0",
            "-show_entries",
        ])
        .arg("stream=width,height,nb_read_frames,r_frame_rate,codec_name,pix_fmt")
        .args(["-of", "default=nw=1"])
        .arg(&path)
        .output()
        .unwrap();
    let text = String::from_utf8_lossy(&probe.stdout).to_string();
    assert!(
        probe.status.success(),
        "ffprobe: {}",
        String::from_utf8_lossy(&probe.stderr)
    );
    for want in [
        "width=9",
        "height=7",
        "nb_read_frames=4",
        "r_frame_rate=12/1",
        "codec_name=rawvideo",
    ] {
        assert!(
            text.lines().any(|l| l == want),
            "ffprobe output lacks {want}: {text}"
        );
    }
    for k in [0usize, 3] {
        let dec = std::process::Command::new("ffmpeg")
            .args(["-v", "error", "-i"])
            .arg(&path)
            .args([
                "-vf",
                &format!("select=eq(n\\,{k})"),
                "-frames:v",
                "1",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "rgb24",
                "-",
            ])
            .output()
            .unwrap();
        assert!(
            dec.status.success(),
            "ffmpeg: {}",
            String::from_utf8_lossy(&dec.stderr)
        );
        let want: Vec<u8> = frames[k].pixels.iter().flatten().copied().collect();
        assert_eq!(
            dec.stdout, want,
            "decoded frame {k} differs from the frame that was written"
        );
    }
    let _ = std::fs::remove_dir_all(&dir);
}

// --------------------------------------------------------------------------- rasteriser

#[test]
fn r2_triangles_cover_45_and_55_pixels_without_overlap() {
    let red = Color32::from_rgb(255, 0, 0);
    let blue = Color32::from_rgb(0, 0, 255);
    let t1 = mesh_prim(
        &[(0., 0., red), (10., 0., red), (0., 10., red)],
        &[0, 1, 2],
        full(10, 10),
    );
    let t2 = mesh_prim(
        &[(10., 0., blue), (10., 10., blue), (0., 10., blue)],
        &[0, 1, 2],
        full(10, 10),
    );
    let bg = Color32::BLACK;
    let a = raster(std::slice::from_ref(&t1), 10, 10, bg);
    let b = raster(std::slice::from_ref(&t2), 10, 10, bg);
    let count = |i: &RgbImage| i.pixels.iter().filter(|p| **p != [0, 0, 0]).count();
    // Pixel centres (x+.5, y+.5): strictly inside the first triangle when x + y + 1 < 10,
    // i.e. x + y <= 8: 1+2+...+9 = 45; the ten centres with x + y = 9 lie on the shared
    // diagonal, which the top-left rule gives to the second triangle: 45 + 10 = 55.
    assert_eq!(count(&a), 45);
    assert_eq!(count(&b), 55);
    for (pa, pb) in a.pixels.iter().zip(&b.pixels) {
        assert!(
            *pa == [0, 0, 0] || *pb == [0, 0, 0],
            "a pixel is covered twice"
        );
    }
    let both = raster(&[t1, t2], 10, 10, bg);
    assert_eq!(count(&both), 100);
}

#[test]
fn rectangle_of_one_colour_covers_exactly_its_pixels_either_winding() {
    let c = Color32::from_rgb(10, 200, 90);
    // rectangle [2,7) x [3,5) as two triangles; the second primitive has the other winding
    let v = [(2., 3., c), (7., 3., c), (7., 5., c), (2., 5., c)];
    for idx in [[0u32, 1, 2, 0, 2, 3], [0, 2, 1, 0, 3, 2]] {
        let img = raster(&[mesh_prim(&v, &idx, full(10, 10))], 10, 10, Color32::BLACK);
        for y in 0..10 {
            for x in 0..10 {
                let inside = (2..7).contains(&x) && (3..5).contains(&y);
                assert_eq!(
                    px(&img, x, y),
                    if inside { [10, 200, 90] } else { [0, 0, 0] },
                    "pixel {x},{y}"
                );
            }
        }
    }
}

#[test]
fn r3_opaque_and_premultiplied_colours() {
    let opaque = Color32::from_rgb(200, 100, 50);
    let t = mesh_prim(
        &[(0., 0., opaque), (8., 0., opaque), (0., 8., opaque)],
        &[0, 1, 2],
        full(8, 8),
    );
    assert_eq!(
        px(&raster(&[t], 8, 8, Color32::BLACK), 1, 1),
        [200, 100, 50]
    );
    // premultiplied (100, 50, 25, 128) over black: src + dst (1 - a) = (100, 50, 25)
    let pm = Color32::from_rgba_premultiplied(100, 50, 25, 128);
    let t = mesh_prim(
        &[(0., 0., pm), (8., 0., pm), (0., 8., pm)],
        &[0, 1, 2],
        full(8, 8),
    );
    assert_eq!(px(&raster(&[t], 8, 8, Color32::BLACK), 1, 1), [100, 50, 25]);
}

#[test]
fn half_transparent_rectangle_blends_in_gamma_space() {
    // The painters blend the premultiplied sRGB bytes of the vertex colour as they are:
    // premultiplied (128, 0, 0, 128) over the opaque background (0, 0, 255) gives
    // R = 128 + 0 (1 - 128/255) = 128 and B = 0 + 255 (1 - 128/255) = 127. (Blending in linear
    // light would give another red, about 188; `Color32::from_rgba_unmultiplied` itself
    // premultiplies in linear light, so the colour is given premultiplied here.)
    let c = Color32::from_rgba_premultiplied(128, 0, 0, 128);
    let v = [(0., 0., c), (6., 0., c), (6., 6., c), (0., 6., c)];
    let img = raster(
        &[mesh_prim(&v, &[0, 1, 2, 0, 2, 3], full(6, 6))],
        6,
        6,
        Color32::from_rgb(0, 0, 255),
    );
    assert_eq!(px(&img, 3, 3), [128, 0, 127]);
}

#[test]
fn vertex_colours_interpolate_linearly() {
    // vertices (1.5,1.5) (10.5,1.5) (1.5,10.5) have the centroid (4.5, 4.5), the centre of
    // pixel (4, 4): colour = mean = (30 + 90 + 180) / 3 = 100 (red), (60 + 0 + 0)/3 = 20 (green).
    let (c0, c1, c2) = (
        Color32::from_rgb(30, 60, 0),
        Color32::from_rgb(90, 0, 0),
        Color32::from_rgb(180, 0, 90),
    );
    let t = mesh_prim(
        &[(1.5, 1.5, c0), (10.5, 1.5, c1), (1.5, 10.5, c2)],
        &[0, 1, 2],
        full(12, 12),
    );
    let img = raster(&[t], 12, 12, Color32::BLACK);
    assert_eq!(px(&img, 4, 4), [100, 20, 30]);
    // pixel (2, 1): centre (2.5, 1.5); barycentric weights are l1 = (2.5-1.5)/9 = 1/9 on the
    // second vertex, l2 = 0 on the third, l0 = 8/9: red = 30*8/9 + 90/9 = 36.67 -> 37.
    let want = (30.0 * 8.0 / 9.0 + 90.0 / 9.0f64).round() as u8;
    assert_eq!(px(&img, 2, 1)[0], want);
}

#[test]
fn clip_rectangle_limits_the_painted_pixels() {
    let c = Color32::from_rgb(255, 255, 255);
    let v = [(0., 0., c), (10., 0., c), (10., 10., c), (0., 10., c)];
    let clip = Rect::from_min_max(pos2(3.0, 2.0), pos2(6.0, 8.0));
    let img = raster(
        &[mesh_prim(&v, &[0, 1, 2, 0, 2, 3], clip)],
        10,
        10,
        Color32::BLACK,
    );
    for y in 0..10 {
        for x in 0..10 {
            let inside = (3..6).contains(&x) && (2..8).contains(&y);
            assert_eq!(px(&img, x, y) == [255; 3], inside, "pixel {x},{y}");
        }
    }
}

#[test]
fn pixels_per_point_scales_positions_and_clip() {
    let c = Color32::from_rgb(255, 255, 255);
    let v = [(1., 1., c), (3., 1., c), (3., 2., c), (1., 2., c)];
    let prim = mesh_prim(&v, &[0, 1, 2, 0, 2, 3], full(10, 10));
    let img = rasterize(&[prim], &white_atlas(), 10, 10, 2.0, Color32::BLACK).unwrap();
    for y in 0..10 {
        for x in 0..10 {
            let inside = (2..6).contains(&x) && (2..4).contains(&y);
            assert_eq!(px(&img, x, y) == [255; 3], inside, "pixel {x},{y}");
        }
    }
}

#[test]
fn rasterize_refusals() {
    let c = Color32::WHITE;
    let good = mesh_prim(
        &[(0., 0., c), (4., 0., c), (0., 4., c)],
        &[0, 1, 2],
        full(4, 4),
    );
    let atlas = white_atlas();
    assert!(rasterize(std::slice::from_ref(&good), &atlas, 0, 4, 1.0, c)
        .unwrap_err()
        .contains("empty"));
    assert!(rasterize(std::slice::from_ref(&good), &atlas, 4, 4, 0.0, c).is_err());
    // a user texture is refused and named
    let mut user = good.clone();
    if let Primitive::Mesh(m) = &mut user.primitive {
        m.texture_id = TextureId::User(7);
    }
    let e = rasterize(&[user], &atlas, 4, 4, 1.0, c).unwrap_err();
    assert!(e.contains("User(7)"), "{e}");
    // an index outside the vertices, and a count that is not a multiple of 3
    let mut bad = good.clone();
    if let Primitive::Mesh(m) = &mut bad.primitive {
        m.indices = vec![0, 1, 3];
    }
    assert!(rasterize(&[bad], &atlas, 4, 4, 1.0, c)
        .unwrap_err()
        .contains("index 3"));
    let mut bad = good.clone();
    if let Primitive::Mesh(m) = &mut bad.primitive {
        m.indices = vec![0, 1];
    }
    assert!(rasterize(&[bad], &atlas, 4, 4, 1.0, c)
        .unwrap_err()
        .contains("multiple of 3"));
    // no font texture applied yet
    assert!(rasterize(&[good], &FontAtlas::default(), 4, 4, 1.0, c)
        .unwrap_err()
        .contains("font texture"));
}

#[test]
fn font_atlas_full_and_partial_deltas() {
    // Font coverage c becomes premultiplied white with alpha = round(c^0.55 * 255)
    // (epaint's FontImage::srgba_pixels(None)): 1 -> 255, 0 -> 0, 0.5 -> 0.5^0.55 = 0.68302 -> 174.
    let mut atlas = FontAtlas::default();
    let mut full_img = egui::epaint::FontImage::new([4, 3]);
    full_img.pixels[0] = 1.0;
    let mut d = TexturesDelta::default();
    d.set.push((
        TextureId::Managed(0),
        ImageDelta::full(ImageData::Font(full_img), TextureOptions::LINEAR),
    ));
    atlas.apply(&d).unwrap();
    assert_eq!(atlas.size(), [4, 3]);
    assert_eq!(atlas.texel(0, 0), Some([255; 4]));
    assert_eq!(atlas.texel(1, 1), Some([0; 4]));
    // a 2 x 1 patch at (1, 2): coverage 0.5 and 1.0
    let mut patch = egui::epaint::FontImage::new([2, 1]);
    patch.pixels = vec![0.5, 1.0];
    let mut d = TexturesDelta::default();
    d.set.push((
        TextureId::Managed(0),
        ImageDelta::partial([1, 2], ImageData::Font(patch), TextureOptions::LINEAR),
    ));
    atlas.apply(&d).unwrap();
    assert_eq!(atlas.texel(1, 2), Some([174; 4]));
    assert_eq!(atlas.texel(2, 2), Some([255; 4]));
    assert_eq!(atlas.texel(0, 2), Some([0; 4]));
    assert_eq!(atlas.texel(3, 2), Some([0; 4]));
    assert_eq!(atlas.texel(0, 0), Some([255; 4]));
    // a patch outside the texture, and a patch before any full texture, are refused
    let mut d = TexturesDelta::default();
    d.set.push((
        TextureId::Managed(0),
        ImageDelta::partial(
            [3, 2],
            ColorImage::new([2, 2], Color32::WHITE),
            TextureOptions::LINEAR,
        ),
    ));
    assert!(atlas.apply(&d).unwrap_err().contains("exceeds"));
    assert!(FontAtlas::default()
        .apply(&d)
        .unwrap_err()
        .contains("before"));
    // another texture's delta does not touch the font texture
    let mut d = TexturesDelta::default();
    d.set.push((
        TextureId::User(1),
        ImageDelta::full(
            ColorImage::new([2, 2], Color32::RED),
            TextureOptions::NEAREST,
        ),
    ));
    atlas.apply(&d).unwrap();
    assert_eq!(atlas.size(), [4, 3]);
}

/// One headless frame of the real egui context, rasterised.
fn render(
    w: usize,
    h: usize,
    paint: impl Fn(&egui::Context),
) -> (RgbImage, Vec<egui::ClippedPrimitive>) {
    let ctx = egui::Context::default();
    let mut atlas = FontAtlas::default();
    let input = || egui::RawInput {
        screen_rect: Some(full(w, h)),
        ..Default::default()
    };
    let out = ctx.run(input(), |c| paint(c));
    atlas.apply(&out.textures_delta).unwrap();
    let prims = ctx.tessellate(out.shapes, out.pixels_per_point);
    let img = rasterize(&prims, &atlas, w, h, out.pixels_per_point, Color32::WHITE).unwrap();
    (img, prims)
}

#[test]
fn r4_headless_frame_circle_and_label() {
    let colour = Color32::from_rgb(200, 100, 50);
    let c = pos2(30.5, 20.5);
    let (img, prims) = render(120, 60, |ctx| {
        let p = ctx.layer_painter(egui::LayerId::background());
        p.circle_filled(c, 3.8, colour);
        p.text(
            pos2(50.0, 10.0),
            egui::Align2::LEFT_TOP,
            "Load",
            egui::FontId::proportional(14.0),
            Color32::BLACK,
        );
    });
    assert!(!prims.is_empty());
    assert_eq!(px(&img, 30, 20), [200, 100, 50]);
    // the label's glyph pixels are not background
    let dark = (50..100)
        .flat_map(|x| (10..30).map(move |y| (x, y)))
        .filter(|&(x, y)| px(&img, x, y) != [255; 3])
        .count();
    assert!(
        dark > 20,
        "only {dark} label pixels differ from the background"
    );
}

#[test]
fn text_changes_pixels_inside_its_rectangle_and_nowhere_else() {
    let label_rect = std::cell::Cell::new(Rect::NOTHING);
    let with = render(160, 50, |ctx| {
        let p = ctx.layer_painter(egui::LayerId::background());
        let r = p.text(
            pos2(20.0, 12.0),
            egui::Align2::LEFT_TOP,
            "Mode 3  T = 0.25",
            egui::FontId::proportional(15.0),
            Color32::from_rgb(20, 20, 160),
        );
        label_rect.set(r);
    })
    .0;
    let without = render(160, 50, |_| {}).0;
    let r = label_rect.get();
    assert!(
        r.width() > 40.0 && r.height() > 8.0,
        "label rectangle {r:?}"
    );
    let mut inside = 0;
    for y in 0..50 {
        for x in 0..160 {
            let differs = px(&with, x, y) != px(&without, x, y);
            let in_rect = r.contains(pos2(x as f32 + 0.5, y as f32 + 0.5));
            if differs {
                assert!(
                    in_rect,
                    "pixel {x},{y} changed outside the label rectangle {r:?}"
                );
                inside += 1;
            }
        }
    }
    assert!(inside > 30, "only {inside} pixels changed");
}

// ----------------------------------------------------------------- frame plan and record

fn model_with(time_steps: &[f64], modes: usize) -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    let th = CaseResults {
        case_name: "TH".into(),
        time_steps: time_steps.to_vec(),
        ..Default::default()
    };
    m.case_results.insert("TH".into(), th);
    let modal = CaseResults {
        case_name: "MODAL".into(),
        modes: (1..=modes)
            .map(|n| ModalModeResult {
                mode_num: n,
                ..Default::default()
            })
            .collect(),
        ..Default::default()
    };
    m.case_results.insert("MODAL".into(), modal);
    m.case_results.insert(
        "STATIC".into(),
        CaseResults {
            case_name: "STATIC".into(),
            ..Default::default()
        },
    );
    m
}

fn settings(kind: VideoKind) -> VideoSettings {
    VideoSettings {
        kind,
        width: 32,
        height: 24,
        fps: 10,
        magnification: 10.0,
        path: "x.avi".into(),
    }
}

fn multi(case: &str, start: f64, end: f64, increment: f64) -> VideoSettings {
    settings(VideoKind::MultiStep {
        case: case.into(),
        start,
        end,
        increment,
    })
}

fn cyclic(case: &str, mode: usize, cycles: u32, fpc: u32) -> VideoSettings {
    settings(VideoKind::Cyclic {
        case: case.into(),
        mode,
        cycles,
        frames_per_cycle: fpc,
    })
}

#[test]
fn multi_step_plan_follows_the_stated_formula() {
    let times: Vec<f64> = (0..=10).map(|i| i as f64 * 0.1).collect();
    let m = model_with(&times, 3);
    // n = floor((end - start) / increment + 1e-9) + 1 : 1/0.2 -> 5 -> 6 frames at t = 0, .2, ..., 1
    let p = frame_plan(&m, &multi("TH", 0.0, 1.0, 0.2)).unwrap();
    assert_eq!(
        p.iter().map(|f| f.step_idx).collect::<Vec<_>>(),
        [0, 2, 4, 6, 8, 10].map(Some)
    );
    // 1/0.3 = 3.33 -> 4 frames at t = 0, .3, .6, .9
    let p = frame_plan(&m, &multi("TH", 0.0, 1.0, 0.3)).unwrap();
    assert_eq!(
        p.iter().map(|f| f.step_idx).collect::<Vec<_>>(),
        [0, 3, 6, 9].map(Some)
    );
    // a sub-range: t = 0.4, 0.6, 0.8
    let p = frame_plan(&m, &multi("TH", 0.4, 0.8, 0.2)).unwrap();
    assert_eq!(
        p.iter().map(|f| f.step_idx).collect::<Vec<_>>(),
        [4, 6, 8].map(Some)
    );
    // an increment finer than the record repeats the nearest step: t = 0, .05 (-> 0), ..., 0.1
    let p = frame_plan(&m, &multi("TH", 0.0, 0.1, 0.025)).unwrap();
    assert_eq!(p.len(), 5);
    assert_eq!(p[0].step_idx, Some(0));
    assert_eq!(p[4].step_idx, Some(1));
    assert!(p.iter().all(|f| f.phase == 0.0));
}

#[test]
fn cyclic_plan_has_cycles_times_frames_poses_with_uniform_phases() {
    let m = model_with(&[], 3);
    let p = frame_plan(&m, &cyclic("MODAL", 2, 2, 4)).unwrap();
    assert_eq!(p.len(), 8);
    let want = [0.0, 0.5, 1.0, 1.5, 0.0, 0.5, 1.0, 1.5];
    for (f, w) in p.iter().zip(want) {
        assert_eq!(f.step_idx, None);
        assert!(
            (f.phase - w * std::f64::consts::PI).abs() < 1e-12,
            "phase {} want {w} pi",
            f.phase
        );
    }
    assert_eq!(duration_seconds(p.len(), 10), 0.8);
}

fn refused(m: &SapModel, s: VideoSettings, needle: &str) {
    let e = frame_plan(m, &s).expect_err(needle);
    assert!(e.contains(needle), "'{e}' does not mention '{needle}'");
}

#[test]
fn frame_plan_refusals_name_the_reason() {
    let times: Vec<f64> = (0..=10).map(|i| i as f64 * 0.1).collect();
    let m = model_with(&times, 3);
    refused(&m, multi("STATIC", 0.0, 1.0, 0.1), "no saved steps");
    refused(&m, multi("NOPE", 0.0, 1.0, 0.1), "'NOPE' has no results");
    refused(&m, multi("TH", 0.5, 0.5, 0.1), "empty or reversed");
    refused(&m, multi("TH", 0.8, 0.2, 0.1), "empty or reversed");
    refused(&m, multi("TH", 0.0, 1.0, 0.0), "increment");
    refused(&m, multi("TH", 0.0, 1.0, -0.1), "increment");
    refused(&m, multi("TH", 0.0, 1.5, 0.1), "outside the saved steps");
    refused(&m, multi("TH", -0.2, 1.0, 0.1), "outside the saved steps");
    refused(&m, multi("TH", 0.0, f64::NAN, 0.1), "finite");
    refused(&m, multi("TH", 0.0, 1.0, 1e-7), "exceed");
    refused(&m, cyclic("MODAL", 4, 1, 8), "mode 4 is not a mode");
    refused(&m, cyclic("MODAL", 0, 1, 8), "mode 0 is not a mode");
    refused(&m, cyclic("TH", 1, 1, 8), "no mode shapes");
    refused(&m, cyclic("MODAL", 1, 0, 8), "cycles");
    refused(&m, cyclic("MODAL", 1, 1, 1), "frames per cycle");
    let mut s = cyclic("MODAL", 1, 1, 8);
    s.width = 0;
    refused(&m, s, "frame size");
    let mut s = cyclic("MODAL", 1, 1, 8);
    s.height = 4097;
    refused(&m, s, "frame size");
    let mut s = cyclic("MODAL", 1, 1, 8);
    s.fps = 0;
    refused(&m, s, "frames per second");
    let mut s = cyclic("MODAL", 1, 1, 8);
    s.magnification = 0.0;
    refused(&m, s, "magnification");
    let mut s = cyclic("MODAL", 1, 1, 8);
    s.path = " ".into();
    refused(&m, s, "file name");
    // 4096 x 4096 with 30 frames is 1.4 GiB
    let mut s = cyclic("MODAL", 1, 3, 10);
    s.width = 4096;
    s.height = 4096;
    refused(&m, s, "1 GiB");
}

#[test]
fn static_multi_step_cases_use_their_load_factors() {
    let mut m = SapModel::new(Units::KnMC);
    let steps = [0.0, 0.25, 0.5, 1.0]
        .iter()
        .enumerate()
        .map(|(i, f)| sap_model::hinge::NonlinearStaticStep {
            step: i,
            load_factor: *f,
            ..Default::default()
        })
        .collect();
    m.case_results.insert(
        "PUSH".into(),
        CaseResults {
            case_name: "PUSH".into(),
            static_steps: steps,
            ..Default::default()
        },
    );
    let p = frame_plan(&m, &multi("PUSH", 0.0, 1.0, 0.5)).unwrap();
    assert_eq!(
        p.iter().map(|f| f.step_idx).collect::<Vec<_>>(),
        [Some(0), Some(2), Some(3)]
    );
    assert_eq!(eligible_cases(&m, false), ["PUSH"]);
    assert!(eligible_cases(&m, true).is_empty());
}

#[test]
fn record_paints_each_pose_and_writes_a_readable_avi() {
    let m = model_with(&[], 2);
    let s = cyclic("MODAL", 1, 1, 4);
    let poses = frame_plan(&m, &s).unwrap();
    let mut seen = Vec::new();
    let bytes = record(&s, &poses, &mut |ctx, pose| {
        seen.push(pose.phase);
        // a square whose x position follows sin(phase)
        let p = ctx.layer_painter(egui::LayerId::background());
        let x = 12.0 + 8.0 * pose.phase.sin() as f32;
        p.rect_filled(
            Rect::from_min_size(pos2(x, 8.0), vec2(4.0, 4.0)),
            0.0,
            Color32::from_rgb(255, 0, 0),
        );
    })
    .unwrap();
    // the first pose is painted twice (warm-up), then once per frame
    assert_eq!(seen.len(), poses.len() + 1);
    assert_eq!(bytes.len() as u64, super::avi::file_bytes(32, 24, 4));
    let mut names = Vec::new();
    walk(&bytes[12..], &mut names, "");
    assert_eq!(
        names.iter().filter(|n| n.as_str() == "movi/00db").count(),
        4
    );
    // frame k's red square: bottom-up BGR rows of frame k in the 'movi' list
    let frame = super::avi::frame_bytes(32, 24);
    let first_chunk = 12 + 200 + 12 + 8; // data of the first 00db
    let red_x = |k: usize| -> Vec<usize> {
        let data = &bytes[first_chunk + k * (8 + frame)..][..frame];
        let stride = super::avi::row_stride(32);
        let row = 24 - 1 - 9; // image row 9 is stored at bottom-up position 14
        (0..32)
            .filter(|&x| data[row * stride + 3 * x..][..3] == [0, 0, 255])
            .collect()
    };
    // phases 0, pi/2, pi, 3 pi/2: x = 12, 20, 12, 4 and the square is 4 wide
    assert_eq!(red_x(0), vec![12, 13, 14, 15]);
    assert_eq!(red_x(1), vec![20, 21, 22, 23]);
    assert_eq!(red_x(3), vec![4, 5, 6, 7]);
    // an empty plan and bad settings are refused
    assert!(record(&s, &[], &mut |_, _| {})
        .unwrap_err()
        .contains("at least one frame"));
    let mut bad = s.clone();
    bad.fps = 0;
    assert!(record(&bad, &poses, &mut |_, _| {}).is_err());
}

#[test]
fn save_writes_the_whole_file_or_nothing() {
    let dir = std::env::temp_dir().join(format!("structurax_video_save_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let mut s = cyclic("MODAL", 1, 1, 4);
    s.path = dir.join("out.avi").to_string_lossy().to_string();
    save(&s, b"abc").unwrap();
    assert_eq!(std::fs::read(dir.join("out.avi")).unwrap(), b"abc");
    assert!(!dir.join("out.avi.part").exists());
    s.path = dir
        .join("missing_dir")
        .join("out.avi")
        .to_string_lossy()
        .to_string();
    assert!(save(&s, b"abc").unwrap_err().contains("cannot write"));
    let _ = std::fs::remove_dir_all(&dir);
}

// ------------------------------------------------------------------------------- the form

#[test]
fn form_defaults_and_settings_come_from_the_model() {
    let times: Vec<f64> = (0..=10).map(|i| i as f64 * 0.1).collect();
    let m = model_with(&times, 3);
    let mut f = VideoForm::default();
    assert!(!f.is_open());
    f.open(&m, false);
    assert!(f.is_open());
    let s = form_settings(&f, &m).unwrap();
    // The default is the automatic deformed-shape factor of the case (D45):
    // this model has no displacement, for which the factor is 1.
    assert_eq!(s.magnification, 1.0);
    match s.kind {
        VideoKind::MultiStep {
            case,
            start,
            end,
            increment,
        } => {
            assert_eq!(case, "TH");
            assert_eq!((start, end), (0.0, 1.0));
            assert!((increment - 0.1).abs() < 1e-12);
        }
        other => panic!("{other:?}"),
    }
    assert!(
        summary(&f, &m).starts_with("11 frames, 1.10 s at 10 frames per second"),
        "{}",
        summary(&f, &m)
    );
    let mut f = VideoForm::default();
    f.open(&m, true);
    let s = form_settings(&f, &m).unwrap();
    assert_eq!(
        s.kind,
        VideoKind::Cyclic {
            case: "MODAL".into(),
            mode: 1,
            cycles: 3,
            frames_per_cycle: 20
        }
    );
    // 60 frames at 10 fps
    assert!(
        summary(&f, &m).starts_with("60 frames, 6.00 s"),
        "{}",
        summary(&f, &m)
    );
    // a model without recordable cases: the form opens but cannot produce settings
    let empty = SapModel::new(Units::KnMC);
    f.open(&empty, true);
    assert!(summary(&f, &empty).starts_with("Cannot record"));
    assert!(form_settings(&f, &empty).is_err());
}

#[test]
fn window_renders_fields_and_the_fidelity_note() {
    let m = model_with(&[0.0, 0.5, 1.0], 2);
    let mut form = VideoForm::default();
    form.open(&m, false);
    let ctx = egui::Context::default();
    let mut texts = String::new();
    for _ in 0..2 {
        let out = ctx.run(
            egui::RawInput {
                screen_rect: Some(full(900, 700)),
                ..Default::default()
            },
            |c| {
                assert!(window(c, &mut form, &m).is_none());
            },
        );
        for cs in &out.shapes {
            fn walk_shape(s: &egui::epaint::Shape, t: &mut String) {
                match s {
                    egui::epaint::Shape::Vec(v) => v.iter().for_each(|x| walk_shape(x, t)),
                    egui::epaint::Shape::Text(tx) => {
                        t.push_str(&tx.galley.job.text);
                        t.push('\n');
                    }
                    _ => {}
                }
            }
            walk_shape(&cs.shape, &mut texts);
        }
    }
    for want in [
        "Load case",
        "Start (time)",
        "Time increment",
        "Magnification",
        "Frames per second",
        "AVI file name",
        "OK",
        "Cancel",
        "not pixel for pixel",
        "3 frames",
    ] {
        assert!(texts.contains(want), "form text lacks '{want}':\n{texts}");
    }
    // a closed form paints nothing and returns nothing
    form.close();
    let out = ctx.run(
        egui::RawInput {
            screen_rect: Some(full(900, 700)),
            ..Default::default()
        },
        |c| {
            assert!(window(c, &mut form, &m).is_none());
        },
    );
    assert!(out.shapes.is_empty());
}

/// Writes a small real video to the directory named by `STRUCTURAX_VIDEO_SAMPLE_DIR`
/// (nothing is written, and the test passes, when the variable is not set).
#[test]
fn sample_video_for_visual_inspection() {
    let Ok(dir) = std::env::var("STRUCTURAX_VIDEO_SAMPLE_DIR") else {
        return;
    };
    let m = model_with(&[], 2);
    let s = VideoSettings {
        width: 320,
        height: 200,
        fps: 10,
        ..cyclic("MODAL", 1, 1, 8)
    };
    let poses = frame_plan(&m, &s).unwrap();
    let bytes = record(&s, &poses, &mut |ctx, pose| {
        let p = ctx.layer_painter(egui::LayerId::background());
        p.rect_filled(full(320, 200), 0.0, Color32::from_rgb(245, 245, 235));
        let d = 25.0 * pose.phase.sin() as f32;
        let pts = [
            pos2(60.0, 160.0),
            pos2(60.0 + d, 90.0),
            pos2(160.0 + d, 50.0),
            pos2(260.0 + d, 90.0),
            pos2(260.0, 160.0),
        ];
        for w in pts.windows(2) {
            p.line_segment(
                [w[0], w[1]],
                egui::Stroke::new(3.0_f32, Color32::from_rgb(30, 90, 200)),
            );
        }
        for q in pts {
            p.circle_filled(q, 5.0, Color32::from_rgb(200, 60, 40));
        }
        p.rect_filled(
            Rect::from_min_size(pos2(20.0, 20.0), vec2(60.0, 20.0)),
            3.0,
            Color32::from_rgba_premultiplied(0, 90, 0, 120),
        );
        p.text(
            pos2(10.0, 170.0),
            egui::Align2::LEFT_TOP,
            format!("Mode 1  phase {:.2} rad", pose.phase),
            egui::FontId::proportional(14.0),
            Color32::BLACK,
        );
    })
    .unwrap();
    std::fs::write(format!("{dir}/sample.avi"), bytes).unwrap();
}
