use super::*;
use sap_model::{AreaSection, ModalModeResult};

fn model_with_section() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    m.area_sections.insert("S".into(), AreaSection::default());
    m
}

fn square_area(m: &mut SapModel, name: &str, pts: [[f64; 3]; 4]) {
    let mut names = Vec::new();
    for (i, p) in pts.iter().enumerate() {
        names.push(m.add_point(format!("{name}{i}"), p[0], p[1], p[2]));
    }
    m.add_area(name, names, "S");
}

fn results_with(m: &SapModel, u: [f64; 6]) -> CaseResults {
    let mut cr = CaseResults::default();
    for k in m.points.keys() {
        cr.joint_displacements.insert(k.clone(), u);
    }
    cr
}

const SEL: StepSelection = StepSelection { mode_idx: 0, use_step: false, step_idx: 0 };

// ---------------------------------------------------------------- value_color

/// The function as it stood in gui.rs (`normalized_contour_color`), copied verbatim.
fn reference_color(val: f64, lo: f64, hi: f64, alpha: u8) -> Color32 {
    if !val.is_finite() || !lo.is_finite() || !hi.is_finite() {
        return Color32::from_rgba_unmultiplied(90, 110, 140, alpha);
    }
    let span = hi - lo;
    let t = if span.abs() > 1e-15 { ((val - lo) / span).clamp(0.0, 1.0) as f32 } else { 0.5 };
    let (r, g, b) = if t < 0.25 {
        let s = t / 0.25;
        (40.0, 90.0 + 150.0 * s, 245.0)
    } else if t < 0.50 {
        let s = (t - 0.25) / 0.25;
        (40.0 + 40.0 * s, 240.0, 245.0 * (1.0 - s) + 90.0 * s)
    } else if t < 0.75 {
        let s = (t - 0.50) / 0.25;
        (80.0 + 175.0 * s, 240.0 - 20.0 * s, 90.0 * (1.0 - s))
    } else {
        let s = (t - 0.75) / 0.25;
        (255.0, 220.0 * (1.0 - 0.72 * s), 45.0 * (1.0 - s))
    };
    Color32::from_rgba_unmultiplied(r as u8, g as u8, b as u8, alpha)
}

#[test]
fn value_color_is_bitwise_the_old_ramp() {
    let mut n = 0;
    for k in 0..=200 {
        let v = -1.0 + 2.0 * k as f64 / 200.0;
        for alpha in [0u8, 115, 255] {
            assert_eq!(value_color(v, -1.0, 1.0, alpha), reference_color(v, -1.0, 1.0, alpha));
            n += 1;
        }
    }
    assert_eq!(n, 603);
    for (v, lo, hi) in [(f64::NAN, 0.0, 1.0), (0.0, f64::NEG_INFINITY, 1.0), (3.0, 3.0, 3.0), (-5.0, 0.0, 1.0)]
    {
        assert_eq!(value_color(v, lo, hi, 9), reference_color(v, lo, hi, 9));
    }
}

#[test]
fn value_color_ramp_anchor_points() {
    // t = 0: (40, 90, 245); t = 0.25: branch 2 with s = 0: (40, 240, 245);
    // t = 0.5: branch 3 with s = 0: (80, 240, 90); t = 0.75: branch 4, s = 0:
    // (255, 220, 45); t = 1: (255, 220 * 0.28 = 61.6 -> 61, 0).
    let c = |v: f64| value_color(v, 0.0, 1.0, 200);
    assert_eq!(c(0.0), Color32::from_rgba_unmultiplied(40, 90, 245, 200));
    assert_eq!(c(0.25), Color32::from_rgba_unmultiplied(40, 240, 245, 200));
    assert_eq!(c(0.5), Color32::from_rgba_unmultiplied(80, 240, 90, 200));
    assert_eq!(c(0.75), Color32::from_rgba_unmultiplied(255, 220, 45, 200));
    assert_eq!(c(1.0), Color32::from_rgba_unmultiplied(255, 61, 0, 200));
    assert_eq!(c(f64::NAN), Color32::from_rgba_unmultiplied(90, 110, 140, 200));
}

// ------------------------------------------------------- joint displacements

#[test]
fn global_displacement_of_a_joint_turned_30_degrees() {
    // local_axes_deg A = 30 is a rotation about Z: columns of R are the local
    // axes, j1 = (cos 30, sin 30, 0), j2 = (-sin 30, cos 30, 0), and
    // u_global = R u_local (documented in sap_solver::frame).
    let mut m = SapModel::new(Units::KnMC);
    m.add_point("J", 0.0, 0.0, 0.0);
    m.points.get_mut("J").unwrap().local_axes_deg = [30.0, 0.0, 0.0];
    let (s, c) = 30f64.to_radians().sin_cos();
    let mut cr = CaseResults::default();
    // U1 = 1 and R1 = 1 -> (cos 30, sin 30, 0) for the translation and the rotation.
    cr.joint_displacements.insert("J".into(), [1.0, 0.0, 0.0, 1.0, 0.0, 0.0]);
    let g = joint_displacement_global(&m, &cr, "J", SEL).unwrap();
    assert_eq!(g, [c, s, 0.0, c, s, 0.0]);
    // U2 = 2, R2 = 3 -> 2 j2 = (-2 sin 30, 2 cos 30, 0) and 3 j2. A transposed
    // matrix would give (+sin 30, cos 30).
    cr.joint_displacements.insert("J".into(), [0.0, 2.0, 5.0, 0.0, 3.0, 7.0]);
    let g = joint_displacement_global(&m, &cr, "J", SEL).unwrap();
    let expect = [-2.0 * s, 2.0 * c, 5.0, -3.0 * s, 3.0 * c, 7.0];
    for i in 0..6 {
        assert!((g[i] - expect[i]).abs() < 1e-15, "{i}: {} vs {}", g[i], expect[i]);
    }
}

#[test]
fn global_displacement_with_an_explicit_axes_matrix() {
    // Columns j1 = (0,1,0), j2 = (0,0,1), j3 = (1,0,0). u_local = (1,2,3) gives
    // 1 j1 + 2 j2 + 3 j3 = (3, 1, 2); (4,5,6) gives (6, 4, 5).
    let mut m = SapModel::new(Units::KnMC);
    m.add_point("J", 0.0, 0.0, 0.0);
    m.points.get_mut("J").unwrap().local_axes_matrix =
        Some([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]);
    let mut cr = CaseResults::default();
    cr.joint_displacements.insert("J".into(), [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    assert_eq!(joint_displacement_global(&m, &cr, "J", SEL).unwrap(), [3.0, 1.0, 2.0, 6.0, 4.0, 5.0]);
    // Default axes: unchanged. Unknown joint or no result: None.
    m.points.get_mut("J").unwrap().local_axes_matrix = None;
    assert_eq!(
        joint_displacement_global(&m, &cr, "J", SEL).unwrap(),
        [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    );
    assert!(joint_displacement_global(&m, &cr, "X", SEL).is_none());
    assert!(joint_displacement_global(&m, &CaseResults::default(), "J", SEL).is_none());
}

#[test]
fn global_displacement_follows_the_step_and_mode_selection() {
    let mut m = SapModel::new(Units::KnMC);
    m.add_point("J", 0.0, 0.0, 0.0);
    let mut cr = CaseResults::default();
    cr.modes.push(ModalModeResult {
        mode_shape: [("J".to_string(), [9.0, 0.0, 0.0, 0.0, 0.0, 0.0])].into_iter().collect(),
        ..Default::default()
    });
    let g = joint_displacement_global(&m, &cr, "J", SEL).unwrap();
    assert_eq!(g[0], 9.0);
}

// -------------------------------------------------------------- area axes

#[test]
fn area_axes_of_a_horizontal_area_and_its_angle() {
    // Corners (0,0,0) (1,0,0) (1,1,0) (0,1,0): v_xi = (2,0,0), v_eta = (0,2,0),
    // e3 = (0,0,1); horizontal default: e1 = X, e2 = Y. Angle 90 turns local 1
    // to +Y and local 2 to -X (e1 = cos b e1_0 + sin b e2_0).
    let mut m = model_with_section();
    square_area(&mut m, "A", [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.]]);
    let e = area_local_axes(&m, "A").unwrap();
    assert_eq!(e, [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
    let cr = results_with(&m, [1.0, 2.0, 3.0, 0.0, 0.0, 0.0]);
    let v = |m: &SapModel, c| area_vertex_values(m, &cr, "A", c, SEL).unwrap();
    assert_eq!(v(&m, DisplacementComponent::Area3), vec![3.0; 4]);
    assert_eq!(v(&m, DisplacementComponent::Ux), vec![1.0; 4]);
    assert_eq!(v(&m, DisplacementComponent::Uy), vec![2.0; 4]);
    assert_eq!(v(&m, DisplacementComponent::Uz), vec![3.0; 4]);
    m.areas.get_mut("A").unwrap().angle_deg = 90.0;
    for (c, want) in [
        (DisplacementComponent::Area1, 2.0),  // Area 1 = Uy
        (DisplacementComponent::Area2, -1.0), // Area 2 = -Ux
        (DisplacementComponent::Area3, 3.0),
    ] {
        for x in v(&m, c) {
            assert!((x - want).abs() < 1e-15, "{c:?}: {x}");
        }
    }
}

#[test]
fn area_axes_of_a_vertical_xz_area_follow_the_right_hand_rule() {
    // (0,0,0) (1,0,0) (1,0,1) (0,0,1): v_xi = (2,0,0), v_eta = (0,0,2),
    // cross = (0,-4,0), e3 = -Y; not horizontal (h = 1): e2 = (0,0,1),
    // e1 = e2 x e3 = (1,0,0). Area 3 = -Uy. Reversed corner order flips e3.
    let mut m = model_with_section();
    square_area(&mut m, "A", [[0., 0., 0.], [1., 0., 0.], [1., 0., 1.], [0., 0., 1.]]);
    let e = area_local_axes(&m, "A").unwrap();
    assert_eq!(e, [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]]);
    let cr = results_with(&m, [1.0, 2.0, 3.0, 0.0, 0.0, 0.0]);
    assert_eq!(
        area_vertex_values(&m, &cr, "A", DisplacementComponent::Area3, SEL).unwrap(),
        vec![-2.0; 4]
    );
    square_area(&mut m, "B", [[0., 0., 0.], [0., 0., 1.], [1., 0., 1.], [1., 0., 0.]]);
    let cr = results_with(&m, [1.0, 2.0, 3.0, 0.0, 0.0, 0.0]);
    assert_eq!(
        area_vertex_values(&m, &cr, "B", DisplacementComponent::Area3, SEL).unwrap(),
        vec![2.0; 4]
    );
}

#[test]
fn area_axes_of_a_tilted_area_match_hand_values_and_the_solver() {
    // (0,0,0) (1,0,0) (1,1,1) (0,1,1): v_xi = (2,0,0), v_eta = (0,2,2),
    // cross = (0,-4,4), e3 = (0,-1,1)/sqrt2; h = 1/sqrt2,
    // e2 = (0, 1/sqrt2, 1/sqrt2), e1 = e2 x e3 = (1,0,0).
    let mut m = model_with_section();
    square_area(&mut m, "T", [[0., 0., 0.], [1., 0., 0.], [1., 1., 1.], [0., 1., 1.]]);
    let r = std::f64::consts::FRAC_1_SQRT_2;
    let e = area_local_axes(&m, "T").unwrap();
    let want = [[1.0, 0.0, 0.0], [0.0, r, r], [0.0, -r, r]];
    for i in 0..3 {
        for k in 0..3 {
            assert!((e[i][k] - want[i][k]).abs() < 1e-15, "{i}{k}: {}", e[i][k]);
        }
    }
    // Angle: the element axes turn by area.angle_deg alone (here 30); the section's material
    // angle (20) turns the material inside the element, not these axes (Analysis Reference
    // p. 193): e1 = cos30 e1_0 + sin30 e2_0, e2 = -sin30 e1_0 + cos30 e2_0.
    m.areas.get_mut("T").unwrap().angle_deg = 30.0;
    m.area_sections.get_mut("S").unwrap().mat_angle_deg = 20.0;
    let e = area_local_axes(&m, "T").unwrap();
    let (s, c) = 30f64.to_radians().sin_cos();
    let want = [[c, s * r, s * r], [-s, c * r, c * r], [0.0, -r, r]];
    for i in 0..3 {
        for k in 0..3 {
            assert!((e[i][k] - want[i][k]).abs() < 1e-15, "{i}{k}: {}", e[i][k]);
        }
    }
    // The solver's own function with the same inputs.
    let coords = [[0., 0., 0.], [1., 0., 0.], [1., 1., 1.], [0., 1., 1.]];
    assert_eq!(e, area_local_system(&coords, 30.0).0);
    // Rows are orthonormal and right-handed.
    for i in 0..3 {
        for k in 0..3 {
            let d: f64 = (0..3).map(|j| e[i][j] * e[k][j]).sum();
            assert!((d - if i == k { 1.0 } else { 0.0 }).abs() < 1e-15);
        }
    }
}

#[test]
fn area_axes_of_a_triangle_use_its_three_corners() {
    // (0,0,0) (2,0,0) (0,3,0): v_xi = (2,0,0), v_eta = (0,3,0): e3 = +Z.
    let mut m = model_with_section();
    for (n, p) in [("a", [0., 0., 0.]), ("b", [2., 0., 0.]), ("c", [0., 3., 0.])] {
        m.add_point(n, p[0], p[1], p[2]);
    }
    m.add_area("T", vec!["a".into(), "b".into(), "c".into()], "S");
    assert_eq!(
        area_local_axes(&m, "T").unwrap(),
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    );
    let cr = results_with(&m, [0.0, 0.0, 4.0, 0.0, 0.0, 0.0]);
    assert_eq!(
        area_vertex_values(&m, &cr, "T", DisplacementComponent::Uz, SEL).unwrap(),
        vec![4.0; 3]
    );
}

#[test]
fn area_values_use_global_components_of_a_rotated_joint_and_refuse_missing_input() {
    let mut m = model_with_section();
    square_area(&mut m, "A", [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.]]);
    // Joint A0 turned 90 degrees about Z: its local 1 is global +Y, so a local
    // U1 = 1 is global Uy = 1 (cos 90 ~ 6e-17: tolerance 1e-15).
    m.points.get_mut("A0").unwrap().local_axes_deg = [90.0, 0.0, 0.0];
    let cr = results_with(&m, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    let ux = area_vertex_values(&m, &cr, "A", DisplacementComponent::Ux, SEL).unwrap();
    let uy = area_vertex_values(&m, &cr, "A", DisplacementComponent::Uy, SEL).unwrap();
    assert!(ux[0].abs() < 1e-15 && (uy[0] - 1.0).abs() < 1e-15);
    assert_eq!((ux[1], uy[1]), (1.0, 0.0));

    // A corner without a result is an error that names the joint, not a zero.
    let mut cr2 = cr.clone();
    cr2.joint_displacements.shift_remove("A2");
    let e = area_vertex_values(&m, &cr2, "A", DisplacementComponent::Ux, SEL).unwrap_err();
    assert!(e.contains("'A2'") && e.contains("'A'"), "{e}");
    assert!(area_vertex_values(&m, &cr, "nope", DisplacementComponent::Ux, SEL).is_err());
    assert!(area_vertex_values(&m, &cr, "nope", DisplacementComponent::Area1, SEL).is_err());
    assert!(area_local_axes(&m, "nope").unwrap_err().contains("'nope'"));
    m.areas.get_mut("A").unwrap().section = "missing".into();
    assert!(area_local_axes(&m, "A").unwrap_err().contains("'missing'"));
    m.areas.get_mut("A").unwrap().section = "S".into();
    m.points.shift_remove("A1");
    assert!(area_local_axes(&m, "A").unwrap_err().contains("'A1'"));
}

#[test]
fn an_auto_meshed_area_contours_its_own_corners() {
    let mut m = model_with_section();
    square_area(&mut m, "A", [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.]]);
    m.areas.get_mut("A").unwrap().auto_mesh = (4, 4);
    let cr = results_with(&m, [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
    let v = area_vertex_values(&m, &cr, "A", DisplacementComponent::Uz, SEL).unwrap();
    assert_eq!(v.len(), 4);
}

// -------------------------------------------------------------- range, ticks

#[test]
fn contour_range_automatic_user_and_errors() {
    let r = contour_range(&[3.0, f64::NAN, -2.0, 5.0, f64::INFINITY], (0.0, 0.0)).unwrap();
    assert_eq!(r, Some(ContourRange { lo: -2.0, hi: 5.0 }));
    assert_eq!(contour_range(&[], (0.0, 0.0)).unwrap(), None);
    assert_eq!(contour_range(&[f64::NAN], (0.0, 0.0)).unwrap(), None);
    assert_eq!(
        contour_range(&[7.0], (0.0, 0.0)).unwrap(),
        Some(ContourRange { lo: 7.0, hi: 7.0 })
    );
    // A user range is used as given, whatever the values.
    assert_eq!(
        contour_range(&[3.0], (-1.0, 0.0)).unwrap(),
        Some(ContourRange { lo: -1.0, hi: 0.0 })
    );
    assert_eq!(
        contour_range(&[], (1.0, 2.0)).unwrap(),
        Some(ContourRange { lo: 1.0, hi: 2.0 })
    );
    for bad in [(2.0, 1.0), (1.0, 1.0), (0.0, -1.0), (f64::NAN, 1.0), (0.0, f64::INFINITY)] {
        let e = contour_range(&[1.0], bad).unwrap_err();
        assert_eq!(e, "the contour minimum must be below the maximum", "{bad:?}");
    }
}

#[test]
fn legend_ticks_are_equally_spaced_and_end_at_hi() {
    let t = legend_ticks(ContourRange { lo: 0.0, hi: 8.0 }, 8);
    assert_eq!(t, vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]);
    // lo + (hi - lo) k / n = -1 + 2k/10.
    let t = legend_ticks(ContourRange { lo: -1.0, hi: 1.0 }, LEGEND_INTERVALS);
    assert_eq!(t.len(), 11);
    for (k, v) in t.iter().enumerate() {
        assert!((v - (-1.0 + 0.2 * k as f64)).abs() < 1e-15, "{k}: {v}");
    }
    assert_eq!(t[0], -1.0);
    assert_eq!(t[10], 1.0);
    let odd = legend_ticks(ContourRange { lo: 0.1, hi: 0.7 }, 3);
    assert_eq!(*odd.last().unwrap(), 0.7);
    assert!(legend_ticks(ContourRange { lo: 0.0, hi: 1.0 }, 0).is_empty());
}

// ---------------------------------------------------------------- the mesh

fn mesh_area(m: &egui::Mesh, p: &[Pos2]) -> f32 {
    m.indices
        .chunks(3)
        .map(|t| {
            let (a, b, c) = (m.vertices[t[0] as usize].pos, m.vertices[t[1] as usize].pos, m.vertices[t[2] as usize].pos);
            let _ = p;
            ((b - a).x * (c - a).y - (b - a).y * (c - a).x).abs() / 2.0
        })
        .sum()
}

#[test]
fn quad_mesh_counts_positions_values_and_colours() {
    let sq = [Pos2::new(0.0, 0.0), Pos2::new(2.0, 0.0), Pos2::new(2.0, 2.0), Pos2::new(0.0, 2.0)];
    let vals = [0.0, 1.0, 2.0, 3.0];
    let range = ContourRange { lo: 0.0, hi: 3.0 };
    for s in 1..=5usize {
        let m = colored_polygon(&sq, &vals, range, 150, s);
        assert_eq!(m.vertices.len(), (s + 1) * (s + 1), "s = {s}");
        assert_eq!(m.indices.len(), 3 * 2 * s * s, "s = {s}");
        assert!(m.is_valid());
        assert!(m.indices.iter().all(|&i| (i as usize) < m.vertices.len()));
        assert!((mesh_area(&m, &sq) - 4.0).abs() < 1e-4, "triangles tile the square");
        // Value of a vertex at (x, y) in [0,2]^2: bilinear
        // (1-u)(1-v)*0 + u(1-v)*1 + u v*2 + (1-u) v*3 with u = x/2, v = y/2.
        // Vertex k sits at i = k mod (s+1), j = k div (s+1): u = i/s, w = j/s.
        for (k, v) in m.vertices.iter().enumerate() {
            let (u, w) = ((k % (s + 1)) as f64 / s as f64, (k / (s + 1)) as f64 / s as f64);
            assert!((v.pos.x as f64 - 2.0 * u).abs() < 1e-6 && (v.pos.y as f64 - 2.0 * w).abs() < 1e-6);
            let val = u * (1.0 - w) + 2.0 * u * w + 3.0 * (1.0 - u) * w;
            assert_eq!(v.color, value_color(val, 0.0, 3.0, 150), "s = {s}, vertex {k}");
        }
    }
    // s = 2: nine vertices, eight triangles, centre (vertex 4) at (1,1): value
    // (0+1+2+3)/4 = 1.5; edge midpoint (1,0) (vertex 1): (0+1)/2 = 0.5.
    let m = colored_polygon(&sq, &vals, range, 255, 2);
    assert_eq!((m.vertices.len(), m.indices.len() / 3), (9, 8));
    assert_eq!(m.vertices[4].pos, Pos2::new(1.0, 1.0));
    assert_eq!(m.vertices[4].color, value_color(1.5, 0.0, 3.0, 255));
    assert_eq!(m.vertices[1].pos, Pos2::new(1.0, 0.0));
    assert_eq!(m.vertices[1].color, value_color(0.5, 0.0, 3.0, 255));
    // Corners keep their own values.
    assert_eq!(m.vertices[0].color, value_color(0.0, 0.0, 3.0, 255));
    assert_eq!(m.vertices[2].color, value_color(1.0, 0.0, 3.0, 255));
    assert_eq!(m.vertices[8].color, value_color(2.0, 0.0, 3.0, 255));
    assert_eq!(m.vertices[6].color, value_color(3.0, 0.0, 3.0, 255));
}

#[test]
fn triangle_mesh_counts_positions_values_and_colours() {
    let tri = [Pos2::new(0.0, 0.0), Pos2::new(4.0, 0.0), Pos2::new(0.0, 4.0)];
    let vals = [0.0, 4.0, 8.0];
    let range = ContourRange { lo: 0.0, hi: 8.0 };
    for s in 1..=5usize {
        let m = colored_polygon(&tri, &vals, range, 150, s);
        assert_eq!(m.vertices.len(), (s + 1) * (s + 2) / 2, "s = {s}");
        assert_eq!(m.indices.len(), 3 * s * s, "s = {s}");
        assert!(m.is_valid());
        assert!((mesh_area(&m, &tri) - 8.0).abs() < 1e-4, "triangles tile the triangle");
        // Barycentric: value = 4 x/4*1 + 8 y/4 = x + 2y at the point (x, y).
        // Rows j = 0..=s hold s+1-j vertices i = 0..=s-j: value (4i + 8j)/s.
        let mut k = 0;
        for j in 0..=s {
            for i in 0..=(s - j) {
                let v = &m.vertices[k];
                let (x, y) = (4.0 * i as f64 / s as f64, 4.0 * j as f64 / s as f64);
                assert!((v.pos.x as f64 - x).abs() < 1e-6 && (v.pos.y as f64 - y).abs() < 1e-6);
                let val = (4 * i + 8 * j) as f64 / s as f64;
                assert_eq!(v.color, value_color(val, 0.0, 8.0, 150), "s = {s}, vertex {k}");
                k += 1;
            }
        }
        assert_eq!(k, m.vertices.len());
    }
    // s = 2: six vertices, four triangles; vertex 4 = (i 1, j 1) at (2,2),
    // value 0.5*4 + 0.5*8 = 6; vertex 3 = (0,1) at (0,2), value 4.
    let m = colored_polygon(&tri, &vals, range, 255, 2);
    assert_eq!((m.vertices.len(), m.indices.len() / 3), (6, 4));
    assert_eq!(m.vertices[4].pos, Pos2::new(2.0, 2.0));
    assert_eq!(m.vertices[4].color, value_color(6.0, 0.0, 8.0, 255));
    assert_eq!(m.vertices[3].pos, Pos2::new(0.0, 2.0));
    assert_eq!(m.vertices[3].color, value_color(4.0, 0.0, 8.0, 255));
}

#[test]
fn polygon_mesh_refuses_degenerate_input_with_an_empty_mesh() {
    let p = |n: usize| -> Vec<Pos2> { (0..n).map(|i| Pos2::new(i as f32, (i * i) as f32)).collect() };
    let range = ContourRange { lo: 0.0, hi: 1.0 };
    assert!(colored_polygon(&p(4), &[0.0; 4], range, 255, 0).is_empty());
    assert!(colored_polygon(&p(3), &[0.0; 3], range, 255, 0).is_empty());
    for n in [0usize, 1, 2, 5, 6] {
        assert!(colored_polygon(&p(n), &vec![0.0; n], range, 255, 4).is_empty(), "n = {n}");
    }
    assert!(colored_polygon(&p(4), &[0.0; 3], range, 255, 4).is_empty(), "length mismatch");
    // Non-finite values give the neutral colour, still a valid mesh.
    let m = colored_polygon(&p(3), &[f64::NAN, 0.0, 1.0], range, 255, 2);
    assert!(m.is_valid());
    assert_eq!(m.vertices[0].color, value_color(f64::NAN, 0.0, 1.0, 255));
}

// -------------------------------------------------------------- read-out etc.

#[test]
fn readout_is_joint_local_in_display_units() {
    let mut m = SapModel::new(Units::KnMC);
    m.add_point("J", 0.0, 0.0, 0.0);
    m.points.get_mut("J").unwrap().local_axes_deg = [30.0, 0.0, 0.0];
    let mut cr = CaseResults::default();
    cr.joint_displacements.insert("J".into(), [0.001, -0.002, 0.003, 0.1, 0.2, 0.3]);
    m.case_results.insert("D".into(), cr);
    // Stored local values, not rotated; m -> mm multiplies the translations by
    // 1000, rotations stay in rad.
    let r = joint_readout(&m, "D", "J", SEL, Units::KnMmC).unwrap();
    assert_eq!(r.joint, "J");
    let f = Units::KnMC.length_factor_to(Units::KnMmC);
    assert_eq!(f, 1000.0);
    assert_eq!(r.values, [0.001 * f, -0.002 * f, 0.003 * f, 0.1, 0.2, 0.3]);
    assert_eq!(
        r.labels.to_vec(),
        ["U1 [mm]", "U2 [mm]", "U3 [mm]", "R1 [rad]", "R2 [rad]", "R3 [rad]"]
    );
    let same = joint_readout(&m, "D", "J", SEL, Units::KnMC).unwrap();
    assert_eq!(same.values, [0.001, -0.002, 0.003, 0.1, 0.2, 0.3]);
    assert_eq!(same.labels[0], "U1 [m]");

    assert!(joint_readout(&m, "nope", "J", SEL, Units::KnMC).unwrap_err().contains("'nope'"));
    assert!(joint_readout(&m, "D", "K", SEL, Units::KnMC).unwrap_err().contains("'K'"));
    m.add_point("L", 1.0, 0.0, 0.0);
    assert!(joint_readout(&m, "D", "L", SEL, Units::KnMC).unwrap_err().contains("'L'"));
}

#[test]
fn step_count_and_animation_step() {
    use sap_model::hinge::NonlinearStaticStep;
    let mut cr = CaseResults::default();
    assert_eq!(step_count(&cr), 0);
    cr.time_steps = vec![0.0, 0.1, 0.2];
    assert_eq!(step_count(&cr), 0, "no stored history displacements");
    cr.time_history_disp.insert("J".into(), vec![[0.0; 6]; 3]);
    assert_eq!(step_count(&cr), 3);
    let step = |n| NonlinearStaticStep {
        step: n,
        load_factor: 0.0,
        monitored_displacement: None,
        base_reactions: [0.0; 6],
        joint_displacements: Default::default(),
        joint_reactions: Default::default(),
        frame_forces: Default::default(),
        hinges: Vec::new(),
    };
    let mut cr = CaseResults::default();
    cr.static_steps = vec![step(0)];
    assert_eq!(step_count(&cr), 0, "one saved state is a single state");
    cr.static_steps.push(step(1));
    cr.static_steps.push(step(2));
    cr.static_steps.push(step(3));
    assert_eq!(step_count(&cr), 4);
    cr.modes.push(ModalModeResult::default());
    assert_eq!(step_count(&cr), 0, "modal results are not stepped");

    // floor(elapsed / period) mod n.
    assert_eq!(animation_step(0.0, 4, 0.5), 0);
    assert_eq!(animation_step(0.49, 4, 0.5), 0);
    assert_eq!(animation_step(0.5, 4, 0.5), 1);
    assert_eq!(animation_step(1.99, 4, 0.5), 3);
    assert_eq!(animation_step(2.0, 4, 0.5), 0);
    assert_eq!(animation_step(5.2, 4, 0.5), 2); // floor(10.4) = 10, 10 mod 4
    assert_eq!(animation_step(7.0, 1, 0.5), 0);
    assert_eq!(animation_step(1.0, 0, 0.5), 0);
    assert_eq!(animation_step(1.0, 4, 0.0), 0);
    assert_eq!(animation_step(-1.0, 4, 0.5), 0);
    assert_eq!(animation_step(f64::NAN, 4, 0.5), 0);
    assert_eq!(animation_step(1e300, 4, 1e-300), 0);
}

// ------------------------------------------------- example 2-010 (opt-in)

#[test]
fn vendor_example_2_010_readout_and_global_components() {
    let Some(dir) = std::env::var_os("STRUCTURAX_VENDOR_FIXTURES") else {
        return;
    };
    let dir = std::path::PathBuf::from(dir);
    let mut ran = 0;
    // a: 8 x 16 mesh, b: 24 x 48. Printed U1 0.12175 / 0.12387 in; closed form
    // R sigma / E with sigma = p R cos(pi/N) / t, N = 16 / 48 around the
    // circumference: 60 * 58.8471 / 29000 = 0.121753, 60 * 59.8715 / 29000 = 0.123872.
    for (stem, n_around, printed) in [("a", 16.0f64, 0.12175), ("b", 48.0, 0.12387)] {
        for kind in ["thin", "thick"] {
            let path = dir.join(format!("example_2_010{stem}_{kind}.sdb"));
            if !path.exists() {
                eprintln!("skipped: {} not found", path.display());
                continue;
            }
            let mut model = sap_format::load_sdb_file(&path).expect("import");
            sap_solver::analyze_model(&mut model).expect("analysis");
            // sigma = p R cos(pi/N) / t with p = 1 k/in2, R = 60 in, t = 1 in; U1 = R sigma / E.
            let closed = 60.0 * (60.0 * (std::f64::consts::PI / n_around).cos()) / 29000.0;
            let cr = &model.case_results["PRES"];
            let total = cr.joint_displacements.len();
            let mut checked = 0;
            for joint in cr.joint_displacements.keys() {
                let ro = joint_readout(&model, "PRES", joint, SEL, Units::KipInF).unwrap();
                assert!((ro.values[0] - printed).abs() < 5e-6, "{joint}: U1 {}", ro.values[0]);
                assert!((ro.values[0] - closed).abs() < 1e-6 * closed, "{joint}: U1 {}", ro.values[0]);
                // The global Ux contour value is U1 cos(theta) at the joint's angle.
                let g = joint_displacement_global(&model, cr, joint, SEL).unwrap();
                let pt = &model.points[joint];
                let th = pt.y.atan2(pt.x);
                assert!((g[0] - ro.values[0] * th.cos()).abs() < 1e-9, "{joint}: Ux {}", g[0]);
                checked += 1;
            }
            assert!(checked == total && total > 0);
            eprintln!("2-010{stem} {kind}: {checked} joints, U1 {printed}, closed form {closed}");
            ran += 1;
        }
    }
    eprintln!("2-010: {ran} files analysed");
}
