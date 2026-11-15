//! End-to-end check of the local-axis rules used by `SapModel::replicate_radial`
//! and `SapModel::reverse_area_local_3` (package P6, `sap-model/src/radial.rs`).
//!
//! The rules are statements about how the SOLVER defines frame, area and joint
//! local axes, so they are tested against the solver: a structure replicated
//! about a vertical axis must respond exactly as the rotated original, with
//! displacements rotated by the same angle about Z (rotation invariance of
//! linear elasticity; closed form, no stored numbers). A negative control undoes
//! the angle transformation and requires the response to differ, so the test
//! cannot pass vacuously.
//!
//! Reversing local 3 with "same global loads" must leave the response unchanged.
//!
//! Units: kip, inch. Tolerance: 1e-9 relative to the largest displacement.

use sap_model::radial::ReverseKeep;
use sap_model::{AreaLoadUniform, FrameLoadDistributed, PointLoadForce, SapModel};
use sap_solver::analyze_model;

fn rot_z(v: [f64; 3], deg: f64) -> [f64; 3] {
    let (s, c) = deg.to_radians().sin_cos();
    [c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]]
}

fn find(m: &SapModel, c: [f64; 3]) -> String {
    m.points
        .values()
        .find(|p| {
            (p.x - c[0]).abs() < 1e-9 && (p.y - c[1]).abs() < 1e-9 && (p.z - c[2]).abs() < 1e-9
        })
        .unwrap_or_else(|| panic!("no joint at {c:?}"))
        .name
        .clone()
}

fn solved(m: &SapModel) -> sap_model::CaseResults {
    let mut m = m.clone();
    m.load_patterns.get_mut("DEAD").unwrap().self_wt_multiplier = 0.0;
    analyze_model(&mut m).expect("analysis");
    m.case_results["DEAD"].clone()
}

/// Largest deviation between the copy's displacements and the rotated
/// originals, relative to the largest original displacement.
fn rotation_mismatch(m: &SapModel, src: &SapModel, theta: f64) -> (f64, f64) {
    let res = solved(m);
    let mut worst: f64 = 0.0;
    let mut scale: f64 = 0.0;
    for p in src.points.values() {
        let c = rot_z([p.x, p.y, p.z], theta);
        let copy = find(m, c);
        let (a, b) = (
            res.joint_displacements[&p.name],
            res.joint_displacements[&copy],
        );
        let t = rot_z([a[0], a[1], a[2]], theta);
        let r = rot_z([a[3], a[4], a[5]], theta);
        for k in 0..3 {
            worst = worst.max((b[k] - t[k]).abs()).max((b[k + 3] - r[k]).abs());
            scale = scale.max(a[k].abs());
        }
    }
    (worst / scale, scale)
}

fn column(up: bool, angle: f64) -> SapModel {
    let mut m = SapModel::default();
    let (zi, zj) = if up { (0.0, 30.0) } else { (30.0, 0.0) };
    m.add_point("1", 5.0, 0.0, zi);
    m.add_point("2", 5.0, 0.0, zj);
    m.points.get_mut("1").unwrap().restraint = [true; 6];
    m.add_frame("C", "1", "2", "FSEC1");
    let f = m.frames.get_mut("C").unwrap();
    f.angle_deg = angle;
    // different loads along local 2 and local 3 so that a wrong angle shows
    for (dir, v) in [(2, 1.0), (3, 0.4)] {
        f.loads_dist.push(FrameLoadDistributed {
            load_pat: "DEAD".into(),
            my_type: 1,
            dir,
            csys: "Local".into(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: v,
            val2: v,
        });
    }
    m
}

fn select_all(m: &mut SapModel) {
    m.points.values_mut().for_each(|p| p.selected = true);
    m.frames.values_mut().for_each(|p| p.selected = true);
    m.areas.values_mut().for_each(|p| p.selected = true);
}

#[test]
fn vertical_frames_up_and_down_rotate_exactly_with_the_angle_rule() {
    for up in [true, false] {
        let mut m = column(up, 30.0);
        select_all(&mut m);
        let src = m.clone();
        let theta = 50.0;
        m.replicate_radial([0.0, 0.0], theta, 1).unwrap();
        assert_eq!(m.frames.len(), 2);
        let (err, scale) = rotation_mismatch(&m, &src, theta);
        assert!(
            scale > 1e-3,
            "the column must actually deflect (up={up}): {scale}"
        );
        assert!(err < 1e-9, "up={up}: rotation mismatch {err:e}");
        // Negative control: the same geometry with the ORIGINAL angle responds differently.
        let copy = m
            .frames
            .values()
            .find(|f| f.name != "C")
            .unwrap()
            .name
            .clone();
        m.frames.get_mut(&copy).unwrap().angle_deg = 30.0;
        let (bad, _) = rotation_mismatch(&m, &src, theta);
        assert!(
            bad > 1e-3,
            "up={up}: untransformed angle must differ, got {bad:e}"
        );
        // ... and the wrong sign for this direction differs too.
        m.frames.get_mut(&copy).unwrap().angle_deg = 30.0 + if up { -theta } else { theta };
        let (bad, _) = rotation_mismatch(&m, &src, theta);
        assert!(bad > 1e-3, "up={up}: wrong sign must differ, got {bad:e}");
    }
}

fn panel(normal_up: bool, angle: f64) -> SapModel {
    let mut m = SapModel::default();
    let pts: [[f64; 2]; 4] = [[0.0, 0.0], [20.0, 0.0], [20.0, 10.0], [0.0, 10.0]];
    let order: [usize; 4] = if normal_up {
        [0, 1, 2, 3]
    } else {
        [0, 3, 2, 1]
    };
    for (i, p) in pts.iter().enumerate() {
        m.add_point((i + 1).to_string(), p[0] + 3.0, p[1] + 1.0, 0.0);
    }
    for k in [1, 4] {
        m.points.get_mut(&k.to_string()).unwrap().restraint = [true; 6];
    }
    m.add_area(
        "A",
        order.iter().map(|i| (i + 1).to_string()).collect(),
        "ASEC1",
    );
    let a = m.areas.get_mut("A").unwrap();
    a.angle_deg = angle;
    for (dir, v) in [(1, 0.002), (2, 0.0007)] {
        a.loads_uniform.push(AreaLoadUniform {
            load_pat: "DEAD".into(),
            csys: "Local".into(),
            dir,
            val: v,
        });
    }
    m
}

#[test]
fn areas_with_vertical_normal_rotate_exactly_with_the_angle_rule() {
    for up in [true, false] {
        let mut m = panel(up, 20.0);
        select_all(&mut m);
        let src = m.clone();
        let theta = 35.0;
        m.replicate_radial([0.0, 0.0], theta, 1).unwrap();
        let (err, scale) = rotation_mismatch(&m, &src, theta);
        assert!(scale > 1e-6, "panel must deflect (up={up}): {scale}");
        assert!(err < 1e-9, "up={up}: rotation mismatch {err:e}");
        let copy = m
            .areas
            .values()
            .find(|a| a.name != "A")
            .unwrap()
            .name
            .clone();
        m.areas.get_mut(&copy).unwrap().angle_deg = 20.0;
        let (bad, _) = rotation_mismatch(&m, &src, theta);
        assert!(
            bad > 1e-3,
            "up={up}: untransformed angle must differ, got {bad:e}"
        );
        m.areas.get_mut(&copy).unwrap().angle_deg = 20.0 + if up { -theta } else { theta };
        let (bad, _) = rotation_mismatch(&m, &src, theta);
        assert!(bad > 1e-3, "up={up}: wrong sign must differ, got {bad:e}");
    }
}

#[test]
fn inclined_and_wall_panels_keep_their_angle_under_rotation() {
    // Vertical wall in the XZ plane, loaded in its local 1 and 2 directions.
    let mut m = SapModel::default();
    for (n, p) in [
        ("1", [4.0, 0.0, 0.0]),
        ("2", [24.0, 0.0, 0.0]),
        ("3", [24.0, 0.0, 12.0]),
        ("4", [4.0, 0.0, 12.0]),
    ] {
        m.add_point(n, p[0], p[1], p[2]);
    }
    for k in ["1", "2"] {
        m.points.get_mut(k).unwrap().restraint = [true; 6];
    }
    m.add_area(
        "A",
        ["1", "2", "3", "4"].map(String::from).to_vec(),
        "ASEC1",
    );
    let a = m.areas.get_mut("A").unwrap();
    a.angle_deg = 15.0;
    for (dir, v) in [(1, 0.002), (2, -0.001)] {
        a.loads_uniform.push(AreaLoadUniform {
            load_pat: "DEAD".into(),
            csys: "Local".into(),
            dir,
            val: v,
        });
    }
    select_all(&mut m);
    let src = m.clone();
    m.replicate_radial([0.0, 0.0], 70.0, 1).unwrap();
    let copy = m.areas.values().find(|a| a.name != "A").unwrap();
    assert_eq!(
        copy.angle_deg, 15.0,
        "normal is horizontal: no angle change"
    );
    let (err, scale) = rotation_mismatch(&m, &src, 70.0);
    assert!(scale > 1e-6 && err < 1e-9, "err {err:e} scale {scale:e}");
}

fn beam_with_rotated_joint(set: impl Fn(&mut sap_model::PointObj)) -> SapModel {
    let mut m = SapModel::default();
    m.add_point("1", 20.0, 3.0, 0.0);
    m.add_point("2", 120.0, 3.0, 0.0);
    m.points.get_mut("1").unwrap().restraint = [true; 6];
    m.add_frame("B", "1", "2", "FSEC1");
    let p = m.points.get_mut("2").unwrap();
    p.spring = [5.0, 0.5, 2.0, 0.0, 0.0, 0.0];
    p.spring_is_local = true;
    set(p);
    p.loads_force.push(PointLoadForce {
        load_pat: "DEAD".into(),
        csys: "LOCAL".into(),
        force: [1.0, 0.5, -0.3, 0.0, 0.0, 0.0],
    });
    m
}

#[test]
fn joint_local_axes_rotate_with_euler_angle_a_and_with_explicit_matrices() {
    let sets: Vec<(&str, Box<dyn Fn(&mut sap_model::PointObj)>)> = vec![
        (
            "euler A,B,C",
            Box::new(|p| p.local_axes_deg = [20.0, 10.0, 5.0]),
        ),
        (
            "euler A only",
            Box::new(|p| p.local_axes_deg = [20.0, 0.0, 0.0]),
        ),
        (
            "matrix",
            Box::new(|p| {
                // columns: e1 = (0.6, 0.8, 0), e2 = (-0.8, 0.6, 0) rotated 30 deg about e1
                let (s, c) = 30.0f64.to_radians().sin_cos();
                let e1 = [0.6, 0.8, 0.0];
                let b = [-0.8, 0.6, 0.0];
                let e2: [f64; 3] = std::array::from_fn(|k| c * b[k] + s * [0.0, 0.0, 1.0][k]);
                let e3: [f64; 3] = std::array::from_fn(|k| -s * b[k] + c * [0.0, 0.0, 1.0][k]);
                p.local_axes_matrix = Some([
                    [e1[0], e2[0], e3[0]],
                    [e1[1], e2[1], e3[1]],
                    [e1[2], e2[2], e3[2]],
                ]);
            }),
        ),
    ];
    for (what, set) in sets {
        let mut m = beam_with_rotated_joint(&*set);
        select_all(&mut m);
        let src = m.clone();
        m.replicate_radial([0.0, 0.0], 40.0, 2).unwrap();
        assert_eq!(m.frames.len(), 3);
        // Displacements are reported along the joint's own axes (or global for
        // identity axes), so a rotated copy has the SAME local response.
        let res = solved(&m);
        let o = &res.joint_displacements["2"];
        assert!(
            o.iter().map(|v| v.abs()).fold(0.0, f64::max) > 1e-3,
            "{what}"
        );
        for k in 1..=2 {
            let theta = 40.0 * k as f64;
            let c = rot_z([120.0, 3.0, 0.0], theta);
            let copy = find(&m, c);
            let d = &res.joint_displacements[&copy];
            let worst = (0..6).map(|i| (d[i] - o[i]).abs()).fold(0.0, f64::max);
            let scale = o.iter().map(|v| v.abs()).fold(0.0, f64::max);
            assert!(worst / scale < 1e-9, "{what} copy {k}: {worst:e}");
        }
        // Negative control: undo the axis rotation on one copy.
        let copy = find(&m, rot_z([120.0, 3.0, 0.0], 40.0));
        let src_p = src.points["2"].clone();
        let p = m.points.get_mut(&copy).unwrap();
        p.local_axes_deg = src_p.local_axes_deg;
        p.local_axes_matrix = src_p.local_axes_matrix;
        let res = solved(&m);
        let d = &res.joint_displacements[&copy];
        let worst = (0..6).map(|i| (d[i] - o[i]).abs()).fold(0.0, f64::max);
        let scale = o.iter().map(|v| v.abs()).fold(0.0, f64::max);
        assert!(
            worst / scale > 1e-3,
            "{what}: control must differ ({worst:e})"
        );
    }
}

#[test]
fn reversing_local_3_with_same_global_loads_leaves_the_response_unchanged() {
    // Inclined, vertical and horizontal panels with loads in every local
    // direction, a pressure on the top face and a gradient.
    let layouts: [([[f64; 3]; 4], &str); 3] = [
        (
            [
                [0.0, 0.0, 0.0],
                [20.0, 0.0, 0.0],
                [20.0, 10.0, 0.0],
                [0.0, 10.0, 0.0],
            ],
            "flat",
        ),
        (
            [
                [0.0, 0.0, 0.0],
                [20.0, 0.0, 0.0],
                [20.0, 0.0, 12.0],
                [0.0, 0.0, 12.0],
            ],
            "wall",
        ),
        (
            [
                [0.0, 0.0, 0.0],
                [20.0, 0.0, 0.0],
                [20.0, 8.0, 6.0],
                [0.0, 8.0, 6.0],
            ],
            "inclined",
        ),
    ];
    for (quad, what) in layouts {
        let mut m = SapModel::default();
        for (i, p) in quad.iter().enumerate() {
            m.add_point((i + 1).to_string(), p[0], p[1], p[2]);
        }
        for k in ["1", "2"] {
            m.points.get_mut(k).unwrap().restraint = [true; 6];
        }
        m.add_area(
            "A",
            ["1", "2", "3", "4"].map(String::from).to_vec(),
            "ASEC1",
        );
        let a = m.areas.get_mut("A").unwrap();
        a.angle_deg = 25.0;
        a.selected = true;
        for (dir, v) in [(1, 0.002), (2, -0.001), (3, 0.0005), (6, 0.0003)] {
            a.loads_uniform.push(AreaLoadUniform {
                load_pat: "DEAD".into(),
                csys: if dir <= 3 {
                    "Local".into()
                } else {
                    "GLOBAL".into()
                },
                dir,
                val: v,
            });
        }
        a.loads_pressure.push(sap_model::AreaLoadSurfacePressure {
            load_pat: "DEAD".into(),
            face: -2,
            val: 0.0004,
            pattern_name: None,
        });
        // uniform temperature change and a through-thickness gradient
        for (my_type, val) in [(1, 20.0), (2, 3.0)] {
            a.loads_temp.push(sap_model::AreaLoadTemperature {
                load_pat: "DEAD".into(),
                my_type,
                val,
                pattern_name: None,
            });
        }
        let before = solved(&m);
        let mut r = m.clone();
        r.reverse_area_local_3(ReverseKeep::SameGlobalLoads)
            .unwrap();
        let after = solved(&r);
        let scale = before
            .joint_displacements
            .values()
            .flat_map(|d| d.iter().map(|v| v.abs()))
            .fold(0.0, f64::max);
        assert!(scale > 1e-6, "{what}");
        for (name, d) in &before.joint_displacements {
            for i in 0..6 {
                assert!(
                    (d[i] - after.joint_displacements[name][i]).abs() / scale < 1e-9,
                    "{what}: joint {name} dof {i}: {} vs {}",
                    d[i],
                    after.joint_displacements[name][i]
                );
            }
        }
        // "Same local loads" keeps the numbers, so the loads in local 2 and 3
        // and the face pressure turn with the axes: the response must change.
        let mut l = m.clone();
        l.reverse_area_local_3(ReverseKeep::SameLocalLoads).unwrap();
        let flipped = solved(&l);
        let diff = before
            .joint_displacements
            .iter()
            .flat_map(|(n, d)| {
                let f = &flipped.joint_displacements[n];
                (0..6).map(move |i| (d[i] - f[i]).abs())
            })
            .fold(0.0, f64::max);
        assert!(
            diff / scale > 1e-3,
            "{what}: same-local reversal must change the response"
        );
    }
}

// --- generated structures analyse and behave as intended ---------------------

use sap_model::generate::{BraceKind, GridRegion, SpanDirection, VerticalPlane};

fn sum_reactions(res: &sap_model::CaseResults) -> [f64; 6] {
    let mut s = [0.0; 6];
    for r in res.joint_reactions.values() {
        for k in 0..6 {
            s[k] += r[k];
        }
    }
    s
}

fn braced_bay(kind: Option<BraceKind>) -> SapModel {
    let mut m = SapModel::default();
    // x = 0, 120 ; y = 0 ; z = 0, 96 (inches)
    m.quick_grid([2, 1, 2], [120.0, 100.0, 96.0], [0.0; 3])
        .unwrap();
    let r = GridRegion {
        x: (0, 1),
        y: (0, 0),
        z: (0, 1),
    };
    m.quick_draw_columns(r, "FSEC1").unwrap();
    m.quick_draw_beams(r, true, false, "FSEC1").unwrap();
    if let Some(k) = kind {
        m.quick_draw_braces(r, VerticalPlane::XZ, k, "FSEC1")
            .unwrap();
    }
    for p in m.points.values_mut() {
        if p.z == 0.0 {
            p.restraint = [true; 6];
        }
    }
    let top = m
        .points
        .values_mut()
        .find(|p| p.x == 0.0 && p.z == 96.0)
        .unwrap();
    top.loads_force.push(PointLoadForce {
        load_pat: "DEAD".into(),
        csys: "GLOBAL".into(),
        force: [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    });
    m
}

#[test]
fn quick_draw_braces_are_pinned_stable_and_carry_the_lateral_load() {
    let p = 10.0; // kip, global X at the top-left joint
    let free = solved(&braced_bay(None));
    let top_free = free.joint_displacements[&find(&braced_bay(None), [0.0, 0.0, 96.0])][0];
    for kind in [BraceKind::X, BraceKind::V, BraceKind::InvertedV] {
        let m = braced_bay(Some(kind));
        let res = solved(&m);
        // Equilibrium: base reactions balance the applied load (kip).
        let sum = sum_reactions(&res);
        assert!(
            (sum[0] + p).abs() < 1e-9 * p,
            "{kind:?}: sum Rx = {}",
            sum[0]
        );
        // The braces stiffen the bay.
        let top = res.joint_displacements[&find(&m, [0.0, 0.0, 96.0])][0];
        assert!(
            top < 0.5 * top_free,
            "{kind:?}: braced {top} vs bare {top_free}"
        );
        // Pinned braces: no bending moment at the ends, a real axial force.
        let braces: Vec<_> = m
            .frames
            .values()
            .filter(|f| f.release_i[4] && f.release_i[5])
            .collect();
        assert_eq!(braces.len(), 2, "{kind:?}");
        for b in braces {
            let st = &res.frame_forces[&b.name];
            let (first, last) = (&st[0], &st[st.len() - 1]);
            for s in [first, last] {
                assert!(
                    s.m2.abs() < 1e-6 && s.m3.abs() < 1e-6,
                    "{kind:?} {}: {s:?}",
                    b.name
                );
            }
            assert!(
                first.p.abs() > 0.1,
                "{kind:?}: brace carries axial force {}",
                first.p
            );
        }
    }
}

#[test]
fn secondary_beams_reach_the_girders_only_through_the_mesh_flag() {
    let mut m = SapModel::default();
    m.quick_grid([3, 3, 1], [120.0, 120.0, 1.0], [0.0, 0.0, 96.0])
        .unwrap();
    let region = GridRegion {
        x: (0, 2),
        y: (0, 2),
        z: (0, 0),
    };
    m.quick_draw_beams(region, true, true, "FSEC1").unwrap();
    let grid_joints: Vec<String> = m.points.keys().cloned().collect();
    for j in &grid_joints {
        m.points.get_mut(j).unwrap().restraint = [true; 6];
    }
    let s = m
        .quick_draw_secondary_beams(region, SpanDirection::X, 2, "FSEC1")
        .unwrap();
    let w = 0.01; // kip/in, gravity
    for n in &s.frames_created {
        m.frames
            .get_mut(n)
            .unwrap()
            .loads_dist
            .push(FrameLoadDistributed {
                load_pat: "DEAD".into(),
                my_type: 1,
                dir: 10,
                csys: "GLOBAL".into(),
                rel_dist1: 0.0,
                rel_dist2: 1.0,
                val1: w,
                val2: w,
            });
    }
    // 4 bays x 2 secondary beams x 120 in
    let total = w * 4.0 * 2.0 * 120.0;
    let res = solved(&m);
    let rz = sum_reactions(&res)[2];
    assert!(
        (rz - total).abs() < 1e-9 * total,
        "sum Rz {rz} vs {total} kip"
    );
    // Without the girder mesh flag the beam ends are not connected to the girders.
    let mut loose = m.clone();
    for f in loose.frames.values_mut() {
        f.auto_mesh_at_joints = false;
    }
    let mut work = loose.clone();
    work.load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    match analyze_model(&mut work) {
        Err(_) => {}
        Ok(()) => {
            let rz = sum_reactions(&work.case_results["DEAD"])[2];
            assert!(
                (rz - total).abs() > 1e-3 * total,
                "control must differ: {rz}"
            );
        }
    }
}

#[test]
fn generated_diaphragms_make_each_level_rigid_and_independent() {
    let build = |with_diaphragm: bool| {
        let mut m = SapModel::default();
        // x = 0, 120 ; y = 0, 100 ; z = 0, 96, 192
        m.quick_grid([2, 2, 3], [120.0, 100.0, 96.0], [0.0; 3])
            .unwrap();
        let region = GridRegion {
            x: (0, 1),
            y: (0, 1),
            z: (0, 2),
        };
        m.quick_draw_columns(region, "FSEC1").unwrap();
        for p in m.points.values_mut() {
            p.restraint = [p.z == 0.0; 6];
            p.selected = p.z > 0.0;
        }
        if with_diaphragm {
            assert_eq!(m.assign_diaphragm("D", true).unwrap(), ["D_1", "D_2"]);
        }
        let name = find(&m, [0.0, 0.0, 192.0]);
        m.points
            .get_mut(&name)
            .unwrap()
            .loads_force
            .push(PointLoadForce {
                load_pat: "DEAD".into(),
                csys: "GLOBAL".into(),
                force: [10.0, 4.0, 0.0, 0.0, 0.0, 0.0],
            });
        m
    };
    // Largest in-plane departure from rigid-body motion, relative to the largest displacement.
    let departure = |m: &SapModel| {
        let res = solved(m);
        let scale = res
            .joint_displacements
            .values()
            .map(|d| d[0].abs().max(d[1].abs()))
            .fold(0.0, f64::max);
        let mut worst: f64 = 0.0;
        for z in [96.0, 192.0] {
            let level: Vec<_> = m.points.values().filter(|p| p.z == z).collect();
            let (r, ur) = (level[0], res.joint_displacements[&level[0].name]);
            for p in &level[1..] {
                let u = res.joint_displacements[&p.name];
                let w = ur[5];
                let want_x = ur[0] - w * (p.y - r.y);
                let want_y = ur[1] + w * (p.x - r.x);
                worst = worst
                    .max((u[0] - want_x).abs())
                    .max((u[1] - want_y).abs())
                    .max((u[5] - w).abs() * 100.0);
            }
        }
        (worst / scale, scale, res)
    };
    let m = build(true);
    let (err, scale, res) = departure(&m);
    assert!(scale > 1e-3);
    assert!(err < 1e-4, "diaphragm kinematics violated: {err:e}");
    let sum = sum_reactions(&res);
    assert!(
        (sum[0] + 10.0).abs() < 1e-3 && (sum[1] + 4.0).abs() < 1e-3,
        "{sum:?}"
    ); // penalty constraints: ~1e-5 relative
       // Levels are independent: the two storeys move differently.
    let a = res.joint_displacements[&find(&m, [0.0, 0.0, 96.0])][0];
    let b = res.joint_displacements[&find(&m, [0.0, 0.0, 192.0])][0];
    assert!((a - b).abs() > 1e-3 * scale, "{a} vs {b}");
    // Negative control: without the diaphragms the floors are not rigid.
    let (bad, _, _) = departure(&build(false));
    assert!(bad > 1e-3, "control must violate rigidity: {bad:e}");
}
