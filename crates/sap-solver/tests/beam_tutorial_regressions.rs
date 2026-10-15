//! Closed-form beam checks for the three introductory tutorial workflows.
//!
//! These are purpose-built analytical models, not reconstructions of the video
//! examples: playlist metadata was available, but the videos were not watched.
use sap_format::{parse_s2k, write_s2k};
use sap_model::{FrameLoadDistributed, LoadCase, SapModel};
use sap_solver::analyze_model;
use sap_units::Units;

const E: f64 = 30_000.0; // kip/in^2
const I: f64 = 120.0; // in^4, bending about local 2 for local-3 transverse load
const L: f64 = 120.0; // in per span
const W: f64 = 0.02; // kip/in, positive local-3 direction

fn base_model() -> SapModel {
    let mut m = SapModel::new(Units::KipInF);
    m.load_cases.clear();
    m.load_patterns["DEAD"].self_wt_multiplier = 0.0;
    m.materials["A992Fy50"].e = E;
    m.materials["A992Fy50"].g = E / (2.0 * (1.0 + 0.3));
    let s = m.frame_sections.get_mut("FSEC1").unwrap();
    s.i22 = I;
    s.i33 = I;
    s.as2 = 0.0;
    s.as3 = 0.0; // Euler-Bernoulli reference: shear deformation disabled.
    s.area = 1.0;
    s.torsional_j = 1.0;
    m
}

fn add_beam(m: &mut SapModel, name: &str, i: &str, j: &str) {
    m.add_frame(name, i, j, "FSEC1");
    m.frames
        .get_mut(name)
        .unwrap()
        .loads_dist
        .push(FrameLoadDistributed {
            load_pat: "DEAD".into(),
            my_type: 1,
            dir: 3, // uniform force per length in local 3 (global Y for this frame)
            csys: "LOCAL".into(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: W,
            val2: W,
        });
}

fn add_span_point(m: &mut SapModel, name: &str, x: f64, restraint: [bool; 6]) {
    m.add_point(name, x, 0.0, 0.0);
    m.points.get_mut(name).unwrap().restraint = restraint;
}

fn static_case(m: &mut SapModel) {
    m.load_cases
        .insert("DEAD".into(), LoadCase::linear_static("DEAD", "DEAD", 1.0));
}

fn close(label: &str, actual: f64, expected: f64, abs_tol: f64, rel_tol: f64) {
    assert!(actual.is_finite(), "{label} is not finite: {actual}");
    let error = (actual - expected).abs();
    assert!(
        error <= abs_tol.max(rel_tol * expected.abs()),
        "{label}: got {actual:.9e}, expected {expected:.9e}, error {error:.3e}"
    );
}

fn s2k_round_trip(m: &SapModel) -> SapModel {
    parse_s2k(&write_s2k(m)).expect("beam model S2K round trip should parse")
}

#[test]
fn simply_supported_uniform_load_matches_euler_bernoulli_solution_and_s2k_roundtrip() {
    let mut m = base_model();
    // Pin and roller restrain the loaded global-Y direction and leave the
    // local bending rotation (global Z) free.
    add_span_point(&mut m, "A", 0.0, [true, true, true, true, true, false]);
    add_span_point(&mut m, "M", L / 2.0, [true, false, true, true, true, false]);
    add_span_point(&mut m, "B", L, [false, true, true, true, true, false]);
    add_beam(&mut m, "AM", "A", "M");
    add_beam(&mut m, "MB", "M", "B");
    static_case(&mut m);

    let mut imported = s2k_round_trip(&m);
    for model in [&mut m, &mut imported] {
        analyze_model(model).expect("simply supported beam should analyze");
        let r = &model.case_results["DEAD"];
        let total = W * L;
        let uy_mid = r.joint_displacements["M"][1];
        close(
            "support A reaction Y",
            r.joint_reactions["A"][1].abs(),
            total / 2.0,
            1e-7,
            2e-6,
        );
        close(
            "support B reaction Y",
            r.joint_reactions["B"][1].abs(),
            total / 2.0,
            1e-7,
            2e-6,
        );
        close(
            "midspan |M2|",
            r.frame_forces["AM"]
                .iter()
                .max_by(|a, b| a.rel_dist.total_cmp(&b.rel_dist))
                .unwrap()
                .m2
                .abs(),
            W * L.powi(2) / 8.0,
            1e-6,
            2e-6,
        );
        close(
            "midspan local-3 deflection",
            uy_mid.abs(),
            5.0 * W * L.powi(4) / (384.0 * E * I),
            1e-7,
            3e-5,
        );
    }
}

#[test]
fn loaded_frame_station_deflection_includes_distributed_load_curvature() {
    // Keep a one-element analytical oracle too: midpoint station recovery must
    // include the load's interior deflection, not only Hermite-interpolate the
    // end DOFs. The nodal checks above would not detect that output-path defect.
    let mut simply_supported = base_model();
    add_span_point(
        &mut simply_supported,
        "A",
        0.0,
        [true, true, true, true, true, false],
    );
    add_span_point(
        &mut simply_supported,
        "B",
        L,
        [false, true, true, true, true, false],
    );
    add_beam(&mut simply_supported, "AB", "A", "B");
    static_case(&mut simply_supported);
    analyze_model(&mut simply_supported).expect("one-element simple beam should analyze");
    let ss_mid = simply_supported.case_results["DEAD"].frame_forces["AB"]
        .iter()
        .find(|s| (s.rel_dist - 0.5).abs() < 1e-8)
        .unwrap()
        .local_disp[2]
        .abs();
    close(
        "one-element simply-supported station deflection",
        ss_mid,
        5.0 * W * L.powi(4) / (384.0 * E * I),
        1e-7,
        3e-5,
    );

    let mut continuous = base_model();
    add_span_point(
        &mut continuous,
        "A",
        0.0,
        [true, true, true, true, true, false],
    );
    add_span_point(
        &mut continuous,
        "B",
        L,
        [false, true, true, true, true, false],
    );
    add_span_point(
        &mut continuous,
        "C",
        2.0 * L,
        [false, true, true, true, true, false],
    );
    add_beam(&mut continuous, "AB", "A", "B");
    add_beam(&mut continuous, "BC", "B", "C");
    static_case(&mut continuous);
    analyze_model(&mut continuous).expect("one-element-per-span continuous beam should analyze");
    for frame in ["AB", "BC"] {
        let mid = continuous.case_results["DEAD"].frame_forces[frame]
            .iter()
            .find(|s| (s.rel_dist - 0.5).abs() < 1e-8)
            .unwrap()
            .local_disp[2]
            .abs();
        close(
            "one-element continuous-span station deflection",
            mid,
            W * L.powi(4) / (192.0 * E * I),
            1e-7,
            4e-5,
        );
    }
}

#[test]
fn cantilever_tip_force_matches_closed_form_reaction_moment_and_deflection() {
    let mut m = base_model();
    add_span_point(&mut m, "A", 0.0, [true; 6]);
    add_span_point(&mut m, "B", L, [true, false, true, true, true, false]);
    // Do not retain the common UDL helper for this point-load workflow.
    m.add_frame("AB", "A", "B", "FSEC1");
    let p = m.points.get_mut("B").unwrap();
    p.restraint = [true, false, true, true, true, false];
    p.loads_force.push(sap_model::PointLoadForce {
        load_pat: "DEAD".into(),
        csys: "GLOBAL".into(),
        force: [0.0, W * L, 0.0, 0.0, 0.0, 0.0],
    });
    static_case(&mut m);
    analyze_model(&mut m).expect("cantilever should analyze");
    let r = &m.case_results["DEAD"];
    close(
        "fixed-end reaction Y",
        r.joint_reactions["A"][1].abs(),
        W * L,
        1e-7,
        2e-6,
    );
    close(
        "fixed-end reaction moment Z",
        r.joint_reactions["A"][5].abs(),
        W * L.powi(2),
        1e-6,
        2e-6,
    );
    close(
        "tip deflection Y",
        r.joint_displacements["B"][1].abs(),
        (W * L) * L.powi(3) / (3.0 * E * I),
        1e-7,
        3e-5,
    );
    let tip = r.frame_forces["AB"]
        .iter()
        .max_by(|a, b| a.rel_dist.total_cmp(&b.rel_dist))
        .unwrap();
    close("cantilever tip moment", tip.m2.abs(), 0.0, 1e-6, 0.0);
}

#[test]
fn two_span_continuous_beam_matches_three_moment_solution() {
    let mut m = base_model();
    add_span_point(&mut m, "A", 0.0, [true, true, true, true, true, false]);
    add_span_point(
        &mut m,
        "M1",
        L / 2.0,
        [true, false, true, true, true, false],
    );
    add_span_point(&mut m, "B", L, [false, true, true, true, true, false]);
    add_span_point(
        &mut m,
        "M2",
        1.5 * L,
        [true, false, true, true, true, false],
    );
    add_span_point(&mut m, "C", 2.0 * L, [false, true, true, true, true, false]);
    add_beam(&mut m, "AM1", "A", "M1");
    add_beam(&mut m, "M1B", "M1", "B");
    add_beam(&mut m, "BM2", "B", "M2");
    add_beam(&mut m, "M2C", "M2", "C");
    static_case(&mut m);
    analyze_model(&mut m).expect("two-span continuous beam should analyze");
    let r = &m.case_results["DEAD"];
    let ql = W * L;
    close(
        "outer reaction A",
        r.joint_reactions["A"][1].abs(),
        3.0 * ql / 8.0,
        1e-7,
        3e-6,
    );
    close(
        "interior reaction B",
        r.joint_reactions["B"][1].abs(),
        5.0 * ql / 4.0,
        1e-7,
        3e-6,
    );
    close(
        "outer reaction C",
        r.joint_reactions["C"][1].abs(),
        3.0 * ql / 8.0,
        1e-7,
        3e-6,
    );
    let left_end = r.frame_forces["M1B"]
        .iter()
        .max_by(|a, b| a.rel_dist.total_cmp(&b.rel_dist))
        .unwrap();
    let right_start = r.frame_forces["BM2"]
        .iter()
        .min_by(|a, b| a.rel_dist.total_cmp(&b.rel_dist))
        .unwrap();
    close(
        "interior support moment from left span",
        left_end.m2.abs(),
        W * L.powi(2) / 8.0,
        1e-6,
        3e-6,
    );
    close(
        "interior support moment from right span",
        right_start.m2.abs(),
        W * L.powi(2) / 8.0,
        1e-6,
        3e-6,
    );
    close(
        "left span midspan positive moment",
        r.frame_forces["AM1"]
            .iter()
            .max_by(|a, b| a.rel_dist.total_cmp(&b.rel_dist))
            .unwrap()
            .m2
            .abs(),
        W * L.powi(2) / 16.0,
        1e-6,
        3e-6,
    );
    close(
        "right span midspan positive moment",
        r.frame_forces["BM2"]
            .iter()
            .max_by(|a, b| a.rel_dist.total_cmp(&b.rel_dist))
            .unwrap()
            .m2
            .abs(),
        W * L.powi(2) / 16.0,
        1e-6,
        3e-6,
    );
    close(
        "left span midspan deflection",
        r.joint_displacements["M1"][1].abs(),
        W * L.powi(4) / (192.0 * E * I),
        1e-7,
        4e-5,
    );
    close(
        "right span midspan deflection",
        r.joint_displacements["M2"][1].abs(),
        W * L.powi(4) / (192.0 * E * I),
        1e-7,
        4e-5,
    );
}
