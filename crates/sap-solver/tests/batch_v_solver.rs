//! Batch V (solver) regressions. Every expected value is a hand calculation written out in the
//! comment of its test; none is read from the code under test.
use sap_model::*;
use sap_solver::analyze_model;

fn close(what: &str, got: f64, want: f64, tol: f64) {
    assert!(
        (got - want).abs() <= tol * want.abs().max(1e-12),
        "{what}: got {got:.12e}, want {want:.12e}"
    );
}

fn pattern(m: &mut SapModel, name: &str) {
    m.load_patterns.insert(
        name.into(),
        LoadPattern {
            name: name.into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
}

fn force(m: &mut SapModel, joint: &str, pat: &str, f: [f64; 6]) {
    m.points
        .get_mut(joint)
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: pat.into(),
            csys: "GLOBAL".into(),
            force: f,
        });
}

fn constraint(m: &mut SapModel, name: &str, kind: ConstraintType, dofs: [bool; 6], joints: &[&str]) {
    m.constraints.insert(
        name.into(),
        ConstraintDef {
            name: name.into(),
            constraint_type: kind,
            axis: ConstraintAxis::Z,
            dofs,
        },
    );
    for j in joints {
        m.points.get_mut(*j).unwrap().constraints.push(name.into());
    }
}

// ------------------------------------------------------------------ D106 -----------------

/// kN, m. Joint A (0,0,0) restrained U1, U2, R1, R2, R3 and free in U3; joint B (1,0,0) fully
/// restrained; an Equal constraint on U3 joins A and B; no frame; Fz = -10 kN at A. The
/// constraint carries the whole load into B, so by statics the reaction at B is Fz = +10 kN
/// and the base reaction is Fz = +10 kN (the vertical spring of 1 kN/m at A takes 1 x u_A,
/// u_A being about 10/penalty, i.e. 1e-6 relative). Before the change both were 0.
fn equal_model(spring_at_a: f64) -> SapModel {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    m.add_point("A", 0.0, 0.0, 0.0);
    m.add_point("B", 1.0, 0.0, 0.0);
    m.points.get_mut("A").unwrap().restraint = [true, true, false, true, true, true];
    m.points.get_mut("A").unwrap().spring[2] = spring_at_a;
    m.points.get_mut("B").unwrap().restraint = [true; 6];
    constraint(
        &mut m,
        "EQ",
        ConstraintType::Equal,
        [false, false, true, false, false, false],
        &["A", "B"],
    );
    pattern(&mut m, "P");
    m.load_cases
        .insert("P".into(), LoadCase::linear_static("P", "P", 1.0));
    force(&mut m, "A", "P", [0.0, 0.0, -10.0, 0.0, 0.0, 0.0]);
    m
}

#[test]
fn equal_constraint_to_a_support_reports_the_reaction() {
    let mut m = equal_model(1.0);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["P"];
    close("Fz at B", r.joint_reactions["B"][2], 10.0, 1e-6);
    close("base Fz", r.base_reactions[2], 10.0, 1e-6);
    assert!(r.joint_displacements["A"][2].abs() < 1e-5);
}

/// Two joints at z = 0 tied by a Diaphragm (axis Z: UX, UY, RZ about the master), one fully
/// fixed (A at the origin), the other (B at x = 2) restrained except U1, U2, R3, with
/// Fx = 5 kN at B. The only path to the ground is the diaphragm into A, so the base reaction
/// is Fx = -5 kN and A's reaction is Fx = -5 kN (B has no reaction in X). A moment about Z
/// is not loaded; the force acts at B on the line y = 0 of the master, so RZ = 0.
#[test]
fn diaphragm_to_a_fixed_joint_reports_the_base_shear() {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    m.add_point("A", 0.0, 0.0, 0.0);
    m.add_point("B", 2.0, 0.0, 0.0);
    m.points.get_mut("A").unwrap().restraint = [true; 6];
    m.points.get_mut("B").unwrap().restraint = [false, false, true, true, true, false];
    constraint(&mut m, "D", ConstraintType::Diaphragm, [true; 6], &["A", "B"]);
    pattern(&mut m, "P");
    m.load_cases
        .insert("P".into(), LoadCase::linear_static("P", "P", 1.0));
    force(&mut m, "B", "P", [5.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["P"];
    close("base Fx", r.base_reactions[0], -5.0, 1e-6);
    close("A Fx", r.joint_reactions["A"][0], -5.0, 1e-6);
    assert!(r.base_reactions[1].abs() < 1e-6);
}

/// Negative: without the constraint the unconnected joint B has no force path, the model of
/// the first test is a mechanism at A; instead check a frame model without constraints: a
/// column of EA/L = 29000 x 10 / 100 = 2900 kip/in under 10 kip (kip, in) has the reaction
/// -10 kip by statics, as before.
#[test]
fn a_frame_without_constraints_keeps_its_reaction() {
    let mut m = SapModel::new(sap_units::Units::KipInF);
    m.load_cases.clear();
    m.load_patterns.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = 29_000.0;
    mat.u = 0.3;
    mat.g = 11_000.0;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    m.materials.insert("MAT".into(), mat);
    m.frame_sections.insert(
        "S".into(),
        FrameSection {
            name: "S".into(),
            material: "MAT".into(),
            area: 10.0,
            i22: 100.0,
            i33: 100.0,
            torsional_j: 200.0,
            as2: 0.0,
            as3: 0.0,
            ..FrameSection::default()
        },
    );
    m.add_point("B", 0.0, 0.0, 0.0);
    m.add_point("T", 0.0, 0.0, 100.0);
    m.points.get_mut("B").unwrap().restraint = [true; 6];
    m.add_frame("C", "B", "T", "S");
    pattern(&mut m, "P");
    m.load_cases
        .insert("P".into(), LoadCase::linear_static("P", "P", 1.0));
    force(&mut m, "T", "P", [0.0, 0.0, -10.0, 0.0, 0.0, 0.0]);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["P"];
    close("Fz", r.joint_reactions["B"][2], 10.0, 1e-9);
    close("base Fz", r.base_reactions[2], 10.0, 1e-9);
}

/// Negative of D106: a floor diaphragm that touches no support. Two columns (kip, in) fixed at
/// the base, the tops tied by a Diaphragm, Fx = 5 kip at one top: the bases carry the whole
/// load, so the base reactions sum to Fx = -5 and, the columns being equal and the diaphragm
/// keeping the tops equal in X, each base takes -2.5 (the load acts on the line of the
/// columns, so the diaphragm does not rotate).
#[test]
fn a_diaphragm_that_touches_no_support_keeps_the_statics() {
    let mut m = SapModel::new(sap_units::Units::KipInF);
    m.load_cases.clear();
    m.load_patterns.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = 29_000.0;
    mat.u = 0.3;
    mat.g = 11_000.0;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    m.materials.insert("MAT".into(), mat);
    m.frame_sections.insert(
        "S".into(),
        FrameSection {
            name: "S".into(),
            material: "MAT".into(),
            area: 10.0,
            i22: 100.0,
            i33: 100.0,
            torsional_j: 200.0,
            as2: 0.0,
            as3: 0.0,
            ..FrameSection::default()
        },
    );
    for (i, x) in [0.0, 100.0].into_iter().enumerate() {
        let (b, t) = (format!("B{i}"), format!("T{i}"));
        m.add_point(b.clone(), x, 0.0, 0.0);
        m.add_point(t.clone(), x, 0.0, 100.0);
        m.points.get_mut(&b).unwrap().restraint = [true; 6];
        m.add_frame(format!("C{i}"), b, t, "S");
    }
    constraint(&mut m, "D", ConstraintType::Diaphragm, [true; 6], &["T0", "T1"]);
    pattern(&mut m, "P");
    m.load_cases
        .insert("P".into(), LoadCase::linear_static("P", "P", 1.0));
    force(&mut m, "T0", "P", [5.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["P"];
    // Tolerance 1e-4: the diaphragm is a penalty (measured 8e-6 relative from its solve
    // roundoff); the reactions come from the frames only, as before D106.
    close("sum of base shears", r.base_reactions[0], -5.0, 1e-4);
    close("base 0", r.joint_reactions["B0"][0], -2.5, 1e-4);
    close("base 1", r.joint_reactions["B1"][0], -2.5, 1e-4);
}

// ------------------------------------------------------------------ D108 -----------------

/// kN, m, t. One ShellThin area 1 m x 1 m, thickness `t`, concrete E, unit weight `gamma`,
/// unit mass 0; the four corner joints restrained except U3, each with a vertical spring
/// 250 kN/m and a mass 2.5 t in U3 (k = 1000 kN/m, M = 10 t, w = 10 rad/s). The corner
/// rotations are restrained, so the slab carries nothing and the four springs take the load.
/// DEAD: uniform area load 10 kN/m2 downward (W = 10 kN), u = -W/k = -0.01 m; or area self
/// weight 25 kN/m3 x 0.04 m x 1 m2 = 1 kN, u = -0.001 m. A NonlinearDynamic case that
/// continues from NonlinearStatic DEAD with a zero history load must stay at that u (a0 = 0).
/// Before the change the initial load kept in the history lacked the area load, so u
/// oscillated between u0 and -u0 with period 2 pi / 10 = 0.628 s.
fn slab_model(thickness: f64, unit_weight: f64, uniform: f64, self_weight: f64) -> SapModel {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "CONC".into();
    mat.e = 2.5e7;
    mat.u = 0.2;
    mat.g = 2.5e7 / 2.4;
    mat.unit_weight = unit_weight;
    mat.unit_mass = 0.0;
    m.materials.insert("CONC".into(), mat);
    let mut sec = AreaSection::default();
    sec.name = "SLAB".into();
    sec.material = "CONC".into();
    sec.prop_type = AreaPropType::Shell;
    sec.shell_type = ShellType::ShellThin;
    sec.thickness_membrane = thickness;
    sec.thickness_bending = thickness;
    m.area_sections.insert("SLAB".into(), sec);
    for (n, x, y) in [("1", 0.0, 0.0), ("2", 1.0, 0.0), ("3", 1.0, 1.0), ("4", 0.0, 1.0)] {
        m.add_point(n, x, y, 0.0);
        let p = m.points.get_mut(n).unwrap();
        p.restraint = [true, true, false, true, true, true];
        p.spring[2] = 250.0;
        p.mass[2] = 2.5;
    }
    m.add_area("A", vec!["1".into(), "2".into(), "3".into(), "4".into()], "SLAB");
    m.load_patterns.insert(
        "DEAD".into(),
        LoadPattern {
            name: "DEAD".into(),
            pat_type: LoadPatternType::Dead,
            self_wt_multiplier: self_weight,
        },
    );
    pattern(&mut m, "ZERO");
    if uniform != 0.0 {
        m.areas.get_mut("A").unwrap().loads_uniform.push(AreaLoadUniform {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            dir: 6,
            val: -uniform,
        });
    }
    let mut ns = LoadCase::linear_static("NS", "DEAD", 1.0);
    ns.case_type = LoadCaseType::NonlinearStatic;
    m.load_cases.insert("NS".into(), ns);
    m.functions.insert(
        "STEP".into(),
        DynamicFunction {
            name: "STEP".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: vec![(0.0, 1.0), (10.0, 1.0)],
        },
    );
    let mut nd = LoadCase::linear_static("ND", "ZERO", 1.0);
    nd.loads.clear();
    nd.case_type = LoadCaseType::NonlinearDynamic;
    nd.initial_case = Some("NS".into());
    nd.modal_damping = 0.0;
    nd.history_loads = vec![("STEP".into(), "ZERO".into(), 1.0)];
    nd.time_step_dt = 0.01;
    nd.num_time_steps = 100;
    m.load_cases.insert("ND".into(), nd);
    m
}

fn assert_continuation_holds(m: &mut SapModel, want_u: f64) {
    analyze_model(m).unwrap();
    for j in ["1", "2", "3", "4"] {
        close(
            &format!("static u at {j}"),
            m.case_results["NS"].joint_displacements[j][2],
            want_u,
            1e-6,
        );
        let trace = &m.case_results["ND"].time_history_disp[j];
        assert_eq!(trace.len(), 101);
        for (k, d) in trace.iter().enumerate() {
            close(&format!("step {k} at {j}"), d[2], want_u, 1e-6);
        }
    }
}

#[test]
fn history_continuing_from_a_slab_load_stays_in_equilibrium() {
    // uniform load 10 kN/m2 on 1 m2: W = 10 kN, u = -10/1000
    assert_continuation_holds(&mut slab_model(0.2, 0.0, 10.0, 0.0), -0.01);
}

#[test]
fn history_continuing_from_slab_self_weight_stays_in_equilibrium() {
    // 25 kN/m3 x 0.04 m x 1 m2 = 1 kN, u = -1/1000
    assert_continuation_holds(&mut slab_model(0.04, 25.0, 0.0, 1.0), -0.001);
}

/// Negative: with no slab load at all the static displacement is 0 and the history stays
/// at 0 (nothing was invented by the new block).
#[test]
fn history_continuing_from_an_unloaded_slab_stays_at_rest() {
    assert_continuation_holds(&mut slab_model(0.2, 0.0, 0.0, 0.0), 0.0);
}

// ------------------------------------------------------------------ D112 -----------------

/// kip, in. Joints A (0,0,0) and B (1,0,0), each with a U1 spring k = 100 kip/in and every
/// other DOF restrained, tied by an Equal constraint on U1; a separate bar C-D, E = 29000,
/// A = 1e6, L = 100 (EA/L = 2.9e8, so the plain pivot test, 1e-11 x penalty with penalty =
/// 1e6 x 2.9e8, rejects any DOF with stiffness below 2.9e3, above k1 + k2 = 200); a gap link
/// between A and B that stays open (the two joints move together, the gap closes only at
/// -0.03 in) so that the case goes through the nonlinear system. P = 10 kip in U1 at A:
/// u_A = u_B = P / (k1 + k2) = 10 / 200 = 0.05 in. Before: SingularMatrix.
fn constrained_springs(mass: f64) -> SapModel {
    let mut m = SapModel::new(sap_units::Units::KipInF);
    m.load_cases.clear();
    m.load_patterns.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "M".into();
    mat.e = 29000.0;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    m.materials.insert("M".into(), mat);
    m.frame_sections.insert(
        "S".into(),
        FrameSection {
            name: "S".into(),
            material: "M".into(),
            area: 1e6,
            ..FrameSection::default()
        },
    );
    m.add_point("A", 0.0, 0.0, 0.0);
    m.add_point("B", 1.0, 0.0, 0.0);
    m.add_point("C", 0.0, 50.0, 0.0);
    m.add_point("D", 100.0, 50.0, 0.0);
    for j in ["A", "B"] {
        let p = m.points.get_mut(j).unwrap();
        p.restraint = [false, true, true, true, true, true];
        p.spring[0] = 100.0;
        p.mass[0] = mass;
    }
    m.points.get_mut("C").unwrap().restraint = [true; 6];
    let d = m.points.get_mut("D").unwrap();
    d.restraint = [false, true, true, true, true, true];
    d.mass[0] = mass;
    m.add_frame("BAR", "C", "D", "S");
    constraint(
        &mut m,
        "EQ",
        ConstraintType::Equal,
        [true, false, false, false, false, false],
        &["A", "B"],
    );
    let mut gap = LinkProp {
        name: "GAP".into(),
        link_type: LinkPropType::Gap,
        ..LinkProp::default()
    };
    gap.dof_nonlinear[0] = true;
    gap.ke[0] = 0.0;
    gap.k_nl[0] = 150.0;
    gap.param1[0] = 0.03;
    m.link_props.insert("GAP".into(), gap);
    m.links.insert(
        "L".into(),
        LinkObj {
            name: "L".into(),
            point_i: "A".into(),
            point_j: Some("B".into()),
            prop: "GAP".into(),
            angle_deg: 0.0,
            selected: false,
        },
    );
    pattern(&mut m, "P");
    force(&mut m, "A", "P", [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    m
}

#[test]
fn nonlinear_static_with_a_constraint_and_a_stiff_bar_is_not_singular() {
    let mut m = constrained_springs(0.0);
    let mut nl = LoadCase::linear_static("NL", "P", 1.0);
    nl.case_type = LoadCaseType::NonlinearStatic;
    m.load_cases.insert("NL".into(), nl);
    analyze_model(&mut m).unwrap();
    let u = &m.case_results["NL"].joint_displacements;
    // The penalty of the Equal constraint is 1e6 x 2.9e8 (the stiff bar), so the roundoff of
    // the penalty solve is about eps x penalty / (k1 + k2) = 2.2e-16 x 2.9e14 / 200 = 3e-4
    // relative: the tolerance is that bound (measured 6e-5), not 1e-6.
    close("u_A", u["A"][0], 0.05, 1e-3);
    close("u_B", u["B"][0], 0.05, 1e-3);
}

/// NonlinearDynamic of the same model with the step load P (mass 0.01 at A, B and D; undamped):
/// the step response of the two springs oscillates between 0 and 2 P / (k1 + k2) = 0.1 in, so
/// 0 <= u_A <= 0.1 at every step (bound with 1e-3 relative slack for the time integration).
#[test]
fn nonlinear_dynamic_with_a_constraint_and_a_stiff_bar_runs_within_the_step_bound() {
    let mut m = constrained_springs(0.01);
    m.functions.insert(
        "STEP".into(),
        DynamicFunction {
            name: "STEP".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: vec![(0.0, 1.0), (10.0, 1.0)],
        },
    );
    let mut nd = LoadCase::linear_static("ND", "P", 1.0);
    nd.loads.clear();
    nd.case_type = LoadCaseType::NonlinearDynamic;
    nd.modal_damping = 0.0;
    nd.history_loads = vec![("STEP".into(), "P".into(), 1.0)];
    // w^2 = (k1 + k2) / (2 m) = 2000, T = 0.14 s; the mass term of the integrator,
    // 4 m / dt^2 = 2000 at dt = 0.01 s, with k1 = 100 stays below the old pivot threshold of
    // 2900 (1e-5 of the stiff bar's diagonal), which is what the old code tripped on.
    nd.time_step_dt = 0.01;
    nd.num_time_steps = 100;
    // The residual of the penalty-constrained DOFs has a roundoff floor of about
    // eps x penalty x |u| = 2.2e-16 x 2.9e14 x 0.1 = 6e-3 (penalty = 1e6 x the bar's 2.9e8),
    // which the default relative tolerance 1e-9 cannot reach with scale about 10 to 100:
    // the tolerance is 1e-3 here (1e-4 stalled at a residual of 2e-5, 1e-2 broke the bound), a documented property of the penalty, not of the repair.
    nd.convergence_tol = 1e-3;
    m.load_cases.insert("ND".into(), nd);
    analyze_model(&mut m).unwrap();
    let trace = &m.case_results["ND"].time_history_disp["A"];
    assert_eq!(trace.len(), 101);
    let peak = trace.iter().fold(0.0f64, |p, d| p.max(d[0]));
    for (k, d) in trace.iter().enumerate() {
        assert!(d[0] >= -1e-9 && d[0] <= 0.1 * (1.0 + 1e-3), "step {k}: u_A = {}", d[0]);
    }
    assert!(peak > 0.09, "the step response must reach about 0.1, got {peak}");
}

/// Negative: the same springs and constraint without the stiff bar behave as before: the
/// static answer P / (k1 + k2) = 0.05 in.
#[test]
fn constrained_springs_without_the_stiff_bar_keep_their_answer() {
    let mut m = constrained_springs(0.0);
    m.frames.clear();
    let mut nl = LoadCase::linear_static("NL", "P", 1.0);
    nl.case_type = LoadCaseType::NonlinearStatic;
    m.load_cases.insert("NL".into(), nl);
    analyze_model(&mut m).unwrap();
    close("u_A", m.case_results["NL"].joint_displacements["A"][0], 0.05, 1e-6);
}

/// Negative of D112 (the brief's: a model without a constraint): the same joints and stiff bar
/// with no Equal constraint and a gap that stays open (opening 1000 in): A carries P alone,
/// u_A = P / k1 = 10 / 100 = 0.1 in and u_B = 0, to 1e-9 (no penalty is present, so no
/// penalty roundoff).
#[test]
fn nonlinear_static_without_a_constraint_is_exact() {
    let mut m = constrained_springs(0.0);
    m.constraints.clear();
    for p in m.points.values_mut() {
        p.constraints.clear();
    }
    m.link_props.get_mut("GAP").unwrap().param1[0] = 1000.0;
    let mut nl = LoadCase::linear_static("NL", "P", 1.0);
    nl.case_type = LoadCaseType::NonlinearStatic;
    m.load_cases.insert("NL".into(), nl);
    analyze_model(&mut m).unwrap();
    let u = &m.case_results["NL"].joint_displacements;
    close("u_A", u["A"][0], 0.1, 1e-9);
    assert!(u["B"][0].abs() < 1e-12, "u_B {}", u["B"][0]);
}

// ------------------------------------------------------------------ D111 -----------------

/// kip, in. Bar A-B along X, L = 100, EA/L = 3000 kip/in, a parallel spring of 500 kip/in at B,
/// limits `tc` = (compression on, C, tension on, T), axial loads in pattern DEAD.
fn tc_bar(tc: (bool, f64, bool, f64)) -> SapModel {
    let mut m = SapModel::default();
    m.active_dofs = [true, false, false, false, false, false];
    m.load_patterns.get_mut("DEAD").unwrap().self_wt_multiplier = 0.0;
    m.load_cases.clear();
    m.add_point("A", 0.0, 0.0, 0.0);
    m.add_point("B", 100.0, 0.0, 0.0);
    m.points.get_mut("A").unwrap().restraint = [true; 6];
    m.points.get_mut("B").unwrap().spring[0] = 500.0;
    let mut mat = Material::default_steel_kip_in();
    mat.name = "STEEL".into();
    mat.e = 30000.0;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    m.materials.insert("STEEL".into(), mat);
    m.frame_sections.insert(
        "BAR".into(),
        FrameSection {
            name: "BAR".into(),
            material: "STEEL".into(),
            area: 10.0,
            ..FrameSection::default()
        },
    );
    m.add_frame("BAR1", "A", "B", "BAR");
    m.frames.get_mut("BAR1").unwrap().tc_limits = Some(tc);
    let mut nl = LoadCase::linear_static("NL", "DEAD", 1.0);
    nl.case_type = LoadCaseType::NonlinearStatic;
    m.load_cases.insert("NL".into(), nl);
    m
}

fn axial_point(m: &mut SapModel, rel: f64, val: f64) {
    m.frames
        .get_mut("BAR1")
        .unwrap()
        .loads_point
        .push(FrameLoadPoint {
            load_pat: "DEAD".into(),
            my_type: 1,
            dir: 1,
            csys: "LOCAL".into(),
            rel_dist: rel,
            val,
        });
}

fn axial_uniform(m: &mut SapModel, q: f64) {
    m.frames
        .get_mut("BAR1")
        .unwrap()
        .loads_dist
        .push(FrameLoadDistributed {
            load_pat: "DEAD".into(),
            my_type: 1,
            dir: 1,
            csys: "LOCAL".into(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: q,
            val2: q,
        });
}

/// Tension-limited member (T = 50, no compression limit) with axial point loads +200 at
/// 0.25 L and -200 at 0.75 L: W = 0, but P(x) steps by 200 at each load, so the force inside
/// the member differs from the end forces by up to 200 and the extremes p_avg +- |W|/2 miss
/// it; refused by name. Both limits (T = 50, C = -60) with a uniform axial load
/// of 2 kip/in (W = 200 > T + |C| = 110): the member has no admissible state, refused.
#[test]
fn tc_members_with_sign_changing_or_excessive_axial_span_loads_are_refused() {
    let mut m = tc_bar((false, 0.0, true, 50.0));
    axial_point(&mut m, 0.25, 200.0);
    axial_point(&mut m, 0.75, -200.0);
    let err = analyze_model(&mut m).unwrap_err().to_string();
    assert!(err.contains("BAR1") && err.contains("NL") && err.contains("both signs"), "{err}");

    let mut m = tc_bar((true, -60.0, true, 50.0));
    axial_uniform(&mut m, 2.0);
    let err = analyze_model(&mut m).unwrap_err().to_string();
    assert!(err.contains("BAR1") && err.contains("NL") && err.contains("more than their sum"), "{err}");
}

/// Negative: a single-signed axial load keeps running. The tension-limited bar with the
/// uniform axial load 100 kip/in (W = 10000 >> T) of `adversarial_tc_limits.rs` still gives
/// the reaction -T = -50 at A; and with both limits a uniform load of 1 kip/in (W = 100 <=
/// T + |C| = 110) is accepted.
#[test]
fn tc_members_with_single_signed_axial_span_loads_still_run() {
    let mut m = tc_bar((false, 0.0, true, 50.0));
    axial_uniform(&mut m, 100.0);
    analyze_model(&mut m).unwrap();
    close("reaction at A", m.case_results["NL"].joint_reactions["A"][0], -50.0, 1e-6);

    let mut m = tc_bar((true, -60.0, true, 50.0));
    axial_uniform(&mut m, 1.0);
    analyze_model(&mut m).unwrap();
}

// ------------------------------------------------------------------ D110 -----------------

/// A column of modulus `e`, area `a`, length `l` along Z, its base joint held by a one-joint
/// link with all six degrees of freedom fixed, an axial load `p` at the top (consistent units,
/// the solver does not know them). A fixed link DOF is a joint constraint (Analysis Reference
/// p. 270), so the top displacement is the free elastic shortening p l / (e a).
/// (N, m) e = 2e11, a = 0.1, l = 3, p = 1e6: 1e6 x 3 / (2e11 x 0.1) = 1.5e-4 m; the old
/// 1e10 penalty (series with EA/L = 6.6667e9) gave 2.5e-4.
/// (kip, in) e = 29000, a = 10, l = 100, p = 10: 10 x 100 / (29000 x 10) = 3.4482759e-3 in.
fn fixed_link_column(e: f64, a: f64, l: f64, p: f64) -> f64 {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "M".into();
    mat.e = e;
    mat.u = 0.3;
    mat.g = e / 2.6;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    m.materials.insert("M".into(), mat);
    m.frame_sections.insert(
        "S".into(),
        FrameSection {
            name: "S".into(),
            material: "M".into(),
            area: a,
            i22: a * a / 12.0,
            i33: a * a / 12.0,
            torsional_j: a * a / 6.0,
            as2: 0.0,
            as3: 0.0,
            ..FrameSection::default()
        },
    );
    m.add_point("B", 0.0, 0.0, 0.0);
    m.add_point("T", 0.0, 0.0, l);
    m.add_frame("C", "B", "T", "S");
    let prop = LinkProp {
        name: "FIX".into(),
        link_type: LinkPropType::Linear,
        dof_active: [true; 6],
        dof_fixed: [true; 6],
        ..LinkProp::default()
    };
    m.link_props.insert("FIX".into(), prop);
    m.links.insert(
        "L".into(),
        LinkObj {
            name: "L".into(),
            point_i: "B".into(),
            point_j: None,
            prop: "FIX".into(),
            angle_deg: 0.0,
            selected: false,
        },
    );
    pattern(&mut m, "P");
    m.load_cases
        .insert("P".into(), LoadCase::linear_static("P", "P", 1.0));
    force(&mut m, "T", "P", [0.0, 0.0, -p, 0.0, 0.0, 0.0]);
    analyze_model(&mut m).unwrap();
    -m.case_results["P"].joint_displacements["T"][2]
}

#[test]
fn fixed_link_dofs_are_a_scaled_penalty_in_newton_metres() {
    close("top shortening (N, m)", fixed_link_column(2e11, 0.1, 3.0, 1e6), 1.5e-4, 1e-5);
}

/// Negative: a kip-in model with the same kind of link keeps its value (3.4482759e-3 in).
/// There EA/L = 2900 kip/in is tiny against both the old 1e10 and the new penalty (at least
/// 1e12), so either way the series error is below 1e-8 and the answer is the free elastic
/// shortening to 1e-4 relative.
#[test]
fn fixed_link_in_kip_inches_is_unchanged() {
    close(
        "top shortening (kip, in)",
        fixed_link_column(29000.0, 10.0, 100.0, 10.0),
        10.0 * 100.0 / (29000.0 * 10.0),
        1e-4,
    );
}

/// Interplay of D110 and the multipoint constraints: two (N, m) columns of the test above, each
/// base held by its own fixed link, the tops tied by an Equal constraint on U3, the load P on
/// one top only. The two columns share P, so each shortens by P L / (2 E A) = 1e6 x 3 /
/// (2 x 2e11 x 0.1) = 7.5e-5 m. The constraint takes the same penalty as the fixed link (1e6
/// times the largest diagonal before the links), so the two factors do not stack.
#[test]
fn fixed_links_and_an_equal_constraint_share_the_load() {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "M".into();
    mat.e = 2e11;
    mat.u = 0.3;
    mat.g = 2e11 / 2.6;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    m.materials.insert("M".into(), mat);
    m.frame_sections.insert(
        "S".into(),
        FrameSection {
            name: "S".into(),
            material: "M".into(),
            area: 0.1,
            i22: 1e-3,
            i33: 1e-3,
            torsional_j: 1e-3,
            as2: 0.0,
            as3: 0.0,
            ..FrameSection::default()
        },
    );
    m.link_props.insert(
        "FIX".into(),
        LinkProp {
            name: "FIX".into(),
            link_type: LinkPropType::Linear,
            dof_active: [true; 6],
            dof_fixed: [true; 6],
            ..LinkProp::default()
        },
    );
    for (i, x) in [0.0, 2.0].into_iter().enumerate() {
        let (b, t) = (format!("B{i}"), format!("T{i}"));
        m.add_point(b.clone(), x, 0.0, 0.0);
        m.add_point(t.clone(), x, 0.0, 3.0);
        m.add_frame(format!("C{i}"), b.clone(), t.clone(), "S");
        m.links.insert(
            format!("L{i}"),
            LinkObj {
                name: format!("L{i}"),
                point_i: b,
                point_j: None,
                prop: "FIX".into(),
                angle_deg: 0.0,
                selected: false,
            },
        );
    }
    constraint(
        &mut m,
        "EQ",
        ConstraintType::Equal,
        [false, false, true, false, false, false],
        &["T0", "T1"],
    );
    pattern(&mut m, "P");
    m.load_cases
        .insert("P".into(), LoadCase::linear_static("P", "P", 1.0));
    force(&mut m, "T0", "P", [0.0, 0.0, -1e6, 0.0, 0.0, 0.0]);
    analyze_model(&mut m).unwrap();
    let u = &m.case_results["P"].joint_displacements;
    // With the constraint penalty stacked on the link penalty (1e12 x the structure) this
    // gave 7.500097e-5, 1.3e-5 off; the tolerance below fails on that value.
    println!("tops {:e} {:e}", -u["T0"][2], -u["T1"][2]);
    close("top 0", -u["T0"][2], 7.5e-5, 5e-6);
    close("top 1", -u["T1"][2], 7.5e-5, 5e-6);
}

// ------------------------------------------------------------------ D109 -----------------

/// kN, m. Level cable, span 100, EA = 1e9 (E = 1e9, A = 1), self weight 1 kN/m, shaped for a
/// horizontal tension H = 1250 (or, in the negative test, a given undeformed length). Irvine's
/// elastic catenary for a level span (load per unit undeformed length, V = w L0 / 2 at each
/// end, 100 = H L0/EA + (2H/w) asinh(w L0 / (2H))), solved numerically in AUDIT/py/hand_cable.py
/// with `python3 -I`: L0 = 100.026544 for H = 1250 under the self weight. Case A (self weight
/// only): H = 1250.000, end tension 1251.000. Case B (self weight + 1 kN/m on the cable) acts
/// on that SAME cable length: H = 2494.183, end tension 2496.188. Before the change case B
/// shaped its own length (100.106575) and kept H = 1250: end tension 1254.002.
fn cable_model(params: CableParams) -> SapModel {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "M".into();
    mat.e = 1.0e9;
    mat.unit_mass = 0.0;
    mat.unit_weight = 1.0;
    m.materials.insert("M".into(), mat);
    m.frame_sections.insert(
        "S".into(),
        FrameSection {
            name: "S".into(),
            material: "M".into(),
            area: 1.0,
            ..FrameSection::default()
        },
    );
    m.add_point("I", 0.0, 0.0, 0.0);
    m.add_point("J", 100.0, 0.0, 0.0);
    for p in ["I", "J"] {
        m.points.get_mut(p).unwrap().restraint = [true; 6];
    }
    m.add_frame("C", "I", "J", "S");
    m.frames.get_mut("C").unwrap().cable_params = Some(params);
    m.load_patterns.insert(
        "DEAD".into(),
        LoadPattern {
            name: "DEAD".into(),
            pat_type: LoadPatternType::Dead,
            self_wt_multiplier: 1.0,
        },
    );
    pattern(&mut m, "LIVE");
    m.frames
        .get_mut("C")
        .unwrap()
        .loads_dist
        .push(FrameLoadDistributed {
            load_pat: "LIVE".into(),
            my_type: 1,
            dir: 6,
            csys: "GLOBAL".into(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: -1.0,
            val2: -1.0,
        });
    let mut a = LoadCase::linear_static("A", "DEAD", 1.0);
    a.case_type = LoadCaseType::NonlinearStatic;
    m.load_cases.insert("A".into(), a);
    let mut b = LoadCase::linear_static("B", "DEAD", 1.0);
    b.case_type = LoadCaseType::NonlinearStatic;
    b.loads.push(LoadCaseAssignment {
        load_name: "LIVE".into(),
        scale_factor: 1.0,
    });
    m.load_cases.insert("B".into(), b);
    m
}

fn end_tension(m: &SapModel, case: &str) -> f64 {
    let st = &m.case_results[case].frame_forces["C"];
    st.iter().map(|s| s.p.abs()).fold(0.0, f64::max)
}

#[test]
fn horizontal_tension_cable_is_shaped_by_self_weight_only() {
    let mut m = cable_model(CableParams {
        undeformed_length: None,
        target_tension_i: None,
        horizontal_tension: Some(1250.0),
        num_segments: 1,
    });
    analyze_model(&mut m).unwrap();
    // Measured: A 1251.000131 / H 1250.000000, B 2496.187859 / H 2494.182936 against the hand
    // values 1251.00013 / 1250 / 2496.18786 / 2494.18294 (2e-9 relative): the load acts per
    // unit undeformed length in the solver as in the hand model.
    close("case A end tension", end_tension(&m, "A"), 1251.00013, 1e-6);
    close("case B end tension", end_tension(&m, "B"), 2496.18786, 1e-6);
    // the horizontal component (the support reaction in X) is the shaped tension in case A
    close("case A H", m.case_results["A"].joint_reactions["I"][0].abs(), 1250.0, 1e-6);
    close("case B H", m.case_results["B"].joint_reactions["I"][0].abs(), 2494.18294, 1e-6);
}

/// Negative: a cable with a given undeformed length 100.026544 (the length above) is
/// case independent: case A gives 1251.000 as before, and case B the same 2496.188 as the
/// horizontal-tension cable, since the same length is loaded the same way.
#[test]
fn undeformed_length_cable_is_unchanged() {
    let mut m = cable_model(CableParams {
        undeformed_length: Some(100.026544),
        target_tension_i: None,
        horizontal_tension: None,
        num_segments: 1,
    });
    analyze_model(&mut m).unwrap();
    close("case A end tension", end_tension(&m, "A"), 1251.000, 1e-3);
    close("case B end tension", end_tension(&m, "B"), 2496.188, 1e-3);
}

/// Negative: a cable shaped by the tension at its I end (target_tension_i = 1251.00013 =
/// sqrt(1250^2 + 50.01327^2), the end tension of case A above) gets the same length from the
/// self weight and behaves like the horizontal-tension cable: 1251.000 and 2496.188.
#[test]
fn target_tension_i_cable_is_unchanged() {
    let mut m = cable_model(CableParams {
        undeformed_length: None,
        target_tension_i: Some(1251.00013),
        horizontal_tension: None,
        num_segments: 1,
    });
    analyze_model(&mut m).unwrap();
    close("case A end tension", end_tension(&m, "A"), 1251.000, 1e-3);
    close("case B end tension", end_tension(&m, "B"), 2496.188, 1e-3);
}

// ------------------------------------------------------------------ D107 -----------------

/// Consistent units (kN, m, t, s): one joint, mass 1 in X and Y, U3 and rotations restrained,
/// grounded springs k1 = 100 along (1, 1)/sqrt2 and k2 = 400 along (-1, 1)/sqrt2, as the
/// coupled global matrix [[250, -150], [-150, 250]]. Flat spectrum Sa = 1. Modes: w = 10 and
/// 20 rad/s (T = 0.62832, 0.31416 s), phi1 = (1, 1)/sqrt2, phi2 = (-1, 1)/sqrt2, so
/// Gamma1 = (1/sqrt2, 1/sqrt2), Gamma2 = (-1/sqrt2, 1/sqrt2). The response to the X spectrum
/// in X is mode 1: Gx phi_x Sa / w^2 = 0.5 / 100 = 0.005 and mode 2: 0.5 / 400 = 0.00125, so
/// SRSS of the modes gives sqrt(0.005^2 + 0.00125^2) = 0.0051539; the Y spectrum gives the same
/// UX magnitude, and SRSS of the two directions is UX = UY = sqrt(2) 0.0051539 = 0.0072887 m.
/// (Before: the signed sum Gx + Gy gave mode 1 doubled and mode 2 cancelled: 0.0100000.)
/// With CQC at 5 % damping the cross terms of the X and Y directions are +2 rho a b and
/// -2 rho a b in UX and cancel in the sum of squares: the same 0.0072887.
fn rs_model(combo: ModalComboMethod, directions: &[&str]) -> SapModel {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    m.add_point("J", 0.0, 0.0, 0.0);
    let p = m.points.get_mut("J").unwrap();
    p.mass = [1.0, 1.0, 0.0, 0.0, 0.0, 0.0];
    p.restraint = [false, false, true, true, true, true];
    let mut kc = [0.0f64; 21];
    kc[0] = 250.0; // U1U1
    kc[1] = -150.0; // U1U2
    kc[2] = 250.0; // U2U2 (column-major upper triangle: U1U1, U1U2, U2U2, ...)
    p.spring_coupled = Some(kc);
    m.load_cases
        .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 2));
    m.functions.insert(
        "SPEC".into(),
        DynamicFunction {
            name: "SPEC".into(),
            is_time_history: false,
            damping_ratio: 0.05,
            points: vec![(0.0, 1.0), (10.0, 1.0)],
        },
    );
    let mut rs = LoadCase::modal_eigen("RS", 2);
    rs.case_type = LoadCaseType::ResponseSpectrum;
    rs.initial_case = Some("MODAL".into());
    rs.modal_combo = combo;
    rs.modal_damping = 0.05;
    rs.spectrum_loads = directions
        .iter()
        .map(|d| ("SPEC".to_string(), d.to_string(), 1.0))
        .collect();
    m.load_cases.insert("RS".into(), rs);
    m
}

#[test]
fn spectrum_directions_are_combined_by_srss() {
    let single = (0.005f64.powi(2) + 0.00125f64.powi(2)).sqrt();
    assert!((single - 0.0051539).abs() < 1e-7);
    let want = std::f64::consts::SQRT_2 * single;
    for combo in [ModalComboMethod::Srss, ModalComboMethod::Cqc] {
        let mut m = rs_model(combo, &["U1", "U2"]);
        analyze_model(&mut m).unwrap();
        let u = m.case_results["RS"].joint_displacements["J"];
        close(&format!("UX {combo:?}"), u[0], want, 1e-6);
        close(&format!("UY {combo:?}"), u[1], want, 1e-6);
    }
}

/// Negative: a single direction is the modal combination alone, 0.0051539 in X (SRSS of
/// the two modes, closed form above) and two functions in one direction (U1 and its alias X)
/// are refused by name.
#[test]
fn a_single_spectrum_direction_is_unchanged_and_two_loads_in_one_are_refused() {
    let single = (0.005f64.powi(2) + 0.00125f64.powi(2)).sqrt();
    let mut m = rs_model(ModalComboMethod::Srss, &["U1"]);
    analyze_model(&mut m).unwrap();
    let u = m.case_results["RS"].joint_displacements["J"];
    close("UX", u[0], single, 1e-6);
    close("UY (the X spectrum in Y)", u[1], single, 1e-6);

    for dirs in [["U1", "U1"], ["U1", "X"]] {
        let mut m = rs_model(ModalComboMethod::Srss, &dirs);
        let err = analyze_model(&mut m).unwrap_err().to_string();
        assert!(err.contains("RS") && err.contains("more than one"), "{err}");
    }
}
