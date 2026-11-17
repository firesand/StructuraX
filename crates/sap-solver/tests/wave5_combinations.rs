//! Wave 5, step 3a: combinations are a maximum and a minimum (CSI Analysis
//! Reference pp. 351-353). The arithmetic of every combination type on
//! hand-set results is tested in `combos.rs`; here the solver produces the
//! contributors. Units are kip, in, s.
use sap_model::results::is_single_valued;
use sap_model::*;
use sap_solver::analyze_model;

fn close(what: &str, got: f64, want: f64, tol: f64) {
    assert!(
        (got - want).abs() <= tol * want.abs().max(1.0),
        "{what}: got {got}, want {want}"
    );
}

fn combo(m: &mut SapModel, name: &str, kind: ComboType, cases: &[(&str, f64)]) {
    m.load_combos.insert(
        name.into(),
        LoadCombo {
            name: name.into(),
            combo_type: kind,
            cases: cases.iter().map(|(c, f)| (c.to_string(), *f)).collect(),
        },
    );
}

fn bare() -> SapModel {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    m
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

/// The example of p. 353 (COMB1, COMB2) with a real response-spectrum case:
/// one mass m = 2 on a spring k = 50 along U1 (everything else restrained),
/// so w^2 = k/m = 25. A static load of 175 gives LL = 175 / 50 = 3.5; the
/// constant spectrum Sa = 50 gives QUAKE = Sa / w^2 = 2.0 (one mode, one
/// degree of freedom, participation 1 for the mass-normalised mode).
fn sdof() -> SapModel {
    let mut m = bare();
    m.add_point("C", 0.0, 0.0, 0.0);
    let c = &mut m.points["C"];
    c.mass = [2.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    c.restraint = [false, true, true, true, true, true];
    c.spring = [50.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    pattern(&mut m, "LL");
    m.points["C"].loads_force.push(PointLoadForce {
        load_pat: "LL".into(),
        csys: "GLOBAL".into(),
        force: [175.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    });
    m.load_cases.insert("LL".into(), LoadCase::linear_static("LL", "LL", 1.0));
    m.load_cases.insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 1));
    m.functions.insert(
        "SPEC".into(),
        DynamicFunction {
            name: "SPEC".into(),
            is_time_history: false,
            damping_ratio: 0.05,
            points: vec![(0.0, 50.0), (10.0, 50.0)],
        },
    );
    let mut rs = LoadCase::linear_static("QUAKE", "LL", 1.0);
    rs.case_type = LoadCaseType::ResponseSpectrum;
    rs.loads.clear();
    rs.initial_case = Some("MODAL".into());
    rs.spectrum_loads = vec![("SPEC".into(), "U1".into(), 1.0)];
    m.load_cases.insert("QUAKE".into(), rs);
    m
}

#[test]
fn solver_spectrum_case_gives_the_manual_pairs_of_comb1_and_comb2() {
    let mut m = sdof();
    combo(&mut m, "COMB1", ComboType::LinearAdd, &[("LL", 1.0), ("QUAKE", 1.0)]);
    combo(&mut m, "COMB2", ComboType::Envelope, &[("LL", 1.0), ("QUAKE", 1.0)]);
    combo(&mut m, "STATIC", ComboType::LinearAdd, &[("LL", 2.0)]);
    analyze_model(&mut m).unwrap();
    // The tolerance 1e-9 covers the one-degree-of-freedom eigenvalue solution.
    close("LL", m.case_results["LL"].joint_displacements["C"][0], 3.5, 1e-9);
    close("QUAKE", m.case_results["QUAKE"].joint_displacements["C"][0], 2.0, 1e-9);
    let pair = |name: &str| {
        let hi = &m.case_results[name];
        let lo = hi.combo_min.as_ref().unwrap_or_else(|| panic!("{name} has no minimum"));
        (hi.joint_displacements["C"][0], lo.joint_displacements["C"][0])
    };
    // COMB1: 3.5 + 2.0 = 5.5 and 3.5 - 2.0 = 1.5 (p. 353).
    let (mx, mn) = pair("COMB1");
    close("COMB1 max", mx, 5.5, 1e-9);
    close("COMB1 min", mn, 1.5, 1e-9);
    // COMB2: max(3.5, 2.0) = 3.5 and min(3.5, -2.0) = -2.0.
    let (mx, mn) = pair("COMB2");
    close("COMB2 max", mx, 3.5, 1e-9);
    close("COMB2 min", mn, -2.0, 1e-9);
    // An additive combination of single-valued cases keeps one value.
    assert!(m.case_results["STATIC"].combo_min.is_none());
    assert!(is_single_valued(&m, "STATIC") && !is_single_valued(&m, "COMB1"));
    close("STATIC", m.case_results["STATIC"].joint_displacements["C"][0], 7.0, 1e-9);
}

/// A cantilever with automatic subdivision into 4 elements and two static
/// load cases of opposite sign: the envelope carries the minimum too, and both
/// rows have the stations of the object (not of the elements) and no internal
/// joints.
#[test]
fn envelope_minimum_is_collapsed_to_the_object_like_the_maximum() {
    let mut m = bare();
    m.active_dofs = [true, true, true, true, true, true];
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.unit_weight = 0.0;
    mat.unit_mass = 0.0;
    m.materials.insert("MAT".into(), mat);
    m.frame_sections.insert(
        "SEC".into(),
        FrameSection {
            name: "SEC".into(),
            material: "MAT".into(),
            prop_type: FramePropType::General,
            area: 10.0,
            i33: 100.0,
            i22: 100.0,
            torsional_j: 200.0,
            as2: 0.0,
            as3: 0.0,
            modifiers: [1.0; 8],
            ..FrameSection::default()
        },
    );
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 100.0, 0.0, 0.0);
    m.points["1"].restraint = [true; 6];
    m.add_frame("B", "1", "2", "SEC");
    m.frames["B"].auto_mesh_segments = 4;
    for (name, f) in [("UP", 3.0), ("DOWN", -5.0)] {
        pattern(&mut m, name);
        m.points["2"].loads_force.push(PointLoadForce {
            load_pat: name.into(),
            csys: "GLOBAL".into(),
            force: [0.0, 0.0, f, 0.0, 0.0, 0.0],
        });
        m.load_cases.insert(name.into(), LoadCase::linear_static(name, name, 1.0));
    }
    combo(&mut m, "ENV", ComboType::Envelope, &[("UP", 1.0), ("DOWN", 1.0)]);
    analyze_model(&mut m).unwrap();
    let up = &m.case_results["UP"];
    let hi = &m.case_results["ENV"];
    let lo = hi.combo_min.as_ref().expect("an envelope has a minimum");
    for r in [up, hi, lo] {
        assert_eq!(r.joint_displacements.keys().collect::<Vec<_>>(), ["1", "2"]);
    }
    let stations = |r: &CaseResults| -> Vec<f64> { r.frame_forces["B"].iter().map(|s| s.station).collect() };
    assert!(stations(up).len() >= 2);
    assert_eq!(stations(hi), stations(up));
    assert_eq!(stations(lo), stations(up));
    assert_eq!(lo.frame_forces.keys().collect::<Vec<_>>(), ["B"]);
    // Tip displacement along Z: maximum is that of UP (it is positive), the
    // minimum that of DOWN, and DOWN = -5/3 of UP (linear, same stiffness).
    let (u_up, (u_hi, u_lo)) = (
        up.joint_displacements["2"][2],
        (hi.joint_displacements["2"][2], lo.joint_displacements["2"][2]),
    );
    assert!(u_up > 0.0);
    close("max", u_hi, u_up, 1e-12);
    close("min", u_lo, -5.0 / 3.0 * u_up, 1e-9);
}
