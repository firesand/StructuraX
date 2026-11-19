//! The rows of the tutorial video that wave 5 closes: 1.1 Grid Only, 1.16 the
//! seismic eccentricity of a diaphragm, 1.17 the read-out of the loads of an
//! object, 3.2 a section with unequal flanges made in the dialog.

use super::ext::{analyze, click_on_row, menu, scroll_window, text_containing, unlock};
use super::harness::{close, Run};
use crate::gui::CustomSectionShape;
use sap_model::{
    ConstraintAxis, ConstraintDef, ConstraintType, FramePropType, FrameSection, LoadCase,
    LoadPattern, LoadPatternType, Material, SapModel,
};
use sap_units::Units;

// ------------------------------------------------------------------ row 1.1

#[test]
fn grid_only_in_kn_m_makes_a_3_by_3_by_3_grid_and_refuses_a_negative_spacing() {
    let mut run = Run::new(SapModel::new(Units::KipInF));
    // The units selector of the status bar, as a user picks it.
    run.click_text("Units: Kip, in, F");
    run.click_text("kN, m, C");
    assert_eq!(run.app.model.present_units, Units::KnMC);
    // File > New Model from Templates > Grid Only.
    run.app.model.add_point("OLD", 1.0, 1.0, 1.0);
    menu(&mut run, &["File", "New Model from Templates... (Ctrl+N)", "▦ Grid Only"]);
    assert!(run.app.model.points.is_empty(), "Grid Only starts from scratch");
    assert_eq!(run.app.model.present_units, Units::KnMC, "the units are kept");
    assert!(run.has_text("Create grid lines"), "the Quick Grid Lines form is open: {:?}", run.shown());
    // 3 x 3 bays of 6 m and 3 storeys of 3.5 m: 4 lines in each direction.
    for (i, v) in ["4", "4", "4"].iter().enumerate() {
        run.set_number_at("Lines in X", i, v);
    }
    for (i, v) in ["6", "6", "3.5"].iter().enumerate() {
        run.set_number_at("Spacing X", i, v);
    }
    for (i, v) in ["0", "0", "0"].iter().enumerate() {
        run.set_number_at("First line X", i, v);
    }
    run.click_text("Create grid lines");
    let grids = &run.app.model.coord_systems["GLOBAL"].grids;
    // Reference: line i of a direction lies at i x spacing (arithmetic of the request).
    for (axis, step) in [("X", 6.0), ("Y", 6.0), ("Z", 3.5)] {
        let got: Vec<f64> = grids.iter().filter(|g| g.axis == axis).map(|g| g.coordinate).collect();
        let want: Vec<f64> = (0..4).map(|i| i as f64 * step).collect();
        assert_eq!(got, want, "{axis} lines (exact: i x 6 and i x 3.5 are representable products)");
    }
    assert!(run.app.model.points.is_empty() && run.app.model.frames.is_empty() && run.app.model.areas.is_empty());
    // A negative spacing is refused with a message that names it, and the grid is unchanged.
    let before = run.app.model.coord_systems["GLOBAL"].grids.clone();
    run.set_number_at("Spacing X", 0, "-6");
    run.click_text("Create grid lines");
    assert!(run.app.status_msg.contains("spacing"), "{}", run.app.status_msg);
    assert_eq!(run.app.model.coord_systems["GLOBAL"].grids, before);
}

// ------------------------------------------------------------------ row 1.16

const G: f64 = 9.806_65;
const MASS: f64 = 10.0; // kN s^2 / m at each of the 4 joints of a storey
const H1: f64 = 3.0;
const H2: f64 = 6.0;
const COLUMNS: [(f64, f64); 4] = [(0.0, 0.0), (10.0, 0.0), (0.0, 6.0), (10.0, 6.0)];

/// Two storeys of four columns each (kN, m): rigid Z diaphragms D1 at z = 3 and
/// D2 at z = 6, masses at the storey joints, fixed bases (6 restraints at each of
/// the 4 base joints: no rigid-body motion is left), a Quake pattern EQ and the case CASE.
pub(super) fn two_storeys() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    m.load_cases.clear();
    m.load_patterns.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = 2.0e8;
    mat.g = 2.0e8 / 2.6;
    mat.unit_weight = 0.0;
    mat.unit_mass = 0.0;
    m.materials.insert("MAT".into(), mat);
    let sec = FrameSection {
        name: "SEC".into(),
        material: "MAT".into(),
        prop_type: FramePropType::General,
        area: 0.05,
        i33: 5.0e-4,
        i22: 5.0e-4,
        torsional_j: 1.0e-3,
        as2: 0.0,
        as3: 0.0,
        ..FrameSection::default()
    };
    m.frame_sections.insert("SEC".into(), sec);
    m.load_patterns.insert(
        "EQ".into(),
        LoadPattern { name: "EQ".into(), pat_type: LoadPatternType::Quake, self_wt_multiplier: 0.0 },
    );
    for n in ["D1", "D2"] {
        m.constraints.insert(
            n.into(),
            ConstraintDef {
                name: n.into(),
                constraint_type: ConstraintType::Diaphragm,
                axis: ConstraintAxis::Z,
                dofs: [true, true, false, false, false, true],
            },
        );
    }
    for (k, (x, y)) in COLUMNS.iter().enumerate() {
        let (b, a, t) = (format!("B{k}"), format!("L1{k}"), format!("L2{k}"));
        m.add_point(b.clone(), *x, *y, 0.0);
        m.points[&b].restraint = [true; 6];
        m.add_point(a.clone(), *x, *y, H1);
        m.add_point(t.clone(), *x, *y, H2);
        for (p, d) in [(&a, "D1"), (&t, "D2")] {
            m.points[p].mass = [MASS, MASS, MASS, 0.0, 0.0, 0.0];
            m.points[p].constraints.push(d.into());
        }
        m.add_frame(format!("C1{k}"), b, a.clone(), "SEC");
        m.add_frame(format!("C2{k}"), a, t, "SEC");
    }
    m.load_cases.insert("CASE".into(), LoadCase::linear_static("CASE", "EQ", 1.0));
    m
}

/// The storey forces of ASCE 7-10 for the data typed into the window below
/// (Ss 1.5, S1 0.5, Fa = Fv = 1, R 5, Ie 1, T = 1 s, TL 8 s), from the equations
/// of the Lateral Loads Manual section 2.14 written out here: V = Cs W with
/// Cs = min(SDS/(R/Ie), SD1/(T R/Ie)) >= 0.044 SDS Ie (12.8-2, 12.8-3, 12.8-5) and
/// F_x = Cvx V, Cvx = w_x h_x^k / sum(w h^k) with k = 1 + (T - 0.5)/2 = 1.25 for
/// 0.5 s < T < 2.5 s (12.8-11, 12.8-12).
pub(super) fn storey_forces() -> [f64; 2] {
    let (sds, sd1, ri, t): (f64, f64, f64, f64) = (2.0 / 3.0 * 1.5, 2.0 / 3.0 * 0.5, 5.0, 1.0);
    let cs = (sds / ri).min(sd1 / (t * ri)).max(0.044 * sds);
    let w = 4.0 * MASS * G;
    let v = cs * 2.0 * w;
    let k = 1.0 + (t - 0.5) / 2.0;
    let (a, b) = (H1.powf(k), H2.powf(k));
    [v * a / (a + b), v * b / (a + b)]
}

/// Opens Define > Mass Source and Auto Seismic Loads, the automatic tab, and types the
/// data of `storey_forces`, the direction and the ratio; then presses the set button.
pub(super) fn define_in_the_window(run: &mut Run, direction: &str, ratio: f64) {
    if run.has_text("Mass Source and Auto Seismic Loads") {
        scroll_window(run, "Mass Source and Auto Seismic Loads", -2000.0);
    } else {
        menu(run, &["Define", "Mass Source and Auto Seismic Loads..."]);
    }
    run.click_text("Auto seismic (ASCE 7-10)");
    click_on_row(run, "Direction", direction);
    run.set_number("Eccentricity ratio", &ratio.to_string());
    click_on_row(run, "Period", "User defined");
    run.set_number("T (s)", "1");
    for (label, v) in [("R", "5"), ("Omega0", "3"), ("Cd", "4.5"), ("Ie", "1"), ("Ss (g)", "1.5"), ("S1 (g)", "0.5"), ("TL (s)", "8"), ("Fa", "1"), ("Fv", "1")] {
        run.set_number(label, v);
    }
    scroll_window(run, "Mass Source and Auto Seismic Loads", 600.0);
    run.click_text("Set automatic lateral load");
    assert!(run.app.status_msg.contains("automatic lateral load"), "{}", run.app.status_msg);
}

/// Analyze, then scroll the window to the computed load it paints under the definition.
pub(super) fn analyze_and_show_report(run: &mut Run) {
    analyze(run);
    assert!(run.app.model.is_locked, "{}", run.app.status_msg);
    scroll_window(run, "Mass Source and Auto Seismic Loads", 4000.0);
    assert!(text_containing(run, "Computed in case 'CASE'").is_some(), "the report is not on screen: {:?}", run.shown());
}

/// The numbers of the painted line of diaphragm `d`: (F, width, e, Mz).
pub(super) fn painted_moment(run: &Run, d: &str) -> Option<(f64, f64, f64, f64)> {
    let line = text_containing(run, &format!("Diaphragm {d}: F = "))?;
    let num = |after: &str| -> f64 {
        let rest = line.split(after).nth(1).unwrap_or_else(|| panic!("'{after}' in '{line}'"));
        rest.split([',', ' ']).next().unwrap().parse().unwrap()
    };
    Some((num("F = "), num("width = "), num("e = "), num("Mz = ")))
}

#[test]
fn the_seismic_window_eccentricity_gives_ratio_times_width_times_force_and_the_statics_of_it() {
    let mut run = Run::new(two_storeys());
    let forces = storey_forces();
    let base_mz = |run: &Run| run.app.model.case_results["CASE"].base_reactions;
    // The case without eccentricity: the reference for the statics.
    define_in_the_window(&mut run, "X", 0.0);
    analyze_and_show_report(&mut run);
    assert!(run.app.model.is_locked, "{}", run.app.status_msg);
    assert!(painted_moment(&run, "D1").is_none(), "no moment line without an eccentricity");
    let base_x = base_mz(&run);
    // The painted report: the storey forces of the equations. Four decimals are printed.
    let report = &run.app.model.case_results["CASE"].auto_lateral[0];
    for (j, want) in [("L10", forces[0] / 4.0), ("L20", forces[1] / 4.0)] {
        let f = report.joint_forces.iter().find(|f| f.joint == j).unwrap().force;
        // Both sides are the same equations evaluated in floating point: 1e-12 covers powf and
        // the sums, nothing physical.
        close(&format!("joint force {j}"), f, want, 1e-12);
    }
    unlock(&mut run);
    define_in_the_window(&mut run, "Y", 0.0);
    analyze_and_show_report(&mut run);
    let base_y = base_mz(&run);
    // (direction, ratio, width of the diaphragms perpendicular to the load, sense of the
    // couple about Z by the project's choice: -1 for X loads, +1 for Y loads, times the sign
    // of the ratio).
    for (direction, ratio, width, sense, base) in [
        ("X", 0.05, 6.0, -1.0, base_x),
        ("X", -0.05, 6.0, -1.0, base_x),
        ("Y", 0.05, 10.0, 1.0, base_y),
    ] {
        unlock(&mut run);
        define_in_the_window(&mut run, direction, ratio);
        analyze_and_show_report(&mut run);
        assert!(run.app.model.is_locked, "{direction} {ratio}: {}", run.app.status_msg);
        let mut total = 0.0;
        for (d, f) in [("D1", forces[0]), ("D2", forces[1])] {
            let (pf, pw, pe, pm) = painted_moment(&run, d)
                .unwrap_or_else(|| panic!("{direction} {ratio}: no line of {d} in {:?}", run.shown()));
            let m = sense * ratio * width * f;
            total += m;
            // The window prints four decimals: the allowed difference is half a unit of
            // the last printed digit.
            assert!((pf - f).abs() <= 5.0e-5, "{d} F {pf} against {f}");
            assert!((pw - width).abs() <= 5.0e-5, "{d} width {pw}");
            assert!((pe - ratio * width).abs() <= 5.0e-5, "{d} e {pe}");
            assert!((pm - m).abs() <= 5.0e-5, "{direction} {ratio} {d}: Mz {pm} against {m}");
        }
        // Statics: the applied moment about Z of the loads is `total` more than before, so
        // the base reaction moment about the origin changes by -total and the shears do not
        // change. The diaphragms are penalty constraints: the measured equilibrium residual
        // is 1e-8 of F x width (as in sap-solver/tests/wave5_eccentricity.rs).
        let now = base_mz(&run);
        let scale = (forces[0] + forces[1]) * width;
        assert!(
            (now[5] - base[5] + total).abs() <= 1e-7 * scale,
            "{direction} {ratio}: MZ changed by {} against {}",
            now[5] - base[5],
            -total
        );
        for i in 0..5 {
            assert!((now[i] - base[i]).abs() <= 1e-7 * scale, "reaction {i} changed with the eccentricity");
        }
        assert!(total.abs() > 0.1 * scale * 0.05, "the moment is not trivial");
    }
}

// ------------------------------------------------------------------ row 3.2

/// A cantilever of 3 m with the section UNEQ made in the dialog, a tip load of 10 kN in -Z
/// (kN, m): fixed at joint 1 (6 restraints), the tip free.
fn cantilever() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    m.load_cases.clear();
    m.load_patterns.clear();
    m.frame_sections.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "STEEL".into();
    mat.e = 2.0e8;
    mat.u = 0.3;
    mat.g = 2.0e8 / 2.6;
    mat.unit_weight = 0.0;
    mat.unit_mass = 0.0;
    m.materials.insert("STEEL".into(), mat);
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 3.0, 0.0, 0.0);
    m.points["1"].restraint = [true; 6];
    m.load_patterns.insert(
        "P".into(),
        LoadPattern { name: "P".into(), pat_type: LoadPatternType::Other, self_wt_multiplier: 0.0 },
    );
    m.load_cases.insert("P".into(), LoadCase::linear_static("P", "P", 1.0));
    m.points.get_mut("2").unwrap().loads_force.push(sap_model::PointLoadForce {
        load_pat: "P".into(),
        csys: "GLOBAL".into(),
        force: [0.0, 0.0, -10.0, 0.0, 0.0, 0.0],
    });
    m
}

#[test]
fn a_section_with_unequal_flanges_made_in_the_dialog_gives_the_hand_deflection() {
    let mut m = cantilever();
    m.add_frame("F1", "1", "2", "TMP");
    m.frame_sections.insert("TMP".into(), FrameSection { name: "TMP".into(), material: "STEEL".into(), ..FrameSection::default() });
    m.frames.get_mut("F1").unwrap().selected = true;
    let mut run = Run::new(m);
    run.app.custom_sec_material = "STEEL".into();
    run.app.show_section_modal = true;
    run.app.custom_sec_name = "UNEQ".into();
    run.app.custom_sec_shape = CustomSectionShape::WideFlangeI;
    // Meters: d 0.20, top flange 0.10 x 0.010, bottom flange 0.06 x 0.005, web 0.004.
    (run.app.custom_sec_d, run.app.custom_sec_b, run.app.custom_sec_tf) = (0.20, 0.10, 0.010);
    (run.app.custom_sec_bb, run.app.custom_sec_tfb, run.app.custom_sec_tw) = (0.06, 0.005, 0.004);
    run.app.custom_sec_symmetric = false;
    run.settle();
    run.click_text("Add Custom Section (& Assign to Selected Frames)");
    let sec = run.app.model.frame_sections.get("UNEQ").unwrap_or_else(|| panic!("{}", run.app.status_msg)).clone();
    assert_eq!(run.app.model.frames["F1"].section, "UNEQ", "assigned to the selected frame");
    // I33 by hand: three rectangles (b, h, y of the centroid from the bottom), parallel-axis
    // theorem about the centroid of the whole.
    let parts = [(0.06, 0.005, 0.0025), (0.004, 0.185, 0.0975), (0.10, 0.010, 0.195)];
    let area: f64 = parts.iter().map(|(b, h, _)| b * h).sum();
    let ybar = parts.iter().map(|(b, h, y)| b * h * y).sum::<f64>() / area;
    let i33: f64 = parts.iter().map(|(b, h, y)| b * h * h * h / 12.0 + b * h * (y - ybar).powi(2)).sum();
    // 3e-4 + 7.4e-4 + 1e-3 = 2.04e-3 m2; ybar = 0.1415 m (hand: 3e-4*0.0025+7.4e-4*0.0975+1e-3*0.195
    // = 2.67e-4 over 2.04e-3).
    close("A", sec.area, 2.04e-3, 1e-9);
    close("I33", sec.i33, i33, 1e-9);
    // Beam theory with shear deformation: delta = P L^3 / (3 E I33) + P L / (G As2), the
    // textbook cantilever. The shear area for loads in the local 2 direction is taken as the
    // depth times the web thickness, As2 = t3 tw = 0.20 x 0.004 = 8e-4 m2 (the project's
    // convention for I sections, `sap_section::make_i_section`; the manual does not give one
    // in the Analysis Reference for a user-built section): the stored value is checked against it
    // and the comparison below uses the hand value. E = 2e8 and G = E / 2.6 are the material
    // typed in `cantilever`.
    let as2 = 0.20 * 0.004;
    close("As2", sec.as2, as2, 1e-12);
    run.app.model.frame_sections.shift_remove("TMP");
    analyze(&mut run);
    assert!(run.app.model.is_locked, "{}", run.app.status_msg);
    let uz = run.app.model.case_results["P"].joint_displacements["2"][2];
    let want = -(10.0 * 27.0 / (3.0 * 2.0e8 * i33) + 10.0 * 3.0 / (2.0e8 / 2.6 * as2));
    close("tip deflection", uz, want, 1e-9);
    // Without the shear term the comparison would be off by the shear share (1.3 %), so the
    // tolerance 1e-9 also checks that term; the bending term alone is 98.7 % of the total. The
    // tolerance is that of a closed form against a linear solve of a one-element model.
    // A section of equal flanges with the same top flange has another I33: the deflection
    // would differ by more than 10 %, so a dialog that ignored the bottom flange fails here.
    let equal_i33: f64 = {
        let ye = [(0.10, 0.010, 0.005), (0.004, 0.18, 0.1), (0.10, 0.010, 0.195)];
        let a: f64 = ye.iter().map(|(b, h, _)| b * h).sum();
        let yb = ye.iter().map(|(b, h, y)| b * h * y).sum::<f64>() / a;
        ye.iter().map(|(b, h, y)| b * h * h * h / 12.0 + b * h * (y - yb).powi(2)).sum()
    };
    assert!((equal_i33 - i33).abs() > 0.1 * i33, "the unequal section is distinguishable");
}

// ------------------------------------------------------------------ row 1.17

/// The numbers in a painted text, in order ("-12.5 kN/m at 0 m ..." gives -12.5, 0, ...): the words
/// that parse as a number (a unit such as "kN/m2" does not).
fn numbers_in(text: &str) -> Vec<f64> {
    text.split([' ', ',', '='])
        .filter_map(|w| w.parse::<f64>().ok())
        .collect()
}

#[test]
fn the_object_window_paints_the_assigned_loads_in_the_display_units_asked_for() {
    use crate::gui::DisplayMode;
    use sap_model::{AreaLoadUniform, FrameLoadDistributed, FrameLoadPoint, PointLoadForce};
    // A slab of 6 m x 6 m with a beam on its lower edge (kN, m). The loads are assigned values:
    // the beam -12.5 kN/m over its length and a point load -30 kN at mid-span, joint 3 a force
    // -5 kN in Z, the slab -3.5 kN/m2 in global Z.
    let mut m = SapModel::new(Units::KnMC);
    m.load_cases.clear();
    for (id, x, y) in [("1", 0.0, 0.0), ("2", 6.0, 0.0), ("3", 6.0, 6.0), ("4", 0.0, 6.0)] {
        m.add_point(id, x, y, 0.0);
    }
    m.add_frame("F1", "1", "2", "FS");
    m.add_area("A1", ["1", "2", "3", "4"].map(String::from).to_vec(), "SH");
    m.frames.get_mut("F1").unwrap().loads_dist.push(FrameLoadDistributed {
        load_pat: "DEAD".into(),
        my_type: 1,
        dir: 6,
        csys: "GLOBAL".into(),
        rel_dist1: 0.0,
        rel_dist2: 1.0,
        val1: -12.5,
        val2: -12.5,
    });
    m.frames.get_mut("F1").unwrap().loads_point.push(FrameLoadPoint {
        load_pat: "LIVE".into(),
        my_type: 1,
        dir: 6,
        csys: "GLOBAL".into(),
        rel_dist: 0.5,
        val: -30.0,
    });
    m.points.get_mut("3").unwrap().loads_force.push(PointLoadForce {
        load_pat: "DEAD".into(),
        csys: "GLOBAL".into(),
        force: [0.0, 0.0, -5.0, 0.0, 0.0, 0.0],
    });
    m.areas.get_mut("A1").unwrap().loads_uniform.push(AreaLoadUniform {
        load_pat: "LIVE".into(),
        csys: "GLOBAL".into(),
        dir: 6,
        val: -3.5,
    });
    let mut run = Run::new(m);
    run.app.display_mode = DisplayMode::Undeformed;
    run.settle();
    // Joint 3: right-click, the window opens with the joint's force record.
    run.right_click(run.screen([6.0, 6.0, 0.0]));
    assert!(run.has_text("Object Information") && run.has_text("Joint 3 - Loads"), "{:?}", run.shown());
    let force = text_containing(&run, "F3 = ").expect("the joint force line");
    assert_eq!(numbers_in(&force), vec![0.0, 0.0, -5.0, 0.0, 0.0, 0.0]);
    // The beam, in the model units: the distributed record and the point record.
    run.right_click(run.screen([3.0, 0.0, 0.0]));
    assert!(run.has_text("Frame F1 - Loads"), "{:?}", run.shown());
    let dist = text_containing(&run, "kN/m at").expect("the distributed load line");
    assert_eq!(numbers_in(&dist), vec![-12.5, 0.0, -12.5, 6.0]);
    let point = text_containing(&run, "-30 kN at").expect("the point load line");
    assert_eq!(numbers_in(&point), vec![-30.0, 3.0]);
    assert!(run.has_text("DEAD") && run.has_text("LIVE"));
    // N, mm: 1 kN = 1000 N and 1 m = 1000 mm, so 1 kN/m = 1 N/mm (exact).
    run.choose("Units", "N, mm, C");
    let dist = text_containing(&run, "N/mm at").expect("N/mm");
    assert_eq!(numbers_in(&dist), vec![-12.5, 0.0, -12.5, 6000.0]);
    assert_eq!(numbers_in(&text_containing(&run, "-30000 N at").expect("N")), vec![-30000.0, 3000.0]);
    // Kip, ft: 1 kip = 4.4482216152605 kN (definition of the pound-force), 1 ft = 0.3048 m
    // (definition). The window prints six decimals: half a unit of the last one is allowed.
    run.choose("Units", "Kip, ft, F");
    let (kip, ft) = (4.448_221_615_260_5, 0.3048);
    let dist = numbers_in(&text_containing(&run, "kip/ft at").expect("kip/ft"));
    let (w, len) = (-12.5 * ft / kip, 6.0 / ft);
    for (got, want) in dist.iter().zip([w, 0.0, w, len]) {
        assert!((got - want).abs() <= 5.0e-7, "{dist:?} against {want}");
    }
    // The slab: -3.5 kN/m2 in kip/ft2 = -3.5 x 0.3048^2 / 4.4482216152605.
    run.right_click(run.screen([3.0, 3.0, 0.0]));
    assert!(run.has_text("Area A1 - Loads"), "{:?}", run.shown());
    let area = numbers_in(&text_containing(&run, "kip/ft2").expect("kip/ft2 line"));
    let want = -3.5 * ft * ft / kip;
    assert!((area[0] - want).abs() <= 5.0e-7, "{area:?} against {want}");
    assert!(run.has_text("LIVE") && run.has_text("Global Z"));
}
