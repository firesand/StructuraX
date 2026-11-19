//! Wave 5, ledger D12: measurements of the Ritz-vector recurrence on an
//! axisymmetric model.
//!
//! Model (kN, m): a 12-sided cylinder of thin shell quadrilaterals, radius 5,
//! height 8 in 4 courses, fixed base, unit translational mass at each joint
//! above the base (zero material density, so the mass matrix is the one written
//! here and the M inner products below are exact). Starting loads: unit
//! accelerations UX and UY.
//!
//! What is ASSERTED: the Ritz modes are M-orthonormal, `max |Phi' M Phi - I| <=
//! 1e-10`, because the algorithm M-orthonormalises every vector twice
//! (Analysis Reference p. 372: Gram-Schmidt against the earlier vectors) and the
//! modes are the Rayleigh-Ritz combinations of an M-orthonormal basis; 1e-10 is
//! the redundancy threshold of the algorithm, far above roundoff.
//!
//! What is MEASURED and printed (run with `--nocapture`), not asserted, because
//! nothing in the manual gives a value: the relative period difference of each
//! consecutive pair, the energy of the circumferential harmonics n != 1 relative
//! to n = 1 (DFT of the radial displacement around the top ring), the number of
//! vectors generated per load, and for the natural modes the period and dominant
//! harmonic. The manual (p. 372) says only that Ritz modes "are biased by the
//! starting load vectors" and are not natural modes.
use sap_model::*;
use sap_solver::analyze_model;

const SIDES: usize = 12;
const RADIUS: f64 = 5.0;
const COURSES: usize = 4;
const HEIGHT: f64 = 8.0;
const MASS: f64 = 1.0;

fn joint(ring: usize, k: usize) -> String {
    format!("R{ring}_{k}")
}

fn cylinder(vectors: Option<usize>, eigen_modes: usize) -> SapModel {
    let mut m = SapModel::new(sap_units::Units::KnMC);
    m.load_cases.clear();
    m.load_patterns.clear();
    m.area_sections.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "CONC".into();
    mat.e = 3.0e7;
    mat.u = 0.2;
    mat.g = 3.0e7 / 2.4;
    mat.unit_weight = 0.0;
    mat.unit_mass = 0.0;
    m.materials.insert("CONC".into(), mat);
    m.area_sections.insert(
        "WALL".into(),
        AreaSection {
            name: "WALL".into(),
            material: "CONC".into(),
            shell_type: ShellType::ShellThin,
            thickness_membrane: 0.1,
            thickness_bending: 0.1,
            ..AreaSection::default()
        },
    );
    for ring in 0..=COURSES {
        for k in 0..SIDES {
            let theta = 2.0 * std::f64::consts::PI * k as f64 / SIDES as f64;
            let z = HEIGHT * ring as f64 / COURSES as f64;
            let name = joint(ring, k);
            m.add_point(name.clone(), RADIUS * theta.cos(), RADIUS * theta.sin(), z);
            if ring == 0 {
                m.points[&name].restraint = [true; 6];
            } else {
                m.points[&name].mass = [MASS, MASS, MASS, 0.0, 0.0, 0.0];
            }
        }
    }
    for ring in 0..COURSES {
        for k in 0..SIDES {
            let k2 = (k + 1) % SIDES;
            m.add_area(
                format!("A{ring}_{k}"),
                vec![
                    joint(ring, k),
                    joint(ring, k2),
                    joint(ring + 1, k2),
                    joint(ring + 1, k),
                ],
                "WALL",
            );
        }
    }
    let mut case = LoadCase::modal_eigen("M", vectors.unwrap_or(eigen_modes));
    if vectors.is_some() {
        case.modal_sub_type = ModalSubType::Ritz;
        for load in ["UX", "UY"] {
            case.loads.push(LoadCaseAssignment {
                load_name: load.into(),
                scale_factor: 1.0,
            });
        }
    }
    m.load_cases.insert("M".into(), case);
    m
}

/// Mass-weighted inner product of two mode shapes (translations only: the
/// rotational masses are zero).
fn m_dot(a: &ModalModeResult, b: &ModalModeResult) -> f64 {
    a.mode_shape
        .iter()
        .map(|(name, va)| {
            let vb = &b.mode_shape[name];
            MASS * (0..3).map(|d| va[d] * vb[d]).sum::<f64>()
        })
        .sum::<f64>()
}

/// Energy of the circumferential harmonics of the radial displacement at the
/// top ring: `e[n]` for n = 0..=6 (DFT of 12 samples, conjugate pairs folded).
fn harmonics(mode: &ModalModeResult) -> [f64; 7] {
    let samples: Vec<f64> = (0..SIDES)
        .map(|k| {
            let theta = 2.0 * std::f64::consts::PI * k as f64 / SIDES as f64;
            let u = mode.mode_shape[&joint(COURSES, k)];
            u[0] * theta.cos() + u[1] * theta.sin()
        })
        .collect();
    let mut e = [0.0; 7];
    for (n, slot) in e.iter_mut().enumerate() {
        let (mut re, mut im) = (0.0, 0.0);
        for (k, s) in samples.iter().enumerate() {
            let a = 2.0 * std::f64::consts::PI * (n * k) as f64 / SIDES as f64;
            re += s * a.cos();
            im -= s * a.sin();
        }
        let power = (re * re + im * im) / (SIDES * SIDES) as f64;
        *slot = if n == 0 || n == SIDES / 2 {
            power
        } else {
            2.0 * power
        };
    }
    e
}

fn off_one_ratio(mode: &ModalModeResult) -> f64 {
    let e = harmonics(mode);
    let off: f64 = e
        .iter()
        .enumerate()
        .filter(|(n, _)| *n != 1)
        .map(|(_, v)| v)
        .sum();
    off / e[1].max(1e-300)
}

fn run(vectors: usize) -> SapModel {
    let mut m = cylinder(Some(vectors), 0);
    analyze_model(&mut m).unwrap();
    m
}

#[test]
fn ritz_modes_are_m_orthonormal_and_the_harmonic_content_is_measured() {
    for vectors in [10usize, 20] {
        let m = run(vectors);
        let case = &m.case_results["M"];
        assert_eq!(case.modes.len(), vectors, "modes requested {vectors}");
        let mut worst: f64 = 0.0;
        for (i, a) in case.modes.iter().enumerate() {
            for (j, b) in case.modes.iter().enumerate() {
                let want = if i == j { 1.0 } else { 0.0 };
                worst = worst.max((m_dot(a, b) - want).abs());
            }
        }
        println!("== {vectors} vectors: max |Phi' M Phi - I| = {worst:e}");
        assert!(worst <= 1e-10, "orthonormality {worst:e}");
        for p in &case.load_participation {
            println!("   load {} generated {}", p.load_name, p.vectors_generated);
        }
        println!("   mode  period [s]      pair diff (rel)   off-n=1 / n=1");
        for (i, mode) in case.modes.iter().enumerate() {
            let pair = if i % 2 == 1 {
                let prev = case.modes[i - 1].period_sec;
                format!("{:.3e}", (prev - mode.period_sec).abs() / prev)
            } else {
                "-".to_string()
            };
            println!(
                "   {:>3}   {:.9}   {:>14}   {:.3e}",
                i + 1,
                mode.period_sec,
                pair,
                off_one_ratio(mode)
            );
        }
    }
}

#[test]
fn harmonic_content_after_a_given_number_of_cycles_of_one_load() {
    // One load (UX) limited to c cycles: the span is c Ritz vectors. The
    // largest off-n=1 content among its modes, against c, shows how roundoff
    // in the other harmonics grows with the cycle count.
    for load in ["UX", "UY"] {
        println!("== {load} alone, c cycles: worst off-n=1 / n=1 (energy ratio) among the c modes");
        let mut previous = f64::NAN;
        for cycles in 1..=8usize {
            let mut m = cylinder(Some(cycles), 0);
            m.load_cases
                .get_mut("M")
                .unwrap()
                .loads
                .retain(|l| l.load_name == load);
            analyze_model(&mut m).unwrap();
            let modes = &m.case_results["M"].modes;
            let worst = modes.iter().map(off_one_ratio).fold(0.0_f64, f64::max);
            // Amplitude factor per cycle: the square root of the energy ratio.
            println!(
                "   c = {cycles}: {} modes, worst {worst:.3e}, amplitude {:.3e}, per cycle x{:.1}",
                modes.len(),
                worst.sqrt(),
                (worst / previous).sqrt()
            );
            previous = worst;
        }
    }
}

#[test]
fn natural_modes_by_dominant_harmonic() {
    // The hypothesis of the ledger: the shell harmonics n >= 2 are softer than
    // n = 1 (longer periods), so K^-1 M amplifies them more.
    let mut m = cylinder(None, 36);
    analyze_model(&mut m).unwrap();
    println!("== natural modes: period and dominant harmonic of the radial displacement");
    let mut first = [f64::NAN; 7];
    for (i, mode) in m.case_results["M"].modes.iter().enumerate() {
        let e = harmonics(mode);
        let n = (0..7).max_by(|a, b| e[*a].total_cmp(&e[*b])).unwrap();
        if first[n].is_nan() {
            first[n] = mode.period_sec;
        }
        println!("   {:>3}  T = {:.6}  n = {n}", i + 1, mode.period_sec);
    }
    println!("   first mode of each harmonic (n: T): {first:.5?}");
}
