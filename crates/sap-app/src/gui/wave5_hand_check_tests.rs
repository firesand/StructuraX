//! Tests of package P16 of wave 5: the repairs of what the hand check of the
//! real program found (D52 to D55, D45). The application is run through
//! `render_ui`; the checked values are the strings it paints and the model and
//! window state it changes. The independent side of each comparison is named
//! in the test.

use super::wave5_windows_tests::Run;
use super::*;
use sap_model::FrameSection;

// ------------------------------------------------------------------- D52

/// I33 of a rectangle 0.1 m wide and 0.2 m deep: b h^3 / 12 = 0.1 * 0.008 /
/// 12 = 6.6667e-5 m4 (closed form, written out here).
const I33_M4: f64 = 0.1 * 0.2 * 0.2 * 0.2 / 12.0;

fn kn_m_model() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    m.frame_sections.clear();
    m.frame_sections.insert(
        "R".into(),
        FrameSection { name: "R".into(), area: 0.02, i33: I33_M4, ..FrameSection::default() },
    );
    m.add_point("1", 0.05, 0.0, 0.0);
    m.add_point("2", 3.2, 0.0, 0.0);
    m
}

#[test]
fn the_explorer_prints_section_properties_with_significant_digits() {
    let mut run = Run::new(kn_m_model());
    run.settle();
    // Before the repair: "A=0.02, I33=0.0".
    let line = run.all_texts().into_iter().find(|t| t.starts_with("• R")).expect("section line");
    let i33: f64 = line.split("I33=").nth(1).unwrap().trim().parse().unwrap();
    assert!((i33 - I33_M4).abs() <= 1e-4 * I33_M4, "{line}");
    assert!(line.contains("A=0.02,"), "{line}");
    // The same section in kip, in: 1 in = 0.0254 m, so I33 grows by 0.0254^-4.
    let mut m = kn_m_model();
    m.convert_units_in_place(Units::KipInF);
    let mut run = Run::new(m);
    run.settle();
    let want = I33_M4 / 0.0254_f64.powi(4);
    let line = run.all_texts().into_iter().find(|t| t.starts_with("• R")).expect("section line");
    let i33: f64 = line.split("I33=").nth(1).unwrap().trim().parse().unwrap();
    assert!((i33 - want).abs() <= 1e-4 * want, "{line}: want {want}");
    // A joint at x = 0.05 m is not "0.1" (the joint list is opened by the user).
    let mut run = Run::new(kn_m_model());
    run.click_text("Joints (2)");
    assert!(run.has_text("Jt 1: (0.05, 0, 0)"), "{:?}", run.all_texts());
}

#[test]
fn quantity_text_keeps_five_significant_digits_in_any_unit() {
    use view::quantity_text as q;
    assert_eq!(q(6.6666667e-5), "0.000066667");
    // At least four decimals and five significant digits.
    assert_eq!(q(7931.4159), "7931.4159");
    assert_eq!(q(14.7), "14.7");
    assert_eq!(q(0.00033299), "0.00033299");
    assert_eq!(q(0.0000123456789), "0.000012346");
    assert_eq!(q(-0.0), "0");
    assert_eq!(q(1.5e-9), "1.5e-9");
    assert_eq!(q(123456789.0), "123456789");
    assert_eq!(q(1.5e15), "1.5e15");
}
