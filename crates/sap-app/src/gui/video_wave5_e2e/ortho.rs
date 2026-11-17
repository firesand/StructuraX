//! An orthotropic area with a material angle, driven by prescribed joint displacements, through
//! Analyze > Run Analysis and the Shells form.

use super::ext::{analyze, menu};
use super::harness::{close, Run};
use sap_model::{
    AreaPropType, AreaSection, LoadCase, LoadPattern, LoadPatternType, Material, PlaneType,
    PointLoadDispl, SapModel,
};
use sap_units::Units;

#[test]
fn prescribed_displacements_on_an_orthotropic_area_give_the_rotated_resultants_in_the_element_axes() {
    // The orthotropic material of Example 2-015 (kip, in): E1 = 30000, E2 = 15000, nu_y = 0.2
    // (so nu_x = nu_y E1/E2 = 0.4, 1 - nu_x nu_y = 0.92), G12 = 6600, t = 2. A rectangle whose four
    // joints are held and displaced by u1 = 1e-4 x, u2 = 0 carries the uniform strain e11 = 1e-4
    // exactly (a bilinear element reproduces a linear field), so the resultants are those of the
    // plane-stress law rotated by the material angle theta (Jones, Mechanics of Composite
    // Materials, transformed reduced stiffness), written out here:
    //   Q11 = E1/0.92, Q22 = E2/0.92, Q12 = nu_y E1/0.92, Q66 = G12,
    //   Qb11 = Q11 c^4 + 2 (Q12 + 2 Q66) s^2 c^2 + Q22 s^4,
    //   Qb12 = (Q11 + Q22 - 4 Q66) s^2 c^2 + Q12 (s^4 + c^4),
    //   Qb16 = (Q11 - Q12 - 2 Q66) c^3 s - (Q22 - Q12 - 2 Q66) c s^3,
    //   F = t e11 (Qb11, Qb12, Qb16): theta 0: (6.52174, 1.30435, 0); theta 30: (5.35141, 1.65946, 0.91102).
    // The resultants of the element axes (the material axes would give another F11 and F12 at 30
    // degrees) are what the stored result and the Shells form show.
    let (e1, e2, nu_y, g12, t, e11) = (30000.0, 15000.0, 0.2, 6600.0, 2.0, 1.0e-4);
    let den = 1.0 - nu_y * (nu_y * e1 / e2);
    let (q11, q22, q12, q66) = (e1 / den, e2 / den, nu_y * e1 / den, g12);
    let mut f11s = Vec::new();
    for (angle, hand) in [(0.0_f64, [6.521_74, 1.304_35, 0.0]), (30.0, [5.351_41, 1.659_46, 0.911_02])] {
        let (s, c) = angle.to_radians().sin_cos();
        let f11 = t * e11 * (q11 * c.powi(4) + 2.0 * (q12 + 2.0 * q66) * s * s * c * c + q22 * s.powi(4));
        let f22 = t * e11 * ((q11 + q22 - 4.0 * q66) * s * s * c * c + q12 * (s.powi(4) + c.powi(4)));
        let f12 = t * e11 * ((q11 - q12 - 2.0 * q66) * c.powi(3) * s - (q22 - q12 - 2.0 * q66) * c * s.powi(3));
        // The closed form of this test agrees with the six digits the derivation above prints.
        for (got, want) in [f11, f22, f12].into_iter().zip(hand) {
            assert!((got - want).abs() <= 6.0e-6, "closed form {got} against {want}");
        }
        let mut mat = Material::default_steel_kip_in();
        mat.name = "ORTHO".into();
        mat.e = e1;
        mat.u = nu_y;
        mat.g = g12;
        mat.unit_weight = 0.0;
        mat.unit_mass = 0.0;
        mat.e_ortho = Some([e1, e2, 1.0]);
        mat.u_ortho = Some([nu_y, 0.0, 0.0]);
        mat.g_ortho = Some([g12, 11693.0, 11693.0]);
        mat.a_ortho = Some([0.0; 3]);
        let sec = AreaSection {
            name: "AS".into(),
            material: "ORTHO".into(),
            prop_type: AreaPropType::Shell,
            plane_type: PlaneType::PlaneStress,
            mat_angle_deg: angle,
            thickness_membrane: t,
            thickness_bending: t,
            ..AreaSection::default()
        };
        let mut m = SapModel::new(Units::KipInF);
        m.load_cases.clear();
        m.load_patterns.clear();
        m.frame_sections.clear();
        m.area_sections.clear();
        m.materials.insert("ORTHO".into(), mat);
        m.area_sections.insert("AS".into(), sec);
        for (id, x, y) in [("1", 0.0, 0.0), ("2", 96.0, 0.0), ("3", 96.0, 48.0), ("4", 0.0, 48.0)] {
            m.add_point(id, x, y, 0.0);
            m.points.get_mut(id).unwrap().restraint = [true; 6];
            m.points.get_mut(id).unwrap().loads_displ.push(PointLoadDispl {
                load_pat: "P".into(),
                csys: "GLOBAL".into(),
                displ: [e11 * x, 0.0, 0.0, 0.0, 0.0, 0.0],
            });
        }
        m.add_area("A", ["1", "2", "3", "4"].map(String::from).to_vec(), "AS");
        m.load_patterns.insert(
            "P".into(),
            LoadPattern { name: "P".into(), pat_type: LoadPatternType::Other, self_wt_multiplier: 0.0 },
        );
        m.load_cases.insert("P".into(), LoadCase::linear_static("P", "P", 1.0));
        let mut run = Run::new(m);
        analyze(&mut run);
        assert!(run.app.model.is_locked, "{angle}: {}", run.app.status_msg);
        let r = run.app.model.case_results["P"].area_results["A"].clone();
        eprintln!("orthotropic {angle}: stored F11 {} F22 {} F12 {} against {f11} {f22} {f12}", r.f11, r.f22, r.f12);
        // 1e-8: far above the roundoff measured here (1e-16) and far below the 18 % that the angle
        // changes F11 by.
        close(&format!("F11 at {angle}"), r.f11, f11, 1e-8);
        close(&format!("F22 at {angle}"), r.f22, f22, 1e-8);
        if f12.abs() > 1e-9 {
            close(&format!("F12 at {angle}"), r.f12, f12, 1e-8);
        } else {
            assert!(r.f12.abs() <= 1e-8 * f11, "F12 {} at {angle}", r.f12);
        }
        // The Shells form: Display > Show Forces/Stresses > Shells..., Resultant Forces, F11, OK.
        run.app.active_case = "P".into();
        run.settle();
        menu(&mut run, &["Display", "Show Forces/Stresses", "Shells..."]);
        run.click_text("Resultant Forces");
        run.click_text("F11");
        run.click_text("OK");
        run.settle();
        // The uniform field: every legend tick is the value, painted with four digits.
        assert!(run.has_text("Membrane F11 [kip/in]"), "{:?}", run.shown());
        assert!(run.has_text(&format!("{f11:.3}")), "tick {f11:.3} in {:?}", run.shown());
        f11s.push(f11);
    }
    // The angle matters: F11 differs by 18 % between the two.
    assert!((f11s[1] / f11s[0] - 1.0).abs() > 0.15, "{f11s:?}");
}
