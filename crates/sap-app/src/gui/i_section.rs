//! The custom I-section of the section dialog: top and bottom flange width and
//! thickness (D18). `sap_section::make_i_section` already takes the bottom
//! flange separately; this file only checks the dimensions the dialog passes it
//! and names the field that is wrong.


/// The six dimensions of an I-section, in model length units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct IDimensions {
    /// Total depth t3.
    pub d: f64,
    /// Top flange width t2.
    pub bt: f64,
    /// Top flange thickness tf.
    pub tt: f64,
    /// Bottom flange width t2b.
    pub bb: f64,
    /// Bottom flange thickness tfb.
    pub tb: f64,
    /// Web thickness tw.
    pub tw: f64,
}

/// Refuses a dimension that is not a positive finite number, naming it, and a
/// combination that leaves no web or a web wider than a flange.
pub(super) fn validate(s: &IDimensions) -> Result<(), String> {
    for (name, v) in [
        ("Depth t3", s.d),
        ("Top flange width t2", s.bt),
        ("Top flange thickness tf", s.tt),
        ("Bottom flange width t2b", s.bb),
        ("Bottom flange thickness tfb", s.tb),
        ("Web thickness tw", s.tw),
    ] {
        if !(v.is_finite() && v > 0.0) {
            return Err(format!("Invalid I-section: {name} must be a positive number (it is {v})."));
        }
    }
    if s.tt + s.tb >= s.d {
        return Err(format!(
            "Invalid I-section: the flanges tf + tfb = {} leave no web in the depth t3 = {}.",
            s.tt + s.tb,
            s.d
        ));
    }
    if s.tw >= s.bt {
        return Err(format!(
            "Invalid I-section: the web thickness tw = {} must be less than the top flange width t2 = {}.",
            s.tw, s.bt
        ));
    }
    if s.tw >= s.bb {
        return Err(format!(
            "Invalid I-section: the web thickness tw = {} must be less than the bottom flange width t2b = {}.",
            s.tw, s.bb
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use sap_model::FrameSection;
    use sap_section::make_i_section;

    /// The section the dialog makes: checked by `validate`, then `make_i_section`.
    fn build(name: &str, material: &str, d: &IDimensions) -> Result<FrameSection, String> {
        validate(d)?;
        Ok(make_i_section(name, material, d.d, d.bt, d.tt, d.tw, d.bb, d.tb))
    }

    const R3: IDimensions = IDimensions { d: 20.0, bt: 10.0, tt: 1.0, bb: 6.0, tb: 0.5, tw: 0.4 };

    /// Reference R3 (written out here, independent of `make_i_section`): three
    /// rectangles, flange 10 x 1 on top, web 0.4 x 18.5, flange 6 x 0.5 below.
    /// A = 10 + 7.4 + 3 = 20.4; centroid from the bottom
    /// (3 * 0.25 + 7.4 * 9.75 + 10 * 19.5) / 20.4 = 13.132352941;
    /// I33 = sum(b h^3 / 12 + A (y - ybar)^2); I22 = sum(h b^3 / 12) = 92.432.
    #[test]
    fn an_i_section_with_unequal_flanges_has_the_closed_form_properties() {
        let sec = build("UNEQUAL", "STEEL", &R3).unwrap();
        let a = [(6.0, 0.5, 0.25), (0.4, 18.5, 9.75), (10.0, 1.0, 19.5)];
        let area: f64 = a.iter().map(|(b, h, _)| b * h).sum();
        let ybar: f64 = a.iter().map(|(b, h, y)| b * h * y).sum::<f64>() / area;
        let i33: f64 = a.iter().map(|(b, h, y)| b * h * h * h / 12.0 + b * h * (y - ybar).powi(2)).sum();
        let i22: f64 = a.iter().map(|(b, h, _)| h * b * b * b / 12.0).sum();
        // The printed values of the reference.
        assert!((area - 20.4).abs() < 1e-12);
        assert!((ybar - 13.132352941).abs() < 1e-9);
        assert!((i22 - 92.432).abs() < 1e-9);
        assert!((i33 - 1199.942647).abs() < 1e-6, "{i33}");
        for (what, got, want) in [("area", sec.area, area), ("i33", sec.i33, i33), ("i22", sec.i22, i22)] {
            assert!((got - want).abs() <= 1e-9 * want, "{what}: {got} against {want}");
        }
    }

    #[test]
    fn equal_flanges_give_the_symmetric_section() {
        let sym = IDimensions { d: 24.0, bt: 9.0, tt: 0.68, bb: 9.0, tb: 0.68, tw: 0.44 };
        let sec = build("W", "STEEL", &sym).unwrap();
        let hw: f64 = 24.0 - 2.0 * 0.68;
        let i33 = 2.0 * (9.0 * 0.68_f64.powi(3) / 12.0 + 9.0 * 0.68 * (12.0 - 0.34_f64).powi(2))
            + 0.44 * hw.powi(3) / 12.0;
        assert!((sec.i33 - i33).abs() <= 1e-9 * i33);
    }

    #[test]
    fn a_wrong_dimension_is_refused_with_its_name() {
        for (mutate, name) in [
            (IDimensions { d: 0.0, ..R3 }, "Depth t3"),
            (IDimensions { bt: -1.0, ..R3 }, "Top flange width t2"),
            (IDimensions { tt: 0.0, ..R3 }, "Top flange thickness tf"),
            (IDimensions { bb: f64::NAN, ..R3 }, "Bottom flange width t2b"),
            (IDimensions { tb: -0.5, ..R3 }, "Bottom flange thickness tfb"),
            (IDimensions { tw: f64::INFINITY, ..R3 }, "Web thickness tw"),
        ] {
            let e = validate(&mutate).unwrap_err();
            assert!(e.contains(name), "{name}: {e}");
            assert!(build("X", "M", &mutate).is_err());
        }
        // Flanges that fill the depth; a web wider than either flange.
        assert!(validate(&IDimensions { tt: 10.0, tb: 10.0, ..R3 }).unwrap_err().contains("no web"));
        assert!(validate(&IDimensions { tw: 6.0, ..R3 }).unwrap_err().contains("bottom flange width"));
        assert!(validate(&IDimensions { bt: 0.3, ..R3 }).unwrap_err().contains("top flange width"));
    }
}
