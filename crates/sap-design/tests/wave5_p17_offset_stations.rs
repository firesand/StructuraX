//! Wave 5, P17: design of members with end offsets. The output stations of such a member run
//! from the face of the support at the I end (distance `ioff` from joint I) to the face at
//! the J end (`L - joff`); `rel_dist = station / L`, so no station is at 0 or 1 (CSI
//! Analysis Reference, "Effect upon Internal Force Output").
use sap_design::{
    steel::compute_cb_for_unbraced_ratio, compute_cb_from_stations, try_run_concrete_design,
    try_run_steel_design,
};
use sap_model::{
    CaseResults, ConcreteFramingType, ConcreteRebarConfig, FrameStationResult, MatType, Material,
    SapModel, SteelDesignOverwrite,
};
use sap_section::make_rectangular;
use sap_units::Units;

/// Stations at the distances `x` (from joint I) of a member of length `len`, with the given
/// M3 and a V2 that does not depend on the position.
fn at(len: f64, x: &[f64], m3: &[f64], v2: f64) -> Vec<FrameStationResult> {
    x.iter()
        .zip(m3)
        .map(|(&x, &m)| FrameStationResult {
            station: x,
            rel_dist: x / len,
            v2,
            m3: m,
            ..Default::default()
        })
        .collect()
}

fn near(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol * b.abs().max(1e-300)
}

// ---------------------------------------------------------------------------------------
// Cb: the quarter points refer to the span between the faces of the supports.
// ---------------------------------------------------------------------------------------

/// AISC F1-1 (CSI Steel Frame Design AISC 360-10, p. 3-34): Cb = 12.5 Mmax /
/// (2.5 Mmax + 3 MA + 4 MB + 3 MC), M at the quarter, centre and three-quarter point of the
/// unbraced segment. Member L = 120 with offsets 10 (I) and 20 (J): the segment runs from
/// x = 10 to x = 100 (clear length 90); its quarter points are x = 32.5, 55 and 77.5.
/// M3 at x = 10, 32.5, 55, 77.5, 100 is 0, 30, 40, 30, 0:
/// Cb = 12.5 * 40 / (2.5 * 40 + 3 * 30 + 4 * 40 + 3 * 30) = 500 / 440.
/// The baseline samples relative 0.25, 0.5, 0.75 of the whole L (x = 30, 60, 90).
#[test]
fn cb_quarter_points_lie_between_the_support_faces() {
    let len = 120.0;
    let s = at(len, &[10.0, 32.5, 55.0, 77.5, 100.0], &[0.0, 30.0, 40.0, 30.0, 0.0], 0.0);
    let hand = 500.0 / 440.0;
    // Interpolation between stations leaves roundoff of a few ulps.
    assert!(near(compute_cb_from_stations(&s), hand, 1e-12), "{}", compute_cb_from_stations(&s));
    // The same diagram as a plain station list of the clear length (0 to 1).
    let plain = at(90.0, &[0.0, 22.5, 45.0, 67.5, 90.0], &[0.0, 30.0, 40.0, 30.0, 0.0], 0.0);
    assert!(near(compute_cb_from_stations(&plain), hand, 1e-12));
}

/// Three stations x = 10, 55, 100 of L = 120 with M3 = 0, 40, 0: the quarter points of the
/// span (x = 32.5, 77.5) read 20 by linear interpolation: Cb = 500 / (100 + 60 + 160 + 60).
#[test]
fn cb_interpolates_inside_the_stations() {
    let s = at(120.0, &[10.0, 55.0, 100.0], &[0.0, 40.0, 0.0], 0.0);
    assert!(near(compute_cb_from_stations(&s), 500.0 / 380.0, 1e-12));
}

/// Braced at half the (clear) length: two segments of 45 each, M3 from 0 at x = 10 to 40 at
/// x = 55 and back to 0 at x = 100. In the first segment Mmax = 40, MA = 10, MB = 20, MC = 30
/// (linear), Cb = 500 / (100 + 30 + 80 + 90) = 500 / 300; the second is its mirror image.
/// The baseline reads the moment at relative 0 (no station there) as the last station's.
#[test]
fn cb_of_unbraced_segments_is_measured_from_the_first_face() {
    let s = at(120.0, &[10.0, 55.0, 100.0], &[0.0, 40.0, 0.0], 0.0);
    assert!(near(compute_cb_for_unbraced_ratio(&s, 0.5), 500.0 / 300.0, 1e-12));
    let plain = at(90.0, &[0.0, 45.0, 90.0], &[0.0, 40.0, 0.0], 0.0);
    assert!(near(compute_cb_for_unbraced_ratio(&plain, 0.5), 500.0 / 300.0, 1e-12));
}

// ---------------------------------------------------------------------------------------
// Steel: a member with offsets equals the same demands on a plain station list.
// ---------------------------------------------------------------------------------------

fn steel_member(len: f64, off_i: f64, off_j: f64) -> SapModel {
    let mut m = SapModel::new(Units::KipInF);
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 0.0, 0.0, len);
    m.points["1"].restraint = [true, true, true, false, false, true];
    m.points["2"].restraint = [true, true, false, false, false, false];
    m.add_frame("B1", "1", "2", "FSEC1");
    let f = m.frames.get_mut("B1").unwrap();
    f.end_offset_i = off_i;
    f.end_offset_j = off_j;
    m
}

fn put(m: &mut SapModel, stations: Vec<FrameStationResult>) {
    let mut cr = CaseResults {
        case_name: "S".into(),
        ..Default::default()
    };
    cr.frame_forces.insert("B1".into(), stations);
    m.case_results.insert("S".into(), cr);
}

/// L = 510 with offsets 10 and 20: the clear length 480 is the unbraced length, and the
/// lateral-torsional buckling governs. The moment diagram (kip-in) over the five stations
/// at 10, 130, 250, 370, 490 is 100, 1500, 2000, 900, 0. The design must equal the design of
/// the same five stations on a member of L = 480 without offsets (stations at 0, 120, 240,
/// 360, 480), and the Cb must be the one of the diagram: with MA = 1500, MB = 2000,
/// MC = 900 and Mmax = 2000 it is 25000 / (5000 + 4500 + 8000 + 2700) = 25000 / 20200. A
/// design with that Cb given as an overwrite has the same ratio.
#[test]
fn steel_with_offsets_equals_the_plain_station_list() {
    let m3 = [100.0, 1500.0, 2000.0, 900.0, 0.0];
    let mut a = steel_member(510.0, 10.0, 20.0);
    put(&mut a, at(510.0, &[10.0, 130.0, 250.0, 370.0, 490.0], &m3, 1.0));
    try_run_steel_design(&mut a).unwrap();
    let mut b = steel_member(480.0, 0.0, 0.0);
    put(&mut b, at(480.0, &[0.0, 120.0, 240.0, 360.0, 480.0], &m3, 1.0));
    try_run_steel_design(&mut b).unwrap();
    let (ra, rb) = (&a.steel_design_results["B1"], &b.steel_design_results["B1"]);
    assert!(
        near(ra.total_ratio, rb.total_ratio, 1e-12),
        "{} against {}",
        ra.total_ratio,
        rb.total_ratio
    );
    let with_cb = |cb: f64| {
        let mut c = steel_member(480.0, 0.0, 0.0);
        c.frames.get_mut("B1").unwrap().steel_overwrite = Some(SteelDesignOverwrite {
            cb: Some(cb),
            ..Default::default()
        });
        put(&mut c, at(480.0, &[0.0, 120.0, 240.0, 360.0, 480.0], &m3, 1.0));
        try_run_steel_design(&mut c).unwrap();
        c.steel_design_results["B1"].total_ratio
    };
    assert!(near(ra.total_ratio, with_cb(25000.0 / 20200.0), 1e-12));
    // The Cb enters the ratio: with Cb = 1 the ratio is larger.
    assert!(with_cb(1.0) > ra.total_ratio * 1.01);
}

// ---------------------------------------------------------------------------------------
// Concrete beam: the ends are the first and the last station.
// ---------------------------------------------------------------------------------------

fn beam(len: f64, off_i: f64, off_j: f64, framing: ConcreteFramingType) -> SapModel {
    let mut m = SapModel::new(Units::KipInF);
    m.concrete_design_code = "ACI 318-14".into();
    let mut concrete = Material::default_concrete_kip_in();
    concrete.fy = 60.0;
    concrete.fys = 60.0;
    m.materials.insert(concrete.name.clone(), concrete);
    let mut section = make_rectangular("CSEC", "4000Psi", 20.0, 20.0);
    let mut rebar = Material::default_steel_kip_in();
    rebar.name = "REBAR".into();
    rebar.mat_type = MatType::Rebar;
    rebar.fy = 60.0;
    rebar.fys = 60.0;
    m.materials.insert("REBAR".into(), rebar);
    section.concrete_rebar = Some(ConcreteRebarConfig {
        is_column: false,
        rebar_mat_long: "REBAR".into(),
        rebar_mat_confine: "REBAR".into(),
        cover_top: 2.5,
        cover_bot: 2.5,
        num_bars_3dir: 4,
        num_bars_2dir: 4,
        total_rebar_area: 8.0,
    });
    m.frame_sections.insert("CSEC".into(), section);
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", len, 0.0, 0.0);
    m.add_frame("B1", "1", "2", "CSEC");
    let f = m.frames.get_mut("B1").unwrap();
    f.end_offset_i = off_i;
    f.end_offset_j = off_j;
    f.concrete_framing = framing;
    m
}

fn beam_numbers(m: &mut SapModel) -> [f64; 4] {
    try_run_concrete_design(m).unwrap();
    let r = &m.concrete_design_results["B1"];
    [r.as_top, r.as_bot, r.as_side, r.av_over_s]
}

/// Special moment frame (ACI 18.6.3.2; CSI Concrete Frame Design ACI 318-14, section 3.5.1:
/// "At any end (support) of the beam, the beam positive moment capacity ... would not be less
/// that 1/2 of the beam negative moment capacity ... at that end", and "1/4 of the maximum of
/// positive or negative moment capacities of any of the beam end (support) stations").
/// L = 270, offsets 10 and 20, stations at 10, 70, 130, 190, 250 with M3 = -1000, 200, 500,
/// 100, -3000 kip-in (the larger end moment is at the LAST station). By hand: the end
/// positive moment at the last station is at least 0.5 * 3000 = 1500, at the first 500, the
/// largest end moment is 3000 and the span floor 0.25 * 3000 = 750, so the design positive
/// moment is 1500 and the negative one 3000. The reference is the design of ordinary framing
/// (no floors) of M3 = -1000, 200, 1500, 100, -3000, whose positive and negative demands are
/// those numbers. On the baseline no station is an end (rel_dist is 10/270 to 250/270), the
/// largest end moment is zero and the positive demand stays 500.
#[test]
fn special_moment_frame_floors_act_at_the_support_faces() {
    let v = 2.0;
    let x = [10.0, 70.0, 130.0, 190.0, 250.0];
    let mut a = beam(270.0, 10.0, 20.0, ConcreteFramingType::SwaySpecial);
    put(&mut a, at(270.0, &x, &[-1000.0, 200.0, 500.0, 100.0, -3000.0], v));
    let got = beam_numbers(&mut a);
    let xr = [0.0, 60.0, 120.0, 180.0, 240.0];
    let mut r = beam(240.0, 0.0, 0.0, ConcreteFramingType::SwayOrdinary);
    put(&mut r, at(240.0, &xr, &[-1000.0, 200.0, 1500.0, 100.0, -3000.0], v));
    let want = beam_numbers(&mut r);
    for k in 0..4 {
        assert!(near(got[k], want[k], 1e-12), "item {k}: {} against {}", got[k], want[k]);
    }
    // The floor acts: without it the positive demand is 500 and the bottom steel smaller.
    let mut o = beam(270.0, 10.0, 20.0, ConcreteFramingType::SwayOrdinary);
    put(&mut o, at(270.0, &x, &[-1000.0, 200.0, 500.0, 100.0, -3000.0], v));
    let plain = beam_numbers(&mut o);
    assert!(got[1] > plain[1] * 1.5, "{} against {}", got[1], plain[1]);
    // The same member without offsets and with the same demands as a plain list.
    let mut p = beam(240.0, 0.0, 0.0, ConcreteFramingType::SwaySpecial);
    put(&mut p, at(240.0, &xr, &[-1000.0, 200.0, 500.0, 100.0, -3000.0], v));
    let same = beam_numbers(&mut p);
    for k in 0..4 {
        assert!(near(got[k], same[k], 1e-12), "item {k}");
    }
}

/// Intermediate moment frame (ACI 18.4.2.2): end positive at least 1/3 of the negative, span
/// at least 1/5 of the largest end. Ends (first station) M3 = -3000 and (last) -600 with a
/// midspan of +400: the end positive floor at the first end is 1000, so the design positive
/// moment is 1000, the span floor 0.2 * 3000 = 600 is below. Reference: ordinary framing
/// with the midspan 1000.
#[test]
fn intermediate_moment_frame_floors_act_at_the_support_faces() {
    let v = 2.0;
    let mut a = beam(270.0, 10.0, 20.0, ConcreteFramingType::SwayIntermediate);
    put(&mut a, at(270.0, &[10.0, 130.0, 250.0], &[-3000.0, 400.0, -600.0], v));
    let got = beam_numbers(&mut a);
    let mut r = beam(240.0, 0.0, 0.0, ConcreteFramingType::SwayOrdinary);
    put(&mut r, at(240.0, &[0.0, 120.0, 240.0], &[-3000.0, 1000.0, -600.0], v));
    let want = beam_numbers(&mut r);
    for k in 0..4 {
        assert!(near(got[k], want[k], 1e-12), "item {k}: {} against {}", got[k], want[k]);
    }
}

// ---------------------------------------------------------------------------------------
// Members without offsets: the numbers are those of the baseline, bit for bit.
// ---------------------------------------------------------------------------------------

fn bits(values: &[f64]) -> Vec<u64> {
    values.iter().map(|v| v.to_bits()).collect()
}

/// Everything without offsets that reads the stations by position, collected as numbers.
fn no_offset_numbers() -> Vec<(&'static str, Vec<f64>)> {
    let mut out = Vec::new();
    let lists: [(&[f64], &[f64]); 4] = [
        (&[0.0, 0.25, 0.5, 0.75, 1.0], &[10.0, 35.0, 50.0, 40.0, 5.0]),
        (&[0.0, 0.5, 1.0], &[-30.0, 20.0, 45.0]),
        (&[0.0, 0.125, 0.3, 0.5, 0.8, 1.0], &[0.0, 12.0, 33.0, 41.0, 7.0, -9.0]),
        (&[0.0, 1.0], &[80.0, -80.0]),
    ];
    for (n, (rel, m3)) in lists.iter().enumerate() {
        let s = at(1.0, rel, m3, 0.0);
        let mut v = vec![compute_cb_from_stations(&s)];
        for ratio in [0.5, 0.3, 0.25, 0.7, 0.01, 1.0] {
            v.push(compute_cb_for_unbraced_ratio(&s, ratio));
        }
        out.push((["cb 5", "cb 3", "cb 6", "cb 2"][n], v));
    }
    // A steel member with the unbraced ratio 0.4 overwritten, and the plain one.
    for ratio in [None, Some(0.4)] {
        let mut m = steel_member(480.0, 0.0, 0.0);
        m.frames.get_mut("B1").unwrap().steel_overwrite = Some(SteelDesignOverwrite {
            l_ratio_ltb: ratio,
            ..Default::default()
        });
        let x = [0.0, 120.0, 240.0, 360.0, 480.0];
        put(&mut m, at(480.0, &x, &[100.0, 1500.0, 2000.0, 900.0, 0.0], 1.0));
        try_run_steel_design(&mut m).unwrap();
        out.push(("steel", vec![m.steel_design_results["B1"].total_ratio]));
    }
    // Concrete beams of the three framing types.
    for (name, framing) in [
        ("beam special", ConcreteFramingType::SwaySpecial),
        ("beam intermediate", ConcreteFramingType::SwayIntermediate),
        ("beam ordinary", ConcreteFramingType::SwayOrdinary),
    ] {
        let mut m = beam(240.0, 0.0, 0.0, framing);
        let x = [0.0, 60.0, 120.0, 180.0, 240.0];
        put(&mut m, at(240.0, &x, &[-1000.0, 200.0, 500.0, 100.0, -3000.0], 2.0));
        out.push((name, beam_numbers(&mut m).to_vec()));
    }
    out
}

#[test]
fn members_without_offsets_are_bitwise_unchanged() {
    let got = no_offset_numbers();
    let pinned: Vec<(&str, Vec<u64>)> = PINNED.iter().map(|(n, b)| (*n, b.to_vec())).collect();
    assert_eq!(got.len(), pinned.len());
    for ((name, v), (pname, b)) in got.iter().zip(&pinned) {
        assert_eq!(name, pname);
        assert_eq!(&bits(v), b, "{name}");
    }
}

/// The design length of a member with offsets is the length between the support faces, the
/// same span as the first to the last output station (`L - ioff - joff`): 510 - 10 - 20.
#[test]
fn design_length_is_the_span_between_the_faces() {
    let m = steel_member(510.0, 10.0, 20.0);
    let l = sap_design::frame_clear_length(&m, &m.frames["B1"]);
    assert!((l - 480.0).abs() < 1e-9, "{l}");
}

/// Bit patterns printed on the baseline (`W5_S4.patch`) before the change.
const PINNED: &[(&str, &[u64])] = &[
    ("cb 5", &[0x3ff22e8ba2e8ba2f, 0x3ff4834834834835, 0x3ff10572620ae4c4, 0x3ff1642c8590b216, 0x3ff34951b66f16f5, 0x3ff00d2637100693, 0x3ff22e8ba2e8ba2f]),
    ("cb 3", &[0x3ffd82192e29f79b, 0x3ff4924924924925, 0x3ff0be82fa0be830, 0x3ff2000000000000, 0x3ff2762762762762, 0x3ff0124924924925, 0x3ffd82192e29f79b]),
    ("cb 6", &[0x3ff5456545654565, 0x3ff72194668b5faf, 0x3ff10c3dbcf090c4, 0x3ff1bacf914c1bad, 0x3ff3f38ae6765873, 0x3ff0100c0906c514, 0x3ff5456545654565]),
    ("cb 2", &[0x40022e8ba2e8ba2f, 0x3ffaaaaaaaaaaaab, 0x3ff1642c8590b216, 0x3ff4000000000000, 0x3ff50d79435e50d8, 0x3ff0210842108421, 0x40022e8ba2e8ba2f]),
    ("steel", &[0x3ffea8fe71f773f1]),
    ("steel", &[0x3fe1de0c82824071]),
    ("beam special", &[0x400bd6cf58437ec6, 0x3ffa80ecd0d3f42a, 0, 0]),
    ("beam intermediate", &[0x400bd6cf58437ec6, 0x3ff2aaaaaaaaaaab, 0, 0]),
    ("beam ordinary", &[0x400bd6cf58437ec6, 0x3fe6e26732223d86, 0, 0]),
];
