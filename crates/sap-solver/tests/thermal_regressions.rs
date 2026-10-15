use sap_model::*;
use sap_solver::analyze_model;

const LENGTH: f64 = 12.;
const ALPHA: f64 = 1.2e-5;
const E: f64 = 200000.;
fn close(a: f64, b: f64) {
    assert!((a - b).abs() < 2e-9 * (1. + b.abs()), "{a} != {b}");
}
fn thermal(kind: i32, start: f64, end: f64, fixed: bool, shear: bool) -> SapModel {
    let mut m = SapModel::default();
    m.load_patterns["DEAD"].self_wt_multiplier = 0.;
    m.load_cases.clear();
    m.load_cases
        .insert("T".into(), LoadCase::linear_static("T", "DEAD", 1.));
    m.add_point("I", 0., 0., 0.);
    m.add_point("J", LENGTH, 0., 0.);
    m.points["I"].restraint = [true; 6];
    m.points["J"].restraint = [fixed; 6];
    m.points["I"].pattern_values.insert("PROFILE".into(), start);
    m.points["J"].pattern_values.insert("PROFILE".into(), end);
    m.add_frame("F", "I", "J", "FSEC1");
    let sec = &mut m.frame_sections["FSEC1"];
    sec.area = 3.;
    sec.i22 = 2.;
    sec.i33 = 4.;
    sec.torsional_j = 1.;
    sec.as2 = if shear { 0.4 } else { 0. };
    sec.as3 = if shear { 0.8 } else { 0. };
    let mat = &mut m.materials[&sec.material];
    mat.e = E;
    mat.g = 80000.;
    mat.a = ALPHA;
    mat.unit_weight = 0.;
    mat.unit_mass = 0.;
    m.frames["F"].loads_temp.push(FrameLoadTemperature {
        load_pat: "DEAD".into(),
        my_type: kind,
        val: 1.,
        pattern_name: Some("PROFILE".into()),
    });
    m
}

#[test]
fn free_thermal_axial_strain_matches_linear_temperature_field_at_every_station() {
    for (start, end) in [(20., 20.), (10., 40.), (-20., 10.)] {
        let mut m = thermal(1, start, end, false, false);
        analyze_model(&mut m).unwrap();
        for station in &m.case_results["T"].frame_forces["F"] {
            let x = station.station;
            close(
                station.local_disp[0],
                ALPHA * (start * x + (end - start) * x * x / (2. * LENGTH)),
            );
            close(station.p, 0.);
        }
    }
}

#[test]
fn thermal_gradients_produce_stress_free_curvature_in_both_planes_with_or_without_shear() {
    for kind in [2, 3] {
        for shear in [false, true] {
            for (start, end) in [(2., 2.), (2., 5.), (-3., 4.)] {
                let mut m = thermal(kind, start, end, false, shear);
                analyze_model(&mut m).unwrap();
                let r = &m.case_results["T"];
                for station in &r.frame_forces["F"] {
                    let x = station.station;
                    close(
                        station.local_disp[(kind - 1) as usize],
                        -ALPHA * (start * x * x / 2. + (end - start) * x * x * x / (6. * LENGTH)),
                    );
                    for f in [
                        station.p, station.v2, station.v3, station.t, station.m2, station.m3,
                    ] {
                        close(f, 0.);
                    }
                }
                let rotation = ALPHA * LENGTH * (start + end) / 2.;
                // local 2=global Z, local 3=-global Y for the X-aligned member.
                let global_rotation = if kind == 2 { 4 } else { 5 };
                close(r.joint_displacements["J"][global_rotation], rotation);
            }
        }
    }
}

#[test]
fn fixed_thermal_gradient_reactions_satisfy_equilibrium_and_zero_euler_deflection() {
    for kind in [2, 3] {
        for (start, end) in [(2., 2.), (2., 5.)] {
            let mut m = thermal(kind, start, end, true, false);
            analyze_model(&mut m).unwrap();
            let r = &m.case_results["T"];
            let inertia = if kind == 2 { 4. } else { 2. };
            for station in &r.frame_forces["F"] {
                let gradient = start + (end - start) * station.rel_dist;
                let bending = if kind == 2 { station.m3 } else { -station.m2 };
                let shear = if kind == 2 { station.v2 } else { station.v3 };
                close(bending, E * inertia * ALPHA * gradient);
                close(shear, -E * inertia * ALPHA * (end - start) / LENGTH);
                close(station.local_disp[(kind - 1) as usize], 0.);
            }
            for component in r.base_reactions {
                close(component, 0.);
            }
        }
    }
}

#[test]
fn fixed_axial_temperature_field_has_constant_force_but_nonzero_internal_expansion() {
    let (start, end) = (10., 40.);
    let mut m = thermal(1, start, end, true, false);
    analyze_model(&mut m).unwrap();
    for station in &m.case_results["T"].frame_forces["F"] {
        let x = station.station;
        close(station.p, -E * 3. * ALPHA * (start + end) / 2.);
        close(
            station.local_disp[0],
            ALPHA * (end - start) / 2. * (x * x / LENGTH - x),
        );
    }
}

#[test]
fn fixed_linear_gradient_with_shear_matches_closed_form_compatible_curvature() {
    let (start, end) = (2., 5.);
    for kind in [2, 3] {
        let mut m = thermal(kind, start, end, true, true);
        analyze_model(&mut m).unwrap();
        let (inertia, area) = if kind == 2 { (4., 0.4) } else { (2., 0.8) };
        let ei = E * inertia;
        let ga = 80000. * area;
        let slope = ei * ALPHA * (end - start) / LENGTH / (1. + 12. * ei / (ga * LENGTH * LENGTH));
        let curvature_slope = slope / ei - ALPHA * (end - start) / LENGTH;
        for station in &m.case_results["T"].frame_forces["F"] {
            let x = station.station;
            let exact = curvature_slope * (x * x * x / 6. - LENGTH * x * x / 4.) - slope * x / ga;
            close(station.local_disp[(kind - 1) as usize], exact);
            close(if kind == 2 { station.v2 } else { station.v3 }, -slope);
            close(
                if kind == 2 { station.m3 } else { -station.m2 },
                ei * ALPHA * (start + end) / 2. + slope * (x - LENGTH / 2.),
            );
        }
    }
}

#[test]
fn stepped_area_fixed_thermal_expansion_uses_series_flexibility_and_piecewise_strain() {
    let mut m = thermal(1, 20., 20., true, false);
    let mut small = m.frame_sections["FSEC1"].clone();
    small.name = "SMALL".into();
    small.area = 2.;
    let mut large = small.clone();
    large.name = "LARGE".into();
    large.area = 5.;
    m.frame_sections.insert(small.name.clone(), small);
    m.frame_sections.insert(large.name.clone(), large);
    m.frame_sections["FSEC1"].prop_type = FramePropType::Variable;
    m.frame_sections["FSEC1"].non_prismatic = vec![("SMALL", 3.), ("LARGE", 9.)]
        .into_iter()
        .map(|(name, length)| NonPrismaticSegment {
            start_sec: name.into(),
            end_sec: name.into(),
            length,
            is_variable_length: false,
            ei22_variation: 1,
            ei33_variation: 1,
        })
        .collect();
    analyze_model(&mut m).unwrap();
    let axial = -ALPHA * 20. * LENGTH / (3. / (E * 2.) + 9. / (E * 5.));
    for station in &m.case_results["T"].frame_forces["F"] {
        let x = station.station;
        close(station.p, axial);
        close(
            station.local_disp[0],
            ALPHA * 20. * x + axial * (x.min(3.) / (E * 2.) + (x - 3.).max(0.) / (E * 5.)),
        );
    }
}

#[test]
fn tapered_selfweight_station_moments_preserve_end_equilibrium_and_exact_first_moment() {
    let mut m = thermal(1, 0., 0., false, false);
    m.load_patterns["DEAD"].self_wt_multiplier = 1.;
    let material = m.frame_sections["FSEC1"].material.clone();
    m.materials[&material].unit_weight = 1.;
    let mut small = m.frame_sections["FSEC1"].clone();
    small.name = "SMALL".into();
    small.area = 1.;
    let mut large = small.clone();
    large.name = "LARGE".into();
    large.area = 3.;
    m.frame_sections.insert(small.name.clone(), small);
    m.frame_sections.insert(large.name.clone(), large);
    m.frame_sections["FSEC1"].prop_type = FramePropType::Variable;
    m.frame_sections["FSEC1"].non_prismatic = vec![NonPrismaticSegment {
        start_sec: "SMALL".into(),
        end_sec: "LARGE".into(),
        length: 1.,
        is_variable_length: true,
        ei22_variation: 1,
        ei33_variation: 1,
    }];
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["T"];
    close(r.base_reactions[2], 2. * LENGTH);
    close(r.base_reactions[4], -7. * LENGTH * LENGTH / 6.);
    for station in &r.frame_forces["F"] {
        let x = station.station;
        let shear = -(2. * LENGTH - x - x * x / LENGTH);
        let moment =
            -(7. * LENGTH * LENGTH / 6. - 2. * LENGTH * x + x * x / 2. + x * x * x / (3. * LENGTH));
        close(station.v2, shear);
        close(station.m3, moment);
    }
    // The review's public recovery reproducer also requires exact J endpoint
    // consistency for a supplied end-force vector.
    let f = &m.frames["F"];
    let sec = &m.frame_sections["FSEC1"];
    let mat = &m.materials[&material];
    let (_, axes) = sap_solver::frame::frame_local_axes([0.; 3], [LENGTH, 0., 0.], 0.);
    let out = sap_solver::frame::recover_frame_stations(
        &m,
        f,
        sec,
        mat,
        &axes,
        LENGTH,
        &[0.; 12],
        &[0.; 12],
        &[("DEAD".into(), 1.)],
    );
    close(out.last().unwrap().m3, 0.);
}
