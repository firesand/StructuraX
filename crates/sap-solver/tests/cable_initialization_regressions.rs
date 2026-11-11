use sap_solver::cable::{evaluate_cable_element, CableElement};

#[test]
fn horizontal_tension_initialization_stiff_taut_and_inclined_sweep() {
    // Consistent kip/in units: spans/weights below are specified in ft and
    // kip/ft, areas in in^2, E=30,000 kip/in^2. This is an independently chosen
    // grid, not the unavailable original reviewer sweep.
    let mut checked = 0;
    let mut old_threshold_exceeded = 0;
    let mut maximum_relative_error = 0.0_f64;
    let mut failures = Vec::new();
    for span_ft in [10., 30., 100., 300., 1000.] {
        for area in [0.1, 1., 10., 100.] {
            for target in [0.1, 1., 10., 100.] {
                for weight_per_ft in [0.0001, 0.001, 0.01, 0.1] {
                    // Avoid implausible enormous-sag or >2% elastic-strain
                    // configurations; include small-sag/high-EA/H cases.
                    if weight_per_ft * span_ft / target > 4. || target / (30_000. * area) > 0.02 {
                        continue;
                    }
                    for slope in [0., 0.25] {
                        checked += 1;
                        let result = CableElement::from_horizontal_tension(
                            [0.; 3],
                            [12. * span_ft, 0., slope * 12. * span_ft],
                            area,
                            30_000.,
                            0.,
                            target,
                            [0., 0., -weight_per_ft / 12.],
                            0.,
                        )
                        .and_then(|element| evaluate_cable_element(&element, [0.; 3], [0.; 3]));
                        match result {
                            Ok(state) => {
                                let relative = (state.horizontal_tension-target).abs()/target;
                                maximum_relative_error = maximum_relative_error.max(relative);
                                old_threshold_exceeded += usize::from(relative > 1e-12);
                                // Independent physical target check, not the
                                // initializer's conditioning-based criterion.
                                if relative > 1e-9 {
                                    failures.push(format!("span={span_ft} A={area} H={target} w={weight_per_ft} slope={slope}: achieved H={} relative={relative}",state.horizontal_tension));
                                }
                            }
                            Err(error) => failures.push(format!("span={span_ft} A={area} H={target} w={weight_per_ft} slope={slope}: {error}")),
                        }
                    }
                }
            }
        }
    }
    println!("checked={checked}; formerly-too-strict threshold exceeded={old_threshold_exceeded}; max relative physical H error={maximum_relative_error:e}");
    assert!(
        failures.is_empty(),
        "{} failures:\n{}",
        failures.len(),
        failures.join("\n")
    );
    assert!(checked > 400);
}

#[test]
fn shape_acceptance_and_physical_tension_are_invariant_under_unit_scaling() {
    for length_unit in [0.001_f64, 1. / 12., 1., 1000.] {
        for force_unit in [0.001_f64, 1., 1000.] {
            let target = force_unit;
            let element = CableElement::from_horizontal_tension(
                [0.; 3],
                [120. * length_unit, 0., 30. * length_unit],
                10. * length_unit.powi(2),
                30_000. * force_unit / length_unit.powi(2),
                0.,
                target,
                [0., 0., -(0.01 / 12.) * force_unit / length_unit],
                0.,
            )
            .unwrap();
            let state = evaluate_cable_element(&element, [0.; 3], [0.; 3]).unwrap();
            assert!(
                (state.horizontal_tension / target - 1.).abs() < 1e-9,
                "length scale={length_unit}, force scale={force_unit}, H={}",
                state.horizontal_tension
            );
        }
    }
}

#[test]
fn unloaded_slack_zero_force_scale_is_exact_equilibrium_not_nonconvergence() {
    use sap_solver::cable::solve_cable_network;
    let element = CableElement::from_undeformed_length(
        [0.; 3],
        [10., 0., 0.],
        1.,
        1000.,
        0.,
        12.,
        [0.; 3],
        0.,
    )
    .unwrap();
    let elements = [(0, 1, element)];
    let fixed = [[true; 3], [false, true, true]];
    let loads = [[0.; 3]; 2];
    let (u, state) = solve_cable_network(2, &elements, &fixed, &loads, None, 1, 1e-12).unwrap();
    assert_eq!(u, vec![[0.; 3]; 2]);
    assert_eq!(state[0].f_j, [0.; 3]);
    // Zero applied force and zero internal force satisfy R=0 exactly. No
    // arbitrary force floor is needed, and invalid relative tolerances reject.
    for tolerance in [0., 1., 2., f64::NAN, f64::INFINITY] {
        assert!(solve_cable_network(2, &elements, &fixed, &loads, None, 1, tolerance).is_err());
    }
}
