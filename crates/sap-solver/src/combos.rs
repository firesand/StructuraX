//! Load-combination evaluation (moved out of `lib.rs`).

use indexmap::IndexMap;
use sap_model::results::{combo_step, is_range_case, is_single_valued};
use sap_model::{
    CaseResults, ComboType, FrameStationResult, LoadCombo, SapModel, SolidStress,
};

use crate::solid::solid_stress_from_components;

/// Evaluates all `LoadCombo`s in `model.load_combos` in topological order and stores
/// combined `CaseResults` in `model.case_results`.
///
/// A combination is evaluated only when every contributor has results (D78). If one
/// has none (a case with `RunCase=No`, a name that is no case or combination, a
/// combination that is itself unavailable or circular, no contributor at all), the
/// combination gets no result under any name and is listed with the reason in
/// `model.unavailable_results`; the contributors that exist are never combined
/// without it, and a zero scale factor does not excuse a missing contributor.
pub(crate) fn evaluate_load_combos(model: &mut SapModel) {
    let n_combos = model.load_combos.len();
    if n_combos == 0 {
        return;
    }
    let combo_idx_map: IndexMap<String, usize> = model
        .load_combos
        .keys()
        .enumerate()
        .map(|(idx, name)| (name.clone(), idx))
        .collect();
    let mut in_degree = vec![0usize; n_combos];
    let mut dependents: Vec<Vec<usize>> = vec![Vec::new(); n_combos];
    for (idx, combo) in model.load_combos.values().enumerate() {
        let mut seen_deps = std::collections::BTreeSet::new();
        for (src_name, _) in &combo.cases {
            if model.case_results.contains_key(src_name) {
                continue;
            }
            if let Some(&dep_idx) = combo_idx_map.get(src_name) {
                if seen_deps.insert(dep_idx) {
                    in_degree[idx] += 1;
                    if dep_idx != idx {
                        dependents[dep_idx].push(idx);
                    }
                }
            }
        }
    }

    let mut ready = std::collections::BinaryHeap::new();
    for (idx, &deg) in in_degree.iter().enumerate() {
        if deg == 0 {
            ready.push(std::cmp::Reverse(idx));
        }
    }
    let mut topo_order = Vec::with_capacity(n_combos);
    while let Some(std::cmp::Reverse(idx)) = ready.pop() {
        topo_order.push(idx);
        for &next in &dependents[idx] {
            in_degree[next] -= 1;
            if in_degree[next] == 0 {
                ready.push(std::cmp::Reverse(next));
            }
        }
    }

    let evaluated: std::collections::BTreeSet<usize> = topo_order.iter().copied().collect();
    for combo_idx in topo_order {
        let combo = model.load_combos[combo_idx].clone();
        if let Some(reason) = sap_model::results::combo_gap(model, &combo) {
            model.unavailable_results.insert(combo.name.clone(), reason);
            continue;
        }
        let Some(mut maximum) = combine_pass(model, &combo, Sense::Max) else {
            model
                .unavailable_results
                .insert(combo.name.clone(), "it has no contributing results".to_string());
            continue;
        };
        if is_single_valued(model, &combo.name) {
            // One value per quantity: maximum and minimum coincide, nothing is stored
            // twice, and the sum is the plain linear one (Analysis Reference p. 352).
            rederive_single_valued(&mut maximum);
            model.case_results.insert(combo.name.clone(), maximum);
            continue;
        }
        let mut minimum =
            combine_pass(model, &combo, Sense::Min).unwrap_or_else(|| maximum.clone());
        // Principal and von Mises values are not combined (p. 213: they exist for
        // single-valued results only): zero in both rows, and every reader asks
        // `is_single_valued` before showing one.
        clear_derived(&mut maximum);
        clear_derived(&mut minimum);
        minimum.case_name = combo.name.clone();
        maximum.combo_min = Some(Box::new(minimum));
        model.case_results.insert(combo.name.clone(), maximum);
    }
    // Circular combinations, and those that depend on one, were not ordered.
    for (idx, combo) in model.load_combos.clone().values().enumerate() {
        if !evaluated.contains(&idx) {
            let reason = sap_model::results::combo_gap(model, combo)
                .unwrap_or_else(|| "its contributors form a circular dependency".to_string());
            model.unavailable_results.insert(combo.name.clone(), reason);
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Sense {
    Max,
    Min,
}

/// The entry of `key` in the minimum result of a contributor, or the value of
/// the contributor itself when it has none.
fn lo_of<'a, T>(map: Option<&'a IndexMap<String, T>>, key: &str, hi: &'a T) -> &'a T {
    map.and_then(|m| m.get(key)).unwrap_or(hi)
}

/// One accumulation step for one sense (see [`sap_model::results::combo_step`]).
#[allow(clippy::too_many_arguments)]
fn step(
    combo_type: ComboType,
    sense: Sense,
    acc: f64,
    hi: f64,
    lo: f64,
    range: bool,
    sf: f64,
    is_first: bool,
) -> f64 {
    combo_step(combo_type, sense == Sense::Max, acc, (hi, lo), range, sf, is_first)
}

/// Combines the contributors of `combo` into the maximum or the minimum result.
/// `None` when no contributor has results.
fn combine_pass(model: &SapModel, combo: &LoadCombo, sense: Sense) -> Option<CaseResults> {
    let mut comb_res = CaseResults {
        case_name: combo.name.clone(),
        ..Default::default()
    };
    let ct = combo.combo_type;
    let finalize_op = |acc: f64| -> f64 {
        let r = acc.max(0.0).sqrt();
        if sense == Sense::Min {
            -r
        } else {
            r
        }
    };

    // A combination reports an area, solid or link result for an object only if
    // every contributing case reports one; otherwise the sum would silently be
    // that of a subset of the cases (a response-spectrum, modal or buckling case,
    // for example, carries no area, solid or link results).
    let contributing: Vec<&CaseResults> = combo
        .cases
        .iter()
        .filter_map(|(name, _)| model.case_results.get(name))
        .collect();
    let in_all = |has: &dyn Fn(&CaseResults) -> bool| contributing.iter().all(|c| has(c));

    let mut contributed = 0usize;
    for (src_name, sf) in &combo.cases {
        let Some(src) = model.case_results.get(src_name) else {
            continue;
        };
        let range = is_range_case(model, src_name);
        let lo = src.combo_min.as_deref();
        let is_first = contributed == 0;
        contributed += 1;
        let st = |acc: f64, hi: f64, lo: f64, first: bool| {
            step(ct, sense, acc, hi, lo, range, *sf, first)
        };
        let st6 = |acc: &mut [f64; 6], hi: &[f64; 6], lo: &[f64; 6], first: bool| {
            for k in 0..6 {
                acc[k] = st(acc[k], hi[k], lo[k], first);
            }
        };

        for (pname, d) in &src.joint_displacements {
            let key_first = is_first || !comb_res.joint_displacements.contains_key(pname);
            let entry = comb_res
                .joint_displacements
                .entry(pname.clone())
                .or_insert([0.0; 6]);
            st6(entry, d, lo_of(lo.map(|l| &l.joint_displacements), pname, d), key_first);
        }

        for (pname, r) in &src.joint_reactions {
            let key_first = is_first || !comb_res.joint_reactions.contains_key(pname);
            let entry = comb_res
                .joint_reactions
                .entry(pname.clone())
                .or_insert([0.0; 6]);
            st6(entry, r, lo_of(lo.map(|l| &l.joint_reactions), pname, r), key_first);
        }

        {
            let lo_br = lo.map_or(&src.base_reactions, |l| &l.base_reactions);
            st6(&mut comb_res.base_reactions, &src.base_reactions, lo_br, is_first);
        }

        for (fname, stations) in &src.frame_forces {
            let key_first = is_first || !comb_res.frame_forces.contains_key(fname);
            let entry = comb_res
                .frame_forces
                .entry(fname.clone())
                .or_insert_with(|| {
                    stations
                        .iter()
                        .map(|s| FrameStationResult {
                            station: s.station,
                            rel_dist: s.rel_dist,
                            ..Default::default()
                        })
                        .collect()
                });
            let lo_stations = lo
                .and_then(|l| l.frame_forces.get(fname))
                .filter(|l| l.len() == stations.len())
                .unwrap_or(stations);
            for ((dst_s, src_s), lo_s) in entry.iter_mut().zip(stations.iter()).zip(lo_stations) {
                dst_s.p = st(dst_s.p, src_s.p, lo_s.p, key_first);
                dst_s.v2 = st(dst_s.v2, src_s.v2, lo_s.v2, key_first);
                dst_s.v3 = st(dst_s.v3, src_s.v3, lo_s.v3, key_first);
                dst_s.t = st(dst_s.t, src_s.t, lo_s.t, key_first);
                dst_s.m2 = st(dst_s.m2, src_s.m2, lo_s.m2, key_first);
                dst_s.m3 = st(dst_s.m3, src_s.m3, lo_s.m3, key_first);
                for k in 0..3 {
                    dst_s.local_disp[k] = st(
                        dst_s.local_disp[k],
                        src_s.local_disp[k],
                        lo_s.local_disp[k],
                        key_first,
                    );
                }
            }
        }

        for (aname, ar) in &src.area_results {
            if !in_all(&|c| c.area_results.contains_key(aname)) {
                continue;
            }
            let key_first = is_first || !comb_res.area_results.contains_key(aname);
            let entry = comb_res.area_results.entry(aname.clone()).or_default();
            let lr = lo_of(lo.map(|l| &l.area_results), aname, ar);
            entry.f11 = st(entry.f11, ar.f11, lr.f11, key_first);
            entry.f22 = st(entry.f22, ar.f22, lr.f22, key_first);
            entry.f12 = st(entry.f12, ar.f12, lr.f12, key_first);
            entry.m11 = st(entry.m11, ar.m11, lr.m11, key_first);
            entry.m22 = st(entry.m22, ar.m22, lr.m22, key_first);
            entry.m12 = st(entry.m12, ar.m12, lr.m12, key_first);
            entry.v13 = st(entry.v13, ar.v13, lr.v13, key_first);
            entry.v23 = st(entry.v23, ar.v23, lr.v23, key_first);
            entry.s11_top = st(entry.s11_top, ar.s11_top, lr.s11_top, key_first);
            entry.s22_top = st(entry.s22_top, ar.s22_top, lr.s22_top, key_first);
            entry.s12_top = st(entry.s12_top, ar.s12_top, lr.s12_top, key_first);
            entry.svm_top = st(entry.svm_top, ar.svm_top, lr.svm_top, key_first);
            entry.s33 = st(entry.s33, ar.s33, lr.s33, key_first);
        }

        for (sname, sr) in &src.solid_results {
            if !in_all(&|c| c.solid_results.contains_key(sname)) {
                continue;
            }
            let key_first = is_first || !comb_res.solid_results.contains_key(sname);
            let entry = comb_res.solid_results.entry(sname.clone()).or_default();
            let lr = lo_of(lo.map(|l| &l.solid_results), sname, sr);
            let stress = |dst: &mut SolidStress, hi: &SolidStress, lo: &SolidStress| {
                dst.s11 = st(dst.s11, hi.s11, lo.s11, key_first);
                dst.s22 = st(dst.s22, hi.s22, lo.s22, key_first);
                dst.s33 = st(dst.s33, hi.s33, lo.s33, key_first);
                dst.s12 = st(dst.s12, hi.s12, lo.s12, key_first);
                dst.s13 = st(dst.s13, hi.s13, lo.s13, key_first);
                dst.s23 = st(dst.s23, hi.s23, lo.s23, key_first);
                dst.s_max = st(dst.s_max, hi.s_max, lo.s_max, key_first);
                dst.s_mid = st(dst.s_mid, hi.s_mid, lo.s_mid, key_first);
                dst.s_min = st(dst.s_min, hi.s_min, lo.s_min, key_first);
                dst.von_mises = st(dst.von_mises, hi.von_mises, lo.von_mises, key_first);
            };
            stress(&mut entry.center_stress, &sr.center_stress, &lr.center_stress);
            for g in 0..8 {
                stress(&mut entry.gauss_stresses[g], &sr.gauss_stresses[g], &lr.gauss_stresses[g]);
                stress(
                    &mut entry.nodal_stresses[g],
                    &sr.nodal_stresses[g],
                    &lr.nodal_stresses[g],
                );
            }
            for k in 0..8 {
                for d in 0..3 {
                    entry.nodal_forces[k][d] = st(
                        entry.nodal_forces[k][d],
                        sr.nodal_forces[k][d],
                        lr.nodal_forces[k][d],
                        key_first,
                    );
                }
            }
        }

        for (lname, lf) in &src.link_forces {
            if !in_all(&|c| c.link_forces.contains_key(lname)) {
                continue;
            }
            let key_first = is_first || !comb_res.link_forces.contains_key(lname);
            let entry = comb_res
                .link_forces
                .entry(lname.clone())
                .or_insert([0.0; 6]);
            st6(entry, lf, lo_of(lo.map(|l| &l.link_forces), lname, lf), key_first);
        }
    }

    if contributed == 0 {
        return None;
    }

    if ct == ComboType::Srss {
        map_all(&mut comb_res, &finalize_op);
    }
    Some(comb_res)
}

/// Applies `f` to every combined number of `r`.
fn map_all(r: &mut CaseResults, f: &dyn Fn(f64) -> f64) {
    for d in r.joint_displacements.values_mut() {
        for v in d {
            *v = f(*v);
        }
    }
    for x in r.joint_reactions.values_mut() {
        for v in x {
            *v = f(*v);
        }
    }
    for v in &mut r.base_reactions {
        *v = f(*v);
    }
    for stations in r.frame_forces.values_mut() {
        for s in stations {
            s.p = f(s.p);
            s.v2 = f(s.v2);
            s.v3 = f(s.v3);
            s.t = f(s.t);
            s.m2 = f(s.m2);
            s.m3 = f(s.m3);
            for v in &mut s.local_disp {
                *v = f(*v);
            }
        }
    }
    for ar in r.area_results.values_mut() {
        ar.f11 = f(ar.f11);
        ar.f22 = f(ar.f22);
        ar.f12 = f(ar.f12);
        ar.m11 = f(ar.m11);
        ar.m22 = f(ar.m22);
        ar.m12 = f(ar.m12);
        ar.v13 = f(ar.v13);
        ar.v23 = f(ar.v23);
        ar.s11_top = f(ar.s11_top);
        ar.s22_top = f(ar.s22_top);
        ar.s12_top = f(ar.s12_top);
        ar.svm_top = f(ar.svm_top);
        ar.s33 = f(ar.s33);
    }
    let fs = |s: &mut SolidStress| {
        s.s11 = f(s.s11);
        s.s22 = f(s.s22);
        s.s33 = f(s.s33);
        s.s12 = f(s.s12);
        s.s13 = f(s.s13);
        s.s23 = f(s.s23);
        s.s_max = f(s.s_max);
        s.s_mid = f(s.s_mid);
        s.s_min = f(s.s_min);
        s.von_mises = f(s.von_mises);
    };
    for sr in r.solid_results.values_mut() {
        fs(&mut sr.center_stress);
        for g in 0..8 {
            fs(&mut sr.gauss_stresses[g]);
            fs(&mut sr.nodal_stresses[g]);
        }
        for nf in &mut sr.nodal_forces {
            for v in nf {
                *v = f(*v);
            }
        }
    }
    for lf in r.link_forces.values_mut() {
        for v in lf {
            *v = f(*v);
        }
    }
}

/// Single-valued result: von Mises and principal values are not linear in the
/// stress components, so they are taken from the combined components.
fn rederive_single_valued(r: &mut CaseResults) {
    for ar in r.area_results.values_mut() {
        ar.svm_top = crate::area::plane_von_mises(ar.s11_top, ar.s22_top, ar.s12_top);
    }
    let rederive = |st: &mut SolidStress| {
        *st = solid_stress_from_components([st.s11, st.s22, st.s33, st.s12, st.s13, st.s23]);
    };
    for sr in r.solid_results.values_mut() {
        rederive(&mut sr.center_stress);
        for g in 0..8 {
            rederive(&mut sr.gauss_stresses[g]);
            rederive(&mut sr.nodal_stresses[g]);
        }
    }
}

/// Multi-valued result: the derived fields are not combined (D04).
fn clear_derived(r: &mut CaseResults) {
    for ar in r.area_results.values_mut() {
        ar.svm_top = 0.0;
    }
    let clear = |s: &mut SolidStress| {
        s.s_max = 0.0;
        s.s_mid = 0.0;
        s.s_min = 0.0;
        s.von_mises = 0.0;
    };
    for sr in r.solid_results.values_mut() {
        clear(&mut sr.center_stress);
        for g in 0..8 {
            clear(&mut sr.gauss_stresses[g]);
            clear(&mut sr.nodal_stresses[g]);
        }
    }
}

#[cfg(test)]
mod tests {
    //! The arithmetic of Analysis Reference pp. 351-353 on hand-set results (no
    //! solver): every expected number is the manual's own or computed in a comment.
    use super::*;
    use sap_model::{AreaResult, LoadCase, LoadCaseType};

    const SPECTRUM: LoadCaseType = LoadCaseType::ResponseSpectrum;
    const STATIC: LoadCaseType = LoadCaseType::LinearStatic;

    /// A model whose cases are `(name, type, joint U1, axial P)`; joint `J` and
    /// frame `B` (one station) carry those numbers.
    fn model(cases: &[(&str, LoadCaseType, f64, f64)]) -> SapModel {
        let mut m = SapModel::default();
        m.load_cases.clear();
        for (name, ty, u, p) in cases {
            let mut lc = LoadCase::linear_static(*name, "DEAD", 1.0);
            lc.case_type = *ty;
            m.load_cases.insert((*name).into(), lc);
            let mut cr = CaseResults {
                case_name: (*name).into(),
                ..Default::default()
            };
            cr.joint_displacements
                .insert("J".into(), [*u, 0.0, 0.0, 0.0, 0.0, 0.0]);
            cr.frame_forces.insert(
                "B".into(),
                vec![FrameStationResult {
                    p: *p,
                    ..Default::default()
                }],
            );
            m.case_results.insert((*name).into(), cr);
        }
        m
    }

    fn combo(m: &mut SapModel, name: &str, ty: ComboType, cases: &[(&str, f64)]) {
        m.load_combos.insert(
            name.into(),
            LoadCombo {
                name: name.into(),
                combo_type: ty,
                cases: cases.iter().map(|(c, f)| ((*c).to_string(), *f)).collect(),
            },
        );
    }

    /// `(maximum, minimum)` of joint U1 and of the axial force of a result.
    fn pair(m: &SapModel, name: &str) -> ((f64, f64), (f64, f64)) {
        let hi = &m.case_results[name];
        let lo = hi.combo_min.as_deref().unwrap_or(hi);
        (
            (hi.joint_displacements["J"][0], lo.joint_displacements["J"][0]),
            (hi.frame_forces["B"][0].p, lo.frame_forces["B"][0].p),
        )
    }

    fn near(got: f64, want: f64) {
        assert!((got - want).abs() <= 1e-12, "got {got}, want {want}");
    }

    /// Analysis Reference p. 353, the example of COMB1 and COMB2: a static case
    /// LL of 3.5 and a response-spectrum case QUAKE of 2.0 (stored positive).
    #[test]
    fn manual_p353_comb1_and_comb2() {
        let mut m = model(&[("LL", STATIC, 3.5, 0.0), ("QUAKE", SPECTRUM, 2.0, 0.0)]);
        combo(&mut m, "COMB1", ComboType::LinearAdd, &[("LL", 1.0), ("QUAKE", 1.0)]);
        combo(&mut m, "COMB2", ComboType::Envelope, &[("LL", 1.0), ("QUAKE", 1.0)]);
        evaluate_load_combos(&mut m);
        // COMB1: 3.5 + 2.0 = 5.5 and 3.5 - 2.0 = 1.5.
        let ((mx, mn), _) = pair(&m, "COMB1");
        near(mx, 5.5);
        near(mn, 1.5);
        // COMB2: max(3.5, 2.0) = 3.5 and min(3.5, -2.0) = -2.0.
        let ((mx, mn), _) = pair(&m, "COMB2");
        near(mx, 3.5);
        near(mn, -2.0);
    }

    /// Analysis Reference p. 353: axial forces GRAV 10, WINDX 5, WINDY 3 (static)
    /// and EQ 7 (spectrum); WIND = SRSS(WINDX, WINDY) = +-5.830951895 (sqrt 34);
    /// GRAVEQ = GRAV + EQ = 17 / 3; GRAVWIN = GRAV + WIND = 15.830951895 /
    /// 4.169048105; SEVERE = envelope of the two = 17 / 3.
    #[test]
    fn manual_p353_axial_forces() {
        let mut m = model(&[
            ("GRAV", STATIC, 0.0, 10.0),
            ("WINDX", STATIC, 0.0, 5.0),
            ("WINDY", STATIC, 0.0, 3.0),
            ("EQ", SPECTRUM, 0.0, 7.0),
        ]);
        combo(&mut m, "WIND", ComboType::Srss, &[("WINDX", 1.0), ("WINDY", 1.0)]);
        combo(&mut m, "GRAVEQ", ComboType::LinearAdd, &[("GRAV", 1.0), ("EQ", 1.0)]);
        combo(&mut m, "GRAVWIN", ComboType::LinearAdd, &[("GRAV", 1.0), ("WIND", 1.0)]);
        combo(&mut m, "SEVERE", ComboType::Envelope, &[("GRAVEQ", 1.0), ("GRAVWIN", 1.0)]);
        evaluate_load_combos(&mut m);
        let w = 5.830951895; // the manual's value, 7 digits kept by the tolerance below
        for (name, hi, lo) in [
            ("WIND", w, -w),
            ("GRAVEQ", 17.0, 3.0),
            ("GRAVWIN", 10.0 + w, 10.0 - w),
            ("SEVERE", 17.0, 3.0),
        ] {
            let (_, (mx, mn)) = pair(&m, name);
            // sqrt(34) = 5.8309518948...: the printed digits bound the comparison.
            assert!((mx - hi).abs() < 1e-8 && (mn - lo).abs() < 1e-8, "{name}: {mx} {mn}");
        }
        // And exactly: sqrt(5^2 + 3^2).
        near(pair(&m, "WIND").1 .0, 34.0f64.sqrt());
    }

    /// Range (p. 352): "the sum of the positive maximum values" and "the sum of
    /// the negative minimum values" of the scaled contributions.
    #[test]
    fn range_combination() {
        let mut m = model(&[("A", STATIC, 3.0, 0.0), ("B", STATIC, -2.0, 0.0), ("C", STATIC, 1.5, 0.0)]);
        combo(&mut m, "R", ComboType::RangeAdd, &[("A", 1.0), ("B", 1.0), ("C", 1.0)]);
        combo(&mut m, "RN", ComboType::RangeAdd, &[("A", -1.0), ("B", 1.0), ("C", 1.0)]);
        evaluate_load_combos(&mut m);
        // R: maximum 3 + 1.5 = 4.5, minimum -2.
        let ((mx, mn), _) = pair(&m, "R");
        near(mx, 4.5);
        near(mn, -2.0);
        // RN: A scaled by -1 is -3: maximum 1.5, minimum -3 - 2 = -5.
        let ((mx, mn), _) = pair(&m, "RN");
        near(mx, 1.5);
        near(mn, -5.0);
        assert!(!sap_model::results::is_single_valued(&m, "R"));
    }

    /// Absolute: maximum is the sum of the larger absolute values, minimum its
    /// negative (p. 352).
    #[test]
    fn absolute_combination() {
        let mut m = model(&[("A", STATIC, 3.0, 0.0), ("B", STATIC, -2.0, 0.0), ("RS", SPECTRUM, 4.0, 0.0)]);
        combo(&mut m, "ABS", ComboType::AbsAdd, &[("A", 1.0), ("B", 2.0), ("RS", -0.5)]);
        evaluate_load_combos(&mut m);
        // |3| + |-2 * 2| + |4 * 0.5| = 3 + 4 + 2 = 9.
        let ((mx, mn), _) = pair(&m, "ABS");
        near(mx, 9.0);
        near(mn, -9.0);
    }

    /// An additive combination of a static case with minus a spectrum case:
    /// D + (-1) RS has the maximum D + RS and the minimum D - RS (the pair of the
    /// spectrum case is (+v, -v), and a negative factor swaps it).
    #[test]
    fn additive_with_negative_factor_on_a_spectrum_case() {
        let mut m = model(&[("D", STATIC, 3.0, 0.0), ("RS", SPECTRUM, 2.0, 0.0)]);
        combo(&mut m, "C", ComboType::LinearAdd, &[("D", 1.0), ("RS", -1.0)]);
        evaluate_load_combos(&mut m);
        let ((mx, mn), _) = pair(&m, "C");
        near(mx, 5.0);
        near(mn, 1.0);
    }

    /// A combination of single-valued cases stays single valued: no minimum, and
    /// the values are those of the plain loop `acc += v * sf` bit for bit.
    #[test]
    fn single_valued_additive_is_unchanged() {
        let vals = [(0.1f64, 1.2f64), (-0.7, 1.6), (3.3e-3, -0.9)];
        let mut m = model(&[
            ("A", STATIC, vals[0].0, 1.0),
            ("B", STATIC, vals[1].0, 2.0),
            ("C", STATIC, vals[2].0, 3.0),
        ]);
        combo(
            &mut m,
            "S",
            ComboType::LinearAdd,
            &[("A", vals[0].1), ("B", vals[1].1), ("C", vals[2].1)],
        );
        evaluate_load_combos(&mut m);
        let mut acc = 0.0f64;
        for (v, sf) in vals {
            acc += v * sf;
        }
        let cr = &m.case_results["S"];
        assert!(cr.combo_min.is_none());
        assert_eq!(cr.joint_displacements["J"][0].to_bits(), acc.to_bits());
    }

    /// D78 (replaces the old `missing_contributors_are_skipped`, which expected the
    /// envelope of the contributors that exist, -2.0): a combination with a contributor
    /// that has no results is unavailable, with the reason, under any name.
    #[test]
    fn missing_contributors_make_the_combination_unavailable() {
        let mut m = model(&[("A", STATIC, -1.0, 0.0)]);
        combo(&mut m, "E", ComboType::Envelope, &[("MISSING", 1.0), ("A", 2.0)]);
        combo(&mut m, "NONE", ComboType::Envelope, &[("MISSING", 1.0)]);
        combo(&mut m, "OK", ComboType::Envelope, &[("A", 2.0)]);
        evaluate_load_combos(&mut m);
        assert!(!m.case_results.contains_key("E"));
        assert!(!m.case_results.contains_key("NONE"));
        assert!(m.unavailable_results["E"].contains("'MISSING'"));
        assert!(m.unavailable_results["NONE"].contains("'MISSING'"));
        let ((mx, mn), _) = pair(&m, "OK");
        near(mx, -2.0);
        near(mn, -2.0);
    }

    /// Derived fields of a multi-valued result are not combined: 0 in both rows.
    /// A single-valued sum re-derives von Mises from the combined components.
    #[test]
    fn derived_fields_are_zero_for_multi_valued_results() {
        let mut m = model(&[("A", STATIC, 0.0, 0.0), ("B", STATIC, 0.0, 0.0)]);
        for (name, s11) in [("A", 100.0), ("B", -60.0)] {
            m.case_results[name].area_results.insert(
                "S".into(),
                AreaResult {
                    s11_top: s11,
                    svm_top: s11.abs(),
                    ..Default::default()
                },
            );
        }
        combo(&mut m, "ENV", ComboType::Envelope, &[("A", 1.0), ("B", 1.0)]);
        combo(&mut m, "SUM", ComboType::LinearAdd, &[("A", 1.0), ("B", 1.0)]);
        evaluate_load_combos(&mut m);
        let env = &m.case_results["ENV"];
        let lo = env.combo_min.as_ref().unwrap();
        near(env.area_results["S"].s11_top, 100.0);
        near(lo.area_results["S"].s11_top, -60.0);
        assert_eq!((env.area_results["S"].svm_top, lo.area_results["S"].svm_top), (0.0, 0.0));
        // SUM: s11 = 40, plane von Mises of (40, 0, 0) is 40.
        near(m.case_results["SUM"].area_results["S"].svm_top, 40.0);
    }
}
