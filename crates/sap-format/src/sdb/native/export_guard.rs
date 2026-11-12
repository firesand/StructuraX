//! Conservative semantic boundary for experimental native stream synthesis.
use crate::FormatError;
use sap_model::SapModel;
use serde_json::Value;

pub(super) fn unsupported(field: impl Into<String>, reason: impl Into<String>) -> FormatError {
    FormatError::UnsupportedNativeExport {
        field: field.into(),
        reason: reason.into(),
    }
}
fn reference(exists: bool, field: String, target: &str) -> Result<(), FormatError> {
    if exists {
        Ok(())
    } else {
        Err(unsupported(field, format!("missing referenced name '{target}'; native export never substitutes index 1 or drops a load")))
    }
}
fn csys(model: &SapModel, name: &str, field: String) -> Result<(), FormatError> {
    reference(
        name.eq_ignore_ascii_case("GLOBAL")
            || name.eq_ignore_ascii_case("LOCAL")
            || model.coord_systems.contains_key(name),
        field,
        name,
    )
}
fn pattern(model: &SapModel, name: &str, field: String) -> Result<(), FormatError> {
    reference(model.load_patterns.contains_key(name), field, name)
}
fn acceleration(name: &str) -> bool {
    matches!(
        name,
        "U1" | "U2" | "U3" | "U4" | "U5" | "U6" | "u1" | "u2" | "u3" | "u4" | "u5" | "u6"
    )
}

pub(super) fn validate(model: &SapModel) -> Result<(), FormatError> {
    if let Some(saved) = &model.s2k_preservation {
        let issues = crate::s2k::preservation::native_projection_issues(saved)?;
        if !issues.is_empty() {
            return Err(unsupported("s2k_preservation", format!("native synthesis cannot carry unrepresented S2K source data: {}; retain the source or save a private snapshot", issues.join("; "))));
        }
    }
    // The native index maps are built from record names, while references use map
    // keys. Inconsistent names must not become duplicate or substituted indices.
    macro_rules! identities {
        ($($collection:ident),+ $(,)?) => {$({
            if model.$collection.len() > i32::MAX as usize { return Err(unsupported(stringify!($collection),"record count exceeds native i32 indexing")); }
            for (key, value) in &model.$collection {
                if key.is_empty() || key != &value.name { return Err(unsupported(format!("{}.{}",stringify!($collection),key), "map key must be a nonempty exact match for record name")); }
            }
        })+};
    }
    identities!(
        coord_systems,
        materials,
        frame_sections,
        area_sections,
        solid_sections,
        link_props,
        constraints,
        points,
        frames,
        areas,
        solids,
        links,
        functions,
        load_patterns,
        load_cases,
        load_combos
    );
    reference(
        model.coord_systems.contains_key("GLOBAL"),
        "coord_systems.GLOBAL".into(),
        "GLOBAL",
    )?;
    for (name, section) in &model.frame_sections {
        reference(
            model.materials.contains_key(&section.material),
            format!("frame_sections.{name}.material"),
            &section.material,
        )?;
        for seg in &section.non_prismatic {
            for target in [&seg.start_sec, &seg.end_sec] {
                reference(
                    model.frame_sections.contains_key(target),
                    format!("frame_sections.{name}.non_prismatic"),
                    target,
                )?;
            }
        }
        for target in &section.auto_select_list {
            reference(
                model.frame_sections.contains_key(target),
                format!("frame_sections.{name}.auto_select_list"),
                target,
            )?;
        }
        if let Some(rebar) = &section.concrete_rebar {
            for target in [&rebar.rebar_mat_long, &rebar.rebar_mat_confine] {
                reference(
                    model.materials.contains_key(target),
                    format!("frame_sections.{name}.concrete_rebar"),
                    target,
                )?;
            }
        }
    }
    for (name, s) in &model.area_sections {
        reference(
            model.materials.contains_key(&s.material),
            format!("area_sections.{name}.material"),
            &s.material,
        )?;
    }
    for (name, s) in &model.solid_sections {
        reference(
            model.materials.contains_key(&s.material),
            format!("solid_sections.{name}.material"),
            &s.material,
        )?;
    }
    for (name, p) in &model.points {
        for target in &p.constraints {
            reference(
                model.constraints.contains_key(target),
                format!("points.{name}.constraints"),
                target,
            )?;
        }
        for l in &p.loads_force {
            pattern(
                model,
                &l.load_pat,
                format!("points.{name}.loads_force.load_pat"),
            )?;
            csys(model, &l.csys, format!("points.{name}.loads_force.csys"))?;
        }
        for l in &p.loads_displ {
            pattern(
                model,
                &l.load_pat,
                format!("points.{name}.loads_displ.load_pat"),
            )?;
            csys(model, &l.csys, format!("points.{name}.loads_displ.csys"))?;
        }
    }
    for (name, f) in &model.frames {
        for target in [&f.point_i, &f.point_j] {
            reference(
                model.points.contains_key(target),
                format!("frames.{name}.points"),
                target,
            )?;
        }
        reference(
            model.frame_sections.contains_key(&f.section),
            format!("frames.{name}.section"),
            &f.section,
        )?;
        for l in &f.loads_dist {
            pattern(
                model,
                &l.load_pat,
                format!("frames.{name}.loads_dist.load_pat"),
            )?;
            csys(model, &l.csys, format!("frames.{name}.loads_dist.csys"))?;
        }
        for l in &f.loads_point {
            pattern(
                model,
                &l.load_pat,
                format!("frames.{name}.loads_point.load_pat"),
            )?;
            csys(model, &l.csys, format!("frames.{name}.loads_point.csys"))?;
        }
        for l in &f.loads_temp {
            pattern(
                model,
                &l.load_pat,
                format!("frames.{name}.loads_temp.load_pat"),
            )?;
        }
        for pat in f.loads_gravity.keys() {
            pattern(model, pat, format!("frames.{name}.loads_gravity"))?;
        }
        if let Some(c) = &f.cable_params {
            if !(1..=i32::MAX as usize).contains(&c.num_segments) {
                return Err(unsupported(
                    format!("frames.{name}.cable_params.num_segments"),
                    "requires 1..=i32::MAX",
                ));
            }
            let specs = [
                c.undeformed_length,
                c.target_tension_i,
                c.horizontal_tension,
            ];
            if specs
                .into_iter()
                .flatten()
                .any(|x| !x.is_finite() || x <= 0.)
            {
                return Err(unsupported(
                    format!("frames.{name}.cable_params"),
                    "specified cable targets must be finite and positive",
                ));
            }
            if specs.into_iter().flatten().count() > 1 {
                return Err(unsupported(
                    format!("frames.{name}.cable_params"),
                    "native S_CableData selects at most one governing specification (undeformed_length, target_tension_i, or horizontal_tension)",
                ));
            }
        }
    }
    for (name, a) in &model.areas {
        for target in &a.points {
            reference(
                model.points.contains_key(target),
                format!("areas.{name}.points"),
                target,
            )?;
        }
        reference(
            model.area_sections.contains_key(&a.section),
            format!("areas.{name}.section"),
            &a.section,
        )?;
        for l in &a.loads_uniform {
            pattern(
                model,
                &l.load_pat,
                format!("areas.{name}.loads_uniform.load_pat"),
            )?;
            csys(model, &l.csys, format!("areas.{name}.loads_uniform.csys"))?;
        }
        for l in &a.loads_pressure {
            pattern(
                model,
                &l.load_pat,
                format!("areas.{name}.loads_pressure.load_pat"),
            )?;
        }
        for l in &a.loads_temp {
            pattern(
                model,
                &l.load_pat,
                format!("areas.{name}.loads_temp.load_pat"),
            )?;
        }
        if a.auto_mesh.0 == 0
            || a.auto_mesh.1 == 0
            || a.auto_mesh.0 > i32::MAX as usize
            || a.auto_mesh.1 > i32::MAX as usize
        {
            return Err(unsupported(
                format!("areas.{name}.auto_mesh"),
                "requires positive i32 subdivision counts",
            ));
        }
    }
    for (name, s) in &model.solids {
        for target in &s.points {
            reference(
                model.points.contains_key(target),
                format!("solids.{name}.points"),
                target,
            )?;
        }
        reference(
            model.solid_sections.contains_key(&s.section),
            format!("solids.{name}.section"),
            &s.section,
        )?;
        for l in &s.loads_surface {
            pattern(
                model,
                &l.load_pat,
                format!("solids.{name}.loads_surface.load_pat"),
            )?;
        }
        for pat in s.loads_temp.keys() {
            pattern(model, pat, format!("solids.{name}.loads_temp"))?;
        }
    }
    for (name, l) in &model.links {
        reference(
            model.points.contains_key(&l.point_i),
            format!("links.{name}.point_i"),
            &l.point_i,
        )?;
        if let Some(target) = &l.point_j {
            reference(
                model.points.contains_key(target),
                format!("links.{name}.point_j"),
                target,
            )?;
        }
        reference(
            model.link_props.contains_key(&l.prop),
            format!("links.{name}.prop"),
            &l.prop,
        )?;
    }
    for name in model.load_cases.keys() {
        if model.load_combos.contains_key(name) {
            return Err(unsupported(
                format!("load_cases.{name}"),
                "case/combo names overlap, so native references would be ambiguous",
            ));
        }
    }
    for (name, case) in &model.load_cases {
        if let Some(control) = &case.displacement_control {
            // The native record is written only for nonlinear static cases with
            // one plain joint displacement; anything else would not read back.
            let monitored_ok = control.monitored.as_ref().is_some_and(|(point, dof)| {
                model.points.contains_key(point) && *dof < 6
            });
            if case.case_type != sap_model::LoadCaseType::NonlinearStatic || !monitored_ok {
                return Err(unsupported(
                    format!("load_cases.{name}.displacement_control"),
                    "displacement control needs a nonlinear static case and one existing monitored joint displacement",
                ));
            }
        }
        if let Some(target) = &case.initial_case {
            reference(
                model.load_cases.contains_key(target),
                format!("load_cases.{name}.initial_case"),
                target,
            )?;
        }
        for l in &case.loads {
            reference(
                model.load_patterns.contains_key(&l.load_name) || acceleration(&l.load_name),
                format!("load_cases.{name}.loads"),
                &l.load_name,
            )?;
        }
        for (function, dir, _) in &case.spectrum_loads {
            reference(
                model.functions.contains_key(function),
                format!("load_cases.{name}.spectrum_loads.function"),
                function,
            )?;
            if !matches!(dir.as_str(), "U1" | "U2" | "U3" | "u1" | "u2" | "u3") {
                return Err(unsupported(
                    format!("load_cases.{name}.spectrum_loads.direction"),
                    "native export currently requires an explicit U1/U2/U3 direction",
                ));
            }
        }
        for (function, load, _) in &case.history_loads {
            reference(
                model.functions.contains_key(function),
                format!("load_cases.{name}.history_loads.function"),
                function,
            )?;
            reference(
                model.load_patterns.contains_key(load) || acceleration(load),
                format!("load_cases.{name}.history_loads.load"),
                load,
            )?;
        }
    }
    for (name, combo) in &model.load_combos {
        for (target, _) in &combo.cases {
            reference(
                model.load_cases.contains_key(target) || model.load_combos.contains_key(target),
                format!("load_combos.{name}.cases"),
                target,
            )?;
        }
    }
    for (name, p) in &model.link_props {
        if !p.nonlinear_data_complete {
            return Err(unsupported(format!("link_props.{name}.nonlinear_data_complete"),"incomplete source law data cannot be certified by synthesizing a parameter vector; use a private snapshot"));
        }
    }
    if let Some(entry) = model.unmodeled_source.first() {
        return Err(unsupported(
            "unmodeled_source",
            format!(
                "the source carried data this model does not represent ({}); a synthesized file would silently omit it",
                entry.description
            ),
        ));
    }
    if !model.project_info.is_empty() {
        return Err(unsupported(
            "project_info",
            "native project metadata serialization has not been verified",
        ));
    }
    for (name, p) in &model.points {
        if let Some(coupled) = &p.spring_coupled {
            if p.spring.iter().any(|&v| v != 0.0) {
                return Err(unsupported(
                    format!("points.{name}.spring_coupled"),
                    "native TYSpring stores a single 21-term stiffness matrix; combine uncoupled and coupled springs before export",
                ));
            }
            if coupled.iter().all(|v| v.is_finite())
                && coupled
                    .iter()
                    .enumerate()
                    .all(|(idx, &val)| crate::sdb::SPRING_DIAG_INDICES.contains(&idx) || val == 0.0)
            {
                return Err(unsupported(
                    format!("points.{name}.spring_coupled"),
                    "coupled spring matrix with zero off-diagonals canonicalizes to uncoupled spring on native reimport; store diagonal terms in points.spring",
                ));
            }
        }
        if p.local_axes_matrix.is_some() {
            return Err(unsupported(
                format!("points.{name}.local_axes_matrix"),
                "explicit joint axis-matrix serialization is not implemented",
            ));
        }
    }
    for (name, section) in &model.area_sections {
        // Only thin and thick shells have an observed native section-type code.
        if section.prop_type == sap_model::AreaPropType::Shell
            && !matches!(
                section.shell_type,
                sap_model::ShellType::ShellThin | sap_model::ShellType::ShellThick
            )
        {
            return Err(unsupported(
                format!("area_sections.{name}.shell_type"),
                "only thin and thick shell sections have an observed native type code",
            ));
        }
    }
    for (name, area) in &model.areas {
        // The native auto-mesh record is written for equal counts only; the
        // vendor edge order for unequal counts is not established.
        if area.auto_mesh.0 != area.auto_mesh.1 {
            return Err(unsupported(
                format!("areas.{name}.auto_mesh"),
                "unequal auto-mesh counts have no verified native edge order; mesh the area explicitly or use S2K",
            ));
        }
    }
    for (name, frame) in &model.frames {
        if frame.auto_mesh_segments > i32::MAX as usize {
            return Err(unsupported(
                format!("frames.{name}.auto_mesh_segments"),
                "requires an i32 segment count",
            ));
        }
        // Joint offsets are written in the one form the corpus shows: element
        // local, equal at both ends.
        if frame.joint_offset_i != [0.0; 3] || frame.joint_offset_j != [0.0; 3] {
            if frame.joint_offset_is_global {
                return Err(unsupported(
                    format!("frames.{name}.joint_offset_is_global"),
                    "global joint offsets have no verified native coordinate-system code; give them in the element local system or use S2K",
                ));
            }
            if frame.joint_offset_i != frame.joint_offset_j {
                return Err(unsupported(
                    format!("frames.{name}.joint_offset_j"),
                    "joint offsets that differ between the ends have no verified native end order; use S2K",
                ));
            }
        }
        // The segment count of a cable object is its own cable setting; what
        // an automatic frame subdivision count means on one is not established.
        if frame.cable_params.is_some() && frame.auto_mesh_segments > 1 {
            return Err(unsupported(
                format!("frames.{name}.auto_mesh_segments"),
                "automatic frame subdivision of a cable object has no verified native meaning",
            ));
        }
    }
    for (name, m) in &model.materials {
        if m.fys != m.fy {
            return Err(unsupported(
                format!("materials.{name}.fys"),
                "independent shear-rebar yield has no verified native representation; this importer uses the material Fy as a reconstruction fallback",
            ));
        }
        if m.e_ortho.is_some() || m.u_ortho.is_some() || m.g_ortho.is_some() || m.a_ortho.is_some()
        {
            let (Some(eo), Some(uo), Some(go), Some(ao)) =
                (m.e_ortho, m.u_ortho, m.g_ortho, m.a_ortho)
            else {
                return Err(unsupported(
                    format!("materials.{name}.orthotropic"),
                    "native orthotropic material requires complete [e_ortho, u_ortho, g_ortho, a_ortho] arrays",
                ));
            };
            if [eo, uo, go, ao]
                .into_iter()
                .flatten()
                .all(|v| v.is_finite())
                && [m.e, m.u, m.g, m.a].into_iter().all(|v| v.is_finite())
                && (m.e != eo[0] || m.u != uo[0] || m.g != go[0] || m.a != ao[0])
            {
                return Err(unsupported(
                    format!("materials.{name}.orthotropic"),
                    "primary scalar properties (e, u, g, a) must equal index [0] of their orthotropic arrays",
                ));
            }
        }
    }
    for (name, c) in &model.coord_systems {
        if !c.grids.is_empty() {
            return Err(unsupported(
                format!("coord_systems.{name}.grids"),
                "native grid serialization is not implemented",
            ));
        }
    }
    validate_finite_json(&semantic_value(model)?, &mut Vec::new())?;
    Ok(())
}

fn semantic_model(model: &SapModel) -> SapModel {
    let mut copy = model.clone();
    copy.is_locked = false;
    // validate() permits this only after proving the saved source has no opaque
    // tables, columns or omitted records. The input retains its original text.
    copy.s2k_preservation = None;
    copy.case_results.clear();
    copy.steel_design_results.clear();
    copy.concrete_design_results.clear();
    copy
}
fn semantic_value(model: &SapModel) -> Result<Value, FormatError> {
    serde_json::to_value(semantic_model(model))
        .map_err(|e| unsupported("model", format!("cannot inspect native projection: {e}")))
}
fn nullable(path: &[String]) -> bool {
    match path {
        [root, _, field] if root == "materials" => matches!(
            field.as_str(),
            "e_ortho" | "u_ortho" | "g_ortho" | "a_ortho"
        ),
        [root, _, field] if root == "frame_sections" => matches!(
            field.as_str(),
            "concrete_rebar" | "file_name" | "name_in_file"
        ),
        [root, _, field] if root == "points" => {
            matches!(field.as_str(), "spring_coupled" | "local_axes_matrix")
        }
        [root, _, field] if root == "frames" => {
            matches!(field.as_str(), "tc_limits" | "cable_params")
        }
        [root, _, field, quantity] if root == "frames" && field == "cable_params" => matches!(
            quantity.as_str(),
            "undeformed_length" | "target_tension_i" | "horizontal_tension"
        ),
        [root, _, loads, _, field] if root == "frames" && loads == "loads_temp" => {
            field == "pattern_name"
        }
        [root, _, loads, _, field]
            if root == "areas" && matches!(loads.as_str(), "loads_temp" | "loads_pressure") =>
        {
            field == "pattern_name"
        }
        [root, _, field] if root == "links" => field == "point_j",
        [root, _, field] if root == "load_cases" => field == "initial_case",
        _ => false,
    }
}
fn validate_finite_json(value: &Value, path: &mut Vec<String>) -> Result<(), FormatError> {
    match value {
        Value::Null if !nullable(path) => {
            return Err(unsupported(
                path.join("."),
                "nonfinite numeric data is not exportable",
            ))
        }
        Value::Object(map) => {
            for (k, v) in map {
                path.push(k.clone());
                validate_finite_json(v, path)?;
                path.pop();
            }
        }
        Value::Array(array) => {
            for (i, v) in array.iter().enumerate() {
                path.push(format!("[{i}]"));
                validate_finite_json(v, path)?;
                path.pop();
            }
        }
        _ => {}
    }
    Ok(())
}
fn first_difference(a: &Value, b: &Value, path: &str) -> Option<String> {
    if a == b {
        return None;
    }
    match (a, b) {
        (Value::Object(a), Value::Object(b)) => {
            for key in a.keys().chain(b.keys()) {
                let p = if path.is_empty() {
                    key.clone()
                } else {
                    format!("{path}.{key}")
                };
                match (a.get(key), b.get(key)) {
                    (Some(a), Some(b)) => {
                        if let Some(p) = first_difference(a, b, &p) {
                            return Some(p);
                        }
                    }
                    _ => return Some(p),
                }
            }
            None
        }
        (Value::Array(a), Value::Array(b)) if a.len() == b.len() => a
            .iter()
            .zip(b)
            .enumerate()
            .find_map(|(i, (a, b))| first_difference(a, b, &format!("{path}[{i}]"))),
        _ => Some(path.into()),
    }
}
/// Backstop for represented fields not yet audited individually. Results and
/// lock state are intentionally excluded: generated streams are unanalysed.
pub(super) fn verify_projection(
    original: &SapModel,
    projected: &SapModel,
) -> Result<(), FormatError> {
    if let Some(field) =
        first_difference(&semantic_value(original)?, &semantic_value(projected)?, "")
    {
        return Err(unsupported(field,"native serialization/projection would change represented model data; no file was written"));
    }
    Ok(())
}
