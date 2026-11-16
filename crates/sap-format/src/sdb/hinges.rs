//! Frame hinges of the native format: the `HingeProp` and `FrameHingeAssign`
//! sections, and the nonlinear static parameters that go with them.
//!
//! Both sections lie past the sequential reader of the model importer, so they
//! are located from the full-stream inventory and decoded here.
//!
//! Evidence is Examples 1-026 A and B, the only corpus files with a hinge
//! record. Every field that is mapped is read from there; a value that the two
//! files do not show is "not established": the frame keeps no hinge and an
//! unmodeled entry names it, so analysis refuses instead of working on another
//! model. What is established:
//!
//! * `S_HingeProp.m_type` 6 and 2 are the OAPI hinge types M3 and V2, with
//!   `m_behavior` 2 (deformation controlled). Other types are not read.
//! * `S_HingeNLProp` of a user-defined uncoupled hinge: `m_type` 1,
//!   `m_npoints` 9, one curve, `m_symmetric`, no calculated yield, all four
//!   yield and deformation scale factors 1. The curve `m_d`/`m_F` lists the 9
//!   points -E, -D, -C, -B, A, B, C, D, E; the files have the point A at the
//!   origin and the negative half the mirror of the positive half.
//!   `m_acceptpositive` is IO, LS, CP and `m_acceptnegative` its negative.
//! * `S_Hinge.m_ipropass` is the 1-based index of the defined property and
//!   `m_iprop` that of the object's own copy of it (`m_generated` 1, `m_obj`
//!   the line index, `m_n` the hinge ordinal), identical in every other field.
//!   `m_drel` is the relative distance from end I (the fixed end, where the
//!   published values need the hinge); the files show only 0.
//! * `S_HingeOverwrites`: Model A stores auto-subdivide false and Model B true,
//!   both with relative length 0.02 and "do not drop load" false.
//! * `FrameHingeAssign` has one record per line object, in line order.
//! * `m_AfterLastPt` 0 is stored as `beyond_e: None`: its meaning is not
//!   established, so a hinge that reaches point E is refused by the analysis.
use super::native::inspect_native_sdb;
use super::*;
use sap_model::hinge::{
    FrameHinge, FrameHingeOverwrite, HingeAcceptance, HingeDof, HingePoint, HingeProp,
    NonlinearStaticParams, ResultsSaved,
};

type Raw = HashMap<String, SdbVal>;

/// Parameters of each nonlinear static case, or why they are not established.
pub(super) type NlStaticParams = Vec<(String, Result<NonlinearStaticParams, String>)>;

fn bad<T>(what: impl Into<String>) -> Result<T, String> {
    Err(what.into())
}

fn field<'a>(record: &'a Raw, key: &str) -> Result<&'a SdbVal, String> {
    record
        .get(key)
        .ok_or_else(|| format!("field {key} is absent"))
}

fn int(record: &Raw, key: &str) -> Result<i32, String> {
    field(record, key)?
        .as_i32()
        .ok_or_else(|| format!("field {key} is not an integer"))
}

fn real(record: &Raw, key: &str) -> Result<f64, String> {
    field(record, key)?
        .as_f64()
        .ok_or_else(|| format!("field {key} is not a number"))
}

fn flag(record: &Raw, key: &str) -> Result<bool, String> {
    field(record, key)?
        .as_bool()
        .ok_or_else(|| format!("field {key} is not a flag"))
}

fn object<'a>(record: &'a Raw, key: &str) -> Result<&'a Raw, String> {
    field(record, key)?
        .as_obj()
        .ok_or_else(|| format!("field {key} is not a record"))
}

fn reals<'a>(record: &'a Raw, key: &str) -> Result<&'a [f64], String> {
    field(record, key)?
        .as_arr_f64()
        .ok_or_else(|| format!("field {key} is not a list of numbers"))
}

fn pin_int(record: &Raw, key: &str, observed: i32) -> Result<(), String> {
    match int(record, key)? {
        v if v == observed => Ok(()),
        v => bad(format!("{key} is {v}, only {observed} is established")),
    }
}

fn pin_real(record: &Raw, key: &str, observed: f64) -> Result<(), String> {
    match real(record, key)? {
        v if v == observed => Ok(()),
        v => bad(format!("{key} is {v}, only {observed} is established")),
    }
}

fn pin_flag(record: &Raw, key: &str, observed: bool) -> Result<(), String> {
    match flag(record, key)? {
        v if v == observed => Ok(()),
        v => bad(format!("{key} is {v}, only {observed} is established")),
    }
}

fn pin_reals(record: &Raw, key: &str, observed: &[f64]) -> Result<(), String> {
    let values = reals(record, key)?;
    if values == observed {
        Ok(())
    } else {
        bad(format!(
            "{key} is {values:?}, only {observed:?} is established"
        ))
    }
}

fn values_equal(a: &SdbVal, b: &SdbVal) -> bool {
    match (a, b) {
        (SdbVal::Null, SdbVal::Null) => true,
        (SdbVal::Bool(a), SdbVal::Bool(b)) => a == b,
        (SdbVal::I32(a), SdbVal::I32(b)) => a == b,
        (SdbVal::I64(a), SdbVal::I64(b)) => a == b,
        (SdbVal::F64(a), SdbVal::F64(b)) => a.to_bits() == b.to_bits(),
        (SdbVal::Str(a), SdbVal::Str(b)) => a == b,
        (SdbVal::ArrBool(a), SdbVal::ArrBool(b)) => a == b,
        (SdbVal::ArrI32(a), SdbVal::ArrI32(b)) => a == b,
        (SdbVal::ArrF64(a), SdbVal::ArrF64(b)) => {
            a.len() == b.len() && a.iter().zip(b).all(|(a, b)| a.to_bits() == b.to_bits())
        }
        (SdbVal::Obj(a), SdbVal::Obj(b)) => records_equal(a, b, &[]),
        (SdbVal::ArrObj(a), SdbVal::ArrObj(b)) => {
            a.len() == b.len() && a.iter().zip(b).all(|(a, b)| records_equal(a, b, &[]))
        }
        _ => false,
    }
}

/// Equality of two records over all fields except the named ones.
fn records_equal(a: &Raw, b: &Raw, ignore: &[&str]) -> bool {
    let kept = |r: &Raw| r.keys().filter(|k| !ignore.contains(&k.as_str())).count();
    kept(a) == kept(b)
        && a.iter()
            .filter(|(k, _)| !ignore.contains(&k.as_str()))
            .all(|(k, v)| b.get(k).is_some_and(|w| values_equal(v, w)))
}

/// A user-defined uncoupled hinge property as the files show it, or the
/// reason it is not established.
fn decode_prop(record: &Raw) -> Result<HingeProp, String> {
    let name = field(record, "m_name")?
        .as_str()
        .filter(|n| !n.is_empty())
        .ok_or("the property has no name")?
        .to_string();
    pin_flag(record, "m_alive", true)?;
    pin_int(record, "m_behavior", 2)?;
    let dof = match int(record, "m_type")? {
        2 => HingeDof::V2,
        6 => HingeDof::M3,
        other => {
            return bad(format!(
                "hinge type {other} is not established (only 2 and 6)"
            ))
        }
    };
    pin_flag(
        object(record, "m_HingeCurveData")?,
        "m_CurveDataExists",
        false,
    )?;
    pin_int(object(record, "m_FiberHinge")?, "m_nFiber", 0)?;
    let nl = object(record, "m_nlprop")?;
    pin_int(nl, "m_type", 1)?;
    pin_int(nl, "m_npoints", 9)?;
    pin_int(nl, "m_nCurves", 1)?;
    pin_flag(nl, "m_calcyield", false)?;
    pin_flag(nl, "m_calcdyield", false)?;
    pin_flag(nl, "m_symmetric", true)?;
    pin_int(nl, "m_SymmetryCondition", 0)?;
    pin_flag(nl, "m_reldis", false)?;
    pin_real(nl, "m_hlength", 1.0)?;
    pin_int(nl, "m_AfterLastPt", 0)?;
    pin_int(nl, "m_MLPType", 0)?;
    pin_int(nl, "m_ForceControlLoadDrop", 0)?;
    // The files show all four scale factors as 1, which cannot tell which
    // field scales what, so no other value is read.
    for key in ["m_Yield", "m_dyield", "m_yieldn", "m_dyieldn"] {
        pin_real(nl, key, 1.0)?;
    }
    let curves = field(nl, "m_curve")?
        .as_arr_obj()
        .ok_or("m_curve is not a list of records")?;
    let [curve] = curves else {
        return bad(format!("{} curves, only 1 is established", curves.len()));
    };
    let (d, f) = (reals(curve, "m_d")?, reals(curve, "m_F")?);
    if d.len() != 9 || f.len() != 9 {
        return bad("the curve does not have 9 points");
    }
    if d[4] != 0.0 || f[4] != 0.0 {
        return bad("point A is not the origin");
    }
    if (0..4).any(|k| d[3 - k] != -d[5 + k] || f[3 - k] != -f[5 + k]) {
        return bad("the negative half of the curve does not mirror the positive half");
    }
    let points: [HingePoint; 4] = std::array::from_fn(|k| HingePoint::new(f[5 + k], d[5 + k]));
    pin_reals(curve, "m_fdep", &[0.0; 6])?;
    if curve.get("m_kabs").and_then(SdbVal::as_arr_i32) != Some(&[0; 6]) {
        return bad("m_kabs is not all 0");
    }
    pin_reals(curve, "m_acceptpositiveFC", &[0.5, 0.8, 1.0])?;
    pin_reals(curve, "m_acceptnegativeFC", &[-0.5, -0.8, -1.0])?;
    let [io, ls, cp] = *reals(curve, "m_acceptpositive")? else {
        return bad("the acceptance criteria are not 3 values");
    };
    if reals(curve, "m_acceptnegative")? != [-io, -ls, -cp] {
        return bad("the negative acceptance criteria do not mirror the positive ones");
    }
    let mut prop = HingeProp::symmetric(name, dof, points);
    prop.acceptance_positive = HingeAcceptance { io, ls, cp };
    prop.acceptance_negative = HingeAcceptance {
        io: -io,
        ls: -ls,
        cp: -cp,
    };
    // `m_AfterLastPt` 0: the meaning is not established.
    prop.beyond_e = None;
    prop.validate()?;
    Ok(prop)
}

/// The hinges of a frame as (index of the defined property in the section,
/// relative distance from end I), and its overwrite.
type FrameData = (Vec<(usize, f64)>, Option<FrameHingeOverwrite>);

/// Decodes the hinges of the line object `line` (1-based) from its record;
/// `props` is the whole `HingeProp` section. `Ok(None)` when it has none.
fn decode_frame(line: usize, assign: &Raw, props: &[Raw]) -> Result<Option<FrameData>, String> {
    let hinges = field(assign, "m_Hinge")?
        .as_arr_obj()
        .ok_or("m_Hinge is not a list of records")?;
    if usize::try_from(int(assign, "m_nhinge")?) != Ok(hinges.len()) {
        return bad("m_nhinge does not match the hinge records");
    }
    if hinges.is_empty() {
        return Ok(None);
    }
    let mut out = Vec::new();
    for (k, hinge) in hinges.iter().enumerate() {
        pin_int(hinge, "m_AutoHingeType", 0)?;
        pin_int(hinge, "m_ProgramGenerated", 0)?;
        pin_flag(hinge, "m_readabsdist", false)?;
        pin_real(hinge, "m_drel", 0.0)?;
        let index = |key: &str| -> Result<usize, String> {
            usize::try_from(int(hinge, key)?)
                .ok()
                .filter(|i| (1..=props.len()).contains(i))
                .ok_or_else(|| format!("{key} is not a property index"))
        };
        let (defined, generated) = (index("m_ipropass")?, index("m_iprop")?);
        let (defined_record, generated_record) = (&props[defined - 1], &props[generated - 1]);
        pin_int(defined_record, "m_generated", 0)?;
        pin_int(generated_record, "m_generated", 1)?;
        pin_flag(generated_record, "m_alive", true)?;
        if usize::try_from(int(generated_record, "m_obj")?) != Ok(line)
            || usize::try_from(int(generated_record, "m_n")?) != Ok(k + 1)
        {
            return bad("the object's own property copy does not belong to this hinge");
        }
        const OWN: [&str; 6] = [
            "m_name",
            "m_generated",
            "m_n",
            "m_obj",
            "m_Selected",
            "m_alive",
        ];
        if !records_equal(defined_record, generated_record, &OWN) {
            return bad("the object's own property copy differs from the defined property");
        }
        out.push((defined - 1, real(hinge, "m_drel")?));
    }
    let overwrites = object(assign, "m_Overwrites")?;
    pin_real(overwrites, "m_AutoSubdivideRelLength", 0.02)?;
    pin_flag(overwrites, "m_DoNotDropLoad", false)?;
    let overwrite = flag(overwrites, "m_AutoSubdivideLineObj")?.then_some(FrameHingeOverwrite {
        auto_subdivide: true,
        relative_length: 0.02,
        do_not_drop_load: false,
    });
    Ok(Some((out, overwrite)))
}

/// Parameters of a nonlinear static case from its analysis case record, or the
/// reason they are not established. They are applied only to models with an
/// imported hinge.
pub(super) fn nl_static_params(case: &Raw) -> Result<NonlinearStaticParams, String> {
    let control = case
        .get("m_SolControl")
        .and_then(SdbVal::as_obj)
        .ok_or("the case has no solution control record")?;
    let nonlinearity = case
        .get("m_Nonlinearity")
        .and_then(SdbVal::as_obj)
        .ok_or("the case has no nonlinearity record")?;
    // Hinges act only when frame hinge nonlinearity is on.
    pin_flag(nonlinearity, "m_nlframeh", true)?;
    pin_int(nonlinearity, "m_nlmattype", 2)?;
    pin_flag(control, "m_UseEventStepping", true)?;
    pin_real(control, "m_toleventF", 0.01)?;
    pin_real(control, "m_TolEventD", 0.01)?;
    let multiple = flag(control, "m_SaveMultipleSteps")?;
    let mut flags = [false; 6];
    for (slot, key) in flags.iter_mut().zip([
        "m_saveinitialstatefirst",
        "m_saveinitialstateother",
        "m_savefirstevent",
        "m_saveeveryevent",
        "m_savemaxmin",
        "m_saveslopechange",
    ]) {
        *slot = flag(control, key)?;
    }
    let observed = if multiple {
        [true, true, true, false, true, true]
    } else {
        [false; 6]
    };
    if flags != observed {
        return bad(format!(
            "the saved-state options {flags:?} are not the ones established with multiple states {multiple}"
        ));
    }
    let count = |key: &str, least: i32| -> Result<usize, String> {
        match int(control, key)? {
            v if v >= least => Ok(v as usize),
            v => bad(format!("{key} is {v}, below {least}")),
        }
    };
    let max_total_steps = count("m_MaxTotalSteps", 1)?;
    let max_null_steps = count("m_MaxFailedSubSteps", 0)?;
    let results_saved = if multiple {
        let min_saved = count("m_MinSavedStates", 1)?;
        let max_saved = count("m_MaxSavedStates", 1)?;
        if min_saved > max_saved {
            return bad("the minimum number of saved states exceeds the maximum");
        }
        ResultsSaved::MultipleStates {
            min_saved,
            max_saved,
            positive_only: flag(control, "m_PositiveOnly")?,
        }
    } else {
        ResultsSaved::FinalState
    };
    Ok(NonlinearStaticParams {
        results_saved,
        max_total_steps,
        max_null_steps,
        ..NonlinearStaticParams::default()
    })
}

/// The `HingeProp` and `FrameHingeAssign` records of a native stream and the
/// number of `S_Hinge` records in it. `None` when the stream cannot be
/// inventoried or the sections cannot be read.
fn read_sections(sdb: &SdbFile) -> Option<(Vec<Raw>, Vec<Raw>, usize)> {
    let document = inspect_native_sdb(sdb).ok()?;
    let top = &document.inventory().top_level;
    let read = |name: &str| -> Option<Vec<Raw>> {
        let Some(span) = top
            .iter()
            .find(|t| t.name == name && t.wire_type == "struct[]")
        else {
            return Some(Vec::new());
        };
        let mut reader = BinarySdbReader::new(&sdb.decompressed_stream);
        reader.pos = span.offset;
        reader.read_struct_array(true).ok()
    };
    let total = document
        .records()
        .iter()
        .filter(|r| r.class_id == "S_Hinge")
        .count();
    Some((read("HingeProp")?, read("FrameHingeAssign")?, total))
}

/// Imports the frame hinges of a native stream into `model` and, when at least
/// one was imported, the nonlinear static parameters of its cases.
///
/// `lines` are the line object names, 1-based (index 0 is unused);
/// `nl_static` the parameters decoded for each nonlinear static case. Hinges
/// that cannot be imported leave an unmodeled entry for their frame, and every
/// `S_Hinge` record is accounted for, so none is dropped silently. For a model
/// without an imported hinge `nl_static` is left at its default.
pub(super) fn import(
    sdb: &SdbFile,
    lines: &[String],
    nl_static: &NlStaticParams,
    model: &mut SapModel,
) {
    match read_sections(sdb) {
        Some((props, assigns, total)) => {
            import_decoded(&props, &assigns, total, lines, nl_static, model);
        }
        // A stream that cannot be inventoried is recorded by `skipped::record`.
        None if inspect_native_sdb(sdb).is_err() => {}
        None => model.unmodeled_source.push(UnmodeledSourceData {
            scope: UnmodeledScope::NonlinearCases,
            description: "frame hinge data that could not be decoded".into(),
        }),
    }
}

fn refuse(model: &mut SapModel, count: usize, who: &str, why: &str) {
    model.unmodeled_source.push(UnmodeledSourceData {
        scope: UnmodeledScope::NonlinearCases,
        description: format!(
            "{count} frame hinge assignment(s) of {who} that are not modeled: {why}"
        ),
    });
}

fn import_decoded(
    props: &[Raw],
    assigns: &[Raw],
    total: usize,
    lines: &[String],
    nl_static: &NlStaticParams,
    model: &mut SapModel,
) {
    if total == 0 {
        return;
    }
    if assigns.len() + 1 != lines.len() {
        refuse(
            model,
            total,
            "the model",
            "the hinge records do not match the line objects one to one",
        );
        return;
    }
    let mut accounted = 0usize;
    let mut imported = 0usize;
    for (index, assign) in assigns.iter().enumerate() {
        let count = assign
            .get("m_Hinge")
            .and_then(SdbVal::as_arr_obj)
            .map_or(0, <[Raw]>::len);
        if count == 0 {
            continue;
        }
        accounted += count;
        let name = &lines[index + 1];
        let decoded = decode_frame(index + 1, assign, props).and_then(|data| {
            let (hinges, overwrite) = data.ok_or("no hinge")?;
            if !model.frames.contains_key(name) {
                return bad("the line object is not imported as a frame");
            }
            let mut converted = Vec::new();
            for (prop_index, rel_dist) in hinges {
                let record = &props[prop_index];
                let prop = decode_prop(record).map_err(|why| {
                    let label = record.get("m_name").and_then(SdbVal::as_str).unwrap_or("?");
                    format!("property '{label}': {why}")
                })?;
                let clash = |known: &HingeProp| *known != prop;
                if model.hinge_props.get(&prop.name).is_some_and(clash) {
                    return bad(format!(
                        "two different hinge properties are named '{}'",
                        prop.name
                    ));
                }
                converted.push((prop, rel_dist));
            }
            Ok((converted, overwrite))
        });
        match decoded {
            Ok((hinges, overwrite)) => {
                imported += hinges.len();
                if let Some(frame) = model.frames.get_mut(name) {
                    for (prop, rel_dist) in &hinges {
                        frame.hinges.push(FrameHinge {
                            prop: prop.name.clone(),
                            rel_dist: *rel_dist,
                        });
                    }
                    frame.hinge_overwrite = overwrite;
                }
                for (prop, _) in hinges {
                    model.hinge_props.insert(prop.name.clone(), prop);
                }
            }
            Err(why) => refuse(model, count, &format!("frame '{name}'"), &why),
        }
    }
    if accounted != total {
        refuse(
            model,
            total.saturating_sub(accounted),
            "the model",
            "hinge records outside the frame assignments",
        );
    }
    // The parameters matter only to hinges, so they are read only for a model
    // with an imported hinge; every other model keeps the defaults and its
    // native export is unaffected.
    if imported > 0 {
        for (case, params) in nl_static {
            match params {
                Ok(params) => {
                    if let Some(lc) = model.load_cases.get_mut(case) {
                        lc.nl_static = params.clone();
                    }
                }
                Err(why) => model.unmodeled_source.push(UnmodeledSourceData {
                    scope: UnmodeledScope::Case(case.clone()),
                    description: format!(
                        "nonlinear static parameters of case '{case}' that are not modeled: {why}"
                    ),
                }),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sap_model::LoadCaseType;

    fn rec(pairs: Vec<(&str, SdbVal)>) -> Raw {
        pairs.into_iter().map(|(k, v)| (k.to_string(), v)).collect()
    }

    /// A curve of the 9 points -E..E from the positive points B..E.
    fn curve_with(d: [f64; 4], f: [f64; 4], f_at_a: f64) -> Raw {
        let mut dd = vec![];
        let mut ff = vec![];
        for k in (0..4).rev() {
            dd.push(-d[k]);
            ff.push(-f[k]);
        }
        dd.push(0.0);
        ff.push(f_at_a);
        dd.extend(d);
        ff.extend(f);
        rec(vec![
            ("m_d", SdbVal::ArrF64(dd)),
            ("m_F", SdbVal::ArrF64(ff)),
            ("m_fdep", SdbVal::ArrF64(vec![0.0; 6])),
            ("m_kabs", SdbVal::ArrI32(vec![0; 6])),
            ("m_acceptpositive", SdbVal::ArrF64(vec![2.0, 4.0, 6.0])),
            ("m_acceptnegative", SdbVal::ArrF64(vec![-2.0, -4.0, -6.0])),
            ("m_acceptpositiveFC", SdbVal::ArrF64(vec![0.5, 0.8, 1.0])),
            ("m_acceptnegativeFC", SdbVal::ArrF64(vec![-0.5, -0.8, -1.0])),
        ])
    }

    fn moment_curve() -> Raw {
        curve_with([0.0, 0.04, 0.04, 0.4], [1440.0, 1920.0, 480.0, 480.0], 0.0)
    }

    fn shear_curve() -> Raw {
        curve_with([0.0, 0.4, 0.8, 1.2], [70.0, 80.0, 90.0, 100.0], 0.0)
    }

    /// A hinge property record in the layout of Example 1-026.
    fn prop(name: &str, hinge_type: i32, generated: i32, obj: i32, n: i32, curve: Raw) -> Raw {
        let nl = rec(vec![
            ("m_type", SdbVal::I32(1)),
            ("m_npoints", SdbVal::I32(9)),
            ("m_nCurves", SdbVal::I32(1)),
            ("m_calcyield", SdbVal::Bool(false)),
            ("m_calcdyield", SdbVal::Bool(false)),
            ("m_symmetric", SdbVal::Bool(true)),
            ("m_SymmetryCondition", SdbVal::I32(0)),
            ("m_reldis", SdbVal::Bool(false)),
            ("m_hlength", SdbVal::F64(1.0)),
            ("m_AfterLastPt", SdbVal::I32(0)),
            ("m_MLPType", SdbVal::I32(0)),
            ("m_ForceControlLoadDrop", SdbVal::I32(0)),
            ("m_Yield", SdbVal::F64(1.0)),
            ("m_dyield", SdbVal::F64(1.0)),
            ("m_yieldn", SdbVal::F64(1.0)),
            ("m_dyieldn", SdbVal::F64(1.0)),
            ("m_curve", SdbVal::ArrObj(vec![curve])),
        ]);
        rec(vec![
            ("m_name", SdbVal::Str(name.into())),
            ("m_alive", SdbVal::Bool(true)),
            ("m_Selected", SdbVal::Bool(false)),
            ("m_type", SdbVal::I32(hinge_type)),
            ("m_behavior", SdbVal::I32(2)),
            ("m_generated", SdbVal::I32(generated)),
            ("m_obj", SdbVal::I32(obj)),
            ("m_n", SdbVal::I32(n)),
            ("m_nlprop", SdbVal::Obj(nl)),
            (
                "m_HingeCurveData",
                SdbVal::Obj(rec(vec![("m_CurveDataExists", SdbVal::Bool(false))])),
            ),
            (
                "m_FiberHinge",
                SdbVal::Obj(rec(vec![("m_nFiber", SdbVal::I32(0))])),
            ),
        ])
    }

    fn moment_prop() -> Raw {
        prop("Moment", 6, 0, 0, 0, moment_curve())
    }

    fn hinge(defined: i32, generated: i32) -> Raw {
        rec(vec![
            ("m_ipropass", SdbVal::I32(defined)),
            ("m_iprop", SdbVal::I32(generated)),
            ("m_AutoHingeType", SdbVal::I32(0)),
            ("m_ProgramGenerated", SdbVal::I32(0)),
            ("m_readabsdist", SdbVal::Bool(false)),
            ("m_drel", SdbVal::F64(0.0)),
        ])
    }

    fn assign(hinges: Vec<Raw>, subdivide: bool) -> Raw {
        rec(vec![
            ("m_nhinge", SdbVal::I32(hinges.len() as i32)),
            ("m_Hinge", SdbVal::ArrObj(hinges)),
            (
                "m_Overwrites",
                SdbVal::Obj(rec(vec![
                    ("m_AutoSubdivideLineObj", SdbVal::Bool(subdivide)),
                    ("m_AutoSubdivideRelLength", SdbVal::F64(0.02)),
                    ("m_DoNotDropLoad", SdbVal::Bool(false)),
                ])),
            ),
        ])
    }

    /// Defined "Moment" (1) and "Shear" (2), then the object's copies (3, 4)
    /// of the one line object.
    fn sections(subdivide: bool) -> (Vec<Raw>, Vec<Raw>) {
        (
            vec![
                moment_prop(),
                prop("Shear", 2, 0, 0, 0, shear_curve()),
                prop("1H1", 6, 1, 1, 1, moment_curve()),
                prop("1H2", 2, 1, 1, 2, shear_curve()),
            ],
            vec![assign(vec![hinge(1, 3), hinge(2, 4)], subdivide)],
        )
    }

    fn model_with_frame() -> SapModel {
        let mut model = SapModel::default();
        model.add_point("1", 0.0, 0.0, 0.0);
        model.add_point("2", 24.0, 0.0, 0.0);
        model.add_frame("1", "1", "2", "BEAM");
        let mut case = sap_model::LoadCase::linear_static("NL", "DEAD", 1.0);
        case.case_type = LoadCaseType::NonlinearStatic;
        model.load_cases.insert("NL".into(), case);
        model
    }

    fn import_into(
        model: &mut SapModel,
        sections: &(Vec<Raw>, Vec<Raw>),
        total: usize,
        nl: &NlStaticParams,
    ) {
        let lines = vec![String::new(), "1".to_string()];
        import_decoded(&sections.0, &sections.1, total, &lines, nl, model);
    }

    fn params() -> NonlinearStaticParams {
        NonlinearStaticParams {
            results_saved: ResultsSaved::MultipleStates {
                min_saved: 6,
                max_saved: 10,
                positive_only: false,
            },
            ..NonlinearStaticParams::default()
        }
    }

    /// Replaces `key` of the record reached from `record` through the named
    /// nested records (the first element of a list of records).
    fn set(record: &mut Raw, path: &[&str], key: &str, value: SdbVal) {
        let mut at = record;
        for step in path {
            at = match at.get_mut(*step).unwrap() {
                SdbVal::Obj(o) => o,
                SdbVal::ArrObj(a) => &mut a[0],
                _ => panic!("{step}"),
            };
        }
        at.insert(key.into(), value);
    }

    #[test]
    fn the_layout_of_example_1_026_decodes_to_the_two_properties_and_hinges() {
        let (props, assigns) = sections(true);
        let moment = decode_prop(&props[0]).unwrap();
        assert_eq!(moment.dof, HingeDof::M3);
        assert_eq!(moment.positive[1], HingePoint::new(1920.0, 0.04));
        assert_eq!(moment.negative[0].force, -1440.0);
        assert_eq!(moment.acceptance_positive.cp, 6.0);
        assert_eq!(moment.acceptance_negative.io, -2.0);
        assert_eq!(moment.beyond_e, None);
        let shear = decode_prop(&props[1]).unwrap();
        assert_eq!(shear.dof, HingeDof::V2);
        let (hinges, overwrite) = decode_frame(1, &assigns[0], &props).unwrap().unwrap();
        assert_eq!(hinges, vec![(0, 0.0), (1, 0.0)]);
        assert_eq!(overwrite, Some(FrameHingeOverwrite::default()));
        let (_, overwrite) = decode_frame(1, &sections(false).1[0], &props)
            .unwrap()
            .unwrap();
        assert_eq!(overwrite, None);
    }

    #[test]
    fn a_property_value_the_files_do_not_show_is_refused() {
        let nl = ["m_nlprop"];
        let ssc = ["m_nlprop", "m_curve"];
        let cases: Vec<(&[&str], &str, SdbVal, &str)> = vec![
            (&[], "m_type", SdbVal::I32(7), "hinge type 7"),
            (&[], "m_type", SdbVal::I32(11), "hinge type 11"),
            (&[], "m_behavior", SdbVal::I32(1), "m_behavior is 1"),
            (&[], "m_alive", SdbVal::Bool(false), "m_alive is false"),
            (&nl, "m_type", SdbVal::I32(2), "m_type is 2"),
            (&nl, "m_npoints", SdbVal::I32(11), "m_npoints is 11"),
            (&nl, "m_nCurves", SdbVal::I32(2), "m_nCurves is 2"),
            (&nl, "m_calcyield", SdbVal::Bool(true), "m_calcyield"),
            (&nl, "m_calcdyield", SdbVal::Bool(true), "m_calcdyield"),
            (&nl, "m_symmetric", SdbVal::Bool(false), "m_symmetric"),
            (&nl, "m_AfterLastPt", SdbVal::I32(1), "m_AfterLastPt is 1"),
            (&nl, "m_reldis", SdbVal::Bool(true), "m_reldis"),
            (&nl, "m_Yield", SdbVal::F64(2.0), "m_Yield is 2"),
            (&nl, "m_dyield", SdbVal::F64(2.0), "m_dyield is 2"),
            (&nl, "m_yieldn", SdbVal::F64(2.0), "m_yieldn is 2"),
            (&nl, "m_dyieldn", SdbVal::F64(2.0), "m_dyieldn is 2"),
            (&ssc, "m_kabs", SdbVal::ArrI32(vec![1; 6]), "m_kabs"),
            (
                &ssc,
                "m_d",
                SdbVal::ArrF64(vec![0.0; 7]),
                "does not have 9 points",
            ),
            (
                &ssc,
                "m_acceptnegative",
                SdbVal::ArrF64(vec![-1.0, -4.0, -6.0]),
                "negative acceptance",
            ),
            (
                &ssc,
                "m_acceptpositive",
                SdbVal::ArrF64(vec![6.0, 4.0, 2.0]),
                "acceptance",
            ),
            (
                &["m_HingeCurveData"],
                "m_CurveDataExists",
                SdbVal::Bool(true),
                "m_CurveDataExists",
            ),
        ];
        for (path, key, value, reason) in cases {
            let mut record = moment_prop();
            set(&mut record, path, key, value);
            let error = decode_prop(&record).unwrap_err();
            assert!(error.contains(reason), "{key}: {error}");
        }
    }

    #[test]
    fn a_curve_that_is_not_the_established_layout_is_refused() {
        let with_curve = |curve: Raw| {
            let mut record = moment_prop();
            set(
                &mut record,
                &["m_nlprop"],
                "m_curve",
                SdbVal::ArrObj(vec![curve]),
            );
            decode_prop(&record).unwrap_err()
        };
        // Point A off the origin.
        let a = curve_with([0.0, 0.04, 0.04, 0.4], [1440.0, 1920.0, 480.0, 480.0], 5.0);
        assert!(with_curve(a).contains("point A"));
        // A negative half that is not the mirror.
        let mut a = moment_curve();
        a.insert(
            "m_F".into(),
            SdbVal::ArrF64(vec![
                -480.0, -480.0, -1920.0, -1000.0, 0.0, 1440.0, 1920.0, 480.0, 480.0,
            ]),
        );
        assert!(with_curve(a).contains("mirror"));
        // Force at B must be positive (the property validation).
        let a = curve_with([0.0, 0.04, 0.04, 0.4], [0.0, 1920.0, 480.0, 480.0], 0.0);
        assert!(with_curve(a).contains("point B"));
        // Deformations may not decrease from B to E.
        let a = curve_with([0.0, 0.4, 0.04, 0.4], [1440.0, 1920.0, 480.0, 480.0], 0.0);
        assert!(with_curve(a).contains("must not decrease"));
    }

    #[test]
    fn a_hinge_value_the_files_do_not_show_is_refused() {
        let (props, _) = sections(true);
        let cases: Vec<(&str, SdbVal, &str)> = vec![
            ("m_drel", SdbVal::F64(0.5), "m_drel is 0.5"),
            ("m_AutoHingeType", SdbVal::I32(1), "m_AutoHingeType is 1"),
            ("m_ProgramGenerated", SdbVal::I32(1), "m_ProgramGenerated"),
            ("m_readabsdist", SdbVal::Bool(true), "m_readabsdist"),
            ("m_ipropass", SdbVal::I32(0), "not a property index"),
            ("m_iprop", SdbVal::I32(9), "not a property index"),
            ("m_ipropass", SdbVal::I32(3), "m_generated is 1"),
            ("m_iprop", SdbVal::I32(1), "m_generated is 0"),
            ("m_iprop", SdbVal::I32(4), "does not belong"),
        ];
        for (key, value, reason) in cases {
            let mut h = hinge(1, 3);
            h.insert(key.into(), value);
            let error = decode_frame(1, &assign(vec![h], true), &props).unwrap_err();
            assert!(error.contains(reason), "{key}: {error}");
        }
        // The copy belongs to another line object, or differs.
        let error = decode_frame(2, &assign(vec![hinge(1, 3)], true), &props).unwrap_err();
        assert!(error.contains("does not belong"), "{error}");
        let mut changed = props.clone();
        set(
            &mut changed[2],
            &["m_nlprop"],
            "m_hlength",
            SdbVal::F64(2.0),
        );
        let error = decode_frame(1, &assign(vec![hinge(1, 3)], true), &changed).unwrap_err();
        assert!(error.contains("differs"), "{error}");
        // Counts and overwrites.
        let mut a = assign(vec![hinge(1, 3)], true);
        a.insert("m_nhinge".into(), SdbVal::I32(2));
        assert!(decode_frame(1, &a, &props)
            .unwrap_err()
            .contains("m_nhinge"));
        for (key, value, reason) in [
            ("m_DoNotDropLoad", SdbVal::Bool(true), "m_DoNotDropLoad"),
            ("m_AutoSubdivideRelLength", SdbVal::F64(0.05), "0.05"),
        ] {
            let mut a = assign(vec![hinge(1, 3)], true);
            set(&mut a, &["m_Overwrites"], key, value);
            let error = decode_frame(1, &a, &props).unwrap_err();
            assert!(error.contains(reason), "{key}: {error}");
        }
    }

    #[test]
    fn imported_hinges_and_the_case_parameters_reach_the_model() {
        let mut model = model_with_frame();
        let nl: NlStaticParams = vec![("NL".into(), Ok(params()))];
        import_into(&mut model, &sections(true), 2, &nl);
        assert!(
            model.unmodeled_source.is_empty(),
            "{:?}",
            model.unmodeled_source
        );
        assert_eq!(model.hinge_props.len(), 2);
        let frame = &model.frames["1"];
        assert_eq!(frame.hinges.len(), 2);
        assert_eq!(frame.hinges[0].prop, "Moment");
        assert_eq!(frame.hinges[1].prop, "Shear");
        assert_eq!(frame.hinge_overwrite, Some(FrameHingeOverwrite::default()));
        assert_eq!(model.load_cases["NL"].nl_static, params());

        let mut model = model_with_frame();
        import_into(&mut model, &sections(false), 2, &nl);
        assert_eq!(model.frames["1"].hinge_overwrite, None);
    }

    #[test]
    fn a_refused_frame_keeps_no_hinge_and_leaves_the_parameters_alone() {
        let mut bad_hinge = hinge(2, 4);
        bad_hinge.insert("m_drel".into(), SdbVal::F64(1.0));
        let mut sections = sections(true);
        sections.1[0] = assign(vec![hinge(1, 3), bad_hinge], true);
        let mut model = model_with_frame();
        let nl: NlStaticParams = vec![("NL".into(), Ok(params()))];
        import_into(&mut model, &sections, 2, &nl);
        assert!(model.frames["1"].hinges.is_empty());
        assert!(model.hinge_props.is_empty());
        assert!(model.load_cases["NL"].nl_static.is_default());
        assert_eq!(model.unmodeled_source.len(), 1);
        let entry = &model.unmodeled_source[0];
        assert_eq!(entry.scope, UnmodeledScope::NonlinearCases);
        assert!(
            entry
                .description
                .starts_with("2 frame hinge assignment(s) of frame '1'")
                && entry.description.contains("m_drel is 1"),
            "{}",
            entry.description
        );
    }

    #[test]
    fn hinge_records_that_are_not_all_accounted_for_are_refused() {
        let none: NlStaticParams = Vec::new();
        // More S_Hinge records than the frame assignments hold.
        let mut model = model_with_frame();
        import_into(&mut model, &sections(true), 3, &none);
        assert_eq!(model.unmodeled_source.len(), 1);
        assert!(model.unmodeled_source[0]
            .description
            .starts_with("1 frame hinge assignment(s) of the model"));
        // The assignments do not match the line objects one to one.
        let mut model = model_with_frame();
        let mut two = sections(true);
        two.1.push(assign(vec![], true));
        import_into(&mut model, &two, 2, &none);
        assert!(model.frames["1"].hinges.is_empty());
        assert!(model.unmodeled_source[0]
            .description
            .starts_with("2 frame hinge assignment(s) of the model"));
        // A line object that is not a frame.
        let mut model = model_with_frame();
        model.frames.clear();
        import_into(&mut model, &sections(true), 2, &none);
        assert!(model.unmodeled_source[0]
            .description
            .contains("not imported as a frame"));
        // No S_Hinge record at all: nothing to say.
        let mut model = model_with_frame();
        import_into(&mut model, &(vec![], vec![assign(vec![], true)]), 0, &none);
        assert!(model.unmodeled_source.is_empty());
    }

    #[test]
    fn case_parameters_that_are_not_established_name_the_case_in_a_hinged_model_only() {
        let nl: NlStaticParams = vec![("NL".into(), Err("flags".into()))];
        let mut model = model_with_frame();
        import_into(&mut model, &sections(true), 2, &nl);
        assert_eq!(model.frames["1"].hinges.len(), 2);
        assert_eq!(model.unmodeled_source.len(), 1);
        assert_eq!(
            model.unmodeled_source[0].scope,
            UnmodeledScope::Case("NL".into())
        );
        // With no hinge in the model the case is not touched and not reported.
        let mut model = model_with_frame();
        import_into(&mut model, &(vec![], vec![assign(vec![], true)]), 0, &nl);
        assert!(model.unmodeled_source.is_empty());
        assert!(model.load_cases["NL"].nl_static.is_default());
    }

    fn case_record(multiple: bool) -> Raw {
        let mut control = vec![
            ("m_UseEventStepping", SdbVal::Bool(true)),
            ("m_toleventF", SdbVal::F64(0.01)),
            ("m_TolEventD", SdbVal::F64(0.01)),
            ("m_SaveMultipleSteps", SdbVal::Bool(multiple)),
            ("m_PositiveOnly", SdbVal::Bool(false)),
            ("m_MinSavedStates", SdbVal::I32(6)),
            ("m_MaxSavedStates", SdbVal::I32(10)),
            ("m_MaxTotalSteps", SdbVal::I32(200)),
            ("m_MaxFailedSubSteps", SdbVal::I32(50)),
        ];
        let on = [true, true, true, false, true, true];
        for (key, on) in [
            "m_saveinitialstatefirst",
            "m_saveinitialstateother",
            "m_savefirstevent",
            "m_saveeveryevent",
            "m_savemaxmin",
            "m_saveslopechange",
        ]
        .into_iter()
        .zip(on)
        {
            control.push((key, SdbVal::Bool(multiple && on)));
        }
        rec(vec![
            ("m_SolControl", SdbVal::Obj(rec(control))),
            (
                "m_Nonlinearity",
                SdbVal::Obj(rec(vec![
                    ("m_nlframeh", SdbVal::Bool(true)),
                    ("m_nlmattype", SdbVal::I32(2)),
                ])),
            ),
        ])
    }

    #[test]
    fn case_parameters_follow_the_solution_control_record() {
        assert_eq!(nl_static_params(&case_record(true)).unwrap(), params());
        let single = nl_static_params(&case_record(false)).unwrap();
        assert_eq!(single, NonlinearStaticParams::default());
        let control = ["m_SolControl"];
        let mut record = case_record(true);
        set(&mut record, &control, "m_PositiveOnly", SdbVal::Bool(true));
        set(&mut record, &control, "m_MaxTotalSteps", SdbVal::I32(300));
        set(&mut record, &control, "m_MaxFailedSubSteps", SdbVal::I32(7));
        let got = nl_static_params(&record).unwrap();
        assert_eq!(got.max_total_steps, 300);
        assert_eq!(got.max_null_steps, 7);
        assert_eq!(
            got.results_saved,
            ResultsSaved::MultipleStates {
                min_saved: 6,
                max_saved: 10,
                positive_only: true
            }
        );
    }

    #[test]
    fn case_parameters_the_files_do_not_show_are_refused() {
        let control = ["m_SolControl"];
        let nonlinearity = ["m_Nonlinearity"];
        let cases: Vec<(bool, &[&str], &str, SdbVal, &str)> = vec![
            (
                true,
                &control,
                "m_UseEventStepping",
                SdbVal::Bool(false),
                "m_UseEventStepping",
            ),
            (
                true,
                &control,
                "m_toleventF",
                SdbVal::F64(0.05),
                "m_toleventF",
            ),
            (
                true,
                &control,
                "m_TolEventD",
                SdbVal::F64(0.05),
                "m_TolEventD",
            ),
            (
                true,
                &control,
                "m_saveeveryevent",
                SdbVal::Bool(true),
                "saved-state options",
            ),
            (
                true,
                &control,
                "m_savemaxmin",
                SdbVal::Bool(false),
                "saved-state options",
            ),
            (
                false,
                &control,
                "m_saveslopechange",
                SdbVal::Bool(true),
                "saved-state options",
            ),
            (
                true,
                &control,
                "m_MaxTotalSteps",
                SdbVal::I32(0),
                "m_MaxTotalSteps",
            ),
            (
                true,
                &control,
                "m_MaxFailedSubSteps",
                SdbVal::I32(-1),
                "m_MaxFailedSubSteps",
            ),
            (
                true,
                &control,
                "m_MinSavedStates",
                SdbVal::I32(0),
                "m_MinSavedStates",
            ),
            (
                true,
                &control,
                "m_MinSavedStates",
                SdbVal::I32(11),
                "exceeds",
            ),
            (
                true,
                &nonlinearity,
                "m_nlframeh",
                SdbVal::Bool(false),
                "m_nlframeh",
            ),
            (
                true,
                &nonlinearity,
                "m_nlmattype",
                SdbVal::I32(1),
                "m_nlmattype",
            ),
        ];
        for (multiple, path, key, value, reason) in cases {
            let mut record = case_record(multiple);
            set(&mut record, path, key, value);
            let error = nl_static_params(&record).unwrap_err();
            assert!(error.contains(reason), "{key}: {error}");
        }
        assert!(nl_static_params(&rec(vec![])).is_err());
    }
}
