//! Test C: the pieces of part 4 of the video. (1) A line drawn with the
//! PolyFrame tool and broken at selected joints equals the divide command;
//! (2) a library material reaches the analysis; (3) the opt-in vendor library.

use super::{frame, grid_app, key, click, pane};
use crate::gui::materials::{add_material, Pick};
use crate::gui::{SapGuiApp, ToolMode, ViewProjection};
use eframe::egui::{self, Pos2, Rect, Vec2};
use sap_format::material_library::{
    library_files_in, library_material, read_material_library_dir, MaterialLibrary,
};
use sap_model::{
    FramePropType, FrameSection, LoadCase, LoadPattern, LoadPatternType, MatType, PointLoadForce,
    SapModel,
};
use sap_solver::analyze_model;
use sap_units::Units;
use std::path::PathBuf;

// ---------------------------------------------------------------------------
// C1: draw, add joints, break at intersections = divide

/// Clicks the button `label` of the workflows window (the window scrolls, so
/// the wheel moves it until the button is drawn), as the tests of
/// `workflows.rs` do.
fn click_workflow_button(app: &mut SapGuiApp, ctx: &egui::Context, label: &str) {
    let input = |time: f64| egui::RawInput {
        screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1400.0, 3000.0))),
        time: Some(time),
        ..Default::default()
    };
    let find = |out: &egui::FullOutput| {
        out.shapes.iter().find_map(|shape| match &shape.shape {
            egui::epaint::Shape::Text(text) if text.galley.job.text == label => {
                Some(text.pos + text.galley.size() * 0.5)
            }
            _ => None,
        })
    };
    let mut time = 100.0;
    let mut found = None;
    for _ in 0..40 {
        time += 0.1;
        let _ = ctx.run(input(time), |ctx| app.render_workflows(ctx));
        time += 0.1;
        let out = ctx.run(input(time), |ctx| app.render_workflows(ctx));
        if let Some(pos) = find(&out) {
            found = Some(pos);
            break;
        }
        time += 0.1;
        let mut wheel = input(time);
        wheel.events = vec![
            egui::Event::PointerMoved(Pos2::new(120.0, 200.0)),
            egui::Event::MouseWheel {
                unit: egui::MouseWheelUnit::Point,
                delta: Vec2::new(0.0, -150.0),
                modifiers: Default::default(),
            },
        ];
        let _ = ctx.run(wheel, |ctx| app.render_workflows(ctx));
    }
    let pos = found.unwrap_or_else(|| panic!("button {label:?} must become visible"));
    for pressed in [true, false] {
        time += 0.1;
        let mut raw = input(time);
        raw.events = vec![
            egui::Event::PointerMoved(pos),
            egui::Event::PointerButton {
                pos,
                button: egui::PointerButton::Primary,
                pressed,
                modifiers: Default::default(),
            },
        ];
        let _ = ctx.run(raw, |ctx| app.render_workflows(ctx));
    }
}

#[test]
fn a_polyframe_line_with_nine_joints_broken_at_intersections_equals_divide_into_ten() {
    let z0 = 0.0;
    // Two typed joints 10 m apart (a 6-8-10 triangle), neither on the grid.
    let (a, b) = ([1.0, 2.0, z0], [7.0, 10.0, z0]);
    let (mut app, ctx, mut time) = grid_app(ViewProjection::PlanXY, z0, ToolMode::PolyFrame);
    app.model.add_point("A", a[0], a[1], a[2]);
    app.model.add_point("B", b[0], b[1], b[2]);
    frame(&mut app, &ctx, &mut time, Vec::new());
    let (view, _) = pane(&app);
    click(&mut app, &ctx, &mut time, view.project(a));
    click(&mut app, &ctx, &mut time, view.project(b));
    key(&mut app, &ctx, &mut time, egui::Key::Enter);
    assert_eq!(app.model.frames.len(), 1, "{}", app.status_msg);
    assert_eq!(app.model.points.len(), 2, "the clicks snapped to the typed joints");
    let line = app.model.frames.values().next().unwrap();
    assert_eq!((line.point_i.as_str(), line.point_j.as_str()), ("A", "B"));
    let section = line.section.clone();
    let line_name = line.name.clone();

    // Nine joints at equal spacing on it, typed.
    for k in 1..10 {
        let t = k as f64 / 10.0;
        app.model.add_point(
            format!("P{k}"),
            a[0] + (b[0] - a[0]) * t,
            a[1] + (b[1] - a[1]) * t,
            z0,
        );
    }
    for k in 1..10 {
        app.model.points[&format!("P{k}")].selected = true;
    }
    app.model.frames[&line_name].selected = true;
    app.workflows.open = true;
    app.workflows.tab = 0;
    click_workflow_button(&mut app, &ctx, "Break Selected at Intersections");
    assert!(
        app.status_msg.starts_with("Broke the selected frames"),
        "{}",
        app.status_msg
    );

    // The reference: the same line divided into ten.
    let mut reference = SapModel::new(Units::KnMC);
    reference.add_point("A", a[0], a[1], a[2]);
    reference.add_point("B", b[0], b[1], b[2]);
    reference.add_frame("1", "A", "B", section.clone());
    reference.frames["1"].selected = true;
    reference.divide_selected_frames(10).unwrap();

    let sorted = |m: &SapModel| -> Vec<(String, [f64; 3])> {
        let mut v: Vec<_> = m.points.values().map(|p| (p.name.clone(), [p.x, p.y, p.z])).collect();
        v.sort_by(|p, q| p.1.partial_cmp(&q.1).unwrap());
        v
    };
    let (got, want) = (sorted(&app.model), sorted(&reference));
    assert_eq!(got.len(), 11);
    assert_eq!(got.len(), want.len());
    for ((_, c), (_, d)) in got.iter().zip(&want) {
        for k in 0..3 {
            assert!((c[k] - d[k]).abs() <= 1e-12, "{c:?} against {d:?}");
        }
    }
    assert_eq!(app.model.frames.len(), 10);
    assert_eq!(app.model.frames.len(), reference.frames.len());
    let pairs = |m: &SapModel, order: &[(String, [f64; 3])]| {
        let index = |name: &str| order.iter().position(|(n, _)| n == name).unwrap();
        let mut v: Vec<(usize, usize)> = m
            .frames
            .values()
            .map(|f| (index(&f.point_i), index(&f.point_j)))
            .collect();
        v.sort();
        v
    };
    let expected: Vec<(usize, usize)> = (0..10).map(|k| (k, k + 1)).collect();
    assert_eq!(pairs(&app.model, &got), expected);
    assert_eq!(pairs(&reference, &want), expected);
    assert!(app.model.frames.values().all(|f| f.section == section));
}

// ---------------------------------------------------------------------------
// C2: a library material reaches the analysis

/// One concrete in N and mm (E 36000 N/mm2, f'c 60 N/mm2), in the element
/// names and units the reader's own tests use.
const C60_XML: &str = r#"<?xml version="1.0" encoding="utf-8" ?>
<propertyFile xmlns="http://www.csiberkeley.com">
  <control lengthUnits="mm" forceUnits="N">
    <fileID>P7</fileID>
  </control>
  <region name="TestRegion" lengthUnits="mm" forceUnits="N">
    <collection type="Materials">
      <standard name="Synthetic">
        <material type="concrete" grade="C60" name="C60" isDefault="true">
          <weightDensity>2.5E-05</weightDensity>
          <massDensity>2.55E-09</massDensity>
          <modulusOfElasticity>36000</modulusOfElasticity>
          <poissonsRatio>0.2</poissonsRatio>
          <coefficientOfThermalExpansion>0.00001</coefficientOfThermalExpansion>
          <compressiveStrength>60</compressiveStrength>
          <isLightWeight>false</isLightWeight>
          <shearStrengthReductionFactor>1</shearStrengthReductionFactor>
        </material>
      </standard>
    </collection>
  </region>
</propertyFile>
"#;

fn temp_dir(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("structurax_p7_{}_{tag}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// Tip deflection of a 3 m square cantilever with a 50 kN tip load, section
/// shear areas `shear_factor * A`.
fn cantilever_tip(model: &SapModel, side: f64, length: f64, load: f64, shear_factor: f64) -> f64 {
    let mut m = model.clone();
    m.load_cases.clear();
    m.load_patterns.clear();
    let area = side * side;
    m.frame_sections.insert(
        "SQ".into(),
        FrameSection {
            name: "SQ".into(),
            material: "C60".into(),
            prop_type: FramePropType::General,
            area,
            i33: side.powi(4) / 12.0,
            i22: side.powi(4) / 12.0,
            torsional_j: 0.141 * side.powi(4),
            as2: shear_factor * area,
            as3: shear_factor * area,
            modifiers: [1.0; 8],
            ..FrameSection::default()
        },
    );
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", length, 0.0, 0.0);
    m.points["1"].restraint = [true; 6];
    m.add_frame("F", "1", "2", "SQ");
    m.load_patterns.insert(
        "P".into(),
        LoadPattern { name: "P".into(), pat_type: LoadPatternType::Dead, self_wt_multiplier: 0.0 },
    );
    m.load_cases.insert("P".into(), LoadCase::linear_static("P", "P", 1.0));
    m.points["2"].loads_force.push(PointLoadForce {
        load_pat: "P".into(),
        csys: "GLOBAL".into(),
        force: [0.0, 0.0, -load, 0.0, 0.0, 0.0],
    });
    analyze_model(&mut m).unwrap();
    -m.case_results["P"].joint_displacements["2"][2]
}

#[test]
fn a_library_material_added_through_the_command_reaches_the_tip_deflection() {
    let dir = temp_dir("c60");
    std::fs::write(dir.join("csimateriallibrarytest.xml"), C60_XML).unwrap();
    let library = read_material_library_dir(&dir).unwrap();
    let _ = std::fs::remove_dir_all(&dir);

    let mut model = SapModel::new(Units::KnMC);
    let pick = Pick {
        region: "TestRegion".into(),
        type_label: "concrete".into(),
        standard: "Synthetic".into(),
        grade: "C60".into(),
    };
    let message = add_material(&mut model, &library, &pick, "C60").unwrap();
    assert!(message.contains("C60"), "{message}");

    // 1 N/mm2 = 1e6 N/m2 = 1e3 kN/m2.
    let e = 36_000.0 * 1e3;
    let nu = 0.2;
    let g = e / (2.0 * (1.0 + nu));
    let m = &model.materials["C60"];
    assert_eq!(m.mat_type, MatType::Concrete);
    assert!((m.e - e).abs() <= 1e-12 * e, "E {} against {e}", m.e);
    assert!((m.fc - 60.0e3).abs() <= 1e-12 * 60.0e3);
    assert!((m.g - g).abs() <= 1e-12 * g);

    let (side, length, load): (f64, f64, f64) = (0.4, 3.0, 50.0);
    let (area, inertia) = (side * side, side.powi(4) / 12.0);
    // (a) Without shear deformation (shear areas 0): P L^3 / (3 E I).
    let bending = load * length.powi(3) / (3.0 * e * inertia);
    let tip = cantilever_tip(&model, side, length, load, 0.0);
    assert!((tip - bending).abs() <= 1e-9 * bending, "{tip} against {bending}");
    // (b) With the shear area 5/6 A: P L^3 / (3 E I) + P L / (G As).
    let with_shear = bending + load * length / (g * (5.0 / 6.0) * area);
    let tip = cantilever_tip(&model, side, length, load, 5.0 / 6.0);
    assert!((tip - with_shear).abs() <= 1e-9 * with_shear, "{tip} against {with_shear}");
    assert!(with_shear > bending * 1.01, "the shear term is not negligible");
    // The material is the only thing that carries E: another E changes the result.
    let mut softer = model.clone();
    softer.materials["C60"].e *= 0.5;
    softer.materials["C60"].g *= 0.5;
    let soft = cantilever_tip(&softer, side, length, load, 0.0);
    assert!((soft - 2.0 * bending).abs() <= 1e-9 * bending);
}

// ---------------------------------------------------------------------------
// C3: opt-in vendor library (STRUCTURAX_VENDOR_FIXTURES)

fn vendor_library() -> Option<MaterialLibrary> {
    let dir = PathBuf::from(std::env::var_os("STRUCTURAX_VENDOR_FIXTURES")?);
    if !library_files_in(&dir).is_ok_and(|f| !f.is_empty()) {
        eprintln!("skipped: no material library files in {}", dir.display());
        return None;
    }
    Some(read_material_library_dir(&dir).unwrap())
}

#[test]
fn vendor_a36_has_fy_36_ksi_and_special_names_survive_s2k() {
    let Some(library) = vendor_library() else { return };
    let mut model = SapModel::new(Units::KnMC);
    let pick_of = |e: &sap_format::material_library::LibraryEntry| Pick {
        region: e.region.clone(),
        type_label: e.type_label.clone(),
        standard: e.standard.clone(),
        grade: e.grade.clone(),
    };

    // A36 of the United States file, Fy = 36 ksi in kN/m2.
    let a36 = library
        .entries
        .iter()
        .find(|e| e.region == "United States" && e.name == "A36")
        .expect("the United States library has A36");
    add_material(&mut model, &library, &pick_of(a36), "A36").unwrap();
    let ksi = 4.448_221_615_260_5 / (0.0254 * 0.0254); // kip/in2 in kN/m2
    let fy = 36.0 * ksi;
    let m = &model.materials["A36"];
    assert_eq!(m.mat_type, MatType::Steel);
    assert!((m.fy - fy).abs() <= 1e-9 * fy, "Fy {} against {fy}", m.fy);

    // `C30/37` (Europe) and one Cyrillic grade (Russia).
    let eu = library
        .entries
        .iter()
        .find(|e| e.region == "Europe" && e.name == "C30/37")
        .expect("the European library has C30/37");
    let ru = library
        .entries
        .iter()
        .find(|e| {
            e.region == "Russia"
                && !e.name.is_ascii()
                && library_material(e, Units::KnMC, &e.name).is_ok()
        })
        .expect("the Russian library has a convertible Cyrillic name");
    let names = [eu.name.clone(), ru.name.clone()];
    for entry in [eu, ru] {
        add_material(&mut model, &library, &pick_of(entry), &entry.name).unwrap();
    }
    eprintln!("Cyrillic grade used: {}", ru.name);
    let text = sap_format::write_s2k(&model);
    let back = sap_format::parse_s2k(&text)
        .unwrap_or_else(|e| panic!("the S2K of the library materials does not parse: {e}"));
    for name in std::iter::once(&"A36".to_string()).chain(names.iter()) {
        let (a, b) = (&model.materials[name], back.materials.get(name));
        let b = b.unwrap_or_else(|| panic!("material {name:?} lost in the S2K round trip"));
        assert_eq!(a.mat_type, b.mat_type, "{name}");
        for (x, y) in [(a.e, b.e), (a.fy, b.fy), (a.fc, b.fc), (a.unit_weight, b.unit_weight)] {
            assert!((x - y).abs() <= 1e-9 * x.abs().max(y.abs()), "{name}: {x} against {y}");
        }
    }

    // The experimental native writer (not the private container) on the same
    // model: the outcome is printed, and a written file must read back with
    // both names.
    let path = std::env::temp_dir().join(format!("structurax_p7_names_{}.sdb", std::process::id()));
    let _ = std::fs::remove_file(&path);
    match sap_format::save_native_sdb_file(&model, &path) {
        Ok(()) => {
            let native = sap_format::load_sdb_file(&path);
            let _ = std::fs::remove_file(&path);
            let native = native.expect("native import");
            for name in &names {
                assert!(native.materials.contains_key(name.as_str()), "{name} lost in the native file");
            }
        }
        Err(error) => panic!("native export with C30/37 and a Cyrillic name was refused: {error}"),
    }
}
