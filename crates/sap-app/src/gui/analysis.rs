//! Native case/function editor over the validated API definitions.
use super::*;
use sap_model::{
    DynamicFunction, LoadCaseAssignment, LoadCaseType, ModalComboMethod, ModalSubType, NLGeomType,
    TimeIntegrationMethod,
};

pub(super) struct AnalysisEditor {
    pub open: bool,
    pub tab: usize,
    case: LoadCase,
    function: DynamicFunction,
    samples: String,
}
impl Default for AnalysisEditor {
    fn default() -> Self {
        Self {
            open: false,
            tab: 0,
            case: LoadCase::linear_static("STATIC1", "DEAD", 1.0),
            function: DynamicFunction {
                name: "FUNCTION1".into(),
                is_time_history: true,
                damping_ratio: 0.05,
                points: vec![(0.0, 0.0), (1.0, 1.0)],
            },
            samples: "0 0\n1 1".into(),
        }
    }
}
fn parse_samples(text: &str) -> Result<Vec<(f64, f64)>, String> {
    let mut points = Vec::new();
    for (idx, line) in text.lines().enumerate() {
        let line = line.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        if points.len() >= 100_000 {
            return Err("Function editor accepts at most 100000 samples.".into());
        }
        let parts: Vec<_> = line
            .split(|c: char| c.is_whitespace() || c == ',')
            .filter(|s| !s.is_empty())
            .collect();
        if parts.len() != 2 {
            return Err(format!("Line {}: enter exactly two numbers.", idx + 1));
        }
        let x = parts[0]
            .parse::<f64>()
            .map_err(|_| format!("Line {}: invalid first number.", idx + 1))?;
        let y = parts[1]
            .parse::<f64>()
            .map_err(|_| format!("Line {}: invalid second number.", idx + 1))?;
        points.push((x, y));
    }
    Ok(points)
}
fn save_function(model: &mut SapModel, state: &AnalysisEditor) -> Result<String, String> {
    let mut function = state.function.clone();
    function.points = parse_samples(&state.samples)?;
    sap_api::set_function_definition(model, function).map_err(|e| e.to_string())?;
    Ok(format!("Saved function {}.", state.function.name))
}
fn new_case(kind: LoadCaseType, name: &str) -> LoadCase {
    let mut case = LoadCase::linear_static(name, "DEAD", 1.0);
    case.case_type = kind;
    if matches!(
        kind,
        LoadCaseType::Modal
            | LoadCaseType::ResponseSpectrum
            | LoadCaseType::LinearHistory
            | LoadCaseType::LinearDynamic
            | LoadCaseType::NonlinearHistory
            | LoadCaseType::NonlinearDynamic
    ) {
        case.loads.clear();
    }
    if kind == LoadCaseType::Modal {
        case = LoadCase::modal_eigen(name, 12);
    }
    case
}
fn text_choice(
    ui: &mut egui::Ui,
    id: impl std::hash::Hash,
    value: &mut String,
    choices: impl Iterator<Item = String>,
) {
    egui::ComboBox::from_id_salt(id)
        .selected_text(value.as_str())
        .show_ui(ui, |ui| {
            for name in choices {
                ui.selectable_value(value, name.clone(), name);
            }
        });
}
fn static_loads(ui: &mut egui::Ui, model: &SapModel, case: &mut LoadCase) {
    let mut remove = None;
    for (i, load) in case.loads.iter_mut().enumerate() {
        ui.horizontal(|ui| {
            text_choice(
                ui,
                ("case_pattern", i),
                &mut load.load_name,
                model.load_patterns.keys().cloned(),
            );
            ui.add(egui::DragValue::new(&mut load.scale_factor).speed(0.1));
            if ui.button("Remove").clicked() {
                remove = Some(i);
            }
        });
    }
    if let Some(i) = remove {
        case.loads.remove(i);
    }
    if ui.button("Add Pattern").clicked() {
        case.loads.push(LoadCaseAssignment {
            load_name: model
                .load_patterns
                .keys()
                .next()
                .cloned()
                .unwrap_or_default(),
            scale_factor: 1.0,
        });
    }
}
fn dynamic_loads(
    ui: &mut egui::Ui,
    model: &SapModel,
    loads: &mut Vec<(String, String, f64)>,
    history: bool,
) {
    let mut remove = None;
    for (i, (function, target, scale)) in loads.iter_mut().enumerate() {
        ui.horizontal(|ui| {
            text_choice(
                ui,
                ("case_function", history, i),
                function,
                model
                    .functions
                    .values()
                    .filter(|f| f.is_time_history == history)
                    .map(|f| f.name.clone()),
            );
            text_choice(
                ui,
                ("case_target", history, i),
                target,
                ["U1", "U2", "U3"]
                    .into_iter()
                    .map(str::to_owned)
                    .chain(model.load_patterns.keys().filter(|_| history).cloned()),
            );
            ui.add(egui::DragValue::new(scale).speed(0.1));
            if ui.button("Remove").clicked() {
                remove = Some(i);
            }
        });
    }
    if let Some(i) = remove {
        loads.remove(i);
    }
    if ui.button("Add Function Assignment").clicked() {
        loads.push((
            model
                .functions
                .values()
                .find(|f| f.is_time_history == history)
                .map(|f| f.name.clone())
                .unwrap_or_default(),
            "U1".into(),
            1.0,
        ));
    }
}
fn case_controls(ui: &mut egui::Ui, model: &SapModel, case: &mut LoadCase) {
    ui.horizontal(|ui| {
        ui.label("Name:");
        ui.text_edit_singleline(&mut case.name);
        ui.checkbox(&mut case.run_flag, "Run this case");
    });
    let mut kind = case.case_type;
    egui::ComboBox::from_id_salt("analysis_case_type")
        .selected_text(format!("{kind:?}"))
        .show_ui(ui, |ui| {
            for value in [
                LoadCaseType::LinearStatic,
                LoadCaseType::NonlinearStatic,
                LoadCaseType::Modal,
                LoadCaseType::Buckling,
                LoadCaseType::ResponseSpectrum,
                LoadCaseType::LinearHistory,
                LoadCaseType::NonlinearHistory,
            ] {
                ui.selectable_value(&mut kind, value, format!("{value:?}"));
            }
        });
    if kind != case.case_type {
        *case = new_case(kind, &case.name);
    }
    ui.label(
        "Changing case type resets its settings. Saving an existing name replaces that definition.",
    );
    match case.case_type {
        LoadCaseType::LinearStatic | LoadCaseType::NonlinearStatic | LoadCaseType::Buckling => {
            static_loads(ui, model, case);
            if case.case_type == LoadCaseType::NonlinearStatic {
                ui.horizontal(|ui| {
                    ui.label("Geometry:");
                    ui.selectable_value(&mut case.nl_geom, NLGeomType::None, "Small displacement");
                    ui.selectable_value(&mut case.nl_geom, NLGeomType::PDelta, "P-Delta");
                });
                ui.horizontal(|ui| {
                    ui.label("Iterations:");
                    ui.add(egui::DragValue::new(&mut case.max_iterations).range(1..=10000));
                    ui.label("Tolerance:");
                    ui.add(
                        egui::DragValue::new(&mut case.convergence_tol)
                            .speed(1e-7)
                            .max_decimals(12),
                    );
                });
                ui.label("Element applicability and convergence are checked when analysis runs.");
            }
            if case.case_type == LoadCaseType::Buckling {
                ui.horizontal(|ui| {
                    ui.label("Buckling modes:");
                    ui.add(egui::DragValue::new(&mut case.num_buckling_modes).range(1..=10000));
                });
            }
        }
        LoadCaseType::Modal => {
            let old_subtype = case.modal_sub_type;
            ui.horizontal(|ui| {
                ui.selectable_value(&mut case.modal_sub_type, ModalSubType::Eigen, "Eigen");
                ui.selectable_value(&mut case.modal_sub_type, ModalSubType::Ritz, "Ritz");
                ui.label("Maximum modes:");
                ui.add(egui::DragValue::new(&mut case.max_modes).range(1..=10000));
            });
            if old_subtype != case.modal_sub_type && case.modal_sub_type == ModalSubType::Eigen {
                case.loads.clear();
            }
            if case.modal_sub_type == ModalSubType::Ritz {
                ui.label("Starting load patterns:");
                static_loads(ui, model, case);
            }
        }
        LoadCaseType::ResponseSpectrum => {
            let mut source = case.initial_case.clone().unwrap_or_default();
            ui.horizontal(|ui| {
                ui.label("Modal source:");
                text_choice(
                    ui,
                    "analysis_modal_source",
                    &mut source,
                    model
                        .load_cases
                        .values()
                        .filter(|c| c.case_type == LoadCaseType::Modal && c.name != case.name)
                        .map(|c| c.name.clone()),
                );
            });
            case.initial_case = if source.is_empty() {
                None
            } else {
                Some(source)
            };
            ui.horizontal(|ui| {
                for method in [
                    ModalComboMethod::Cqc,
                    ModalComboMethod::Srss,
                    ModalComboMethod::AbsSum,
                ] {
                    ui.selectable_value(&mut case.modal_combo, method, format!("{method:?}"));
                }
                ui.label("Damping ratio:");
                ui.add(egui::DragValue::new(&mut case.modal_damping).speed(0.005));
            });
            dynamic_loads(ui, model, &mut case.spectrum_loads, false);
            ui.label(
                "Function ordinates × scale are spectral acceleration in current model units.",
            );
        }
        LoadCaseType::LinearHistory
        | LoadCaseType::LinearDynamic
        | LoadCaseType::NonlinearHistory
        | LoadCaseType::NonlinearDynamic => {
            if matches!(
                case.case_type,
                LoadCaseType::NonlinearHistory | LoadCaseType::NonlinearDynamic
            ) {
                ui.label("Nonlinear link history from rest; continuation and geometric nonlinearity are unavailable.");
                ui.horizontal(|ui| {
                    ui.label("Maximum iterations per step:");
                    ui.add(egui::DragValue::new(&mut case.max_iterations).range(1..=10000));
                    ui.label("Convergence tolerance:");
                    ui.add(
                        egui::DragValue::new(&mut case.convergence_tol)
                            .speed(1e-7)
                            .max_decimals(12),
                    );
                });
            }
            ui.horizontal(|ui| {
                ui.label("Steps:");
                ui.add(egui::DragValue::new(&mut case.num_time_steps).range(1..=1_000_000));
                ui.label("dt (seconds):");
                ui.add(
                    egui::DragValue::new(&mut case.time_step_dt)
                        .speed(0.001)
                        .max_decimals(10),
                );
            });
            egui::ComboBox::from_id_salt("analysis_integrator")
                .selected_text(format!("{:?}", case.time_integration))
                .show_ui(ui, |ui| {
                    for method in [
                        TimeIntegrationMethod::NewmarkAverage,
                        TimeIntegrationMethod::NewmarkLinear,
                        TimeIntegrationMethod::HilberHughesTaylor,
                        TimeIntegrationMethod::WilsonTheta,
                    ] {
                        ui.selectable_value(
                            &mut case.time_integration,
                            method,
                            format!("{method:?}"),
                        );
                    }
                });
            if case.time_integration == TimeIntegrationMethod::HilberHughesTaylor {
                ui.horizontal(|ui| {
                    ui.label("HHT alpha [-1/3,0]:");
                    ui.add(egui::DragValue::new(&mut case.hht_alpha).speed(0.01));
                });
            }
            if case.time_integration == TimeIntegrationMethod::WilsonTheta {
                ui.horizontal(|ui| {
                    ui.label("Wilson theta (>=1.37):");
                    ui.add(egui::DragValue::new(&mut case.wilson_theta).speed(0.01));
                });
            }
            ui.horizontal(|ui| {
                ui.label("Rayleigh mass alpha:");
                ui.add(egui::DragValue::new(&mut case.rayleigh_damping.0).speed(0.001));
                ui.label("Stiffness beta:");
                ui.add(egui::DragValue::new(&mut case.rayleigh_damping.1).speed(0.001));
            });
            dynamic_loads(ui, model, &mut case.history_loads, true);
            ui.label("U1/U2/U3 are global base acceleration. Other targets are joint-force patterns. Ordinate × scale multiplies the selected excitation.");
            ui.label(
                "History patterns cannot contain self-weight, member loads or support motion yet.",
            );
        }
        _ => {
            ui.label("This imported case type is not editable here. Choose a supported type to create a replacement.");
        }
    }
}
impl SapGuiApp {
    pub(super) fn render_analysis_editor(&mut self, ctx: &egui::Context) {
        let mut state = std::mem::take(&mut self.analysis_editor);
        let mut open = state.open;
        if open {
            egui::Window::new("Analysis Cases and Functions").open(&mut open).default_width(690.0).vscroll(true).show(ctx,|ui| {
                ui.horizontal(|ui|{ui.selectable_value(&mut state.tab,0,"Cases");ui.selectable_value(&mut state.tab,1,"Functions");});
                ui.label(format!("Model units: {}. Time and period are seconds.",self.model.present_units.s2k_label()));
                if self.model.is_locked {ui.colored_label(Color32::YELLOW,"Use Edit > Unlock Model before saving definitions.");}
                let mut outcome=None;
                if state.tab==0 {
                    egui::ComboBox::from_id_salt("analysis_read_case").selected_text("Read existing case").show_ui(ui,|ui| {
                        for (name,case) in &self.model.load_cases {if ui.button(name).clicked() {state.case=case.clone();ui.close_menu();}}
                    });
                    case_controls(ui,&self.model,&mut state.case);
                    if ui.button("Save Analysis Case").clicked() {outcome=Some(sap_api::set_case_definition(&mut self.model,state.case.clone()).map(|_|format!("Saved case {}.",state.case.name)).map_err(|e|e.to_string()));}
                    ui.separator();
                    ui.label("Cases selected to run:");
                    let mut toggle=None;
                    for (name,case) in &self.model.load_cases {let mut run=case.run_flag;if ui.checkbox(&mut run,name).changed() {toggle=Some((name.clone(),run));}}
                    if let Some((name,run))=toggle {
                        if self.model.is_locked {outcome=Some(Err("Unlock the model before changing run selection.".into()));}
                        else {self.model.load_cases[&name].run_flag=run;self.model.invalidate_results();outcome=Some(Ok(format!("Case {name}: run={run}.")));if state.case.name==name {state.case.run_flag=run;}}
                    }
                } else {
                    egui::ComboBox::from_id_salt("analysis_read_function").selected_text("Read existing function").show_ui(ui,|ui| {
                        for (name,function) in &self.model.functions {if ui.button(name).clicked() {state.function=function.clone();state.samples=function.points.iter().map(|(x,y)|format!("{x} {y}")).collect::<Vec<_>>().join("\n");ui.close_menu();}}
                    });
                    ui.horizontal(|ui| {ui.label("Name:");ui.text_edit_singleline(&mut state.function.name);});
                    ui.horizontal(|ui| {ui.selectable_value(&mut state.function.is_time_history,true,"Time history");ui.selectable_value(&mut state.function.is_time_history,false,"Response spectrum");});
                    if !state.function.is_time_history {ui.horizontal(|ui| {ui.label("Reference damping ratio:");ui.add(egui::DragValue::new(&mut state.function.damping_ratio).speed(0.005));});}
                    ui.label("One pair per line: time/value or period/spectral acceleration. Spaces or commas; # starts a comment.");
                    ui.add(egui::TextEdit::multiline(&mut state.samples).code_editor().desired_rows(9).desired_width(f32::INFINITY));
                    if ui.button("Save Dynamic Function").clicked() {outcome=Some(save_function(&mut self.model,&state));}
                }
                if let Some(result)=outcome {self.status_msg=match result {Ok(message)=>{self.inspected_frame=None;message},Err(error)=>format!("Definition error: {error}")};}
                ui.separator();ui.label(&self.status_msg);
            });
        }
        state.open = open;
        self.analysis_editor = state;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn sample_text_and_type_switch_do_not_silently_drop_bad_input() {
        assert_eq!(
            parse_samples("# time,value\n0, 0\n1 2 # peak\n").unwrap(),
            vec![(0., 0.), (1., 2.)]
        );
        assert!(parse_samples("0 1 2").is_err());
        let mut model = SapModel::new(Units::KipInF);
        model.case_results.insert("old".into(), Default::default());
        let before = model.clone();
        let mut editor = AnalysisEditor::default();
        editor.samples = "0 0\n0 1".into();
        assert!(save_function(&mut model, &editor).is_err());
        assert_eq!(model, before);
        editor.samples = "0 0\n1 2".into();
        save_function(&mut model, &editor).unwrap();
        assert_eq!(
            model.functions["FUNCTION1"].points,
            vec![(0., 0.), (1., 2.)]
        );
        assert!(model.case_results.is_empty());
        let history = new_case(LoadCaseType::LinearHistory, "H");
        assert!(history.loads.is_empty());
        assert!(history.spectrum_loads.is_empty());
    }
    #[test]
    fn editor_renders_case_types_and_pointer_click_saves_function() {
        let mut app = SapGuiApp::new(Some(PathBuf::from("/nonexistent/analysis-editor.s2k")));
        app.model = SapModel::new(Units::KipInF);
        app.analysis_editor.open = true;
        let ctx = egui::Context::default();
        let input = |time| egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1400., 1200.))),
            time: Some(time),
            ..Default::default()
        };
        let mut time = 0.;
        for kind in [
            LoadCaseType::LinearStatic,
            LoadCaseType::NonlinearStatic,
            LoadCaseType::Modal,
            LoadCaseType::Buckling,
            LoadCaseType::ResponseSpectrum,
            LoadCaseType::LinearHistory,
            LoadCaseType::NonlinearHistory,
            LoadCaseType::NonlinearDynamic,
        ] {
            app.analysis_editor.case = new_case(kind, "TEST");
            time += 1.;
            let out = ctx.run(input(time), |ctx| app.render_analysis_editor(ctx));
            assert!(!out.shapes.is_empty());
        }
        app.analysis_editor.tab = 1;
        time += 1.;
        let _ = ctx.run(input(time), |ctx| app.render_analysis_editor(ctx));
        time += 1.;
        let out = ctx.run(input(time), |ctx| app.render_analysis_editor(ctx));
        let pos = out
            .shapes
            .iter()
            .find_map(|shape| {
                if let egui::epaint::Shape::Text(text) = &shape.shape {
                    if text.galley.job.text == "Save Dynamic Function" {
                        Some(text.pos + text.galley.size() * 0.5)
                    } else {
                        None
                    }
                } else {
                    None
                }
            })
            .expect("function save button visible");
        let mut down = input(time + 1.);
        down.events = vec![
            egui::Event::PointerMoved(pos),
            egui::Event::PointerButton {
                pos,
                button: egui::PointerButton::Primary,
                pressed: true,
                modifiers: Default::default(),
            },
        ];
        let _ = ctx.run(down, |ctx| app.render_analysis_editor(ctx));
        let mut up = input(time + 1.1);
        up.events = vec![egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed: false,
            modifiers: Default::default(),
        }];
        let _ = ctx.run(up, |ctx| app.render_analysis_editor(ctx));
        assert!(app.model.functions.contains_key("FUNCTION1"));
    }
}
