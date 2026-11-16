//! The material library reader: a synthetic file in every portable test, the
//! vendor's files only in the opt-in tests (STRUCTURAX_VENDOR_FIXTURES).

use sap_format::material_library::{
    is_library_file_name, library_material, parse_material_library, read_material_library_dir,
    read_material_library_file, units_for, MaterialLibrary, MaterialLibraryError,
};
use sap_format::{parse_s2k, write_s2k};
use sap_model::{MatType, SapModel};
use sap_units::{ForceUnit, LengthUnit, Units};
use std::path::{Path, PathBuf};

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol * a.abs().max(b.abs()).max(f64::MIN_POSITIVE)
}

/// A unique directory under the system temporary directory.
fn temp_dir(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("structurax_matlib_{}_{tag}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// A synthetic library in N and mm with one block in kN and m and one
/// lightweight concrete, one aluminum and one unknown type.
const SYNTH: &str = r#"<?xml version="1.0" encoding="utf-8" ?>
<!-- a comment -->
<propertyFile xmlns="http://www.csiberkeley.com"
 xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
 xsi:schemaLocation="http://www.csiberkeley.com CSiMaterialLibrary.xsd">
  <control lengthUnits="mm" forceUnits="N">
    <fileID>Test</fileID>
  </control>
  <region name="TestRegion" lengthUnits="mm" forceUnits="N">
    <collection type="Materials">
      <standard name="EN &amp; Test">
        <material type="Steel" grade="S355" name="S355" isDefault="true">
          <weightDensity>7.69728639422648E-05</weightDensity>
          <massDensity>7.84904737995992E-09</massDensity>
          <modulusOfElasticity>210000</modulusOfElasticity>
          <poissonsRatio>0.3</poissonsRatio>
          <coefficientOfThermalExpansion>0.0000117</coefficientOfThermalExpansion>
          <minimumYieldStress>355</minimumYieldStress>
          <minimumTensileStress>510</minimumTensileStress>
          <effectiveYieldStress>390</effectiveYieldStress>
          <strainAtUltimate>0.2</strainAtUltimate>
        </material>
        <material type="concrete" grade="C30/37" name="C30/37">
          <weightDensity>2.5E-05</weightDensity>
          <massDensity>2.55E-09</massDensity>
          <modulusOfElasticity>33000</modulusOfElasticity>
          <poissonsRatio>0.2</poissonsRatio>
          <coefficientOfThermalExpansion>0.00001</coefficientOfThermalExpansion>
          <compressiveStrength>30</compressiveStrength>
          <isLightWeight>false</isLightWeight>
          <shearStrengthReductionFactor>1</shearStrengthReductionFactor>
        </material>
        <material type="concrete" grade="LW 30" name="LW30">
          <weightDensity>1.9E-05</weightDensity>
          <massDensity>1.9E-09</massDensity>
          <modulusOfElasticity>20000</modulusOfElasticity>
          <poissonsRatio>0.2</poissonsRatio>
          <coefficientOfThermalExpansion>0.00001</coefficientOfThermalExpansion>
          <compressiveStrength>30</compressiveStrength>
          <isLightWeight>true</isLightWeight>
          <shearStrengthReductionFactor>0.85</shearStrengthReductionFactor>
        </material>
        <material type="concrete" grade="Reduced" name="RED">
          <weightDensity>2.5E-05</weightDensity>
          <massDensity>2.55E-09</massDensity>
          <modulusOfElasticity>30000</modulusOfElasticity>
          <poissonsRatio>0.2</poissonsRatio>
          <coefficientOfThermalExpansion>0.00001</coefficientOfThermalExpansion>
          <compressiveStrength>25</compressiveStrength>
          <shearStrengthReductionFactor>0.85</shearStrengthReductionFactor>
        </material>
        <material type="aluminum" grade="6061" name="AL6061">
          <weightDensity>2.7E-05</weightDensity>
          <massDensity>2.7E-09</massDensity>
          <modulusOfElasticity>70000</modulusOfElasticity>
          <poissonsRatio>0.33</poissonsRatio>
          <coefficientOfThermalExpansion>0.000023</coefficientOfThermalExpansion>
        </material>
        <material type="Glass" grade="G" name="GLASS">
          <weightDensity>2.5E-05</weightDensity>
        </material>
        <material type="rebar" grade="Cyr &#x41;&#x0421;" name="REB" forceUnits="kN" lengthUnits="m">
          <weightDensity>78.5</weightDensity>
          <massDensity>7.85</massDensity>
          <modulusOfElasticity>2E+08</modulusOfElasticity>
          <poissonsRatio>0.3</poissonsRatio>
          <coefficientOfThermalExpansion>1.2E-05</coefficientOfThermalExpansion>
          <minimumYieldStress>400000</minimumYieldStress>
          <minimumTensileStress>500000</minimumTensileStress>
        </material>
        <material type="Steel" grade="NoE" name="NOE">
          <weightDensity>7.69728639422648E-05</weightDensity>
          <massDensity>7.84904737995992E-09</massDensity>
          <poissonsRatio>0.3</poissonsRatio>
          <coefficientOfThermalExpansion>0.0000117</coefficientOfThermalExpansion>
          <minimumYieldStress>355</minimumYieldStress>
          <minimumTensileStress>510</minimumTensileStress>
        </material>
      </standard>
    </collection>
  </region>
</propertyFile>
"#;

fn synth() -> MaterialLibrary {
    let mut lib = MaterialLibrary::default();
    lib.merge(parse_material_library(SYNTH, "synth.xml").unwrap());
    lib
}

fn syntax_line(text: &str) -> (usize, String) {
    match parse_material_library(text, "t.xml") {
        Err(MaterialLibraryError::Syntax { line, message, .. })
        | Err(MaterialLibraryError::Invalid { line, message, .. }) => (line, message),
        other => panic!("expected an error, got {other:?}"),
    }
}

#[test]
fn synthetic_file_is_read_with_units_entities_and_flags() {
    let lib = synth();
    assert_eq!(lib.entries.len(), 8);
    assert_eq!(lib.regions(), vec!["TestRegion".to_string()]);
    let s355 = lib
        .find("TestRegion", "STEEL", "EN & Test", "S355")
        .unwrap();
    assert_eq!(s355.units, Units::NMmC);
    assert_eq!(s355.mat_type, Some(MatType::Steel));
    assert!(s355.is_default);
    assert_eq!(s355.values.e, Some(210000.0));
    assert_eq!(s355.values.fye, Some(390.0));
    assert_eq!(s355.source, "synth.xml");
    assert_eq!(s355.line, 12);
    // The case of the type does not matter, the case of the grade does.
    assert!(lib
        .find("TestRegion", "Concrete", "EN & Test", "C30/37")
        .is_some());
    assert!(lib
        .find("TestRegion", "concrete", "EN & Test", "c30/37")
        .is_none());
    // Block override and numeric character references.
    let reb = lib
        .find("TestRegion", "rebar", "EN & Test", "Cyr A\u{0421}")
        .unwrap();
    assert_eq!(reb.units, Units::KnMC);
    let lw = lib
        .find("TestRegion", "concrete", "EN & Test", "LW 30")
        .unwrap();
    assert!(lw.values.lightweight);
    assert_eq!(lw.values.shear_reduction, Some(0.85));
    assert_eq!(lib.type_labels("TestRegion").len(), 5);
}

#[test]
fn units_of_the_file_convert_with_exact_factors() {
    let lib = synth();
    let s355 = lib
        .find("TestRegion", "steel", "EN & Test", "S355")
        .unwrap();
    let m = library_material(s355, Units::KnMC, "S355").unwrap();
    assert_eq!(m.mat_type, MatType::Steel);
    // N/mm2 to kN/m2: 1e-3 / 1e-6 = 1e3.
    assert!(close(m.e, 2.1e8, 1e-12));
    assert!(close(m.fy, 355_000.0, 1e-12));
    assert!(close(m.fu, 510_000.0, 1e-12));
    // N/mm3 to kN/m3: 1e-3 / 1e-9 = 1e6.
    assert!(close(m.unit_weight, 76.9728639422648, 1e-12));
    // N s2/mm4 to kN s2/m4: 1e-3 / 1e-12 = 1e9.
    assert!(close(m.unit_mass, 7.84904737995992, 1e-12));
    // G = E / (2 (1 + nu)) = 2.1e8 / 2.6.
    assert!(close(m.g, 2.1e8 / 2.6, 1e-12));
    assert!(close(m.g, 80_769_230.769_230_77, 1e-12));
    assert_eq!(m.u, 0.3);
    // The thermal coefficient is per degree C in both unit systems.
    assert!(close(m.a, 1.17e-5, 1e-12));
    assert_eq!(m.fc, 0.0);
    // Weight and mass density are two separate values: the file's pair is not
    // the g of the units (7.69728639422648 / 7.84904737995992 = 9.80665).
    // Making the file inconsistent changes only the one that was edited.
    let mut odd = s355.clone();
    odd.values.mass_density = Some(1.0e-9);
    let m2 = library_material(&odd, Units::KnMC, "S355").unwrap();
    assert!(close(m2.unit_weight, m.unit_weight, 1e-15));
    assert!(close(m2.unit_mass, 1.0, 1e-12));

    // The same entry in the units of the file: no change.
    let same = library_material(s355, Units::NMmC, "S355").unwrap();
    assert_eq!(same.e, 210_000.0);
    assert_eq!(same.fy, 355.0);
    // Without a separate grade for shear reinforcement, fys follows fy, as it
    // does for the built-in steel and after a native import; the native
    // writer refuses a steel whose fys it would have to change.
    assert_eq!(same.fys, 355.0);
    assert_eq!(m.fys, m.fy);

    // Concrete: fc, fy = fys = 0.
    let c = lib
        .find("TestRegion", "concrete", "EN & Test", "C30/37")
        .unwrap();
    let cm = library_material(c, Units::KnMC, "C30").unwrap();
    assert_eq!(cm.mat_type, MatType::Concrete);
    assert!(close(cm.fc, 30_000.0, 1e-12));
    assert_eq!((cm.fy, cm.fu, cm.fys), (0.0, 0.0, 0.0));
    assert!(close(cm.e, 3.3e7, 1e-12));

    // A kN, m block read into a kN, m model is not scaled.
    let r = lib
        .find("TestRegion", "rebar", "EN & Test", "Cyr A\u{0421}")
        .unwrap();
    let rm = library_material(r, Units::KnMC, "R").unwrap();
    assert_eq!(rm.mat_type, MatType::Rebar);
    assert_eq!(rm.e, 2.0e8);
    assert_eq!(rm.unit_weight, 78.5);
    assert_eq!(rm.unit_mass, 7.85);
    assert_eq!(rm.fy, 400_000.0);
    // ... and to kip, in it is scaled with the factors of sap_units.
    let ri = library_material(r, Units::KipInF, "R").unwrap();
    let ksi = Units::KnMC.stress_factor_to(Units::KipInF);
    assert!(close(ri.e, 2.0e8 * ksi, 1e-12));
    assert!(close(ri.e, 2.0e8 / 6_894.757_293_168, 1e-9));
}

#[test]
fn refusals_name_the_material_and_never_approximate() {
    let lib = synth();
    let get = |t: &str, g: &str| lib.find("TestRegion", t, "EN & Test", g).unwrap();
    let e = library_material(get("concrete", "LW 30"), Units::KnMC, "X").unwrap_err();
    assert!(e.contains("LW30") && e.contains("lightweight"), "{e}");
    let e = library_material(get("concrete", "Reduced"), Units::KnMC, "X").unwrap_err();
    assert!(e.contains("RED") && e.contains("shear"), "{e}");
    let e = library_material(get("aluminum", "6061"), Units::KnMC, "X").unwrap_err();
    assert!(e.contains("AL6061") && e.contains("not supported"), "{e}");
    let e = library_material(get("Glass", "G"), Units::KnMC, "X").unwrap_err();
    assert!(e.contains("GLASS") && e.contains("not supported"), "{e}");
    let e = library_material(get("steel", "NoE"), Units::KnMC, "X").unwrap_err();
    assert!(e.contains("NOE") && e.contains("modulus"), "{e}");
    let e = library_material(get("steel", "S355"), Units::KnMC, "  ").unwrap_err();
    assert!(e.contains("name"), "{e}");
    let mut bad = get("steel", "S355").clone();
    bad.values.nu = Some(0.5);
    assert!(library_material(&bad, Units::KnMC, "X").is_err());
    let mut bad = get("steel", "S355").clone();
    bad.values.fu = Some(300.0);
    assert!(library_material(&bad, Units::KnMC, "X")
        .unwrap_err()
        .contains("Fu"));
    let mut bad = get("steel", "S355").clone();
    bad.values.weight_density = None;
    assert!(library_material(&bad, Units::KnMC, "X")
        .unwrap_err()
        .contains("weight density"));
    let mut bad = get("steel", "S355").clone();
    bad.values.mass_density = None;
    assert!(library_material(&bad, Units::KnMC, "X")
        .unwrap_err()
        .contains("mass density"));
}

#[test]
fn directory_rule_alphabetical_later_wins_and_extensionless_names() {
    let dir = temp_dir("dir");
    let file = |name: &str, steel_e: &str, grade: &str, type_label: &str| {
        let text = SYNTH
            .replace(
                "<modulusOfElasticity>210000<",
                &format!("<modulusOfElasticity>{steel_e}<"),
            )
            .replace("grade=\"S355\"", &format!("grade=\"{grade}\""))
            .replace(
                "type=\"Steel\" grade",
                &format!("type=\"{type_label}\" grade"),
            );
        // Only the first material of the synthetic file is of interest; the
        // rest repeats in each file and is replaced by the later file.
        std::fs::write(dir.join(name), text).unwrap();
    };
    // Alphabetical without regard to case: A, b, c (no extension).
    file("CSiMaterialLibraryB.xml", "2", "S355", "Steel");
    file("csimateriallibrarya.XML", "1", "S355", "STEEL");
    file("csimateriallibraryc", "3", "S355", "steel");
    // Not read: schema, other extensions, other names, a directory.
    std::fs::write(dir.join("csimateriallibrary.xsd"), "<<garbage").unwrap();
    std::fs::write(dir.join("csimateriallibraryd.txt"), "<<garbage").unwrap();
    std::fs::write(dir.join("csimateriallibrary.xml.bak"), "<<garbage").unwrap();
    std::fs::write(dir.join("other.xml"), "<<garbage").unwrap();
    std::fs::create_dir_all(dir.join("csimateriallibrarydir")).unwrap();

    let lib = read_material_library_dir(&dir).unwrap();
    // Eight keys; the later files replaced the earlier ones.
    assert_eq!(lib.entries.len(), 8);
    let s355 = lib
        .find("TestRegion", "Steel", "EN & Test", "S355")
        .unwrap();
    assert_eq!(s355.values.e, Some(3.0));
    assert_eq!(s355.source, "csimateriallibraryc");
    // The case-folded type is one key: no second S355 with another label.
    assert_eq!(lib.entries.iter().filter(|e| e.grade == "S355").count(), 1);

    // Without the extension-less file the middle file wins.
    std::fs::remove_file(dir.join("csimateriallibraryc")).unwrap();
    let lib = read_material_library_dir(&dir).unwrap();
    let s355 = lib
        .find("TestRegion", "steel", "EN & Test", "S355")
        .unwrap();
    assert_eq!(s355.values.e, Some(2.0));
    assert_eq!(s355.source, "CSiMaterialLibraryB.xml");

    // A broken file is an error naming the file and line, never skipped.
    std::fs::write(dir.join("csimateriallibraryz.xml"), "<a>\n</b>").unwrap();
    let error = read_material_library_dir(&dir).unwrap_err().to_string();
    assert!(
        error.contains("csimateriallibraryz.xml") && error.contains("line 2"),
        "{error}"
    );

    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn name_rule_and_empty_or_missing_folders() {
    for ok in [
        "CSiMaterialLibraryChina.xml",
        "csimateriallibrary.XML",
        "csimateriallibraryunitedstat",
        "csimateriallibrary",
    ] {
        assert!(is_library_file_name(ok), "{ok}");
    }
    for bad in [
        "csimateriallibrary.xsd",
        "csimateriallibrary.txt",
        "csimateriallibraryx.xml.bak",
        "materials.xml",
        "xcsimateriallibrary.xml",
    ] {
        assert!(!is_library_file_name(bad), "{bad}");
    }
    let dir = temp_dir("empty");
    assert!(matches!(
        read_material_library_dir(&dir),
        Err(MaterialLibraryError::NoFiles { .. })
    ));
    let _ = std::fs::remove_dir_all(&dir);
    assert!(matches!(
        read_material_library_dir(&dir),
        Err(MaterialLibraryError::Io { .. })
    ));
    assert!(matches!(
        read_material_library_file(&dir.join("nothing.xml")),
        Err(MaterialLibraryError::Io { .. })
    ));
}

#[test]
fn one_file_is_read_by_path_whatever_its_name() {
    let dir = temp_dir("one");
    let path = dir.join("my materials.txt");
    std::fs::write(&path, SYNTH).unwrap();
    let entries = read_material_library_file(&path).unwrap();
    assert_eq!(entries.len(), 8);
    assert_eq!(entries[0].source, "my materials.txt");
    // Invalid UTF-8 is refused with a line.
    let bad = dir.join("bad.xml");
    std::fs::write(&bad, b"<propertyFile>\n\xff</propertyFile>").unwrap();
    let error = read_material_library_file(&bad).unwrap_err().to_string();
    assert!(
        error.contains("line 2") && error.contains("UTF-8"),
        "{error}"
    );
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn unit_pairs_map_to_units_and_an_impossible_pair_is_refused_with_its_line() {
    assert_eq!(
        units_for(ForceUnit::Kip, LengthUnit::In),
        Some(Units::KipInF)
    );
    assert_eq!(units_for(ForceUnit::N, LengthUnit::Mm), Some(Units::NMmC));
    assert_eq!(units_for(ForceUnit::Kn, LengthUnit::M), Some(Units::KnMC));
    assert_eq!(units_for(ForceUnit::Lb, LengthUnit::Ft), Some(Units::LbFtF));
    assert_eq!(
        units_for(ForceUnit::Tonf, LengthUnit::Cm),
        Some(Units::TonCmC)
    );
    // Pairs with no unit system: refused.
    assert_eq!(units_for(ForceUnit::Kip, LengthUnit::Mm), None);
    assert_eq!(units_for(ForceUnit::Lb, LengthUnit::M), None);
    // Every one of the sixteen variants is reachable from its own pair.
    for code in 1..=16 {
        let u = Units::from_code(code).unwrap();
        assert_eq!(units_for(u.force_unit(), u.length_unit()), Some(u));
    }

    let text = "<propertyFile>\n<control forceUnits=\"Kip\" lengthUnits=\"in\"/>\n\
                <region name=\"R\">\n<collection type=\"Materials\" forceUnits=\"Kip\" \
                lengthUnits=\"mm\">\n</collection>\n</region>\n</propertyFile>";
    let (line, message) = syntax_line(text);
    assert_eq!(line, 4);
    assert!(
        message.contains("Kip") || message.contains("kip"),
        "{message}"
    );
    let error = parse_material_library(text, "t.xml")
        .unwrap_err()
        .to_string();
    assert!(error.starts_with("t.xml: line 4"), "{error}");

    // A material-level override of a US file into lb and psi-like units.
    let text = "<propertyFile><control forceUnits=\"Kip\" lengthUnits=\"in\"/>\
                <region name=\"R\"><collection type=\"M\"><standard name=\"S\">\
                <material type=\"concrete\" grade=\"g\" name=\"n\" forceUnits=\"lb\" lengthUnits=\"in\">\
                <weightDensity>0.0868</weightDensity><massDensity>1</massDensity>\
                <modulusOfElasticity>3605000</modulusOfElasticity><poissonsRatio>0.2</poissonsRatio>\
                <coefficientOfThermalExpansion>5.5E-06</coefficientOfThermalExpansion>\
                <compressiveStrength>4000</compressiveStrength></material>\
                </standard></collection></region></propertyFile>";
    let e = parse_material_library(text, "t.xml").unwrap();
    assert_eq!(e[0].units, Units::LbInF);
    let m = library_material(&e[0], Units::KipInF, "n").unwrap();
    assert!(close(m.fc, 4.0, 1e-12));
    assert!(close(m.e, 3605.0, 1e-12));
}

#[test]
fn unsupported_xml_is_refused_with_the_line() {
    let cases: [(&str, usize, &str); 11] = [
        ("<?xml version=\"1.0\"?>\n<!DOCTYPE a>\n<a/>", 2, "DTD"),
        ("<a>\n\n<![CDATA[x]]></a>", 3, "CDATA"),
        ("<a>\n<x:b/></a>", 2, "prefix"),
        ("<a>\n&foo;</a>", 2, "entity"),
        ("<a>\n&#xZZ;</a>", 2, "entity"),
        ("<a>\n<b></c></a>", 2, "does not match"),
        ("<a>\n<b>\n", 3, "not closed"),
        ("<a/>\nstray", 2, "outside"),
        ("<a/>\n<b/>", 2, "more than one root"),
        ("<a b=c/>", 1, "not quoted"),
        ("<a b=\"1\"\n b=\"2\"/>", 2, "duplicate"),
    ];
    for (text, line, word) in cases {
        let (got, message) = syntax_line(text);
        assert_eq!(got, line, "{text:?}: {message}");
        assert!(message.contains(word), "{text:?}: {message}");
    }
    // Readable XML that is not a library.
    let (line, message) = syntax_line("\n<other/>");
    assert_eq!((line, message.contains("propertyFile")), (2, true));
    let (line, message) = syntax_line("<propertyFile>\n</propertyFile>");
    assert_eq!((line, message.contains("control")), (1, true));
    // A bad number names the element's line.
    let text = SYNTH.replace(
        "<poissonsRatio>0.3</poissonsRatio>",
        "<poissonsRatio>x</poissonsRatio>",
    );
    let (line, message) = syntax_line(&text);
    assert_eq!(line, 16);
    assert!(message.contains("poissonsRatio"), "{message}");
    // Empty input.
    assert!(parse_material_library("", "t.xml").is_err());
    assert!(parse_material_library("   \n", "t.xml").is_err());
}

#[test]
fn truncated_and_garbage_input_never_panics() {
    // Every prefix of the synthetic file (at character boundaries).
    for end in 0..=SYNTH.len() {
        if SYNTH.is_char_boundary(end) {
            let _ = parse_material_library(&SYNTH[..end], "cut.xml");
        }
    }
    // Pseudo-random text from the characters that matter to the reader.
    let alphabet: Vec<char> = "<>/=\"'&;!-?[]# \n\tabx1:\u{0421}\u{feff}"
        .chars()
        .collect();
    let mut state: u64 = 0x9E37_79B9_7F4A_7C15;
    for _ in 0..3000 {
        let mut text = String::new();
        let n = 1 + (state >> 40) as usize % 60;
        for _ in 0..n {
            state = state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            text.push(alphabet[(state >> 33) as usize % alphabet.len()]);
        }
        let _ = parse_material_library(&text, "junk.xml");
    }
    // Deep nesting is refused, not recursed into.
    let deep = "<a>".repeat(10_000);
    assert!(parse_material_library(&deep, "deep.xml").is_err());
}

#[test]
fn library_names_survive_an_s2k_round_trip_or_the_failure_is_reported() {
    let lib = synth();
    let mut model = SapModel::new(Units::KnMC);
    let c = lib
        .find("TestRegion", "concrete", "EN & Test", "C30/37")
        .unwrap();
    model.materials.insert(
        "C30/37".into(),
        library_material(c, Units::KnMC, "C30/37").unwrap(),
    );
    let s = lib
        .find("TestRegion", "steel", "EN & Test", "S355")
        .unwrap();
    let cyr = "\u{0421}245_\u{0441}\u{0442}\u{0430}\u{043b}\u{044c}";
    model
        .materials
        .insert(cyr.into(), library_material(s, Units::KnMC, cyr).unwrap());
    let back = parse_s2k(&write_s2k(&model)).expect("the S2K text of the model reads back");
    for name in ["C30/37", cyr] {
        let a = &model.materials[name];
        let b = back
            .materials
            .get(name)
            .unwrap_or_else(|| panic!("material {name} lost in the S2K round trip"));
        assert_eq!(b.mat_type, a.mat_type);
        assert!(close(b.e, a.e, 1e-9), "{name}: E {} vs {}", b.e, a.e);
        assert!(close(b.unit_weight, a.unit_weight, 1e-9));
        assert!(close(b.fy.max(b.fc), a.fy.max(a.fc), 1e-9));
    }
}

// ---------------------------------------------------------------------------
// Opt-in vendor files (STRUCTURAX_VENDOR_FIXTURES = directory with the files)
// ---------------------------------------------------------------------------

fn vendor_dir() -> Option<PathBuf> {
    let dir = PathBuf::from(std::env::var_os("STRUCTURAX_VENDOR_FIXTURES")?);
    if library_dir_has_files(&dir) {
        Some(dir)
    } else {
        eprintln!("skipped: no material library files in {}", dir.display());
        None
    }
}

fn library_dir_has_files(dir: &Path) -> bool {
    sap_format::material_library::library_files_in(dir)
        .map(|f| !f.is_empty())
        .unwrap_or(false)
}

#[test]
fn vendor_files_have_the_expected_totals_and_values() {
    let Some(dir) = vendor_dir() else { return };
    // Eight files: six with the .xml extension, two whose names the
    // extraction of the installer truncated (no extension).
    let files = sap_format::material_library::library_files_in(&dir).unwrap();
    assert_eq!(files.len(), 8, "{files:?}");
    let lib = read_material_library_dir(&dir).unwrap();
    let total: usize = files
        .iter()
        .map(|f| read_material_library_file(f).unwrap().len())
        .sum();
    // 287 elements and 287 unique keys (type case-folded).
    assert_eq!(total, 287);
    assert_eq!(lib.entries.len(), 287);
    let count = |region: &str| lib.entries.iter().filter(|e| e.region == region).count();
    for (region, n) in [
        ("China", 76),
        ("Europe", 18),
        ("India", 16),
        ("Italy", 21),
        ("New Zealand", 33),
        ("Russia", 74),
        ("Spain", 13),
        ("United States", 36),
    ] {
        assert_eq!(count(region), n, "{region}");
    }
    assert_eq!(lib.regions().len(), 8);
    // Only three unit systems occur in the files.
    let mut units: Vec<Units> = lib.entries.iter().map(|e| e.units).collect();
    units.sort_by_key(|u| u.code());
    units.dedup();
    assert_eq!(units, vec![Units::KipInF, Units::KnMC, Units::NMmC]);

    // What can and cannot be added: every refusal is of one of the planned kinds.
    let mut ok = 0;
    let mut refused = std::collections::BTreeMap::<String, usize>::new();
    for e in &lib.entries {
        match library_material(e, Units::KnMC, &e.name) {
            Ok(_) => ok += 1,
            Err(message) => {
                let reason = if message.contains("lightweight") {
                    "lightweight"
                } else if message.contains("shear") {
                    "shear"
                } else if message.contains("not supported") {
                    "type"
                } else {
                    panic!("unexpected refusal: {message}")
                };
                *refused.entry(reason.into()).or_default() += 1;
            }
        }
    }
    eprintln!("convertible {ok}, refused {refused:?}");
    assert_eq!(ok + refused.values().sum::<usize>(), 287);
    // Four lightweight concretes (shear factor 0.85) and five aluminum and
    // cold-formed materials are all in the United States file.
    assert_eq!(refused.get("lightweight"), Some(&4));
    assert_eq!(refused.get("type"), Some(&5));
    assert_eq!(refused.get("shear"), None);
    assert!(lib
        .entries
        .iter()
        .filter(|e| e.values.lightweight || e.values.shear_reduction.is_some_and(|r| r != 1.0))
        .all(|e| e.region == "United States"));

    // United States, in the units of the file (kip, in).
    let a36 = lib
        .entries
        .iter()
        .find(|e| e.region == "United States" && e.name == "A36")
        .unwrap();
    assert_eq!(a36.grade, "Grade 36");
    assert_eq!(a36.units, Units::KipInF);
    let m = library_material(a36, Units::KipInF, "A36").unwrap();
    assert_eq!((m.e, m.fy, m.fu), (29000.0, 36.0, 58.0));
    let a992 = lib
        .entries
        .iter()
        .find(|e| e.region == "United States" && e.name == "A992Fy50")
        .unwrap();
    let m = library_material(a992, Units::KipInF, "A992Fy50").unwrap();
    assert_eq!((m.fy, m.fu), (50.0, 65.0));
    let c4000 = lib
        .entries
        .iter()
        .find(|e| e.region == "United States" && e.name == "4000Psi")
        .unwrap();
    let m = library_material(c4000, Units::KipInF, "4000Psi").unwrap();
    // The file gives f'c = 4 ksi and E = 3604.9965 ksi (57000 sqrt(4000 psi)).
    assert_eq!(m.fc, 4.0);
    assert_eq!(m.e, 3604.9965);
    assert_eq!(m.mat_type, MatType::Concrete);
    // The file's weight and mass density are not related by g in the file.
    assert_eq!(m.unit_weight, 0.00008680555556);
    assert_eq!(m.unit_mass, 0.0000002248332521);

    // Conversion: A36 added to a kN, m model, Fy = 36 ksi in kN/m2.
    let ksi_in_kn_m2 = ForceUnit::Kip.to_newtons() / LengthUnit::In.to_meters().powi(2) / 1000.0;
    assert!(close(ksi_in_kn_m2, 6_894.757_293_168, 1e-9));
    let m = library_material(a36, Units::KnMC, "A36").unwrap();
    assert!(close(m.fy, 36.0 * ksi_in_kn_m2, 1e-9));
    assert!(close(m.fy, 36.0 * 6_894.757_293_168, 1e-9));
    assert!(close(m.fu, 58.0 * 6_894.757_293_168, 1e-9));
    assert!(close(m.e, 29000.0 * 6_894.757_293_168, 1e-9));
    assert!(close(m.g, m.e / (2.0 * 1.3), 1e-12));
    // Unit weight kip/in3 to kN/m3.
    let uw = a36.values.weight_density.unwrap() * 4.448_221_615_260_5 / 0.0254_f64.powi(3);
    assert!(close(m.unit_weight, uw, 1e-9));

    // Europe: EN S355 in N and mm.
    let s355 = lib
        .entries
        .iter()
        .find(|e| e.region == "Europe" && e.grade == "S355")
        .unwrap();
    assert_eq!(s355.units, Units::NMmC);
    assert_eq!(s355.values.e, Some(210000.0));
    assert_eq!(s355.values.fy, Some(355.0));
    assert_eq!(s355.values.fu, Some(510.0));
    let m = library_material(s355, Units::NMmC, "S355").unwrap();
    assert_eq!((m.e, m.fy, m.fu), (210000.0, 355.0, 510.0));
    // C30/37
    let c3037 = lib
        .entries
        .iter()
        .find(|e| e.region == "Europe" && e.grade.contains("C30/37"))
        .unwrap();
    assert_eq!(c3037.values.e, Some(33000.0));
    assert_eq!(c3037.values.fc, Some(30.0));

    // Russia (kN, m): C245_(2...20mm): E 2.06e8, weight 77, mass 7.85; they
    // differ by a factor that is not g, so neither was derived from the other.
    let c245 = lib
        .entries
        .iter()
        .find(|e| e.region == "Russia" && e.grade.contains("C245_(2"))
        .unwrap();
    assert_eq!(c245.units, Units::KnMC);
    assert_eq!(c245.values.e, Some(2.06e8));
    assert_eq!(c245.values.weight_density, Some(77.0));
    assert_eq!(c245.values.mass_density, Some(7.85));
    assert!((77.0_f64 / 7.85 - 9.80665).abs() > 1e-3);
    let m = library_material(c245, Units::KnMC, "C245").unwrap();
    assert_eq!((m.unit_weight, m.unit_mass), (77.0, 7.85));

    // New Zealand: 33 materials were checked above; its units are kN and m.
    assert!(lib
        .entries
        .iter()
        .filter(|e| e.region == "New Zealand")
        .all(|e| e.units == Units::KnMC));
}

#[test]
fn vendor_library_names_round_trip_through_s2k() {
    let Some(dir) = vendor_dir() else { return };
    let lib = read_material_library_dir(&dir).unwrap();
    let mut model = SapModel::new(Units::KnMC);
    let mut expected = Vec::new();
    for e in &lib.entries {
        let name = format!("{}|{}", e.region, e.grade);
        if let Ok(m) = library_material(e, Units::KnMC, &name) {
            model.materials.insert(name.clone(), m);
            expected.push(name);
        }
    }
    let back = parse_s2k(&write_s2k(&model)).expect("the S2K text reads back");
    for name in expected {
        let a = &model.materials[&name];
        let b = back
            .materials
            .get(&name)
            .unwrap_or_else(|| panic!("material {name} lost in the S2K round trip"));
        assert!(close(a.e, b.e, 1e-9), "{name}");
    }
}
