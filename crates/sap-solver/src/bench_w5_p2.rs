//! Wave 5 verification rows (package P2): constitutive laws of area elements.
//!
//! The expected values are closed forms (Analysis Reference pp. 75, 218), written out in
//! `crates/sap-solver/tests/wave5_area_materials.rs`.

use sap_model::{AreaObj, AreaPropType, AreaSection, Material, PlaneType};

use crate::area::{area_local_system, recover_area_result};
use crate::benchmarks::BenchmarkVerificationRow;

/// Corners of the distorted quadrilateral patch (counter-clockwise, in the X-Y plane).
const PATCH: [[f64; 3]; 4] = [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.2, 1.5, 0.0], [-0.1, 1.2, 0.0]];

/// Recovered result of a linear displacement field on the patch, in element axes.
fn patch_result(
    sec: &AreaSection,
    mat: &Material,
    field: impl Fn(f64, f64) -> [f64; 3],
) -> sap_model::AreaResult {
    let area = AreaObj::new("A", vec!["1".into(), "2".into(), "3".into(), "4".into()], &sec.name);
    let (_, _, xy) = area_local_system(&PATCH, area_axes(&area, sec));
    let mut u = vec![0.0; 24];
    for (i, c) in PATCH.iter().enumerate() {
        let d = field(c[0], c[1]);
        u[6 * i..6 * i + 3].copy_from_slice(&d);
    }
    recover_area_result(&area, sec, mat, &xy, &u, Vec::new())
}

fn area_axes(area: &AreaObj, sec: &AreaSection) -> f64 {
    crate::area::area_axes_angle(area, sec)
}

fn plane_strain_patch() -> Vec<BenchmarkVerificationRow> {
    let mut mat = Material::default_steel_kip_in();
    mat.e = 1000.0;
    mat.u = 0.3;
    mat.g = 1000.0 / 2.6;
    let mut sec = AreaSection::default();
    sec.material = mat.name.clone();
    sec.prop_type = AreaPropType::Plane;
    sec.plane_type = PlaneType::PlaneStrain;
    sec.thickness_membrane = 1.0;
    sec.thickness_bending = 1.0;
    let r = patch_result(&sec, &mat, |x, y| {
        [1e-3 * x + 0.1e-3 * y, 0.1e-3 * x + 0.5e-3 * y, 0.0]
    });
    vec![BenchmarkVerificationRow::new(
        "Plane strain patch (Analysis Reference p. 218)",
        "e11 1e-3, e22 0.5e-3, g12 0.2e-3; E 1000, nu 0.3",
        "S33 = nu (S11 + S22) [ksi]",
        0.865_385,
        r.s33,
        0.001,
    )]
}

/// Orthotropic membrane of Example 2-015 (E1 30000, E2 15000, nu12 0.2, G12 6600, t = 2) with
/// its material axes 30 degrees from the element axes, strain e11 = 1e-4 in element axes:
/// F11 = t Q_bar11 e11 = 2 * 26757.065 * 1e-4 = 5.351413 (Jones, transformed reduced stiffness).
fn orthotropic_material_angle() -> Vec<BenchmarkVerificationRow> {
    let mut mat = Material::default_steel_kip_in();
    mat.e_ortho = Some([30000.0, 15000.0, 1.0]);
    mat.u_ortho = Some([0.2, 0.0, 0.0]);
    mat.g_ortho = Some([6600.0, 11693.0, 11693.0]);
    let mut sec = AreaSection::default();
    sec.material = mat.name.clone();
    sec.mat_angle_deg = 30.0;
    sec.thickness_membrane = 2.0;
    sec.thickness_bending = 2.0;
    let r = patch_result(&sec, &mat, |x, _| [1e-4 * x, 0.0, 0.0]);
    vec![BenchmarkVerificationRow::new(
        "Orthotropic membrane at a 30 degree material angle",
        "e11 1e-4 (element axes); E1 30000, E2 15000, nu12 0.2, G12 6600, t 2",
        "F11 = t Q11bar e11 [k/in]",
        5.351_413,
        r.f11,
        0.001,
    )]
}

pub(crate) fn rows() -> Vec<BenchmarkVerificationRow> {
    let mut rows = Vec::new();
    rows.extend(plane_strain_patch());
    rows.extend(orthotropic_material_angle());
    rows
}
