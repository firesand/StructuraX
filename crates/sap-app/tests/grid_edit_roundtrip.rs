//! S2K round trip of grid lines with typed ids and the 2D template on typed
//! grid lines. Same text in, same grid out: ids and coordinates must come back
//! exactly (1e-12 for coordinates that were written as decimal text).

use sap_model::generate::GridAxis;
use sap_model::{FrameType2D, SapModel};

fn grids(m: &SapModel) -> Vec<(String, String, f64)> {
    m.coord_systems["GLOBAL"]
        .grids
        .iter()
        .map(|g| (g.axis.clone(), g.id.clone(), g.coordinate))
        .collect()
}

fn line(id: &str, c: f64) -> (String, f64) {
    (id.to_string(), c)
}

fn round_trip(m: &SapModel) -> SapModel {
    let text = sap_format::write_s2k(m);
    sap_format::parse_s2k(&text).unwrap_or_else(|e| panic!("re-reading the S2K text failed: {e}"))
}

#[test]
fn typed_grid_ids_survive_an_s2k_round_trip() {
    let mut m = SapModel::default();
    m.set_grid_lines(
        GridAxis::X,
        &[line("A", 0.0), line("Gate", 2.5), line("B1", 10.0), line("Axis-3", 11.25)],
    )
    .unwrap();
    m.set_grid_lines(GridAxis::Y, &[line("1", -3.0), line("1a", 0.1), line("2", 7.0)])
        .unwrap();
    m.set_grid_lines(GridAxis::Z, &[line("Roof", 12.0), line("Base", 0.0)])
        .unwrap();
    let back = round_trip(&m);
    assert_eq!(grids(&back), grids(&m));
    // Visibility and bubble location come back as well.
    for (a, b) in back.coord_systems["GLOBAL"].grids.iter().zip(&m.coord_systems["GLOBAL"].grids) {
        assert_eq!((a.visible, &a.bubble_loc), (b.visible, &b.bubble_loc), "{}", a.id);
    }
}

#[test]
fn ids_with_blanks_and_symbols_survive_an_s2k_round_trip() {
    let mut m = SapModel::default();
    m.set_grid_lines(
        GridAxis::X,
        &[line("Grid 1", 0.0), line("X/Y", 4.0), line("C(2)", 9.0), line("D_4", 12.0)],
    )
    .unwrap();
    let back = round_trip(&m);
    assert_eq!(grids(&back), grids(&m));
}

#[test]
fn non_ascii_grid_ids_survive_an_s2k_round_trip() {
    let mut m = SapModel::default();
    m.set_grid_lines(GridAxis::X, &[line("Ось1", 0.0), line("Achse-\u{00e4}", 5.0)])
        .unwrap();
    let back = round_trip(&m);
    assert_eq!(grids(&back), grids(&m));
}

#[test]
fn the_custom_template_grid_survives_an_s2k_round_trip() {
    let mut m = SapModel::default();
    m.generate_2d_frame_custom(
        FrameType2D::PortalFrame,
        &[0.0, 4.0, 10.0, 14.0],
        &[0.0, 3.0, 6.0, 9.0, 12.0],
        true,
        "FSEC1",
        "FSEC1",
        "FSEC1",
        0.0,
    )
    .unwrap();
    let back = round_trip(&m);
    assert_eq!(grids(&back), grids(&m));
    assert_eq!(back.points.len(), 20);
    assert_eq!(back.frames.len(), 28);
}
